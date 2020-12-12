from leg_connection import leg_connection
import numpy as np
import copy
import math
import cmath
import transform
import threading
import time
from gait import Gait
from quadleg import QuadLeg
from quadbody import QuadBody
import csv
from simple_pid import PID

import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})


class Quadruped:

    def __init__(self, simulation=False):

        if simulation == True:
            self.joint_state_subscriber = rospy.Subscriber('/quadruped/joint_states',JointState,
                                                                   self.joint_state_subscriber_callback)
            self.jtp = rospy.Publisher('/quadruped/joint_trajectory_controller/command',JointTrajectory,queue_size=1)

        self.simulation = simulation
        self.body = QuadBody()
        self.joint_state = np.zeros(12)
        self.COM = np.zeros(3) # to do
        self.global_rpy = np.zeros(3) # to do

        #transformation for all legs from center of quadruped to legs
        self.tc_right_front = transform.transformFull(0,0,0,self.body.width*0.5, (self.body.length/2)*0.9, self.body.height*0.5)
        self.tc_right_back = transform.transformFull(0,0,0,self.body.width*0.5, -(self.body.length/2)*0.9, self.body.height*0.5)
        self.tc_left_front = transform.transformFull(0,0,math.pi, self.body.width*0.5, (self.body.length/2)*0.9, self.body.height*0.5)
        self.tc_left_back = transform.transformFull(0,0,math.pi, self.body.width*0.5, -(self.body.length/2)*0.9, self.body.height*0.5)

        self.legs = []
        self.legs.append(QuadLeg(joints=np.array([-0.0, -0.55, 0.9]), bodyPosition=self.tc_right_front, name="right_front", side="right", frontBack="front", simulation=self.simulation))
        self.legs.append(QuadLeg(joints=np.array([-0.0, 0.55, -0.9]), bodyPosition=self.tc_left_back, name="left_back",  side="left", frontBack="back", simulation=self.simulation))
        self.legs.append(QuadLeg(joints=np.array([-0.0, 0.55, -0.9]), bodyPosition=self.tc_left_front, name="left_front", side="left", frontBack="front", simulation=self.simulation))
        self.legs.append(QuadLeg(joints=np.array([-0.0, -0.55, 0.9]), bodyPosition=self.tc_right_back, name="right_back", side="right", frontBack="back", simulation=self.simulation))

        self.joint_state[0:3] = self.legs[0].joints
        self.joint_state[3:6] = self.legs[3].joints
        self.joint_state[6:9] = self.legs[2].joints
        self.joint_state[9:12] = self.legs[1].joints
        self.jointCommands = np.zeros((4,3))

        jointArr = []
        swingArr = []
        for i in range(len(self.legs)):
            jointArr.append(self.legs[i].joints)
            swingArr.append(self.legs[i].swing)

        jointArr = np.array(jointArr)

        self.COM = self.getCOM(jointArr, swingArr)
        self.x_local_goal = 0
        self.y_local_goal = 0.02
        self.z_local_goal = -0.32
        self.x_width_des = 0
        self.roll_des = 0
        self.pitch_des = 0
        self.yaw_des = 0

        self.roll_err = 0
        self.pitch_err = 0
        self.yaw_err = 0

        self.feet_sensor_readings = np.zeros((4,3))

        self.commandFrequency = 65
        self.gait = Gait(self,frequency = self.commandFrequency)

        self.setTranslationalVelocity(0.0)
        self.setAngularVelocity(0.0)
        self.gaitStyle = 0
        self.gait.phase = 1
        self.readyToWalk = False


        self.csvFileReadLog = open("test_read_log.txt", 'a')
        self.csvFileCommandLog = open("test_command_log.txt", 'a')

        self.roll_meas = 0
        self.pitch_meas = 0
        self.yaw_meas = 0

        self.roll_offset = 0
        self.pitch_offset = 0
        self.yaw_offset = 0

        self.roll_actual = 0
        self.pitch_actual = 0
        self.yaw_actual = 0

        self.x_err = 0
        self.y_err = 0
        self.z_err = 0

        self.pGainRoll = 1
        self.pGainPitch = 1
        self.pGainZ = 0



        self.angle_pid = False

        if self.simulation == False:
            #self.leg_con = leg_connection(name_serial_port='/dev/ttyACM0',using_current=True)
            self.leg_con = leg_connection(name_serial_port='/dev/ttyACM0')
            self.time_frequency = time.time()
            self.stateUpdateFrequency = 65 # hz
            time.sleep(3)
            self.moveThread = threading.Thread(target=self.move).start()
            self.stateReadThread = threading.Thread(target=self.updateState).start()

            self.pidZ = PID(1.0, 0, 0.05)
            self.pidZ.sample_time = 1.0 / self.stateUpdateFrequency
            self.pidZ.setpoint = self.z_local_goal

        else:
            self.moveThread = threading.Thread(target=self.move).start()



    def setCommandFrequency(self, val):
        self.commandFrequency = max(10, int(val))
        self.gait.setFrequency(self.commandFrequency)

    def setStepSize(self, val):
        self.gait.setStepSize(max(0, val))

    def setStepHeight(self, val):
        self.gait.setStepHeight(max(0.1, val))

    def setCycleTime(self, val):
        self.gait.setCycleTime(max(1.0, val))

    def setGait(self, val):
        self.gaitStyle = val

    def switchGait(self):
        self.gaitStyle += 1
        if self.gaitStyle >= 2:
            self.gaitStyle = 0

    def getCOM(self, jointArr, swingArr):
        # COM = center of mass
        # computation of C.O.M given leg positions and swing phase booleans
        COM = np.zeros(3)
        nr_contacts = 0
        for i in range(len(swingArr)):
            if swingArr[i] == False:
                local_ee = self.legs[i].computeLocalForwardKinematics(jointArr[i])
                if self.legs[i].side == "left":
                    COM[0] += -local_ee[0]
                    COM[1] += -local_ee[1]
                else:
                    COM[0] += local_ee[0]
                    COM[1] += local_ee[1]
                COM[2] += local_ee[2]
                nr_contacts += 1
        COM = COM / nr_contacts
        return COM

    def getSupportPolygon(self, jointArr, swingArr):
        """ Returns the xy coordinates making up the support polygon. """
        sp = np.zeros((4,2))
        for i in range(len(swing)):
            if swing[i] == False:
                sp[i][0] = self.legs[i].__computeLocalForwardKinematics(jointArr[i])[0]
                sp[i][1] = self.legs[i].__computeLocalForwardKinematics(jointArr[i])[1]

        return sp

    def computeLocalInverseKinematics(self, xyz):
        # the COM can be thought of as centered on a plane defined by the global
        # x,y position, with the z coordinates from each leg, from this roll and
        # pitch can be extracted as well as z coordinate
        pass

    def setTranslationalVelocity(self, vel):
        self.transl_vel = vel

    def setAngularVelocity(self, vel):
        self.ang_vel = vel

    def move(self):
        t_period = 1 / self.gait.frequency

        self.yCOMdesired = 0.0
        self.xCOMdesired = 0.0
        y_local_goal = 0
        stepSize = 0.05 # in meters
        stanceVel = 0.001/8 # arbitrary unit, depends on gait frequency
        #stanceVel = 0.01/8 # arbitrary unit, depends on gait frequency
        zLegVel = 0.0001
        #zLegVel = 0.0005
        zLegMargin = 0.003
        while True:
            ts = int(round(time.time() * 1000))
            if self.readyToWalk == True:
                if self.gaitStyle == 0:
                    self.gait.waveGait(self.ang_vel, self.z_local_goal, self.y_local_goal)

                    #csv_string = str(self.gait.t) + "," + str(self.legs[0].joints).replace("[","").replace("]","").replace(" ", ",") + str(self.legs[1].joints).replace("[","").replace("]","").replace(" ", ",") + str(self.legs[2].joints).replace("[","").replace("]","").replace(" ", ",") + str(self.legs[3].joints).replace("[","").replace("]","").replace(" ", ",") + "," + str(self.legs[0].swing) + "," + str(self.legs[1].swing) + "," + str(self.legs[2].swing) + "," + str(self.legs[3].swing) + "\n"
                    #self.csvFileCommandLog.write(csv_string)
                    #self.csvFileCommandLog.flush()

            self.computeLegs()
            te = int(round(time.time() * 1000))
            t_sleep = (t_period-((te-ts))*0.001)
            if t_sleep > 0:
                time.sleep((t_period-((te-ts))*0.001))


    def sendJointCommand(self, joints):
        self.jointCommands = joints
        if self.simulation == True:
            self.sendJointCommandSim(joints)
        else:
            self.sendJointCommandRobot(joints)

    def sendJointCommandRobot(self, joints):
        time_step=time.time()-self.time_frequency
        frequency=1/time_step
        #margin specified based on the gait....
        margin = 0.5
        if frequency<(self.gait.frequency+margin):
            #### update interface to shoulder joints...
            temp=joints

            ####sending with velocities. Note that if sending 0 or above the limit<150, the
            #### the motors will use the previously used velocity
            radians_array=np.append(np.append(np.append(temp[0],-temp[2]),temp[3]),-temp[1])
            #velocities=[20,40,80,20,40,80,20,40,80,20,40,80]
            #self.leg_con.execute_joint_pos_radians_with_vel(radians_array,velocities)

            #sending without velocities
            self.leg_con.execute_joint_position_radians(radians_array)


            self.time_frequency=time.time()

    def sendJointCommandSim(self, joint_msg):
        joint_name_lst = ['front_right_leg3_joint', 'front_right_leg2_joint', 'front_right_leg1_joint',
                                       'front_left_leg3_joint', 'front_left_leg2_joint', 'front_left_leg1_joint',
                                       'back_right_leg3_joint', 'back_right_leg2_joint','back_right_leg1_joint',
                                       'back_left_leg3_joint', 'back_left_leg2_joint', 'back_left_leg1_joint']
        jtp_msg = JointTrajectory()
        jtp_msg.joint_names = joint_name_lst
        point = JointTrajectoryPoint()
        point.positions = self.__makeJointStateMsg2(joint_msg)
        point.velocities = np.zeros(len(joint_name_lst))
        point.accelerations = np.zeros(len(joint_name_lst))
        point.effort = np.zeros(len(joint_name_lst))
        point.time_from_start = rospy.Duration(1.0/60.0)
        jtp_msg.points.append(point)

        self.jtp.publish(jtp_msg)


    def __makeJointStateMsg2(self,q):

        jointState = []

        # leg 0 / right front
        jointState.append(q[0][0]) # joint 1 shoulder
        jointState.append(q[0][1]) # joint 2 elbow
        jointState.append(q[0][2]) # joint 3 wrist

        # leg 2 / left front
        jointState.append(q[2][0]) # joint 1 shoulder
        jointState.append(q[2][1]) # joint 2 elbow
        jointState.append(q[2][2]) # joint 3 wrist

        # leg 3 / right back
        jointState.append(q[3][0]) # joint 1 shouprint("YCOMREAL: ", yCOMReal)lder
        jointState.append(q[3][1]) # joint 2 elbow
        jointState.append(q[3][2]) # joint 3 wrist

        # leg 1 / left back
        jointState.append(q[1][0]) # joint 1 shoulder
        jointState.append(q[1][1]) # joint 2 elbow
        jointState.append(q[1][2]) # joint 3 wrist

        return np.array(jointState)

    def __makeJointStateMsg(self,q):

        jointState = []
        # leg 0 / right front
        jointState.append(q[0][0]) # joint 1 shoulder
        jointState.append(q[0][1]) # joint 2 elbow
        jointState.append(q[0][2]) # joint 3 wrist

        # leg 1 / left back
        jointState.append(q[1][0]) # joint 1 shoulder
        jointState.append(q[1][1]) # joint 2 elbow
        jointState.append(q[1][2]) # joint 3 wrist

        # leg 3 / right back
        jointState.append(q[3][0]) # joint 1 shoulder
        jointState.append(q[3][1]) # joint 2 elbow
        jointState.append(q[3][2]) # joint 3 wrist

        # leg 2 / left front
        jointState.append(q[2][0]) # joint 1 shoulder
        jointState.append(q[2][1]) # joint 2 elbow
        jointState.append(q[2][2]) # joint 3 wrist

        return np.array(jointState)

    def makeJointStateMsg(self,q):

        jointState = []
        # leg 0 / right front
        jointState.append(q[0][0]) # joint 1 shoulder
        jointState.append(q[0][1]) # joint 2 elbow
        jointState.append(q[0][2]) # joint 3 wrist

        # leg 1 / left back
        jointState.append(q[1][0]) # joint 1 shoulder
        jointState.append(q[1][1]) # joint 2 elbow
        jointState.append(q[1][2]) # joint 3 wrist

        # leg 3 / right back
        jointState.append(q[3][0]) # joint 1 shoulder
        jointState.append(q[3][1]) # joint 2 elbow
        jointState.append(q[3][2]) # joint 3 wrist

        # leg 2 / left front
        jointState.append(q[2][0]) # joint 1 shoulder
        jointState.append(q[2][1]) # joint 2 elbow
        jointState.append(q[2][2]) # joint 3 wrist

        return np.array(jointState)

    def resetIMU(self):
        self.roll_offset = self.roll_meas
        self.pitch_offset = self.pitch_meas
        self.yaw_offset = self.yaw_meas

    def computeLegs(self):
        """ compute joint values for each leg based on COM: x, y, z, width
        roll and pitch

        - output: jointAngles 3 [shoulder, elbow, wrist] for each 4 legs [front_right,
            back_left, front_left, back_right]
        """

        # generate coordinates based on COM
        self.translateX()
        self.translateY()
        self.translateZ()

        # compute roll and pitch angles for each leg
        rollAng = self.getRollAngle(self.roll_des + (self.pGainRoll*self.roll_err))
        pitchAng = self.getPitchAngle(self.pitch_des + (self.pGainPitch*self.pitch_err))

        # compute global z values for each leg

        z_roll_diff = 0.5 * self.body.width * math.sin(self.roll_des + (self.pGainRoll*self.roll_err))
        z_pitch_diff = 0.5 * self.body.length * math.sin(self.pitch_des + (self.pGainPitch*self.pitch_err))

        # roll
        for i in range(len(self.legs)):
            if self.legs[i].swing == False:
                if self.legs[i].side == "left":
                    self.legs[i].z_global_goal -= z_roll_diff
                else:
                    self.legs[i].z_global_goal += z_roll_diff
        # pitch
        for i in range(len(self.legs)):
            if self.legs[i].swing == False:
                if self.legs[i].frontBack == "back":
                    self.legs[i].z_global_goal -= z_pitch_diff
                else:
                    self.legs[i].z_global_goal += z_pitch_diff

            #print(i, self.legs[i].x_global_goal, self.legs[i].y_global_goal, self.legs[i].z_global_goal)

        # construct rotation matrices
        rotRoll = []
        rotPitch = []

        for i in range(len(self.legs)):
            rotRoll.append(transform.roty(rollAng[i]))
            rotPitch.append(transform.rotx(pitchAng[i]))


        # construct r vectors
        rVec = self.getRVectors()
        q = []

        # compute global coordinates for each leg
        for i in range(len(self.legs)):
            xyz = np.array([ [0],
                        [0],
                        [rVec[i]]])
            xyz = np.matmul(rotRoll[i], xyz)
            xyz = np.matmul(rotPitch[i], xyz)

            xyz[2] = -xyz[2]
            if self.legs[i].side == "left":
                xyz[1] = -xyz[1]

            # compute joint angles
            cartessian_coordinates = np.array([xyz[0], xyz[1], xyz[2]])
            cartessian_coordinates = np.reshape(cartessian_coordinates, (3))
            q.append(self.legs[i].computeLocalInverseKinematics(cartessian_coordinates))

        q = np.array(q)
        if self.simulation == False:
            q[1][0] = -q[1][0]
            q[2][0] = -q[2][0]

        # send jointCommand
        self.sendJointCommand(q)

    def setRoll(self, ang):
        self.roll_des = max(-0.4, min(0.4,ang))
        self.computeLegs()

    def setPitch(self, ang):
        self.pitch_des = max(-0.4, min(0.4,ang))
        self.computeLegs()

    def setYaw(self, ang):
        self.yaw_des = max(-0.9, min(0.9,ang))
        self.computeLegs()

    def setX(self, x):
        x = max(-0.09, min(0.09, x))
        self.x_local_goal = x
        self.computeLegs()

    def setY(self, y):
        y = max(-0.12, min(0.12, y))
        self.y_local_goal = y
        self.computeLegs()

    def setZ(self, z):
        z = max(-0.4, min(-0.2, z))
        self.z_local_goal = z
        self.pidZ.setpoint = self.z_local_goal
        self.computeLegs()

    def setWidth(self, w):
        w = max(-0.1, min(0.12, w))
        self.x_width_des = w
        self.computeLegs()

    def translateX(self):
        for i in range(len(self.legs)):

            if self.legs[i].side == "left":
                self.legs[i].setXGlobal(self.legs[i].x_local_goal+ self.legs[i].l1 - self.x_local_goal + self.x_width_des)
            else:
                self.legs[i].setXGlobal(self.legs[i].x_local_goal + self.legs[i].l1 + self.x_local_goal + self.x_width_des)
    def translateY(self):
        for i in range(len(self.legs)):
            self.legs[i].setYGlobal(self.legs[i].y_local_goal + self.y_local_goal)

    def translateZ(self):
        for i in range(len(self.legs)):
            #self.legs[i].setZGlobal(self.legs[i].z_local_goal + self.z_local_goal + (self.pGainZ*self.z_err))
            self.legs[i].setZGlobal(self.legs[i].z_local_goal + self.z_local_goal)# + self.z_err)

    def getRollAngle(self, ang):
        q = []

        for i in range(len(self.legs)):
            if self.legs[i].swing == False:

                if self.legs[i].side == "left":
                    h = self.z_local_goal - (0.5*self.body.width) * math.sin(ang)
                else:
                    h = self.z_local_goal + (0.5*self.body.width) * math.sin(ang)
                k = (0.5*self.body.width) - (0.5 * self.body.width * math.cos(ang))

                if self.legs[i].side == "left":
                    rVec = math.sqrt(pow(h,2) + pow(self.legs[i].x_global_goal,2))
                    beta = math.atan2(self.legs[i].x_global_goal, abs(h))
                    psi = ang - beta
                else:
                    rVec = math.sqrt(pow(h,2) + pow(self.legs[i].x_global_goal,2))
                    beta = math.atan2(self.legs[i].x_global_goal, abs(h))
                    psi = ang + beta



                if self.legs[i].side == "left":
                    q.append(-psi)
                else:
                    q.append(psi)

            else:
                q.append(0) # or beta

        q = np.array(q)
        return q

    def getRVectors(self,):

        rVecs = []

        k = (0.5*self.body.length) - (0.5 * self.body.length * math.cos(self.pitch_des))

        for i in range(len(self.legs)):
            if self.legs[i].side == "left":
                rVecs.append(math.sqrt(pow(self.legs[i].z_global_goal,2) + pow(self.legs[i].x_global_goal,2) + pow(self.legs[i].y_global_goal+k,2)))
            else:
                rVecs.append(math.sqrt(pow(self.legs[i].z_global_goal,2) + pow(self.legs[i].x_global_goal,2) + pow(self.legs[i].y_global_goal+k,2)))

        return rVecs

    def getPitchAngle(self, ang):

        q = []

        for i in range(len(self.legs)):
            if self.legs[i].swing == False:

                if self.legs[i].frontBack == "back":
                    h = self.z_local_goal - (0.5*self.body.length) * math.sin(ang)
                else:
                    h = self.z_local_goal + (0.5*self.body.length) * math.sin(ang)
                k = (0.5*self.body.length) - (0.5 * self.body.length * math.cos(ang))

                rVec = math.sqrt(pow(h,2) + pow(self.legs[i].y_global_goal+k,2))

                beta = math.atan2(k-self.legs[i].y_global_goal, abs(h))

                psi = ang + beta

                q.append(psi)

            else:
                q.append(0) # or maybe beta?

        q = np.array(q)
        return q



    def setLegZ(self, leg, z):

        q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, self.legs[0].z_local_goal]))
        q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, self.legs[1].z_local_goal]))
        q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, self.legs[2].z_local_goal]))
        q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, self.legs[3].z_local_goal]))


        if leg == 1:
            q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, z]))

        elif leg == 2:
            q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, z]))

        elif leg == 3:
            q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, z]))

        elif leg == 4:
            q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, z]))

        joints = np.array([q1,q2,q3,q4])

        if leg == 1:
            self.legs[0].z_local_goal = z
        elif leg == 2:
            self.legs[1].z_local_goal = z
        elif leg == 3:
            self.legs[2].z_local_goal = z
        elif leg == 4:
            self.legs[3].z_local_goal = z

        self.sendJointCommand(joints)

    def setLegX(self, leg, x):

        q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, self.legs[0].z_local_goal]))
        q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, self.legs[1].z_local_goal]))
        q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, self.legs[2].z_local_goal]))
        q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, self.legs[3].z_local_goal]))
        if leg == 1:
            q1 = self.legs[0].computeLocalInverseKinematics(np.array([x, self.legs[0].y_local_goal, self.legs[0].z_local_goal]))

        elif leg == 2:
            q2 = self.legs[1].computeLocalInverseKinematics(np.array([x, self.legs[1].y_local_goal, self.legs[1].z_local_goal]))
            q2[0] = -q2[0]
        elif leg == 3:
            q3 = self.legs[2].computeLocalInverseKinematics(np.array([x, self.legs[2].y_local_goal, self.legs[2].z_local_goal]))
            q3[0] = -q3[0]
        elif leg == 4:
            q4 = self.legs[3].computeLocalInverseKinematics(np.array([x, self.legs[3].y_local_goal, self.legs[3].z_local_goal]))

        joints = np.array([q1,q2,q3,q4])

        if leg == 1:
            self.legs[0].x_local_goal = x
        elif leg == 2:
            self.legs[1].x_local_goal = x
        elif leg == 3:
            self.legs[2].x_local_goal = x
        elif leg == 4:
            self.legs[3].x_local_goal = x

        self.sendJointCommand(joints)

    def setAllLegX(self, x):


        q1 = self.legs[0].computeLocalInverseKinematics(np.array([x + self.x_local_goal, self.legs[0].y_local_goal, self.legs[0].z_local_goal]))
        q2 = self.legs[1].computeLocalInverseKinematics(np.array([x - self.x_local_goal, self.legs[1].y_local_goal, self.legs[1].z_local_goal]))
        q2[0] = -q2[0]
        q3 = self.legs[2].computeLocalInverseKinematics(np.array([x - self.x_local_goal, self.legs[2].y_local_goal, self.legs[2].z_local_goal]))
        q3[0] = -q3[0]
        q4 = self.legs[3].computeLocalInverseKinematics(np.array([x + self.x_local_goal, self.legs[3].y_local_goal, self.legs[3].z_local_goal]))

        joints = np.array([q1,q2,q3,q4])

        self.legs[0].x_local_goal = x + self.x_local_goal
        self.legs[1].x_local_goal = x - self.x_local_goal
        self.legs[2].x_local_goal = x - self.x_local_goal
        self.legs[3].x_local_goal = x + self.x_local_goal
        self.x_width_des = x

        self.sendJointCommand(joints)

    def setLegY(self, leg, y):

        q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, self.legs[0].z_local_goal]))
        q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, self.legs[1].z_local_goal]))
        q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, self.legs[2].z_local_goal]))
        q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, self.legs[3].z_local_goal]))

        if leg == 1:
            q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, y, self.legs[0].z_local_goal]))

        elif leg == 2:
            q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, -y, self.legs[1].z_local_goal]))

        elif leg == 3:
            q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, -y, self.legs[2].z_local_goal]))

        elif leg == 4:
            q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, y, self.legs[3].z_local_goal]))

        joints = np.array([q1,q2,q3,q4])

        if leg == 1:
            self.legs[0].y_local_goal = y
        elif leg == 2:
            self.legs[1].y_local_goal = -y
        elif leg == 3:
            self.legs[2].y_local_goal = -y
        elif leg == 4:
            self.legs[3].y_local_goal = y

        self.sendJointCommand(joints)

    def setLegSwing(self, leg, bool):

        if leg == 1:
            self.legs[0].swing = bool

        elif leg == 2:
            self.legs[1].swing = bool

        elif leg == 3:
            self.legs[2].swing = bool

        elif leg == 4:
             self.legs[3].swing = bool

    def updateState(self):

        t_period = 1 / self.stateUpdateFrequency

        while True:
            ts = int(round(time.time() * 1000))

            self.update_positions_leg_from_robot()
            jointArr = []
            swingArr = []
            for i in range(len(self.legs)):
                jointArr.append(self.legs[i].joints)
                swingArr.append(self.legs[i].swing)

            jointArr = np.array(jointArr)
            self.COM = self.getCOM(jointArr, swingArr)
            #err = self.z_local_goal - self.COM[2]

            #self.z_err = self.pidZ(self.COM[2])
            #print(self.z_err, "\t", self.z_local_goal, self.COM[2])
            #print("COM: ", self.COM, "\t PID output: ", self.z_err, "\t measured: ", self.COM[2], "\t err: ", err)

            #if self.readyToWalk == True:
            #    for i in range(len(self.legs)):
            #        print("t: ", self.gait.t, "\t joint ", i, ":    ", self.jointCommands[i] - self.legs[i].joints, "\t swing: ", self.legs[i].swing)

            #print()
            te = int(round(time.time() * 1000))
            t_sleep = (t_period-((te-ts))*0.001)
            if t_sleep > 0:
                time.sleep((t_period-((te-ts))*0.001))

    def update_positions_leg_from_robot(self):
        #updating the independent leg positions.
        current_pos, feet, imu= self.leg_con.get_status()
        self.legs[0].setJointPositions(current_pos[0:3])##front right # correct
        self.legs[2].setJointPositions(-current_pos[3:6])##fron left # correct
        self.legs[3].setJointPositions(current_pos[6:9])##back right # correct
        self.legs[1].setJointPositions(-current_pos[9:12])##back left
        print(self.legs[0].joints, "\t", self.legs[1].joints, "\t", self.legs[2].joints, "\t", self.legs[3].joints, "\t",)
        self.feet_sensor_readings = feet
        self.roll_meas = float(imu[0].split(",")[3])
        self.pitch_meas = float(imu[0].split(",")[2])
        self.yaw_meas = float(imu[0].split(",")[1])

        self.roll_actual = round(self.roll_meas - self.roll_offset,3)
        self.pitch_actual = -round(self.pitch_meas - self.pitch_offset,3)
        self.yaw_actual = round(self.yaw_meas - self.yaw_offset,3)

        if self.angle_pid == True:

            self.roll_err = self.roll_des - self.roll_actual
            self.pitch_err = self.pitch_des - self.pitch_actual
            self.yaw_err = self.yaw_des - self.yaw_actual

        #print(self.roll_actual, self.pitch_actual, self.yaw_actual)


        #fr fl br bl
        #csv_string = str(self.gait.t) + "," + str(self.legs[0].joints).replace("[","").replace("]","").replace(" ", ",") + str(self.legs[1].joints).replace("[","").replace("]","").replace(" ", ",") + str(self.legs[2].joints).replace("[","").replace("]","").replace(" ", ",") + str(self.legs[3].joints).replace("[","").replace("]","").replace(" ", ",") + "," + str(feet[0]) + "," + str(feet[3]) + "," + str(feet[1]) + "," + str(feet[2]) + str(imu[0]) + "\n"
        #self.csvFileReadLog.write(csv_string)
        #self.csvFileReadLog.flush()
    def joint_state_subscriber_callback(self, joint_state):
            joint_state_cp = copy.deepcopy(joint_state)
            joint_state_cp = np.asarray(joint_state_cp.position)
            joint_state_cp[0:3] = joint_state_cp[0:3]
            joint_state_cp[9:12] = joint_state_cp[9:12]
            joint_state_cp[6:9] = joint_state_cp[6:9]
            joint_state_cp[3:6] = joint_state_cp[3:6]


            self.legs[0].setJointPositions(joint_state_cp[0:3]+self.legs[0].joint_initial)
            self.legs[1].setJointPositions(joint_state_cp[6:9]+self.legs[1].joint_initial)
            self.legs[2].setJointPositions(joint_state_cp[9:12]+self.legs[2].joint_initial)
            self.legs[3].setJointPositions(joint_state_cp[3:6]+self.legs[3].joint_initial)

            self.joint_state = np.asarray(joint_state.position)


            self.joint_state[0:3] = np.flip(self.joint_state[0:3])
            self.joint_state[9:12] = np.flip(self.joint_state[9:12])
            self.joint_state[6:9] = np.flip(self.joint_state[6:9])
            self.joint_state[3:6] = np.flip(self.joint_state[3:6])

            jointArr = []
            swingArr = []
            for i in range(len(self.legs)):
                jointArr.append(self.legs[i].joints)
                swingArr.append(self.legs[i].swing)

            jointArr = np.array(jointArr)

            self.COM = self.getCOM(jointArr, swingArr)
