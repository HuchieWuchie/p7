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

class Quadruped:

    def __init__(self, simulation=False):

        if simulation == True:
            import rospy
            from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from sensor_msgs.msg import JointState

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


        jointArr = []
        swingArr = []
        for i in range(len(self.legs)):
            jointArr.append(self.legs[i].joints)
            swingArr.append(self.legs[i].swing)

        jointArr = np.array(jointArr)

        self.COM = self.getCOM(jointArr, swingArr)
        self.x_local_goal = self.COM[0]
        self.y_local_goal = self.COM[1]
        self.z_local_goal = self.COM[2]


        self.gait = Gait(self.legs,frequency = 10)
        self.commandFrequency = 30
        self.setTranslationalVelocity(0.0)
        self.setAngularVelocity(0.0)
        self.gaitStyle = 0
        self.gait.phase = 1
        self.readyToWalk = False
        self.moveThread = threading.Thread(target=self.move).start()

        if self.simulation == False:
            self.leg_con = leg_connection(name_serial_port='/dev/ttyACM0')
            self.time_frequency = time.time()


    def changeGait(self):
        self.gaitStyle += 1
        if self.gaitStyle > 2:
            self.gaitStyle = 0

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
        y_local_goal = 0
        stepSize = 0.1 # in meters
        stanceVel = 0.0002 # arbitrary unit, depends on gait frequency
        while True:
            ts = int(round(time.time() * 1000))

            if self.readyToWalk == True:
                """
                if self.gaitStyle == 0:
                    moving, q0,q1,q2,q3 = self.gait.creep(self.transl_vel, self.ang_vel, self.legs, self.z_local_goal, self.y_local_goal)
                    joints = np.array([q0,q1,q2,q3])
                    self.sendJointCommand(joints)
                elif self.gaitStyle == 1:
                    moving, q0,q1,q2,q3 = self.gait.trot(self.transl_vel, self.ang_vel, self.legs, self.z_local_goal, self.y_local_goal)
                    joints = np.array([q0,q1,q2,q3])
                    self.sendJointCommand(joints)


                elif self.gaitStyle == 2:
                    """
                if self.gait.phase == 3 and self.gait.t == 0.0:
                    y_local_goal = -self.legs[2].y_local_goal
                if self.gait.phase == 4 and self.gait.t == 0.0:
                    y_local_goal = self.legs[0].y_local_goal

                if self.gait.phase == 6 and self.gait.t == 0.0:
                    y_local_goal = self.legs[3].y_local_goal
                if self.gait.phase == 7 and self.gait.t == 0.0:
                    y_local_goal = -self.legs[1].y_local_goal
                moving, q0,q1,q2,q3 = self.gait.stableCreep(0.15, self.legs, self.z_local_goal, y_local_goal)
                joints = np.array([q0,q1,q2,q3])
                self.sendJointCommand(joints)

                if self.gait.phase == 1:
                    self.yCOMdesired = 0.0
                    if self.COM[1] < self.yCOMdesired + .01 and self.COM[1] > self.yCOMdesired - .01:
                        self.gait.phase += 1
                    elif self.yCOMdesired > self.COM[1]:
                        self.setYIncrement(stanceVel)
                    elif self.yCOMdesired < self.COM[1]:
                        self.setYIncrement(-stanceVel)



                elif self.gait.phase == 2:
                    # move COM towards y -0.13
                    self.yCOMdesired = -0.08
                    if self.COM[1] < self.yCOMdesired + .01 and self.COM[1] > self.yCOMdesired - .01:
                        self.gait.phase += 1
                    elif self.yCOMdesired > self.COM[1]:
                        self.setYIncrement(stanceVel)
                    elif self.yCOMdesired < self.COM[1]:
                        self.setYIncrement(-stanceVel)

                elif self.gait.phase == 5:
                    # move COM towards y 0.13
                    self.yCOMdesired = 0.08
                    if self.COM[1] < self.yCOMdesired + .01 and self.COM[1] > self.yCOMdesired - .01:
                        self.gait.phase += 1
                    elif self.yCOMdesired > self.COM[1]:
                        self.setYIncrement(stanceVel)
                    elif self.yCOMdesired < self.COM[1]:
                        self.setYIncrement(-stanceVel)


                elif self.gait.phase == 8:
                    # move COM towards y 0.0
                    self.yCOMdesired = 0.0
                    if self.COM[1] < self.yCOMdesired + .01 and self.COM[1] > self.yCOMdesired - .01:
                        self.gait.phase += 1
                    elif self.yCOMdesired > self.COM[1]:
                        self.setYIncrement(stanceVel)
                    elif self.yCOMdesired < self.COM[1]:
                        self.setYIncrement(-stanceVel)

            te = int(round(time.time() * 1000))
            t_sleep = (t_period-((te-ts))*0.001)
            if t_sleep > 0:
                time.sleep((t_period-((te-ts))*0.001))


    def sendJointCommand(self, joints):
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

    def setX(self, x):

        q1 = self.legs[0].joints
        q2 = self.legs[1].joints
        q3 = self.legs[2].joints
        q4 = self.legs[3].joints

        q = []

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal + x, self.legs[0].y_local_goal, self.legs[0].z_local_goal])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal -x, self.legs[1].y_local_goal, self.legs[1].z_local_goal])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal -x, self.legs[2].y_local_goal, self.legs[2].z_local_goal])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal + x, self.legs[3].y_local_goal, self.legs[3].z_local_goal])))

        if np.array_equal(q1,q[0]) == False:
            if np.array_equal(q2,q[1]) == False:
                if np.array_equal(q3,q[2]) == False:
                    if np.array_equal(q4,q[3]) == False:
                        self.legs[0].x_local_goal += x
                        self.legs[1].x_local_goal += - x
                        self.legs[2].x_local_goal += - x
                        self.legs[3].x_local_goal += x

        q = np.array(q)
        self.sendJointCommand(q)

    def setY(self, y):

        q1 = self.legs[0].joints
        q2 = self.legs[1].joints
        q3 = self.legs[2].joints
        q4 = self.legs[3].joints

        q = []

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, y, self.legs[0].z_local_goal])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, -y, self.legs[1].z_local_goal])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, -y, self.legs[2].z_local_goal])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, y, self.legs[3].z_local_goal])))

        if np.array_equal(q1,q[0]) == False:
            if np.array_equal(q2,q[1]) == False:
                if np.array_equal(q3,q[2]) == False:
                    if np.array_equal(q4,q[3]) == False:
                        self.legs[0].y_local_goal = y
                        self.legs[1].y_local_goal =  -y
                        self.legs[2].y_local_goal =  -y
                        self.legs[3].y_local_goal =  y
                        self.y_local_goal = y

        q = np.array(q)
        self.sendJointCommand(q)

    def setYIncrement(self, delta):

        q1 = self.legs[0].joints
        q2 = self.legs[1].joints
        q3 = self.legs[2].joints
        q4 = self.legs[3].joints

        q = []
        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, self.legs[0].z_local_goal])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, self.legs[1].z_local_goal])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, self.legs[2].z_local_goal])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, self.legs[3].z_local_goal])))

        if np.array_equal(q1,q[0]) == False:
            if np.array_equal(q2,q[1]) == False:
                if np.array_equal(q3,q[2]) == False:
                    if np.array_equal(q4,q[3]) == False:
                        self.legs[0].y_local_goal +=  delta
                        self.legs[1].y_local_goal -=  delta
                        self.legs[2].y_local_goal -=  delta
                        self.legs[3].y_local_goal +=  delta
                        self.y_local_goal += delta
        q = np.array(q)
        self.sendJointCommand(q)


    def setZ(self, z):

        q = []

        q1 = self.legs[0].joints
        q2 = self.legs[1].joints
        q3 = self.legs[2].joints
        q4 = self.legs[3].joints

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, z])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, z])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, z])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, z])))

        if np.array_equal(q1,q[0]) == False:
            if np.array_equal(q2,q[1]) == False:
                if np.array_equal(q3,q[2]) == False:
                    if np.array_equal(q4,q[3]) == False:
                        self.legs[0].z_local_goal = z
                        self.legs[1].z_local_goal = z
                        self.legs[2].z_local_goal = z
                        self.legs[3].z_local_goal = z
                        self.z_local_goal = z

        q = np.array(q)
        self.sendJointCommand(q)

    def setLegYZ(self, leg, y,z):

        q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, self.legs[0].y_local_goal, self.legs[0].z_local_goal]))
        q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, self.legs[1].y_local_goal, self.legs[1].z_local_goal]))
        q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, self.legs[2].y_local_goal, self.legs[2].z_local_goal]))
        q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, self.legs[3].y_local_goal, self.legs[3].z_local_goal]))

        if leg == 1:
            q1 = self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local_goal, y, z]))
            self.legs[0].y_local_goal = y
            self.legs[0].z_local_goal = z
        elif leg == 2:
            q2 = self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local_goal, -y, z]))
            self.legs[1].y_local_goal = -y
            self.legs[1].z_local_goal = z
        elif leg == 3:
            q3 = self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local_goal, -y, z]))
            self.legs[2].y_local_goal = -y
            self.legs[2].z_local_goal = z
        elif leg == 4:
            q4 = self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local_goal, y, z]))
            self.legs[3].y_local_goal = y
            self.legs[3].z_local_goal = z
        joints = np.array([q1,q2,q3,q4])

        self.sendJointCommand(joints)

    def setRoll(self, ang):

        q = []

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local, self.legs[0].y_local, self.legs[0].z_local-ang])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local, self.legs[1].y_local, self.legs[1].z_local+ang])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local, self.legs[2].y_local, self.legs[2].z_local+ang])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local, self.legs[3].y_local, self.legs[3].z_local-ang])))

        jointState = self.__makeJointStateMsg(q)

        return jointState

    def setPitch(self, ang):

        q = []

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local, self.legs[0].y_local, self.legs[0].z_local-ang])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local, self.legs[1].y_local, self.legs[1].z_local-ang])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local, self.legs[2].y_local, self.legs[2].z_local+ang])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local, self.legs[3].y_local, self.legs[3].z_local+ang])))

        jointState = self.__makeJointStateMsg(q)

        return jointState

    def setYaw(self, ang):

        q = []

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([self.legs[0].x_local+ang, self.legs[0].y_local, self.legs[0].z_local])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([self.legs[1].x_local-ang, self.legs[1].y_local, self.legs[1].z_local])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([self.legs[2].x_local+ang, self.legs[2].y_local, self.legs[2].z_local])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([self.legs[3].x_local-ang, self.legs[3].y_local, self.legs[3].z_local])))

        jointState = self.__makeJointStateMsg(q)

        return jointState

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

    def updateState(self):

        self.update_positions_leg_from_robot()

        jointArr = []
        swingArr = []
        for i in range(len(self.legs)):
            jointArr.append(self.legs[i].joints)
            swingArr.append(self.legs[i].swing)

        jointArr = np.array(jointArr)
        self.COM = self.getCOM(jointArr, swingArr)

    def update_positions_leg_from_robot(self):
        #updating the independent leg positions.
        current_pos,__ =self.leg_con.read_leg_status()
        self.legs[0].setJointPositions(current_pos[0:3])##front right
        self.legs[2].setJointPositions(current_pos[3:6])##fron left
        self.legs[3].setJointPositions(current_pos[6:9])##back right
        self.legs[1].setJointPositions(current_pos[9:12])##back left

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
