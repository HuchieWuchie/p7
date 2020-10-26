import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
import copy
import math
import cmath
import transform
import threading
import time
import rospy
from gait import Gait
from quadleg import QuadLeg
from quadbody import QuadBody

class Quadruped:

    def __init__(self, simulation=False):

        self.simulation = simulation
        self.body = QuadBody()
        self.joint_state = np.zeros(12)
        self.global_position = np.zeros(3) # to do
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

        self.joint_state_subscriber = rospy.Subscriber('/quadruped/joint_states',JointState,
                                                               self.joint_state_subscriber_callback)

        self.global_position = self.computeLocalForwardKinematics()
        self.x_local_goal = self.global_position[0]
        self.y_local_goal = self.global_position[1]
        self.z_local_goal = self.global_position[2]

        self.jtp = rospy.Publisher('/quadruped/joint_trajectory_controller/command',JointTrajectory,queue_size=1)
        self.gait = Gait(self.legs,frequency = 100)
        self.setTranslationalVelocity(0.0)
        self.setAngularVelocity(0.0)
        self.moveThread = threading.Thread(target=self.move).start()

    def computeLocalForwardKinematics(self):
        # the COM can be thought of as centered on a plane defined by the global
        # x,y position, with the z coordinates from each leg, from this roll and
        # pitch can be extracted as well as z coordinate

        # the following assumes all legs are placed on the ground, we will need
        # to refine the algorithm with ground contact sensors to determine which
        # legs should be part of this equation
        xyz = np.zeros(3)
        nr_contacts = 0
        for i in range(len(self.legs)):
            if self.legs[i].swing == False:
                local_ee = self.legs[i].computeLocalForwardKinematics(self.legs[i].joints)
                xyz[0] += self.legs[i].EE[0]
                xyz[1] += self.legs[i].linkEE[0][1]
                xyz[2] += local_ee[2]
                nr_contacts += 1
        xyz = xyz / nr_contacts
        return xyz

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
        while True:
            ts = int(round(time.time() * 1000))
            transl_delta = 0.25*(self.transl_vel / self.gait.frequency)
            moving, q0,q1,q2,q3 = self.gait.move(self.transl_vel, self.ang_vel, self.legs, self.z_local_goal, self.y_local_goal)
            if moving == False:
                continue
            joints = np.array([q0,q1,q2,q3])
            self.sendJointCommand(joints)

            te = int(round(time.time() * 1000))
            time.sleep((t_period-((te-ts))*0.001))

    def sendJointCommand(self, joint_msg):
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
        jointState.append(q[3][0]) # joint 1 shoulder
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

        q.append(self.legs[0].computeLocalInverseKinematics(np.array([x, self.legs[0].y_local_goal, self.legs[0].z_local_goal])))
        q.append(self.legs[1].computeLocalInverseKinematics(np.array([-x, self.legs[1].y_local_goal, self.legs[1].z_local_goal])))
        q.append(self.legs[2].computeLocalInverseKinematics(np.array([-x, self.legs[2].y_local_goal, self.legs[2].z_local_goal])))
        q.append(self.legs[3].computeLocalInverseKinematics(np.array([x, self.legs[3].y_local_goal, self.legs[3].z_local_goal])))

        if np.array_equal(q1,q[0]) == False:
            if np.array_equal(q2,q[1]) == False:
                if np.array_equal(q3,q[2]) == False:
                    if np.array_equal(q4,q[3]) == False:
                        self.legs[0].x_local_goal = x
                        self.legs[1].x_local_goal = - x
                        self.legs[2].x_local_goal = - x
                        self.legs[3].x_local_goal = x

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


            self.global_position = self.computeLocalForwardKinematics()
