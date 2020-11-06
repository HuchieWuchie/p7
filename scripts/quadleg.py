import numpy as np
import copy
import math
import cmath
import transform
from gait import Gait

class QuadLeg:
    """docstring for ."""

    def __init__(self, joints = np.array([0.0, -1.0, 2.0], dtype='double'), bodyPosition = transform.transformTranslation(0, 0, 0), name="", side="", frontBack="", simulation=False):

        self.joint_initial = np.zeros(3)
        self.simulation = simulation
        self.name=name
        self.side=side
        self.frontBack = frontBack
        self.swing = False # boolean describing swing phase or not
        if self.simulation == True:
            # The initial values from Gazebo simulation, as initial joint
            # positions from Gazebo are [0, 0, 0], so to fix internal
            # kinematics
            self.joint_initial = joints


        joints.dtype = 'double'
        self.joints = np.empty(0, dtype='double')
        self.joints = joints + self.joint_initial
        self.joints_goal = joints
        self.bodyPosition = bodyPosition

        self.linkEE = np.array([
                                [0, 0, 0],
                                [0, 0, 0],
                                [0, 0, 0],
                                [0, 0, 0]], dtype='double')
        self.EE = np.array([0, 0, 0], dtype='double')

        self.l1 = 0.05
        self.l2 = 0.26
        self.l3 = 0.26

        self.__setDHParams()
        self.setJointPositions(self.joints)
        xyz = self.computeLocalForwardKinematics(self.joints)
        self.x_local_goal = xyz[0]
        self.y_local_goal = xyz[1]
        self.z_local_goal = xyz[2]

    def setJointPositions(self, joints=np.array([0.0, -1.0, 2.0])):
        """In radians """
        self.joints = joints

        self.__setDHParams()
        self.__computeForwardKinematics()


    def __setDHParams(self):
        # denavit hartenburg parameters
        # [alpha, a, d, theta]
        self.dhLink1 = np.array([math.pi/2, 0, 0, self.joints[0]])
        self.dhLink2 = np.array([0, self.l1, 0, -math.pi/2])
        self.dhLink3 = np.array([-math.pi/2, 0, 0, self.joints[1]])
        self.dhLink4 = np.array([0, self.l2, 0, self.joints[2]])
        self.dhLink5 = np.array([0, self.l3, 0, 0]) #end effector aka foot

    def __computeForwardKinematics(self):
        self.__setDHParams()

        self.t01 = np.matmul(self.bodyPosition, transform.denavitToTransform(self.dhLink1[0], self.dhLink1[1], self.dhLink1[2], self.dhLink1[3]))
        self.t12 = transform.denavitToTransform(self.dhLink2[0], self.dhLink2[1], self.dhLink2[2], self.dhLink2[3])
        self.t23 = transform.denavitToTransform(self.dhLink3[0], self.dhLink3[1], self.dhLink3[2], self.dhLink3[3])
        self.t34 = transform.denavitToTransform(self.dhLink4[0], self.dhLink4[1], self.dhLink4[2], self.dhLink4[3])
        self.t45 = transform.denavitToTransform(self.dhLink5[0], self.dhLink5[1], self.dhLink5[2], self.dhLink5[3])

        self.t02 = np.matmul(self.t01, self.t12)
        self.t03 = np.matmul(self.t02, self.t23)
        self.t04 = np.matmul(self.t03, self.t34)
        self.t05 = np.matmul(self.t04, self.t45)


        self.linkEE[0] = self.t01[0:3, 3]
        self.linkEE[1] = self.t03[0:3, 3]
        self.linkEE[2] = self.t04[0:3, 3]
        self.linkEE[3] = self.t05[0:3, 3] # end effector aka foot
        self.EE = self.linkEE[3]

    def computeLocalForwardKinematics(self, joint):

        t01 = transform.denavitToTransform(self.dhLink1[0], self.dhLink1[1], self.dhLink1[2], joint[0])
        t12 = transform.denavitToTransform(self.dhLink2[0], self.dhLink2[1], self.dhLink2[2], self.dhLink2[3])
        t23 = transform.denavitToTransform(self.dhLink3[0], self.dhLink3[1], self.dhLink3[2], joint[1])
        t34 = transform.denavitToTransform(self.dhLink4[0], self.dhLink4[1], self.dhLink4[2], joint[2])
        t45 = transform.denavitToTransform(self.dhLink5[0], self.dhLink5[1], self.dhLink5[2], self.dhLink5[3])

        t02 = np.matmul(t01, t12)
        t03 = np.matmul(t02, t23)
        t04 = np.matmul(t03, t34)
        t05 = np.matmul(t04, t45)

        EE = t05[0:3, 3] # end effector aka foot
        return EE

    def computeLocalInverseKinematics(self, position):
        """
        positions are x y z positions for the end effector (leg tip)
        """
        assert position.shape == (3,)
        assert position.dtype.type is np.float64

        x = position[0]
        y = position[1]
        z = position[2]
        l1 = self.l1
        l2 = self.l2
        l3 = self.l3

        self.x_local_goal = x
        self.y_local_goal = y
        self.z_local_goal = z

        R = (x**2 + y**2 + z**2 - l1**2 - l2**2 - l3**2)/(2*l2*l3)
        if R > 1.0:
            #print("Not a feasible position")
            return self.joints - self.joint_initial

        joint = np.zeros(3)

        # functions as long as z < 0 so not above the body
        joint[0] = math.atan2(z,x)-math.atan2(-math.sqrt(x**2 + z**2 - self.l1**2), self.l1)

        # functions as long as q3 > 0 so no q <= 0
        if self.side == "left":
            joint[2] = math.atan2(math.sqrt(1 - R**2), R)

        else:
            joint[2] = math.atan2(-math.sqrt(1 - R**2), R)


        # functions as long as q3 > 0 so AGAIN no 1 <=0
        joint[1] = math.atan2(y, math.sqrt(pow(x,2)+pow(z,2) - pow(l1,2))) - math.atan2(self.l3*math.sin(joint[2]), self.l2+self.l3*math.cos(joint[2]))

        joint = joint - self.joint_initial

        return joint

    def getEEPosition(self):
        return self.EE
