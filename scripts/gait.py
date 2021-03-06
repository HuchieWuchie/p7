import numpy as np
from fractions import Fraction
import time

class Gait:
    """Currently only contains creep gait"""

    def __init__(self, quad, frequency):
        self.leg_swing = {'front_right': False,
                    'front_left': False,
                    'back_right': False,
                    'back_left': False}
        self.t = 0 # paramaterization of gait phase from 0 to 1
        self.frequency = frequency
        self.quad = quad

        self.supportPhase = 0 # Used to keep track of which COM manipulation is needed

        self.setStepHeight(0.1) # currently not used
        self.setStepSize(0.05) # currently not used
        self.setDutyCycle(0.875)
        #self.setDutyCycle(0.751)
        self.setCycleTime(30.0)
        self.setAngularVelocity(0.0)
        self.td1 = 0.375
        self.td2 = 0.875
        self.xDelta = 0
        self.zDelta = 0
        self.stepX = 0

        self.getTResolution()
        self.getCOMTransitionTime()

        self.t0 = 0
        self.t1 = 0

        # phase offsets for front right, back left, front left, back right
        self.phaseOffset = np.array([0.5, self.dutyCycle, 0.0, self.dutyCycle-0.5])

        self.legsDelta = np.zeros((4,3))
        self.tracker = 0

    def setFrequency(self, val):
        """ Set hz for gait calculations """
        self.frequency = max(10, val)

    def setStepHeight(self, h):
        """ Ground clearance height """
        self.stepHeight = h

    def setStepSize(self, s):
        """ Forward step size in y axis """
        self.stepSize = s

    def setCycleTime(self, t):
        """ Time in seconds for a complete gait cycle to complete """
        self.cycleTime = t
        self.getTResolution()
        self.getCOMTransitionTime()

    def setPhase(self, arr):
        """ Set the phase offset for each leg """
        pass

    def getTResolution(self):
        """ get T parameter resolution"""
        self.tRes = (1/self.cycleTime) / self.frequency

    def getStride(self):
        """ The length which the body moves during a gait cycle """
        return self.stepSize

    def setDutyCycle(self, val):
        """ Percentage of gait cycle which a leg will be in support phase """
        self.dutyCycle = max(0.75, min(1.0, val))
        self.getSwingTime()

    def setStepRotX(self,x):
        self.stepRotX = x

    def getStepRotX(self):
        return self.stepRotX + max(-0.05,min(0.05,self.ang_vel))

    def setAngularVelocity(self, vel):
        self.ang_vel = vel
        self.setStepRotX(vel*0.1)

    def getTDistribution(self):
        n = (self.cycleTime * self.swingTime) * self.frequency
        n = int(n)
        self.n = n

        sum = 0
        for i in range(n):
            j = (2/(n-1)) * i
            sum += 1 / (1 + pow(2.2,-1+(2*(j))))

        self.divisionConstant = sum

    def getTVelocity(self, n):
        sum = 0
        for p in range(n):
            j = (2/(self.n)) * p
            res = (1 / (1 + pow(2.2, -1+(2*(j)))))
            sum += res
            #print(p, res, sum)
        sum = sum / self.divisionConstant
        #print(sum)
        return sum


    def getSwingTime(self):
        """ Percentage of gait cycle which a leg will be in swing phase """
        self.swingTime = 1.0 - self.dutyCycle

    def waveGait(self, ang_vel, zCOM, yCOM):
        """ Wave gait, returns leg angles and booleans for swing phase of legs
        """
        #self.ang_vel = ang_vel
        self.zCOM = zCOM
        self.getTDistribution()

        # COM manipulation between swing phases
        # Described in deltas: COM_x, COM_y, legFR_z, legBL_z, legFL_z, legBR_z
        self.COMDelta = np.array([[0, 0, 0, 0.07, 0.07, 0],
                                [0, 0, 0.07, -0.0, -0.0, 0.07]])
        #self.COMDelta = np.array([[0.05, 0, 0, 0.0, 0.0, 0],
        #                        [-0.05, 0, 0.0, -0.0, -0.0, 0.0]])

        # phase offsets for front right, back left, front left, back right
        self.phaseOffset = np.array([0.5, self.dutyCycle-self.swingTime, 0.0, self.dutyCycle-0.5-self.swingTime])

        # uncomment for non wave gaits
        #self.COMDelta = np.array([[0, 0, 0, 0.0, 0.0, 0],
        #                        [0, 0, 0.0, -0.0, -0.0, 0.0]])

        self.phaseOffset = np.array([0.0, 0.0, 0.5, 0.5]) # trot
        #self.phaseOffset = np.array([0.0, 0.5, 0.0, 0.5]) # pronking

        if self.t > 0.0 and self.t <= 0.5:
            self.supportPhase = 0

        elif self.t > 0.5 and self.t <= 1.0:
            self.supportPhase = 1

        legAng = np.zeros((4,3))

        for i in range(len(self.quad.legs)):

            #stepsize control x
            if self.t < 0.1:
                self.stepX = self.getStepRotX()
            stepX = self.stepX

            #stepX = self.stepRotX
            if i == 0:
                stepX = -stepX
            elif i == 1:
                stepX = -stepX
            elif i == 2:
                stepX = stepX
            elif i == 3:
                stepX = stepX

            tStart = self.phaseOffset[i] + self.dutyCycle
            if self.phaseOffset[i] + self.dutyCycle > 1.0:
                tStart = abs(1.0 - (self.dutyCycle + self.phaseOffset[i]))

            if self.t > tStart and self.t < self.swingTime + tStart:

                # do swing cycle here
                t = (self.t - tStart) / self.swingTime # compute swing phase t parameterization resolution
                n = int(t * self.cycleTime * self.swingTime * self.frequency)
                #tRes = self.getTVelocity(n)

                tSwing = self.getTVelocity(n)
                if tSwing > 1.0:
                    tSwing = 1.0

                y, z = self.trajectorySwingYZ(-0, 0, self.stepSize, tSwing)
                x, _ = self.trajectorySwingXZ(0, 0, tSwing, stepX, self.quad.legs[i].name)
                self.quad.legs[i].x_local_goal = x
                self.quad.legs[i].y_local_goal = -y
                self.quad.legs[i].z_local_goal = z
                self.quad.legs[i].swing = False

            else:

                # Do support phase here
                t = 0
                firstSupportPhase = self.dutyCycle + self.phaseOffset[i]
                if firstSupportPhase >= 1.0:
                    firstSupportPhase = abs(1.0 - firstSupportPhase)

                if self.t < firstSupportPhase:
                    t = (self.t + self.dutyCycle - firstSupportPhase) / self.dutyCycle
                else:
                    t = (self.t - firstSupportPhase - self.swingTime) / self.dutyCycle

                z = 0
                xDelta = 0

                if self.t >= self.td1-0.04 and self.t <= self.td1 + self.swingTime + 0.04:
                    if i == 0 or i == 1:
                        z = -0.01
                        if i == 1:
                            z = -0.015
                    if self.t >= self.td1 and self.t <= self.td1 + self.swingTime:
                        z += self.zDelta - (((self.t-self.td1)/self.swingTime) * self.zDelta)
                        xDelta = self.xDelta - (((self.t-self.td1)/self.swingTime) * self.xDelta)

                elif self.t >= self.td2-0.04 and self.t <= self.td2 + self.swingTime + 0.04:
                    if i == 2 or i == 3:
                        z = -0.01
                    if self.t >= self.td2 and self.t <= self.td2 + self.swingTime:
                        z += self.zDelta - (((self.t-self.td2)/self.swingTime) * self.zDelta)
                        xDelta = self.xDelta - (((self.t-self.td2)/self.swingTime) * self.xDelta)

                xDelta = - xDelta

                x, _ = self.trajectoryStanceXZ(-xDelta, 0, stepX, t)
                #print(i, t, x)

                #print("t: ", self.t, " i: ", i, "  z: ", z)
                y, z = self.trajectoryStance(0, z, self.stepSize, t)

                self.quad.legs[i].x_local_goal = x
                self.quad.legs[i].y_local_goal = -y
                self.quad.legs[i].z_local_goal = z
                self.quad.legs[i].swing = False



        swing = []
        for i in range(len(self.quad.legs)):
            swing.append(self.quad.legs[i].swing)
        self.t += self.tRes
        if self.t > 1.0:
            self.t = 0.0
        #print()

    def discontinuousGait(self, zCOM, yCOM):
        """ Continuous gait, returns leg angles and booleans for swing phase of legs
        """
        self.zCOM = zCOM

        # phase offsets for front right, back left, front left, back right
        self.phaseOffset = np.array([0.5, self.dutyCycle, 0.0, self.dutyCycle-0.5])
        self.phaseOffset -= self.swingTime

        # COM manipulation between swing phases
        # Described in deltas: COM_x, COM_y, legFR_z, legBL_z, legFL_z, legBR_z
        self.COMDelta = np.array([[0, 0, 0, 0.07, 0.07, 0],
                                [0, 0, 0.07, -0.0, -0.0, 0.07]])

        if self.t > 0.0 and self.t <= 0.375:
            self.supportPhase = 0

        #if self.t > 0.5 and self.t <= 0.625:
        #    self.supportPhase = 1

        elif self.t > 0.5 and self.t <= 0.875:
            self.supportPhase = 1

        legAng = np.zeros((4,3))
        self.getDelta()

        for i in range(len(self.quad.legs)):

            x = self.quad.x_local_goal + self.legsDelta[i][0]
            y = self.quad.legs[i].y_local_goal + self.legsDelta[i][1]
            z = self.zCOM + self.legsDelta[i][2]

            legAng[i] = self.quad.legs[i].computeLocalInverseKinematics(np.array([x,y,z]))
            self.quad.legs[i].swing = False

            tStart = self.phaseOffset[i] + self.dutyCycle
            if self.phaseOffset[i] + self.dutyCycle > 1.0:
                tStart = abs(1.0 - (self.dutyCycle + self.phaseOffset[i]))

            if self.t > tStart and self.t < self.swingTime + tStart:

                # do swing cycle here
                if self.tracker == 0:
                    if self.quad.legs[i].side == "right":
                        self.y_leg_goal = -self.quad.legs[i].y_local_goal
                    else:
                        self.y_leg_goal = self.quad.legs[i].y_local_goal
                    self.tracker = 1
                t = (self.t - tStart) / self.swingTime # compute swing phase t parameterization resolution
                #y, z = self.trajectorySwingYZ(-yCOM+self.stepSize, zCOM, self.stepSize, t)
                y, z = self.trajectorySwingYZ(self.y_leg_goal, zCOM, self.stepSize, t)
                x = self.quad.legs[i].x_local_goal
                if self.quad.legs[i].side == "right":
                    y = -y
                legAng[i] = self.quad.legs[i].computeLocalInverseKinematics(np.array([x, y, z]))
                if t >= 1.0 - self.tRes/self.swingTime:
                    self.quad.legs[i].y_local_goal = y
                    self.tracker = 0
                self.quad.legs[i].swing = True

            if self.t > 0.875:
                if self.tracker == 0:
                    self.tracker = 1
                    self.y_leg_goal = self.quad.legs[i].y_local_goal

                t = (self.t - 0.875) / self.swingTime # compute swing phase t parameterization resolution
                y, z = self.trajectoryStance(-yCOM, zCOM, self.stepSize, t)
                if self.quad.legs[i].side == "right":
                    y = -y

                legAng[i] = self.quad.legs[i].computeLocalInverseKinematics(np.array([self.quad.legs[i].x_local_goal, y, z]))

        swing = []
        for i in range(len(self.quad.legs)):
            swing.append(self.quad.legs[i].swing)
        self.t += self.tRes
        if self.t > 1.0:
            self.t = 0.0
            self.tracker = 0


        return swing, legAng[0], legAng[1], legAng[2], legAng[3]

    def getCOMTransitionTime(self):
        """" sets the time to manipulate COM between phases """
        denominator = Fraction(self.dutyCycle).limit_denominator().denominator
        self.COMTime = self.tRes / (((denominator - 4) / 4.0 ) / denominator)

    def getDelta(self):

        prevIdx = self.supportPhase - 1
        if self.supportPhase > 4:
            self.supportPhase = 0
            prevIdx = -1

        #if self.quad.COM[0] > self.COMDelta[self.supportPhase][0]:
        #     self.quad.deltaX(-abs((self.COMDelta[self.supportPhase][0]-self.COMDelta[prevIdx][0]) * self.COMTime))

        #else:
        #    self.quad.deltaX(abs((self.COMDelta[self.supportPhase][0]-self.COMDelta[prevIdx][0]) * self.COMTime))
        for i in range(len(self.legsDelta)):

            if self.legsDelta[i][2] > self.COMDelta[self.supportPhase][2+i]:
                self.legsDelta[i][2] -= abs((self.COMDelta[self.supportPhase][2+i]-self.COMDelta[prevIdx][2+i]) * self.COMTime)
            else:
                self.legsDelta[i][2] += abs((self.COMDelta[self.supportPhase][2+i]-self.COMDelta[prevIdx][2+i]) * self.COMTime)

            if self.legsDelta[i][1] > self.COMDelta[self.supportPhase][1]:
                self.legsDelta[i][1] -= abs((self.COMDelta[self.supportPhase][1]-self.COMDelta[prevIdx][1]) * self.COMTime)
            else:
                self.legsDelta[i][1] += abs((self.COMDelta[self.supportPhase][1]-self.COMDelta[prevIdx][1]) * self.COMTime)


        #print(self.supportPhase, self.legsDelta, abs((self.COMDelta[self.supportPhase][5]-self.COMDelta[prevIdx][5]) * self.COMTime), self.COMDelta[self.supportPhase][5])
    def bezierCurveCLinear(self, p0, p1, t):
        """returns position as a function of parameter t"""

        if t <= 0:
            return p1[0], p1[1]

        elif t >= 1:
            return p0[0], p0[1]

        position = (1-t)*p0 + t*p1

        return position[0], position[1]

    def bezierCurveCubic(self, p0, p1, p2, p3, t):
        """returns position as a function of parameter t"""

        if t < 0 or t > 1:
            return p0, 0, 0

        position = pow((1-t),3)*p0 + 3*pow((1-t),2)*t*p1 + 3*(1-t)*pow(t,2)*p2 + pow(t,3)*p3

        return position[0], position[1]

    def trajectorySwingYZ(self, y0, z0, stepSize, t):
        """ Creates a trajectory for the swing phase in the YZ plane,
        for linear movement """
        p0 = np.array([y0-(0.5*stepSize), z0])
        p1 = np.array([y0-(0.5*stepSize), z0+self.stepHeight])
        p2 = np.array([y0+(0.5*stepSize)+(0.5*stepSize), z0+self.stepHeight])
        p3 = np.array([y0+(0.5*stepSize), z0])
        y, z = self.bezierCurveCubic(p0,p1,p2,p3,t)
        return y, z

    def trajectorySwingXZ(self, x0, z0, t, stepX, name=""):
        """ Creates a trajectory for the swing phase in the XZ plane,
        for turning. """
        #if name=="left_front":
        #    ang_vel = -ang_vel
        #elif name=="right_back":
        #    ang_vel = -ang_vel
        p0 = np.array([x0, z0])
        p1 = np.array([x0, z0+self.stepHeight])
        p2 = np.array([x0+stepX+(0.5*stepX), z0+self.stepHeight])
        p3 = np.array([x0+stepX, z0])
        x, z = self.bezierCurveCubic(p0,p1,p2,p3,t)
        return x, z

    def trajectoryStance(self, y0, z0, stepSize, t):
        """ Support phase trajectory """
        p0 = np.array([y0+(0.5*stepSize), z0])
        p1 = np.array([y0-(0.5*stepSize), z0])
        y, z = self.bezierCurveCLinear(p0, p1, t)
        #p0 = np.array([y0+stepSize, z0])
        #p1 = np.array([y0+stepSize, z0-0.04])
        #p2 = np.array([y0, z0])
        #p3 = np.array([y0, z0])
        #y, z = self.bezierCurveCubic(p0, p1, p2, p3, t)
        return y, z

    def trajectoryStanceXZ(self, x0, z0, stepX, t):
        """ Support phase trajectory """
        t = 2*t
        if t > 0.98:
            t = 0.99
        p0 = np.array([x0+stepX, z0])
        p1 = np.array([x0, z0])
        x, z = self.bezierCurveCLinear(p0, p1, t)
        #p0 = np.array([y0+stepSize, z0])
        #p1 = np.array([y0+stepSize, z0-0.04])
        #p2 = np.array([y0, z0])
        #p3 = np.array([y0, z0])
        #y, z = self.bezierCurveCubic(p0, p1, p2, p3, t)
        return x, z
