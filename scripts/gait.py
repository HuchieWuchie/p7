import numpy as np

class Gait:
    """Currently only contains creep gait"""

    def __init__(self, legs,frequency):
        self.leg_swing = {'front_right': False,
                    'front_left': False,
                    'back_right': False,
                    'back_left': False}
        self.t = 0 # paramaterization of gait phase from 0 to 1
        self.frequency = frequency

        self.setHeight(0.2) # currently not used
        self.setStepSize(0.00) # currently not used
        self.setPhaseTime(0.2)
        self.phase = 1
        self.y0 = -legs[0].y_local_goal
        self.y1 = legs[1].y_local_goal
        self.y2 = legs[2].y_local_goal
        self.y3 = -legs[3].y_local_goal

        self.t0 = 0
        self.t1 = 0
        self.t2 = 0
        self.t3 = 0
        self.i = 0

    def setHeight(self, h):
        self.height = h

    def setStepSize(self, s):
        self.stepSize = s

    def setPhaseTime(self, t):
        self.phaseTime = t

    def getStepSize(self):
        self.stepSize = self.transl_vel * self.phaseTime

    def creep(self, transl_vel, ang_vel, legs, current_z, current_y):
        """ input translational velocity and angular velocity in m/s and rad/s
            output end effector positions for each leg that satisfies that
        """

        self.transl_vel = 4*transl_vel
        self.ang_vel = ang_vel
        self.getStepSize()
        self.getTResolution()
        self.t = self.t + self.t_res

        if self.t > 1.0:
            self.t = 1.0

        legAng = []

        self.t0 += 0.25*self.t_res
        self.t1 += 0.25*self.t_res
        self.t2 += 0.25*self.t_res
        self.t3 += 0.25*self.t_res
        if self.t0 > 1.0:
            self.t0 = 1.0
        if self.t1 > 1.0:
            self.t1 = 1.0
        if self.t2 > 1.0:
            self.t2 = 1.0
        if self.t3 > 1.0:
            self.t3 = 1.0


        # When self.i < 5. compute standing legs for legs not moving
        for i in range(len(legs)):
            legAng.append(legs[i].computeLocalInverseKinematics(np.array([legs[i].x_local_goal, legs[i].y_local_goal, legs[i].z_local_goal])))

        if self.i >= 3:
            y, z = self.trajectoryStance(-current_y, current_z, self.t0)
            legFR = legs[0].computeLocalInverseKinematics(np.array([legs[0].x_local_goal, -y, z]))
            legAng[0] = legFR

        if self.i >= 2:
            y, z = self.trajectoryStance(-current_y, current_z, self.t1)
            legBL = legs[1].computeLocalInverseKinematics(np.array([legs[1].x_local_goal, y, z]))
            legAng[1] = legBL

        if self.i >= 1:
            y, z = self.trajectoryStance(-current_y, current_z, self.t2)
            legFL = legs[2].computeLocalInverseKinematics(np.array([legs[2].x_local_goal, y, z]))
            legAng[2] = legFL

        if self.i > 4:
            y, z = self.trajectoryStance(-current_y, current_z, self.t3)
            legBR = legs[3].computeLocalInverseKinematics(np.array([legs[3].x_local_goal, -y, z]))
            legAng[3] = legBR

        # close the phase loop
        if self.phase > 4:
            self.phase = 1

        if self.phase == 1:
            self.leg_swing['front_left'] = True
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = True
            legs[3].swing = False

            if self.t == 1.0:
                self.t2 = 0.0
                self.leg_swing['front_left'] = False
                legs[2].swing = False

        elif self.phase == 2:
            self.leg_swing['back_left'] = True
            legs[0].swing = False
            legs[1].swing = True
            legs[2].swing = False
            legs[3].swing = False

            if self.t == 1.0:
                self.t1 = 0.0
                self.leg_swing['back_left'] = False
                legs[1].swing = False
                self.i += 1

        elif self.phase == 3:
            self.leg_swing['front_right'] = True
            legs[0].swing = True
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

            if self.t == 1.0:
                self.t0 = 0.0
                self.leg_swing['front_right'] = False
                legs[0].swing = False
                self.i += 1

        elif self.phase == 4:
            self.leg_swing['back_right'] = True
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = True

            if self.t == 1.0:
                self.t3 = 0.0
                self.leg_swing['back_right'] = False
                legs[3].swing = False
                self.i += 1

        for i in range(len(legs)):
            if legs[i].swing == True:
                y, z = self.trajectorySwingYZ(-current_y, current_z, self.t)
                x_goal = legs[i].x_local_goal
                x, _ = self.trajectorySwingXZ(legs[i].x_local_goal, current_z, self.t, legs[i].name)
                if legs[i].side == "right":
                    y = -y
                legAng[i] = legs[i].computeLocalInverseKinematics(np.array([x, y, z]))
                legs[i].x_local_goal = x_goal

        if self.t == 1.0:
            self.t = 0.0
            self.phase += 1

        return self.leg_swing, legAng[0], legAng[1], legAng[2], legAng[3]

    def stableCreep(self, stepSize, legs, current_z, current_y):
        """ input translational velocity and angular velocity in m/s and rad/s
            output end effector positions for each leg that satisfies that
        """

        self.stepSize = stepSize
        self.t_res = 0.3 / self.frequency
        if self.phase == 3 or self.phase == 4 or self.phase == 6 or self.phase == 7:
            self.t = self.t + self.t_res
        if self.t > 1.0:
            self.t = 1.0

        legAng = []

        # When self.i < 5. compute standing legs for legs not moving
        for i in range(len(legs)):
            legAng.append(legs[i].computeLocalInverseKinematics(np.array([legs[i].x_local_goal, legs[i].y_local_goal, legs[i].z_local_goal])))

        # close the phase loop
        if self.phase > 8:
            self.phase = 1

        if self.phase == 1:
            # Initial position
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = False
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

        elif self.phase == 2:
            # relocate COM to -0.13
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = False
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

        elif self.phase == 3:
            self.leg_swing['front_left'] = True
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = False
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = True
            legs[3].swing = False

            if self.t == 1.0:
                self.t = 0.0
                self.phase += 1
                self.leg_swing['front_left'] = False
                legs[2].swing = False

        elif self.phase == 4:
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = True
            self.leg_swing['back_right'] = False
            legs[0].swing = True
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

            if self.t == 1.0:
                self.t = 0.0
                self.phase += 1
                self.leg_swing['front_right'] = False
                legs[0].swing = False

        elif self.phase == 5:
            # relocate COM to 0.13
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = False
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

        elif self.phase == 6:
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = True
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = True

            if self.t == 1.0:
                self.t = 0.0
                self.phase += 1
                self.leg_swing['back_right'] = False
                legs[3].swing = False

        elif self.phase == 7:
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = True
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = False
            legs[0].swing = False
            legs[1].swing = True
            legs[2].swing = False
            legs[3].swing = False

            if self.t == 1.0:
                self.t = 0.0
                self.phase += 1
                self.leg_swing['back_left'] = False
                legs[1].swing = False

        elif self.phase == 8:
            # relocate COM to 0.0
            self.leg_swing['front_left'] = False
            self.leg_swing['back_left'] = False
            self.leg_swing['front_right'] = False
            self.leg_swing['back_right'] = False
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

        for i in range(len(legs)):
            if legs[i].swing == True:
                y, z = self.trajectorySwingYZ(-current_y, current_z, self.t)
                if legs[i].side == "right":
                    y = -y

                legAng[i] = legs[i].computeLocalInverseKinematics(np.array([legs[i].x_local_goal, y, z]))
                legs[i].y_local_goal = y

        return self.leg_swing, legAng[0], legAng[1], legAng[2], legAng[3]

    def trot(self, transl_vel, ang_vel, legs, current_z, current_y):
        """ input translational velocity and angular velocity in m/s and rad/s
            output end effector positions for each leg that satisfies that
        """
        self.setPhaseTime(0.4)
        self.transl_vel = 4*transl_vel
        self.ang_vel = ang_vel
        self.getStepSize()
        self.getTResolution()
        self.t = self.t + self.t_res
        if self.t > 1.0:
            self.t = 1.0

        legAng = []

        self.t0 += self.t_res
        self.t1 += self.t_res
        self.t2 += self.t_res
        self.t3 += self.t_res
        if self.t0 > 1.0:
            self.t0 = 1.0
        if self.t1 > 1.0:
            self.t1 = 1.0
        if self.t2 > 1.0:
            self.t2 = 1.0
        if self.t3 > 1.0:
            self.t3 = 1.0


        # When self.i < 5. compute standing legs for legs not moving
        for i in range(len(legs)):
            legAng.append(legs[i].computeLocalInverseKinematics(np.array([legs[i].x_local_goal, legs[i].y_local_goal, legs[i].z_local_goal])))

        if self.i >= 2:
            y, z = self.trajectoryStance(-current_y, current_z, self.t0)
            legFR = legs[0].computeLocalInverseKinematics(np.array([legs[0].x_local_goal, -y, z]))
            legAng[0] = legFR

        if self.i >= 2:
            y, z = self.trajectoryStance(-current_y, current_z, self.t1)
            legBL = legs[1].computeLocalInverseKinematics(np.array([legs[1].x_local_goal, y, z]))
            legAng[1] = legBL

        if self.i >= 1:
            y, z = self.trajectoryStance(-current_y, current_z, self.t2)
            legFL = legs[2].computeLocalInverseKinematics(np.array([legs[2].x_local_goal, y, z]))
            legAng[2] = legFL

        if self.i > 1:
            y, z = self.trajectoryStance(-current_y, current_z, self.t3)
            legBR = legs[3].computeLocalInverseKinematics(np.array([legs[3].x_local_goal, -y, z]))
            legAng[3] = legBR

        # close the phase loop
        if self.phase > 2:
            self.phase = 1

        if self.phase == 1:
            self.leg_swing['front_left'] = True
            self.leg_swing['back_right'] = True
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = True
            legs[3].swing = True

            if self.t == 1.0:
                self.t2 = 0.0
                self.t3 = 0.0
                self.leg_swing['front_left'] = False
                self.leg_swing['back_right'] = False
                legs[2].swing = False
                legs[3].swing = False

        elif self.phase == 2:
            self.leg_swing['back_left'] = True
            self.leg_swing['front_right'] = True
            legs[0].swing = True
            legs[1].swing = True
            legs[2].swing = False
            legs[3].swing = False

            if self.t == 1.0:
                self.t1 = 0.0
                self.t0 = 0.0
                self.leg_swing['back_left'] = False
                self.leg_swing['front_right'] = False
                legs[1].swing = False
                legs[0].swing = False
                self.i += 1

        for i in range(len(legs)):
            if legs[i].swing == True:
                y, z = self.trajectorySwingYZ(-current_y, current_z, self.t)
                x_goal = legs[i].x_local_goal
                x, _ = self.trajectorySwingXZ(legs[i].x_local_goal, current_z, self.t, legs[i].name)
                if legs[i].side == "right":
                    y = -y
                #elif legs[i].side == "right":
                #    if legs[i].frontBack == "back":
                #        x = -x
                #legAng[i] = legs[i].computeLocalInverseKinematics(np.array([legs[i].x_local_goal, y, z]))
                legAng[i] = legs[i].computeLocalInverseKinematics(np.array([x, y, z]))
                legs[i].x_local_goal = x_goal

        if self.t == 1.0:
            self.t = 0.0
            self.phase += 1

        return self.leg_swing, legAng[0], legAng[1], legAng[2], legAng[3]

    def getTResolution(self):
        """ get T parameter based on translational velocity """
        self.t_res = (1/self.phaseTime) / self.frequency

    def bezierCurveCLinear(self, p0, p1, t):
        """returns position, velocity and acceleration as a function of parameter t"""

        if t < 0 or t > 1:
            return p0, 0, 0

        position = (1-t)*p0 + t*p1

        return position[0], position[1]

    def bezierCurveCubic(self, p0, p1, p2, p3, t):
        """returns position, velocity and acceleration as a function of parameter t"""

        if t < 0 or t > 1:
            return p0, 0, 0

        position = pow((1-t),3)*p0 + 3*pow((1-t),2)*t*p1 + 3*(1-t)*pow(t,2)*p2 + pow(t,3)*p3

        return position[0], position[1]

    def trajectorySwingYZ(self, y0, z0, t):
        p0 = np.array([y0, z0])
        p1 = np.array([y0, z0+self.height])
        p2 = np.array([y0+self.stepSize+(0.5*self.stepSize), z0+self.height])
        p3 = np.array([y0+self.stepSize, z0])
        y, z = self.bezierCurveCubic(p0,p1,p2,p3,t)
        return y, z

    def trajectorySwingXZ(self, x0, z0, t, name=""):
        ang_vel = 0.1*self.ang_vel
        if name=="left_front":
            ang_vel = -ang_vel
        elif name=="right_back":
            ang_vel = -ang_vel
        p0 = np.array([x0, z0])
        p1 = np.array([x0, z0+self.height])
        p2 = np.array([x0+ang_vel+(0.5*ang_vel), z0+self.height])
        p3 = np.array([x0+ang_vel, z0])
        x, z = self.bezierCurveCubic(p0,p1,p2,p3,t)
        return x, z

    def trajectoryStance(self, y0, z0, t):
        p0 = np.array([y0+self.stepSize, z0])
        p1 = np.array([y0, z0])
        y, z = self.bezierCurveCLinear(p0, p1, t)
        return y, z
