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
        self.phase = 0
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

    def move(self, transl_vel, ang_vel, legs, current_z, current_y):
        """ input translational velocity and angular velocity in m/s and rad/s
            output end effector positions for each leg that satisfies that
        """
        #print(current_z, " \t",current_z+self.height)
        if transl_vel == 0 and ang_vel == 0:
            self.phase = 0
            self.t = 0
            self.i = 0
            return False, 0, 0, 0, 0

        if transl_vel != 0 or ang_vel != 0:
            if self.phase == 0:
                self.phase = 1
                self.i = 1


        self.transl_vel = 4*transl_vel
        self.ang_vel = ang_vel
        self.getTResolution()
        self.t = self.t + self.t_res
        if self.t > 1.0:
            self.t = 1.0

        self.y0 = -current_y
        self.y1 = -current_y
        self.y2 = -current_y
        self.y3 = -current_y

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

        legFR = legs[0].computeLocalInverseKinematics(np.array([legs[0].x_local_goal, legs[0].y_local_goal, legs[0].z_local_goal]))
        legBL = legs[1].computeLocalInverseKinematics(np.array([legs[1].x_local_goal, legs[1].y_local_goal, legs[1].z_local_goal]))
        legFL = legs[2].computeLocalInverseKinematics(np.array([legs[2].x_local_goal, legs[2].y_local_goal, legs[2].z_local_goal]))
        legBR = legs[3].computeLocalInverseKinematics(np.array([legs[3].x_local_goal, legs[3].y_local_goal, legs[3].z_local_goal]))

        if self.i >= 3:
            leg0_y0_stand = self.y0+self.stepSize
            leg0_p0_stand = np.array([leg0_y0_stand, current_z])
            leg0_p1_stand = np.array([self.y0, current_z])
            leg0_y, leg0_z = self.bezierCurveCLinear(leg0_p0_stand, leg0_p1_stand, self.t0)
            legFR = legs[0].computeLocalInverseKinematics(np.array([legs[0].x_local_goal, -leg0_y, leg0_z]))

        if self.i >= 2:
            leg1_y0_stand = self.y1+self.stepSize
            leg1_p0_stand = np.array([leg1_y0_stand, current_z])
            leg1_p1_stand = np.array([self.y1, current_z])
            leg1_y, leg1_z = self.bezierCurveCLinear(leg1_p0_stand, leg1_p1_stand, self.t1)
            legBL = legs[1].computeLocalInverseKinematics(np.array([legs[1].x_local_goal, leg1_y, leg1_z]))

        if self.i >= 1:
            leg2_y0_stand = self.y2+self.stepSize
            leg2_p0_stand = np.array([leg2_y0_stand, current_z])
            leg2_p1_stand = np.array([self.y2, current_z])
            leg2_y, leg2_z = self.bezierCurveCLinear(leg2_p0_stand, leg2_p1_stand, self.t2)
            legFL = legs[2].computeLocalInverseKinematics(np.array([legs[2].x_local_goal, leg2_y, leg2_z]))

        if self.i > 4:
            leg3_y0_stand = self.y3+self.stepSize
            leg3_p0_stand = np.array([leg3_y0_stand, current_z])
            leg3_p1_stand = np.array([self.y3, current_z])
            leg3_y, leg3_z = self.bezierCurveCLinear(leg3_p0_stand, leg3_p1_stand, self.t3)
            legBR = legs[3].computeLocalInverseKinematics(np.array([legs[3].x_local_goal, -leg3_y, leg3_z]))


        if self.phase > 4:
            self.phase = 1

        if self.phase == 1:
            self.leg_swing['front_left'] = True
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = True
            legs[3].swing = False
            #xif self.t == self.t_res:
            #    #self.y2 = legs[2].y_local_goal #-0.1
                #print(self.y2)

            p0 = np.array([self.y2, current_z])
            p1 = np.array([self.y2, current_z+self.height])
            p2 = np.array([self.y2+self.stepSize+(0.5*self.stepSize), current_z+self.height])
            p3 = np.array([self.y2+self.stepSize, current_z])
            yz = self.bezierCurveCubic(p0,p1,p2,p3,self.t)

            legFL = legs[2].computeLocalInverseKinematics(np.array([legs[2].x_local_goal, yz[0], yz[1]]))

            if self.t == 1.0:
                self.t = 0.0
                self.t2 = 0.0
                self.phase += 1
                self.leg_swing['front_left'] = False
                legs[2].swing = False
            return self.leg_swing, legFR, legBL, legFL, legBR

        elif self.phase == 2:
            self.leg_swing['back_left'] = True
            legs[0].swing = False
            legs[1].swing = True
            legs[2].swing = False
            legs[3].swing = False

            #if self.t == self.t_res:
            #    self.y1 = legs[1].y_local_goal #-0.1
            p0 = np.array([self.y1, current_z])
            p1 = np.array([self.y1, current_z+self.height])
            p2 = np.array([self.y1+self.stepSize+(0.5*self.stepSize), current_z+self.height])
            p3 = np.array([self.y1+self.stepSize, current_z])
            yz = self.bezierCurveCubic(p0,p1,p2,p3,self.t)

            legBL = legs[1].computeLocalInverseKinematics(np.array([legs[1].x_local_goal, yz[0], yz[1]]))

            if self.t == 1.0:
                self.t = 0.0
                self.t1 = 0.0
                self.phase += 1
                self.leg_swing['back_left'] = False
                legs[1].swing = False
                self.i += 1
            return self.leg_swing, legFR, legBL, legFL, legBR

        elif self.phase == 3:
            self.leg_swing['front_right'] = True
            legs[0].swing = True
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = False

            #if self.t == self.t_res:
            #    self.y0 = -legs[0].y_local_goal #-0.1
            p0 = np.array([self.y0, current_z])
            p1 = np.array([self.y0, current_z+self.height])
            p2 = np.array([self.y0+self.stepSize+(0.5*self.stepSize), current_z+self.height])
            p3 = np.array([self.y0+self.stepSize, current_z])
            yz = self.bezierCurveCubic(p0,p1,p2,p3,self.t)

            legFR = legs[0].computeLocalInverseKinematics(np.array([legs[0].x_local_goal, -yz[0], yz[1]]))

            if self.t == 1.0:
                self.t = 0.0
                self.t0 = 0.0
                self.phase += 1
                self.leg_swing['front_right'] = False
                legs[0].swing = False
                self.i += 1
            return self.leg_swing, legFR, legBL, legFL, legBR

        elif self.phase == 4:
            self.leg_swing['back_right'] = True
            legs[0].swing = False
            legs[1].swing = False
            legs[2].swing = False
            legs[3].swing = True
            #if self.t == self.t_res:
            #    self.y3 = -legs[3].y_local_goal #-0.1
            p0 = np.array([self.y3, current_z])
            p1 = np.array([self.y3, current_z+self.height])
            p2 = np.array([self.y3+self.stepSize+(0.5*self.stepSize), current_z+self.height])
            p3 = np.array([self.y3+self.stepSize, current_z])
            yz = self.bezierCurveCubic(p0,p1,p2,p3,self.t)

            legBR = legs[3].computeLocalInverseKinematics(np.array([legs[3].x_local_goal, -yz[0], yz[1]]))

            if self.t == 1.0:
                self.t = 0.0
                self.t3 = 0.0
                self.phase += 1
                self.leg_swing['back_right'] = False
                legs[3].swing = False
                self.i += 1
            return self.leg_swing, legFR, legBL, legFL, legBR

    def getTResolution(self):
        """ get T parameter based on translational velocity """
        self.t_res = self.transl_vel*4 / self.frequency

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

        return position
