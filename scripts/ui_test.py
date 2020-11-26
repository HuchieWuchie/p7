import tkinter as tk
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
from math import pi
import math

from quadruped import Quadruped
from transform import *

import threading
import time

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

class Circle:
    """docstring circle."""

    def __init__(self, quadruped):
        self.setCycleTime(10)
        self.setRadius(7.5)
        self.setLeg(3)
        self.t = 0
        self.setFrequency(75) # 70 works
        self.getTResolution()
        self.circle = False
        self.square = False
        self.quad = quadruped

        self.executionThread = threading.Thread(target=self.execute).start()

    def setFrequency(self, val):
        self.frequency = int(max(0,val))
        print(self.frequency)

    def setCycleTime(self, val):
        """ Cycle time in seconds """
        self.cycleTime = max(0,val)
        print(self.cycleTime)

    def setRadius(self, val):
        """ Radius in meters """
        self.radius = max(0, (val*0.01))

    def setLeg(self, leg):
        """ Set leg that performs circle motion """
        """ 0 = FR, 1 = BL, 2 = FL, 3 = BR """
        self.leg = int(round(max(0,min(3,leg)),0))

    def getTResolution(self):
        """ Computes the t parameter resolution for each time step """
        self.tRes = (1 / self.frequency) / self.cycleTime

    def getCirclePath(self):
        """ A paramterized path of the circle, returns y, z """
        #y = -self.quad.y_local_goal + self.radius * math.cos(2*math.pi*self.t)
        #z = self.quad.z_local_goal + self.radius * math.sin(2*math.pi*self.t)

        y = self.radius * math.cos(2*math.pi*self.t)
        z = self.quad.z_local_goal + self.radius * math.sin(2*math.pi*self.t)
        return y, z

    def getSquarePath(self):
        """ A paramterized path of the square, returns y, z """
        if self.t < 0.25 and self.t >= 0.0:
            y = (self.radius * 8 * self.t) - self.radius
            z = self.quad.z_local_goal - self.radius

        elif self.t < 0.5 and self.t >= 0.25:
            y = self.radius
            z = self.quad.z_local_goal - self.radius + ((self.t-0.25) * 8 * self.radius)

        elif self.t < 0.75 and self.t >= 0.5:
            y = -self.radius * 8 * (self.t-0.5) + self.radius
            z = self.quad.z_local_goal + self.radius

        elif self.t <= 1.0 and self.t >= 0.75:
            y = -self.radius
            z = self.quad.z_local_goal + self.radius - ((self.t-0.75) * 8 * self.radius)

        return y, z

    def execute(self):

        while True:
            tstart = int(round(time.time() * 1000))
            if self.circle == True:
                self.getTResolution()
                if self.t >= 1.0:
                    self.t = 0
                else:
                    self.t += self.tRes
                    if self.t > 1.0:
                        self.t = 1.0

                legAng = np.zeros((4,3))

                for i in range(len(self.quad.legs)):
                    x = self.quad.legs[i].x_local_goal
                    y = self.quad.legs[i].y_local_goal
                    z = self.quad.legs[i].z_local_goal

                    if self.leg == i:
                        y, z = self.getCirclePath()
                        x = 0.065
                        if self.quad.legs[i].side == "right":
                            y = -y

                        print(round(self.t,2), "\t", round(x,3), round(y,3), round(z,3))

                    legAng[i] = self.quad.legs[i].computeLocalInverseKinematics(np.array([x,y,z]))

                joints = np.array([legAng[0],legAng[1],legAng[2],legAng[3]])
                tend = int(round(time.time() * 1000))
                t_sleep = ((1/self.frequency)-((tend-tstart))*0.001)

                self.quad.sendJointCommand(joints)
                time.sleep(max(0,t_sleep))

            if self.square == True:
                self.getTResolution()
                if self.t >= 1.0:
                    self.t = 0
                else:
                    self.t += self.tRes
                    if self.t > 1.0:
                        self.t = 1.0

                legAng = np.zeros((4,3))

                for i in range(len(self.quad.legs)):
                    x = self.quad.legs[i].x_local_goal
                    y = self.quad.legs[i].y_local_goal
                    z = self.quad.legs[i].z_local_goal

                    if self.leg == i:
                        y, z = self.getSquarePath()
                        x = 0.065
                        if self.quad.legs[i].side == "right":
                            y = -y

                        print(round(self.t,2), "\t", round(x,3), round(y,3), round(z,3))

                    legAng[i] = self.quad.legs[i].computeLocalInverseKinematics(np.array([x,y,z]))

                joints = np.array([legAng[0],legAng[1],legAng[2],legAng[3]])
                tend = int(round(time.time() * 1000))
                t_sleep = ((1/self.frequency)-((tend-tstart))*0.001)

                self.quad.sendJointCommand(joints)
                time.sleep(max(0,t_sleep))


def function(id):
    global i_limit
    global joint_state, joint_name_lst, jst, joint_sliders
    global circle

    if i_limit > 20:

        val = joint_sliders[id].get()

        if id == 0:
            quadruped.deltaX(val)

        elif id == 1:
            quadruped.setY(val)

        elif id == 2:
            quadruped.setZ(val)

        elif id == 6:
            quadruped.setAngularVelocity(val)

        elif id == 7:
            quadruped.gait.setStepSize(val)

        elif id == 8:
            quadruped.setLegZ(1, val)

        elif id == 9:
            quadruped.setLegZ(3, val)

        elif id == 10:
            quadruped.setLegZ(4, val)

        elif id == 11:
            quadruped.setLegZ(2, val)
        elif id == 12:
            # circle time
            circle.setCycleTime(val)

        elif id == 13:
            # circle radius
            circle.setRadius(val)

        elif id == 14:
            quadruped.setAllLegX(x=val)


        elif id == 15:
            quadruped.setLegY(1, val)

        elif id == 16:
            quadruped.setLegY(3, val)

        elif id == 17:
            quadruped.setLegY(4, val)

        elif id == 18:
            quadruped.setLegY(2, val)

        elif id == 19:
            q = []
            for i in range(len(quadruped.legs)):
                #q.append(quadruped.legs[i].computeLocalInverseKinematics(np.array([quadruped.legs[i].x_local_goal, quadruped.legs[i].y_local_goal, quadruped.legs[i].z_local_goal])))
                q.append(quadruped.legs[i].joints)
            q[3][1] = val
            q = np.array(q)
            quadruped.sendJointCommand(q)

        elif id == 20:
            q = []
            for i in range(len(quadruped.legs)):
                q.append(quadruped.legs[i].joints)
            q[3][2] = val
            q = np.array(q)
            quadruped.sendJointCommand(q)
        """
        elif id == 3:
            joint_state = quadruped.setRoll(val)

        elif id == 4:
            joint_state = quadruped.setPitch(val)

        elif id == 5:
            joint_state = quadruped.setYaw(val)
        """
    i_limit += 1

def startCircle():
    global rdy, circle

    if rdy == 0:
        circle.circle = True
        rdy = 1
    else:
        circle.circle = False
        rdy = 0

def startSquare():
    global rdy, circle

    if rdy == 0:
        circle.square = True
        rdy = 1
    else:
        circle.square = False
        rdy = 0

rdy = 0
sim = False
i_limit = 0

if sim == True:
    rospy.init_node('kinematics_node', disable_signals=True)

quadruped = Quadruped(simulation=sim)
circle = Circle(quadruped)



root = tk.Tk()
root.title("Quadruped GUI")

labels = []
labels.append(tk.Label(root, text="X: "))
labels.append(tk.Label(root, text="Y: "))
labels.append(tk.Label(root, text="Z: "))
labels.append(tk.Label(root, text="Roll: "))
labels.append(tk.Label(root, text="Pitch: "))
labels.append(tk.Label(root, text="Yaw: "))
labels.append(tk.Label(root, text="Angular Velocity: "))
labels.append(tk.Label(root, text="Step Size: "))
labels.append(tk.Label(root, text="Leg front right z: "))
labels.append(tk.Label(root, text="Leg front left z: "))
labels.append(tk.Label(root, text="Leg back right z: "))
labels.append(tk.Label(root, text="Leg back left z: "))
labels.append(tk.Label(root, text="Phase time: "))
labels.append(tk.Label(root, text="Circle Radius: "))
labels.append(tk.Label(root, text="Set all leg x: "))
labels.append(tk.Label(root, text="Leg front right y: "))
labels.append(tk.Label(root, text="Leg front left y: "))
labels.append(tk.Label(root, text="Leg back right y: "))
labels.append(tk.Label(root, text="Leg back left y: "))
labels.append(tk.Label(root, text="Set Back Right [1] angle: "))
labels.append(tk.Label(root, text="Set Back Right [2] angle: "))
labels.append(tk.Label(root, text="Activate Circle: "))
labels.append(tk.Label(root, text="Activate Square: "))



for i in range(len(labels)):
    labels[i].grid(row=i, column=1)

joint_sliders = []


joint_sliders.append(tk.Scale((root), from_=-0.05, to=0.05, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(0)))
#joint_sliders[-1].set(quadruped.COM[0])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.6, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(1)))
joint_sliders[-1].set(quadruped.COM[1])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(2)))
joint_sliders[-1].set(quadruped.COM[2])
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(3)))
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(4)))
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(5)))
joint_sliders.append(tk.Scale((root), from_=0, to=1.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(6)))
joint_sliders.append(tk.Scale((root), from_=0.0, to=1.5, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(7)))
joint_sliders.append(tk.Scale((root), from_=-0.52, to=0.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(8)))
joint_sliders[-1].set(-0.25)
joint_sliders.append(tk.Scale((root), from_=-0.52, to=0.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(9)))
joint_sliders[-1].set(-0.25)
joint_sliders.append(tk.Scale((root), from_=-0.52, to=0.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(10)))
joint_sliders[-1].set(-0.25)
joint_sliders.append(tk.Scale((root), from_=-0.52, to=0.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(11)))
joint_sliders[-1].set(-0.25)
joint_sliders.append(tk.Scale((root), from_=0.1, to=10.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(12)))
joint_sliders.append(tk.Scale((root), from_=0.0, to=1, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(13)))
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(14)))
joint_sliders[-1].set(0.065)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(15)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(16)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(17)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(18)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=0.0, to=math.pi, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(19)))
joint_sliders[-1].set(-0.)

joint_sliders.append(tk.Scale((root), from_=-math.pi, to=math.pi, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(20)))
joint_sliders[-1].set(-0.)

joint_sliders.append(tk.Button((root), text="Circle activate", command=startCircle))
joint_sliders.append(tk.Button((root), text="Square activate", command=startSquare))

for i in range(len(joint_sliders)):
    joint_sliders[i].grid(row=i, column=2)

exitButton = tk.Button(root, text="Exit Program", command=root.quit)
exitButton.grid(row=30, column = 2)

root.mainloop()
