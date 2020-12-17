import tkinter as tk
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
from math import pi

from quadruped import Quadruped
from transform import *
import time
import math

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

def function(id):
    global i_limit
    global joint_state, joint_name_lst, jst, joint_sliders

    if i_limit > 20:

        val = joint_sliders[id].get()

        if id == 0:
            quadruped.setX(val)

        elif id == 1:
            quadruped.setY(val)

        elif id == 2:
            quadruped.setZ(val)

        elif id == 3:
            joint_state = quadruped.setRoll(val)

        elif id == 4:
            joint_state = quadruped.setPitch(val)

        elif id == 5:
            joint_state = quadruped.setYaw(val)

        elif id == 6:
            quadruped.setWidth(val)

        elif id == 7:
            quadruped.gait.setStepSize(val)

        elif id == 8:
            quadruped.gait.td1 = val

        elif id == 9:
            quadruped.gait.td2 = val

        elif id == 10:
            quadruped.gait.xDelta = val

        elif id == 11:
            quadruped.gait.zDelta = -val
        elif id == 12:
            quadruped.gait.setCycleTime(val)

        elif id == 13:
            quadruped.gait.setStepHeight(val)


        elif id == 14:
            quadruped.setAngularVelocity(val)


        elif id == 15:
            quadruped.setLegY(1, val)

        elif id == 16:
            quadruped.setLegY(3, val)

        elif id == 17:
            quadruped.setLegY(4, val)

        elif id == 18:
            quadruped.setLegY(2, val)

        elif id == 19:

            """
            z = -0.35
            x = 0.065
            r = math.sqrt(pow(z,2) + pow(x,2))
            leg = 1
            quadruped.forkFunc(leg, val, r)
            """
            quadruped.rollLeg(val)

    i_limit += 1

def changeGait():
    global rdy
    if rdy == 0:
        quadruped.readyToWalk = True
        rdy = 1
    else:
        quadruped.readyToWalk = False
        rdy = 0

def resetIMU():
    quadruped.resetIMU()

def pidActivate():
    quadruped.angle_pid = True

rdy = 0
sim = False
i_limit = 0

if sim == True:
    rospy.init_node('kinematics_node', disable_signals=True)

quadruped = Quadruped(simulation=sim)
print(quadruped.feet_sensor_readings)



root = tk.Tk()
root.title("Quadruped GUI")

labels = []
labels.append(tk.Label(root, text="X: "))
labels.append(tk.Label(root, text="Y: "))
labels.append(tk.Label(root, text="Z: "))
labels.append(tk.Label(root, text="Roll: "))
labels.append(tk.Label(root, text="Pitch: "))
labels.append(tk.Label(root, text="Yaw: "))
labels.append(tk.Label(root, text="Stance width: "))
labels.append(tk.Label(root, text="Step Size: "))
labels.append(tk.Label(root, text="td1 FL/BR: "))
labels.append(tk.Label(root, text="td2 FR/BL: "))
labels.append(tk.Label(root, text="xDelta: "))
labels.append(tk.Label(root, text="zDelta: "))
labels.append(tk.Label(root, text="Phase time: "))
labels.append(tk.Label(root, text="Step height: "))
labels.append(tk.Label(root, text="Angular velocity: "))
labels.append(tk.Label(root, text="Leg front right y: "))
labels.append(tk.Label(root, text="Leg front left y: "))
labels.append(tk.Label(root, text="Leg back right y: "))
labels.append(tk.Label(root, text="Leg back left y: "))
labels.append(tk.Label(root, text="Fork: "))
labels.append(tk.Label(root, text="Activate gait: "))
labels.append(tk.Label(root, text="Reset IMU: "))
labels.append(tk.Label(root, text="Activate PID angle control: "))


for i in range(len(labels)):
    labels[i].grid(row=i, column=1)

joint_sliders = []


joint_sliders.append(tk.Scale((root), from_=-0.07, to=0.07, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(0)))
#joint_sliders[-1].set(quadruped.COM[0])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.6, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(1)))
joint_sliders[-1].set(quadruped.COM[1])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(2)))
joint_sliders[-1].set(quadruped.COM[2])
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(3)))
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(4)))
joint_sliders.append(tk.Scale((root), from_=-0.9, to=0.9, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(5)))
joint_sliders.append(tk.Scale((root), from_=-0.1, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(6)))
joint_sliders.append(tk.Scale((root), from_=0.0, to=1.5, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(7)))
joint_sliders.append(tk.Scale((root), from_=0.0, to=1.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(8)))
joint_sliders[-1].set(0.375)
joint_sliders.append(tk.Scale((root), from_=-0.0, to=1.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(9)))
joint_sliders[-1].set(0.875)
joint_sliders.append(tk.Scale((root), from_=-0.1, to=0.1, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(10)))
joint_sliders[-1].set(0.0)
joint_sliders.append(tk.Scale((root), from_=-0.0, to=0.1, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(11)))
joint_sliders[-1].set(0.0)
joint_sliders.append(tk.Scale((root), from_=0.1, to=100.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(12)))
joint_sliders.append(tk.Scale((root), from_=0.0, to=1, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(13)))
joint_sliders.append(tk.Scale((root), from_=-1.0, to=1.0, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(14)))
joint_sliders[-1].set(0.00)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(15)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(16)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(17)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(18)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.5, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(19)))
joint_sliders[-1].set(-0.)

joint_sliders.append(tk.Button((root), text="Gait activate", command=changeGait))
joint_sliders.append(tk.Button((root), text="Reset IMU", command=resetIMU))
joint_sliders.append(tk.Button((root), text="Activate PID angle control", command=pidActivate))

for i in range(len(joint_sliders)):
    joint_sliders[i].grid(row=i, column=2)

exitButton = tk.Button(root, text="Exit Program", command=root.quit)
exitButton.grid(row=30, column = 2)

root.mainloop()
