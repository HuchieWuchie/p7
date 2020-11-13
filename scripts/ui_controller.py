import tkinter as tk
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
from math import pi

from quadruped import Quadruped
from transform import *

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

def function(id):
    global i_limit
    global joint_state, joint_name_lst, jst, joint_sliders

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
            quadruped.gait.setCycleTime(val)

        elif id == 13:
            quadruped.gait.setStepHeight(val)

        elif id == 14:
            for i in range(len(quadruped.legs)):
                quadruped.setLegX(leg = i+1, x = val)


        elif id == 15:
            quadruped.setLegY(1, val)

        elif id == 16:
            quadruped.setLegY(3, val)

        elif id == 17:
            quadruped.setLegY(4, val)

        elif id == 18:
            quadruped.setLegY(2, val)
        """
        elif id == 3:
            joint_state = quadruped.setRoll(val)

        elif id == 4:
            joint_state = quadruped.setPitch(val)

        elif id == 5:
            joint_state = quadruped.setYaw(val)
        """
    i_limit += 1

def changeGait():
    global rdy

    if rdy == 0:
        quadruped.readyToWalk = True
        rdy = 1
    else:
        quadruped.readyToWalk = False
        rdy = 0

def incrWidth():
    quadruped.deltaX(0.01)

def decrWidth():
    quadruped.deltaX(-0.01)

rdy = 0
sim = False
i_limit = 0

if sim == True:
    rospy.init_node('kinematics_node', disable_signals=True)

quadruped = Quadruped(simulation=sim)



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
labels.append(tk.Label(root, text="Step height: "))
labels.append(tk.Label(root, text="Set all leg x: "))
labels.append(tk.Label(root, text="Leg front right y: "))
labels.append(tk.Label(root, text="Leg front left y: "))
labels.append(tk.Label(root, text="Leg back right y: "))
labels.append(tk.Label(root, text="Leg back left y: "))
labels.append(tk.Label(root, text="Activate gait: "))
labels.append(tk.Label(root, text="Increase x: "))
labels.append(tk.Label(root, text="Decrease x: "))


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
joint_sliders.append(tk.Scale((root), from_=0.1, to=100.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(12)))
joint_sliders.append(tk.Scale((root), from_=0.1, to=1, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(13)))
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(14)))
joint_sliders.append(tk.Scale((root), from_=-0.2, to=.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(15)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(16)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(17)))
joint_sliders[-1].set(-0.)
joint_sliders.append(tk.Scale((root), from_=-0.2, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(18)))
joint_sliders[-1].set(-0.)

joint_sliders.append(tk.Button((root), text="Gait activate", command=changeGait))
joint_sliders.append(tk.Button((root), text="Increase leg width", command=incrWidth))
joint_sliders.append(tk.Button((root), text="Decrease leg width", command=decrWidth))

for i in range(len(joint_sliders)):
    joint_sliders[i].grid(row=i, column=2)

exitButton = tk.Button(root, text="Exit Program", command=root.quit)
exitButton.grid(row=30, column = 2)

root.mainloop()
