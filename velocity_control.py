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
offset=3
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

        elif id == 3:
            quadruped.gait.setStepSize(val)

        elif id == 4:
            quadruped.gait.setCycleTime(val)

        elif id == 5:
            setForwardVelocity(val)

        elif id == 6:
            setRotationalVelocity(val)
        elif id==7:
            setVelocityX(val)

    i_limit += 1

def changeGait():
    global rdy

    if rdy == 0:
        quadruped.readyToWalk = True
        rdy = 1
    else:
        quadruped.readyToWalk = False
        rdy = 0

rdy = 0
sim = True
i_limit = 0

if sim == True:
    rospy.init_node('kinematics_node', disable_signals=True)

quadruped = Quadruped(simulation=sim)

def setVelocityX(vel_x):
    cycle_time=3.3
    max_step_size=0.04
    quadruped.gait.setSideways(True)
    if abs(vel_x)<0.0001:
        quadruped.gait.setStepX(0.0)
        if abs(quadruped.gait.getStepSize())< 0.0001:
            quadruped.readyToWalk=False
            quadruped.setZ(-0.38)
        return 0
    walking=quadruped.readyToWalk
    step_size=cycle_time*vel_x
    step_size=min(max_step_size,step_size)
    quadruped.setY(0.03)
    stepy=quadruped.gait.getStepSize()
    quadruped.gait.setCycleTime(cycle_time)
    quadruped.gait.setStepX(step_size)
    quadruped.readyToWalk=True
    if abs(quadruped.gait.getStepRotX())<abs(step_size):
        quadruped.gait.setSideways(True)
        quadruped.gait.setRotation(False)
        quadruped.gait.setCycleTime(cycle_time)
        quadruped.gait.setStepX(step_size)
        quadruped.readyToWalk=True
    else:
        quadruped.gait.setSideways(False)
        quadruped.gait.setRotation(True)

#max vel= 0.014
def setRotationalVelocity(rotVel):
    rotVel=rotVel*0.21
    cycle_time=3.3
    max_step_size=0.04
    if abs(rotVel)<0.0001:
        quadruped.gait.setStepRotX(0.0)
        if abs(quadruped.gait.getStepSize())< 0.0001:
            quadruped.readyToWalk=False
            quadruped.setZ(-0.38)
        return 0
    walking=quadruped.readyToWalk
    step_size=cycle_time*rotVel
    stepy=quadruped.gait.getStepSize()
    step_size=min(max_step_size,step_size)
    if abs(quadruped.gait.getStepX())<abs(step_size):
        quadruped.gait.setSideways(False)
        quadruped.gait.setRotation(True)
        quadruped.setY(0.03)
        quadruped.gait.setCycleTime(cycle_time)
        quadruped.gait.setStepRotX(step_size)
        quadruped.readyToWalk=True
    else:
        quadruped.gait.setSideways(True)
        quadruped.gait.setRotation(False)



def setForwardVelocity(vel):
    cycle_time=3.3
    max_step_size=0.04
    if abs(vel)<0.0001:
        quadruped.gait.setStepSize(0.0)
        if abs(quadruped.gait.getStepX()) < 0.0001:
            quadruped.readyToWalk=False
            quadruped.setZ(-0.38)
        return 0
    walking=quadruped.readyToWalk
    if walking:
        quadruped.readyToWalk = False
    quadruped.setZ(-0.38)
        #time.sleep(0.1)
    step_size=cycle_time*vel
    step_size=min(max_step_size,step_size)
    quadruped.setY(0.03)
    quadruped.gait.setCycleTime(cycle_time)
    quadruped.gait.setStepSize(step_size)
    quadruped.readyToWalk=True


root = tk.Tk()
root.title("Quadruped GUI")

labels = []
labels.append(tk.Label(root, text="X: "))
labels.append(tk.Label(root, text="Y: "))
labels.append(tk.Label(root, text="Z: "))

labels.append(tk.Label(root, text="Step Size: "))
labels.append(tk.Label(root, text="Phase time: "))
labels.append(tk.Label(root, text="velocity"))

labels.append(tk.Label(root, text="angular velocity"))
labels.append(tk.Label(root, text="velocity x"))


labels.append(tk.Label(root, text="Activate gait: "))


for i in range(len(labels)):
    labels[i].grid(row=i, column=1)

joint_sliders = []


joint_sliders.append(tk.Scale((root), from_=-0.05, to=0.05, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(0)))
#joint_sliders[-1].set(quadruped.COM[0])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.6, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(1)))
joint_sliders[-1].set(quadruped.COM[1])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(2)))
joint_sliders[-1].set(quadruped.COM[2])
joint_sliders.append(tk.Scale((root), from_=0.0, to=1.5, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(3)))
joint_sliders.append(tk.Scale((root), from_=0.1, to=100.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(4)))
joint_sliders.append(tk.Scale((root), from_=-0.01, to=0.01, resolution=0.002, length=400, orient='horizontal', command=lambda x:function(5)))
joint_sliders.append(tk.Scale((root), from_=-0.0577, to=0.0577, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(6)))
joint_sliders.append(tk.Scale((root), from_=-0.01, to=0.01, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(7)))


joint_sliders.append(tk.Button((root), text="Gait activate", command=changeGait))

for i in range(len(joint_sliders)):
    joint_sliders[i].grid(row=i, column=2)

exitButton = tk.Button(root, text="Exit Program", command=root.quit)
exitButton.grid(row=30, column = 2)

root.mainloop()



###change from rotate_to_forward.
###change from forward_to_rotate

### setting vx

    #diff=abs(quadruped.gait.getStepsizeLeftY()-quadruped.gait.getStepsizeLeftY())
    #-8 and 8 0.4
    #8 and 8 0.3
    #-12 and 8 0.12
    #20 and 0 0.4
    #-20 and -8 0.3
    #-8 and -8 0.00
    #compare to other step size and check y.
    #or transition phase
    #if diff>
    #print(quadruped.getY())



### setting rotz


##setting the step size
#quadruped.gait.setStepSize(val)

#quadruped.gait.setStepSizeX(val)
