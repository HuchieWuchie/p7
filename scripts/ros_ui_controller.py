import tkinter as tk
import numpy as np
from math import pi

from quadruped import Quadruped
from transform import *
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int8, Int16, Bool
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

def function(id):
    global i_limit
    global joint_state, joint_name_lst, jst, joint_sliders

    global command_frequency_pub, step_height_pub, step_size_pub, cycle_time_pub
    global switch_gait_pub, gait_pub, translational_velocity_pub, angular_velocity_pub
    global set_x_pub, set_y_pub, set_z_pub, delta_x_pub, delta_y_pub, set_leg_z_pub
    global set_leg_x_pub, gait_active_pub

    if i_limit > 20:

        val = joint_sliders[id].get()

        if id == 0:
            set_x_pub.publish(val)

        elif id == 1:
            set_y_pub.publish(val)

        elif id == 2:
            set_z_pub.publish(val)

        elif id == 6:
            translational_velocity_pub.publish(val)

        elif id == 7:
            angular_velocity_pub.publish(val)

        elif id == 8:
            step_size_pub.publish(val)

        elif id == 9:
            step_height_pub.publish(val)

        elif id == 10:
            cycle_time_pub.publish(val)

        elif id == 11:
            command_frequency_pub.publish(val)

        elif id == 12:
            set_leg_x_pub.publish(data=[val, val, val, val])
            
        """
        elif id == 3:
            joint_state = quadruped.setRoll(val)

        elif id == 4:
            joint_state = quadruped.setPitch(val)

        elif id == 5:
            joint_state = quadruped.setYaw(val)
        """
    i_limit += 1

def activateGait():
    global active
    pass

    if active == False:
        gait_active_pub.publish(active)
        active = True
    else:
        gait_active_pub.publish(active)
        active = False


#def command_frequency_pub()

active = 0
i_limit = 0

command_frequency_pub = rospy.Publisher('/quadrupedUI/setCommandFrequency', Int16, queue_size=0)
step_height_pub = rospy.Publisher('/quadrupedUI/setStepHeight', Float32, queue_size=0)
step_size_pub = rospy.Publisher('/quadrupedUI/setStepSize', Float32, queue_size=0)
cycle_time_pub = rospy.Publisher('/quadrupedUI/setCycleTime', Float32, queue_size=0)
switch_gait_pub = rospy.Publisher('/quadrupedUI/switchGait', Int8, queue_size=0)
gait_pub = rospy.Publisher('/quadrupedUI/setGait', Int8, queue_size=0)
translational_velocity_pub = rospy.Publisher('/quadrupedUI/setTranslationalVelocity', Float32, queue_size=0)
angular_velocity_pub = rospy.Publisher('/quadrupedUI/setAngularVelocity', Float32, queue_size=0)
set_x_pub = rospy.Publisher('/quadrupedUI/setX', Float32, queue_size=0)
set_y_pub = rospy.Publisher('/quadrupedUI/setY', Float32, queue_size=0)
set_z_pub = rospy.Publisher('/quadrupedUI/setZ', Float32, queue_size=0)
delta_x_pub = rospy.Publisher('/quadrupedUI/deltaX', Float32, queue_size=0)
delta_y_pub = rospy.Publisher('/quadrupedUI/deltaY', Float32, queue_size=0)
set_leg_z_pub = rospy.Publisher('/quadrupedUI/setLegZ', Float32MultiArray, queue_size=0)
set_leg_x_pub = rospy.Publisher('/quadrupedUI/setLegX', Float32MultiArray, queue_size=0)
gait_active_pub = rospy.Publisher('/quadrupedUI/gaitActive', Bool, queue_size=0)

rospy.init_node('quadrupedUI_node', disable_signals=True)

root = tk.Tk()
root.title("Quadruped GUI")

labels = []
labels.append(tk.Label(root, text="X: "))
labels.append(tk.Label(root, text="Y: "))
labels.append(tk.Label(root, text="Z: "))
labels.append(tk.Label(root, text="Roll: "))
labels.append(tk.Label(root, text="Pitch: "))
labels.append(tk.Label(root, text="Yaw: "))
labels.append(tk.Label(root, text="Translational Velocity: "))
labels.append(tk.Label(root, text="Angular Velocity: "))
labels.append(tk.Label(root, text="Step size: "))
labels.append(tk.Label(root, text="Step height: "))
labels.append(tk.Label(root, text="Cycle time: "))
labels.append(tk.Label(root, text="Command frequency: "))
labels.append(tk.Label(root, text="Set all leg x: "))
labels.append(tk.Label(root, text="Activate gait: "))


for i in range(len(labels)):
    labels[i].grid(row=i, column=1)

joint_sliders = []


joint_sliders.append(tk.Scale((root), from_=-0.05, to=0.05, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(0)))
#joint_sliders[-1].set(quadruped.COM[0])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.6, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(1)))
#joint_sliders[-1].set(quadruped.COM[1])
joint_sliders.append(tk.Scale((root), from_=-0.6, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(2)))
#joint_sliders[-1].set(quadruped.COM[2])
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(3)))
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(4)))
joint_sliders.append(tk.Scale((root), from_=-0.4, to=0.4, resolution=0.001, length=400, orient='horizontal', command=lambda x:function(5)))
joint_sliders.append(tk.Scale((root), from_=0, to=1.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(6)))
joint_sliders.append(tk.Scale((root), from_=0, to=1.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(7)))
joint_sliders.append(tk.Scale((root), from_=0.0, to=0.3, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(8)))
joint_sliders.append(tk.Scale((root), from_=0.1, to=0.4, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(9)))
joint_sliders.append(tk.Scale((root), from_=1.0, to=50.0, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(10)))
joint_sliders.append(tk.Scale((root), from_=10, to=200, resolution=1, length=400, orient='horizontal', command=lambda x:function(11)))
joint_sliders.append(tk.Scale((root), from_=-0.20, to=0.2, resolution=0.01, length=400, orient='horizontal', command=lambda x:function(12)))


joint_sliders.append(tk.Button((root), text="Gait activate", command=activateGait))

for i in range(len(joint_sliders)):
    joint_sliders[i].grid(row=i, column=2)

exitButton = tk.Button(root, text="Exit Program", command=root.quit)
exitButton.grid(row=30, column = 2)

root.mainloop()
