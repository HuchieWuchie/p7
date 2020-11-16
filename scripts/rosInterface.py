import numpy as np
from math import pi

from quadruped import Quadruped
from transform import *
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int8, Int16, Bool
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

def command_frequency_subscriber_callback(value):
    global quadruped
    quadruped.setCommandFrequency(value.data)

def step_height_subscriber_callback(value):
    global quadruped
    quadruped.setStepHeight(value.data)

def step_size_subscriber_callback(value):
    global quadruped
    quadruped.setStepSize(value.data)

def cycle_time_subscriber_callback(value):
    global quadruped
    quadruped.setCycleTime(value.data)


def switch_gait_subscriber_callback(value):
    global quadruped
    quadruped.switchGait()

def gait_subscriber_callback(value):
    global quadruped
    quadruped.setGait(value.data)

def translational_velocity_subscriber_callback(value):
    global quadruped
    quadruped.setTranslationalVelocity(value.data)


def angular_velocity_subscriber_callback(value):
    global quadruped
    quadruped.setAngularVelocity(value.data)


def set_x_subscriber_callback(value):
    global quadruped
    quadruped.setX(value.data)


def set_y_subscriber_callback(value):
    global quadruped
    quadruped.setY(value.data)


def set_z_subscriber_callback(value):
    global quadruped
    quadruped.setZ(value.data)


def delta_x_subscriber_callback(value):
    global quadruped
    quadruped.deltaX(value.data)


def delta_y_subscriber_callback(value):
    global quadruped
    quadruped.deltaY(value.data)


def leg_x_subscriber_callback(value):
    global quadruped
    for i in range(4):
        if value.data[i] != 0:
            quadruped.setLegX(i+1, value.data[i])

def leg_z_subscriber_callback(value):
    global quadruped
    for i in range(4):
        if value.data[i] != 0:
            quadruped.setLegZ(i+1, value.data[i])

def gait_active_subscriber_callback(value):
    global quadruped
    quadruped.readyToWalk = value.data


rospy.init_node('quadruped', disable_signals=True)
quadruped = Quadruped(simulation = True)

command_frequency_subscriber = rospy.Subscriber('/quadrupedUI/setCommandFrequency',Int16,
                                                       command_frequency_subscriber_callback)

step_height_subscriber = rospy.Subscriber('/quadrupedUI/setStepHeight',Float32,
                                                       step_height_subscriber_callback)

step_size_subscriber = rospy.Subscriber('/quadrupedUI/setStepSize',Float32,
                                                       step_size_subscriber_callback)

cycle_time_subscriber = rospy.Subscriber('/quadrupedUI/setCycleTime',Float32,
                                                       cycle_time_subscriber_callback)

switch_gait_subscriber = rospy.Subscriber('/quadrupedUI/switchGait',Int8,
                                                       switch_gait_subscriber_callback)

gait_subscriber = rospy.Subscriber('/quadrupedUI/setGait',Int8,
                                                       gait_subscriber_callback)

translational_velocity_subscriber = rospy.Subscriber('/quadrupedUI/setTranslationalVelocity',Float32,
                                                       translational_velocity_subscriber_callback)

angular_velocity_subscriber = rospy.Subscriber('/quadrupedUI/setAngularVelocity',Float32,
                                                       angular_velocity_subscriber_callback)

set_x_subscriber = rospy.Subscriber('/quadrupedUI/setX',Float32,
                                                       set_x_subscriber_callback)

set_y_subscriber = rospy.Subscriber('/quadrupedUI/setY',Float32,
                                                       set_y_subscriber_callback)

set_z_subscriber = rospy.Subscriber('/quadrupedUI/setZ',Float32,
                                                       set_z_subscriber_callback)

delta_x_subscriber = rospy.Subscriber('/quadrupedUI/deltaX',Float32,
                                                       delta_x_subscriber_callback)

delta_y_subscriber = rospy.Subscriber('/quadrupedUI/deltaY',Float32,
                                                       delta_y_subscriber_callback)

set_leg_z_subscriber = rospy.Subscriber('/quadrupedUI/setLegZ', Float32MultiArray,
                                                       leg_z_subscriber_callback)

set_leg_x_subscriber = rospy.Subscriber('/quadrupedUI/setLegX', Float32MultiArray,
                                                       leg_x_subscriber_callback)

set_gait_active = rospy.Subscriber('/quadrupedUI/gaitActive', Bool,
                                                       gait_active_subscriber_callback)


rospy.spin()
