from leg_connection import leg_connection
import numpy as np
import copy
import math
import cmath
import transform
import threading
import time
from gait import Gait
from quadleg import QuadLeg
from quadbody import QuadBody
import csv

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

width = 0.28
length = 0.7
height = 0.1

tc_left_front = transform.transformFull(0,0,math.pi, width*0.5, (length/2)*0.9, height*0.5)

leg = QuadLeg(joints=np.array([-0.0, -1.0644, 2.0833]), bodyPosition=tc_left_front, name="right_front", side="right", frontBack="front", simulation=False)

print(leg.computeLocalInverseKinematics(np.array([0.065, 0.0, -0.3])))
print(leg.computeLocalForwardKinematics(leg.joints))
print(leg.computeLocalForwardKinematics(np.array([0.0, 0.9845, -1.9306])))
