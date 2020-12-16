import rospy
#from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tkinter as tk


import numpy as np
from math import pi

from quadruped import Quadruped
from transform import *


#/gazebo/model_states This has the states of the model in gazebo.
#Taken directly from gazebo, real state estimation is probably needed



class robotController:

    def __init__(self):
        rdy = 0
        sim = True
        #rospy.init_node('kinematics_node', disable_signals=True)
        self.quadruped = Quadruped(simulation=sim)
        rospy.init_node('state_updater')
        rospy.Subscriber("/quadruped/velocity",Twist,self.callback_velocity)

        self.quadruped.setCycleTime(15)
        #create init process

        #rospy.Subscriber("/gazebo/model_states",ModelStates,self.model_callback)
        self.prevMoveX=0
        self.prevMoveY=0

    def model_callback(self,model_state):
        vel_x=model_state.pose[1].position.x
        vel_y=model_state.pose[1].position.y

        print(vel_x)
        print(vel_x)

    def callback_velocity(self,twist_msg):
        vely=twist_msg.linear.y
        velrot=twist_msg.linear.z
        velx=twist_msg.linear.x
        self.quadruped.setZ(-0.36)
        print("vel Y",vely)
        print("velrot ",velrot)
        print("vel X",velx)
        self.setForwardVelocity(vely)
        self.setRotationalVelocity(velrot)
        self.setVelocityX(velx)
    ### NOT USED!!!!
    def setVelocityLeftY(self,vel):
        current_cycle_time=self.quadruped.gait.getCycleTime()

        if abs(vel)<0.001:
            self.quadruped.gait.setStepsizeLeftY(0.0)
            self.quadruped.readyToWalk=False
            return 0
        walking=self.quadruped.readyToWalk
        if walking:
            self.quadruped.readyToWalk = False
            #time.sleep(0.1)

        step_size=current_cycle_time*vel
        if step_size>0:
            step_size=min(0.20,step_size)
        else:
            step_size=max(-0.20,step_size)

        #quadruped.gait.setCycleTime(cycle_time)
        self.quadruped.gait.setStepsizeLeftY(step_size)
        self.moveX=(self.quadruped.gait.getStepsizeLeftY()+self.quadruped.gait.getStepsizeRightY())/2
        dif=abs(self.moveX)-abs(self.prevMoveX)

        if abs(self.prevMoveX-self.moveX)>0.01:
            y=self.moveX*0.5
            self.quadruped.setY(y)
            self.prevMoveX=self.moveX
        self.quadruped.readyToWalk=True
    ### NOT USED!!!
    def setVelocityRightY(self,vel):
        current_cycle_time=self.quadruped.gait.getCycleTime()

        if abs(vel)<0.001:
            self.quadruped.gait.setStepsizeRightY(0.0)
            self.quadruped.readyToWalk=False
            return 0
        walking=self.quadruped.readyToWalk
        if walking:
            self.quadruped.readyToWalk = False
            #time.sleep(0.1)
        step_size=current_cycle_time*vel
        if step_size>0:
            step_size=min(0.20,step_size)
        else:
            step_size=max(-0.20,step_size)

        ## MAx velocity is 0.02

        self.quadruped.gait.setStepsizeRightY(step_size)

        self.moveY=(self.quadruped.gait.getStepsizeLeftY()+self.quadruped.gait.getStepsizeRightY())/2
        if abs(self.prevMoveY-self.moveY)>0.01:
            y=self.moveY*0.5
            self.quadruped.setY(y)
            self.prevMoveY=self.moveY

        self.quadruped.readyToWalk=True
        #max vel= 0.014
    #max vel= 0.014
    ###
    ### NOT USED!!!
    def setRotationalVelocity1(self,rotVel):
        rotVel=rotVel*0.21
        cycle_time=3.5
        max_step_size=0.05
        if abs(rotVel)<0.0001:
            self.quadruped.gait.setStepX(0.0)
            if abs(self.quadruped.gait.getStepSize())< 0.0001:
                self.quadruped.readyToWalk=False
                self.quadruped.setZ(-0.38)
            return 0
        walking=self.quadruped.readyToWalk
        step_size=cycle_time*rotVel
        if step_size>0:
            step_size=min(max_step_size,step_size)
        else:
            step_size=max(-max_step_size,step_size)
        print("velrot ",step_size)
        self.quadruped.setY(0.06)
        self.quadruped.gait.setCycleTime(cycle_time)
        self.quadruped.gait.setStepX(step_size)
        self.quadruped.readyToWalk=True
    ### used to change velocity x
    def setVelocityX(self,vel_x):
        cycle_time=3.3
        max_step_size=0.04
        self.quadruped.gait.setSideways(True)
        if abs(vel_x)<0.0001:
            self.quadruped.gait.setStepX(0.0)
            if abs(self.quadruped.gait.getStepSize())< 0.0001:
                self.quadruped.readyToWalk=False
                self.quadruped.setZ(-0.38)
            return 0
        walking=self.quadruped.readyToWalk
        step_size=cycle_time*vel_x
        if step_size>0:
            step_size=min(max_step_size,step_size)
        else:
            step_size=max(-max_step_size,step_size)
        self.quadruped.setY(0.04)
        stepy=self.quadruped.gait.getStepSize()
        self.quadruped.gait.setCycleTime(cycle_time)
        self.quadruped.gait.setStepX(step_size)
        self.quadruped.readyToWalk=True
        #print("stepRot",self.quadruped.gait.getStepRotX())
        #print("stepsize x",step_size)
        if abs(self.quadruped.gait.getStepRotX())<abs(step_size):
            self.quadruped.gait.setSideways(True)
            self.quadruped.gait.setRotation(False)
            self.quadruped.setY(0.03)
            self.quadruped.gait.setCycleTime(cycle_time)
            self.quadruped.gait.setStepX(step_size)
            self.quadruped.readyToWalk=True
        else:
            self.quadruped.gait.setSideways(False)
            self.quadruped.gait.setRotation(True)

    ###Used to change rotational velocity
    def setRotationalVelocity(self,rotVel):
        rotVel=rotVel*0.21
        cycle_time=3.3
        max_step_size=0.04
        if abs(rotVel)<0.0001:
            self.quadruped.gait.setStepRotX(0.0)
            if abs(self.quadruped.gait.getStepSize())< 0.0001:
                self.quadruped.readyToWalk=False
                self.quadruped.setZ(-0.38)
            return 0
        walking=self.quadruped.readyToWalk
        step_size=cycle_time*rotVel
        step_size=min(max_step_size,step_size)
        if step_size>0:
            step_size=min(max_step_size,step_size)
        else:
            step_size=max(-max_step_size,step_size)
        stepy=self.quadruped.gait.getStepSize()
        if abs(self.quadruped.gait.getStepX())<abs(step_size):
            self.quadruped.gait.setSideways(False)
            self.quadruped.gait.setRotation(True)
            self.quadruped.setY(0.03)
            self.quadruped.gait.setCycleTime(cycle_time)
            self.quadruped.gait.setStepRotX(step_size)
            self.quadruped.readyToWalk=True
        else:
            self.quadruped.gait.setSideways(True)
            self.quadruped.gait.setRotation(False)

    ### set forward velocity Y
    def setForwardVelocity(self,vel):
        cycle_time=3.3
        max_step_size=0.04
        if abs(vel)<0.0001:
            self.quadruped.gait.setStepSize(0.0)
            if abs(self.quadruped.gait.getStepX()) < 0.0001:
                self.quadruped.readyToWalk=False
                self.quadruped.setZ(-0.38)
            return 0
        walking=self.quadruped.readyToWalk
        if walking:
            self.quadruped.readyToWalk = False
            #time.sleep(0.1)
        step_size=cycle_time*vel
        if step_size>0:
            step_size=min(max_step_size,step_size)
        else:
            step_size=max(-max_step_size,step_size)
        self.quadruped.setY(0.03)
        self.quadruped.gait.setCycleTime(cycle_time)
        self.quadruped.gait.setStepSize(step_size)
        self.quadruped.readyToWalk=True



if __name__ == "__main__":

    rcon=robotController()
    root = tk.Tk()
    root.title("Quadruped GUI")
    exitButton = tk.Button(root, text="Exit Program", command=root.quit)
    exitButton.grid(row=30, column = 2)
    root.mainloop()
