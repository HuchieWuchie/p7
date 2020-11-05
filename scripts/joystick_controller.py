import pygame as pg
import numpy as np
from quadruped import Quadruped
from pygame.locals import *
import rospy
import time

pg.init()
joysticks = []
clock = pg.time.Clock()
keepPlaying = True
joystick = pg.joystick.init()

# for al the connected joysticks
for i in range(0, pg.joystick.get_count()):
    joysticks.append(pg.joystick.Joystick(i))
    joysticks[-1].init()
    print ("Detected joystick "),joysticks[-1].get_name(),"'"

rospy.init_node('joystick_controller_node', disable_signals=True)
quadruped = Quadruped()

readyToWalk = False
yCOMdesired = 0.0
clock.tick(100)
t_period = 1 / 10
while True:
    ts = int(round(time.time() * 1000))

    if readyToWalk == False:
        quadruped.setTranslationalVelocity(0)
        quadruped.setAngularVelocity(0)

    # activate or deactivate gait based on LT
    if joysticks[-1].get_axis(2) >= 0.2:

        if readyToWalk == False:
            quadruped.setY(yCOMdesired)
            if quadruped.gaitStyle == 0:
                if yCOMdesired >= 0.1:
                    yCOMdesired = 0.1
                    quadruped.setY(yCOMdesired)
                    readyToWalk = True
                    quadruped.readyToWalk = True

            elif quadruped.gaitStyle == 1:
                if yCOMdesired >= 0.0:
                    yCOMdesired = 0.0
                    quadruped.setY(yCOMdesired)
                    readyToWalk = True
                    quadruped.readyToWalk = True
            yCOMdesired += 0.01

    else:
        if readyToWalk == True:
            quadruped.setY(yCOMdesired)

            if yCOMdesired <= 0.0:
                yCOMdesired = 0.0
                quadruped.setY(yCOMdesired)
                readyToWalk = False
                quadruped.readyToWalk = False

            yCOMdesired -= 0.01

        else:
            quadruped.setZ(quadruped.z_local_goal)

    if joysticks[-1].get_button(0) == True:
        if quadruped.transl_vel < 0.15:
            quadruped.setTranslationalVelocity(quadruped.transl_vel+0.01)


    elif joysticks[-1].get_button(2) == True:
        if quadruped.transl_vel >= 0.0:
            quadruped.setTranslationalVelocity(quadruped.transl_vel-0.01)

    if joysticks[-1].get_button(1) == True:
        if quadruped.readyToWalk == False:
            quadruped.switchGait()

    if joysticks[-1].get_axis(0)>=0.2 or joysticks[-1].get_axis(0)<=-0.2:
        quadruped.setAngularVelocity(0.75*joysticks[-1].get_axis(0))
    else:
        quadruped.setAngularVelocity(0)
    # Change Z based on Y and A
    if joysticks[-1].get_axis(5) >= 0.5:
        quadruped.setZ(quadruped.z_local_goal+0.01)
    elif joysticks[-1].get_button(5) == True:
        quadruped.setZ(quadruped.z_local_goal-0.01)
    for event in pg.event.get():
        if event.type == JOYAXISMOTION:
            pass#print(event)


        elif event.type == JOYBUTTONUP or event.type == JOYBUTTONDOWN:
            pass #print(event.button)
        #print(event.type)

        #    print ("A Has Been Pressed")

    te = int(round(time.time() * 1000))
    time.sleep((t_period-((te-ts))*0.001))
