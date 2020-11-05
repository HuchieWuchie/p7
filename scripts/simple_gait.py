from quadruped import Quadruped

start_z=-0.3
start_y=0
import time

quadruped = Quadruped(simulation=False)


##### Generally this should go to the position that maximizes the stability margin.
def go_to_stable_position(targetY):
    quadruped.update_positions_leg_from_robot()
    resolution=20
    for i in range(0,resolution):
        step=targetY/resolution
        quadruped.setDelta_Y(step)
        #small delay
        time.sleep(0.05)

def step_leg_forward(id,step_length,step_height):
    pos_cart=quadruped.get_EE_positions()[id]
    traject=pos_cart
    if id==0 or id==3:
        traject[1]=traject[1]+step_length/2
        traject[2]=traject[2]+step_height
        quadruped.setLegYZ(id,traject[1],traject[2])
        time.sleep(0.05)
        traject[1]=traject[1]+step_length/2
        traject[2]=traject[2]-step_height
        quadruped.setLegYZ(id,traject[1],traject[2])
    else:
        traject[1]=traject[1]-step_length/2
        traject[2]=traject[2]+step_height
        quadruped.setLegYZ(id,traject[1],traject[2])
        time.sleep(0.05)
        traject[1]=traject[1]-step_length/2
        traject[2]=traject[2]-step_height
        quadruped.setLegYZ(id,traject[1],traject[2])

def main():
    ##### Make it go backwards to most stable position.
    time.sleep(2)
    go_to_stable_position(0.1)
    time.sleep(1)
    ##### move the front right leg
    step_leg_forward(0,0.1,0.1)
    time.sleep(1)
    ##### move the front left leg
    step_leg_forward(2,0.1,0.1)
    time.sleep(1)
    ##### make it go forwards to most stable position.
    go_to_stable_position(-0.2)
    time.sleep(1)
    #### move back right leg
    step_leg_forward(3,0.1,0.1)
    time.sleep(1)
    ##### move the back left leg
    step_leg_forward(1,0.1,0.1)

def main2():
    time.sleep(2)
    for i in range(0,10):
        step=-0.1/10
        quadruped.setDelta_Y(step)
        time.sleep(0.05)

if __name__ == "__main__":
    time.sleep(1)
    quadruped.setY(start_y)
    time.sleep(1)
    quadruped.setZ(start_z)
    time.sleep(5)
    quadruped.setLegY(1,-0.1)
    time.sleep(1)
    quadruped.setLegY(2,0.1)

