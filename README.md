# p7

## Requirements

> - ROS Noetic (possibly earlier)
> - Ubuntu 18+ (20 for Noetic)

## How to run

Remember to source the setup bash file.
Two terminals.

1: roslaunch quadruped quadruped_controll.launch

2: python3 velocity_controller.py
To put the quadruped into a starting position

3: python3 state_updater.py

4. run the MPCros.m script in matlab

## To do list:

- Change quadruped model to real model using stl files
- Get a stable gait up and running
