from leg_connection import leg_connection
import time


###### CONTAINS EXAMPLES OF THE 3 DIFFERENT TYPES OF INTERFACE CURRENTLY
###### AVIALABLE FOR THE MOTORS.
###### IT NEEDS TO CORRESPOND TO THE TEENSY PROGRAM CURRENTLY ON THE MOTORS.

def use_velocity_interface():
    ##### Example usage of functions to use with Serial_velocity:
    ##### This is for pure velocity interface
    ##### Description in the classes
    leg_con=leg_connection(name_serial_port='/dev/tty.usbmodem58778701',use_velocity=True)
    velocity_commands=[0,0,10,0,0,10,0,0,10,0,0,10]
    leg_con.execute_joint_velocities(velocity_commands)
    position_radians,current_velocity,foot_sensors=leg_con.read_leg_status_velocity()

def use_position_and_velocity_interface():
    ###### Example usages of the functions with the Serial_example_w_velocity
    ###### This interface works on position, but you can update the velocity interface
    leg_con=leg_connection(name_serial_port='/dev/tty.usbmodem58778701')
    radians_array=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    velocities=[0,0,10,0,0,10,0,0,10,0,0,10]
    ##also sends velocities. ##takes in an array of 4 velocities.
    ##if left as zero, keeps using current velocity.
    leg_con.execute_joint_pos_radians_with_vel(radians_array,velocities)
    ##### returns radians_array, foot_status
    radians_array,foot_status=leg_con.read_leg_status()

def use_position_interface():
    leg_con=leg_connection(name_serial_port='/dev/tty.usbmodem58778701')
    radians_array=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    ##### Use these functions if you are only using position.
    ##### Should support both np arrays and lists. Description in class.
    leg_con.execute_joint_position_radians(radians_array)

    ##### returns radians array and foot sensor status. Description in class.
    radians_array,foot_status=leg_con.read_leg_status()

if __name__ == '__main__':
    #use_velocity_interface()

    #use_position_and_velocity_interface()

    use_position_interface()