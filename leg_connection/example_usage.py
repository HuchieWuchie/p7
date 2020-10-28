from leg_connection import leg_connection

if __name__ == '__main__':

    #example usage. The serialport is handled inside the class
    leg_con=leg_connection(name_serial_port='/dev/tty.usbmodem58778701')

    radians_array=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    velocities=[0,0,10,0,0,10,0,0,10,0,0,10]

    ###### use this function if you are working with the velocity
    ##also sends velocities. ##takes in an array of 4 velocities.
    ##if left as zero, keeps using current velocity.
    leg_con.execute_joint_pos_radians_with_vel(radians_array,velocities)


    ##### Use this function if you are only using position.
    ##### Should support both np arrays and lists. Description in class.
    #leg_con.execute_joint_position_radians(radians_array)



    ##### returns radians array and foot sensor status. Description in class.
    #radians_array,foot_status=leg_con.read_leg_status()
    #print(radians_array)
    #-1.40841089