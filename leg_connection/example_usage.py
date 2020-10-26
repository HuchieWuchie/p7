from leg_connection import leg_connection

if __name__ == '__main__':

    #example usage. The serialport is handled inside the class
    leg_con=leg_connection(name_serial_port='/dev/tty.usbmodem58778701')

    radians_array=[0]

    ##### Should support both np arrays and lists. Description in class.
    leg_con.execute_joint_position_radians(radians_array)


    ##### returns radians array and foot sensor status. Description in class.
    radians_array,foot_status=leg_con.read_leg_status()
