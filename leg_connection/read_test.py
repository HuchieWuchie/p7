from leg_connection import leg_connection
import time
import numpy as np
#serial_object= serial_teensy()
serial_port_name='/dev/tty.usbmodem58778701'
leg_con=leg_connection(name_serial_port='/dev/tty.usbmodem58778701',using_current=False)
np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

frequency=80
while True:
    time.sleep(1/frequency)
    pos_rad,_,imu= leg_con.get_status()

    print(imu)
    #leg_con.execute_joint_position_radians(pos_raw)
    i=0


