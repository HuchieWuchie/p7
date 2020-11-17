from teensy_serial import serial_teensy
import numpy as np
import threading
import time
import os

class leg_connection:
    def __init__(self,name_serial_port='/dev/tty.usbmodem58778701',use_velocity=False, using_current=False,using_error_message=False):
        ###### inits the serial connection here. Specify serial name here
        self.serial=serial_teensy(serial_name=name_serial_port,use_velocity=use_velocity)
        self.using_current=using_current
        self.using_error_messages=using_error_message
        #assigning the offsets
        #new
        self.offset_raw_leg_1=[2585,1497,875+1022] #leg 1   Front Right zero 1 is 2625
        self.offset_raw_leg_2=[2091,1517,1481+1022] #leg 2  Front Left   zero is 1500

        self.offset_raw_leg_3=[1100,1011,2945+1022]  #leg 3  Back Right    3150
        self.offset_raw_leg_4=[1050,2536,2443+1022]  #leg 4  Back left     1000

        #old..
        #self.offset_raw_leg_1=[2585,1507-511,900+1022] #leg 1   Front Right zero 1 is 2625
        #self.offset_raw_leg_2=[2091-100,1520-511,1400+1022] #leg 2  Front Left   zero is 1500
        #self.offset_raw_leg_3=[1100,997-511,2957+1022]  #leg 3  Back Right    3150
        #self.offset_raw_leg_4=[1050-100,2535-511,2488+1022]  #leg 4  Back left     1000

        self.offset_raw=np.append(np.append(np.append(self.offset_raw_leg_1,self.offset_raw_leg_2),self.offset_raw_leg_3),self.offset_raw_leg_4)

        self.negative_threshold=4092

        self.number_of_motors=12
        self.current_threshold = 500  #####mA
        #time.sleep(5)
        self.moveThread = threading.Thread(target=self.read).start()
    def set_offset(self, offset_raw_value):
        self.offset_raw = offset_raw_value
        self.offset_radians = self.serial.convert_rawvalue_2_radians(offset_raw_value)

    def get_nparray_double(self, list):
        return np.asarray(list, dtype='double')

    ######## function to make the robot go to joint positions in radians.#######
    ######## Note that limits are set on the teensy. If the joint positions are out of reach,
    ######## the motors will not be able to execute the commands.
    ######## checked and working at 10 Hz. would probably work on 20 Hz as well.
    ######## The legs are numerated as follows:
    ######## First 3 values are for the front right, the following 3 is front left, following 3 is back right, back left
    def execute_joint_position_radians(self,array_of_joint_positions):
        joint_pos_raw=self.serial.convert_radians_2_rawvalue(array_of_joint_positions)
        joint_pos_execute_raw=joint_pos_raw+self.offset_raw
        #print(joint_pos_execute_raw)
        self.serial.send_positions_array_ints(np.asarray(joint_pos_execute_raw,dtype=np.int64))
    ######## Update this function to check whether it is within the limits or not.

    ####similar to the function above, however this contains velocity updates as well.
    def execute_joint_pos_radians_with_vel(self,array_of_joint_positions,velocities):
        joint_pos_raw=self.serial.convert_radians_2_rawvalue(array_of_joint_positions)
        joint_pos_execute_raw=joint_pos_raw+self.offset_raw
        full_array=np.append(joint_pos_execute_raw,velocities)
        self.serial.send_pos_vel_array_ints(np.asarray(full_array,dtype=np.int64))

    def execute_joint_velocities(self,array_of_velocities):
        array_ints_send = self.send_for_negative_values(array_of_velocities)
        self.serial.send_positions_array_ints(np.asarray(array_ints_send,dtype=np.int64))

    ###### Funtion to read status of the legs. It returns an array os joint angles and an array foot sensors
    ###### The array of foot sensors are numerated as:
    ###### 0 is front right, 1 is front left, 2 is back right, 3 is back left.
    def read_leg_status(self):
        status=self.serial.read_status()
        #print(status)
        temp=np.asarray(status[0:12],dtype=np.int_)
        pos_rad=self.serial.convert_rawvalue_2_radians(temp-self.offset_raw)

        foot_sensors=np.asarray(status[12:16],dtype=np.int_)
        imu_sensor=np.asarray(status)[16:17]
        return pos_rad,foot_sensors,imu_sensor

    def read_leg_status_w_current(self):
        status=self.serial.read_status()
        pos_rad=self.serial.convert_rawvalue_2_radians(np.asarray(status)[0:self.number_of_motors]-self.offset_raw)
        present_current=self.check_for_negative_values(np.asarray(status)[self.number_of_motors:self.number_of_motors*2])
        present_current=present_current*3.36
        foot_sensors=np.asarray(status)[self.number_of_motors*2:self.number_of_motors*2+4]
        return pos_rad,foot_sensors,present_current

    def read_leg_status_w_error_message(self):
        status=self.serial.read_status()
        pos_rad=self.serial.convert_rawvalue_2_radians(np.asarray(status)[0:self.number_of_motors]-self.offset_raw)
        present_current=self.check_for_negative_values(np.asarray(status)[self.number_of_motors:self.number_of_motors*2])
        present_current=present_current*3.36
        present_error_message=np.asarray(status)[self.number_of_motors*2:self.number_of_motors*3]
        foot_sensors=np.asarray(status)[self.number_of_motors*3:self.number_of_motors*3+4]
        return pos_rad,foot_sensors,present_current,present_error_message

    def read_leg_status_velocity(self):
        status=self.serial.read_status()
        pos_raw=self.serial.convert_rawvalue_2_radians(np.asarray(status)[0:self.number_of_motors]-self.offset_raw)
        vel_raw=self.check_for_negative_values(np.asarray(status)[self.number_of_motors:self.number_of_motors*2])
        foot_sensors=np.asarray(status)[self.number_of_motors*2:self.number_of_motors*2+4]
        return pos_raw,vel_raw,foot_sensors

    def check_for_negative_values(self,array):
        for i in range(0,len(array)):
            if array[i]>self.negative_threshold:
                array[i]=-(self.negative_threshold*2-array[i])
        return array

    def send_for_negative_values(self, array):
        for i in range(0,len(array)):
            if array[i]<0:
                array[i]=(self.negative_threshold*2-array[i])
        return array

    def read(self):
        self.init_current_watch()
        while True:
            if self.using_current:
                self.pos_rad, self.foot, self.current = self.read_leg_status_w_current()
                #function to log the current
                self.watching_current_usage(self.current)
            if self.using_current:
                self.pos_rad, self.foot, self.current,self.error_message = self.read_leg_status_w_error_message()
                self.watching_current_usage(self.current)
                self.watching_error_message(self.error_message)
            else:
                self.pos_rad, self.foot, self.imu = self.read_leg_status()
            #print(self.pos_rad)
            time.sleep(0.001)

    def get_status(self):
        if self.using_current:
            return self.pos_rad, self.foot, self.current
        else:
            return self.pos_rad,self.foot,self.imu


    ####### Previosly used function. Still working, but not supported B-)
    def get_current_angles_raw_value(self):
        pos = self.leg.getEEPosition()
        angles = self.leg.computeLocalInverseKinematics(pos)
        return self.serial.convert_radians_2_rawvalue(angles)

    def get_current_angles_degree(self):
        pos = self.leg.getEEPosition()
        angles = self.leg.computeLocalInverseKinematics(pos)
        return self.serial.convert_radians_2_rawvalue(self.serial.convert_rawvalue_2_degree(angles))

    def go_cartesian_position_from_kinematics(self, position):
        joints_radians = self.leg.computeLocalInverseKinematics(position)
        joint_raw_value = self.serial.convert_radians_2_rawvalue(joints_radians)
        output_raw_value = self.serial.convert_radians_2_rawvalue(self.offset_radians) + joint_raw_value
        self.serial.send_positions_array_ints(output_raw_value)

    def init_error_watch(self):
        if os.path.isfile("error_watch.txt"):
            os.remove("error_watch.txt")
        f = open("error_watch.txt", "a")
        f.write("If the file is empty, no error has been received on any of the motors!\n")
        f.close()


    def watching_error_message(self,error_message):
        for i in range(0,12):
            error_message



    def init_current_watch(self):
        self.start_time=time.time()
        if os.path.isfile("current_watch.txt"):
            os.remove("current_watch.txt")
        f = open("current_watch.txt", "a")
        f.write("If the file is empty, the current threshold of \t")
        f.write(str(self.current_threshold))
        f.write("\t has not been exceeded on any of the motors!\n")
        f.close()

    def watching_current_usage(self,current):
        for i in range(0,12):
            if current[i]>self.current_threshold or current[i]<-1*self.current_threshold:
                f = open("current_watch.txt", "a")
                ###current usage for motor
                ###write it to file
                f.write("current exceeded for motor\t")
                f.write(str(i))
                f.write("\tWith a Value of\t")
                f.write(str(int(current[i])))
                f.write("\tafter\t")
                f.write(str(int(time.time()-self.start_time)))
                f.write("\tseconds")
                f.write("\n")
                f.close()

