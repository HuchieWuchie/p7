from teensy_serial import serial_teensy
import numpy as np


class leg_connection:
    def __init__(self,name_serial_port='/dev/tty.usbmodem58778701',use_velocity=False):
        ###### inits the serial connection here. Specify serial name here
        self.serial=serial_teensy(serial_name=name_serial_port,use_velocity=use_velocity)

        #assigning the offsets
        self.offset_raw_leg_1=[3675,1507,900+1022] #946 #leg 1   Front Right zero 1 is 2625
        self.offset_raw_leg_2=[1500,1520,1400+1022] #1488 #leg 2  Front Left   zero is 1500
        self.offset_raw_leg_3=[1100,997,2957+1022]  #leg 3  Back Right    3150
        self.offset_raw_leg_4=[1050,2535,2488+1022]  #leg 4  Back left     1000
        self.offset_raw=np.append(np.append(np.append(self.offset_raw_leg_1,self.offset_raw_leg_2),self.offset_raw_leg_3),self.offset_raw_leg_4)

        self.negative_threshold=4092

        self.number_of_motors=12

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
        print(joint_pos_execute_raw)
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
        pos_rad=self.serial.convert_rawvalue_2_radians(np.asarray(status)[0:12]-self.offset_raw)

        foot_sensors=np.asarray(status)[12:16]
        return pos_rad,foot_sensors

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
