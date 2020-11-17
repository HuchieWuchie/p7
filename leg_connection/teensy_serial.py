import serial
import numpy as np
class serial_teensy:
    def __init__(self,number_of_motors=12,serial_name='/dev/tty.usbmodem58778701',use_velocity=False):
        self.ser = serial.Serial(serial_name, 1200000, timeout=0.001)
        #Note that teensy always works at this baudrate.
        self.num_ints = number_of_motors
        self.read_velocites=use_velocity
        self.available=True

        self.length_reading=64
    def convert_rawvalue_2_degree(self,rawvalue):
        return np.asarray(rawvalue)*0.088

    def convert_degree_2_rawvalue(self,degree):
        ints=np.asarray(degree) / 0.088
        return np.asarray(ints,dtype="int")

    def convert_radians_2_rawvalue(self,radians):
        return self.convert_degree_2_rawvalue(np.asarray(radians)*180/np.pi)

    def convert_rawvalue_2_radians(self,rawvalue):
        return self.convert_rawvalue_2_degree(rawvalue)*np.pi/180

    def convert_radians_2_degree(self,radians):
        return np.asarray(radians,dtype='double') * 180 / np.pi

    def convert_degree_2_radians(self,degree):
        return np.asarray(degree,dtype='double') * np.pi / 180

    def serial_available(self):
        return self.ser.readable()

    def serial_reading_line(self):
        #reading
        ####### The 4 added bytes is for the foot sensors.
        if self.read_velocites:
            readings=self.ser.read((self.num_ints*2)*4+4)
        else:
            readings=self.ser.readline()
        #Checking if it is not an "empty" reading from teensy
        stri=readings.decode('ascii')
        if str(readings) != "b\'\'":
            stri=stri[:-2]
            return True,stri
        else:
            return False,False

    def serial_write(self,output):
        #output=','.join(output)
        #ser.write(str(output).encode('utf-8'))
        #print(output)
        #for i in range(0,len(output)):
            #output[i]=bindigits(output[i],32)
        #binary_array=''.join(chr(b) for b in output)
        #print(binary_array)
        #print(output)
        if self.available:
            self.available=False
            self.ser.write(str(output).encode('utf-8'))
            self.ser.flush()
            self.available=True


    def convert_string_to_ints(self,string_of_numbers):
        length=self.length_reading
        #print(length)
        stri=str(string_of_numbers)
        array=[]
        prev_val=0
        for i in range(1, length + 1):
            #if i > length - 4:
            #    print(int(stri[i - 1]))
            #    array.append(int(stri[i - 1]))
            if i%4==0:
                array.append(int(stri[prev_val:i]))
                prev_val=i

        array.append(stri[self.length_reading:len(string_of_numbers)])

        return array




##convert to negative values only relevant for velocity if needed.
    def convert_to_send_str(self,array_of_numbers):
        length_array=len(array_of_numbers)
        string_to_send=""
        if(length_array == self.num_ints):
            for i in range(0,length_array):
                if array_of_numbers[i]==0:
                    string_to_send+="0000"
                else:
                    tempstring=str(array_of_numbers[i])
                    if(len(tempstring)==1):
                        tempstring ="000"+tempstring
                        string_to_send += tempstring
                    elif(len(tempstring)==2):
                        tempstring = "00" + tempstring
                        string_to_send += tempstring
                    elif(len(tempstring)==3):
                        tempstring = "0" + tempstring
                        string_to_send += tempstring
                    else:
                        string_to_send += tempstring
            return string_to_send
        else:
            print("array contatins too little numbers")

    def convert_to_send_str_w_vel(self,array_of_numbers):
        length_array=len(array_of_numbers)
        string_to_send=""

        #one command to update the velocity on each of the leg.
        #should possibly be 8.
        num_velocity_commands=12

        if(length_array == (self.num_ints+num_velocity_commands)):
            for i in range(0,length_array):
                if array_of_numbers[i]==0:
                    string_to_send+="0000"
                else:
                    tempstring=str(array_of_numbers[i])
                    if(len(tempstring)==1):
                        tempstring ="000"+tempstring
                        string_to_send += tempstring
                    elif(len(tempstring)==2):
                        tempstring = "00" + tempstring
                        string_to_send += tempstring
                    elif(len(tempstring)==3):
                        tempstring = "0" + tempstring
                        string_to_send += tempstring
                    else:
                        string_to_send += tempstring
            return string_to_send
        else:
            print("array contatins inaccurate amount of ints")


    def send_positions_array_ints(self,array):
        string_to_send=self.convert_to_send_str(array)
        #print(string_to_send)
        self.serial_write(string_to_send)

    #### updated function to update the velocity on the motors as well.
    #### the string should contatin 4 values more.
    def send_pos_vel_array_ints(self,array):
        string_to_send=self.convert_to_send_str_w_vel(array)
        #print(string_to_send)
        self.serial_write(string_to_send)

    #Reads the current positions and returns an array of ints corresponding to the number of motors
    def read_status(self):
        #self.issue_reading_command()
        new_value=False
        value=0
        while (not new_value):
            new_value, value = self.serial_reading_line()
            #print(new_value)
            #print(value)
        new_array = self.convert_string_to_ints(value)
        #print(new_array)
        return new_array

