#include <vector>
#include <Dynamixel2Arduino.h>  

//Dynamixel setup
#define DXL_SERIAL   Serial2    //serial port tx/rx number used.
const uint8_t DXL_DIR_PIN = 35; // DYNAMIXEL Shield DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;
uint32_t BAUDRATE = 4000000;
using namespace std;

//Specify number of motors and ids here.
#define number_of_motors 12 //12
char buf[number_of_motors*4+1];
const int32_t DXL_ID[number_of_motors]={15,12,8,3,9,14,6,11,2,1,17,5};


//Setup for the two front legs 
//Setup fo Leg number 2 front right left.
//Specify the dxl_ids here in chosen order
//joint number 1 is id 3 joint number 2 is 9 //Joint number 3 defined below
//const int32_t DXL_ID[number_of_motors]={15,12,8,3,9,14};
//It is important to specify the limits...
//new limits
int32_t minlimit[number_of_motors]={1850,950,400, 2100,1100,850,800,570,2500,750,2100,2000};
int32_t maxlimit[number_of_motors]={2400,2100,1400, 2800,2100,1975,1300,1700,3500,1300,3200,3000};

//old limits
//int32_t minlimit[number_of_motors]={3200,1200,600,1300,1250,1080,800,800,2500,750,2260,2000};
//int32_t maxlimit[number_of_motors]={3800,2520,1400,1800,2650,1975,1300,2050,3500,1300,3600,3000};

/*
//Setup fo Leg number 1 front right leg.
//Specify the dxl_ids here in chosen order
const int32_t DXL_ID[number_of_motors]={15,12,8};
//It is important to specify the limits...
int32_t minlimit[number_of_motors]={1450,2500,1050};
int32_t maxlimit[number_of_motors]={2000,4000,1900};
*/

//can be initialized with initial positions and be used as reference positions
//and then be updated from serial. 
vector<vector<int32_t>> motors_targets(number_of_motors, vector<int32_t>(0));



//setup for the foot sensors
#define number_of_pressure 4
//order them correspondenly
int8_t pressurePin[number_of_pressure]={23,22,21,20};
//init the pressure value
int pressureValue[number_of_pressure]={0,0,0,0};

int serial_timeout_milis=1;

/*
 * Setup for the IMU stuff.
 */

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
 
 
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
 
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
 
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
 
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
 
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
 
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };





void setup() {
  //Setting up dynamixels
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  //Can be used to ping
  //ping_all_motors(),
  
  //Function to setup pins for feet sensors
  //setup_input_sensors_feet(feet_input_pins); 
  setup_imu();


  //setting up bulk read//
  setup_bulk_read();
  
  //set the actuator limits
  set_actuator_limits(minlimit,maxlimit);
  setup_bulk_write();

  //Setting the profile
  int16_t velocity_profile=80;
  int16_t acc_profile=60;
  for(int i=0;i<number_of_motors;i++){
    //change_reverse_mode(false,9);
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
    dxl.torqueOn(DXL_ID[i]);
    if(DXL_ID[i]==8,DXL_ID[i]==14,DXL_ID[i]==2,DXL_ID[i]==5){
       //15 3 6 1
      //sets the velocity lower for the shoulder
      //dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
      //dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], 10);//112
      //dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
      //dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], 10);//112
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], velocity_profile);//112
    }else if(DXL_ID[i]==11,DXL_ID[i]==17){
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], velocity_profile);//112
    
    }
    else{
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], 20);//108
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], 40);//112
    }
  }
  set_position_gain(1500);
  set_derivative_gain(0);
  set_integral_gain(0);
  
  //Serial port setup. This baudrate does not matter, teensy always uses 
  //120000
  Serial.begin(115200);
  Serial.setTimeout(serial_timeout_milis);
}

//This have an impact on trajectories
int16_t margin_allowed[number_of_motors]={30,40,40,30,40,40,30,40,40,30,40,40};


bool newdata=false;
size_t reads=0;
//bool update_is_allowed();
//only temporary read array
int32_t array_positions[number_of_motors];
//array used to store current positions
int32_t current_positions[number_of_motors];
//array used to store reference/target positions
int32_t reference_positions[number_of_motors];



void loop() {
  //updating the current positions

  //read_position_gain();
  //read_position_current();
  //ping_all_motors();
  //delay(2000);
  //Serial.println(dxl.getPort());
  //function to update all the foor sensors

  //update_feet_sensors(feet_touching,feet_input_pins);
  readPressureSensors(pressurePin,pressureValue);
  
  read_from_all_motors(current_positions);

  readIMU();

  send_all_positions(current_positions,pressureValue);
  //check if there is update from the serial port;
  bool inputs=input_from_serial(array_positions);
  if(inputs){    
    //maybe add this directly inside the function
    append_motor_executes(array_positions,motors_targets);
    //Serial.print(motors_targets[0][0]);
    //Serial.println(motors_targets[0].size());
    inputs=false;
  }


  //Execute commands from the buffer and removes, 
  //if it has been executed.
  execute_current_target(motors_targets,current_positions,
                         reference_positions);
  
  //set this delay lower...
  delay(0);
}




//old configs
//leg 4 bag left has ids 1,17,5
//leg 3 bag right has id 6,11,2
//const int32_t DXL_ID[number_of_motors]={1,17,5};//{15,12,8,3,9,14,6,11,2,1,17,5};

//limits leg 3 in the bag right:
//6 3500 2950
//11 4000 //2650
//2 1430       //600
//limits
//1 //3900      //4350
//17 900           //2500
//5 2150            //2950


//old config examples
/*    This is used for the initial prototype
//Specify the dxl_ids here in chosen order
const int32_t DXL_ID[number_of_motors]={6,3,17};
//It is important to specify the limits...
int32_t minlimit[number_of_motors]={1000,120,300};
int32_t maxlimit[number_of_motors]={1800,1000,2100};
*/
