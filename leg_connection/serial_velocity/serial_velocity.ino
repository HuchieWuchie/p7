#include <vector>
#include <Dynamixel2Arduino.h>  

//setup for the foot sensors
#define number_of_feet_sensors 4
//array containing info of the sensors
int feet_touching[number_of_feet_sensors]={0,0,0,0};//,0};
//assign accordingly
int feet_input_pins[number_of_feet_sensors]={23,22,21,20};//,0};


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
int32_t minlimit[number_of_motors]={2200,2500,1050,1300,900,1600,2950,2650,600,750,900,2150};
int32_t maxlimit[number_of_motors]={3000,4000,1900,1800,2600,2400,3500,4000,1430,1300,2500,2950};

/*
//Setup fo Leg number 1 front right leg.
//Specify the dxl_ids here in chosen order
const int32_t DXL_ID[number_of_motors]={15,12,8};
//It is important to specify the limits...
int32_t minlimit[number_of_motors]={1450,2500,1050};
int32_t maxlimit[number_of_motors]={2000,4000,1900};
*/


int serial_timeout_milis=1;

void change_reverse_mode(bool input,int32_t id){
  if(input){//Writing it to go to normal mode
    dxl.writeControlTableItem(DRIVE_MODE, id,0);
    //0 0 0 0 0 0 0 0
  }
  else{//writing it to go to reverse mode
    dxl.writeControlTableItem(DRIVE_MODE, id,1);
  }
}


void setup() {
  //Setting up dynamixels
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  //Can be used to ping
  //ping_all_motors(),
  
  //Function to setup pins for feet sensors
  //setup_input_sensors_feet(feet_input_pins); 

  //setting up bulk read//
  setup_bulk_read();

  //set the actuator limits
  set_actuator_limits(minlimit,maxlimit);
  setup_bulk_write();

  //Setting the profile
  int32_t acc_profile=30;
  for(int i=0;i<number_of_motors;i++){
    //change_reverse_mode(false,9);
    dxl.torqueOff(DXL_ID[i]);
    //update to OP_VELOCITY
    dxl.setOperatingMode(DXL_ID[i], OP_VELOCITY);
    //dxl.torqueOn(DXL_ID[i]);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], 0);//108
  }
  //is not gonna be used with velocity control mode
  set_position_gain(1000);
  
  //Serial port setup. This baudrate does not matter, teensy always uses 
  //120000
  Serial.begin(115200);
  Serial.setTimeout(serial_timeout_milis);
}


bool newdata=false;
size_t reads=0;
//bool update_is_allowed();
//only temporary read array
int32_t array_velocities[number_of_motors];

//array used to store current positions
int32_t current_positions[number_of_motors];
//array to store current velocity
int32_t current_velocities[number_of_motors];

//array used to store reference/target positions
//should possibly be an array
int status_array[number_of_motors]={0,0,0,0,0,0,0,0,0,0,0,0};

elapsedMicros starts;

void loop() {
  //updating the current positions

  //read_position_gain();
  //read_position_current();
  //ping_all_motors();
  //4012000000002552

  //Serial.println(starts);
  //starts=0;
  //function to update all the foor sensors
  //update_feet_sensors(feet_touching,feet_input_pins);

  read_from_all_motors(current_positions,current_velocities);

  //
  //check if there is update from the serial port;
  bool inputs=input_from_serial(array_velocities);


  //Execute commands from the buffer and removes, 
  //if it has been executed.
  check_position_limits(current_positions,
                        status_array,
                        minlimit,
                        maxlimit);
                        
  //Serial.println(status_array[0]);      
  execute_current_target(array_velocities,status_array);

  //position checker:

  
  //set this delay lower...
  //delay(1);
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
