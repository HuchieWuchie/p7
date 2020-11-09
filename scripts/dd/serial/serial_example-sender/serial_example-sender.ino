#include <vector>
#include <Dynamixel2Arduino.h>  

//setup for the foot sensors
#define number_of_feet_sensors 4
//array containing info of the sensors
int feet_touching[number_of_feet_sensors]={0,0,1,1};//,0};
//assign accordingly
int feet_input_pins[number_of_feet_senso
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
  int16_t velocity_profile=20;
  int16_t acc_profile=30;
  for(int i=0;i<number_of_motors;i++){rs]={23,22,21,20};//,0};


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
int32_t minlimit[number_of_motors]={3200,1200,600,1300,1250,1080,800,800,2500,750,2260,2000};
int32_t maxlimit[number_of_motors]={3800,2520,1400,1800,2650,1975,1300,2050,3500,1300,3600,3000};

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
  int16_t velocity_profile=20;
  int16_t acc_profile=10;
  for(int i=0;i<number_of_motors;i++){
    //change_reverse_mode(false,9);
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
    dxl.torqueOn(DXL_ID[i]);
    if(DXL_ID[i]==8,DXL_ID[i]==14,DXL_ID[i]==2,DXL_ID[i]==5){
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
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], velocity_profile);//112
    }
  }
  set_position_gain(1000);
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
  
  read_from_all_motors(current_positions);


  send_all_positions(current_positions,feet_touching);
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
  delay(1);
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
