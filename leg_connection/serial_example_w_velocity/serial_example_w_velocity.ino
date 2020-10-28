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
#define number_of_motors 12
#define number_of_velocities 12
char buf[number_of_motors*4+1+number_of_velocities*4];

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

//can be initialized with initial positions and be used as reference positions
//and then be updated from serial. 
vector<vector<int32_t>> motors_targets(number_of_motors, vector<int32_t>(0));
vector<vector<int32_t>> motor_velocities_targets(number_of_velocities, vector<int32_t>(0));
//reference_velocities
int32_t reference_velocities[number_of_velocities];

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

  //Setting tacc profile
  int16_t acc_profile=60;
  for(int i=0;i<number_of_motors;i++){
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
    dxl.torqueOn(DXL_ID[i]);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], acc_profile);//108
  }

  //the velocity profile now needs to be assigned here.
  int16_t velocity_profile=20;
  for(int i=0;i<number_of_velocities;i++){
     reference_velocities[i]=velocity_profile;
  }
  
  //setting the positional gain
  set_position_gain(1000);
  
  //Serial port setup. This baudrate does not matter, teensy always uses 
  //120000
  Serial.begin(115200);
  Serial.setTimeout(serial_timeout_milis);
}

//This have an impact on trajectories
int16_t margin_allowed[number_of_motors]={10,40,40,10,40,40,10,40,40,10,40,40};


bool newdata=false;
size_t reads=0;
//only temporary read arrays
int32_t array_positions[number_of_motors];
int32_t array_velocities[number_of_velocities];

//array used to store current positions and reference velocities
int32_t current_positions[number_of_motors];
//contains the reference velocities on the motors.
//array used to store reference/target positions
int32_t reference_positions[number_of_motors];


void loop() {
  //updating the current positions
  //standard functions used for debug.
  
  //read_position_gain();
  //read_position_current();
  //ping_all_motors();
  
  //delay(2000);
  //Serial.println(dxl.getPort());
  
  //function to update all the foor sensors
  update_feet_sensors(feet_touching,feet_input_pins);
  
  read_from_all_motors(current_positions);


  //check if there is update from the serial port;
  bool inputs=input_from_serial(array_positions,array_velocities);
  if(inputs){
    //maybe add this directly inside the inpit function.
    append_motor_executes(array_positions,
                          array_velocities,
                          motors_targets,
                          motor_velocities_targets);
    
    inputs=false;
  }


  //Execute commands from the motor and velocity buffer 
  //and removes, if it has been executed.
  execute_current_target(motors_targets, 
                         motor_velocities_targets,  
                         current_positions,
                         reference_positions,
                         reference_velocities);
  
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
