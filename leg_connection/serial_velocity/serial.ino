void send_all_positions(int32_t current_pos[],int foot_sensors[],int32_t cur_vel[]){
  //Sending positions
  String string_to_send="";
  for(int i=0;i<number_of_motors;i++){
    if(current_pos[i]>=0){
      string_to_send += value_2_send(current_pos[i]);
    }else{
      string_to_send += value_2_send(0);
    }
  }
  //sending veloities
  //program a handle for negative values...
  int32_t threshold_=4092;
  for(int i=0;i<number_of_motors;i++){
    if(cur_vel[i]>=0){
      string_to_send+=value_2_send(cur_vel[i]);
    }else{
      int32_t val_negative=(threshold_*2-abs(cur_vel[i]));
      string_to_send+=value_2_send(val_negative);
    }
  }
  
  for(int i=0;i<number_of_feet_sensors;i++){
    if(foot_sensors[i]>0){
      string_to_send+="1";
    }else{
      string_to_send+="0";
    }
  }
  
  Serial.print(string_to_send);
  Serial.flush();
}

String value_2_send(int32_t val){
    String temp_str=String(val);
    int len=temp_str.length();
    switch(len){
      case 0:
        return "0000";
        break;
      case 1:
        return ("000"+temp_str);
        break;
      case 2:
        return ("00"+temp_str);
        break;
      case 3:
        return ("0"+temp_str);
        break;
      case 4:
        return temp_str;
        break;   
    }
}

//functions to setup and update the foot sensors:
void setup_input_sensors_feet(int feet_input_pins[]){
  for(int i=0;i<number_of_feet_sensors;i++){
    pinMode(feet_input_pins[i], INPUT);
  }
}

void update_feet_sensors(int feet_touchin[], int feet_input_pins[]){
  for(int i=0;i<number_of_feet_sensors;i++){
      feet_touchin[i]=digitalRead(feet_input_pins[i]);
  }
}


//Setting up bulk read
const uint8_t DXL_ID_CNT = number_of_motors;
const uint16_t user_pkt_buf_cap = 256;
uint8_t user_pkt_buf[user_pkt_buf_cap];
struct br_data_xel{
  //int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed));


struct br_data_xel br_data[number_of_motors];

DYNAMIXEL::InfoBulkReadInst_t br_infos;
DYNAMIXEL::XELInfoBulkRead_t info_xels_br[number_of_motors];

void setup_bulk_read(){
  br_infos.packet.p_buf = user_pkt_buf;
  br_infos.packet.buf_capacity = user_pkt_buf_cap;
  br_infos.packet.is_completed = false;
  br_infos.p_xels = info_xels_br;
  br_infos.xel_count = 0;
  
  for(int i=0;i<number_of_motors;i++){
    bool ping_=ping_example(DXL_ID[i]);
    if(ping_){
  
      dxl.torqueOff(DXL_ID[i]);
      //dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
      Serial.print("ids: ");Serial.println(DXL_ID[i]);
      info_xels_br[i].id = DXL_ID[i];
      info_xels_br[i].addr = 128; //Position of X serise.
      info_xels_br[i].addr_length = 4+4; // Present Position
      info_xels_br[i].p_recv_buf = (uint8_t*)&br_data[i];
      br_infos.xel_count++;
    }else{
      Serial.println("Problem in setup");
    }
    }
    br_infos.is_info_changed = true;

}

//setting up bulk write
//values that can be changed
struct bw_data_xel_pos{
  int32_t goal_velocity;
} __attribute__((packed));


struct bw_data_xel_pos bw_data_xel_pos[number_of_motors];
DYNAMIXEL::InfoBulkWriteInst_t bw_infos;
DYNAMIXEL::XELInfoBulkWrite_t info_xels_bw[number_of_motors];

void setup_bulk_write(){
  bw_infos.packet.p_buf = nullptr;
  bw_infos.packet.is_completed = false;
  bw_infos.p_xels = info_xels_bw;
  bw_infos.xel_count = 0;
  for(int i=0;i<number_of_motors;i++){
    bw_data_xel_pos[i].goal_velocity = 0;
    info_xels_bw[i].id = DXL_ID[i];
    info_xels_bw[i].addr = 104; // Goal Position of X serise.
    info_xels_bw[i].addr_length = 4; // Goal Position
    info_xels_bw[i].p_data = (uint8_t*)&bw_data_xel_pos[i];
    bw_infos.xel_count++;
  }
  bw_infos.is_info_changed = true;
}

//It is changed to bulk read here.
void read_from_all_motors(int32_t current_pos[],int32_t cur_vel[]){
  //change to bulk read here
  uint8_t recv_cnt = dxl.bulkRead(&br_infos); 
  if(recv_cnt>0){
    for(int i=0;i<number_of_motors;i++){
    current_pos[i]=br_data[i].present_position;
    cur_vel[i]=br_data[i].present_velocity;
    //only used for debug
    //Serial.print(DXL_ID[i]);Serial.print("   motor   ");
    //sSerial.println(current_position);
    }
  }
  else{
    //Serial.println("error");
  }
}


void set_actuator_limits(int32_t min_limit[],int32_t max_limit[]){
  for(int i=0;i<number_of_motors;i++){
    dxl.writeControlTableItem(MIN_POSITION_LIMIT, DXL_ID[i], min_limit[i]);
    dxl.writeControlTableItem(MAX_POSITION_LIMIT, DXL_ID[i], max_limit[i]);
  }
}

bool check_actuator_limits(int index,int32_t value){
  if(value>minlimit[index] and value<maxlimit[index])
    return true;
  else
    return false;
}


int8_t queue_size=10;


//needs to check if the value is within the limits otherwise it fails.
void append_motor_executes(int32_t array_of_values[],
vector<vector<int32_t>> &motor_list){
  for(int i=0;i<number_of_motors;i++){
    //only updating if queue size is not exceeded.
    //and the limits are not exceeded
    bool limits=check_actuator_limits(i,array_of_values[i]);
    if(motor_list[i].size()<(queue_size-1) and limits){
      motor_list[i].push_back(array_of_values[i]);
    }
  }
}

//Convert negative values not done yet....
//Current serial port communication functions 
void array_of_ints(int32_t array_output[]){
    int32_t threshold_=4092;
    for(int i=0;i<number_of_motors+1;i++){
      String temp;
      for(int j=0;j<4;j++){
        temp += buf[i*4+j];
      }
      int32_t val=temp.toInt();
      if(val<threshold_){
        array_output[i]=val;
      }else{
        array_output[i]=-(threshold_*2-val);
      }
      temp.remove(0,4);
    }//12341234123456781234123412345678
}




bool input_from_serial(int32_t array_output[]){
  while(Serial.available()){
      newdata=true;
      size_t bytecount = 0;
      reads=Serial.readBytes(buf,number_of_motors*4+1);
   }
   if(newdata){
    //check if read is requested
    if(buf[0]=='R' or buf[0]=='r'){
      //global variables that is being sended
      send_all_positions(current_positions,feet_touching,current_velocities);
      newdata = false;
      return false;
    }
    //update positions reads instead
    array_of_ints(array_output);
    newdata = false;
    return true;
  }
}

int32_t margins=100;
void check_position_limits(int32_t current_pos[],
                           int status_array[],
                           int32_t min_lim[],
                           int32_t max_lim[]){
  for(int i=0;i<number_of_motors;i++){
    if(current_pos[i]<(min_lim[i]+margins)){
      status_array[i]=1;
    }
    else if(current_pos[i]>(max_lim[i]-margins)){
      status_array[i]=-1;
    }else{
      status_array[i]=0;
    }
  }
}
  




//EXECUTES CURRENT VELOCITIES
int32_t velocity_max=200;


void execute_current_target(int32_t velocity_execute[], int stat[]){
  bool new_data=false;
  for(int i=0;i<number_of_motors;i++){
      //type of collision detected
      if(stat[i]==0){
        if(abs(velocity_execute[i])<velocity_max){
          bw_data_xel_pos[i].goal_velocity = velocity_execute[i];
        }
      }else if(stat[i]==1 and velocity_execute[i]>0){
        if(abs(velocity_execute[i])<velocity_max){
          bw_data_xel_pos[i].goal_velocity = velocity_execute[i];
        }
      }else if(stat[i]==-1 and velocity_execute[i]<0){
        if(abs(velocity_execute[i])<velocity_max){
          bw_data_xel_pos[i].goal_velocity = velocity_execute[i];
        }
      }else{
        bw_data_xel_pos[i].goal_velocity = 0;
      }
      bw_infos.is_info_changed = true;
      new_data=true;
  }
  //only update if there is new data
  if(new_data){
    dxl.bulkWrite(&bw_infos);  
  }
}

void ping_all_motors(){
  for(int i=0;i<5;i++){
    ping_example(i);
  }
}


//Example used to read position gain from motors
void set_position_gain(int16_t gain_){
    for(int i=0;i<12;i++){
    dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID[i],gain_);//112
  }
}

//Example used to read position gain from motors
void read_position_gain(){
    for(int i=0;i<12;i++){
    Serial.print("Motor   ");Serial.print(DXL_ID[i]);
    Serial.print("  gain  ");
    Serial.println(dxl.readControlTableItem(POSITION_P_GAIN, DXL_ID[i]));//112
  }
}

void read_position_current(){
    for(int i=0;i<12;i++){
    Serial.print("Motor   ");Serial.print(DXL_ID[i]);
    Serial.print("  gain  ");
    Serial.println(dxl.getPresentCurrent(DXL_ID[i]));//112
  }
}

//example used to ping the motors
bool ping_example(int8_t id){
    Serial.print("Ping ID ");
    Serial.print(id);
    Serial.print(": ");
    if(dxl.ping(id) == true){
      Serial.print("ping succeeded!");
      Serial.print(", Model Number: ");
      Serial.println(dxl.getModelNumber(id));
      return true;
    }else{
      Serial.print("ping failed!, err code: ");
      Serial.println(dxl.getLastLibErrCode());
      Serial.println("");
      return false;
    }
}
