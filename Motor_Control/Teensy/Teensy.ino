#include <isotp.h>
#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <kinetis_flexcan.h>
#include <imxrt_flexcan.h>
#include <isotp_server.h>

//#include <ArduinoSTL.h>

#include "Teensy.h"
// #define _USE_MATH_DEFINES

#include <cmath>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_F; // CAN bus for upper body motors
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_B; // CAN bus for lower body motors

// Globals
float joint_pos_desired[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // desired joint(motor) position [rad]
float rotor_pos[MOTOR_NUM];
uint16_t rotor_pos_raw_arr[MOTOR_NUM];
int r_num[MOTOR_NUM];

float motor_mode = 0;//0: position control; 1: velocity control

// Motor shaft position [rad]
float joint_pos[MOTOR_NUM];
// Motor shaft velocity [rad/s]
float joint_vel[MOTOR_NUM];
// Motor current [A]
float joint_cur[MOTOR_NUM];

constexpr float pi = 3.14159265358979323846;

//                                         0      1      2       3         4      5     6     7     8      9     10      11
float joint_lower_limit[MOTOR_NUM] = {-pi/1.25, -pi/2, -pi/2, -pi/1.25, -pi/2, -pi/2, -pi/3,  0,  -pi/3, -pi/3, -pi/2, -pi/3};
float joint_upper_limit[MOTOR_NUM] = { pi/1.25,  pi/2,  pi/2,  pi/1.25,  pi/2,  pi/2,  pi/3, pi/2, pi/3,  pi/3,  0,     pi/3};
float joint_axis_direction[MOTOR_NUM] = {1, 1, 1, 1, 1, 1, 1, -1, -1, 1, -1, -1};

Teensycomm_struct_t teensy_comm = {{}, {}, {}, {}, {}}; // For holding data sent to Jetson
Jetson_comm_struct_t jetson_comm = {{}};                // For holding data received from Jetson

static uint8_t *ptin = (uint8_t *)(&jetson_comm);
static uint8_t *ptout = (uint8_t *)(&teensy_comm);
static int in_cnt = 0;

bool jetson_comm_ready = false; //flag indicating the communication is ready cans = {&}

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus

// lookup table to relate motor index i to the CAN bus and its motor CAN address
// 0 stands for CAN_F, 1 stands for CAN_B
constexpr bool joint_can_lane[MOTOR_NUM] = {0, 0, 0, 0, 0, 0,
                                  1, 1, 1, 1, 1, 1};
// Motors' id from 0x141 to 0x146
constexpr uint16_t joint_can_addr[MOTOR_NUM] = {0x141, 0x142, 0x143, 0x144, 0x145, 0x146,
                                      0x141, 0x142, 0x143, 0x144, 0x145, 0x146};


bool can_read_success[MOTOR_NUM];

//............foot sensor
constexpr int pin_left_forefoot = 16;
constexpr int pin_left_backheel = 17;
constexpr int pin_right_forefoot = 20;
constexpr int pin_right_backheel = 21;

bool left_touch = 0;
bool right_touch = 0;
constexpr int force_threshold = 128;
//................

CAN_message_t msg_recv_arr[MOTOR_NUM];
bool flag_recv_arr[MOTOR_NUM];

constexpr int MAX_CAN_ATTEMPTS = 200; // try at most MAX_CAN_ATTEMPTS times to 



float motor_multi_anlge[MOTOR_NUM];//motor shaft position directly measured from the motor
void processMotorRotorMultiAngle(int id,uint8_t* buf, bool init_flag=false){
  int64_t rotor_angle = 0; // encodor multi-turn angle
  *(uint8_t *)(&rotor_angle)       = buf[1];
  *((uint8_t *)(&rotor_angle) + 1) = buf[2];
  *((uint8_t *)(&rotor_angle) + 2) = buf[3];
  *((uint8_t *)(&rotor_angle) + 3) = buf[4];
  *((uint8_t *)(&rotor_angle) + 4) = buf[5];
  *((uint8_t *)(&rotor_angle) + 5) = buf[6];
  *((uint8_t *)(&rotor_angle) + 6) = buf[7];  
  if (buf[7]==0xff){*((uint8_t *)(&rotor_angle) + 7) = 0xff;} // negative number
  float rotor_multi_angle = ((float)(rotor_angle))*0.01*pi/180.;
  if (init_flag){ // initalize the r_num and rotor_pos at first call
  r_num[id] = rotor_multi_angle/(2*pi);
  rotor_pos[id] = fmodf(rotor_multi_angle,2*pi);
  }
  motor_multi_anlge[id] = rotor_multi_angle/REDUCTION_RATIO;
}

/*get the motor shaft position using the multi-turn encoder position*/
void getMotorRotorMultiAngle(bool init_flag=false){
  CAN_message_t msg; /// msg to be sent to the motor
  msg.buf[0] = 0x92; // get rotor muiti-angle command
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  CAN_message_t msg_r;//received message from CAN_F/CAN_B
  for (int i = 0; i < MOTOR_NUM / 2; ++i)
  {
    // send msg to CAN_F
    msg.id = joint_can_addr[i];
    CAN_F.write(msg);
    // send msg to CAN_B
    msg.id = joint_can_addr[i + MOTOR_NUM / 2];
    CAN_B.write(msg);
    // TODO: CHEKC IF THERE IS A BETTER WAY THAN DELAY
    delayMicroseconds(10); // wait for 10 microsecond before checking
    for(int k=0;k<MAX_CAN_ATTEMPTS; ++k){
      if (CAN_F.read(msg_r)){
        processMotorRotorMultiAngle(msg_r.id- 0x141, msg_r.buf,init_flag); // from CAN_F
        break;
      }
    }
    for(int k=0;k<MAX_CAN_ATTEMPTS; ++k){
      if (CAN_B.read(msg_r)){
        processMotorRotorMultiAngle(msg_r.id-0x141+MOTOR_NUM / 2,msg_r.buf,init_flag); // from CAN_B
        break;
      }
    }
  }
}

void processMotorStatus(int id);
void readMotorStatus(int motor_id);

/* grouped CAN write then read 
e.g. if NUM_PER_GROUP is 2, the squrence of CAN write then read is:
  CAN_F: (0,1), CAN_B: (6,7)
  CAN_F: (2,3), CAN_B: (8,9)
  CAN_F: (4,5), CAN_B: (10,11)
*/
constexpr int NUM_PER_CAN = MOTOR_NUM/2; // # of motor per CAN BUS
constexpr int NUM_PER_GROUP = 2;
// grouped read and write, MUST BE DEVISABLE
// constexpr int NUM_GROUPED_RW = NUM_PER_CAN/NUM_PER_GROUP; 
// rounding up trick: int val = x/y + !!(x%y);
// The double NOT makes any non-zero value into a 1.
constexpr int NUM_GROUPED_RW = NUM_PER_CAN/NUM_PER_GROUP + !!(NUM_PER_CAN%NUM_PER_GROUP);

/* send zero torque command to the motor and recives motor status*/
void zeroTorqueControl(){
  msg_send.buf[0] = 0xA1; // zero torque command
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = 0x00;
  msg_send.buf[7] = 0x00;

  for (int i = 0; i < MOTOR_NUM; ++i){
    can_read_success[i] = false;
  }

  for(int i = 0; i < NUM_GROUPED_RW; ++i){
    for (int j = 0; j < NUM_PER_GROUP; ++j){
      int k = i*NUM_PER_GROUP+j;
      msg_send.id = joint_can_addr[k];
      CAN_F.write(msg_send);

      msg_send.id = joint_can_addr[k + NUM_PER_CAN];
      CAN_B.write(msg_send);
    }
    for (int j = 0; j < NUM_PER_GROUP; ++j){
      int k = i*NUM_PER_GROUP+j;
      readMotorStatus(k);
      readMotorStatus(k+NUM_PER_CAN);
    }
  }

  // for (int i = 0; i < MOTOR_NUM / 2; ++i)
  // {
  //   msg_send.id = joint_can_addr[i];
  //   CAN_F.write(msg_send);
  //   int j = i + MOTOR_NUM / 2; //motor index
  //   msg_send.id = joint_can_addr[j];
  //   CAN_B.write(msg_send);
  //   // delayMicroseconds(100);
  //   // read tempterature, current,speed, encoder position
  //   readMotorStatus(i);
  //   readMotorStatus(i+MOTOR_NUM / 2);
  // }

}

 // read tempterature, current,speed, encoder position from motor
inline void readMotorStatus(int motor_id){
  if(motor_id<NUM_PER_CAN){ // from CAN_F
    for(int k=0;k<MAX_CAN_ATTEMPTS; ++k){
      if (CAN_F.read(msg_recv)){
        int id = msg_recv.id- 0x141; // actual received id
        can_read_success[id] = true;
        processMotorStatus(id, msg_recv.buf); // from CAN_F
        return;//break;
      }
    }
  }
  else if (motor_id<MOTOR_NUM){  // from CAN_B
    for(int k=0;k<MAX_CAN_ATTEMPTS; ++k){
      if (CAN_B.read(msg_recv)){
        int id = msg_recv.id-0x141+NUM_PER_CAN;
        can_read_success[id] = true;
        processMotorStatus(id,msg_recv.buf); // from CAN_B
        return; //break;
      }
    }
  }
}


/* control the i-th motor with pos_command, and limited by max_speed
Arguments:
  int i: motor index
  float pos_command: desired shaft position rad/s
  uint16_t max_speed: degrees per second of the rotor, 1 degree per second <-> 1 LSB
                      note that you should multiply by REDUCTION_RATIO */
void controlSingleMotor(const int i, float pos_command, const uint16_t max_speed=180*REDUCTION_RATIO){
    // check and clamp the pos_command within the joint limits
    clampInplace(pos_command, joint_lower_limit[i], joint_upper_limit[i]);
    // Convert motor shaft position command [rad] to rotor position command [degree]
    pos_command = pos_command * 180.0 / pi * REDUCTION_RATIO * joint_axis_direction[i];
    int32_t pos = (int32_t)round(pos_command * 100); // 0.01 deg per LSB
    CAN_message_t msg; // For sending data on CAN bus
    msg.buf[0] = 0xA4; // Position control mode with speed limit, refer to motor manual page 13 
    msg.buf[1] = 0x00;
    msg.buf[2] = *(uint8_t *)(&max_speed);
    msg.buf[3] = *((uint8_t *)(&max_speed) + 1);
    msg.buf[4] = *(uint8_t *)(&pos);
    msg.buf[5] = *((uint8_t *)(&pos) + 1);
    msg.buf[6] = *((uint8_t *)(&pos) + 2);
    msg.buf[7] = *((uint8_t *)(&pos) + 3);
    msg.id = joint_can_addr[i];
    if(i<NUM_PER_CAN){ // from CAN_F
      CAN_F.write(msg);
    }
    else if (i<MOTOR_NUM){  // from CAN_B
      CAN_B.write(msg);
    }
}

void dummyControlMotors()  {
    float pos_command = 0; // [rad]
    float t = float(millis()) * 0.01*0.2;
    // pos_command = sin(t) * pi / 4.f;
    pos_command = pi/3;
    // pos_command = pi / 2.f;
    controlSingleMotor(0, pos_command);
    controlSingleMotor(1, pos_command);
    controlSingleMotor(2, pos_command);
    controlSingleMotor(3, pos_command);
    controlSingleMotor(4, pos_command);
    controlSingleMotor(5, pos_command);
    readMotorStatus(0);
    readMotorStatus(1);
    readMotorStatus(2);
    readMotorStatus(3);
    readMotorStatus(4);
    readMotorStatus(5);
  }


/*joint position control*/
void controlMotors(float max_speed=180*REDUCTION_RATIO)
{
  for (int i = 0; i < MOTOR_NUM; ++i){
    can_read_success[i] = false;
  }
  for(int i = 0; i < NUM_GROUPED_RW; ++i){
    for (int j = 0; j < NUM_PER_GROUP; ++j){
      int k = i*NUM_PER_GROUP+j;
      controlSingleMotor(k, joint_pos_desired[k],max_speed);
      int m = k + NUM_PER_CAN;
      controlSingleMotor(m, joint_pos_desired[m],max_speed);
      delayMicroseconds(10); // wait a bit for the motor to process
    }
    
    for (int j = 0; j < NUM_PER_GROUP; ++j){
      int k = i*NUM_PER_GROUP+j;
      readMotorStatus(k);
      readMotorStatus(k+NUM_PER_CAN);
    }
  }
} 

/*helper function to process the motor return message from command 0x9C, 0xA1-0xA8
  id: index of the motor
  buf: pointer to the CAN_message_t object buffer
  init_flag: bool flag inidicating whether initialization is needed
*/
void processMotorStatus(int id, uint8_t* buf)
{
  // Receiving motor angle
  // Transfer hex number to rad
  uint16_t rotor_pos_raw = 0; // Rotor position before devided by gear reduction
  *(uint8_t *)(&rotor_pos_raw) = buf[6];
  *((uint8_t *)(&rotor_pos_raw) + 1) = buf[7];

  // rotor encoder position. 16 bit encoder 65535(0xFFFF) refers to 360 deg (2pi rad)
  float rotor_pos_now = (float)rotor_pos_raw / 65535.0 * 2 * pi;

  rotor_pos_raw_arr[id] = rotor_pos_raw;//FOR DEBUG

  float rotor_pos_diff = rotor_pos_now - rotor_pos[id];

  if (rotor_pos_diff < -pi)
  {
    r_num[id] += 1;
  }
  else if (rotor_pos_diff > pi)
  {
    r_num[id] -= 1;
  }
   rotor_pos[id] = rotor_pos_now;

  // shaft angular position [rad]
  joint_pos[id] = (rotor_pos[id] + r_num[id] * 2 * pi) / REDUCTION_RATIO * joint_axis_direction[id];
  // clampPeroidicInplace(joint_pos[id], -pi, pi);

  // Calculate motor's current [A], -33A ~ 33A
  int16_t cur_raw = 0;
  *(uint8_t *)(&cur_raw) = buf[2];
  *((uint8_t *)(&cur_raw) + 1) = buf[3];
  joint_cur[id] = (float)cur_raw * 33.0 / 2048.0 * joint_axis_direction[id]; // 2048 refers to 33A

  // Calculate shaft velocity [rad/s]
  int16_t rotor_vel_raw = 0;
  *(uint8_t *)(&rotor_vel_raw) = buf[4];
  *((uint8_t *)(&rotor_vel_raw) + 1) = buf[5];
  joint_vel[id] = (float)rotor_vel_raw * pi / (180.0 * REDUCTION_RATIO) * joint_axis_direction[id];
}

void receiveFromSerial()
{
  in_cnt = 0;
  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(jetson_comm)))
  {
    ptin[in_cnt++] = Serial.read();
  }

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(jetson_comm))
  {
    in_cnt = 0; // Clear incoming bytes counter
    jetson_comm_ready = true;
  }

  if (jetson_comm_ready)
  {
    jetson_comm_ready = false;
    motor_mode = jetson_comm.motor_mode;
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
      joint_pos_desired[i] = jetson_comm.comd[i];
      // clampPeroidicInplace(joint_pos_desired[i], -pi, pi);
      // clampInplace(joint_pos_desired[i], joint_lower_limit[i], joint_upper_limit[i]);
    }
  }
}

void sendToSerial(){
  // Save angle, speed and torque into struct teensy_comm
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    teensy_comm.joint_pos[i] = joint_pos[i];
    teensy_comm.joint_vel[i] = joint_vel[i];
    teensy_comm.joint_cur[i] = joint_cur[i];
  }

  teensy_comm.timestamps = (float)micros() / 1000000.0;

  //...............foot sensor
  // from 0-1023, 5v, measure fixed resistor
  teensy_comm.foot_force[0] = analogRead(pin_left_forefoot);  //pin16:v1
  teensy_comm.foot_force[1] = analogRead(pin_left_backheel);  //pin21:v4
  teensy_comm.foot_force[2] = analogRead(pin_right_forefoot); //pin17:v2
  teensy_comm.foot_force[3] = analogRead(pin_right_backheel); //pin20:v3
  //...............
  // Send data structure teensy_comm to Jetson
  Serial.write(ptout, sizeof(teensy_comm));
  // Force immediate transmission
  Serial.send_now();
}

void serialDebugHelper(){

  static int counter = 0;
  static int counter_prev = 0;
  static int can_read_success_counter[MOTOR_NUM];
  static unsigned long t = millis();

  counter++;
  for (int i = 0; i < MOTOR_NUM; ++i){
    can_read_success_counter[i]+= int(can_read_success[i]);
  }

  if (millis() - t > 500)
  {
    t = millis();

    int conter_difference = counter - counter_prev;
    float fps = float(counter - counter_prev)*0.5;
    counter_prev = counter;

    Serial.print(t);
    Serial.print(":");

    Serial.print(fps);
    Serial.print("||");
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
      // Serial.print(joint_pos[i]-joint_pos_desired[i]);
      // Serial.print(" ");

      Serial.print(1-float(can_read_success_counter[i])/float(conter_difference));
      Serial.print(" ");
      can_read_success_counter[i] = 0;
      
      // Serial.print(motor_multi_anlge[i]);
      // Serial.print(joint_cur[i]);
    }
    Serial.print("\n");
  }
}

void setup()
{
  // Open Serial port Baudrate 4mbits/s
  Serial.begin(USB_UART_SPEED);

  // start CAN bus
  CAN_F.begin();
  CAN_F.setBaudRate(1000000);
  CAN_F.setClock(CLK_60MHz);

  CAN_B.begin();
  CAN_B.setBaudRate(1000000);
  CAN_B.setClock(CLK_60MHz);

  // initialize the motor
  getMotorRotorMultiAngle(true); // initialize the motor position
  // reset the motor to its desired_pos
  unsigned long t = millis(); // current time
  while(millis() - t < 2000){//wait for 2000 ms for the motor to reset
    controlMotors(90*REDUCTION_RATIO);      
  }
}



void loop()
{

  controlMotors();
  // dummyControlMotors();
  // zeroTorqueControl();
  // getMotorRotorMultiAngle(false);
  // serialDebugHelper();

  receiveFromSerial();
  sendToSerial();
}
