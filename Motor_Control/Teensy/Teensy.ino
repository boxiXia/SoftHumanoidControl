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
float joint_speed_desired[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float rotor_pos[MOTOR_NUM];
float rotor_pos_prev[MOTOR_NUM];
int r_num[MOTOR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float motor_mode;

float time_now;
float time_former;

// Motor shaft position [rad]
float joint_pos[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Motor shaft velocity [rad/s]
float joint_vel[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Motor current [A]
float joint_cur[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


constexpr float pi = 3.14159265358979323846;

//                                         0          1      2         3       4       5        6        7      8      9      10       11 
float joint_lower_limit[MOTOR_NUM]    = {-pi/1.25, -pi/2, -pi/2,   -pi,      -pi/2,  -pi/2,  -pi/3,      0,  -pi/3, -pi/3,  -pi/2,  -pi/3};
float joint_upper_limit[MOTOR_NUM]    = { pi/1.25,  pi/2,  pi/2,    pi/1.25,  pi/2,   pi/2,   pi/3,   pi/2,   pi/3,  pi/3,      0,   pi/3};
float joint_axis_direction[MOTOR_NUM] = { 1,           1,     1,     1,          1,      1,      1,     -1,     -1,     1,     -1,     -1};


float delta_t;                                                      // loop time difference
Teensycomm_struct_t teensy_comm = {{}, {}, {}, {}, {}}; // For holding data sent to Jetson
Jetson_comm_struct_t jetson_comm = {{}};                            // For holding data received from Jetson

static uint8_t *ptin = (uint8_t *)(&jetson_comm);
static uint8_t *ptout = (uint8_t *)(&teensy_comm);
static int in_cnt = 0;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus

// lookup table to relate motor index i to the CAN bus and its motor CAN address
// 0 stands for CAN_F, 1 stands for CAN_B
bool joint_can_lane[MOTOR_NUM] = {0, 0, 0, 0, 0, 0,
                                  1, 1, 1, 1, 1, 1};
// Motors' id from 0x141 to 0x146
uint16_t joint_can_addr[MOTOR_NUM] = {0x141, 0x142, 0x143, 0x144, 0x145, 0x146,
                                      0x141, 0x142, 0x143, 0x144, 0x145, 0x146};

//............foot sensor
const int pin_left_forefoot = 16;
const int pin_left_backheel = 17;
const int pin_right_forefoot = 20;
const int pin_right_backheel = 21;

bool left_touch = 0;
bool right_touch = 0;
const int force_threshold = 128;
//................

void Motor_Init()
{


  // // Motor position initial CAN bus command
  // // All motors rotate to position 0 rad
  // msg_send.buf[0] = 0x19; //CAN bus position command ID
  // msg_send.buf[1] = 0x00;
  // msg_send.buf[2] = 0x00;
  // msg_send.buf[3] = 0x00;
  // msg_send.buf[4] = 0x00;
  // msg_send.buf[5] = 0x00;
  // msg_send.buf[6] = 0x00;
  // msg_send.buf[7] = 0x00;

  // msg_send.id = joint_can_addr[6];
  // CAN_B.write(msg_send);
  // while (true)
  // {
  //   if (CAN_B.read(msg_recv))
  //   {
  //     processMotorData(6);
  //     break;
  //   }
  // }
  // msg_send.id = joint_can_addr[9];
  // CAN_B.write(msg_send);
  // while (true)
  // {
  //   if (CAN_B.read(msg_recv))
  //   {
  //     processMotorData(9);
  //     break;
  //   }
  // }
  // Motors' IDs range from 0x141 to 0x146.
  // CAN_F is connected to upper body motors(id: 0x141 to 0x146)
  // CAN_B is connected to lower body motors(id: 0x141 to 0x146)
  msg_send.buf[0] = 0xA3; //CAN bus position command ID
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = 0x00;
  msg_send.buf[7] = 0x00;
  for (int i = 0; i < MOTOR_NUM / 2; ++i)
  {
    msg_send.id = joint_can_addr[i];
    CAN_F.write(msg_send);
    while (true)
    {
      if (CAN_F.read(msg_recv))
      {
        processMotorData(i);
        break;
      }
    }
    msg_send.id = joint_can_addr[i + MOTOR_NUM / 2];
    CAN_B.write(msg_send);
    while (true)
    {
      if (CAN_B.read(msg_recv))
      {
        processMotorData(i + MOTOR_NUM / 2);
        break;
      }
    }
  }
  //delay(400); //TODO... CHECK WHAT THIS IS
}

void processMotorData(int id)
{
  // Receiving motor angle
  // Transfer hex number to rad
  int16_t rotor_pos_raw = 0; // Rotor position before devided by gear reduction
  *(uint8_t *)(&rotor_pos_raw) = msg_recv.buf[6];
  *((uint8_t *)(&rotor_pos_raw) + 1) = msg_recv.buf[7];
  //TODO CHECK
  rotor_pos[id] = (float)rotor_pos_raw / 65535.0 * 2 * PI; // 65535(0xFFFF) refers to 2PI
  if (rotor_pos[id] - rotor_pos_prev[id] < -PI)
    r_num[id] += 1;
  else if (rotor_pos[id] - rotor_pos_prev[id] > PI)
    r_num[id] -= 1;

  // Calculate shaft angular position [rad]
  rotor_pos_prev[id] = rotor_pos[id];
  joint_pos[id] = (rotor_pos[id] + r_num[id] * 2 * PI) / REDUCTION_RATIO * joint_axis_direction[id];

  // Calculate shaft velocity [rad/s]
  int16_t rotor_vel_raw = 0;
  *(uint8_t *)(&rotor_vel_raw) = msg_recv.buf[4];
  *((uint8_t *)(&rotor_vel_raw) + 1) = msg_recv.buf[5];
  joint_vel[id] = (float)rotor_vel_raw * PI / (180.0 * REDUCTION_RATIO) * joint_axis_direction[id];

  // Calculate motor's current [A], -33A ~ 33A
  int16_t cur_raw = 0;
  *(uint8_t *)(&cur_raw) = msg_recv.buf[2];
  *((uint8_t *)(&cur_raw) + 1) = msg_recv.buf[3];
  joint_cur[id] = (float)cur_raw* 33.0 / 2048.0 * joint_axis_direction[id]; // 2048 refers to 33A
}

void Angle_Control_Loop(int motor_id, float pos_command, float motor_mode)
{
  // Convert motor shaft angle command [rad] to rotor angle command [degree]
  pos_command = pos_command * 180.0 * REDUCTION_RATIO / PI * joint_axis_direction[motor_id];
  // see motor manual p12 (0xA3)
  int32_t pos = (int32_t)round(pos_command / 0.01);

  int pos_limit = 10;
  // see motor manual p12 (0xA3)
  int32_t poslimit = (int32_t)round(pos_limit / 0.01);
  // Set the CAN message ID as 0x200
  if (motor_mode == 0)
  {
    msg_send.id = joint_can_addr[motor_id];
    msg_send.buf[0] = 0xA4; // Position control mode
    msg_send.buf[1] = 0x00;
    msg_send.buf[2] = *(uint8_t *)(&poslimit);
    msg_send.buf[3] = *((uint8_t *)(&poslimit) + 1);
    msg_send.buf[4] = *(uint8_t *)(&pos);
    msg_send.buf[5] = *((uint8_t *)(&pos) + 1);
    msg_send.buf[6] = *((uint8_t *)(&pos) + 2);
    msg_send.buf[7] = *((uint8_t *)(&pos) + 3);
  }
  else
  {
    msg_send.id = joint_can_addr[motor_id];
    msg_send.buf[0] = 0xA2; // Speed control mode
    msg_send.buf[1] = 0x00;
    msg_send.buf[2] = 0x00;
    msg_send.buf[3] = 0x00;
    msg_send.buf[4] = *(uint8_t *)(&pos);
    msg_send.buf[5] = *((uint8_t *)(&pos) + 1);
    msg_send.buf[6] = *((uint8_t *)(&pos) + 2);
    msg_send.buf[7] = *((uint8_t *)(&pos) + 3);
  }

  if (joint_can_lane[motor_id] == 0)
  {
    CAN_F.write(msg_send); // Write the message on CAN bus
    while (true)
    {
      if (CAN_F.read(msg_recv))
      {
        processMotorData(motor_id);
        break;
      }
    }
  }

  else
  {
    CAN_B.write(msg_send); // Write the message on CAN bus
    while (true)
    {
      if (CAN_B.read(msg_recv))
      {
        processMotorData(motor_id);
        break;
      }
    }
  }
}

void Jetson_Teensy()
{
  in_cnt = 0;

  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(jetson_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(jetson_comm))
  {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Save angle, speed and torque into struct teensy_comm
    for (int i = 0; i < MOTOR_NUM; i++)
    {
      teensy_comm.joint_pos[i] = joint_pos[i];
      teensy_comm.joint_vel[i] = joint_vel[i];
      teensy_comm.joint_cur[i] = joint_cur[i];
    }

    teensy_comm.timestamps = time_now / 1000000.0;

    //...............foot sensor
    // teensy_comm.foot_force[0] = float(analogRead(pin_left_forefoot)) / 1023.0 * 5.0;  //pin16:v1
    // teensy_comm.foot_force[1] = float(analogRead(pin_left_backheel)) / 1023.0 * 5.0;  //pin21:v4
    // teensy_comm.foot_force[2] = float(analogRead(pin_right_forefoot)) / 1023.0 * 5.0; //pin17:v2
    // teensy_comm.foot_force[3] = float(analogRead(pin_right_backheel)) / 1023.0 * 5.0; //pin20:v3
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
}

void setup()
{
  // Switch on CAN bus
  Serial.begin(USB_UART_SPEED);

  CAN_F.begin();
  CAN_F.setBaudRate(1000000);
  CAN_F.setClock(CLK_60MHz);

  CAN_B.begin();
  CAN_B.setBaudRate(1000000);
  CAN_B.setClock(CLK_60MHz);
  // Open Serial port in speed 1000000 Baudrate
  Motor_Init();

  time_former = (float)micros();
}

void loop()
{

  Jetson_Teensy();

  time_now = (float)micros();
  delta_t = (time_now - time_former) / 1000000.0;
  time_former = time_now;

  motor_mode = jetson_comm.motor_mode[0];
  if (motor_mode == 0)
  {
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
      joint_pos_desired[i] = jetson_comm.comd[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
      if (joint_pos_desired[i] > joint_upper_limit[i])
        joint_pos_desired[i] = joint_upper_limit[i];
      else if (joint_pos_desired[i] < joint_lower_limit[i])
        joint_pos_desired[i] = joint_lower_limit[i];
    }
  }
  else if (motor_mode == 1)
  {
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
      joint_pos_desired[i] = jetson_comm.comd[i];
    }

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
      if (teensy_comm.joint_pos[i] > joint_upper_limit[i])
      {
        if (joint_pos_desired[i] > 0)
        {
          joint_pos_desired[i] = 0;
        }
      }
      else if (teensy_comm.joint_pos[i] < joint_lower_limit[i])
        if (joint_pos_desired[i] < 0)
        {
          joint_pos_desired[i] = 0;
        }
    }
  }

  for (int i = 0; i < MOTOR_NUM / 2; ++i)
  {
    Angle_Control_Loop(i, joint_pos_desired[i], motor_mode);
    int j = i + MOTOR_NUM / 2;
    Angle_Control_Loop(j, joint_pos_desired[j], motor_mode);
  }
}
