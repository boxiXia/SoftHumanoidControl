#include <FlexCAN_T4.h>
#include "TeensyCAN.h"
//
// Choose CAN2 as the CAN port. In Teensy4.0, CAN2 pins are
// PIN0(CRX) and PIN1(CTX). Setting RX and TX queue size as
// 256 and 16.
//
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;

// Globals
bool   can_received[NUM_MOTOR]       = {0};   // For checking the data receiving status of each motor
float  shaft_angle_meas[NUM_MOTOR]   = {0.0};   // Angle measured of each motor's shaft, unit degree, 0 - 360
float  rotor_angle_meas[NUM_MOTOR]   = {0.0};   // Angle measured of each motor's rotor, unit 1, 0 - 0x1fff, corresponding to 0 - 360
float  rotor_init_angle[NUM_MOTOR]   = {0.0};
int    rotation_number[NUM_MOTOR]    = {0};
float  torque_meas[NUM_MOTOR]        = {0.0};   // Torque measured from encoder of each motor
float  speed_meas[NUM_MOTOR]         = {0.0};   // The speed of motor shaft
float  desired_speed[NUM_MOTOR]      = {0.0};   // The desired speed of motors
float  speed_command[NUM_MOTOR]      = {0.0};   // Command output of PID controller
float  speed_error[NUM_MOTOR]        = {0.0};   // Error between input and feedback
float  speed_error_former[NUM_MOTOR] = {0.0};   // Error in former time step
float  ang_error[NUM_MOTOR]          = {0.0};
float  ang_command[NUM_MOTOR]        = {0.0};
float  desire_angle[NUM_MOTOR]       = {0.0};
float  command_angle[NUM_MOTOR]      = {0.0};
float  k_p_ang[NUM_MOTOR]            = {0.5};

Teensycomm_struct_t Teensy_comm = {{}, {}, {}, {}};   // For holding data sent to RPi
Jetson_comm_struct_t    RPi_comm    = {{},{}};                   // For holding data received from RPi

static uint8_t  *ptin  = (uint8_t*)(&RPi_comm);
static uint8_t  *ptout = (uint8_t*)(&Teensy_comm);
static int      in_cnt = 0;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus


//
// This function is for doing PID control. In practical using, PI control
// is already enough fast and stable, thus in this function, differential
// part is omitted.
//
void Speed_Control_Loop() {

  // Set the CAN message ID as 0x200
  msg_send.id = 0x200;

  // In this function, a incremental PI controller is used
  for (int i = 0; i < NUM_MOTOR; ++i) {

    ang_error[i] = command_angle[i] - shaft_angle_meas[i];
    desired_speed[i] = constrain(k_p_ang[i] * ang_error[i], -200, 200);  // limits range

    speed_command[i] = (k_p * desired_speed[i] - k_i*speed_meas[i]);//*speed_limit;

    // If the output is positive, then the output shall be smaller than PID_H limit.
    // If the output is negative, then the output shall be larger than PID_L limit.
    if (speed_command[i] < PID_L)
      speed_command[i]= PID_L;
    else if (speed_command[i] > PID_H)///////////////////
      speed_command[i]= PID_H;
    int v = int(speed_command[i]);
    // msg_send.buf is a list of 8 bytes.
    // buf[0] and buf[1] are the high byte and low byte of command sent to motor1
    // buf[2] and buf[3] are the high byte and low byte of command sent to motor2
    // buf[4] and buf[5] are the high byte and low byte of command sent to motor3
    // buf[6] and buf[7] are the high byte and low byte of command sent to motor4
    msg_send.buf[2 * i + 1] = v & 0x00ff;
    msg_send.buf[2 * i] = (v >> 8) & 0x00ff;//////////////
  }
  CAN.write(msg_send); // Write the message on CAN bus
}

//
// Read message from CAN bus
//
void Read_CAN() {
  
  int i = 0;

  // To control 4 motors synchronously, the data from each motor is supposed
  // to be received.
  while (true) {

    // Reading one message
    if (CAN.read(msg_recv)) {

      // The ID of 4 motors are 0x201, 0x202, 0x203 and 0x204.
      // To make sure all the data from each motor is received,
      // a bool list "can_received" is used. When one corresponding
      // data is received, the bool in the list of that motor is turn true.

      int id = msg_recv.id - 0x201; // motor id

      if (!can_received[id]) {

        // The received message data "msg_recv.buf" is a list of 8 bytes.
        // buf[0] and buf[1] are the high byte and low byte of rotor angle
        // buf[2] and buf[3] are the high byte and low byte of rotor speed
        // buf[4] and buf[5] are the high byte and low byte of torque
        // buf[6] and buf[7] are NULL

        int16_t rpm=0, angle=0, torque=0;

        *((uint8_t*)(&rpm)+1) = msg_recv.buf[2];
        *(uint8_t*)(&rpm) = msg_recv.buf[3];
        speed_meas[id] = (float)rpm / DRIVE_RATIO;

        *((uint8_t*)(&angle)+1) = msg_recv.buf[0];
        *(uint8_t*)(&angle) = msg_recv.buf[1];

        angle_calculate((float)angle / 8192.0 * 360.0, id);

        *((uint8_t*)(&torque)+1) = msg_recv.buf[4];
        *(uint8_t*)(&torque) = msg_recv.buf[5];

        torque_meas[id] = 0.00006103515625f*(float)torque;

        can_received[id] = true;
        i++;
      }
    }

    // After all the data of all motors is revceived, turn bool list in all false
    if (i == NUM_MOTOR)
    {
      for (int k = 0; k < NUM_MOTOR; ++k)
        can_received[k] = false;
      break;
    }
  }
}

// Read the initial data from motors
void CAN_init() {
  int i = 0;
  int angle;
  while (true) {
    if (CAN.read(msg_recv)) {
      if (!can_received[msg_recv.id - 0x201]) {

        angle = 0;
        angle |= (int16_t)(unsigned char)msg_recv.buf[0] << 8;
        angle |= (int16_t)(unsigned char)msg_recv.buf[1];

        rotor_angle_meas[msg_recv.id - 0x201] = (float)angle / 8192.0 * 360.0;
        rotor_init_angle[msg_recv.id - 0x201] = rotor_angle_meas[msg_recv.id - 0x201];
        can_received[msg_recv.id - 0x201] = true;
        i++;
      }
    }
    if (i == NUM_MOTOR) {
      for (int k = 0; k < NUM_MOTOR; ++k) {
        can_received[k] = false;
      }
      break;
    }
  }
}


//
// The angle data sent from motor is the rotation angle of the rotor.
// This function is for transfering the rotor angle to shaft angle.
// Inputs are rotor angle, index of motor and speed of motor.
// Results are saved in list "angle_sum".
//
void angle_calculate(float angle, int i) {
  float angle_now;
  
  if (angle - rotor_init_angle[i] >= 0.0)
    angle_now = angle - rotor_init_angle[i];
  else
    angle_now = 360.0 + (angle - rotor_init_angle[i]);

  float angle_former = rotor_angle_meas[i];
  rotor_angle_meas[i] = angle_now; 

  if (angle_now < angle_former && (angle_now - angle_former <-180.0))
    rotation_number[i]++;
  else if (angle_now > angle_former && (angle_now - angle_former >180.0))
    rotation_number[i]--;

  shaft_angle_meas[i] = ((float)rotation_number[i] * 360.0 + angle_now) / DRIVE_RATIO ;

  // // Set the shaft angle between 0 - 360
  // while (shaft_angle_meas[i] > 360.0) {
  //   shaft_angle_meas[i] -= 360.0;
  // }
  // while (shaft_angle_meas[i] < 0.0) {
  //   shaft_angle_meas[i] += 360.0;
  // }

  
}


void setup() {
  // Switch on CAN bus
  CAN.begin();
  CAN.setBaudRate(1000000);
  CAN.setClock(CLK_60MHz);
  CAN_init();

  // Open Serial port in speed 1000000 Baudrate
  Serial.begin(USB_UART_SPEED);
}

void loop() {

  in_cnt = 0;
  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(RPi_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(RPi_comm)) {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Save angle, speed and torque into struct Teensy_comm
    for (int i = 0; i < NUM_MOTOR; i++) {
      Teensy_comm.pos[i]  = shaft_angle_meas[i];////////////////////
      Teensy_comm.vel[i] = speed_meas[i];
      Teensy_comm.cur[i] = torque_meas[i];
    }

    Teensy_comm.timestamps = micros() / 1000000.0;
    // Send data structure Teensy_comm to RPi
    Serial.write(ptout, sizeof(Teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }

  for (int i = 0; i < NUM_MOTOR; i++)
  {
    command_angle[i] = RPi_comm.comd[i];
    k_p_ang[i]=RPi_comm.k[i];
  }
    
  
  // Read data from motors
  Read_CAN();

  // // Do PI angle synchronizing control
  // Angle_Sync_Loop();
  // Do PI speed contol, then sending command to motor through CAN bus
  Speed_Control_Loop();
}
