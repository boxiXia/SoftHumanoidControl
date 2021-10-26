#include <FlexCAN_T4.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include "MadgwickAHRS.h"
#include "Teensy.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_F; // CAN bus for upper body motors
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN_B; // CAN bus for lower body motors


// Globals
float joint_pos_desired[MOTOR_NUM]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   // desired joint(motor) position [rad]
float joint_speed_desired[MOTOR_NUM]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float rotor_pos[MOTOR_NUM];
float rotor_pos_prev[MOTOR_NUM];
int   r_num[MOTOR_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float motor_mode;

float time_now;
float time_former;

// Motor shaft position [rad]
float joint_pos[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Motor shaft velocity [rad/s]
float joint_vel[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Motor current [A]
float joint_cur[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float joint_upper_limit[MOTOR_NUM] = { 2.5132742,  1.4279966,  1.5707964,  2.5132742,  1.4279966,
         1.5707964,  0.7853982, 0.7853982,  1.5707964,  0.7853982,
         1.9634954,  1.5707964};
float joint_lower_limit[MOTOR_NUM] = {-2.5132742, -1.4279966, -1.5707964, -2.5132742, -1.4279966,
        -1.5707964, -0.7853982, -1.9634954, -1.5707964, -0.7853982,
         -0.7853982, -1.5707964};

float delta_t; // loop time difference
Teensycomm_struct_t   teensy_comm = {{}, {}, {}, {}, {}, {}, {}, {}};   // For holding data sent to Jetson
Jetson_comm_struct_t  jetson_comm    = {{}};                   // For holding data received from Jetson

static uint8_t  *ptin  = (uint8_t*)(&jetson_comm);
static uint8_t  *ptout = (uint8_t*)(&teensy_comm);
static int      in_cnt = 0;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus

// lookup table to relate motor index i to the CAN bus and its motor CAN address
// 0 stands for CAN_F, 1 stands for CAN_B
bool joint_can_lane [MOTOR_NUM] = { 0,0,0,0,0,0,
                                    1,1,1,1,1,1};
// Motors' id from 0x141 to 0x146
uint16_t joint_can_addr[MOTOR_NUM] = {0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 
                                      0x141, 0x142, 0x143, 0x144, 0x145, 0x146};
/****************** IMU ***************************************/
Adafruit_LSM6DS sox;
Adafruit_LIS3MDL lis;

sensors_event_t acc;
sensors_event_t gyr;
sensors_event_t temp;
sensors_event_t mag;

float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0;
float gyro_z_offset = 0.0;

//float acc_transform[3][3] = {{0.9991416, 0.0, 0.0}, {0.0, 0.9949671162, 0.0}, {0.0, 0.0, 0.98455696}};
float acc_transform[3][3] = 
{{ 0.99441067,-0.01002755,-0.01367619},
 {-0.00493764, 1.00115685, 0.01292437},
 { 0.00023209,-0.01873136, 1.0017535 }};
float acc_offset[3] = {-0.0003851 , 0.2268528 ,-0.16885345};


const float magn_ellipsoid_center[3] = {-26.1968, 0.416948, -25.6187};
const float magn_ellipsoid_transform[3][3] = {{0.878318, 0.0501339, -0.0246819}, {0.0501339, 0.911703, -0.0155113}, {-0.0246819, -0.0155113, 0.985244}};


float accel[3]={0};  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_b[3]={0};
float magnetom[3]={0};
float magnetom_tmp[3]={0};
float gyro[3]={0};
float foot_force[4];

//............foot sensor
const int pin_left_forefoot = 16;
const int pin_left_backheel = 17;
const int pin_right_forefoot = 20;
const int pin_right_backheel = 21;

bool left_touch = 0;
bool right_touch = 0;
const int force_threshold = 128;
//................

Quaternion qua;
EulerAngles eul;

void read_sensors() {
  sox.getEvent(&acc, &gyr, &temp);
  lis.getEvent(&mag);
  accel[0] = -acc.acceleration.x;
  accel[1] = -acc.acceleration.y;
  accel[2] = -acc.acceleration.z;

  magnetom[0] = mag.magnetic.x;
  magnetom[1] = mag.magnetic.y;
  magnetom[2] = mag.magnetic.z;

  gyro[0] = gyr.gyro.x;
  gyro[1] = gyr.gyro.y;
  gyro[2] = gyr.gyro.z;
}

void sensor_init() {
  for (int i = 0; i < GYRO_CALIBRATION_LOOP_NUM; ++i) {
    sox.getEvent(&acc, &gyr, &temp);
    gyro_x_offset += gyr.gyro.x;
    gyro_y_offset += gyr.gyro.y;
    gyro_z_offset += gyr.gyro.z;
  }
  gyro_x_offset /= GYRO_CALIBRATION_LOOP_NUM;
  gyro_y_offset /= GYRO_CALIBRATION_LOOP_NUM;
  gyro_z_offset /= GYRO_CALIBRATION_LOOP_NUM;
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel_b[0] = accel[0] - acc_offset[0];
    accel_b[1] = accel[1] - acc_offset[1];
    accel_b[2] = accel[2] - acc_offset[2];
    Matrix_Vector_Multiply(acc_transform, accel_b, accel);   
    
    // Compensate magnetometer error
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

    // Compensate gyroscope error
    gyro[0] -= gyro_x_offset;
    gyro[1] -= gyro_y_offset;
    gyro[2] -= gyro_z_offset;
}
/*********************************************************/

void Motor_Init() {

  // Motor position initial CAN bus command
  // All motors rotate to position 0 rad
  msg_send.buf[0] = 0xA3; //CAN bus position command ID
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = 0x00;
  msg_send.buf[7] = 0x00;

  // Motors' IDs range from 0x141 to 0x146.
  // CAN_F is connected to upper body motors(id: 0x141 to 0x146)
  // CAN_B is connected to lower body motors(id: 0x141 to 0x146)
  for (int i = 0; i < MOTOR_NUM / 2; ++i) {
      msg_send.id = joint_can_addr[i];
      CAN_F.write(msg_send);
      while (true) {
        if (CAN_F.read(msg_recv)) {
          processMotorData(i);
          break;
        }
      }
      msg_send.id = joint_can_addr[i + MOTOR_NUM / 2];
      CAN_B.write(msg_send);
      while (true) {
        if (CAN_B.read(msg_recv)) {
          processMotorData(i + MOTOR_NUM / 2);
          break;
        }
      }
  }
  delay(400);
}

void processMotorData(int id) {
  // Receiving motor angle
  // Transfer hex number to rad
  int rotor_pos_raw = 0; // Rotor position before devided by gear reduction
  *(uint8_t *)(&rotor_pos_raw) = msg_recv.buf[6];
  *((uint8_t *)(&rotor_pos_raw)+1) = msg_recv.buf[7];
//TODO CHECK
  rotor_pos[id] = (float)rotor_pos_raw / 65535.0 * 2 * PI; // 65535(0xFFFF) refers to 2PI
  if (rotor_pos[id] - rotor_pos_prev[id] < -PI)
    r_num[id] += 1;
  else if (rotor_pos[id] - rotor_pos_prev[id] > PI)
    r_num[id] -= 1;

  // Calculate shaft angular position [rad]
  rotor_pos_prev[id] = rotor_pos[id];
  joint_pos[id] = (rotor_pos [id]+ r_num[id] * 2 * PI) / REDUCTION_RATIO;

  // Calculate shaft velocity [rad/s]
  int rotor_vel_raw = 0;
  *(uint8_t *)(&rotor_vel_raw) = msg_recv.buf[4];
  *((uint8_t *)(&rotor_vel_raw)+1) = msg_recv.buf[5];
  joint_vel[id] = (float)rotor_vel_raw * PI / (180.0 * REDUCTION_RATIO);

  // Calculate motor's current [A], -33A ~ 33A
  int cur_raw = 0;
  *(uint8_t *)(&cur_raw) = msg_recv.buf[2];
  *((uint8_t *)(&cur_raw)+1) = msg_recv.buf[3];
  joint_cur[id] = (float)cur_raw * 33.0 / 2048.0; // 2048 refers to 33A
}

void Angle_Control_Loop(int motor_id, float pos_command, float motor_mode) {
  // Convert motor shaft angle command [rad] to rotor angle command [degree]
  pos_command = pos_command * 180.0 * REDUCTION_RATIO / PI;
  // see motor manual p12 (0xA3)
  int32_t pos = (int32_t)round(pos_command / 0.01);
  
  int pos_limit = 10;
  // see motor manual p12 (0xA3)
  int32_t poslimit = (int32_t)round(pos_limit / 0.01);
  // Set the CAN message ID as 0x200
  if (motor_mode==0){ 
    msg_send.id = joint_can_addr[motor_id];
    msg_send.buf[0] = 0xA4; // Position control mode
    msg_send.buf[1] = 0x00;
    msg_send.buf[2] = *(uint8_t*)(&poslimit);
    msg_send.buf[3] = *((uint8_t*)(&poslimit)+1);
    msg_send.buf[4] = *(uint8_t*)(&pos);
    msg_send.buf[5] = *((uint8_t*)(&pos)+1);
    msg_send.buf[6] = *((uint8_t*)(&pos)+2);;
    msg_send.buf[7] = *((uint8_t*)(&pos)+3);;
  }
  else{
    msg_send.id = joint_can_addr[motor_id];
    msg_send.buf[0] = 0xA2; // Speed control mode
    msg_send.buf[1] = 0x00;
    msg_send.buf[2] = 0x00;
    msg_send.buf[3] = 0x00;
    msg_send.buf[4] = *(uint8_t*)(&pos);
    msg_send.buf[5] = *((uint8_t*)(&pos)+1);
    msg_send.buf[6] = *((uint8_t*)(&pos)+2);;
    msg_send.buf[7] = *((uint8_t*)(&pos)+3);;
  }


  if (joint_can_lane[motor_id]==0) {
    CAN_F.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (CAN_F.read(msg_recv)) {
        processMotorData(motor_id);
        break;
      }
    }
  }
    
  else {
    CAN_B.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (CAN_B.read(msg_recv)) {
        processMotorData(motor_id);
        break;
      }
    }
  }
}

void Jetson_Teensy () {
  in_cnt = 0;


  // Read all incoming bytes available until incoming structure is complete
  while ((Serial.available() > 0) && (in_cnt < (int)sizeof(jetson_comm)))
    ptin[in_cnt++] = Serial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(jetson_comm)) {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Save angle, speed and torque into struct teensy_comm
    for (int i = 0; i < MOTOR_NUM; i++) {
      teensy_comm.joint_pos[i]  = joint_pos[i];
      teensy_comm.joint_vel[i] = joint_vel[i];
      teensy_comm.joint_cur[i] = joint_cur[i];
    }
    //........
    // teensy_comm.joint_pos[7]  = joint_pos[7]-3.1415926/12;
    // teensy_comm.joint_pos[10]  = joint_pos[10]+3.1415926/12;
    //..............
    // Read data from IMU(MPU9250)
   // Save acceleration (m/s^2) of IMU into struct teensy_comm
    teensy_comm.acc[0] = accel[0];
    teensy_comm.acc[1] = accel[1];
    teensy_comm.acc[2] = accel[2];

    // Save gyroscope (rad/s) of IMU into struct teensy_comm
    teensy_comm.gyr[0] = gyro[0] / 180.0 * PI;
    teensy_comm.gyr[1] = gyro[1] / 180.0 * PI;
    teensy_comm.gyr[2] = gyro[2] / 180.0 * PI;

    teensy_comm.mag[0] = magnetom[0];
    teensy_comm.mag[1] = magnetom[1];
    teensy_comm.mag[2] = magnetom[2];

    teensy_comm.euler[0] = 0;//eul.roll_e;
    teensy_comm.euler[1] = 0;//eul.pitch_e;
    teensy_comm.euler[2] = 0;//eul.yaw_e;

    teensy_comm.timestamps = time_now / 1000000.0;

    //...............foot sensor
    teensy_comm.foot_force[0]  = float(analogRead(pin_left_forefoot))/1023.0*5.0; //pin16:v1
    teensy_comm.foot_force[1] = float(analogRead(pin_left_backheel))/1023.0*5.0; //pin21:v4
    teensy_comm.foot_force[2] = float(analogRead(pin_right_forefoot))/1023.0*5.0;//pin17:v2
    teensy_comm.foot_force[3]= float(analogRead(pin_right_backheel))/1023.0*5.0;//pin20:v3
    
    //...............
    // Send data structure teensy_comm to Jetson
    Serial.write(ptout, sizeof(teensy_comm));

    // Force immediate transmission
    Serial.send_now();
  }
}



void setup() {
  // Switch on CAN bus
  Serial.begin(USB_UART_SPEED);

  pinMode(13, OUTPUT);  
  digitalWrite(13, HIGH);


  // sox.begin_I2C();
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS );
  // sox.setAccelDataRate(LSM6DS_RATE_52_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_52_HZ);

  lis.begin_I2C();          // hardware I2C mode, can pass in address & alt Wire
  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setDataRate(LIS3MDL_DATARATE_40_HZ);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis.setIntThreshold(500);
  lis.configInterrupt(false, false, true, true, false, true); // enabled!
  
  //sensor_init();
  digitalWrite(13, LOW);

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

void loop() {
  
  //read_sensors();
  
  Jetson_Teensy ();
  
  //compensate_sensor_errors();
  
  time_now = (float)micros();
  delta_t = (time_now - time_former) / 1000000.0;
  time_former = time_now;

  // MadgwickQuaternionUpdate(accel[0], accel[1], accel[2],
  //                          gyro[0], gyro[1], gyro[2],
  //                          magnetom[0], magnetom[1], magnetom[2], delta_t);
  // qua.w = q[0];
  // qua.x = q[1];
  // qua.y = q[2];
  // qua.z = q[3];
  // eul = ToEulerAngles(qua);
  // eul.yaw_e += 0.22; // 0.22 rad is the Magnetic Declination in New York
  // if (eul.yaw_e > PI) {
  //   eul.yaw_e -= 2 * PI;
  // }

  
  // for (int i = 0; i < MOTOR_NUM; ++i) {
  //   joint_pos_desired[i] = jetson_comm.comd[i];
  //   if (joint_pos_desired[i] > joint_upper_limit[i])
  //     joint_pos_desired[i] = joint_upper_limit[i];
  //   else if (joint_pos_desired[i] < joint_lower_limit[i])
  //     joint_pos_desired[i] = joint_lower_limit[i];
  // }
  
  // for (int i = 0; i < MOTOR_NUM / 2; ++i){
  //     Angle_Control_Loop(i, joint_pos_desired[i]);
  //     int j = i + MOTOR_NUM / 2;
  //     Angle_Control_Loop(j, joint_pos_desired[j]);
  // }
  motor_mode=jetson_comm.motor_mode[0];
  if(motor_mode==0){
    for (int i = 0; i < MOTOR_NUM; ++i) {
      joint_pos_desired[i] = jetson_comm.comd[i];
    }
//...............
  //joint_pos_desired[1] = jetson_comm.comd[1]+3.1415926/12;
  //joint_pos_desired[2] = jetson_comm.comd[2]-3.1415926/3;
//..................
    for (int i = 0; i < MOTOR_NUM; ++i) {
      if (joint_pos_desired[i] > joint_upper_limit[i])
        joint_pos_desired[i] = joint_upper_limit[i];
      else if (joint_pos_desired[i] < joint_lower_limit[i])
        joint_pos_desired[i] = joint_lower_limit[i];
    }
  }
  else if(motor_mode==1){
    for (int i = 0; i < MOTOR_NUM; ++i) {
      joint_pos_desired[i] = jetson_comm.comd[i];

    }
//...................
  //joint_pos_desired[1] = jetson_comm.comd[1]+3.1415926/12;
  //joint_pos_desired[2] = jetson_comm.comd[2]-3.1415926/3;
//......................
    for (int i = 0; i < MOTOR_NUM; ++i) {
      if (teensy_comm.joint_pos[i] > joint_upper_limit[i]){
        if (joint_pos_desired[i]>0){
          joint_pos_desired[i] = 0;
        }
      }
      else if (teensy_comm.joint_pos[i] < joint_lower_limit[i])
        if (joint_pos_desired[i]<0){
          joint_pos_desired[i] = 0;
        }
    }
  }

  
  for (int i = 0; i < MOTOR_NUM / 2; ++i){
      Angle_Control_Loop(i, joint_pos_desired[i],motor_mode);
      int j = i + MOTOR_NUM / 2;
      Angle_Control_Loop(j, joint_pos_desired[j],motor_mode);
  }

}
