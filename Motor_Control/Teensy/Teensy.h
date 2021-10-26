#ifndef __Teensy_H
#define __Teensy_H

// Defines
#define MOTOR_NUM                 12                 // Number of ESCs
#define USB_UART_SPEED            2000000            // Baudrate of the teeensy USB serial link
#define REDUCTION_RATIO           8.0  // Drive ratio of motor gear box

// Teensy->host communication data structure
typedef struct {
  float    joint_pos[MOTOR_NUM];      // Motors rotation angle
  float    joint_vel[MOTOR_NUM];     // Motors rad/s
  float    joint_cur[MOTOR_NUM];     // Motors current
  float    foot_force[4];
  float    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  float    comd[MOTOR_NUM];            // Desired position, rad
  float    motor_mode[1];
} Jetson_comm_struct_t;

#endif
