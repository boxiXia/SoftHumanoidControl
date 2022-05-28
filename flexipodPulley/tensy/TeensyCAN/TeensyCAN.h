#ifndef __TeensyCAN_H
#define __TeensyCAN_H

// Defines
#define NUM_MOTOR             1                 // Number of ESCs
#define USB_UART_SPEED     1000000            // Baudrate of the teeensy USB serial link
//
// if speed command higher than 32768, the motor rotates clockwise
// if speed command lower than 32768, the motor rotates counter-clockwise
//
#define DirectionBoundary  32768.0   // 32768(dec) = 0x8000(hex)
#define maxBoundary        65535.0   // 0xffff, command upper limit
#define PID_H              12000.0   // PID controller output upper limit, corresponding to 10A
#define PID_L              -12000.0  // PID controller output lower limit,
#define PID_ANG_H          60.0
#define PID_ANG_L          0.0
#define DRIVE_RATIO        19.20320856  // Drive ratio of motor gear box
#define k_p                50.0       // Proportional parameter
#define k_i                100.0       // Integral parameter
//#define k_p_ang            0.5

// Teensy->host communication data structure
typedef struct {
  float    pos[NUM_MOTOR];      // Motors rotation angle
  float    vel[NUM_MOTOR];     // Motors rad/s
  float    cur[NUM_MOTOR];     // Motors torque
  float    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
// sizeof(RPi_comm)=64 to match USB 1.0 buffer size
typedef struct {
  float    comd[NUM_MOTOR];           // Desired Speed, rad/s
  float    k[NUM_MOTOR];
} Jetson_comm_struct_t;
#endif
