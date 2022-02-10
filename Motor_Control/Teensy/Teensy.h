#ifndef __Teensy_H
#define __Teensy_H

// Defines
#define MOTOR_NUM                 12                 // Number of ESCs
#define USB_UART_SPEED            4000000            // Baudrate of the teeensy USB serial link
#define REDUCTION_RATIO           8.0f  // Drive ratio of motor gear box

// Teensy->host communication data structure
typedef struct {
  float    joint_pos[MOTOR_NUM];      // Motors rotation angle
  float    joint_vel[MOTOR_NUM];     // Motors rad/s
  float    joint_cur[MOTOR_NUM];     // Motors current
  int16_t    foot_force[4];
  float    timestamps;
} Teensycomm_struct_t;

// Host->teensy communication data structure
typedef struct {
  float    comd[MOTOR_NUM];            // Desired position, rad
  float    motor_mode;
} Jetson_comm_struct_t;



/* clamp a value data between lower and upper */
template <typename T>
inline void clampInplace(T& n, const T& lower, const T& upper) {
	//assert(lower < upper);
	if (n > upper) { n = upper; }
	else if (n < lower) { n = lower; }
}

/* clamp a value data between lower and upper,
assume periodic between lower and upper */

template <typename T>
void clampPeroidicInplace(T& n, const T& lower, const T& upper) {
	//assert(lower < upper);
	if (n > upper) { n = fmod(n - upper, upper - lower) + lower; }
	else if (n < lower) { n = fmod(n - lower, upper - lower) + upper; }
}

#endif
