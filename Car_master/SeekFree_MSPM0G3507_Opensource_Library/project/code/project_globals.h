#ifndef PROJECT_GLOBALS_H
#define PROJECT_GLOBALS_H

#include <stdint.h>
#include "motor.h"
#include "pid.h"

extern uint8_t task_number;

extern uint8_t start_flag;
extern float target_speed_left;
extern float target_speed_right;
extern float target_gyro_z;
extern float yaw_rate_z;

extern float line_error_filtered;

extern float speed_set;

extern AddressPID_Controller pid_angle;
extern AddressPID_Controller pid_yaw;
extern AddressPID_Controller pid_gyro_z;
extern float target_yaw;

extern uint8_t adc_xunhuan;

extern volatile uint32_t g_timestamp_ms;

extern float battery_voltage;
extern float distance_accum;
extern int32_t enc_left;
extern int32_t enc_right;
extern int32_t enc_left_acc;
extern int32_t enc_right_acc;

extern float target_pwm_left;
extern float target_pwm_right;
#endif /* PROJECT_GLOBALS_H */
