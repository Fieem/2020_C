//
// Created by wenha on 25-11-6.
//

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define MAX_OUTPUT 60
#define MIN_OUTPUT 0
#define MAX_SPEED 50
#define MIN_SPEED -50
#define INTEGRAL_MAX 100
#define INTEGRAL_MIN 0

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float prev_prev_error;
    float filtered_measure;
} IncrementalPID_Controller;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
} AddressPID_Controller;

extern AddressPID_Controller pid_angle;
extern AddressPID_Controller pid_yaw;
extern AddressPID_Controller pid_gyro_z;
extern AddressPID_Controller pid_speed_left;
extern AddressPID_Controller pid_speed_right;

// 目标速度设置 (定义在main.c，单位: rad/s，范围: 0-20)
extern float target_speed_left;
extern float target_speed_right;

// PID控制输出
extern float target_yaw;        // 目标偏航角 (单位: 度)

/*
 * PID输出值 - 定义在car.c中
 * pwm_left_com:  左电机PWM占空比 (范围: 0-60)
 * pwm_right_com: 右电机PWM占空比 (范围: 0-60)
 * 这些值由pid_loop_speed_update()更新，并由Motor_SetLeft/Right()应用到电机
 */

/* Incremental PID (symmetric API) */
void pid_inc_init(IncrementalPID_Controller *pid, float kp, float ki, float kd);
void pid_inc_reset(IncrementalPID_Controller *pid);
float pid_inc_calculate(IncrementalPID_Controller *pid, float setpoint, float measured_value);

/* Positional PID (symmetric API) */
void pid_pos_init(AddressPID_Controller *pid, float kp, float ki, float kd);
void pid_pos_reset(AddressPID_Controller *pid);
float pid_pos_calculate(AddressPID_Controller *pid, float target, float measure,
                        float integral_min, float integral_max,
                        float output_min, float output_max);

/* Control loops */
// void pid_loop_speed_init(void);
void pid_loop_angle_init(void);
void pid_loop_yaw_init(void);
// void pid_loop_speed_update(void);
void pid_loop_angle_update(void);
void pid_loop_yaw_update(void);

void pid_loop_gyro_z_init(void);
void pid_loop_gyro_z_update(void);

void pid_loop_speed_init(void);
void pid_loop_speed_update(void);

#endif // PID_H



