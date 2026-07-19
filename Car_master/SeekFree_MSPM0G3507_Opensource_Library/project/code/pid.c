#include "pid.h"
//#include "icm42688.h"
#include <math.h>
//#include "encoder.h"
//#include "gray.h"
//#include "inertial_nav.h"
#include "project_globals.h"
#include "motor.h"
#include "adc.h"
#include "icm42688.h"
#include "isr.h"
#include "icm.h"
static float pid_clamp(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

void pid_inc_init(IncrementalPID_Controller *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->filtered_measure = 0.0f;
}

void pid_inc_reset(IncrementalPID_Controller *pid)
{
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->filtered_measure = 0.0f;
}

float pid_inc_calculate(IncrementalPID_Controller *pid, float setpoint, float measured_value)
{
    const float alpha = 0.7f;
    pid->filtered_measure = alpha * measured_value + (1.0f - alpha) * pid->filtered_measure;

    const float error = setpoint - measured_value;
    const float delta_error = error - pid->prev_error;
    const float increment = pid->Kp * delta_error +
                            pid->Ki * error +
                            pid->Kd * (delta_error + pid->prev_prev_error - pid->prev_error);

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    return increment;
}

void pid_pos_init(AddressPID_Controller *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

void pid_pos_reset(AddressPID_Controller *pid)
{
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

float pid_pos_calculate(AddressPID_Controller *pid, float target, float measure,
                        float integral_min, float integral_max,
                        float output_min, float output_max)
{
    const float error = target - measure;
    pid->integral += error;
    pid->integral = pid_clamp(pid->integral, integral_min, integral_max);

    float output = pid->Kp * error +
                   pid->Ki * pid->integral +
                   pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;
    output = pid_clamp(output, output_min, output_max);
    return output;
}

// void pid_loop_speed_init(void)
// {
//     pid_inc_init(&pid_left, 1.1f, 2.2f, 0.0f);
//     pid_inc_init(&pid_right, 1.1f, 2.2f, 0.0f);
// }

void pid_loop_angle_init(void)
{
    pid_pos_init(&pid_angle, 3.0f, 0.0f, 0.125f);
}

void pid_loop_yaw_init(void)
{
    pid_pos_init(&pid_yaw, 5.0f, 0.0f, 0.25f);
}
void pid_loop_gyro_z_init(void)
{
    pid_pos_init(&pid_gyro_z, 0.5f, 0.0f, 0.0f);
}

// ============ 速度环（新） ============
AddressPID_Controller pid_speed_left  = {0};
AddressPID_Controller pid_speed_right = {0};

void pid_loop_speed_init(void)
{
    pid_pos_init(&pid_speed_left,  1.3f, 0.06f, 0.0f);
    pid_pos_init(&pid_speed_right, 1.3f, 0.06f, 0.0f);
}

void pid_loop_speed_update(void)
{
    // 编码器只出正数 → PID 控幅值，方向由 target 符号决定
    float left_mag  = pid_pos_calculate(&pid_speed_left,  fabsf(target_speed_left),  (float)enc_left,
                                         0.0f, 500.0f, 0.0f, 30.0f);
    float right_mag = pid_pos_calculate(&pid_speed_right, fabsf(target_speed_right), (float)enc_right,
                                         0.0f, 500.0f, 0.0f, 30.0f);

    target_pwm_left  = (target_speed_left  >= 0.0f) ? left_mag : -left_mag;
    target_pwm_right = (target_speed_right >= 0.0f) ? right_mag : -right_mag;
}
// void pid_loop_speed_update(void)
// {
//     const float inc_left = pid_inc_calculate(&pid_left, target_speed_left, g_encoder_speed_rads_left);
//     const float inc_right = pid_inc_calculate(&pid_right, target_speed_right, g_encoder_speed_rads_right);

//     speed_left_com += inc_left;
//     speed_right_com += inc_right;

//     speed_left_com = pid_clamp(speed_left_com, MIN_OUTPUT, MAX_OUTPUT);
//     speed_right_com = pid_clamp(speed_right_com, MIN_OUTPUT, MAX_OUTPUT);

//     Motor_SetLeft(0); 
//     Motor_SetRight(0);
// }

/**
 * @brief 角度环/循迹环 PID 更新
 *
 * 根据赛道中心偏差 line_error_filtered 计算转向修正量 steer，
 * 再将修正量分配到左右轮目标速度上，使小车回到赛道中心。
 *
 * 控制逻辑：
 * 1. speed_set 为基础前进速度。
 * 2. track_error 表示当前循迹偏差，目标值为 0。
 * 3. 通过位置式 PID 计算转向量 steer。
 * 4. 右轮加 steer、左轮减 steer，形成转向差速。
 * 5. 如果触发特殊转向标志 flag_left 或 flag_right，则直接进入强制转向模式。
 */
void pid_loop_angle_update(void)
{
    speed_set = 17.0f;
    float steer = pid_pos_calculate(&pid_angle, 0.0f, line_error_filtered,
                                     -100.0f, 100.0f, -15.0f, 15.0f);
    target_speed_left  = speed_set + steer;
    target_speed_right = speed_set - steer;
}

/**
 * @brief 偏航环 PID 更新
 *
 * 使用目标偏航角 target_yaw 和当前偏航角 gyro_yaw 计算转向输出 steering_out，
 * 再根据偏航误差动态调整左右轮目标速度。
 *
 * 控制逻辑：
 * 1. 计算当前偏航误差 yaw_error。
 * 2. 通过位置式 PID 计算转向修正量 steering_out。
 * 3. 偏航误差越大，基础速度 speed_set 越小，避免转弯时速度过快。
 * 4. 将 steering_out 叠加到左右轮速度上，实现小车转向。
 */
void pid_loop_yaw_update(void)
{
    speed_set = 17.0f;
    float steer = pid_pos_calculate(&pid_yaw, target_yaw, Yaw_TotalAngle,
                                     -100.0f, 100.0f, -15.0f, 15.0f);
    target_speed_left  = speed_set + steer;
    target_speed_right = speed_set - steer;
}

void pid_loop_gyro_z_update(void)
{
    const float gyro_z_output = pid_pos_calculate(&pid_gyro_z, target_gyro_z, Yaw_g,
                                                  -INTEGRAL_MAX, INTEGRAL_MAX,
                                                  -30.0f, 30.0f);                                         
    // 根据陀螺仪Z轴误差调整左右轮速度，保持小车稳定
    target_speed_right = pid_clamp(speed_set - gyro_z_output, MIN_SPEED, MAX_SPEED);
    target_speed_left  = pid_clamp(speed_set + gyro_z_output, MIN_SPEED, MAX_SPEED);
    static float smooth_left  = 0.0f;                                                                                                                                                                               
    static float smooth_right = 0.0f;                                                                                                                                                                               
    const float alpha = 0.3f;                                                                                                                                                                                       

    float raw_left  = pid_clamp(speed_set + gyro_z_output, MIN_SPEED, MAX_SPEED);                                                                                                                                   
    float raw_right = pid_clamp(speed_set - gyro_z_output, MIN_SPEED, MAX_SPEED);                                                                                                                                   

    smooth_left  = alpha * raw_left  + (1.0f - alpha) * smooth_left;                                                                                                                                                
    smooth_right = alpha * raw_right + (1.0f - alpha) * smooth_right;                                                                                                                                               

    target_speed_left  = smooth_left;                                                                                                                                                                               
    target_speed_right = smooth_right; 
}    