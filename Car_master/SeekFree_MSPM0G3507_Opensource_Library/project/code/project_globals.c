#include "project_globals.h"
#include "pid.h"
#include "motor.h"
#include "adc.h"

uint8_t task_number = 0;

uint8_t start_flag = 0;
float target_speed_left = 0.0f;
float target_speed_right = 0.0f;
float target_gyro_z = 0.0f;
float yaw_rate_z = 0.0f;

float line_error_filtered = 0.0f;

// float speed_set = 0.0f;

AddressPID_Controller pid_angle = {0};
AddressPID_Controller pid_yaw = {0};
AddressPID_Controller pid_gyro_z = {0};
float target_yaw = 0.0f;

uint8_t adc_xunhuan = 1;

volatile uint32_t g_timestamp_ms = 0;

float battery_voltage = 0.0f;
float distance_accum = 0.0f;

int32_t enc_left  = 0;
int32_t enc_right = 0;

int32_t enc_left_acc  = 0;
int32_t enc_right_acc = 0;

float target_pwm_left = 0.0f;
float target_pwm_right = 0.0f;

float total_pulses = 68559.0f;      // 全程编码器脉冲总数，time_control() 计算用
float servo_accum_angle = 0.0f;     // 舵机累加角度，增量式 PD 使用