#ifndef SERVO_H
#define SERVO_H

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

//-------------------------------------------------------------------------------------------------------------------
// 舵机驱动相关定义
//-------------------------------------------------------------------------------------------------------------------

// 舵机 PWM 通道（根据实际硬件连接修改）
#define SERVO_PWM_CHANNEL   (PWM_TIM_A0_CH2_B4)

// 舵机频率固定 50Hz（周期 20ms）
#define SERVO_FREQ_HZ       (50U)

// PWM_DUTY_MAX = 10000 对应的脉宽范围
// 500μs  → 250   (-90°)
// 1500μs → 750   ( 0° 中位)
// 2500μs → 1250  (+90°)
#define SERVO_DUTY_MIN      ( 250)   // 0.5ms 脉宽
#define SERVO_DUTY_MID      ( 750)   // 1.5ms 脉宽（中位）
#define SERVO_DUTY_MAX      (1250)   // 2.5ms 脉宽
#define SERVO_DUTY_PER_DEG  (1000.0f / 180.0f)

// 角度范围
#define SERVO_ANGLE_MIN     (-13.0f)
#define SERVO_ANGLE_MAX     (+13.0f)

// 实车直行时实测的中位 PWM，后续只需调整这个值
#define SERVO_CENTER_DUTY   (400U)

// 舵机转向 PD 参数
#define SERVO_KP            (2.0f)     // 比例增益：线偏差 → 舵角
#define SERVO_KD            (0.0f)      // 微分增益：抑制过冲摆动

//-------------------------------------------------------------------------------------------------------------------
// 函数声明
//-------------------------------------------------------------------------------------------------------------------

/**
 * @brief 初始化舵机
 * @return void
 */
void Servo_Init(void);

/**
 * @brief 设置舵机角度
 * @param angle 目标角度，范围 -90° ~ +90°
 * @return void
 */
void Servo_SetAngle(float angle);

/**
 * @brief 设置舵机原始占空比
 * @param duty 占空比值，范围 SERVO_DUTY_MIN ~ SERVO_DUTY_MAX
 * @return void
 */
void Servo_SetDuty(uint32_t duty);

/**
 * @brief 舵机转向更新（PD 控制，替代 LQR 差速）
 *        有线时 line_error → 舵角，丢线时保持上一帧角度
 * @return void
 */
void servo_steering_update(void);

#endif // SERVO_H
