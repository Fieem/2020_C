#include "servo.h"
#include "zf_driver_pwm.h"
#include "adc.h"
#include <math.h>

//-------------------------------------------------------------------------------------------------------------------
// 舵机驱动模块
//
// 使用 PWM 控制舵机，频率 50Hz（周期 20ms）。
// 标准舵机通过 0.5ms~2.5ms 的脉宽控制角度：
//   0.5ms → -90°
//   1.5ms →   0°（中位）
//   2.5ms → +90°
//
// 占空比计算（PWM_DUTY_MAX = 10000，50Hz，20ms 周期）：
//   duty = 250 + (angle + 90) / 180 * 1000
//
// 使用示例：
//   Servo_Init();             // 初始化舵机
//   Servo_SetAngle(45.0f);    // 转到 +45°
//   Servo_SetDuty(750);       // 转到 0° 中位
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Servo_Init();
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void)
{
    // 50Hz PWM，初始占空比为中位（含安装偏置补偿）
    uint32_t mid_duty = (uint32_t)(250.0f + (SERVO_MID_OFFSET + 90.0f) / 180.0f * 1000.0f + 0.5f);
    pwm_init(SERVO_PWM_CHANNEL, SERVO_FREQ_HZ, mid_duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机角度
// 参数说明     angle           目标角度，范围 -90° ~ +90°
// 返回参数     void
// 使用示例     Servo_SetAngle(30.0f);
//-------------------------------------------------------------------------------------------------------------------
void Servo_SetAngle(float angle)
{
    uint32_t duty;

    // 限幅
    if (angle < SERVO_ANGLE_MIN) {
        angle = SERVO_ANGLE_MIN;
    }
    if (angle > SERVO_ANGLE_MAX) {
        angle = SERVO_ANGLE_MAX;
    }

    // 叠加安装偏置，映射到 duty
    float compensated = angle + SERVO_MID_OFFSET;
    duty = (uint32_t)(250.0f + (compensated + 90.0f) / 180.0f * 1000.0f + 0.5f);

    pwm_set_duty(SERVO_PWM_CHANNEL, duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机原始占空比
// 参数说明     duty            占空比值，范围 250 ~ 1250
// 返回参数     void
// 使用示例     Servo_SetDuty(750);
//-------------------------------------------------------------------------------------------------------------------
void Servo_SetDuty(uint32_t duty)
{
    if (duty < SERVO_DUTY_MIN) {
        duty = SERVO_DUTY_MIN;
    }
    if (duty > SERVO_DUTY_MAX) {
        duty = SERVO_DUTY_MAX;
    }

    pwm_set_duty(SERVO_PWM_CHANNEL, duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机转向更新（PD 控制，替代 LQR 差速）
//              line_error_filtered → 舵机角度
//              有线时 PD 反馈，丢线时保持上一帧角度
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void servo_steering_update(void)
{
    static float prev_line_error = 0.0f;
    static float hold_angle      = 0.0f;

    float angle;

    if (!line_lost) {
        // PD 控制：负号使车回正（线偏左 → 左打舵）
        float d_error = line_error_filtered - prev_line_error;
        prev_line_error = line_error_filtered;
        angle = -(SERVO_KP * line_error_filtered + SERVO_KD * d_error);
        hold_angle = angle;
    } else {
        // 丢线：保持当前角度，不摆舵
        angle = hold_angle;
    }

    Servo_SetAngle(angle);
}
