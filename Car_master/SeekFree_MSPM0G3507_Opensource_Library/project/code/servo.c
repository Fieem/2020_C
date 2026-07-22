#include "servo.h"
#include "zf_driver_pwm.h"
#include "adc.h"
#include "project_globals.h"
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
// 角度映射：以实测中位 PWM 为基准叠加角度增量。
//   duty = SERVO_CENTER_DUTY + angle * SERVO_DUTY_PER_DEG
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
    // 50Hz PWM，初始输出实测中位
    pwm_init(SERVO_PWM_CHANNEL, SERVO_FREQ_HZ, SERVO_CENTER_DUTY);
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

    // 以实测中位 PWM 为基准，叠加目标角度增量
    float duty_f = (float)SERVO_CENTER_DUTY + angle * SERVO_DUTY_PER_DEG;

    if (duty_f < (float)SERVO_DUTY_MIN) {
        duty_f = (float)SERVO_DUTY_MIN;
    }
    if (duty_f > (float)SERVO_DUTY_MAX) {
        duty_f = (float)SERVO_DUTY_MAX;
    }

    duty = (uint32_t)(duty_f + 0.5f);

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
    static float prev_error    = 0.0f;
    static float smooth_delta  = 0.0f;      // 低通滤波后的增量
    const  float delta_alpha   = 0.3f;      // 滤波系数：越小越平滑

    if (!line_lost) {
        // 增量式 PD：误差 → 舵角增量累加，误差消失时角度保持不动
        float d_error = line_error_filtered - prev_error;
        prev_error = line_error_filtered;
        float raw_delta = (SERVO_KP * line_error_filtered + SERVO_KD * d_error);
        smooth_delta = delta_alpha * raw_delta + (1.0f - delta_alpha) * smooth_delta;
        servo_accum_angle += smooth_delta;
        // 限幅
        if (servo_accum_angle < SERVO_ANGLE_MIN) servo_accum_angle = SERVO_ANGLE_MIN;
        if (servo_accum_angle > SERVO_ANGLE_MAX) servo_accum_angle = SERVO_ANGLE_MAX;
    } else {
        smooth_delta = 0.0f;    // 丢线时重置滤波
    }
    // 丢线：servo_accum_angle 保持不动

    Servo_SetAngle(servo_accum_angle);
}
