#include "ti_msp_dl_config.h"
#include "motor.h"
#include "zf_driver_pwm.h"
#include "project_globals.h"

int32_t measure_pwm = 0;
float compensated_left = 0.0f;
float compensated_right = 0.0f;
int32_t current_left_pwm = 0;
int32_t current_right_pwm = 0;
void Motor_Init(void)
{
    // 初始化左电机 PWM 通道
    pwm_init(PWM_LEFT_CHANNEL, 17000, 0); // 初始占空比为 0
    // 初始化右电机 PWM 通道
    pwm_init(PWM_RIGHT_CHANNEL, 17000, 0);   // 初始占空比为 0
    gpio_init(PWM_LEFT_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL); // 设置左电机方向引脚为输出，初始状态为低
    gpio_init(PWM_RIGHT_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL); // 设置右电机方向引脚为输出，初始状态为低
}

void Motor_SetLeft(int32_t PWM)
{
    measure_pwm = PWM;
    uint32_t pwm_duty = 0;
    
    if (PWM >= 0)
    {
        // 正转逻辑
        pwm_duty = (PWM > PWM_MAX_VALUE) ? PWM_MAX_VALUE : (uint32_t)PWM;
        gpio_set_level(PWM_LEFT_DIR_PIN, 1); // 设置方向引脚为高，表示正转
    }
    else
    {
        // 反转逻辑：将负数转为正数作为占空比
        pwm_duty = (-PWM > PWM_MAX_VALUE) ? PWM_MAX_VALUE : (uint32_t)(-PWM);
        gpio_set_level(PWM_LEFT_DIR_PIN, 0); // 设置方向引脚为低，表示反转
    }

    // 调用驱动层 PWM 函数设置占空比
    // 需要将占空比转换为驱动函数期望的范围（0-100）
    pwm_set_duty(PWM_LEFT_CHANNEL, pwm_duty);
}

void Motor_SetRight(int32_t PWM)
{
    measure_pwm = PWM;
    uint32_t pwm_duty = 0;

    if (PWM >= 0)
    {
        // 正转逻辑
        pwm_duty = (PWM > PWM_MAX_VALUE) ? PWM_MAX_VALUE : (uint32_t)PWM;
        gpio_set_level(PWM_RIGHT_DIR_PIN, 1); // 设置方向引脚为高，表示正转
    }
    else
    {
        // 反转逻辑：将负数转为正数作为占空比
        pwm_duty = (-PWM > PWM_MAX_VALUE) ? PWM_MAX_VALUE : (uint32_t)(-PWM);
        gpio_set_level(PWM_RIGHT_DIR_PIN, 0); // 设置方向引脚为低，表示反转
    }

    // 调用驱动层 PWM 函数设置占空比
    // 需要将占空比转换为驱动函数期望的范围（0-100）
    pwm_set_duty(PWM_RIGHT_CHANNEL, pwm_duty);
}

// void Motor_Control(void)
// {
//     const uint8_t d_pwm = 3; // 每次调整的最大 PWM 步长，单位: 占空比百分比
//     static int32_t last_left_pwm = 0;
//     static int32_t last_right_pwm = 0;

//     int32_t current_left_pwm;
//     int32_t current_right_pwm;

//     target_speed_left *= 11.095f / battery_voltage;   // 电压补偿
//     target_speed_right *= 11.095f / battery_voltage;  // 电压补偿

//     current_left_pwm = (int32_t)target_speed_left;
//     current_right_pwm = (int32_t)target_speed_right;

//     // if (current_left_pwm > last_left_pwm + d_pwm)
//     // {
//     //     current_left_pwm = last_left_pwm + d_pwm;
//     // }
//     // else if (current_left_pwm < last_left_pwm - d_pwm)
//     // {
//     //     current_left_pwm = last_left_pwm - d_pwm;
//     // }

//     // if (current_right_pwm > last_right_pwm + d_pwm)
//     // {
//     //     current_right_pwm = last_right_pwm + d_pwm;
//     // }
//     // else if (current_right_pwm < last_right_pwm - d_pwm)
//     // {
//     //     current_right_pwm = last_right_pwm - d_pwm;
//     // }

//     // last_left_pwm = current_left_pwm;
//     // last_right_pwm = current_right_pwm;

//     Motor_SetLeft(current_left_pwm);
//     Motor_SetRight(current_right_pwm);
// }
void Motor_Control(void)
{
    const float nominal_battery = 11.095f;   // 标称补偿基准电压
    const float min_battery = 6.0f;          // 低于这个值认为异常/欠压，禁止输出
    const float max_comp_ratio = 1.3f;       // 最大补偿倍率，避免低压时补偿过猛
    const int32_t max_pwm_step = 200;        // 每次调用 PWM 最大变化量

    static int32_t last_left_pwm = 0;
    static int32_t last_right_pwm = 0;

    float battery = battery_voltage;
    float comp_ratio = 1.0f;
    compensated_left = 0.0f;
    compensated_right = 0.0f;
    current_left_pwm = 0;
    current_right_pwm = 0;

    // 电池电压异常或过低时，直接停机保护
    if (battery < min_battery)
    {
        last_left_pwm = 0;
        last_right_pwm = 0;
        Motor_SetLeft(0);
        Motor_SetRight(0);
        return;
    }

    // 只在局部做补偿，不改写 target_speed_left/right
    comp_ratio = nominal_battery / battery;
    if (comp_ratio > max_comp_ratio)
    {
        comp_ratio = max_comp_ratio;
    }

    compensated_left = target_pwm_left * comp_ratio;
    compensated_right = target_pwm_right * comp_ratio;

    compensated_left *= 100;
    compensated_right *= 100;

    current_left_pwm = (int32_t)compensated_left;
    current_right_pwm = (int32_t)compensated_right;

    if (current_left_pwm > last_left_pwm + max_pwm_step)
    {
        current_left_pwm = last_left_pwm + max_pwm_step;
    }
    else if (current_left_pwm < last_left_pwm - max_pwm_step)
    {
        current_left_pwm = last_left_pwm - max_pwm_step;
    }

    if (current_right_pwm > last_right_pwm + max_pwm_step)
    {
        current_right_pwm = last_right_pwm + max_pwm_step;
    }
    else if (current_right_pwm < last_right_pwm - max_pwm_step)
    {
        current_right_pwm = last_right_pwm - max_pwm_step;
    }

    last_left_pwm = current_left_pwm;
    last_right_pwm = current_right_pwm;

    Motor_SetLeft(current_left_pwm);
    Motor_SetRight(current_right_pwm);
}
