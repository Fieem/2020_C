#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

//-------------------------------------------------------------------------------------------------------------------
// 电机控制相关定义
//-------------------------------------------------------------------------------------------------------------------

// PWM 占空比最大值
#define PWM_MAX_VALUE               (10000)

// 左右电机对应的 PWM 通道（根据实际硬件配置修改）
// 这些需要根据你的硬件连接情况修改为实际使用的 PWM 通道
#define PWM_LEFT_CHANNEL            (PWM_TIM_G0_CH0_B10)    // 左电机 PWM 通道
#define PWM_RIGHT_CHANNEL           (PWM_TIM_G7_CH0_A26)    // 右电机 PWM 通道

#define PWM_LEFT_DIR_PIN           (B11)                    // 左电机方向控制引脚
#define PWM_RIGHT_DIR_PIN          (A27)                    // 右电机方向控制引
    
//-------------------------------------------------------------------------------------------------------------------
// 外部变量声明
//-------------------------------------------------------------------------------------------------------------------

extern int32_t measure_pwm;                                 // PWM 测量值（用于调试）

//-------------------------------------------------------------------------------------------------------------------
// 函数声明
//-------------------------------------------------------------------------------------------------------------------
/**
 * @brief 初始化电机控制模块
 * @return void
 */ 
void Motor_Init(void);

/**
 * @brief 设置左电机 PWM
 * @param PWM 正数表示正转，负数表示反转，范围 -PWM_MAX_VALUE ~ +PWM_MAX_VALUE
 * @return void
 */
void Motor_SetLeft(int32_t PWM);

/**
 * @brief 设置右电机 PWM
 * @param PWM 正数表示正转，负数表示反转，范围 -PWM_MAX_VALUE ~ +PWM_MAX_VALUE
 * @return void
 */
void Motor_SetRight(int32_t PWM);

void Motor_Control(void);
extern float compensated_left;
extern float compensated_right;
extern int32_t current_left_pwm;
extern int32_t current_right_pwm;

#endif // _MOTOR_H_
