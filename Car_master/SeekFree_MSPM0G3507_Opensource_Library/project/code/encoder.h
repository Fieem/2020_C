#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "zf_driver_timer.h"

// 编码器引脚定义
#define ENC_LEFT_A      B26     // 左轮 A 相 → TIM_G6 定时器捕获
#define ENC_LEFT_B      B25     // 左轮 B 相 → GPIO 输入（方向判断）
#define ENC_RIGHT_A     B20     // 右轮 A 相 → TIM_G12 定时器捕获
#define ENC_RIGHT_B     B24     // 右轮 B 相 → GPIO 输入（方向判断）

// 编码器参数
#define ENC_PPR         500     // 每圈脉冲数

// 初始化
void encoder_init(void);

// 获取累计脉冲数（正值=正转，负值=反转）
int32_t encoder_get_left(void);
int32_t encoder_get_right(void);

// 清零累计脉冲数
void encoder_clear_left(void);
void encoder_clear_right(void);

#endif
