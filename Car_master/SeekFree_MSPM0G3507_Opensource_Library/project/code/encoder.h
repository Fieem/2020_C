#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "zf_driver_encoder.h"

// 编码器引脚定义
// 左轮：TIMG8 硬件 QEI，需将编码器重新接到 B6/B7
#define ENC_LEFT_A      B6      // 左轮 A 相 → TIMG8 CH1
#define ENC_LEFT_B      B7      // 左轮 B 相 → TIMG8 CH2
// 右轮：A 相 EXTI 上升沿，B 相 GPIO 采样判向
#define ENC_RIGHT_A     B20     // 右轮 A 相 → GPIO EXTI
#define ENC_RIGHT_B     B17     // 右轮 B 相 → GPIO 输入（方向判断）

#define ENC_LEFT_QEI_CH1    TIMG8_ENCODER1_CH1_B6
#define ENC_LEFT_QEI_CH2    TIMG8_ENCODER1_CH2_B7

// 编码器参数
#define ENC_PPR         500     // 每圈脉冲数

// 初始化
void encoder_init(void);

// 原子获取并清零最近一个控制周期的脉冲数（正值=正转，负值=反转）
int32_t encoder_take_left(void);
int32_t encoder_take_right(void);

#endif
