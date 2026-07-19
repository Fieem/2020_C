#include "encoder.h"
#include "zf_driver_encoder.h"
#include "zf_driver_gpio.h"

// ---- 初始化 ----
void encoder_init(void)
{
    // ============ 硬件编码器：左轮 (TIM_G6 + B26/B25) ============
    encoder_dir_init(TIM_G6, TIMG6_ENCODER1_CH1_B26, ENC_LEFT_B);
    // 覆盖库默认的上拉，改为下拉（匹配编码器空闲低电平）
    gpio_init(ENC_LEFT_B, GPI, GPIO_LOW, GPI_PULL_DOWN);
    //DL_Timer_startCounter(TIM_G6);

    // ============ 硬件编码器：右轮 (TIM_G12 + B20/B24) ============
    encoder_dir_init(TIM_G12, TIMG12_ENCODER1_CH1_B20, ENC_RIGHT_B);
    // 覆盖库默认的上拉，改为下拉
    gpio_init(ENC_RIGHT_B, GPI, GPIO_LOW, GPI_PULL_DOWN);
    //DL_Timer_startCounter(TIM_G12);
}

// ---- 获取左轮脉冲数 ----
int32_t encoder_get_left(void)
{
    return (int32_t)-encoder_get_count(TIM_G6);
}

// ---- 获取右轮脉冲数 ----
int32_t encoder_get_right(void)
{
    return (int32_t)-encoder_get_count(TIM_G12);
}

// ---- 左轮清零 ----
void encoder_clear_left(void)
{
    encoder_clear_count(TIM_G6);
}

// ---- 右轮清零 ----
void encoder_clear_right(void)
{
    encoder_clear_count(TIM_G12);
}
