#include "encoder.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "zf_common_interrupt.h"

static volatile int32_t s_right_exti_count = 0;

static void encoder_right_a_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;

    // 保持原有符号约定：B=1 为正转，B=0 为反转。
    if (gpio_get_level(ENC_RIGHT_B))
    {
        s_right_exti_count++;
    }
    else
    {
        s_right_exti_count--;
    }
}

// ---- 初始化 ----
void encoder_init(void)
{
    // ============ 左轮：TIMG8 硬件 QEI (B6/B7) ============
    encoder_quad_init(TIM_G8, ENC_LEFT_QEI_CH1, ENC_LEFT_QEI_CH2);

    // ============ 右轮：A 相 EXTI + B 相 GPIO (B20/B24) ============
    gpio_init(ENC_RIGHT_B, GPI, GPIO_HIGH, GPI_PULL_UP);
    exti_init(ENC_RIGHT_A, EXTI_TRIGGER_RISING,
              encoder_right_a_callback, NULL);
}

int32_t encoder_take_left(void)
{
    int32_t count;
    uint32 primask = interrupt_global_disable();

    count = (int32_t)encoder_get_count(TIM_G8);
    encoder_clear_count(TIM_G8);

    interrupt_global_enable(primask);
    return count;
}

int32_t encoder_take_right(void)
{
    int32_t count;
    uint32 primask = interrupt_global_disable();

    count = s_right_exti_count;
    s_right_exti_count = 0;

    interrupt_global_enable(primask);
    return count;
}
