#include "beep.h"
#include "zf_driver_gpio.h"
#include "project_globals.h"

static uint32_t buzzer_start_ms = 0;
static uint32_t buzzer_dur_ms   = 0;
static uint8_t  buzzer_active   = 0;

void Buzzer_Init(void)
{
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

// 非阻塞触发蜂鸣器响 ms 毫秒
void Buzzer_BeepMs(uint32_t ms)
{
    gpio_set_level(BUZZER_PIN, 1);
    buzzer_start_ms = g_timestamp_ms;
    buzzer_dur_ms   = ms;
    buzzer_active   = 1;
}

// 默认响 500ms
void Buzzer_Beep(void)
{
    Buzzer_BeepMs(500U);
}

// 主循环里调用，到时间自动关
void Buzzer_Poll(void)
{
    if (buzzer_active && (g_timestamp_ms - buzzer_start_ms >= buzzer_dur_ms))
    {
        gpio_set_level(BUZZER_PIN, 0);
        buzzer_active = 0;
    }
}
