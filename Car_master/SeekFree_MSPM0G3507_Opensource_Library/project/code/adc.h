#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <math.h>
#include "zf_common_headfile.h"

#define AD2 B8
#define AD1 B9
#define AD0 B13

extern int adc_mode;
extern int adc_raw_value[8];

#define ADC_CALIB_KEY_PIN            A30
#define ADC_CALIB_TASK_PERIOD_MS     (20U)
#define ADC_CALIB_DEBOUNCE_MS        (20U)
#define ADC_CALIB_SHORT_PRESS_MS     (60U)
#define ADC_CALIB_LONG_PRESS_MS      (4000U)
#define ADC_CALIB_PRINT_PERIOD_MS    (100U)

typedef enum
{
    ADC_CALIB_IDLE = 0,
    ADC_CALIB_WAIT_WHITE = 1,
    ADC_CALIB_WAIT_BLACK = 2,
    ADC_CALIB_DONE = 3,
} adc_calib_state_enum;

extern adc_calib_state_enum adc_calib_state;

void adc_calibration_task(void);
uint8 adc_calib_flash_load(void);

void adc_capture(void);
void adc_capture_init(void);
void calculate_line_error(void);

extern int adc_background_value[8];
extern int adc_foreground_value[8];
extern int adc_calibrated_value[8];
void adc_calibration_trigger_once(void);
void adc_calib_button_callback(uint32_t event, void *ptr);


extern float line_error_raw;
extern float line_error_filtered;
extern int line_lost;

#define STOP_LINE_THRESHOLD 400   // 8 路灰度值之和超过此值判定为横线停车
#define STOP_LINE_COUNT     5    // 连续检测帧数（5ms/帧，10帧=50ms）

// 状态机
typedef enum {
    STATE_DRIVE      = 0,     // 直行：舵机 0°，不读灰度，等距离
    STATE_TURN       = 1,     // 转弯：舵机固定角 + 差速，读灰度等停车
    STATE_AFTER_TURN = 2,     // 转弯后：舵机 0°，取消差速，读灰度等停车
    STATE_STOP       = 3,     // 停止：目标速度 0
} drive_state_t;

// 状态机阈值
#define DRIVE_DIST_THRESHOLD    55000.0f    // 直行距离阈值（编码器脉冲）
#define TURN_YAW_THRESHOLD      87.0f       // 转弯偏航角阈值（度），超过则回正
#define TURN_SERVO_ANGLE        12.0f       // 转弯时舵机固定角度

// 转弯差速随坡度自适应：diff = MIN + (BASE - MIN) × cos(θ)
#define TURN_DIFF_BASE          15.0f       // 平地差速（最大）
#define TURN_DIFF_MIN            5.0f       // 陡坡差速下限

extern drive_state_t drive_state;

void tracking_control_loop(void);
void check_stop_line(void);

#endif // !ADC_H
