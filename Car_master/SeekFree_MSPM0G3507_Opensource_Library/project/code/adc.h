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


extern float line_error_raw;
extern float line_error_filtered;
extern int line_lost;

void tracking_control_loop(void);

#endif // !ADC_H
