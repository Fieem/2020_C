#include "adc.h"
#include "zf_device_key.h"
#include "zf_driver_flash.h"
#include "project_globals.h"
#include "test.h"
#include "motor.h"
#include "icm.h"
#include "lqr.h"
#include "servo.h"
#include "beep.h"
#include <string.h>
//-------------------------------------------------------------------------------------------------------------------
// 本文件是关于adc光电管灰度值读取的相关内容
//-------------------------------------------------------------------------------------------------------------------

int adc_mode = 0; // 0:正常采集, 1:校准背景，2:校准前景
int adc_raw_value[8] = {0}; // 存储8个通道的ADC值
int adc_background_value[8] = {0}; // 背景校准值
int adc_foreground_value[8] = {0}; // 前景校准值
int adc_calibrated_value[8] = {0}; // 校准后的值
float line_error_raw = 0.0f;

// (cleaned: Yaw_Lock, Yaw_Lock_a, angle_flag, turning removed)

int line_lost = 0;
static uint8_t stop_line_count = 0;

static float clampf_local(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

#define ADC_CALIB_FLASH_SECTOR   (0U)
#define ADC_CALIB_FLASH_PAGE     (0U)
#define ADC_CALIB_MAGIC          (0x434C4238UL) // "CLB8"
#define ADC_CALIB_VERSION        (1U)

typedef struct
{
    uint32 magic;
    uint32 version;
    int32 bg[8];
    int32 fg[8];
    uint32 crc;
} adc_calib_blob_t;

static uint32 adc_calib_crc32_xor(const adc_calib_blob_t *blob)
{
    const uint32 *p = (const uint32 *)blob;
    uint32 x = 0U;
    uint32 i = 0U;

    for (i = 0U; i < (uint32)(sizeof(adc_calib_blob_t) / sizeof(uint32) - 1U); i++)
    {
        x ^= p[i];
    }
    return x;
}

static uint8 adc_calib_flash_save(void)
{
    adc_calib_blob_t blob;
    uint32 i = 0U;

    memset(&blob, 0, sizeof(blob));
    blob.magic = ADC_CALIB_MAGIC;
    blob.version = ADC_CALIB_VERSION;
    for (i = 0U; i < 8U; i++)
    {
        blob.bg[i] = (int32)adc_background_value[i];
        blob.fg[i] = (int32)adc_foreground_value[i];
    }
    blob.crc = adc_calib_crc32_xor(&blob);

    return flash_write_page(ADC_CALIB_FLASH_SECTOR,
                            ADC_CALIB_FLASH_PAGE,
                            (const uint32 *)&blob,
                            (uint16)(sizeof(adc_calib_blob_t) / sizeof(uint32)));
}

uint8 adc_calib_flash_load(void)
{
    adc_calib_blob_t blob;
    uint32 i = 0U;

    memset(&blob, 0xFF, sizeof(blob));
    flash_read_page(ADC_CALIB_FLASH_SECTOR,
                    ADC_CALIB_FLASH_PAGE,
                    (uint32 *)&blob,
                    (uint16)(sizeof(adc_calib_blob_t) / sizeof(uint32)));

    if ((blob.magic != ADC_CALIB_MAGIC) || (blob.version != ADC_CALIB_VERSION))
    {
        return 0U;
    }

    if (blob.crc != adc_calib_crc32_xor(&blob))
    {
        return 0U;
    }

    for (i = 0U; i < 8U; i++)
    {
        adc_background_value[i] = (int)blob.bg[i];
        adc_foreground_value[i] = (int)blob.fg[i];
    }
    return 1U;
}

void adc_capture_init(void)
{
    gpio_init(AD2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(AD1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(AD0, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(AD2, 1);
    gpio_set_level(AD1, 1);
    gpio_set_level(AD0, 1);

    // adc_mode = 0; // 默认模式为正常采集
}
//真值表
//AD2 AD1 AD0
// 0   0   0   CH1
// 0   0   1   CH2
// 0   1   0   CH3
// 0   1   1   CH4
// 1   0   0   CH5
// 1   0   1   CH6
// 1   1   0   CH7
// 1   1   1   CH8
void adc_scan(void)
{
    // adc_mean_filter_convert(ADC1_CH5_B18, 10);
    gpio_set_level(AD2, 0);
    gpio_set_level(AD1, 0);
    gpio_set_level(AD0, 0); // CH1
    system_delay_us(1);
    adc_raw_value[0]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 0);
    gpio_set_level(AD1, 0);
    gpio_set_level(AD0, 1); // CH2
    system_delay_us(1);
    adc_raw_value[1]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 0);
    gpio_set_level(AD1, 1);
    gpio_set_level(AD0, 0); // CH3
    system_delay_us(1);
    adc_raw_value[2]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 0);
    gpio_set_level(AD1, 1);
    gpio_set_level(AD0, 1); // CH4
    system_delay_us(1);
    adc_raw_value[3]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 1);
    gpio_set_level(AD1, 0);
    gpio_set_level(AD0, 0); // CH5
    system_delay_us(1);
    adc_raw_value[4]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 1);
    gpio_set_level(AD1, 0);
    gpio_set_level(AD0, 1); // CH6
    system_delay_us(1);
    adc_raw_value[5]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 1);
    gpio_set_level(AD1, 1);
    gpio_set_level(AD0, 0); // CH7
    system_delay_us(1);
    adc_raw_value[6]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
    gpio_set_level(AD2, 1);
    gpio_set_level(AD1, 1);
    gpio_set_level(AD0, 1); // CH8
    system_delay_us(1);
    adc_raw_value[7]=adc_mean_filter_convert(ADC1_CH5_B18, 15);
}
void adc_capture(void)
{
    adc_scan(); // 采集 8 路 ADC 原始值

    if(adc_mode == 0) // 正常模式：执行归一化
    {
        for(int i = 0; i < 8; i++)
        {
            // 分母是“黑白差值”，太小会导致放大噪声甚至抖动
            int den = adc_foreground_value[i] - adc_background_value[i];

            // 若标定无效（分母太小），该通道直接置 0，避免异常放大
            if((den > -8) && (den < 8))
            {
                adc_calibrated_value[i] = 0;
                continue;
            }

            // 归一化到 0~100；用 float 防止整型截断误差
            float v = ((float)(adc_raw_value[i] - adc_background_value[i]) * 100.0f) / (float)den;

            // 限幅，避免异常值进入后续误差计算
            if(v < 0.0f)   v = 0.0f;
            if(v > 100.0f) v = 100.0f;

            adc_calibrated_value[i] = (int)v;
        }
    }
    else if(adc_mode == 1) // 采白底
    {
        for(int i = 0; i < 8; i++)
        {
            adc_background_value[i] = adc_raw_value[i];
        }
    }
    else if(adc_mode == 2) // 采黑线
    {
        for(int i = 0; i < 8; i++)
        {
            adc_foreground_value[i] = adc_raw_value[i];
        }
    }
}

// 线性跟踪误差计算函数 - 基于校准后的ADC值计算循迹偏差
void calculate_line_error(void) 
{
    float cha_L = 0.0f;
    float cha_R = 0.0f;
    float he_L = 0.0f;
    float he_R = 0.0f;

    // 使用校准后的ADC值进行计算（已归一化到0-100）
    // 左侧和值：前4个传感器
    he_L = adc_calibrated_value[0] + adc_calibrated_value[1] + adc_calibrated_value[2] + adc_calibrated_value[3];
    // 右侧和值：后4个传感器
    he_R = adc_calibrated_value[4] + adc_calibrated_value[5] + adc_calibrated_value[6] + adc_calibrated_value[7];

    // 左侧加权和：位置权重递减
    cha_L = (adc_calibrated_value[0] * 5) + (adc_calibrated_value[1] * 4) + (adc_calibrated_value[2]*0 ) + adc_calibrated_value[3]*0;
    // 右侧加权和：位置权重递减
    cha_R = (adc_calibrated_value[7] * 5) + (adc_calibrated_value[6] * 4) + (adc_calibrated_value[5]*0 ) + adc_calibrated_value[4]*0;
    // 计算差比和
    float denominator = he_L + he_R;

    // 丢线判定：总能量太小，说明看不到有效线，滤波值缓慢回 0
    if (denominator < 50.0f)
    {
        line_lost = 1;
        //line_error_filtered = 0.0f;
        return;
    }
    line_lost = 0;

    // 原始误差
    line_error_raw = (cha_L - cha_R) * 350.0f / denominator;
    line_error_raw = clampf_local(line_error_raw, -500.0f, 500.0f);

    // 缩放后的误差（原代码保持一致）
    float line_error_scaled = line_error_raw * 0.007f;

    // 暂时不做自动偏置补偿，保留原始左右误差的对称性
    float compensated = line_error_scaled;

    // 小死区：抑制中心附近抖动，防止“慢性漂移感”
    if (fabsf(compensated) < 0.015f)
    {
        compensated = 0.0f;
    }

    // 主滤波
    const float alpha = 0.8f;
    line_error_filtered = alpha * compensated + (1.0f - alpha) * line_error_filtered;
}

adc_calib_state_enum adc_calib_state = ADC_CALIB_IDLE;


// check_90_angle() removed — no right-angle turns in slope task
void adc_calibration_task(void)
{
    static uint8 last_sample = 0;
    static uint8 stable_pressed = 0;
    static uint32 debounce_ms = 0;
    static uint32 press_ms = 0;
    static uint32 print_ms = 0;

    uint8 sample_pressed = (gpio_get_level(ADC_CALIB_KEY_PIN) != KEY_RELEASE_LEVEL);

    if(sample_pressed == last_sample)
    {
        if(debounce_ms < ADC_CALIB_DEBOUNCE_MS)
        {
            debounce_ms += ADC_CALIB_TASK_PERIOD_MS;
        }
    }
    else
    {
        last_sample = sample_pressed;
        debounce_ms = 0;
    }

    if((debounce_ms >= ADC_CALIB_DEBOUNCE_MS) && (stable_pressed != sample_pressed))
    {
        stable_pressed = sample_pressed;

        if(0 == stable_pressed)
        {
            if(press_ms >= ADC_CALIB_LONG_PRESS_MS)
            {
                adc_calib_state = ADC_CALIB_WAIT_WHITE;
							
							
                print_ms = 0;
                printsf(0, "cal_state=%d", adc_calib_state);
                printsf(0, "put on white");
            }
            else if(press_ms >= ADC_CALIB_SHORT_PRESS_MS)
            {
                if(ADC_CALIB_WAIT_WHITE == adc_calib_state)
                {
                    adc_mode = 1;
                    adc_capture();
                    adc_mode = 0;

                    adc_calib_state = ADC_CALIB_WAIT_BLACK;
                    printsf(0, "cal_state=%d", adc_calib_state);
                    printsf(0, "put on black");
                }
                else if(ADC_CALIB_WAIT_BLACK == adc_calib_state)
                {
                    adc_mode = 2;
                    adc_capture();
                    adc_mode = 0;

                    adc_calib_state = ADC_CALIB_DONE;
                    adc_calib_flash_save();
                    print_ms = ADC_CALIB_PRINT_PERIOD_MS;

                    printsf(0, "cal_state=%d", adc_calib_state);
                    printsf(0, "done");
                }
            }

            press_ms = 0;
        }
    }

    if(stable_pressed)
    {
        press_ms += ADC_CALIB_TASK_PERIOD_MS;
    }

    if(ADC_CALIB_DONE == adc_calib_state)
    {
        if(print_ms >= ADC_CALIB_PRINT_PERIOD_MS)
        {
            print_ms = 0;

            // printf("%d %d %d %d %d %d %d %d\r\n",
            //        adc_calibrated_value[0], adc_calibrated_value[1],
            //        adc_calibrated_value[2], adc_calibrated_value[3],
            //        adc_calibrated_value[4], adc_calibrated_value[5],
            //        adc_calibrated_value[6], adc_calibrated_value[7]);
            // printf("line_error_filtered: %.2f\r\n", line_error_filtered);
            // printf("gyro_yaw: %.2f， target_yaw: %.2f, target_speed_left: %.2f, target_speed_right: %.2f\r\n", gyro_yaw, target_yaw, target_speed_left, target_speed_right);
            adc_xunhuan = 0; // 停止ADC循环
        }
        else
        {
            print_ms += ADC_CALIB_TASK_PERIOD_MS;
        }
    }
}

void adc_calibration_trigger_once(void)
{
    if (ADC_CALIB_WAIT_WHITE == adc_calib_state)
    {
        adc_mode = 1;
        adc_capture();
        adc_mode = 0;

        adc_calib_state = ADC_CALIB_WAIT_BLACK;
        printsf(0, "cal_state=%d", adc_calib_state);
        printsf(0, "put on black");
        Buzzer_BeepMs(100);
        return;
    }

    if (ADC_CALIB_WAIT_BLACK == adc_calib_state)
    {
        adc_mode = 2;
        adc_capture();
        adc_mode = 0;
        adc_xunhuan = 0; 
        adc_calib_state = ADC_CALIB_DONE;
        adc_calib_flash_save();
        printsf(0, "cal_state=%d", adc_calib_state);
        printsf(0, "done");
        Buzzer_BeepMs(500);
        return;
    }

    /* IDLE 或 DONE 时，第一次触发都进入白底等待 */
    adc_calib_state = ADC_CALIB_WAIT_WHITE;
    printsf(0, "cal_state=%d", adc_calib_state);
    printsf(0, "put on white");
    Buzzer_BeepMs(100);
}


// lap_control_update_5ms() removed — circle counting not needed for slope task

static float ackermann_diff_from_angle(float steer_angle);
static void set_target_speed_by_steer_angle(float steer_angle);

void tracking_control_loop()// 循迹控制主循环（状态机）
{
    static drive_state_t previous_state = STATE_STOP;
    static float last_steer_angle_drive = 0.0f;
    static float last_steer_angle_after = 0.0f;
    static uint32_t line_lost_start_ms = 0U;
    static uint8_t line_lost_timing = 0U;

    // 未发车时不进入状态机，避免状态分支把目标速度重新覆盖为 speed_set
    if (!start_flag) {
        previous_state = STATE_STOP;
        last_steer_angle_drive = 0.0f;
        last_steer_angle_after = 0.0f;
        line_lost_start_ms = 0U;
        line_lost_timing = 0U;
        stop_line_count = 0;
        pid_pos_reset(&pid_angle);
        target_speed_left  = 0.0f;
        target_speed_right = 0.0f;
        target_pwm_left    = 0.0f;
        target_pwm_right   = 0.0f;
        Servo_SetAngle(0.0f);
        pid_pos_reset(&pid_speed_left);
        pid_pos_reset(&pid_speed_right);
        Motor_Control();
        return;
    }

    // 状态切换时清理上一个状态的滤波/保持量，防止重新发车或回正后立即跳转
    if (drive_state != previous_state)
    {
        if (drive_state == STATE_DRIVE)
        {
            last_steer_angle_drive = 0.0f;
            line_lost_start_ms = 0U;
            line_lost_timing = 0U;
            // 以发车时的偏航角作为本段循迹的参考角
            turn_start_yaw = Yaw_TotalAngle;
            pid_pos_reset(&pid_angle);
        }
        else if (drive_state == STATE_AFTER_TURN)
        {
            last_steer_angle_after = 0.0f;
            pid_pos_reset(&pid_angle);
        }
        previous_state = drive_state;
    }

    target_speed_left  = speed_set;
    target_speed_right = speed_set;

    switch (drive_state) {

    case STATE_DRIVE:
        // 灰度误差 PID -> 舵机角度 -> 阿克曼差速
        {
            float steer_angle;

            servo_accum_angle = 0.0f;
            adc_capture();
            calculate_line_error();

            if (!line_lost) {
                line_lost_timing = 0U;
                line_lost_start_ms = 0U;
                steer_angle = STEER_POLARITY *
                              pid_pos_calculate(&pid_angle, 0.0f, line_error_filtered,
                                                -100.0f, 100.0f,
                                                SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
                last_steer_angle_drive = steer_angle;
            } else {
                steer_angle = last_steer_angle_drive;   // 丢线保持上一帧舵角

                if (!line_lost_timing)
                {
                    line_lost_start_ms = g_timestamp_ms;
                    line_lost_timing = 1U;
                }

                if ((g_timestamp_ms - line_lost_start_ms) >= LINE_LOST_TURN_HOLD_MS)
                {
                    turn_start_yaw = Yaw_TotalAngle;
                    drive_state = STATE_TURN;
                }
            }

            if (drive_state == STATE_TURN)
            {
                float turn_angle = TURN_SERVO_ANGLE;
                Servo_SetAngle(turn_angle);
                target_speed_left  = TURN_SPEED_LEFT;
                target_speed_right = TURN_SPEED_RIGHT;
            }
            else
            {
                Servo_SetAngle(steer_angle);
                set_target_speed_by_steer_angle(steer_angle);
            }

        }
        break;

    case STATE_TURN:
        // 舵机固定角度 + 左右轮固定速度，读灰度检测横线停车
        {
            float turn_angle = TURN_SERVO_ANGLE;
            Servo_SetAngle(turn_angle);
            target_speed_left  = TURN_SPEED_LEFT;
            target_speed_right = TURN_SPEED_RIGHT;
        }
        adc_capture();
        check_stop_line();
        if (task_number == 2 || task_number == 3) {
            drive_state = STATE_STOP;
            printsf(0, "STOP!");
            break;
        }
        // 使用进入转弯时的角度作为基准，而不是使用上电后的绝对偏航角
        if (fabsf(Yaw_TotalAngle) >= TURN_YAW_THRESHOLD) {
            drive_state = STATE_AFTER_TURN;
        }
        break;

    case STATE_AFTER_TURN:
        // 灰度误差 PID -> 舵机角度 -> 阿克曼差速
        {
            float steer_angle;

            servo_accum_angle = 0.0f;
            adc_capture();
            calculate_line_error();

            if (!line_lost) {
                steer_angle = STEER_POLARITY *
                              pid_pos_calculate(&pid_angle, 0.0f, line_error_filtered,
                                                -100.0f, 100.0f,
                                                SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
                last_steer_angle_after = steer_angle;
            } else {
                steer_angle = last_steer_angle_after;     // 丢线保持上一帧舵角
            }

            Servo_SetAngle(steer_angle);
            set_target_speed_by_steer_angle(steer_angle);

            check_stop_line();
            if (task_number == 2 || task_number == 3) {
                drive_state = STATE_STOP;
                printsf(0, "STOP!");
            }
        }
        break;

    case STATE_STOP:
        Servo_SetAngle(0.0f);
        target_speed_left  = 0.0f;
        target_speed_right = 0.0f;
        break;
    }

    // 停车状态不再让速度PID根据残余误差产生反向制动输出
    if (drive_state == STATE_STOP)
    {
        target_speed_left  = 0.0f;
        target_speed_right = 0.0f;
        target_pwm_left    = 0.0f;
        target_pwm_right   = 0.0f;
        pid_pos_reset(&pid_speed_left);
        pid_pos_reset(&pid_speed_right);
        Motor_Control();
        return;
    }

    // 速度环 + 电机输出
    pid_loop_speed_update();
    Motor_Control();
}

// 根据舵机实际转向角计算阿克曼差速。
// 约定：正舵角对应左转，此时左轮为内侧轮、速度较低。
static float ackermann_diff_from_angle(float steer_angle)
{
    const float pi = 3.1415926f;
    float ratio;

    // 舵机机械限位为±13°，差速计算也使用限幅后的角度
    if (steer_angle < SERVO_ANGLE_MIN) steer_angle = SERVO_ANGLE_MIN;
    if (steer_angle > SERVO_ANGLE_MAX) steer_angle = SERVO_ANGLE_MAX;

    ratio = (WHEEL_TRACK_MM * tanf(steer_angle * pi / 180.0f))
          / (2.0f * WHEEL_BASE_MM);

    // 防止极端舵角或参数错误导致内侧轮反转
    if (ratio > ACKERMANN_MAX_RATIO) ratio = ACKERMANN_MAX_RATIO;
    if (ratio < -ACKERMANN_MAX_RATIO) ratio = -ACKERMANN_MAX_RATIO;

    return speed_set * ratio;
}

static void set_target_speed_by_steer_angle(float steer_angle)
{
    float diff = ackermann_diff_from_angle(steer_angle);

    if (fabsf(diff) < SPEED_DIFF_DEADZONE)
    {
        diff = 0.0f;
    }

    target_speed_left  = speed_set + diff * 5;
    target_speed_right = speed_set - diff * 5;
}

/*
使用示例：

1. 初始化阶段：
   adc_capture_init();    // 初始化ADC采集
   Motor_init();          // 初始化电机驱动
   
2. 校准阶段（可选）：
   adc_mode = 1;          // 设置为背景校准模式
   adc_capture();         // 采集背景值（白线或无线状态）
   
   adc_mode = 2;          // 设置为前景校准模式  
   adc_capture();         // 采集前景值（黑线状态）
   
   adc_mode = 0;          // 切换回正常采集模式

3. 主循环中调用：
   tracking_control_loop(); // 每个控制周期调用一次

*/

// 横线停车检测：连续多帧灰度之和超阈值才触发，避免误判
void check_stop_line(void)
{
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += adc_calibrated_value[i];
    }

    if (sum > STOP_LINE_THRESHOLD) {
        if (stop_line_count < STOP_LINE_COUNT) {
            stop_line_count++;
            if (stop_line_count >= STOP_LINE_COUNT && task_number != 2) {
                task_number = 2;
                Buzzer_BeepMs(500);     // 停车时蜂鸣 200ms
            }
        }
    } else {
        stop_line_count = 0;     // 一帧不满足就清零
    }
}


void adc_calib_button_callback(uint32_t event, void *ptr)
{
    (void)event;
    (void)ptr;
    static uint32_t last_ms = 0;
    if (g_timestamp_ms - last_ms < 200U) return;  // 200ms 消抖
    last_ms = g_timestamp_ms;
    adc_calibration_trigger_once();
}
