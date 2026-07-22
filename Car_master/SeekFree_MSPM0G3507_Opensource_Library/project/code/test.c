/*******************************************************
 * 串口命令说明（VOFA/串口助手）
 *
 * 通用格式:
 *   KEY=VALUE!
 *   也支持一帧多命令: KEY1=V1,KEY2=V2,KEY3=V3!
 *
 * 帧结束符:
 *   !  或  \r  或  \n   （任意一个即可触发解析）
 *   注意: 必须是英文半角 ! ，不是中文全角 ！
 *
 * 命令列表:
 *   P1=<float>    -> pid_angle.Kp                       循迹环参数
 *   I1=<float>    -> pid_angle.Ki
 *   D1=<float>    -> pid_angle.Kd
 *
 *   P2=<float>    -> pid_yaw.Kp                         角度环参数
 *   I2=<float>    -> pid_yaw.Ki
 *   D2=<float>    -> pid_yaw.Kd
 *
 *   P3=<float>    -> pid_gyro_z.Kp                      陀螺仪Z轴速率环参数
 *   I3=<float>    -> pid_gyro_z.Ki
 *   D3=<float>    -> pid_gyro_z.Kd
 *
 *   TY=<float>    -> target_yaw                         目标航向角（单位度）
 *   SL=<float>    -> target_speed_left                  目标左轮速度
 *   SR=<float>    -> target_speed_right                 目标右轮速度
 *   TASK=<float>  -> task_number                        任务编号（2/3=停车）
 *   TG=<float>    -> target_gyro_z                      目标陀螺仪Z轴角速度
 *
 *   CAL=<float>   -> 校准流程步进一次（value>0.5 触发）
 *   START=<float> -> 启动（value>0.5 触发，start_flag++）
 *   BAT=<float>   -> 读取电池电压（value>0.5 触发）
 *   SHOW=<float>  -> 打印当前PID参数（value>0.5 触发）
 *
 * 示例:
 *   P1=0.80!
 *   START=1!
 *   SL=120,SR=120!
 *   CAL=1!
 *******************************************************/

#include "test.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adc.h"
#include "servo.h"
#include "zf_driver_uart.h"
#include "project_globals.h"
#include "lqr.h"
#include "zf_common_fifo.h"
static void prints_u(uint8_t index, unsigned int val);

#define TEST_VOFA_UART_INDEX      (UART_2)
#define TEST_VOFA_FRAME_MAX_LEN   (64U)

#define PRINTSF_MAX_LINES      (8U)    // 最多缓存8行
#define PRINTSF_MAX_LINE_LEN   (256U)   // 每行最大255字符
#define PRINTSF_TEXT_BUF_LEN   (PRINTSF_MAX_LINES * (PRINTSF_MAX_LINE_LEN + 2U) + 1U)

static char s_vofa_frame_buf[TEST_VOFA_FRAME_MAX_LEN];
static uint8_t s_vofa_frame_len = 0U;

static char s_log_lines[PRINTSF_MAX_LINES][PRINTSF_MAX_LINE_LEN];
static uint8_t s_log_head = 0U;   // 最旧行下标
static uint8_t s_log_count = 0U;  // 当前行数


static int test_key_equal(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0'))
    {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b))
        {
            return 0;
        }
        a++;
        b++;
    }
    return ((*a == '\0') && (*b == '\0'));
}

static int test_vofa_apply_kv(const char *key, float value)
{
    
    if (test_key_equal(key, "TS")) { target_speed_left = value; target_speed_right = value;                  printsf(0,"[VOFA] T_Speed=%.4f\r\n", value);           return 1; }

    if (test_key_equal(key, "P1")) { pid_speed_left.Kp = value;                                              printsf(0,"[VOFA] P1=%.4f\r\n", value);                return 1; }
    if (test_key_equal(key, "I1")) { pid_speed_left.Ki = value;                                              printsf(0,"[VOFA] I1=%.4f\r\n", value);                return 1; }
    if (test_key_equal(key, "D1")) { pid_speed_left.Kd = value;                                              printsf(0,"[VOFA] D1=%.4f\r\n", value);                return 1; }
    if (test_key_equal(key, "P2")) { pid_speed_right.Kp = value;                                             printsf(0,"[VOFA] P1=%.4f\r\n", value);                return 1; }
    if (test_key_equal(key, "I2")) { pid_speed_right.Ki = value;                                             printsf(0,"[VOFA] I1=%.4f\r\n", value);                return 1; }
    if (test_key_equal(key, "D2")) { pid_speed_right.Kd = value;                                             printsf(0,"[VOFA] D1=%.4f\r\n", value);                return 1; }
    if (test_key_equal(key, "TIME")) { speed_set=0.6f*time_control(value);                                        printsf(0,"[VOFA] SPEED=%.4f\r\n", speed_set);         return 1; }
// RSTC command removed — circle counting not needed
    if (test_key_equal(key, "CAL"))
    {
        if (value > 0.5f)
        {
            printsf(0, "[VOFA] CAL=1");
            adc_calibration_trigger_once();
        }
        return 1;
    }
    if (test_key_equal(key, "SHOW"))
    {
        if (value > 0.5f)
        {
            printsf(0, "%.2f, %.2f, %.2f", pid_angle.Kp, pid_angle.Ki, pid_angle.Kd);
            printsf(0, "%.2f, %.2f, %.2f", pid_yaw.Kp,   pid_yaw.Ki,   pid_yaw.Kd);
        }
        return 1;
    }
        if (test_key_equal(key, "START"))
    {
        if (value > 0.5f)
        {
            start_flag++;
            printsf(0, "[VOFA] START=%u", start_flag);
        }
        return 1;
    }
        
    if (test_key_equal(key, "RESTART"))
    {
        if (value > 0.5f)
        {
            start_flag          = 0;
            drive_state         = STATE_DRIVE;
            task_number         = 0;
            distance_accum      = 0.0f;
            servo_accum_angle   = 0.0f;
            turn_start_yaw      = 0.0f;
            pwm_set_duty(SERVO_PWM_CHANNEL, SERVO_DUTY_MID);    // 直接复位到中位
            printsf(0, "[VOFA] RESTART");
        }
        return 1;
    }

    if (test_key_equal(key, "BAT"))
    {
        if (value > 0.5f)
        {
            battery_voltage = adc_mean_filter_convert(ADC0_CH7_A22, 10)*0.0089388f; // 读取 A22 引脚的 ADC 值,3.3f/4096.0f*11.095f
            printsf(0, "[VOFA] BAT=%.4f", battery_voltage);
        }
        return 1;
    }

    return 0;
}

static void test_vofa_parse_segment(char *segment)
{
    char *eq = NULL;
    char *key = NULL;
    char *value_str = NULL;
    char *endptr = NULL;
    float value = 0.0f;

    while (isspace((unsigned char)*segment)) { segment++; }
    if (*segment == '\0')
    {
        return;
    }

    eq = strchr(segment, '=');
    if (eq == NULL)
    {
        return;
    }

    *eq = '\0';
    key = segment;
    value_str = eq + 1;

    while (isspace((unsigned char)*value_str)) { value_str++; }
    if (*value_str == '\0')
    {
        return;
    }

    value = strtof(value_str, &endptr);
    if (endptr == value_str)
    {
        return;
    }

    while ((endptr != NULL) && isspace((unsigned char)*endptr)) { endptr++; }
    if ((endptr != NULL) && (*endptr != '\0'))
    {
        return;
    }

    if (test_vofa_apply_kv(key, value))
    {
        //printsf(0,"[VOFA] %s=%.4f", key, value);
    }
}

void test_vofa_parse_frame(const char *frame)
{
    char work_buf[TEST_VOFA_FRAME_MAX_LEN];
    char *token = NULL;
    size_t len = 0U;

    if (frame == NULL)
    {
        return;
    }

    len = strlen(frame);
    if (len >= TEST_VOFA_FRAME_MAX_LEN)
    {
        len = TEST_VOFA_FRAME_MAX_LEN - 1U;
    }

    memcpy(work_buf, frame, len);
    work_buf[len] = '\0';

    token = strtok(work_buf, ",;");
    while (token != NULL)
    {
        test_vofa_parse_segment(token);
        token = strtok(NULL, ",;");
    }
}

void test_vofa_feed_byte(uint8_t byte)
{
    if ((byte == '!') || (byte == '\r') || (byte == '\n'))
    {
        if (s_vofa_frame_len > 0U)
        {
            s_vofa_frame_buf[s_vofa_frame_len] = '\0';
            test_vofa_parse_frame(s_vofa_frame_buf);
            s_vofa_frame_len = 0U;
        }
        return;
    }

    if (s_vofa_frame_len < (TEST_VOFA_FRAME_MAX_LEN - 1U))
    {
        s_vofa_frame_buf[s_vofa_frame_len++] = (char)byte;
    }
    else
    {
        // 缓冲区溢出，丢弃当前帧并等待下一个分隔符重新开始收包
        s_vofa_frame_len = 0U;
    }
}

void test_vofa_poll(void)
{
    uint8_t ch = 0U;
    while (uart_query_byte(TEST_VOFA_UART_INDEX, &ch))
    {
        test_vofa_feed_byte(ch);
    }
}

static void test_vofa_uart_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    uint8_t ch;
    while (uart_query_byte(TEST_VOFA_UART_INDEX, &ch))
    {
        test_vofa_feed_byte(ch);
    }
}

void test_vofa_init(void)
{
    s_vofa_frame_len = 0U;
    memset(s_vofa_frame_buf, 0, sizeof(s_vofa_frame_buf));
    uart_set_callback(TEST_VOFA_UART_INDEX, test_vofa_uart_callback, NULL);
    uart_set_interrupt_config(TEST_VOFA_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
    // 不用中断回调，走主循环 test_vofa_poll() 轮询，避免 ISR 里调 printf 死锁
    // uart_set_callback(TEST_VOFA_UART_INDEX, test_vofa_uart_callback, NULL);
    // uart_set_interrupt_config(TEST_VOFA_UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void prints(uint8_t index, const char *content)
{
    if (content == NULL)
    {
        content = "";
    }

    printf("page0.t%u.txt=\"%s\"", (unsigned int)index, content);
    printf("%c%c%c", (char)0xFF, (char)0xFF, (char)0xFF);
    //fflush(stdout);
}

static void prints_u(uint8_t index, unsigned int val)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%u", val);
    prints(index, buf);
}



static void printsf_store_line(const char *line)
{
    uint8_t pos;
    if (s_log_count < PRINTSF_MAX_LINES)
    {
        pos = (uint8_t)((s_log_head + s_log_count) % PRINTSF_MAX_LINES);
        s_log_count++;
    }
    else
    {
        pos = s_log_head; // 覆盖最旧
        s_log_head = (uint8_t)((s_log_head + 1U) % PRINTSF_MAX_LINES);
    }

    strncpy(s_log_lines[pos], line, PRINTSF_MAX_LINE_LEN - 1U);
    s_log_lines[pos][PRINTSF_MAX_LINE_LEN - 1U] = '\0';
}

void printsf(uint8_t index, const char *fmt, ...)
{
    char line[PRINTSF_MAX_LINE_LEN];
    char merged[PRINTSF_TEXT_BUF_LEN];
    va_list args;
    size_t off = 0U;
    uint8_t i;

    if (fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    // 可选：避免 Nextion 命令字符串被双引号破坏
    for (i = 0U; line[i] != '\0'; i++)
    {
        if (line[i] == '"') line[i] = '\'';
    }

    printsf_store_line(line);

    merged[0] = '\0';
    for (i = 0U; i < s_log_count; i++)
    {
        uint8_t pos = (uint8_t)((s_log_head + i) % PRINTSF_MAX_LINES);
        int n = snprintf(merged + off, sizeof(merged) - off, "%s%s",
                         s_log_lines[pos], (i + 1U < s_log_count) ? "\r\n" : "");
        if (n < 0) break;
        if ((size_t)n >= (sizeof(merged) - off))
        {
            off = sizeof(merged) - 1U;
            merged[off] = '\0';
            break;
        }
        off += (size_t)n;
    }

    prints(index, merged); // 复用你已有 prints: tX.txt="..." + FFF
}


void printsf_clear(uint8_t index)
{
    uint8_t i;

    s_log_head = 0U;
    s_log_count = 0U;

    for (i = 0U; i < PRINTSF_MAX_LINES; i++)
    {
        s_log_lines[i][0] = '\0';
    }

    prints(index, "");
}