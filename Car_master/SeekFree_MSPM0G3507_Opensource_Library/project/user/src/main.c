/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
************************************************************************** ******************************************/

#include <ti/driverlib/driverlib.h>
#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_driver_pwm.h"
#include "zf_driver_adc.h"
#include "motor.h"
#include "icm42688.h"
#include "project_globals.h"
#include "pid.h"
#include "adc.h"
#include "zf_device_key.h"
#include "test.h"
#include "icm.h"
#define IMU_WARMUP_MS (2000)            // ICM42688 陀螺仪热身时间，单位为毫秒，建议至少 2000ms 以确保陀螺仪稳定
static uint32_t s_last_100ms = 0;       // 上次100ms行为触发时间

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

static void wait_start_key(void)
{
    while (gpio_get_level(A31) == KEY_RELEASE_LEVEL)
    {
        Motor_SetLeft(0);
        Motor_SetRight(0);
        system_delay_ms(10);
    }

    while (gpio_get_level(A31) != KEY_RELEASE_LEVEL)
    {
        system_delay_ms(10);
    }
}

int main (void)
{
clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化
    adc_init(ADC0_CH7_A22, ADC_12BIT);
	// 此处编写用户代码 例如外设初始化代码等
    test_vofa_init();                               // 初始化 VOFA+ 调参接口
    // printf("MSPM0G3507 Opensource Library!\r\n");
    printsf_clear(0);                               // 清空缓存并清空 t0 文本框
    printsf_clear(1); 
    printsf_clear(2); 
    printsf(0, "MSPM0G3507 OpenSource Library");
    key_init(5);                                    // 初始化按键模块 
    //gpio_init(B22, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_TIM_G8_CH1_B22, 1000, 0);          // 1kHz
    // adc_init(ADC0_CH7_A22, ADC_12BIT);           // 初始化 ADC0 的 A22 引脚为 12 位分辨率
    adc_init(ADC1_CH5_B18, ADC_12BIT);
    adc_capture_init();                             // 初始化 ADC 采集相关 GPIO 和状态
    
    if (adc_calib_flash_load())
    {
        adc_xunhuan = 0;
        adc_calib_state = ADC_CALIB_DONE;
        printsf(0, "ADC calib loaded from flash");
    }
    Motor_Init();                                   // 初始化电机控制模块
    pid_loop_angle_init();
    pid_loop_yaw_init();
    pid_loop_gyro_z_init();
	while (adc_xunhuan)
    {  
        // test_vofa_poll();  
        adc_calibration_task();
        system_delay_ms(ADC_CALIB_TASK_PERIOD_MS);
    }
    while (start_flag == 0);
    //wait_start_key();                               // 等待按键按下后开始执行陀螺仪校准和
    printsf(0, "ICM start!");
    Init_ICM42688();                                // 初始化 ICM42688 陀螺仪
    system_delay_ms(IMU_WARMUP_MS);                 // Give the gyro time to thermally settle before bias calibration.
    IMU_calibration();                              // ICM42688 陀螺仪校准
    Filter_Init();
    printsf(0, "fuck!!!!");
    printsf(0, "hello world");

    while (start_flag == 1);
    //wait_start_key();                               // 等待按键按下后开始执行主循环
    pit_ms_init(PIT_TIM_A0,5,NULL,NULL);	
    pit_ms_init(PIT_TIM_A1,5,NULL,NULL);
    interrupt_set_priority(TIMA0_INT_IRQn, 1);      // 中断优先级 0-7 越低越高
    interrupt_set_priority(TIMA1_INT_IRQn, 0);      // 设置定时器A1中断优先级为0
    printsf(0, "start!");

    battery_voltage = adc_mean_filter_convert(ADC0_CH7_A22, 10)*0.0089388f;
    if(battery_voltage < 9.0f)
    {
        printsf(0, "battery_voltage: %.2f", battery_voltage);
        while(1);
    }
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
        // printf("gyro_yaw: %.2f icm42688_gyro_z: %.2f\r\n", gyro_yaw, icm42688_gyro_z);
        uint32_t now = g_timestamp_ms;              // 璇诲綋鍓嶆椂闂存埑
        battery_voltage = adc_mean_filter_convert(ADC0_CH7_A22, 10)*0.0089388f; // 读取 A22 引脚的 ADC 值,3.3f/4096.0f*11.095f
        if ((now - s_last_100ms) >= 100U)
        {
            s_last_100ms = now;
            //printf("Data:  %.2f, %.2f, %.2f, %.2f, %.2f\r\n", target_yaw, gyro_yaw, target_gyro_z, target_speed_left, target_speed_right); 
            //printf("Data: %.2f, %.2f, %.2f, %d, %d\r\n", target_gyro_z, yaw_rate_z, line_error_filtered, current_left_pwm, current_right_pwm);
            //printf("Data: %.2f, %.2f, %.2f\r\n", Yaw_TotalAngle, Pitch_a, Roll_a);
            
        // (board communication removed — not needed for slope task)
        }

        // printf("line_error_filtered: %.2f\r\n", line_error_filtered);
        //printsf(0, "%.2f", gyro_yaw);
        // system_delay_ms(200);
        test_vofa_poll();

    }
}

