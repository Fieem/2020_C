#include "lqr.h"
#include "icm.h"
#include "adc.h"
#include "project_globals.h"

float lqr_speed_set = 30.0f;    // 默认速度，运行时可通过总路程/目标时间重新设定
static int prev_line_lost = 0;

// 时间控制：根据总路程和目标时间计算设定速度
// time: 目标时间（秒），total_pulses: 全程编码器脉冲总数（外部变量）
// 返回值: 设定速度（脉冲/5ms）
int16 time_control(float time)
{
    if (time <= 0.01f) return 0;                    // 防除零
    lqr_speed_set = total_pulses / (time * 200.0f); // 200 = 1s / 5ms
    return (int16)lqr_speed_set;
}

void lqr_update(void)
{
    float steer;

    if (!start_flag) {
        target_speed_left  = 0;
        target_speed_right = 0;
        return;
    }

    if (!line_lost) {
        // 有线：巡线 + 航向
        steer = -(LQR_K1 * line_error_filtered
                + LQR_K2 * (Yaw_TotalAngle - target_yaw)
                + LQR_K3 * Yaw_g);
        prev_line_lost = 0;
        target_yaw = Yaw_TotalAngle;  // 持续同步当前航向
    } else {
        // 丢线：仅航向保持
        if (prev_line_lost == 0) {
            target_yaw = Yaw_TotalAngle;  // 丢线首帧锁定航向
            prev_line_lost = 1;
        }
        steer = -(LQR_K2 * (Yaw_TotalAngle - target_yaw)
                + LQR_K3 * Yaw_g);
    }

    // 限幅
    if (steer >  LQR_STEER_MAX) steer =  LQR_STEER_MAX;
    if (steer < -LQR_STEER_MAX) steer = -LQR_STEER_MAX;

    target_speed_left  = lqr_speed_set + steer;
    target_speed_right = lqr_speed_set - steer;
}
