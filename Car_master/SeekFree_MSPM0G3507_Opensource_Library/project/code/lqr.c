#include "lqr.h"
#include "icm.h"
#include "adc.h"
#include "project_globals.h"

static int prev_line_lost = 0;

void lqr_update(void)
{
    float steer;

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

    target_speed_left  = LQR_SPEED_SET + steer;
    target_speed_right = LQR_SPEED_SET - steer;
}
