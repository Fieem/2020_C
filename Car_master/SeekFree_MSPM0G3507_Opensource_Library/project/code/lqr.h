#ifndef LQR_H
#define LQR_H

// LQR 增益（由 lqr_gain.py 计算）
#define LQR_K1  4.31f    // line_error 增益
#define LQR_K2  1.36f    // heading_error 增益
#define LQR_K3  0.24f    // yaw_rate 增益

#define LQR_SPEED_SET   17.0f    // 基础前进速度 (脉冲/5ms)
#define LQR_STEER_MAX   15.0f    // 最大差速

void lqr_update(void);

#endif
