"""
LQR 控制器增益计算 — 坡道巡线小车

状态 x = [line_error, heading_error(°), yaw_rate(°/s)]
控制 u = steer (脉冲/5ms, 叠加到左右轮速度)
"""

import numpy as np
from scipy.linalg import solve_discrete_are

# ========== 物理参数 ==========
wheel_diam    = 0.065       # 轮径 m
wheel_circ    = np.pi * wheel_diam   # 0.2042 m
ppr           = 500
gear_ratio    = 28
pulses_per_rev = ppr * gear_ratio   # 14000
track_width   = 0.1175      # 轮距 m
Ts            = 0.005       # 采样周期 5ms
forward_speed = 0.05        # 前进速度 m/s (5cm/s)

# steer 转换: 1 脉冲/5ms → 多少 rad/s 的 yaw rate
pulse_to_linvel = (wheel_circ / pulses_per_rev) / Ts  # 0.002917 m/s per pulse/5ms
omega_per_steer = 2.0 * pulse_to_linvel / track_width  # 0.0496 rad/s per pulse/5ms
omega_per_steer_deg = omega_per_steer * 180.0 / np.pi  # 2.845 °/s per pulse/5ms

print(f"1 脉冲/5ms steer → {omega_per_steer_deg:.2f} °/s yaw rate")
print(f"forward_speed: {forward_speed:.3f} m/s")

# ========== 连续状态空间 ==========
# x = [line_error, heading_err(°), yaw_rate(°/s)]
# u = steer (脉冲/5ms)

# A_cont: 连续系统矩阵
# de/dt  = v * sin(theta~°) ≈ forward_speed * theta_rad = forward_speed * theta_deg * pi/180
#         ≈ forward_speed * pi/180 * theta_deg
# dtheta/dt = omega  (yaw_rate in °/s)
# domega/dt = -omega/tau + (omega_per_steer_deg/tau) * u
# tau = yaw rate 响应时间常数 (s)

tau = 0.05  # 电机响应 ~50ms

A_cont = np.array([
    [0.0, forward_speed * np.pi / 180.0, 0.0],   # de/dt
    [0.0, 0.0,                          1.0],    # dtheta/dt
    [0.0, 0.0,                        -1.0/tau]  # domega/dt
])

B_cont = np.array([
    [0.0],
    [0.0],
    [omega_per_steer_deg / tau]
])

print(f"\nA_cont:\n{A_cont}")
print(f"\nB_cont:\n{B_cont}")

# ========== 离散化: 零阶保持 ==========
n = 3
A_disc = np.eye(n) + A_cont * Ts + A_cont @ A_cont * Ts * Ts / 2.0
B_disc = (np.eye(n) * Ts + A_cont * Ts * Ts / 2.0) @ B_cont

print(f"\nA_disc (离散 5ms):\n{A_disc}")
print(f"\nB_disc:\n{B_disc}")

# ========== LQR 权重 ==========
# Q: 对角线 = [line_error_weight, heading_weight, yaw_rate_weight]
# R: steer 权重
#
# 越大 = 惩罚越重 = 越要紧
# Q[0] ↑ → 巡线更紧
# Q[1] ↑ → 航向更准
# Q[2] ↑ → 转弯更平滑
# R   ↑ → 控制量更小 (更软)

Q = np.diag([10.0, 1.0, 0.1])   # 优先巡线，其次航向，最后平滑
R = np.array([[0.5]])            # steer 惩罚

# ========== 解 Riccati ==========
P = solve_discrete_are(A_disc, B_disc, Q, R)
K = np.linalg.inv(R + B_disc.T @ P @ B_disc) @ (B_disc.T @ P @ A_disc)

print(f"\n========== LQR 增益 K = [K1, K2, K3] ==========")
print(f"K1 (line_error)  = {K[0,0]:.6f}")
print(f"K2 (heading_err°) = {K[0,1]:.6f}")
print(f"K3 (yaw_rate °/s) = {K[0,2]:.6f}")
print(f"\nsteer = -({K[0,0]:.4f} * line_error + {K[0,1]:.4f} * heading_err + {K[0,2]:.4f} * yaw_rate)")
print(f"\n你可以调 Q 矩阵和 R 值重新跑，观察 K 的变化趋势")
print(f"当前 Q = diag({Q.diagonal()}), R = {R[0,0]}")
