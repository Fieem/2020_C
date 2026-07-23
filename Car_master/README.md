# 坡道巡线小车

## 硬件平台
- MCU: MSPM0G3507 (80MHz)
- IMU: ICM42688 (SPI1)
- 循迹: 8 路光电对管 (ADC1_CH5 + 3-8 复用器)
- 舵机: 前轮转向 (TIM_A0 CH2, B4)
- 编码器: 双路 500PPR + 1:28 减速组

## 引脚分配

### 电机
| 功能 | 引脚 |
|------|------|
| 左电机 PWM | B10 (TIM_G0) |
| 左电机 DIR | B11 |
| 右电机 PWM | A26 (TIM_G7) |
| 右电机 DIR | A27 |

### 编码器
| 功能 | A 相 | B 相 |
|------|------|------|
| 左轮 (TIMG8 QEI) | B6 (CH1) | B7 (CH2) |
| 右轮 (A相 EXTI) | B20 | B24 |

### 舵机
| 功能 | 引脚 |
|------|------|
| 舵机 PWM | B4 (TIM_A0 CH2) |

### 传感器
| 功能 | 引脚 |
|------|------|
| ICM42688 CS | B19 |
| ICM42688 SCK | B23 (SPI1) |
| ICM42688 MOSI | B22 (SPI1) |
| ICM42688 MISO | B21 (SPI1) |
| ADC 输入 | B18 (ADC1_CH5) |
| ADC 复用 AD2/AD1/AD0 | B8 / B9 / B13 |
| 电池电压 ADC | A22 (ADC0_CH7) |

### 调试
| 功能 | 引脚 |
|------|------|
| UART2 TX (VOFA/printf) | B15 |
| UART2 RX | B16 |
| 蜂鸣器 | A14 |
| 标定按键 | A30 |
| 启动按键 | A31 |

## 定时器分配

| 定时器 | 用途 |
|--------|------|
| TIM_A0 | PWM 舵机 B4 |
| TIM_A1 | 空闲 |
| TIM_G0 | PWM 左电机 B10 |
| TIM_G6 | PIT 按键扫描 (5ms) |
| TIM_G7 | PWM 右电机 A26 |
| TIM_G8 | QEI 左轮编码器 |
| TIM_G12 | PIT 5ms 控制中断 |

## 状态机

```
            ┌─────────────────────────────────────┐
            │            RESTART                  │
            └───────────┬─────────────────────────┘
                        ▼
  ┌──────────────────────────────────────────────────┐
  │                STATE_DRIVE                        │
  │  舵机 PID 循迹 + 阿克曼差速                        │
  │  check_stop_line                                  │
  │  ── 丢线 500ms ──→ STATE_TURN                    │
  └──────────────────────────────────────────────────┘
                        │
                        ▼
  ┌──────────────────────────────────────────────────┐
  │                STATE_TURN                         │
  │  舵机 13° 固定  左轮 25  右轮 7                    │
  │  check_stop_line                                  │
  │  ── Yaw >= 85° ──→ STATE_AFTER_TURN             │
  │  ── 横线停车 ──→ STATE_STOP                      │
  └──────────────────────────────────────────────────┘
                        │
                        ▼
  ┌──────────────────────────────────────────────────┐
  │              STATE_AFTER_TURN                     │
  │  舵机 PID 循迹 + 阿克曼差速（同 DRIVE）             │
  │  check_stop_line                                  │
  │  ── 横线停车 ──→ STATE_STOP                      │
  └──────────────────────────────────────────────────┘
                        │
                        ▼
  ┌──────────────────────────────────────────────────┐
  │                STATE_STOP                         │
  │  速度 = 0  舵机 = 0°  PID 复位                    │
  └──────────────────────────────────────────────────┘
```

### 状态转换条件

| 转换 | 条件 |
|------|------|
| DRIVE → TURN | 连续丢线 500ms |
| TURN → AFTER_TURN | `fabsf(Yaw_TotalAngle) >= 85°` |
| TURN/AFTER_TURN/DRIVE → STOP | `check_stop_line()` 连续 10 帧检测到横线 |

## 舵机参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `SERVO_PWM_CHANNEL` | `PWM_TIM_A0_CH2_B4` | 舵机 PWM 引脚 |
| `SERVO_FREQ_HZ` | 50 | 标准舵机频率 |
| `SERVO_CENTER_DUTY` | 440 | 实测中位 PWM |
| `SERVO_ANGLE_MIN/MAX` | ±13° | 机械限位 |
| `SERVO_ANGLE_DEADZONE` | 7° | 死区 |

## 阿克曼几何

| 参数 | 值 | 说明 |
|------|-----|------|
| `WHEEL_BASE_MM` | 120 | 轴距 |
| `WHEEL_TRACK_MM` | 145 | 轮距 |
| `ACKERMANN_MAX_RATIO` | 0.80 | 内侧轮最低保留 20% |

差速公式：`diff = speed_set × (轮距 / 2·轴距) × tan(舵角)`，限幅后左轮 +diff、右轮 -diff。

## VOFA 串口命令

| 命令 | 说明 |
|------|------|
| `START=1!` | 发车（start_flag++） |
| `RESTART=1!` | 复位全部状态，回到 DRIVE 等待发车 |
| `TIME=<秒>!` | 根据总路程/目标时间设定 `speed_set` |
| `CAL=1!` | 灰度标定触发 |
| `BAT=1!` | 读取电池电压 |
| `SHOW=1!` | 打印当前 PID 参数 |
| `P1=<float>!` | `pid_speed_left.Kp` |
| `I1=<float>!` | `pid_speed_left.Ki` |
| `D1=<float>!` | `pid_speed_left.Kd` |
| `P2=<float>!` | `pid_speed_right.Kp` |
| `I2=<float>!` | `pid_speed_right.Ki` |
| `D2=<float>!` | `pid_speed_right.Kd` |

## 定长行驶

- 轮径 65mm, 减速比 1:28, 500PPR
- 1 米 ≈ 68580 脉冲
- 全程脉冲 `total_pulses = 41135`
- `TIME=<秒>` 命令计算 `speed_set = total_pulses / (time × 200)`

## RESTART 复位内容

`start_flag`, `drive_state`, `task_number`, `distance_accum`, `servo_accum_angle`, `turn_start_yaw` 全部清零，舵机回中位。需重新发 `START=1!` 才能启动。
