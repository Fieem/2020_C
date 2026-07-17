# 坡道巡线小车

## 硬件平台
- MCU: MSPM0G3507
- IMU: ICM42688 (SPI1)
- 循迹: 8 路光电对管 (ADC1_CH5 + 3-8 复用器)
- 编码器: 双路 500PPR + 1:28 减速组 (TIM_G6 / TIM_G12)

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
| 左轮 (TIM_G6) | B26 | B25 |
| 右轮 (TIM_G12) | B20 | B24 |

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
| UART2 TX (printf/VOFA) | B15 |
| UART2 RX | B16 |
| 标定按键 | A30 |
| 启动按键 | A31 |

## 控制架构

```
循迹偏差(line_error) → pid_angle → target_gyro_z → pid_gyro_z → 左右轮速度 → 电机
```

VOFA 串口命令详见 `test.c` 注释。

## 定长行驶
- 轮径 65mm, 减速比 1:28, 500PPR
- 1 米 ≈ 68580 脉冲
- 距离到达后自动停车
