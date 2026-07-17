---
layout: note
title: 云台 PWM 驱动详解
---

[← 学习笔记](.) | 最后更新：2026-07-14 01:16

# 云台 PWM 驱动详解 —— Q&A

> 基于队友 `gimbal.c/h` 实物代码分析。TIMA0 双通道 PWM，PA8 yaw + PA1 pitch。

---

## Q1：硬件是怎么接的？

| 轴 | 引脚 | 定时器通道 | 舵机类型 | 控制方式 |
|----|------|-----------|---------|---------|
| **Yaw** | PA8 | TIMA0 CCP0 | 连续旋转舵机 | 速度控制（脉宽偏离中位 = 转速） |
| **Pitch** | PA1 | TIMA0 CCP1 | 位置舵机 | 角度控制（脉宽直接对应角度） |

两个舵机共用一个 TIMA0，各自独立比较通道。PWM 频率 50Hz（舵机标准）。

---

## Q2：PWM 参数怎么算的？

### 时钟链

```
BUSCLK (80MHz) → ÷8 (分频) → 10MHz → ÷8 (prescale+1) → 1.25MHz
```

$$
\text{Timer tick} = \frac{1}{1.25\text{MHz}} = 0.8\mu s
$$

### 周期

```
周期 = 25000 ticks × 0.8µs = 20000µs = 20ms → 50Hz
```

### 脉宽换算

```c
// gimbal.c:41-45 — µs 转 timer ticks
ticks = pulse_us × 5 / 4 = pulse_us × 1.25

例: 1500µs → 1875 ticks  (中位)
    1000µs → 1250 ticks  (最小)
    2000µs → 2500 ticks  (最大)
```

---

## Q3：API 怎么用？

### Yaw — 连续旋转舵机，速度控制

```c
Gimbal_SetYawSpeed(50);   // 半速右转
Gimbal_SetYawSpeed(-100); // 全速左转
Gimbal_SetYawSpeed(0);    // 停止
Gimbal_StopYaw();         // 同上
```

内部：`脉宽 = 1500 + percent × (2000 - 1500) / 100 = 1500 + percent × 5`

±100 映射到 1000~2000µs，0 = 1500µs = 停止。

### Pitch — 位置舵机，角度控制

```c
Gimbal_SetPitchDeg(90);   // 水平
Gimbal_SetPitchDeg(0);    // 最上
Gimbal_SetPitchDeg(180);  // 最下
```

内部：`脉宽 = 1000 + degree × (2000 - 1000) / 180`

### 直接脉宽（校准用）

```c
Gimbal_SetYawPulseUs(1500);   // 中位停转
Gimbal_SetPitchPulseUs(1250); // 自定义角度
```

范围 1000~2000µs，超出自动钳位。

---

## Q4：初始化和自检

`Gimbal_Init()` 流程：

1. 开 TIMA0 时钟电源 → 配置 PA8/PA1 外设输出
2. 设 PWM：边沿对齐、25000 ticks、立即启动
3. 两路 CC 输出：初始低、非反相、立即更新
4. **默认位姿**：yaw 停止 + pitch = 90°

两个编译期开关（默认都关）：

| 宏 | 作用 |
|----|------|
| `GIMBAL_BOOT_TEST_ENABLE` | 上电 yaw 左右摆动自检 |
| `GIMBAL_GPIO_DEBUG_ENABLE` | PA8 变 GPIO 闪灯——测引脚焊接，**永久卡 while(1)** |

---

## Q5：和 K230 视觉跟踪对接

```
K230 发色块坐标 → MSPM0 收帧 → 算偏差 → 控舵机
```

### 跟踪闭环伪代码

```c
K210_VisionFrame frame;
if (K210_Link_GetLatestVision(&frame) && frame.valid) {
    int16_t x_err = (int16_t)frame.x - 160;  // 画面中心 X=160
    int16_t y_err = (int16_t)frame.y - 120;  // 画面中心 Y=120

    // yaw: 连续旋转，速度 ∝ 偏差
    Gimbal_SetYawSpeed(x_err * GIMBAL_TRACK_KP);

    // pitch: 位置舵机，角度叠加偏差
    static int16_t pitch = 90;
    pitch += y_err * GIMBAL_TRACK_KP * 0.1f;
    CLAMP(pitch, 0, 180);
    Gimbal_SetPitchDeg(pitch);
} else {
    Gimbal_StopYaw();  // 没目标 → 停转
}
```

---

## Q6：连续旋转 vs 位置舵机

| | 连续旋转 (yaw) | 位置舵机 (pitch) |
|------|-----------|----------|
| 1500µs | **停止** | 90°（中位） |
| 1000µs | 全速左转 | 0° |
| 2000µs | 全速右转 | 180° |
| 控制含义 | **速度** | **角度** |
| 需反馈 | 是（视觉闭环） | 否（开环够用） |
| 旋转范围 | 无限制 | 0~180° |
