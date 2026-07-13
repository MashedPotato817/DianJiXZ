---
layout: note
title: PWM电机驱动与PID速度控制
---
[← 学习笔记](.) | 最后更新：2026-07-13 16:05
# PWM 电机驱动与 PID 速度控制 —— Q&A

> 基于 WHEELTEC C07A / MSPM0G3507 实物代码分析

---

## Q1：MCU 怎么控制电机转起来？PWM 是什么？

### 硬件链路

```
MSPM0 → GPIO方向信号 → TB6612 → 电机
      → TimerA PWM信号 → TB6612
```

TB6612 是双 H 桥驱动芯片，每个电机需要两根 GPIO 管方向和一根 PWM 管速度。

### GPIO 方向控制

以左轮为例（`motor.c:23-34`）：

```c
if (pwmA > 0) {
    DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);    // AIN2=高
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);  // AIN1=低  → 正转
    DL_Timer_setCaptureCompareValue(PWM_0_INST, pwmA, ...);
} else {
    DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);    // AIN1=高
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);  // AIN2=低  → 反转
    DL_Timer_setCaptureCompareValue(PWM_0_INST, -pwmA, ...);
}
```

TB6612 真值表：

| AIN1 | AIN2 | PWM | 电机状态 |
| ---- | ---- | --- | -------- |
| 0 | 1 | >0 | 正转，速度正比于 PWM |
| 1 | 0 | >0 | 反转 |
| 0 | 0 | - | 刹车 |
| 1 | 1 | - | 刹车 |

### PWM 信号

PWM = 脉冲宽度调制。MSPM0 TimerA 配置为边沿对齐 PWM：

- 周期：8000（80MHz 时钟，8000 个 tick = 10kHz PWM 频率）
- 比较值：0 ~ 8000，决定占空比
- 占空比 = `比较值 / 8000`

例如比较值 4000 → 50% 占空比 → 电机一半速度。

代码中 PWM 上限 7800（97.5%），留了 2.5% 死区避免 H 桥上下管直通。

### 为什么需要死区？

H 桥的每一侧有两个开关管（上管和下管），分别连电源和地：

```
    VCC
     │
   ┌─┴─┐
   │上管│  ← 上管导通 → 电机端 = VCC
   └─┬─┘
     ├──── 电机
   ┌─┴─┐
   │下管│  ← 下管导通 → 电机端 = GND
   └─┬─┘
     │
    GND
```

**直通（Shoot-Through）**：如果上管和下管同时导通，电源直接经过两个管子短路到地——瞬间大电流烧毁驱动芯片。

正常工作时，上下管交替导通。但 MOSFET 开关不是瞬时的——关断有延迟（几十 ns）。如果 PWM 占空比 100%，上管永远开着；某个时刻 MCU 突然翻转方向，下管还没来得及完全关断，上管就开了 → **短路**。

### 死区怎么防止？

把 PWM 上限锁在 7800（周期 8000），永远留 200 个 tick 的"保险丝"：

$$
\text{最小关断时间} = \frac{200}{80\text{ MHz}} = 2.5\ \mu\text{s}
$$

这 2.5μs 确保每周期至少有一段"上下管都关着"的间隙——方向翻转时，旧管子有足够时间彻底关断，新管子才导通。

**直觉类比**：十字路口红绿灯。如果绿灯还没灭红灯就亮了，两个方向的车会撞上。死区就是"全红灯"的短暂间隙。

> TB6612 芯片内部也自带硬件死区控制，但软件再加一层保护更安全——尤其在比赛中，烧驱动 = 直接淘汰。

---

## Q2：什么是 PID 控制？为什么小车需要它？

### 直觉理解

假设你要让车保持 0.3 m/s 前进：

- **开环**：直接给电机一个固定的 PWM 值，"凭感觉"猜这个 PWM 对应多快
  - 问题：电量降低→同样 PWM 速度变慢；上坡→速度变慢；地面摩擦变化→速度变化
- **闭环**：实时测量实际速度，和目标比，有差距就调 PWM
  - PID 就是这个"怎么调"的算法

### PID 三个分量

误差 $e$ = 目标值 - 测量值

**P（比例）**：$\text{Output} = K_p \times e$
误差越大，输出越强。但单纯 P 会有稳态误差。

**I（积分）**：$\text{Output} \gets \text{Output} + K_i \times e \times \mathrm{d}t$
把历史上的误差累积起来，消除稳态误差。
小车启动时要"慢慢使劲"，就是 I 在起作用。

**D（微分）**：$\text{Output} = K_d \times (e - e_{\text{last}}) / \mathrm{d}t$
预测误差变化趋势，提前反向抑制。
刹车防过冲，就是 D 的作用。

### 为什么小车只用 PI，不用 D？

速度控制场景下：
- P 负责快速响应
- I 负责把速度精确稳定到目标值
- D 会放大编码器噪声（速度测量本身有抖动），反而不好

所以代码里只用了 PI。

---

## Q3：代码里的"增量式 PI"和普通 PID 有什么区别？

### 位置式 PID

> 每次输出 = 当前时刻的绝对控制量

$$\text{Output} = K_p \times e + K_i \times \sum e + K_d \times (e - e_{\text{last}})$$

问题：如果控制周期或参数变了，积分项 $\sum e$ 需要重新积累，输出可能跳变。

### 增量式 PID

> 每次输出 = 在上一周期基础上的增量

$$\Delta\text{Output} = K_p \times (e - e_{\text{last}}) + K_i \times e \qquad \text{// D项省略}$$

$$\text{Output} \gets \text{Output} + \Delta\text{Output}$$

对应代码（`control.c:298-307`）：

```c
int Incremental_PI_Left(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;          // static保留上次的值

    Bias = Target - Encoder;                     // 本次误差
    Pwm += Velocity_KP * (Bias - Last_bias)      // 增量 = Kp × 误差变化
        +  Velocity_KI * Bias;                    //       + Ki × 本次误差

    if (Flag_Stop) Pwm = 0;                      // 停车归零
    if (Pwm > 7800) Pwm = 7800;                  // 上限
    if (Pwm < -7800) Pwm = -7800;                 // 下限

    Last_bias = Bias;                             // 保存误差给下次
    return Pwm;
}
```

### 增量式的优势

1. **积分天然限幅**：Pwm 有上下限，超出后自然"积分饱和"消失
2. **切换平滑**：速度目标突然变化时，输出不会跳变
3. **停车安全**：Flag_Stop 直接清零 Pwm

---

## Q4：Kp 和 Ki 怎么调？当前参数的含义

当前参数：$K_p = 400$, $K_i = 300$

### 直觉理解

假设误差 $0.01\text{ m/s}$（$1\text{ cm/s}$）：

$$
\Delta\text{PWM} = 400 \times (0.01 - e_{\text{last}}) + 300 \times 0.01
$$

- 如果误差不变：$\Delta\text{PWM} = 0 + 3 = 3$，每个周期 PWM 加 3
- 误差突然变大 $0.02$：$\Delta\text{PWM} = 400 \times 0.02 + 300 \times 0.02 = 8 + 6 = 14$，加大输出
- 误差突然变小 $0.02$：$\Delta\text{PWM} = 400 \times (-0.02) + 300 \times 0 = -8$，快速减速

### 调参口诀

| 现象 | 调法 |
| ---- | ---- |
| 速度上不去/响应慢 | 加大 $K_p$ |
| 速度忽快忽慢/抖动 | 减小 $K_p$ |
| 能到目标但一直在目标附近波动 | 减小 $K_i$ |
| 速度到不了目标（稳态有差） | 加大 $K_i$ |
| 启动太猛/冲过头 | 减小 $K_p$ + 减小 $K_i$ |

### 周期的影响

控制周期从 10ms 改成 5ms 之后（`control.h` 里 `Frequency = 200Hz`），同样的 $K_p$ / $K_i$ 效果不一样：
- 更快的周期 → 同样的 $K_i$ 每次加得少但加得快 → 积分效果差不多
- 更快的周期 → 同样的 $K_p$ 灵敏度翻倍

不需要改 PID 参数就能工作，这也是增量式的好处之一。

---

## Q5：Set_PWM 里为什么右轮要取反？

```c
// control.c:203
Set_PWM(MotorA.Motor_Pwm, -MotorB.Motor_Pwm);   // 右轮取反！
```

**原因**：左右电机是镜像安装的。

差速转向时：
- 左转：左轮减速、右轮加速 → 需要左右轮同向旋转
- 但如果电机面对面安装，"正向"对左轮是前进，对右轮也是前进，但物理上一个轮子顺时针转才是前进，另一个轮子需要逆时针转

取反让代码层面统一：`Target_Encoder` 的正值都代表"前进方向"，由 `Set_PWM` 里的负号适配机械方向。

---

## Q6：整体控制链路是什么样的？

```
Gray_Mode() 计算目标
  │  Move_X = 线速度目标 (m/s)
  │  Move_Z = 角速度目标 (rad/s)
  ▼
Get_Target_Encoder(Vx, Vz)
  │  运动学逆解：线速度+角速度 → 左轮速度 + 右轮速度
  │  Target_Encoder_Left  = Vx - Vz × 轮距/2
  │  Target_Encoder_Right = Vx + Vz × 轮距/2
  ▼
Incremental_PI_Left / Right
  │  编码器实测速度 vs 目标速度
  │  PI 控制 → PWM 值(-7800 ~ +7800)
  ▼
Set_PWM(Left, -Right)
  │  TB6612 方向 + PWM 占空比
  ▼
电机转动 → 编码器反馈 → 回到第一步
```

全部在 5ms 中断里完成，一个周期跑一遍。

---

> 最后更新：2026-07-13 16:05
