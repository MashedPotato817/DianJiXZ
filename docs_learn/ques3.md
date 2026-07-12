---
layout: note
title: 8路灰度巡线算法详解
---
[← 学习笔记](.) | 最后更新：2026-07-12
# 8路灰度巡线算法详解 —— Q&A

> 基于 WHEELTEC C07A 当前固件 `control.c` 的 Gray_Mode()

---

## Q1：8路灰度传感器是怎么读出数据的？

### 硬件结构

8 个独立的光电传感器，一字排开，间距 10mm，通过 3-8 模拟开关（类似 74HC4051）分时复用一根 GPIO 读取：

```
传感器0   传感器1   传感器2   ...   传感器7
  ↓         ↓         ↓               ↓
AD2/AD1/AD0 三根地址线 → 选择通道 → 一路输出 → MSPM0 GPIO
```

### 读取流程（`Gray_Read_All()`）

```c
void Gray_Read_All(void)
{
    for (i = 0; i < 8; i++) {
        Gray_Select_Channel(i);             // 三根地址线选通道
        delay_us(50);                       // 等信号稳定
        Gray_Raw[i] = GPIO_Read_Pin(OUT);   // 读电平：高=白(1)，低=黑(0)
        Gray_Data[i] = Gray_ToBlack(raw);   // 转换为 1=黑, 0=白
    }
}
```

`Gray_ToBlack()` 把电平反转：传感器输出低电平表示检测到黑线 → `Gray_Data[i] = 1`。

最终得到 8 个 0/1 值，例如：

```
位置:   0   1   2   3   4   5   6   7
Data:   0   0   1   1   1   0   0   0
                    ↑黑线在这里
```

---

## Q2：怎么从 8 个 0/1 得到黑线的精确位置？

### 加权质心法（Weighted Centroid）

不是简单地"黑线在第3个传感器"，而是用加权平均算出亚像素级的位置（mm）：

```c
// control.c:110-115
for (i = 0; i < 8; i++) {
    if (Gray_Data[i]) {
        pos_sum += Gray_Pos_mm[i];     // 黑传感器的坐标求和
        black_count++;                 // 有几个传感器看到黑
    }
}
Gray_Line_Pos_mm = pos_sum / black_count;  // 加权平均 = 质心
```

每个传感器的 `Gray_Pos_mm`（距中心的实际坐标）：

```
传感器:  0      1      2      3      4      5      6      7
坐标:  -35    -25    -15     -5     +5    +15    +25    +35 mm
```

### 举例

黑线在传感器 2、3、4 下面：

$$
\text{pos\_sum} = (-15) + (-5) + (+5) = -15
$$

$$
\text{black\_count} = 3
$$

$$
\text{Gray\_Line\_Pos\_mm} = \frac{-15}{3} = -5\text{ mm} \quad \to \text{黑线在中心偏左 5mm}
$$

如果只有传感器 3 看到：

$$
\text{Gray\_Line\_Pos\_mm} = \frac{-5}{1} = -5\text{ mm} \quad \to \text{同样结果，但置信度低}
$$

### 为什么比 CCD 好算？

8路灰度是**数字量**（0/1），不需要 CCD 的动态阈值和跳变沿找线。传感器模块内部已经做了比较器，直接给 0/1 结果。但代价是分辨率低（只有8个点）。

---

## Q3：三档变速策略是怎么决策的？

### 代码逻辑

```c
// 默认：直线速度
Move_X = GRAY_BASE_SPEED_MM_S / 1000.0f;   // 300 mm/s → 0.3 m/s

// 根据黑线位置和数量降档
abs_pos = |Gray_Line_Pos_mm|;               // 偏离中心的绝对值
if ((black_count >= 5) || (abs_pos >= 22mm)) {
    Move_X = GRAY_CURVE_SPEED_MM_S / 1000.0f;  // 弯道: 185 mm/s
}
else if (abs_pos >= 10mm) {
    Move_X = GRAY_MID_SPEED_MM_S / 1000.0f;    // 中等偏移: 240 mm/s
}
```

### 决策表

| 条件 | 含义 | 速度 |
| ---- | ---- | ---- |
| 黑线在中心 $\pm 10\text{mm}$ 内 | 直线/微偏 | 300 mm/s |
| 黑线偏离中心 $10 \sim 22\text{mm}$ | 缓弯 | 240 mm/s |
| 黑线偏离超过 $22\text{mm}$ | 急弯 | 185 mm/s |
| $\ge 5$ 个传感器同时看到黑线 | 可能是直角弯/交叉 | 185 mm/s |
| 十字/全黑 ($\ge 7$ 个) | 直冲 | 310 mm/s（特殊逻辑，见 Q4） |
| 8路全白（丢线） | 丢线 | 0 mm/s（原地旋转找线，见 Q5） |

---

## Q4：十字路口/大面积黑线怎么处理？

### 问题

过十字路口时，8 个传感器全部看到黑线（$\text{black\_count} \ge 7$），无法判断线在哪个方向。如果正常巡线算法去算，会得到一个中心附近的位置，然后车会**直走冲过十字**——这正好是我们想要的，但需要防止误触发。

### 十字直冲状态机

```c
// 条件：至少7个传感器黑 + 连续2次检测 + 冷却时间已过
if (black_count >= 7) {
    cross_detect_count++;
    if ((cross_detect_count >= 2) && (cross_cooldown_ticks == 0)) {
        // 触发！32个周期直冲（32×5ms = 160ms）
        cross_run_ticks = 32;
        cross_cooldown_ticks = 60;    // 60个周期冷却（300ms）
    }
}

// 直冲期间：固定高速直行，不走巡线逻辑
if (cross_run_ticks > 0) {
    cross_run_ticks--;
    Move_X = 310 mm/s;    // 最高速
    Move_Z = 0;           // 不转向
    return;               // 跳过所有巡线逻辑
}
```

### 防抖机制

| 机制 | 作用 |
| ---- | ---- |
| 需连续 2 次检测 | 过滤噪声/瞬时干扰 |
| 冷却 60 周期（300ms） | 过完十字后不会立刻再次触发 |
| 直冲 32 周期（160ms） | 足够完全通过十字 |

---

## Q5：丢线了怎么办？

### 丢线判定

```c
if (black_count == 0)          // 8个传感器全白
```

### 丢线恢复策略

```c
// 记住上次黑线在哪边
if (Gray_Line_Pos_mm > 0)     last_search_dir = -1;  // 上次在右边 → 左转找
if (Gray_Line_Pos_mm < 0)     last_search_dir =  1;  // 上次在左边 → 右转找

// 丢线时：停下 + 原地旋转
Move_X = 0 mm/s;                                   // 不动
Move_Z = last_search_dir × GRAY_LOST_TURN_SPEED;   // 1.8 rad/s 旋转
```

逻辑很简单：记得黑线最后在哪一侧，丢线后往那边转，转了就能重新看到线。看到线后立即恢复到正常巡线。

---

## Q6：转向控制是怎么算的？曲率公式是什么？

### 纯追踪法（Pure Pursuit 简化版）

核心思想：传感器在车轮前方 `GRAY_SENSOR_FORWARD_MM = 150mm` 处，测到的偏差 $y$ 可以用来估算路径的曲率。

```c
// control.c:171-174
y_m = Gray_Line_Pos_mm / 1000.0f;                        // 偏差(m)
lookahead_m = GRAY_SENSOR_FORWARD_MM / 1000.0f;           // 前视距离(m)
curvature = (2.0f * y_m) / (lookahead_m² + y_m²);         // 曲率

Move_Z = -STEER_GAIN × Move_X × curvature;               // 角速度 = 增益 × 线速度 × 曲率

// 限幅
if (Move_Z > 3.6) Move_Z = 3.6;     // 最大角速度 3.6 rad/s ≈ 206°/s
if (Move_Z < -3.6) Move_Z = -3.6;
```

曲率公式的数学形式：

$$
\text{curvature} = \frac{2y}{L^2 + y^2}
$$

其中 $y$ 为偏差，$L$ 为前视距离（0.15m）。

角速度计算：

$$
\omega = -K_{\text{steer}} \times v \times \text{curvature}
$$

### 几何直觉

```
          传感器位置（前方150mm）
              │
              │ 偏差 y
              │◄───→
    车  ──────●────────────────── 目标路径
              ↑
            车轮中心

曲率 ≈ 2 × 偏差 / 前视距离²
```

偏差越大 → 曲率越大 → 转角越大。前视距离 150mm 决定了："提前"150mm 看路。

### 参数调优

| 参数 | 当前值 | 作用 |
| ---- | ------ | ---- |
| `STEER_GAIN` | 1.70 | 转向灵敏度，越大越猛 |
| `MAX_ANGULAR_SPEED` | 3.6 rad/s | 防止原地打转 |
| `SENSOR_FORWARD_MM` | 150 | 前视距离，决定预判距离 |

- 转向不足（过弯冲出）→ 加大 STEER_GAIN
- 转向过度（S 形摆动）→ 减小 STEER_GAIN
- 弯道反应慢 → 加大 FORWARD（看得更远）

---

## Q7：Gray_Mode 整体流程图

```
5ms 中断进入 Gray_Mode()
│
├─ Gray_Read_All()          # 读8路传感器
├─ 算 black_count           # 看到黑线的传感器数
│
├─ 十字冷却倒计时 --
│
├─ 正在直冲中？ ──── 是 ──→ 高速直行, return
│
├─ black_count ≥ 7？
│   └─ 是 → 连续2次？ + 冷却结束？ → 触发直冲, return
│
├─ black_count == 0？丢线！
│   └─ 是 → 原地旋转找线, return
│
├─ 加权质心 → Gray_Line_Pos_mm
├─ 三档速度选择
├─ 曲率计算 → Move_Z
├─ Get_Target_Encoder(Move_X, Move_Z)   # 运动学逆解
│
└─ return → ISR继续执行PI → Set_PWM
```

---

> 最后更新：2026-07-12
