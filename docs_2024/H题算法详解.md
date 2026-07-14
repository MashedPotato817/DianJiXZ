---
layout: note
title: H题算法详解
---

[← 2024归档](./) | 最后更新：2026-07-13 16:05

# H 题核心算法详解

> 三个算法覆盖 T4 全部需求：半圆弧巡线、无标记过点检测、路段序列状态机

---

## 算法一：半圆弧纯追踪巡线

### 问题

H 题场地有两段半径 40cm 的半圆弧。和弯道巡线不同——弯道曲率未知、随线走就行；半圆弧曲率恒定（1/0.4 = 2.5 m⁻¹），弧线长度可预判（π × 40 ≈ 126cm）。

### 为什么纯追踪法直接能用

纯追踪法公式（详见 ques3.md Q6）：

$$
\text{curvature} = \frac{2y}{L^2 + y^2}, \quad \omega = -K_{steer} \times v \times \text{curvature}
$$

弧线上灰度偏差 $y$ 稳定在某一侧（取决于左转弧还是右转弧），所以 curvature 基本恒定——角速度输出也是常数。这意味着弧线巡线**非常稳定**，不像普通弯道需要不断修正。

### 参数建议

| 参数 | 弧线值 | 直线值 | 原因 |
|------|--------|--------|------|
| `BASE_SPEED` | 200 mm/s | 300 mm/s | 弧线降速防冲出 |
| `STEER_GAIN` | 1.70 | 1.70 | 同一个增益够用 |
| `MAX_TURN` | 2.2 rad/s | 2.2 rad/s | H 题弧半径大，不需要猛转 |

### 弧线终点的预判

弧长已知 126cm，用编码器累计里程：

```
弧线起点 → 累加里程 → 达到 ~120cm 时进入"终点警戒"
```

警戒状态下，一旦检测到**灰度从持续偏左/右 → 突然居中**（过渡特征），立即判为弧线终点 → 推进状态机。如果里程到 140cm 还没过渡 → 强制推进（兜底）。

---

## 算法二：无标记过点检测

### 问题

H 题场地 ABCD 四个顶点**没有物理标记**。不能像之前的赛道靠"8 路全黑 ≥7"检测十字/过点。只能用弧线↔直道之间的**灰度特征过渡**来识别。

### 过渡特征

弧线和直道切换时，8 路灰度会有一个明显的变化：

```
弧线 → 直道：偏差从 ±30mm 突然回到 0~5mm 附近
直道 → 弧线：偏差从 0~5mm 突然跳到 ±20mm+
```

### 检测算法

```c
#define TRANSITION_WINDOW_TICKS  5U   // 连续5个周期确认
#define TRANSITION_THRESHOLD_MM  18   // 偏差变化超过18mm算过渡

typedef enum {
    SEG_TYPE_NONE = 0,
    SEG_TYPE_ARC,
    SEG_TYPE_STRAIGHT
} SegType_t;

static SegType_t g_last_seg_type = SEG_TYPE_NONE;
static uint8_t g_transition_ticks = 0;

uint8_t Gray_DetectSegmentTransition(void)
{
    float pos_mm = Gray_Line_Pos_mm;  // 当前偏差

    SegType_t cur_type;
    if (fabsf(pos_mm) < 8.0f) {
        cur_type = SEG_TYPE_STRAIGHT;   // 偏差小 → 直道
    } else {
        cur_type = SEG_TYPE_ARC;        // 偏差大 → 弧线
    }

    if (cur_type != g_last_seg_type) {
        g_transition_ticks++;
        if (g_transition_ticks >= TRANSITION_WINDOW_TICKS) {
            g_transition_ticks = 0;
            g_last_seg_type = cur_type;
            return 1;   // 过渡确认！
        }
    } else {
        g_transition_ticks = 0;  // 复位，防噪声
    }

    return 0;
}
```

### 为什么不能只靠单次判断

灰度传感器精度 ~5mm，弧线末端可能偏差在 10mm 徘徊一两个周期。单次判断容易误触发。窗口确认 + 滞后比较消除抖动。

### 里程计辅助

纯靠灰度过渡有死角——如果弧线末端恰好有个小弯，灰度变化不够剧烈。编码器里程计兜底：

```c
if (g_odom_segment_mm > g_segment_expected_mm * 1.1f) {
    // 里程超预期 10% 了，灰度还没过渡 → 强制判为过点
    ForceTransition();
}
```

---

## 算法三：T4 位姿导航状态机（队友实装）

队友 `6e8be58` ~ `69dd12d` 实现了完整的 T4 位姿引导导航。比之前的设计更精细——不是简单的"巡线→过点→下一段"，而是**位姿推算 + 航向对齐 + 对角线直冲 + 重捕获**的多状态流水线。

### 场地坐标系

从代码中反推的场地布局（坐标单位：米）：

```
        A (0, 0)                 B (1.0, 0)
           ┌──────────────────────┐
           │                      │
           │    220cm × 120cm     │
           │                      │
           └──────────────────────┘
        D (0, -0.80)            C (1.0, -0.80)
```

位姿推算起点：A 点 (0, 0)，初始航向 180°（朝下）。

### 九个状态

```c
typedef enum {
    GRAY_TASK4_LINE_AD = 0,      // 巡线 A→D
    GRAY_TASK4_ALIGN_D_TANGENT,  // 对齐 D 点切线方向 (0°)
    GRAY_TASK4_ALIGN_D_TO_B,     // 对齐 D→B 对角线方向 (35°)
    GRAY_TASK4_BRIDGE_D_TO_B,    // 直冲 D→B（航向保持 35°）
    GRAY_TASK4_LINE_BC,          // 巡线 B→C
    GRAY_TASK4_ALIGN_C_TANGENT,  // 对齐 C 点切线方向 (180°)
    GRAY_TASK4_ADVANCE_C,        // 过 C 后前进 200mm
    GRAY_TASK4_ALIGN_C_TO_A,     // 对齐 C→A 对角线方向 (145°)
    GRAY_TASK4_BRIDGE_C_TO_A,    // 直冲 C→A（航向保持 145°）
    GRAY_TASK4_ALIGN_A_TANGENT   // 回到 A 后对齐初始方向 (180°)
} GrayTask4State_t;
```

### 状态流转图

```
LINE_AD ──丢线+位姿到达 D──→ ALIGN_D_TANGENT ──航向=0°对准──→
ALIGN_D_TO_B ──航向=35°对准──→ BRIDGE_D_TO_B ──看到线──→
LINE_BC ──丢线+位姿到达 C──→ ALIGN_C_TANGENT ──航向=180°对准──→
ADVANCE_C ──前进了200mm──→ ALIGN_C_TO_A ──航向=145°对准──→
BRIDGE_C_TO_A ──看到线+位姿到A──→ ALIGN_A_TANGENT ──航向=180°对准──→
LINE_AD（下一圈）
```

### 关键参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `GRAY_TASK4_D_TO_B_DEG` | 35° | D→B 对角线航向 |
| `GRAY_TASK4_C_TO_A_DEG` | 145° | C→A 对角线航向 |
| `GRAY_TASK4_C_ADVANCE_MM` | 200mm | 过 C 后额外前进距离 |
| `GRAY_TASK4_BRIDGE_SPEED_MM_S` | 280 mm/s | 对角线直冲速度 |
| `GRAY_TASK4_BRIDGE_MAX_MM` | 1600mm | 对角线最大冲距 |
| `GRAY_TASK4_ALIGN_ERR_DEG` | 4° | 航向对齐容差 |
| `GRAY_TASK4_ALIGN_STABLE_TICKS` | 5 | 连续 5 周期误差<4°才算对准 |
| `GRAY_TASK4_LOST_CREEP_MM_S` | 80 mm/s | 丢线慢速蠕动速度 |
| `GRAY_TASK4_POINT_TOL_M` | 0.25m | 位姿到达点判据 |

### 航向对齐

不是简单"转到目标角度就切状态"——有**稳定窗口**：

```c
// control.c:641-664
heading_err = 目标航向 - 当前位姿航向;
turn_speed = 0.075 * heading_err - 0.010 * wz;  // PD

if (|heading_err| <= 4°) {
    stable_ticks++;
    if (stable_ticks >= 5) return ALIGNED;  // 连续5周期稳定
} else {
    stable_ticks = 0;
}
if (align_ticks >= 500) 超时强制推进;  // 防卡死
```

### 对角线直冲（BRIDGE 模式）

巡线模式下丢了线 → 不是原地旋转！而是用 **JY62 yaw 保持目标航向** 高速直冲：

```c
// control.c:677-691
bridge_distance += 280mm/s × 0.005s;   // 累积冲距
heading_err = 目标航向 - 位姿航向;
turn_speed = 0.075 * heading_err - 0.010 * wz; // PD航向保持

if (bridge_distance >= 1600mm) 超时放弃;
// 看到黑线 + 重捕获2次确认 → 切换到巡线状态
```

### 位姿推算

启动时位姿初始化为 A(0, 0)，航向 180°。每 5ms 用 JY62 yaw + 编码器速度更新：

```c
// control.c:345-368
pose_heading += wz × 0.005;   // 陀螺仪积分
pose_x += v × cos(heading) × 0.005;
pose_y += v × sin(heading) × 0.005;
```

到达目标点时做**位姿快照**（`PoseSnap`）——把当前推算位姿修正为已知坐标，切断累积误差。
