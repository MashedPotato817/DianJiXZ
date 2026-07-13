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

## 算法三：路段序列状态机

### 数据结构

```c
typedef enum {
    POINT_A = 0,
    POINT_B,
    POINT_C,
    POINT_D
} RoutePoint_t;

typedef enum {
    SEG_ARC_AB = 0,    // A→B 顶弧（左转弧）
    SEG_STRAIGHT_BC,   // B→C 右直
    SEG_ARC_CD,        // C→D 底弧（右转弧）
    SEG_STRAIGHT_DA,   // D→A 左直
    SEG_STOP
} RouteSegment_t;

typedef struct {
    RouteSegment_t seg;
    RoutePoint_t   end_point;        // 终点
    float          speed_mm_s;       // 目标速度
    float          expected_len_mm;  // 预期里程（直线用）
} RouteEntry_t;
```

### T4 路段序列

```c
static const RouteEntry_t g_route_t4[] = {
    { SEG_STRAIGHT_DA,  POINT_D, 300, 60 },   // A→D 左直, 60cm
    { SEG_ARC_CD,       POINT_C, 200, 126 },  // D→C 底弧, 126cm
    { SEG_STRAIGHT_BC,  POINT_B, 300, 60 },   // C→B 右直, 60cm (反向)
    { SEG_ARC_AB,       POINT_A, 200, 126 },  // B→A 顶弧, 126cm (反向)
    { SEG_STOP,         0,       0,   0   }
};
```

> 注：这里假设 A→C 的路径是 A→D→C。实际路径看 pic1 场地示意图确认。

### 状态机主循环（每 5ms 调一次）

```c
static uint8_t  g_current_seg = 0;
static uint8_t  g_lap_count = 0;
static float    g_odom_segment_mm = 0;

void Gray_RouteTick(void)
{
    const RouteEntry_t *seg = &g_route_t4[g_current_seg];

    if (seg->seg == SEG_STOP) {
        Stop();
        TaskComplete();  // 4圈完成
        return;
    }

    // 1. 执行当前路段
    switch (seg->seg) {
    case SEG_ARC_AB:
    case SEG_ARC_CD:
        Gray_LineFollow(seg->speed_mm_s);  // 纯追踪巡线
        break;
    case SEG_STRAIGHT_BC:
    case SEG_STRAIGHT_DA:
        Gray_StraightRun(seg->speed_mm_s);  // 直行，偏差修正
        break;
    }

    // 2. 累计里程
    float v_mps = MotorA.Current_Encoder;  // 或左右平均
    g_odom_segment_mm += v_mps * 1000.0f * 0.005f;

    // 3. 检测过点条件
    uint8_t transition = Gray_DetectSegmentTransition();
    uint8_t odom_ok = (g_odom_segment_mm > seg->expected_len_mm * 0.9f);

    if (transition && odom_ok) {
        // 双重确认 → 过点
        LED_Notify(seg->end_point);   // 声光提示
        g_odom_segment_mm = 0;        // 清零路段里程

        // 推进到下一段
        g_current_seg++;
        if (g_route_t4[g_current_seg].seg == SEG_STOP) {
            g_lap_count++;
            if (g_lap_count >= 4) {
                // 4 圈完成，停在当前位置
            } else {
                g_current_seg = 0;  // 循环回第一段
            }
        }
    }
}
```

### 为什么 A→C→B→D→A 和 A→B→C→D→A 是两套数组

T4 的路径是任务 (3) 的路线——**A→C→B→D→A**。但 A→C 的第一步到底走"左直 D→A 的反向"还是"顶弧 A→B 再往下"，取决于 pic1 中弧线的具体朝向。上面按"先走左直到 D"假设。如果 pic1 显示弧线在顶部和底部（不是左右两侧），则路段序列要调。

但不管具体朝向——路段序列只是数据。**换了 T2（顺时针）只要换一套 `g_route_t2[]` 数组**，控制代码完全不变。这就是状态机 + 数据驱动的好处。
