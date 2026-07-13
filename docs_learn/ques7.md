---
layout: note
title: 陀螺里程计详解
---

[← 学习笔记](.) | 最后更新：2026-07-13 16:05

# 陀螺里程计（Gyro Odometry） —— Q&A

> 基于 WHEELTEC C07A JY62_GapBridge 固件，队友 `43a7625` 实现

---

## Q1：陀螺里程计是什么？为什么需要它？

### 问题

编码器能告诉你"轮子转了多少"，但推不出"车在场地上的绝对位置"。断桥期间灰度全白——巡线失效，你不知道自己冲了多远、偏了没有。H 题的 ABCD 四个顶点没有物理标记，你怎么知道"到了"？

### 方案

**里程计（Odometry）**：用编码器速度 + yaw 角，每个控制周期推算车的位姿 (x, y, θ)。纯惯性推算，不依赖外部标记。

```
编码器速度 → 线速度 v
JY62 yaw   → 朝向角 θ
         ↓
每 5ms 更新一次位姿:
  x += v × cos(θ) × dt
  y += v × sin(θ) × dt
  θ = JY62.yaw
```

---

## Q2：公式怎么推出来的？

### 差速小车的运动学

已知左右轮速度 $V_L$、$V_R$（m/s），轮距 $W$：

$$
v = \frac{V_L + V_R}{2} \qquad \text{（线速度）}
$$

$$
\omega = \frac{V_R - V_L}{W} \qquad \text{（角速度）}
$$

### 位姿更新（Dead Reckoning）

设上一周期的位姿为 $(x_{k-1}, y_{k-1}, \theta_{k-1})$，控制周期 $\Delta t = 0.005s$：

$$
\begin{aligned}
\theta_k &= \text{JY62.yaw} \\
x_k &= x_{k-1} + v_k \times \cos(\theta_k) \times \Delta t \\
y_k &= y_{k-1} + v_k \times \sin(\theta_k) \times \Delta t
\end{aligned}
$$

> **为什么 θ 直接用 JY62 的 yaw 而不是积分 ω？** 因为陀螺仪角速度积分会漂移——JY62 芯片内部已经做了姿态解算和漂移补偿，直接拿 yaw 比自己做积分准得多。

### 里程累积

断桥直冲的累积距离：

$$
\text{distance} = \sum v_k \times \Delta t
$$

当 `distance ≥ 280mm`（预设断桥最大距离），还没看到线 → 放弃，切换丢线恢复。

---

## Q3：实际代码怎么写的？

### 数据结构

```c
typedef struct {
    float x_m;        // 当前位置 X (m)
    float y_m;        // 当前位置 Y (m)
    float yaw_deg;    // 当前朝向 (°)
    float total_dist_m;  // 累计里程 (m)
} OdomPose;
```

### 核心更新函数（5ms 中断里调用）

```c
void Odom_Update(float v_mps, float yaw_deg, float dt)
{
    static OdomPose pose;

    pose.yaw_deg = yaw_deg;
    float yaw_rad = yaw_deg * 3.14159f / 180.0f;

    pose.x_m += v_mps * cosf(yaw_rad) * dt;
    pose.y_m += v_mps * sinf(yaw_rad) * dt;
    pose.total_dist_m += fabsf(v_mps) * dt;
}
```

### 断桥入口记录参考位姿

```c
// control.c:434-441
if (丢线 && 之前稳定巡线 && JY62在线) {
    bridge_active = 1;
    bridge_yaw_ref_deg = yaw_deg;     // 锁定参考朝向
    bridge_distance_mm = 0;           // 清零里程
    Odom_SaveOrigin();                // 记录断桥起点坐标
}
```

### 断桥期间航向保持 + 里程判断

```c
// control.c:412-427
bridge_distance_mm += v_mps * 1000.0f * 0.005f;  // 累积 mm

if (bridge_distance_mm < 280) {
    yaw_err = 参考yaw - 当前yaw;
    Move_Z = 0.075 * yaw_err - 0.010 * wz;   // PD航向保持
} else {
    放弃断桥 → 原地旋转找线
}
```

---

## Q4：误差从哪里来？怎么控制？

### 误差源

| 来源 | 原因 | 影响 |
|------|------|------|
| 编码器滑移 | 轮子打滑、地面不平 | 线速度偏小 |
| yaw 漂移 | JY62 磁场干扰 | 朝向角偏 |
| 采样周期 | 5ms 内假设匀速 | 弧线段累计误差 |
| 初始位姿 | 起点的 (x,y) 不知道 | 绝对位置始终是估计值 |

### 应对

1. **不用绝对坐标做决策**——用**相对变化量**（"冲了 280mm"而不是"到了 (1.2m, 0.5m)"）
2. **看到线后立即重置**——灰度恢复 → 里程计累积归零，切断误差传播
3. **yaw 漂移兜底**——JY62 超过 500ms 无帧 → online=0 → 退回丢线恢复（原地旋转找线），不用里程计了

---

## Q5：和纯追踪法有什么关系？

纯追踪法（ques3.md Q6）用**灰度偏差**算曲率 → 转角。陀螺里程计用 **yaw + 编码器** 算位姿 → 直线冲的距离控制。

两个算法**互补**：

| 场景 | 用哪个 | 为什么 |
|------|--------|--------|
| 有黑线、灰度正常 | 纯追踪法 | 直接看线，精确 |
| 丢线、灰度全白 | 陀螺里程计 + PD 航向保持 | 灰度没用了，靠惯性 |
| 过弧线→直线的过渡 | 编码器里程计 | 弧线长度已知（126cm），里程到了就警觉 |

---

## Q6：H 题怎么用里程计？

H 题场地**没有十字标记**——ABCD 四个顶点不能靠全黑检测。替代方案：

1. **弧线端点 = 里程到了**：顶弧/底弧各约 126cm，编码器跑到 120cm 开始等"灰度从偏→居中"的过渡
2. **直线段长度**：场地总长 220cm 减两个弧的直径（各 80cm）= 60cm 直边。直边跑 55cm 开始等过渡
3. **双重确认**：里程到了 + 灰度过渡出现了 → 确认过点。只有里程或只有灰度都不算，防误判

> 里程计在 H 题里是**辅助判断**，不是主控。主控始终是灰度——有灰度时巡线，里程计只用来"大概知道跑了多远"。
