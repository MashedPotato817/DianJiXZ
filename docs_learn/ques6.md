---
layout: note
title: JY62 IMU与任务状态机
---
[← 学习笔记](.) | 最后更新：2026-07-13 16:05
# JY62 IMU 与任务状态机 —— Q&A

> 基于 WHEELTEC C07A JY62_GapBridge 固件代码分析

---

## Q1：JY62 是什么？有什么用？

### 传感器简介

JY62 是 6 轴 IMU（惯性测量单元），内部集成了 3 轴加速度计 + 3 轴陀螺仪，芯片自带姿态解算，直接输出角度。通过 UART 以 115200bps 发数据帧。

### 输出三组数据

| 帧类型 | 字节 | 含义 | 量程 | 
| ------ | ---- | ---- | ---- |
| 加速度 (0x51) | ax, ay, az | 三轴加速度 | ±16g |
| 陀螺仪 (0x52) | wx, wy, wz | 三轴角速度 | ±2000°/s |
| 角度 (0x53) | roll, pitch, yaw | 欧拉角 | ±180° |

### 在这个项目里用 yaw（偏航角）

- 巡线时：用灰度传感器转弯
- 过断桥/丢线时：灰度看不到线 → 用 yaw 角保持直行方向
- 任务是顺时针还是逆时针绕圈 → 靠 yaw 判断方向

---

## Q2：JY62 的串口帧怎么解析？

### 帧格式

```
帧头 0x55 + 类型(1字节) + 数据(6字节) + 校验和(1字节) = 11字节
```

校验和 = 前 10 个字节累加，取低 8 位。

### 解析状态机（`jy62.c:94-131`）

```c
void JY62_InputByteFromISR(uint8_t data)
{
    // 状态0：等帧头 0x55
    if (g_jy62Index == 0) {
        if (data != 0x55) return;           // 不是帧头就丢掉
        g_jy62Frame[0] = data;
        g_jy62Index = 1;
        return;
    }

    // 状态1：验证类型字节（必须是 0x51/0x52/0x53）
    if (g_jy62Index == 1) {
        if (data不是加速度/陀螺仪/角度) {
            // 类型不合法 → 可能是误触发的0x55 → 回退
            if (data == 0x55) 重新开始;      // 数据里刚好出现了0x55
            else              重置;          // 噪声
            return;
        }
    }

    // 状态2~10：收数据，中间遇到0x55要重新同步
    if (data == 0x55) {
        // 帧内出现0x55 → 这可能是下一帧的头
        g_jy62Frame[0] = 0x55;
        g_jy62Index = 1;
        return;
    }

    g_jy62Frame[g_jy62Index++] = data;

    // 收到完整11字节 → 校验+处理
    if (g_jy62Index >= 11) {
        JY62_HandleFrame(g_jy62Frame);
        g_jy62Index = 0;
    }
}
```

### 关键设计：帧内 0x55 二次同步

数据里可能出现 0x55（比如角度的某个字节刚好是这个值）。解析器遇到这种情况就把它当作新帧头重新同步，避免长期错位。

---

## Q3：JY62 在本项目中是怎么接线的？和蓝牙怎么共用？

### 硬件接线

JY62 和蓝牙模块**共用 MSPM0 的 UART1**：

```
JY62 TX  ─┬─→ MSPM0 PINCM24 (UART1 RX)
蓝牙 TX  ─┘    ← 两个设备往同一根 RX 线上发数据
```

两个设备共用 UART1，MSPM0 无法从电气层面区分字节来自谁——**靠 JY62 帧解析状态机自动过滤**：JY62 帧必须是 `0x55` 头 + 合法类型（0x51/0x52/0x53）+ 11 字节校验通过。蓝牙发的数据不符合这个格式，解析器直接丢弃。

### UART 配置

UART1 硬编码 **115200 bps**，软件到硬件三层一致：

| 层 | 位置 | 值 |
| ---- | ---- | ---- |
| 软件常量 | `jy62.c:10` `JY62_FIXED_BAUD` | 115200 |
| 硬件分频 | `ti_msp_dl_config.h:136-137` | IBRD=21, FBRD=45 |
| 查询接口 | `jy62.h:32` `JY62_GetCurrentBaud()` | 返回 115200 |

分频参数推导：40MHz 总线时钟 ÷ (16 过采样 × 115200) = 21.7 → 整数部分 21 + 小数部分 45/64，**实际波特率 115190.78**（误差 0.008%，完全可用）。

### 为什么 BT_DAMConfig 是空的？

之前蓝牙接收用了 UART1 DMA（见 ques5.md Q2），但这版 JY62 固件的 `BT_DAMConfig()` 是**空函数**（`uart_callback.c:8-10`）：

```c
void BT_DAMConfig(void)
{
    // 什么都没有
}
```

原因：DMA 会独占 UART1 接收通道，JY62 字节也走 UART1 → 关了 DMA，蓝牙和 JY62 数据都走 UART1 中断逐字节接收，解析器自动区分。

### 初始化做了什么

```c
// empty.c:37
JY62_Init();   // 清零所有数据结构，清空 UART1 RX FIFO
```

没有"配置 JY62 模块"的步骤——JY62 **出厂默认**就输出 115200 bps / 10Hz / 角度+陀螺仪+加速度三组帧。代码直接匹配，不需要发 AT 指令切换。

### 超时判离线

```c
// jy62.c:156-159
if (now - lastFrameMs > 500) {
    g_jy62Data.online = 0;   // 500ms 没收到有效帧 → 离线
}
```

`JY62_Tick10ms()` 由 Timer 中断每 10ms 调用一次，维护毫秒时间戳。

---

## Q4：JY62 的数值是怎么换算的？

### 换算公式

```c
// 加速度：16位有符号数 → 实际g值
ax_g = (int16_t)raw / 32768.0f * 16.0f;

// 陀螺仪：16位有符号数 → 实际°/s
wx_dps = (int16_t)raw / 32768.0f * 2000.0f;

// 欧拉角：16位有符号数 → 实际°
yaw_deg = (int16_t)raw / 32768.0f * 180.0f;
```

**原理**：JY62 把满量程映射到 -32768 ~ +32767。比如角度量程 ±180° → `180 / 32768 = 0.0055°/LSB`。

### 角度归一化

```c
static float Gray_NormalizeYawDeg(float yaw_deg)
{
    while (yaw_deg > 180.0f)  yaw_deg -= 360.0f;   // 200° → -160°
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;   // -200° → 160°
    return yaw_deg;
}
```

JY62 的角度输出范围是 ±180°，不会溢出。但做角度差计算时（比如 179° 和 -179°），差的绝对值应该约 2° 而不是 358°，归一化就是把角度折到 -180°~+180°。

---

## Q5：断桥模式是怎么工作的？

### 问题

赛道上有一段没有黑线的区域（断桥/十字路口后空白）。灰度传感器全白 → 丢线。但车不能原地转——必须直走冲过去。

### 方案：IMU 航向保持

```
检测到丢线 + 之前稳定巡线了足够久 + JY62在线
  ↓
记录当前 yaw 角作为参考方向
  ↓
bridge_active = 1   // 进入断桥模式
  ↓
用 yaw 偏差做 PD 转向修正：
  Move_Z = 0.075 × (参考yaw - 当前yaw) - 0.010 × 陀螺仪角速度
  Move_X = 220 mm/s  // 恒定速度直冲
  ↓
冲了 280mm 还没看到线？→ 放弃，恢复普通丢线恢复（原地旋转）
看到线？→ 重捕获2次确认 → 退出断桥
```

### 为什么用 PD 而不是 PI？

- P（yaw偏差比例）：偏了就纠正方向
- D（陀螺仪角速度）：角速度代表"正在转多快"，D 项提前刹车防过冲
- 不需要 I：断桥就几百毫秒，不需要积分消除稳态误差

---

## Q6：任务状态机怎么设计？

### 赛道布局

```
        A ───────── B
        │           │
        │   (赛道)  │
        │           │
        D ───────── C

过桥位置：A→B 和 C→D 是断桥
弧线位置：B→C 和 D→A
```

### 路线段枚举

```c
typedef enum {
    BRIDGE_TOP_LR,      // 顶桥 A→B
    ARC_RIGHT_DOWN,     // 右弧 B→C
    BRIDGE_BOTTOM_RL,   // 底桥 C→D
    ARC_LEFT_UP,        // 左弧 D→A
    ARC_LEFT_DOWN,      // 左弧 A→D (逆时针)
    BRIDGE_BOTTOM_LR,   // 底桥 D→C (逆时针)
    ARC_RIGHT_UP,       // 右弧 C→B (逆时针)
    BRIDGE_TOP_RL,      // 顶桥 B→A (逆时针)
} GrayRouteSegment_t;
```

### 4 种任务模式

| 模式 | 含义 | 停止条件 |
| ---- | ---- | -------- |
| TASK_1_AB_STOP | A→B 过桥停 | 过完顶桥 |
| TASK_2_CW_1LAP | 顺时针一圈 | A→B→C→D→回A |
| TASK_3_CCW_1LAP | 逆时针一圈 | A→D→C→B→回A |
| TASK_4_CCW_4LAP | 逆时针四圈 | 绕4圈回A |

### 触发与切换

按键操作（`Key()`）：
- 单击 → 启停
- 双击 → 切换任务模式（0→1→2→3→0）
- 三击 → 重置任务

启动时自动设置初始路线方向、清零计数。

---

## Q7：LED 通知机制是什么？

调试时看不到串口，靠 LED 判断状态：

```c
// 过完一个点 或 任务结束 → LED闪烁
LED闪烁：亮250ms + 灭250ms，反复6次 → 灭

// 任务模式切换 → 闪烁次数 = 模式号+1
```

闪一下 = 过完一个点。连续闪 = 任务完成。不用连电脑也能确认车跑对了。

---

## Q8：桥入口/出口怎么判断？

### 过桥入口

条件：稳定巡线 ≥ 8 个周期（40ms）+ 当前8路全白 + 还没在桥模式 + JY62在线

```c
if ((!bridge_active) &&
    (line_stable_ticks >= 8) &&   // 确保不是中途丢线而是到了桥头
    (black_count == 0) &&         // 真的丢线了
    JY62_IsOnline())              // IMU可用
{
    Gray_OnBridgeStart();          // 记录当前地点(A/B/C/D)
    bridge_active = 1;             // 进入断桥
}
```

### 过桥出口

条件：在桥模式 + 连续2次看到至少2个传感器有黑线

```c
if (bridge_active && black_count >= 2) {
    bridge_reacquire_count++;
    if (bridge_reacquire_count >= 2) {
        bridge_active = 0;          // 退出断桥
        Gray_OnBridgeFinish();      // 记录到达点
    }
}
```

连续 2 次确认防噪——线刚出来时可能只有 1~2 个传感器偶尔触发。

---

## Q9：JY62 的数据是怎么被收进来的？

数据有**两条路径**——中断为主，主循环兜底：

### 主路径：UART1 中断直接解析

```c
// uart_callback.c:17-24
void UART_1_INST_IRQHandler(void)          // UART1 收到字节就触发
{
    uint8_t data;

    while (DL_UART_Main_receiveDataCheck(UART_1_INST, &data)) {
        JY62_InputByteFromISR(data);       // 直接在 ISR 里逐字节解析！
    }
}
```

为什么不担心 ISR 耗时？`JY62_InputByteFromISR()` 只是几个 `if` 判断 + 写数组——单字节处理只需几十个 CPU 周期。115200bps 下字节间隔约 87μs，MSPM0 @ 80MHz 完全来得及。

### 备用路径：主循环轮询

```c
// uart_callback.c:12-15 → 主循环 main() 里调用
void BTBufferHandler(void)
{
    JY62_Process();    // 也读 UART1 FIFO 喂解析器
}
```

`JY62_Process()` 和 ISR 做同样的事——从 FIFO 读字节调 `JY62_InputByteFromISR()`。正常情况下 ISR 已经把字节收完了，主循环的轮询只是**兜底**（比如瞬间大量字节涌入导致 ISR 来不及收完）。

### 超时计时器

```c
// control.c:516-527 → TimerG 5ms 中断
void TIMER_0_INST_IRQHandler(void)
{
    static uint8_t jy62_tick_div = 0;

    jy62_tick_div++;
    if (jy62_tick_div >= 2U) {             // 每 2 次中断 = 10ms
        jy62_tick_div = 0;
        JY62_Tick10ms();                   // 离线超时计时器 +10ms
    }
    // ...
}
```

`JY62_Tick10ms()` 只做 `nowMs += 10`，用来判断 500ms 超时离线——纯计数器，和数据接收无关。

### 数据流总览

```
UART1 收到字节
    │
    ├──→ UART1_IRQHandler()               ← 中断路径（主力）
    │       └── JY62_InputByteFromISR()    ← 状态机接收+解析
    │
    └──→ BTBufferHandler()                ← 主循环路径（兜底）
            └── JY62_Process()
                  └── JY62_InputByteFromISR()  ← 同一个解析函数

解析后更新 g_jy62Data 结构体
    │
    └──→ Gray_Mode() 每 5ms 调 JY62_GetData() 拿最新 yaw/wz
```

---

> 最后更新：2026-07-13 16:05
