---
layout: note
title: E题参考实现——LieWill仓库分析
---

[← 2024归档](./) | 最后更新：2026-07-14 01:16

# E 题参考实现 —— LieWill 仓库分析

> 仓库：[LieWill/MSPM0G3507_for_car](https://github.com/LieWill/MSPM0G3507_for_car)
> 题目：2024 电赛 E 题（自动行驶小车），和我们的 H 题同平台、不同场地

---

## 一、仓库架构：BSP 模块化分层

```
MSPM0G3507_for_car/
├── BSP/                  # 板级支持包——每个外设独立 .c/.h
│   ├── Motor/            # TB6612 电机驱动（PWM + 方向）
│   ├── PID/              # 4 套 PID 控制器（速度/距离/角度/循迹）
│   ├── QEI/              # 正交编码器（速度 + 里程累计）
│   ├── WIT/              # 维特智能 IMU（偏航角/横滚/俯仰）
│   ├── RIF/              # 8 路红外灰度传感器（并行读取）
│   ├── OLED/             # OLED 显示
│   ├── VOFA/             # VOFA+ 上位机通信（JustFloat 协议）
│   ├── TASK/             # 任务状态机（Q1~Q5 菜单切换）
│   ├── TIMER/            # 系统定时器 + CPU 占用率估算
│   └── FLASH/            # 片内 Flash 读写（掉电保存参数）
├── USER/                 # 应用层
│   ├── main.c            # 主循环：按键选任务 + 调 use_task()
│   └── ti_msp_dl_config.c/h  # SysConfig 生成的硬件配置
└── keil/                 # Keil MDK 工程
```

### 和我们的对比

| | LieWill (E题) | 我们 (H题) |
|---|---|---|
| 驱动组织 | BSP/ 下每外设一个文件夹 | Hardware/ 下平铺 |
| PID | 4 套独立结构体（速度/距离/角度/循迹） | 2 个增量式 PI（左右轮速度） |
| 状态机 | 5 个独立 `question_status._N` | 全局 `GrayRouteSegment_t` 枚举 |
| 调试 | VOFA+ JustFloat 实时调参 | OLED + DataScope 查看 |
| 传感器 | IMU(yaw) + RIF(8路红外) | JY62(yaw+wz) + 灰度(8路) |

---

## 二、核心创新：INS / TRACKING / STOP 三模切换

### 为什么需要双模

E 题场地是**折线**（不是弧线）：车从起点出发，沿固定角度斜走，遇到黑线后转为循迹，走到线尽头再转回惯性导航。

```
INS (惯性导航)
  │  用 IMU 偏航角 + 编码器里程，死推算走直线/斜线
  │
  ├── 检测到黑线？ → 切换到 TRACKING
  │
TRACKING (灰度循迹)
  │  灰度传感器沿黑线走
  │
  ├── 黑线消失？ → 切回 INS，走下一段斜线
  │
STOP
  停车
```

### 代码实现（task.c 核心）

```c
enum { INS, TARCKING, STOP } status = STOP;

// 典型状态机：Q3 的 INS→TRACKING→INS 循环
void use_task(task *select) {
    switch (question_status._3) {
    case 0:  // 起步：设定 INS 目标角度 + 距离
        angle.target = wit_get_yaw();
        go_straight(FIRST_TURN, TURN_DISTANCE);
        question_status._3 = 1;
        break;

    case 1:  // INS 斜走中，等距离到达或遇到黑线
        if (distance.real > distance.target - 300) {
            if (isBlack == true) {  // 提前遇到黑线 → 切循迹
                start_tarcking();
                question_status._3 = 3;
            } else {  // 距离到了没线 → 再走一小段
                go_short(...);
                question_status._3 = 2;
            }
        }
        break;

    case 3:  // 循迹中，等黑线消失
        if (isBlack == false) {  // 线走完了 → 切回 INS
            go_straight(-SECOND_TURN, TURN_DISTANCE);
            question_status._3 = 1;
        }
        break;
    }
}
```

### 关键设计点

**INS 模式下的角度保持**：`go_straight(Angle, length)` 在现有 yaw 基础上**累加**目标角度，不是覆盖：

```c
angle.target += wit_to_int16(Angle);  // 累加！不是 angle.target = ...
```

这样多段路径拼接时角度连续，不会跳变。

**"检测到黑线"的判定**：用 `isBlack` 全局标志——8 路红外中任意一路检测到黑色即为 true。比我们的 `black_count` 判断更简单，适合 E 题"只要碰到线就转循迹"的需求。

---

## 三、多 PID 控制器设计

### 四套独立结构体

```c
// 1. 速度环（增量式 + 可选前馈/二阶前馈/积分）
typedef struct {
    float real, target, kp, kd, out, last_error;
#if defined(SPEED_WITH_KI)
    float ki, accumulate;
#endif
#if defined(SPEED_WITH_feedforward)
    float last_target, kf;
#if defined(SPEED_WITH_Second_order_feedforward)
    float last_target2, kf2;
#endif
#endif
} pid_speed;

// 2. 循迹环（位置式 PD）
typedef struct {
    float real, target, out, last_error, kp, kd;
} pid_rif;

// 3. 距离环（位置式 PD + 限幅）
typedef struct {
    float real, target, kp, kd, last_error, out, limit;
} pid_distance;

// 4. 角度环（PI + 抗积分饱和 + 滤波）
typedef struct {
    int16_t real, target, last_real;
    float out, kp, ki, accumulate;
#if defined(WIT_WITH_KD)
    int16_t last_error;
    float kd;
#endif
} pid_wit;
```

### 亮点 1：速度前馈（Feedforward）

普通 PI 只能"等误差出现再调"。前馈直接根据目标变化预判输出：

```c
// pid.c - pid_run_speed()
input->out += (input->target - input->last_target) * input->kf;  // 一阶前馈
input->out += ((target变化率) - (上次变化率)) * input->kf2;     // 二阶前馈
```

**直觉**：目标速度从 0 跳到 0.3m/s——PI 需要几个周期积累误差才开始用力。前馈看到目标变了，直接加上 `0.3 × kf` 的 PWM 增量，响应快得多。

### 亮点 2：角度环的抗积分饱和

```c
// pid.c - pid_run_wit()
if (input->accumulate > 0 && error > 0)  input->accumulate = 0;  // 同向清零
if (input->accumulate > 20)              input->accumulate = 20;  // 硬上限
if (input->accumulate < -20)             input->accumulate = -20;
```

和我们增量式 PI 的硬限幅（`Pwm > 7800`）不同，这里的抗饱和更精细：
- 积分方向与误差方向相同 → 清零（说明正在"过冲"）
- 积分绝对值超 20 → 钳位

### 亮点 3：距离环的限幅随阶段变化

```c
// task.c - start_straight()
distance.limit = 10;  delay_cycles(8000000);  // 起步限 10
distance.limit = 15;  delay_cycles(8000000);  // 慢慢加大
distance.limit = 20;  delay_cycles(8000000);
distance.limit = 25;  delay_cycles(8000000);
distance.limit = 33;  delay_cycles(8000000);  // 全速
```

这叫**阶梯加速**——不是瞬间给满 PWM，而是分 5 个台阶逐步放大距离环的输出限幅。原理：

- 起步瞬间给满 PWM → 车头翘起 → 灰度传感器离地 → 丢线 → 失控
- 阶梯加速让加速度可控，传感器始终贴地

---

## 四、VOFA+ JustFloat 在线调参（杀手级特性）

### 协议

VOFA+ 是免费的串口上位机，支持多种协议。JustFloat 是最简单的一种：

**上行（MCU → 上位机）**：每路数据 4 字节原始 float（小端），尾部追加 `00 00 80 7F`（float 正无穷，帧尾标识）。

```c
void vofa_transmitfloat(vofa *temp) {
    for (i = 0; i < temp->tx_num; i++)
        for (j = 0; j < 4; j++)
            DL_UART_transmitDataBlocking(..., temp->data[i].c[j]);  // 4字节float
    // 帧尾：00 00 80 7F
    DL_UART_transmitDataBlocking(..., 0x00);
    DL_UART_transmitDataBlocking(..., 0x00);
    DL_UART_transmitDataBlocking(..., 0x80);
    DL_UART_transmitDataBlocking(..., 0x7F);
    temp->tx_num = 0;
}
```

**下行（上位机 → MCU）**：纯文本命令 + `\n` 结尾。

```
SLP2.5\n    → 左轮 Kp 改为 2.5
RP3.0\n     → 循迹环 Kp 改为 3.0
AT45.0\n    → 目标角度改为 45°
ST20.0\n    → 目标速度改为 20
TS\n        → 切换到速度波形显示
TR\n        → 切换到循迹波形显示
OP\n        → 紧急停机
```

### 命令解析

```c
void vofa_receive() {
    if (!rx.complete) return;  // 没收到完整命令（以\n结尾）

    switch (rx.RxBuff[0]) {
    case 'S':  // Speed
        switch (rx.RxBuff[1]) {
        case 'L':  // Left
            switch (rx.RxBuff[2]) {
            case 'P': sscanf(..., "%f", &speed.left.kp); break;
            case 'I': sscanf(..., "%f", &speed.left.ki); break;
            case 'D': sscanf(..., "%f", &speed.left.kd); break;
            }
            break;
        case 'R': ...  // Right
        }
        break;
    case 'R':  // RIF tracking
        ...
    case 'A':  // Angle
        ...
    case 'T':  // Toggle display mode
        ...
    }
    rx.complete = false;
}
```

非常简单——单字符 + 可选子命令 + `sscanf` 解析浮点数。不需要复杂协议。

### 为什么比赛现场这个很重要

| 传统方式 | VOFA+ 方式 |
|---------|-----------|
| 改 PID → 改代码 → 编译 → 烧录 → 重启 → 跑一圈验证 | 串口发 `SLP2.5\n` → 立刻生效 |
| 看速度曲线 → OLED 显示 2 个数字 | 上位机 10 通道波形，同时看目标值/实际值/误差/PWM |
| 调参靠"感觉" | 看波形 → 精确知道欠阻尼还是过冲 |

**比赛现场 30 分钟调试时间内，每一个编译-烧录-重启循环都是浪费。** VOFA+ 让调参变成"边跑边改"。

---

## 五、WIT IMU 协议解析

### 帧格式（UART，和 JY62 的 11 字节不同，这里是 9 字节）

```
0x55  0x53  Roll_L  Roll_H  Pitch_L  Pitch_H  Yaw_L  Yaw_H  Version  Sum
 │      │                                           │               │
 帧头   帧类型（0x53=姿态角）                          └─ 校验（全部字节求和低8位）
```

### 解析状态机

```c
void BLUE_TOOTH_INST_IRQHandler() {
    uint8_t temp;
    switch (Wit_rx.status) {
    case stop:   // 空闲，等 0x55
        if (temp == 0x55) Wit_rx.status = check;
        break;
    case check:  // 等 0x53
        if (temp == 0x53) Wit_rx.status = start;
        else              Wit_rx.status = stop;  // 不是0x53 → 回 idle
        break;
    case start:  // 收9个字节数据
        Wit_rx.buff.c[Wit_rx.size++] = temp;
        if (Wit_rx.size > 8) {  // 收完了
            if (校验OK) {
                angle.yaw   = Wit_rx.buff.data.yaw;    // 更新全局角度
                angle.roll  = Wit_rx.buff.data.roll;
                angle.pitch = Wit_rx.buff.data.pitch;
            }
            Wit_rx.status = stop;  // 回到空闲
        }
        break;
    }
}
```

### 角度换算

WIT IMU 输出的是 16 位原始值（范围 -32768~+32767），需转换为角度：

```c
inline float wit_to_float(int16_t data) {
    return data / 32768.0f * 180.0f;    // 原始值 → 角度(-180°~+180°)
}

inline uint16_t wit_to_int16(float angle) {
    return angle / 180.0f * 32768.0f;   // 角度 → 原始值
}
```

和 JY62 的 `raw/32768 × full_scale` 原理相同，只是封装成了函数。

---

## 六、RIF 灰度传感器——并行读取

### 和我们最大的不同

**我们**：分时选通道（3 根地址线 → 8 选 1 → 读 1 个 GPIO），每次 `Gray_Read_All()` 要 8 个循环 + 每个 delay 50μs。

**LieWill**：8 个传感器直接映射到 GPIOB 的低 8 位，一次 `GPIOB->DIN31_0 & 0xFF` 读出全部 8 路。

```c
inline uint16_t rif_getValue() {
    return GPIOB->DIN31_0 & 0xFF;  // 读 8 个 IO 的当前电平 → 8bit
}
```

速度差距：我们 ~400μs 读 8 路，他们 ~0.1μs 读 8 路。差 4000 倍。

### 线位置计算

更简单粗暴——找到最左和最右的黑传感器序号，取中间：

```c
int rif_to_value(uint16_t value) {
    uint8_t i = 1;    // 从 bit1 开始找（bit0 可能用作其他用途）
    uint8_t j = 10;   // 到 bit9 结束
    for (; i < 10; i++)
        if (value & (1 << i)) break;  // 找第一个黑线 bit
    for (; j > i; j--)
        if (value & (1 << j)) break;  // 找最后一个黑线 bit
    return i + j - 11;  // 中点偏移（0 为中心，负=左，正=右）
}
```

比我们的加权质心法更简单，只有整型加减，但只给整数位置（没有亚像素精度）。对于 E 题的"碰到线就切换"来说够用。

---

## 七、E 题 vs H 题对比

| 维度 | E 题（LieWill 做法） | H 题（我们目前的做法） |
|------|---------------------|----------------------|
| 场地形状 | 折线，固定角度(45°/74°)的直线段 | 半圆弧(r=40cm) + 直道 |
| 核心策略 | **INS 主导**——IMU 角度 + 编码器里程走斜线，碰到黑线切换为循迹 | **灰度主导**——全程巡线，靠过渡特征检测过点 |
| 黑线作用 | 辅助定位——用来"兜底"省得纯惯导漂移 | 全程路径——线在哪车在哪 |
| 过点标记 | 无，靠"黑线出现/消失" + 里程判断 | 无，靠"弧线↔直道过渡" + 里程兜底 |
| IMU 依赖 | 极高——没有 IMU 无法走斜线 | 中等——断桥导航用，弧线巡线主要靠灰度 |
| 传感器 | WIT IMU(yaw) + RIF(8路红外) | JY62 IMU(yaw+wz) + 灰度(8路) |
| 循迹精度 | 低要求——线只是辅助 | 高要求——全程沿弧线走 |
| 可复用的技术 | VOFA+、阶梯加速、多 PID 结构、BSP 分层 | 纯追踪弧线、过渡检测、里程计 |

### 核心差异：E 题是"惯性导航为主"，H 题是"视觉巡线为主"

```
E 题路径（折线）:                H 题路径（弧线）:
  INS 斜走                       沿弧线巡
    ↘                              ↗
     黑线→循迹──→线消失            A ═══ B
                      ↘            │     │
                    INS 转回         │     │  ← 全程沿弧线
                      ↓            D ═══ C
```

---

## 八、可迁移到我们项目的技术点

### 立刻能用

| 技术 | 迁移难度 | 收益 |
|------|---------|------|
| **BSP 模块化分层** | 低——重新整理 Hardware/ 文件夹 | 代码结构清晰，多人协作不冲突 |
| **阶梯加速** | 极低——给 `Set_PWM` 加限幅递增 | 防起步翘头，不依赖硬件改动 |
| **多 PID 独立结构体** | 中——重构 `control.c` | 距离环/角度环/循迹环各自调参，不互相干扰 |

### 短期能用（需少量改动）

| 技术 | 迁移难度 | 收益 |
|------|---------|------|
| **VOFA+ JustFloat** | 中——需要新 UART 通道或复用现有 | 比赛现场调参利器，不需要每次编译烧录 |
| **INS + 灰度双模** | 中——在现有 `Gray_Mode()` 旁加一个 `INS_Mode()` | H 题断桥/弧线终点过渡更可靠 |
| **角度环抗积分饱和** | 低——替换现有 JY62 PD 的限幅逻辑 | 角度控制更平滑 |

### 长期参考

| 技术 | 说明 |
|------|------|
| **速度前馈** | 控制频率 200Hz 时效果不明显，降到 50Hz 才有必要，暂时不急着加 |
| **FLASH 参数保存** | 掉电记住 PID 参数——VOFA+ 调好的参数下次开机还在，不用每次重设 |
| **CPU 占用率估算** | TIMER 模块里的 `get_cpu_usage()`——确认 5ms 中断不会超时 |

---

## 九、一句话总结

LieWill 仓库最值得学的两点：

1. **INS + TRACKING 双模切换**——用 IMU 走直线、用灰度走弧线，两条腿走路比单腿稳
2. **VOFA+ 在线调参**——串口发文本命令改 PID，比赛现场不需要烧录就能调

两个都不依赖特殊硬件，我们的 MSPM0 + JY62 + 灰度传感器完全可以实现。

---

> 最后更新：2026-07-14 01:16
