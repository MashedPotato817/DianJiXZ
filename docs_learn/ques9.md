---
layout: note
title: VOFA+ JustFloat在线调参
---

[← 学习笔记](.) | 最后更新：2026-07-14 01:16

# VOFA+ JustFloat 在线调参 —— Q&A

> 参考 LieWill/MSPM0G3507_for_car 仓库，BSP/VOFA/ 模块

---

## Q1：VOFA+ 是什么？和 DataScope 有什么不同？

### VOFA+

[VOFA+](https://www.vofa.plus/) 是免费的串口上位机（Windows/Linux/Mac），功能和 DataScope 类似——接收串口数据并实时画波形。但多了一个 DataScope 没有的能力：**发送文本命令给 MCU 改参数**。

### 对比

| | DataScope | VOFA+ |
|---|---|---|
| 波形显示 | ✅ | ✅（通道数更多） |
| 在线改参数 | ❌ 只能看 | ✅ 发串口指令改 PID/目标值 |
| 协议复杂度 | 固定 | 可选（JustFloat 最简单） |
| 调参流程 | 看波形 → 停车 → 改代码 → 编译 → 烧录 → 重跑 | 看波形 → 发命令 → 立刻生效 |
| 参数保存 | 编译在固件里 | 需自己写 Flash 存储（否则掉电丢失） |

**比赛现场优势**：30 分钟调试时间，不需要等编译烧录就能改 Kp/Ki/目标速度/目标角度。

---

## Q2：JustFloat 协议怎么工作？

### 上行（MCU → 上位机）：原始浮点字节

每路数据用 4 字节 float（IEEE 754，小端）直接发送，尾部追加 `00 00 80 7F`（float 正无穷 = 帧尾标识）。

```c
void vofa_transmitfloat(vofa *temp) {
    // 1. 发送所有正在排队的浮点数
    for (i = 0; i < temp->tx_num; i++)
        for (j = 0; j < 4; j++)
            UART_SendByte(temp->data[i].c[j]);  // float 拆成 4 字节

    // 2. 发帧尾：4字节正无穷 → 00 00 80 7F
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x80);
    UART_SendByte(0x7F);

    temp->tx_num = 0;  // 清空队列
}
```

### 使用方式

```c
// 在 5ms 中断或主循环里：
vofa_add_value(&bluetooth, speed.left.real);    // CH1: 左轮实际速度
vofa_add_value(&bluetooth, speed.left.target);  // CH2: 左轮目标速度
vofa_add_value(&bluetooth, speed.left.out);     // CH3: PWM 输出
vofa_add_value(&bluetooth, angle.real);          // CH4: 当前偏航角
vofa_transmitfloat(&bluetooth);                  // 打包发出
```

VOFA+ 上位机自动把 CH1~CH4 画成 4 条不同颜色的波形。横轴时间，纵轴数值。

### 下行（上位机 → MCU）：文本命令

以 `\n` 结尾的纯文本。MCU 用 `sscanf` 解析：

```
SLP2.5\n     → 左轮 Kp = 2.5
SLI0.01\n    → 左轮 Ki = 0.01
SRP3.0\n     → 右轮 Kp = 3.0
ST20.0\n     → 目标速度 set_speed = 20
RP1.5\n      → 循迹环 Kp = 1.5
AP0.8\n      → 角度环 Kp = 0.8
AT45.0\n     → 目标角度 = 45°
TS\n         → 切换到速度显示模式
TR\n         → 切换到循迹显示模式
OP\n         → 紧急停机
```

---

## Q3：命令解析怎么实现？

### 核心代码

```c
void vofa_receive() {
    if (!rx.complete) return;  // 没收到完整行（等 \n）

    switch (rx.RxBuff[0]) {
    case 'S':  // Speed 类命令
        switch (rx.RxBuff[1]) {
        case 'T':                             // ST = Set Target speed
            sscanf((char *)&rx.RxBuff[2], "%f", &set_speed);
            break;
        case 'L':                             // SL = Left motor
            switch (rx.RxBuff[2]) {
            case 'P':
                sscanf((char *)&rx.RxBuff[3], "%f", &speed.left.kp);
                break;
            case 'I':
                sscanf((char *)&rx.RxBuff[3], "%f", &speed.left.ki);
                break;
            case 'D':
                sscanf((char *)&rx.RxBuff[3], "%f", &speed.left.kd);
                break;
            }
            break;
        case 'R': ...  // SR = Right motor (同上)
        }
        break;

    case 'R':  // RIF 循迹环
        if (rx.RxBuff[1] == 'P')
            sscanf((char *)&rx.RxBuff[2], "%f", &rif.kp);
        break;

    case 'A':  // Angle 角度环
        if (rx.RxBuff[1] == 'P')
            sscanf((char *)&rx.RxBuff[2], "%f", &angle.kp);
        break;

    case 'T':  // Toggle 切换显示模式
        if (rx.RxBuff[1] == 'S') cmd = SPEED;     // 看速度波形
        if (rx.RxBuff[1] == 'A') cmd = ANGLE;     // 看角度波形
        break;

    case 'O':  // OP = emergency stop
        if (rx.RxBuff[1] == 'P') { ... }
        break;
    }
    rx.complete = false;  // 等待下一条命令
}
```

### 设计要点

1. **单字符首字母**区分命令大类（S=Speed, R=RIF, A=Angle, D=Distance）
2. **逐级 switch**，最多 3 层，不需要查表
3. **`sscanf(buf, "%f", &var)`** 直接从字符串解析浮点数
4. **`\n` 结尾** = 命令结束标志，UART 中断里检测

---

## Q4：怎么接 UART？

### 硬件

```
MSPM0 UART TX → USB转TTL RX → PC VOFA+
MSPM0 UART RX ← USB转TTL TX ← PC VOFA+
```

同一个 UART 口双向通信。LieWill 用的是 WIT IMU 的 UART（IMU 和 VOFA+ 分时复用），但我们如果有空闲 UART 就用独立的。

### 中断接收

```c
void UART_IRQHandler(void) {
    uint8_t byte = DL_UART_receiveData(...);
    rx.RxBuff[rx.size++] = byte;
    if (byte == '\n') {          // 收到换行 = 一条完整命令
        rx.RxBuff[rx.size] = '\0';  // null 终止（方便 sscanf）
        rx.complete = true;
    }
}
```

---

## Q5：和我们的 DataScope 对比，迁移成本多大？

### 当前状态

我们已有一个 UART（`UART_0`）给 DataScope 发包显示变量。DataScope 只做上行（MCU→PC）。

### 迁移步骤

1. **找空闲 UART**（或复用 UART_0）——如果已有 USB 转 TTL 模块就能用
2. **替换发送函数**：把 `DataScope_DP.c` 的调用改成 `vofa_add_value()` + `vofa_transmitfloat()`
3. **加接收中断**：在 `ti_msp_dl_config.c` 的 UART 配置里启用 RX 中断
4. **加 `vofa_receive()` 调用**：放在主循环里
5. **PC 端切换**：从 DataScope 换成 VOFA+

### 最小改动方案

不动 DataScope，只在主循环里加几行试试水：

```c
// 临时测试：每 100ms 发一次 VOFA+ 数据
static uint32_t last_vofa_tick = 0;
if (Systick_getTick() - last_vofa_tick > 20) {  // 100ms
    last_vofa_tick = Systick_getTick();
    vofa_add_value(&v, MotorA.Current_Encoder);
    vofa_add_value(&v, MotorA.Target_Encoder);
    vofa_transmitfloat(&v);
}
```

PC 端打开 VOFA+，选串口 → JustFloat 协议 → 就能看到两条波形。

---

## Q6：有哪些常见的坑？

| 坑 | 说明 |
|----|------|
| **字节序** | JustFloat 是小端（和 MSPM0 一致），不需要转换 |
| **帧尾冲突** | 如果数据恰好也是 `0x00 0x00 0x80 0x7F`（正无穷），上位机会误判为帧尾。概率极低（浮点数 ±∞/NaN 极少出现），基本不用管 |
| **通道顺序** | 上位机按发送顺序分配 CH1/CH2/...，每次 `transmitfloat()` 的 `tx_num` 必须和上位机配置的通道数一致 |
| **发送频率** | 10 通道 × 4 字节 × 200Hz = 8KB/s。9600bps 太慢，至少用 115200bps |
| **sscanf 安全** | `sscanf` 解析失败不会报错，变量保持原值。这不是 bug 是 feature——发错命令不会崩 |
| **命令缓冲区溢出** | `rx.size` 要加判断，超过 `UART_BUFF_SIZE` 就清空或丢弃 |

---

## Q7：和其他调参方式的一键对比

| 方式 | 能看波形 | 能改参数 | 需要编译烧录 | 需要额外硬件 |
|------|---------|---------|-------------|------------|
| OLED 显示 | ❌ 只有数字 | ❌ | — | OLED |
| DataScope | ✅ | ❌ | ✅ 改一次烧一次 | USB-TTL |
| VOFA+ JustFloat | ✅ | ✅ | ❌ | USB-TTL |
| 按键 + OLED 菜单 | ❌ | ✅（预设值） | ❌ | 按键 + OLED |
| 蓝牙 APP | ❌ | ✅ | ❌ | 蓝牙模块 |

VOFA+ 是唯一同时满足"看波形 + 改参数 + 不编译"的方案。

---

> 最后更新：2026-07-14 01:16
