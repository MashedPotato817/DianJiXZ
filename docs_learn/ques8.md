---
layout: note
title: K230 Link 通信 & ESP32 黑匣子
---

[← 学习笔记](.) | 最后更新：2026-07-13 16:05

# K230 Link 通信 & ESP32 黑匣子 —— Q&A

> 基于队友 `k210_link.c/h` + `ESP32S3_Blackbox/` 实物代码分析。协议框架 K230 直接复用。

---

## Q1：K230 和 MSPM0 之间怎么通信？

### 架构

```
K230 (CanMV)                      MSPM0 (Cortex-M0+)
    │                                    │
    │  UART ──── 二进制帧 ────→         │ UART_0 RX
    │                                    │
    │         ←─── ASCII 握手 ────       │ UART_0 TX
    │                                    │
    │  $K210,OK#  应答                   │ $CAR,HELLO# 查询
```

K230/K210 走**独立 UART**（默认 UART_0），不和 JY62/蓝牙（UART_1）抢线。

### 双模协议

通信同时运行两种模式——不需要切换，字节级自动分流：

| 模式 | 格式 | 用途 |
|------|------|------|
| **二进制帧** | `0xAA` + 6 字节数据 + XOR 校验 | 视觉数据高速推送 |
| **ASCII 握手** | `$CAR,HELLO#` / `$K210,OK#` | 在线检测、链路心跳 |

分流的逻辑：收到 `0xAA` → 切到二进制模式收 8 字节；收到 `$` → 切到 ASCII 模式收到 `#` 为止。两条路径完全独立，互不干扰。

---

## Q2：二进制帧的格式是怎么定义的？

### 帧结构

```
字节:   0      1    2    3    4    5    6    7
      0xAA   [X低] [X高] [Y低] [Y高] [A低] [A高] [XOR]
```

| 字段 | 字节范围 | 含义 |
|------|---------|------|
| 帧头 | 0 | 固定 `0xAA` |
| target_x | 1~2 | 目标中心 X 坐标（0~320），小端 |
| target_y | 3~4 | 目标中心 Y 坐标（0~240），小端 |
| area | 5~6 | 色块面积（像素数）。同时作为 valid 标志：area=0 表示无目标 |
| checksum | 7 | 前 7 字节按位异或 |

### 校验计算

```c
uint8_t xor = 0;
for (int i = 0; i < 7; i++) {
    xor ^= frame[i];
}
// xor == frame[7] → 帧有效
```

### 为什么要两种校验？

- **XOR 校验**防传输错误（单字节翻转、噪声）
- **area=0 表示无效**防 K230 没检测到目标时 MSPM0 误以为"目标在 (0,0)"

### 数据解读举例

```
收到帧: AA  64 00  F0 00  2C 01  5B

帧头: 0xAA ✓
X = 0x0064 = 100          → 目标在画面水平 100 像素处
Y = 0x00F0 = 240          → 目标在画面底边
面积 = 0x012C = 300 像素  → 目标不小，置信度可
XOR(AA^64^00^F0^00^2C^01) = 0x5B ✓
```

---

## Q3：ASCII 握手怎么工作的？

### 握手流程

```
1. MSPM0 每 500ms 发一次:  "$CAR,HELLO#"
2. K230 收到后回复:       "$K210,OK#"
3. MSPM0 收到 OK → 置 handshake_ok = 1
4. 超过 1500ms 没收到 OK → 判离线，online = 0
```

### 为什么需要握手？

二进制帧只告诉你"目标在哪"，不告诉你"K230 还活着吗"。如果 K230 死机或掉线，MSPM0 收不到二进制帧，但无法区分"没目标"和"芯片挂了"。握手机制独立于视觉数据，专门监控链路健康。

### 代码实现

```c
// k210_link.c:168-177 — 主循环每 5ms 调一次
if (now - last_hello >= 500ms) {
    K210_UART_SendString("$CAR,HELLO#\r\n");  // 周期性发心跳
}
if (online && now - last_ok >= 1500ms) {
    online = 0;     // 超时离线
    handshake_ok = 0;
}
```

---

## Q4：OLED 诊断页显示什么？

队友 `1d21950` 实现。OLED 切到诊断模式时显示：

```
K230 Link Diag
RX: 12345 B       ← 总收字节数
VF: 320 F         ← 有效视觉帧数
CE: 3 E           ← 校验错误数
LIVE / DEAD       ← 在线状态
```

三位数字实时刷新，不用串口打 log 就能看到：
- RX 在涨 → UART 物理层通了
- VF 在涨 → 协议解析正确，帧头+校验通过
- CE 偶尔跳 → 传输有噪声，正常；CE 持续涨 → 波特率可能不对或线太长干扰
- DEAD → K230 没回应握手或超时

---

## Q5：K230 端怎么配？

K230 烧 CanMV 固件后，用 MicroPython 写：

```python
import sensor, time
from machine import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)    # 320×240
sensor.skip_frames(time=2000)

uart = UART(1, baudrate=115200)      # K230 的 UART1 连 MSPM0 UART_0

clock = time.clock()
while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs([threshold])  # 你的色块阈值

    if blobs:
        b = blobs[0]  # 取最大色块
        # 组帧: 0xAA + x(2B) + y(2B) + area(2B) + XOR
        frame = bytes([0xAA,
                       b.cx() & 0xFF, b.cx() >> 8,
                       b.cy() & 0xFF, b.cy() >> 8,
                       b.area() & 0xFF, b.area() >> 8,
                       0  # XOR 先占位
                      ])
        # 算 XOR 填最后一位
        xor = 0
        for i in range(7):
            xor ^= frame[i]
        frame = frame[:7] + bytes([xor])
        uart.write(frame)

    # 检查握手请求
    if uart.any():
        buf = uart.read()
        if b'$CAR,HELLO#' in buf:
            uart.write(b'$K210,OK#')

    print(clock.fps())
```

> CanMV 的 UART API 和标准 MicroPython 一样，`UART(1, 115200)` 即可。不需要额外配时钟、DMA——CanMV 固件已帮你搞定。

---

## Q6：ESP32-S3 黑匣子是什么？

### 定位

独立于 MSPM0 的**调试工具**。ESP32-S3 板子接在 MSPM0 的调试 UART 上（PB0/PB1），**只监听不发**——像一个不会干扰飞行的黑匣子。

```
MSPM0 调试串口 (PB0/PB1)  →  ESP32-S3  →  WiFi  →  PC 上位机
  发速度/PWM/yaw/状态                      实时转发
```

### 为什么要用 WiFi 而不是串口线？

调车时车在跑，串口线会拖拽/缠绕。WiFi 无线遥测让你在电脑上实时看：
- 左右轮目标速度 vs 实际速度（两条曲线叠在一起）
- 灰度偏差值
- yaw 角
- 当前路段/状态机模式
- 过点事件时间戳

### ESP32 端代码结构

```
ESP32S3_Blackbox/
├── main/
│   ├── main.c              ← WiFi + UART 转发主循环
│   ├── blackbox_config.h   ← WiFi SSID/密码/端口配置
│   └── CMakeLists.txt
├── partitions.csv
└── sdkconfig.defaults
```

用 ESP-IDF 构建。配置好 WiFi 后上电自动连接，PC 端开一个 TCP 客户端连到 ESP32 的 IP 地址即可接收遥测数据流。
