# K230 UART 最小通信 Demo

## 用途与状态

该工程验证 K230 CanMV 与 MSPM0G3507 控制主板的双向 UART 通信，是视觉项目接入前的最小验证工程，不包含数字识别、色块识别或小车任务状态机。

当前已在实物上验证旧主板固件的兼容握手：主板周期发送 `$CAR,HELLO#`，K230 收到后回复 `$K210,OK#`。主板 OLED 显示 `K ON H` 时，表示主板已接收到应答并确认链路在线。

## 硬件与接线

| K230 Port1 | MSPM0G3507 控制主板 |
| --- | --- |
| IO40 / TX | PB7（UART1_RX） |
| IO41 / RX | PB6（UART1_TX） |
| GND | GND |

只连接信号线和公共地。K230 扩展口的 5V 引脚不可接入主控 UART 信号。

## 已验证的旧主板握手

K230 控制台应持续出现以下成对日志：

```text
RX: $CAR,HELLO#
TX: $K210,OK#
```

对应主板工程的 `k210_link.c` 使用该应答将链路标记为在线。`$K230,HELLO#` 是新协议测试报文，旧主板不会处理，可忽略。

## 代码入口

- `uart_demo.c`：MSPM0 侧 UART 演示程序。
- `k230_uart_demo.py`：CanMV 侧握手与测试帧程序。
- `keil/K230_UART_Demo.uvprojx`：Keil 工程，目标 `K230_UART_Demo`。

## 使用步骤

1. 在 Keil 打开 `keil/K230_UART_Demo.uvprojx`，选择目标 `K230_UART_Demo` 后编译、下载。
2. 在 CanMV IDE 打开并运行 `k230_uart_demo.py`。默认使用 Port1；使用 Port2 时将 `K230_PORT` 改为 `2`。
3. 确认 K230 控制台出现上方的收发日志，并检查主板 OLED 显示 `K ON H`。

串口参数固定为 115200、8N1。
