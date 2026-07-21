# K230 UART 最小通信 Demo

该工程验证 K230 CanMV 与 MSPM0G3507 控制主板的双向 UART 通信。双方周期发送 `HELLO`，收到对方 `HELLO` 后回复 `ACK`；两端均已收到 `HELLO` 和 `ACK` 才会进入在线状态，并每秒互发 `DATA`。

## 接线

| K230 Port1 | MSPM0G3507 控制主板 |
| --- | --- |
| IO40 / TX | PB7（UART1_RX） |
| IO41 / RX | PB6（UART1_TX） |
| GND | GND |

只连接信号线和公共地。K230 扩展口的 5V 引脚不可接入主控 UART 信号。

## 成功判据

K230 控制台依次出现 `RX: $MSPM0,HELLO#`、`RX: $MSPM0,ACK#`，随后出现 `LINK OK: 双方 HELLO/ACK 已确认`，即表示双向握手成功。之后每秒能看到双方的 `DATA` 与 `DATA_ACK`。若 3 秒没有收到 MSPM0 数据，会显示 `LINK LOST`。

## 使用步骤

1. 在 Keil 打开 `keil/K230_UART_Demo.uvprojx`，选择目标 `K230_UART_Demo` 后编译、下载。
2. 在 CanMV IDE 打开并运行 `k230_uart_demo.py`。默认使用 Port1；使用 Port2 时将 `K230_PORT` 改为 `2`。
3. 按上方“成功判据”检查握手和双向数据。

串口参数固定为 115200、8N1。
