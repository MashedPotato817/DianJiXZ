# ESP32-S3 遥测黑匣子

## 用途与状态

这是一个可选调试工程：ESP32-S3 从 MSPM0 的 UART 接收文本遥测数据，通过 Wi-Fi 网页、UDP 广播和 SPIFFS 日志提供观测能力。它不参与电机闭环，启用前必须确认不会与视觉、IMU 或蓝牙占用同一串口。

## 功能

- UART telemetry input from MSPM0
- WiFi AP mode
- Browser dashboard at `http://192.168.4.1/`
- Raw telemetry log saved to SPIFFS
- Log download and clear endpoints
- UDP broadcast on port `3333`

## 默认接线

| ESP32-S3 | MSPM0 |
| --- | --- |
| GPIO18 RX | MSPM0 UART0 TX / PB0 |
| GPIO17 TX | optional MSPM0 UART0 RX / PB1 |
| GND | GND |

默认 UART 波特率为 `115200`。

## Wi-Fi 配置

默认 SSID、密码、UART 引脚和日志容量定义在 `main/blackbox_config.h`。连接后访问 `http://192.168.4.1/`；用于比赛或公开网络前，应修改默认凭据。

## 构建

使用安装了 ESP-IDF 扩展的 VS Code 打开本目录，或在 ESP-IDF 终端执行：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 遥测协议

版本 1 接收任意以换行结束的 MSPM0 文本行。

Current MSPM0 line format:

```text
T=123456,TASK=2,LAP=0,MODE=1,BC=0,YAW=472,H=468,X=123,Y=-5,S=130,D=118,V=74,VF=1234,ER=2
```

ESP32 会记录原始行、在浏览器显示，并通过 UDP 广播。

常用缩放字段：

- `YAW` and `H` are degrees x10
- `X`, `Y`, `S`, `D` are millimeters
- `V` is battery voltage x10
