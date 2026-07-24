# firmware 项目导航

本目录不是单一工程，而是多个相互独立的固件项目。进入某个项目之前，请先阅读该项目的 README，并以该项目中的 Keil/ESP-IDF 配置、接线表和验证范围为准。

| 项目 | 平台 | 用途与状态 | 入口 |
| --- | --- | --- | --- |
| [WHEELTEC_C07A_CAR](WHEELTEC_C07A_CAR/README.md) | MSPM0G3507 + WHEELTEC C07A | 基础差速车工程：编码器闭环、8 路灰度巡线、OLED 与遥控。可作为新小车赛题的底盘起点。 | `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` |
| [WHEELTEC_C07A_CAR_SPEEDUP](WHEELTEC_C07A_CAR_SPEEDUP/README.md) | MSPM0G3507 + WHEELTEC C07A | 提速试验工程：基于基础巡线版本，当前采用 `90/75 mm/s` 直线/弯道两档速度，待实车 PID 调整与验证。 | `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` |
| [WHEELTEC_C07A_CAR_JY62_GapBridge](WHEELTEC_C07A_CAR_JY62_GapBridge/README.md) | MSPM0G3507 + JY62 + 云台 | 实验/参考工程，保留断桥、姿态、任务状态机与视觉链路探索；当前入口为云台 PWM 演示，未完成。 | `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` |
| [K230_UART_Demo](K230_UART_Demo/README.md) | K230 CanMV + MSPM0G3507 | K230 与 MSPM0 最小 UART 握手验证；不包含目标识别与小车任务。 | `keil/K230_UART_Demo.uvprojx`、`k230_uart_demo.py` |
| [ESP32S3_Blackbox](ESP32S3_Blackbox/README.md) | ESP32-S3 | 可选遥测黑匣子：UART 接收、Wi-Fi 网页、UDP 与 SPIFFS 日志。 | ESP-IDF 工程根目录 |

## 选择原则

- 要做基础巡线、速度闭环或新的差速车任务：从 `WHEELTEC_C07A_CAR` 复制出新项目，再按赛题改动。
- 要在已验证底盘上调速度、调 PID 或测试小半径弯：使用 `WHEELTEC_C07A_CAR_SPEEDUP`，不要反向改动基础工程。
- 要研究 JY62、断桥、历史 H 题逻辑或云台：仅将 `WHEELTEC_C07A_CAR_JY62_GapBridge` 当作参考，先确认入口程序和串口分配。
- 要验证视觉板串口：使用 `K230_UART_Demo`，验证完成后再迁移协议到目标小车工程。
- 要采集调试数据：单独启用 `ESP32S3_Blackbox`；先确认它与目标工程的 UART 引脚不会冲突。

## 新增项目时

新建目录后至少提供：`README.md`、构建工程文件、入口源文件和必要的配置文件。README 应明确“已验证”与“计划中”，避免把实验分支误当作可直接比赛的工程。
