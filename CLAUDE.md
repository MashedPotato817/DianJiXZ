# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

全国大学生电子设计竞赛（NUEDC）控制类备赛项目——基于 MSPM0G3507 的差速电机小车平台。仓库名 "DianJiXZ" = 电机小车。

## 项目结构

主要代码在 `firmware/WHEELTEC_C07A_CAR/`：

```text
firmware/WHEELTEC_C07A_CAR/
├── empty.c/h              # 入口
├── Control/               # 控制算法层
│   ├── control.c/h        # 巡线 + 分档速度控制
│   ├── show.c/h           # OLED + DataScope 显示
│   └── uart_callback.c/h  # UART 回调
├── Hardware/              # 外设驱动层
│   ├── motor.c/h          # TB6612 电机驱动
│   ├── encoder.c/h        # 编码器
│   ├── CCD.c/h            # 8路灰度传感器
│   ├── oled.c/h           # OLED SSD1306
│   ├── adc.c/h            # ADC 采集
│   ├── key.c/h            # 按键
│   ├── led.c/h            # LED
│   └── board.c/h          # 板级初始化
├── keil/                  # Keil uVision5 工程文件
│   └── empty_LP_MSPM0G3507_nortos_keil.uvprojx
└── source/                # TI DriverLib + CMSIS (subset)
```

## 构建

用 Keil uVision5 打开 `firmware/WHEELTEC_C07A_CAR/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，目标名 `MSPM0G3507_Project`。

命令行构建：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b '...\firmware\WHEELTEC_C07A_CAR\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project'
```

见 README 中的完整构建说明和当前参数表。

## 比赛参考资料

`电赛控制类题目总结_2019-2025.md` — 2019-2025 年电赛控制类题目分析、评分标准、备赛建议和代码框架推荐。

`demand.md` — 待填充的需求文档。

## 协作仓库

队友：`yangran` [https://github.com/yangran12/DianJiXZ](https://github.com/yangran12/DianJiXZ)

```bash
git remote add yangran https://github.com/yangran12/DianJiXZ.git
git fetch yangran
git merge yangran/main
```

队友贡献了 `k210_link.c/h`（K210/K230 通信链路）、**ESP32-S3 黑匣子遥测**、陀螺仪里程计、H 题任务状态机等。关键文件：

| 文件 | 内容 |
|------|------|
| `firmware/K210/k210_uart_test.py` | 视觉芯片端 UART 测试脚本 |
| `firmware/.../Hardware/k210_link.c/h` | MSPM0 端通信——**二进制协议**（0xAA 头+8字节 XOR）和**ASCII 握手**（$CAR,HELLO# / $K210,OK#）。K230 可复用此协议框架 |
| `firmware/ESP32S3_Blackbox/` | ESP32-S3 遥测黑匣子 |

视觉芯片用 **UART_0**（和 JY62/蓝牙的 UART_1 分开）。

## 视觉芯片选型

**已选定 K230（CanMV），放弃 K210。** K210 工具链老旧、MicroPython 支持差、通信未调通。K230 CanMV 固件兼容 OpenMV API（`find_blobs()` 等函数直接可用），¥188~249，淘宝搜"立创庐山派 K230"或"CanMV K230"。详见 `workflow/`。

## 编码规范

编辑 `.c` / `.h` / `.js` / `.html` / `.py` 文件前，**必须先调用** `andrej-karpathy-skills:karpathy-guidelines`，遵循其简洁、手术式修改、不过度设计的原则。

## 项目记忆

项目记忆存放于 `.claude/projects/` 下对应的 session 目录。
