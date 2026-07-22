# AGENTS.md

本文档供 AI 编码助手（Claude Code、Codex、Copilot 等）在操作本仓库时使用。纯技术信息，不含工具专属配置。

## 项目定位

全国大学生电子设计竞赛（NUEDC）控制类备赛项目——基于 **MSPM0G3507**（TI Cortex-M0+ 80MHz）的差速电机小车。仓库名 "DianJiXZ" = 电机小车。

## 硬件平台

| 模块 | 型号/参数 | 接口 |
|------|----------|------|
| MCU | MSPM0G3507 | — |
| 电机驱动 | TB6612 (双 H 桥) | GPIO 方向 + TimerA PWM (10kHz, 0-7800/8000) |
| 编码器 | 正交编码器 13线, 30:1减速比 | GPIO 上升沿中断 |
| 灰度传感器 | 8路红外 (3-8模拟开关分时选通) | 3地址线 + 1数据线 |
| IMU | JY62 6轴 (yaw/pitch/roll) | UART1 115200bps 11字节帧 |
| OLED | SSD1306 128×64 | — |
| 视觉芯片 | **K230 CanMV** (UART_0) | UART 0xAA头+8字节XOR协议 |
| 遥测 | ESP32-S3 黑匣子 | UART → WiFi → PC |

## 项目结构

主要代码在 `firmware/WHEELTEC_C07A_CAR/`：

```text
firmware/WHEELTEC_C07A_CAR/
├── empty.c/h              # 入口
├── Control/               # 控制算法层
│   ├── control.c/h        # 巡线 + 分档速度控制 + 任务状态机
│   ├── show.c/h           # OLED + DataScope 显示
│   └── uart_callback.c/h  # UART 回调
├── Hardware/              # 外设驱动层
│   ├── motor.c/h          # TB6612 电机驱动
│   ├── encoder.c/h        # 编码器 (2倍频)
│   ├── CCD.c/h            # 8路灰度传感器 (加权质心法)
│   ├── oled.c/h           # OLED SSD1306
│   ├── adc.c/h            # ADC 采集 (12位, 3.3V参考)
│   ├── jy62.c/h           # JY62 IMU (0x55帧解析)
│   ├── k210_link.c/h      # K230 视觉通信 (二进制+ASCII握手)
│   ├── key.c/h            # 按键
│   ├── led.c/h            # LED 状态指示
│   └── board.c/h          # 板级初始化
├── keil/                  # Keil uVision5 工程
│   └── empty_LP_MSPM0G3507_nortos_keil.uvprojx
└── source/                # TI DriverLib + CMSIS (subset)
```

## 构建

**IDE**: Keil uVision5 → 打开 `firmware/WHEELTEC_C07A_CAR/keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，目标 `MSPM0G3507_Project`。

**命令行**:

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b '...\firmware\WHEELTEC_C07A_CAR\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project'
```

**外设配置**: 用 TI SysConfig 图形化工具生成 `ti_msp_dl_config.c/h`，**不要手动编辑生成文件**。

## 控制架构

```
5ms TimerG 中断 (200Hz)
├── 读编码器 → 换算速度
├── Gray_Mode() 读8路灰度 → 加权质心 → 三档变速 → 曲率转向
├── Get_Target_Encoder() 运动学逆解 (线速度+角速度→左右轮速)
├── Incremental_PI() ×2 速度闭环 (Kp=400, Ki=300)
└── Set_PWM() 输出TB6612

主循环 while(1)
├── OLED 刷新 + DataScope 显示
├── 蓝牙/UART 数据处理
└── VOFA+/按键/任务状态机推进
```

### 关键参数

| 参数 | 值 |
|------|-----|
| 控制周期 | 5ms (200Hz) |
| 轮距 | 0.130 m |
| 轮周长 | 0.2105 m |
| 编码器分母 | 780 (=13线×2倍频×30减速比) |
| 直线速度 | 300 mm/s |
| 弯道速度 | 185 mm/s |
| 十字速度 | 310 mm/s |
| 转向增益 | 1.70 |
| PWM 频率/上限 | 10kHz / 7800 (97.5%) |
| UART0 | K230 视觉 (复用) |
| UART1 | JY62 IMU + 蓝牙 + VOFA+ 调参 |

## 协作仓库

队友 `yangran` — [https://github.com/yangran12/DianJiXZ](https://github.com/yangran12/DianJiXZ)

```bash
git remote add yangran https://github.com/yangran12/DianJiXZ.git
git fetch yangran
git merge yangran/main
```

**注意**: 只 fetch/merge 拉取队友代码，**绝不 push 到队友仓库**。

队友贡献文件：`firmware/.../Hardware/k210_link.c/h`（K230通信）、`firmware/ESP32S3_Blackbox/`（黑匣子）。

## 编码规范

- C 代码遵循 MSPM0 DriverLib 风格——`DL_xxx()` API + SysConfig 生成配置
- 注释用中文，和现有代码保持一致
- 改动最小化——手术式修改，不过度设计、不重写已有模块
- 新外设参照 `Hardware/` 下现有驱动结构：一个 `.c` + 一个 `.h`，命名小写
- 控制参数放 `Control/control.h` 顶部 `#define`

## AI 协作：如何利用 docs_learn/

`docs_learn/` 是本项目共享知识库——**写代码前先查对应文档**：

| 要做什么 | 先看 |
|----------|------|
| 写/改 PID 控制 | `docs_learn/ques2.md` (PWM+PI), `docs_learn/learn.md` 第一节 |
| 写/改 巡线算法 | `docs_learn/ques3.md` (灰度巡线), `docs_learn/ques4.md` (运动学) |
| 加 DMA 外设 | `docs_learn/ques5.md` (DMA配置模板) |
| 用 IMU | `docs_learn/ques6.md` (JY62帧解析+断桥导航) |
| 加在线调参 | `docs_learn/ques9.md` (VOFA+ JustFloat协议) |
| 写任务状态机 | `docs_learn/ques6.md` 第五部分 + `docs_learn/learn.md` 任务状态机条目 |
| 视觉通信 (K230) | `docs_learn/ques8.md` (K230 Link协议) |
| 理解硬件接口 | `docs_learn/ques1.md` (传感器信号类型) |

每个 Q&A 文档标注了代码出处 (`文件名:行号`)，建议**开两个窗口**——一边读文档一边对着 Keil 工程写代码。

## 视觉芯片

**已选定 K230（CanMV 固件），放弃 K210。** K230 兼容 OpenMV API（`find_blobs()` 等），UART_0 发识别帧给 MSPM0。详见 `workflow/`。

## 当前主攻方向

2024 年 H 题（自动行驶小车）— T4 逆时针 4 圈。赛题分析见 `docs_2024/`。
