# MSPM0 主控工程

> 状态：架构草稿。当前目录不包含源代码、Keil 工程或 SysConfig 生成文件。

本目录将承载智能送药小车的 MSPM0G3507 独立控制工程。后续以 [`../../WHEELTEC_C07A_CAR/`](../../WHEELTEC_C07A_CAR/) 为已验证底盘基线复制建立，送药车专用功能只在本目录维护，不回写通用底盘工程。

## 计划目录

```text
mspm0/
├── README.md                 # 本架构说明
├── keil/                     # 独立 Keil 工程
├── Control/
│   ├── control.c/h           # 5ms 控制调度、速度闭环与电机输出
│   ├── medicine_task.c/h     # 送药任务状态机
│   ├── route.c/h             # 病房路线表、路口与到位判据
│   └── line_follow.c/h       # 红线巡线接口与失线处理
├── Hardware/
│   ├── motor.c/h             # TB6612 电机驱动（底盘复用）
│   ├── encoder.c/h           # 编码器采集（底盘复用）
│   ├── k230_link.c/h         # K230 串口接收、链路状态与识别结果
│   ├── load_detect.c/h       # 装载/卸载检测
│   ├── indicator.c/h         # 红、绿指示灯控制
│   └── board.c/h             # 板级初始化
├── source/                   # TI DriverLib、CMSIS 与 SysConfig 生成文件
└── empty.c/h                 # 工程入口
```

上述为目标结构；仅在对应功能开始实现时创建文件，避免空模块和未使用的抽象。

## 模块边界

| 模块 | 职责 | 不负责 |
| --- | --- | --- |
| `line_follow` | 根据红线位置计算前进/转向请求，处理丢线 | 病房选择、路口路径和任务状态跳转 |
| `route` | 将病房号映射为去返路线、路口动作和停车判据 | 电机 PWM、串口解析 |
| `medicine_task` | 管理 `WAIT_TARGET → WAIT_LOAD → OUTBOUND → ARRIVED → RETURN → FINISHED` | 传感器底层读取、速度 PI |
| `k230_link` | 维护 UART 链路，输出稳定病房号与通信状态 | 直接启动电机或修改任务状态 |
| `load_detect` | 输出已装载、已卸载状态 | 路径决策和指示灯策略 |
| `indicator` | 按任务状态控制红、绿灯 | 状态机跳转 |
| `control` | 调度实时运动控制、将目标速度输出至电机 | 复杂字符串解析、视觉推理和任务策略 |

## 实时职责划分

### 5ms 定时中断

- 读取编码器并计算轮速。
- 当任务状态允许行驶时，执行巡线计算与路径动作。
- 执行左右轮速度 PI，输出 PWM。
- 只使用确定时长的运算；不做 UART 文本解析、OLED 刷新、文件操作或阻塞等待。

### 主循环

- 处理 K230 UART 接收缓冲和链路超时。
- 读取装载/卸载检测结果并执行去抖。
- 推进送药任务状态机，管理故障停车。
- 刷新 OLED/调试信息和记录测试数据。

## 外部接口

- 巡线基线：复用 `WHEELTEC_C07A_CAR/Control/control.c` 中的编码器、速度 PI 和 8 路灰度巡线思路；红线识别逻辑须在 Task 1 实测通过后确定。
- 视觉通信基线：复用 `K230_UART_Demo` 的 IO40/IO41 接线、115200 8N1、`$...#` 帧定界、握手和超时机制；识别结果报文在 Task 2 定稿。
- 路径和装卸检测：分别在 Task 3、Task 4 确认硬件方案与可量化判据后接入。

## 实现约束

- 不修改 `../../WHEELTEC_C07A_CAR/` 的通用底盘基线，也不修改 `../../K230_UART_Demo/` 的验证工程。
- 不修改上级 `topic/` 中的原始赛题资料。
- `ti_msp_dl_config.c/h` 由 TI SysConfig 生成，禁止手动编辑。
- 接入 K230 前先解决 UART1 与 JY62、蓝牙、VOFA+ 的资源冲突，明确送药车最终串口分配。
- 每完成一个模块，按 [`../docs/task1.md`](../docs/task1.md) 至 [`../docs/task6.md`](../docs/task6.md) 的完成判据验证后再进入下一任务。

## 建立工程时的最小复制范围

在进入 Task 5 前，复制 `WHEELTEC_C07A_CAR` 的 Keil 工程、入口文件、`Control/`、`Hardware/`、`source/` 以及 SysConfig 生成文件至本目录；先确保原始底盘功能可独立编译和运行，再做送药车专用改动。
