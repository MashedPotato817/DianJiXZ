# MSPM0 主控工程

> 状态：**源码骨架已完成**（2026-07-25），待复制 Keil 工程 + SysConfig + DriverLib 后可编译。

基于 `WHEELTEC_C07A_CAR` 底盘基线，仅保留送药车必需模块。

## 当前文件结构

```text
mspm0/
├── README.md
├── empty.c/h                    # 主入口：初始化 + 主循环（K230→状态机→OLED）
├── Control/
│   ├── control.c/h              # 5ms ISR、编码器速度换算、PI、运动学逆解、PWM
│   ├── line_follow.c/h          # 8 路灰度巡线（从 Gray_Mode 迁移，待 Codex 实测修正符号）
│   ├── medicine_task.c/h        # 6 状态机 + 故障停车（传感器接入点为 TODO）
│   └── route.c/h                # 路线表 + 到位判据（Task 4 占位）
├── Hardware/
│   ├── motor.c/h                # TB6612 PWM + 方向（PB2/PB3, PA13/14, PA16/17）
│   ├── encoder.c/h              # 编码器 GPIO 中断（PA25/26, PB20/24）
│   ├── board.c/h                # printf → UART0，延时，类型定义
│   ├── k230_link.c/h            # UART1 K230 通信：$...# 协议、握手、心跳、RESULT 帧解析
│   ├── oled.c/h/oledfont.h      # SSD1306 显示
│   ├── key.c/h                  # 按键扫描
│   ├── adc.c/h                  # 电池电压
│   ├── led.c/h                  # PB9 LED
│   ├── load_detect.c/h          # 装载/卸载检测（Task 3 占位）
│   └── indicator.c/h            # 红绿指示灯（待分配 GPIO）
├── keil/                        # ← 待复制
├── source/                      # ← 待复制
├── empty.syscfg                 # ← 待复制 + 修改 UART 分配
└── ti_msp_dl_config.c/h         # ← SysConfig 重新生成
```

## 串口分配

| UART | 引脚 | 用途 | 模式 |
|------|------|------|------|
| UART0 | PA10/PA11 | printf 调试输出 | 115200, 阻塞发送 |
| UART1 | PB6/PB7 | **K230 通信** | 115200 8N1, 轮询 FIFO 接收 |

K230 初始化和协议在 `k230_link.c` 中——握手、心跳、超时逻辑完整，`$K230,RESULT,<ward>,<conf>#` 帧解析已实现。

## K230 协议帧

| 帧 | 方向 | 用途 |
|----|------|------|
| `$K230,HELLO#` / `$MSPM0,HELLO#` | 双向 | 握手（500ms 周期） |
| `$K230,ACK#` / `$MSPM0,ACK#` | 双向 | 握手应答 |
| `$MSPM0,LINK_OK#` | MSPM0→K230 | 握手完成 |
| `$K230,DATA#` / `$MSPM0,DATA#` | 双向 | 心跳（1 Hz） |
| **`$K230,RESULT,<ward>,<conf>#`** | K230→MSPM0 | **识别结果**（ward=1-8, conf=0-100） |
| `$MSPM0,LINK_LOST#` | MSPM0→K230 | 3 秒超时断链 |

## Codex 待办

在写巡线控制逻辑前，Codex 需在实车完成：

| # | 任务 | 依据 |
|---|------|------|
| 1 | 通道映射：遮最左/最右传感器 → 记录 `Gray_Data[0..7]` | research.md §四.1 |
| 2 | 轮子方向：抬车 45mm/s，确认黑线居中→同向前转；偏左/右→哪侧加速 | research.md §四.2 |
| 3 | 红线响应：赛题场地采集白底/红线下 8 路原始读数 | task1.md |
| 4 | 装载传感器调研：称重 vs 限位开关 vs 红外遮挡 | task3.md |

1–2 结果决定 `line_follow.c:80` 的转向符号；3 决定 `GRAY_BLACK_LEVEL` 和 `Gray_ToBlack()` 逻辑。

## 未决事项（不影响当前编译）

| 事项 | 状态 |
|------|------|
| 转向符号修正 | 等 Codex 通道映射结果 |
| 红线判定阈值 | 等 Task 1 实测 |
| 装载检测传感器 | 等 Codex 调研 |
| 红绿 LED GPIO | 待分配 |
| 路线表参数 | 等 Task 4 |

## 编译准备（需手动完成）

1. **复制 Keil 工程**：从 `WHEELTEC_C07A_CAR/keil/` 复制 `.uvprojx`、`.uvoptx`、`.sct`、startup 到 `mspm0/keil/`
2. **复制 DriverLib**：`WHEELTEC_C07A_CAR/source/` → `mspm0/source/`
3. **复制 SysConfig**：`WHEELTEC_C07A_CAR/empty.syscfg` → `mspm0/empty.syscfg`
4. **修改 SysConfig**：用 TI SysConfig GUI 打开 `empty.syscfg`，做以下更改：
   - UART0：启用，115200（调试 printf）
   - UART1：**关闭 DMA 和中断**，波特率改 115200（K230 通信）
   - 移除工程中不需要的源文件引用（`uart_callback.c`、`show.c`、`CCD.c`、`DataScope_DP.c`）
5. **重新生成** `ti_msp_dl_config.c/h`
6. **Keil 工程中**：移除旧文件引用，添加新模块文件到编译列表
