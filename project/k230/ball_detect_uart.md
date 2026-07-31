# K230 小球检测与 UART 输出

> 最后更新：2026-07-31 10:03

> 相机、摆杆和小球运动平面已机械固定。本文件说明 K230 UART 发送、预览、终端与 TXT 日志的当前行为。正式位置标定尚未完成；临时低幅度闭环仅用于联调，不构成性能结论。

## 当前运行边界

- `CALIBRATION_READY=False`：三点映射仅作临时预览/联调，不能用作位置精度结论。
- `UART_ALLOW_UNCALIBRATED=True`：允许发送临时 `x_mm` 和 `valid`，仅限已确认中位、方向、限位后的低幅度闭环。
- `SEND_PERIOD_MS=50`：发送周期配置目标为 20 Hz；已有 K230 日志估算实际发送约 16 Hz，应以实测为准。
- MSPM0 只在首帧合法 BALL 数据时回一次 `$MSPM0,ACK#`；它证明链路可解析，**不是逐帧确认机制**。

纯视觉预览入口：`k230_ball_detect_preview.py`；UART 入口：`k230_ball_detect_uart.py`；参数：`ball_detect_config.py`。

## UART 帧与接线

脚本使用 `cv_lite.grayscale_find_circles()` 检测灰度图中的小球，并发送：

```text
$K230,BALL,<x10>,<valid>,<seq>,<edge>*<crc8>#\r\n
```

- `x10` 为毫米位置乘 10 的有符号整数，例如 `247` 表示 `+24.7 mm`。
- `edge` 固定存在：`-1` 表示最后可靠球心靠近左边缘后丢失，`+1` 表示右边缘，`0` 表示正常或普通漏检。
- `crc8` 是两位十六进制 CRC-8/ATM，参数为 `poly=0x07`、`init=0x00`，覆盖 `$` 后到 `*` 前的 ASCII 字节。
- M0 只接受字段完整、CRC正确且 `x10` 在 `±1500` 内的帧。旧的无校验帧仅适合查阅历史日志，不能进入闭环。

示例：`$K230,BALL,-250,1,1,0*B9#` 表示 `-25.0 mm` 的第1帧有效数据。

```text
K230 IO40 / UART1_TX  -> MSPM0 PB7 / UART1_RX
K230 IO41 / UART1_RX  <- MSPM0 PB6 / UART1_TX
K230 GND              <-> MSPM0 GND
115200, 8N1
```

## 终端与 TXT 日志

默认输出以“运行必要信息”为限，避免 MSPM0 心跳和原始 RX 字节刷屏。

| 位置 | 默认保留 | 默认不输出 |
|---|---|---|
| CanMV 终端 | 启动、安全告警、首次 `LINK ACK`、`STATE OK/HD/LS` 状态变化 | 原始 `RX: ...`、每 500 ms 心跳、连续数值摘要、固定部署配置 |
| TXT 文件 | 上述事件；每秒一条调参摘要 | 原始 `RX: ...`、逐次心跳 |

TXT 摘要示例：

```text
BALL OK VIS=+12.3/1 TX=+11.8/1 edge=+0 seq=123 age=0ms fps=28.4
```

- `VIS`：当前循环从相机画面得到的检测值。
- `TX`：最近一次真正写入 UART 的帧。它可比 `VIS` 旧一个发送周期，因此二者不同不等于串口错误。
- `age`：当前时刻距离最近发送帧的时间；`seq`：最近发送的递增序号。

需要临时增加诊断时，只改 `ball_detect_config.py`：

| 开关 | 默认 | 用途 |
|---|:---:|---|
| `ENABLE_UART_RX_DEBUG` | `False` | 输出原始 MSPM0 回包，排查乱码/回传线时才打开。 |
| `ENABLE_STARTUP_CONFIG_LOG` | `False` | 输出检测模式、IO 与波特率等固定部署信息。 |
| `ENABLE_CONSOLE_SUMMARY` | `False` | 将每秒 TXT 摘要同步打印到终端。 |
| `ENABLE_LOG_SUMMARY` | `True` | 保留每秒 TXT 调参样本；需要极简日志时可关闭。 |

CanMV 固件的 `find sensor ...` 等初始化行不属于应用脚本输出，不能通过上述开关屏蔽。

## 视觉与串口验证

仅验证视觉时，上传 `k230_ball_detect_preview.py` 和 `ball_detect_config.py`，在 CanMV IDE 运行预览脚本。IDE 画面中：蓝线是标定中心，绿圈与红十字是候选球，左上角 `BALL OK` 表示连续确认有效，`BALL LS` 表示未确认或丢球。

首次运行前必须标定 `ball_detect_config.py`：

1. 将小球放在摆杆中心，设置 `IMAGE_CENTER_X`。
2. 将小球向右移动已知距离，计算并设置 `MM_PER_PIXEL`；图像向右应为正值。
3. 收紧 `BALL_ROI` 至摆杆可见区域。
4. 默认 `DETECT_MODE="cvlite_circle"`，需使用含 `cv_lite` 的 CanMV 固件；先在 K230 REPL 执行 `import cv_lite` 确认。
5. 小球颜色稳定时，可把 `DETECT_MODE` 设为 `blob`，并在 CanMV IDE 中标定 `BALL_LAB_THRESHOLD`，作为更快的替代方案。
6. 调整圆半径、连续确认和最大跳变阈值，使静止球先稳定输出 `valid=1`，遮挡后稳定输出 `valid=0`。

未在本机或桌面 Python 上模拟 CanMV 相机 API；需要在目标 K230 固件上验证。正常运行时，终端应先显示一次 `LINK ACK`，随后只在状态切换时显示 `STATE`。TXT 的 `BALL ...` 摘要用于复核视觉、发送节拍与 FPS。

> 最后更新：2026-07-31 10:03
