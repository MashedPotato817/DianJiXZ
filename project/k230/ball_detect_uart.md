# K230 小球检测与 UART 输出

> 相机、摆杆和小球运动平面已机械固定，可开始 `x_mm` 的像素—毫米标定。本文件中的 UART 帧可用于验证传输与解帧；在未完成标定前，视觉位置默认不得用于舵机闭环。

未标定时，`ball_detect_config.py` 中的 `CALIBRATION_READY` 必须保持 `False`。默认情况下，`k230_ball_detect_uart.py` 仍以 20 Hz 发送 `$K230,BALL,0.0,0,<seq>#`，用于验证 UART 链路和 MSPM0 解帧。

本轮已授权低幅度闭环联调，可临时设置 `UART_ALLOW_UNCALIBRATED=True`：脚本会发送当前临时映射的 `x_mm` 和检测 `valid`，控制端必须保持小脉宽限幅与失帧回中位。该开关**不等于完成标定**；联调后应恢复为 `False`，正式闭环与性能结论仍以五点标定结果为准。

纯视觉预览入口：`k230_ball_detect_preview.py`；UART 入口：`k230_ball_detect_uart.py`。UART 脚本也会在 CanMV IDE/LCD 显示同帧预览，便于联调时核对 `OK/HD/LS`、`x_mm` 与串口输出；参数：`ball_detect_config.py`。

当前仅验证视觉时，上传 `k230_ball_detect_preview.py` 和 `ball_detect_config.py`，在 CanMV IDE 运行预览脚本。IDE 画面中：蓝线是标定中心，绿圈与红十字是候选球，左上角 `BALL OK` 表示连续确认有效，`BALL LS` 表示未确认或丢球。

它使用 `cv_lite.grayscale_find_circles()` 检测灰度图中的小球，并发送已经验证的 UART 帧：

```text
$K230,BALL,<x_mm>,<valid>,<seq>#\r\n
```

接线：K230 IO40 到 MSPM0 PB7，K230 IO41 到 PB6，两板共地。

首次运行前必须标定 `ball_detect_config.py`：

1. 将小球放在摆杆中心，设置 `IMAGE_CENTER_X`。
2. 将小球向右移动已知距离，计算并设置 `MM_PER_PIXEL`；图像向右应为正值。
3. 收紧 `BALL_ROI` 至摆杆可见区域。
4. 默认 `DETECT_MODE="cvlite_circle"`，需使用含 `cv_lite` 的 CanMV 固件；先在 K230 REPL 执行 `import cv_lite` 确认。
5. 小球颜色稳定时，可把 `DETECT_MODE` 设为 `blob`，并在 CanMV IDE 中标定 `BALL_LAB_THRESHOLD`，作为更快的替代方案。
6. 调整圆半径、连续确认和最大跳变阈值，使静止球先稳定输出 `valid=1`，遮挡后稳定输出 `valid=0`。

未在本机或桌面 Python 上模拟 CanMV 相机 API；需要在目标 K230 固件上验证。验证时 K230 控制台应持续输出 `valid=1`，MSPM0 端首次有效帧应返回 `$MSPM0,ACK#`。
