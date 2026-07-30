# K230 小球视觉模块

本目录是 H 题平衡滚球系统的 K230 视觉端。它只负责从相机画面中检测小球、计算横向位置并输出状态；不直接控制 MSPM0 底盘或舵机。

当前默认方案是 `cv_lite` 灰度霍夫圆检测，而不是通用 AI 目标检测模型：小球是单一圆形目标，位置连续，使用传统视觉无需训练、标注和部署 `.kmodel`，更适合当前的首轮实测。

## 当前状态与限制

- K230 已实机运行预览脚本，曾观察到 `valid=1`、`x_mm=44.0`、约 `60 FPS`；这只证明当前画面下的圆检测和时序筛选在运行。
- **相机、摆杆和小球运动平面已机械固定**。已用静态 `-50/0/+50 mm` 三点数据完成临时预览标定；在完成五点、每点三次重复测量和总误差评估前，`x_mm` 不能用于舵机控制。
- 当前已完成画面预览和 UART 帧收发验证；下一步补做中心、比例、方向与全行程误差标定。

## 当前文件

| 文件 | 用途 |
| --- | --- |
| `k230_ball_detect_preview.py` | 纯视觉预览。使用双通道相机，叠加检测结果到 CanMV IDE/LCD；不使用 UART。 |
| `ball_detect_config.py` | 圆检测、时序筛选、像素到毫米标定和 UART 参数。 |
| `k230_ball_detect_uart.py` | 在预览验证通过后，发送 BALL 帧给 MSPM0。 |
| `ball_detect_uart.md` | 参数标定与 UART 联调补充说明。 |
| `k230_uart_ball_test.py` | 不使用相机的固定 BALL 帧通信测试。 |
| `k230_uart_rx_test.py` | 仅验证 MSPM0 到 K230 的回传线。 |

## 环境与固件

已知目标板为 DNK230D，K230 端串口使用：

```text
UART1_TX = IO40
UART1_RX = IO41
115200, 8N1
```

预览脚本依赖 `cv_lite`。在 **K230 实机** 的 CanMV REPL 中先执行：

```python
import cv_lite
print("cv_lite OK")
```

导入失败时，不要用桌面 Python 结果替代；记录固件版本和错误信息后，更换为支持 `cv_lite` 的 CanMV 固件再继续。官方 API 参考：

- [CanMV-K230 快速入门](https://www.kendryte.com/k230_canmv/v0.7/zh/)
- [CanMV-K230 API 手册](https://www.kendryte.com/k230_canmv/v0.7/zh/api/)
- [cv_lite 模块 API](https://www.kendryte.com/k230_canmv/v0.7/zh/api/cv_lite/cv_lite.html)

## 第一阶段：只验证小球运动检测

此阶段不要连接 UART，也不要启动 MSPM0。

1. 将 `ball_detect_config.py` 与 `k230_ball_detect_preview.py` 上传到 K230 的同一目录。
2. 在 CanMV IDE 打开并运行 `k230_ball_detect_preview.py`。
3. 查看 IDE 预览：

   - 青线 / 蓝线 / 紫线：按当前标定映射绘制的 `-50 mm / 0 / +50 mm` 参考位置；
   - 黄框：当前 `BALL_ROI`。横向保持全宽，按画面中的白管运动带仅收紧为 `y=85…139 px`；五点日志应复核它是否为小球半径保留了约 `10 px` 的余量；
   - 绿圈和红十字：当前小球候选；
   - `BALL OK`：连续检测达到 `STABLE_FRAMES`，结果有效；
   - `BALL HD`：候选尚在连续确认中，或短暂漏检时保持上次有效位置；
   - `BALL LS`：未检测到、遮挡或候选跳变过大；
   - `x=...mm`：由像素坐标换算的横向位置；向图像右侧为正、左侧为负。

4. 依次进行静止中心、左移、右移、遮挡/移走小球的测试。

| 操作 | 合格现象 |
| --- | --- |
| 小球静止在中心 | `BALL OK`，`x_mm` 接近 0，绿圈稳定跟随。 |
| 缓慢向右移动 | `x_mm` 连续增大且为正。 |
| 缓慢向左移动 | `x_mm` 连续减小且为负。 |
| 遮住或移走小球 | 约 `LOST_FRAMES` 帧后显示 `BALL LS`。 |
| 背景有圆形干扰物 | 不应跳到远处候选；否则先收紧 `BALL_ROI`。 |

这一步只验证视觉输出，不构成小球闭环控制或实车性能验证。

## 标定 `ball_detect_config.py`

当前临时映射来自 `data/` 中三组静态日志的有效帧中位数线性拟合：

```text
x_mm = (px - 160.365) * 0.540652
```

三个标定点的内部拟合残差 RMSE 为 `0.088 mm`；它不包含人工放置、尺量、透视和整数像素量化误差，不能当作系统精度。`CALIBRATION_READY` 必须继续保持 `False`。

> 机械已固定，现可执行本节；在完成并记录标定结果前，仍不得据 `x_mm` 判断真实位置或驱动舵机。

白色内壁安装后，先保持 `CALIBRATION_READY = False`，并按下列方法重新采样：

1. 在 `-50/-25/0/+25/+50 mm` 五个机械位置分别静止放球；每个位置重新放置三次，每次运行预览约 10 秒。
2. 将生成的日志复制到 `project/k230/data/`，按 `white_<位置>mm_run<序号>.txt` 重命名，例如 `white_-25mm_run2.txt`。
3. 日志会记录 `center_px`、`center_py`、`radius_px` 和当前 `roi`。根据所有 `BALL OK` 样本的圆心范围，加上最大半径和约 `10 px` 余量，设置 `BALL_ROI`。
4. 每段日志只取稳定阶段的有效帧中位数；每个位置取三次中位数的中位数，再用五个位置拟合位置映射。
5. 小球放在机械中心，记录拟合零点，写入 `IMAGE_CENTER_X`；根据拟合斜率写入 `MM_PER_PIXEL`。
6. 五点拟合完成后，使用全部 15 段数据计算最大绝对误差和 RMSE；只有误差满足要求，才讨论将 `CALIBRATION_READY` 改为 `True`。

基础关系为：

   ```text
   MM_PER_PIXEL = d_mm / d_px
   ```

根据实际球在画面中的大小调整 `CIRCLE_R_MIN`、`CIRCLE_R_MAX`，再根据误检情况调整：

   - `CIRCLE_ACCUMULATOR` 增大：更严格、误检更少；
   - `MAX_CENTER_JUMP_PX` 减小：更严格拒绝跳变；
   - `STABLE_FRAMES` 增大：更稳，但确认延迟更大；
   - `LOST_FRAMES` 减小：丢球判定更快。

若小球颜色稳定且背景颜色差异明显，可将 `DETECT_MODE` 改为 `blob` 并标定 `BALL_LAB_THRESHOLD`。色块法通常更快，但阈值对光照更敏感。

## 第二阶段：UART 联调

视觉预览稳定后，再使用 `k230_ball_detect_uart.py`。K230 输出帧为：

```text
$K230,BALL,<x_mm>,<valid>,<seq>#\r\n
```

字段约定：

| 字段 | 含义 |
| --- | --- |
| `x_mm` | 小球相对摆杆中心的位置，单位 mm，右正左负。 |
| `valid` | `1` 为连续确认的有效检测；`0` 为丢球、遮挡或未确认。 |
| `seq` | 递增帧序号。 |

接线如下：

```text
K230 IO40 / UART1_TX  -> MSPM0 PB18 / UART2_RX
K230 IO41 / UART1_RX  <- MSPM0 PB17 / UART2_TX
K230 GND              <-> MSPM0 GND
```

先用 `k230_uart_ball_test.py` 验证传输，再运行视觉发送脚本。MSPM0 首次解析 BALL 帧会返回 `$MSPM0,ACK#`；这仅证明协议帧可被解析，是否真实检测到小球仍以 K230 预览中的 `BALL OK` 与位置变化为准。

## 后续边界

完成视觉和 UART 后，才进入舵机中位/限位验证与静止控球。未经舵机机械参数、方向和限位的实测确认，不应把 `x_mm` 直接接入舵机输出。
