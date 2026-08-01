# K230 小球视觉模块

> 最后更新：2026-08-01 15:22

本目录是 H 题平衡滚球系统的 K230 视觉端。它只负责从相机画面中检测小球、计算横向位置并输出状态；不直接控制 MSPM0 底盘或舵机。

当前默认方案是 `cv_lite` 灰度霍夫圆检测，而不是通用 AI 目标检测模型：小球是单一圆形目标，位置连续，使用传统视觉无需训练、标注和部署 `.kmodel`，更适合当前的首轮实测。

## 当前状态与限制

- K230 已完成预览、UART1 双向链路和临时低幅度闭环联调；K230 日志中发送配置为 50 ms，实测约 16 Hz。该结果不等同于位置精度、闭环稳定性或行驶控球达标。
- **相机、摆杆和小球运动平面已机械固定**。静态三点初值已按实体刻度尺参考线读数再次校正；`CALIBRATION_READY` 仍为 `False`。临时 `x_mm` 可用于受限联调，不能用于正式控制/精度结论。
- 当前重点是端到端遥测、滤波 PD 与静摩擦脱困；五点重复标定和行驶控球仍待完成。

## 当前文件

| 文件 | 用途 |
| --- | --- |
| `k230_ball_detect_preview.py` | 纯视觉预览。使用双通道相机，叠加检测结果到 CanMV IDE/LCD；不使用 UART。 |
| `ball_detect_config.py` | 圆检测、时序筛选、像素到毫米标定和 UART 参数。 |
| `k230_ball_detect_uart.py` | 发送 BALL 帧给 MSPM0；预览显示当前视觉帧，TXT 另记录最近发送帧，二者可能不同。 |
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
   - 黄框：当前有效检测区域。初始为 `y=60…109 px`，横向保持全宽；按板载 KEY2 向上移动，按 KEY1 向下移动，每次 `5 px`；
   - KEY1 为 IO35、按下低有效；KEY2 为 IO0、按下高有效。按键调整仅在本次运行中生效，重新运行脚本后恢复 `BALL_ROI` 初值；
   - `CIRCLE_USE_ROI_CROP=True` 时，霍夫圆只处理黄框内的副本；若 K230 实测 FPS 下降或出现内存异常，改为 `False` 回退到全图模式后再排查；
   - 绿圈和红十字：当前小球候选；
   - `BALL OK`：连续检测达到 `STABLE_FRAMES`，结果有效；
   - `BALL HD`：候选尚在连续确认中，或短暂漏检时保持上次有效位置；
   - `BALL LS`：未检测到、遮挡或候选跳变过大；
   - 右上角 `FPS:...` 始终显示，与 `OK/HD/LS` 状态无关；
   - 预览中的 `OK/HD/LS` 表示当前视觉帧状态；UART TXT 摘要同时给出 `VIS`（当前视觉）与 `TX`（最近发送帧），两者可能相差一个发送周期；
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

当前临时映射先由 `data/` 中三组静态日志确定初值，再按实体刻度尺直接固定三个像素锚点：

```text
-50 mm = 64 px
  0 mm = 162 px
+50 mm = 256 px
```

左右跨度分别为 `98 px` 和 `94 px`，因此采用两段简单线性换算：左侧 `0.510204 mm/px`，右侧 `0.531915 mm/px`。预览参考线、画面 `x_mm` 和 UART 数据共用该映射。刻度读数仍未包含人工放置、透视和检测误差，不能当作系统精度；`CALIBRATION_READY` 必须继续保持 `False`。

> 机械已固定，现可执行本节；在完成并记录标定结果前，仍不得据 `x_mm` 判断真实位置或作出正式控制/精度结论。若明确授权低幅度联调，可临时打开 `UART_ALLOW_UNCALIBRATED`；该模式必须保留舵机小脉宽限幅、失帧回中位，并在联调后关闭。

白色内壁安装后，先保持 `CALIBRATION_READY = False`，并按下列方法重新采样：

1. 在 `-50/-25/0/+25/+50 mm` 五个机械位置分别静止放球；每个位置重新放置三次，每次运行预览约 10 秒。
2. 将生成的日志复制到 `project/k230/data/`，按 `white_<位置>mm_run<序号>.txt` 重命名，例如 `white_-25mm_run2.txt`。
3. 日志会记录 `center_px`、`center_py`、`radius_px` 和当前 `roi`。根据所有 `BALL OK` 样本的圆心范围，加上最大半径和约 `10 px` 余量，设置 `BALL_ROI`。
4. 每段日志只取稳定阶段的有效帧中位数；每个位置取三次中位数的中位数，再用五个位置拟合位置映射。
5. 将 `-50/0/+50 mm` 三个位置的稳定圆心分别写入 `MINUS_50_PIXEL_X`、`IMAGE_CENTER_X` 和 `PLUS_50_PIXEL_X`；配置自动计算左右比例。
6. 五点拟合完成后，使用全部 15 段数据计算最大绝对误差和 RMSE；只有误差满足要求，才讨论将 `CALIBRATION_READY` 改为 `True`。

基础关系为：

   ```text
   MM_PER_PIXEL_LEFT  = 50 / (IMAGE_CENTER_X - MINUS_50_PIXEL_X)
   MM_PER_PIXEL_RIGHT = 50 / (PLUS_50_PIXEL_X - IMAGE_CENTER_X)
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
$K230,BALL,<x10>,<valid>,<seq>,<edge>*<crc8>#\r\n
```

字段约定：

| 字段 | 含义 |
| --- | --- |
| `x10` | 小球相对摆杆中心的位置乘 10，右正左负；`247` 表示 `+24.7 mm`。 |
| `valid` | `1` 为连续确认的有效检测；`0` 为丢球、遮挡或未确认。 |
| `seq` | 递增帧序号。 |
| `edge` | `-1/+1` 表示最后可靠球心在图像左/右边缘后连续丢失；`0` 表示正常或普通漏检。仅边缘丢失触发 M0 的限时向中心恢复。 |
| `crc8` | CRC-8/ATM 两位十六进制校验，覆盖 `$` 后到 `*` 前的 ASCII 负载。 |

MSPM0 只接受固定字段、CRC正确且位置在 `±150.0 mm` 内的帧；无校验旧帧不会进入闭环。这样即使UART丢失小数点、逗号或字段字节，也只会增加诊断计数，不会改变舵机命令。

接线如下：

```text
K230 IO40 / UART1_TX  -> MSPM0 PB7 / UART1_RX
K230 IO41 / UART1_RX  <- MSPM0 PB6 / UART1_TX
K230 GND              <-> MSPM0 GND
```

先用 `k230_uart_ball_test.py` 验证传输，再运行视觉发送脚本。MSPM0 首次解析 BALL 帧会返回 `$MSPM0,ACK#`；这仅证明协议帧可被解析，是否真实检测到小球仍以 K230 预览中的 `BALL OK` 与位置变化为准。

终端与 TXT 日志的默认输出、调试开关及 `VIS/TX` 字段解释见 [ball_detect_uart.md](ball_detect_uart.md)。

## K230 与 M0 日志配对

K230 板载时钟未校准时会显示2018年的日期，不能用它与电脑端M0日志自动配对。完整会话编号 = 前缀 + "_" + 六位序号，序号由脚本启动时自动递增（计数器存于 `LOG_FOLDER_PATH/.session_counter`），无需手工维护，每次上电不会重复使用旧编号。前缀在 `ball_detect_config.py` 设置：

```python
LOG_SESSION_PREFIX = "20260731"
```

运行后K230自动写入（以序号递增到的值为例）：

```text
/data/ball_detect_preview/20260731_000011_K230.txt
```

电脑串口助手将同一次M0遥测保存为：

```text
20260731_000011_M0.txt
```

K230启动终端和TXT都会输出 `LOG SESSION: 20260731_000011`，采集前以此复核——用它命名电脑端M0日志即可配对。若计数器文件被误删，脚本会按目录内已有日志的最大序号继续递增，仍不会覆盖旧日志。

## 后续边界

舵机中位、方向和限位已经完成首轮联调；正式控球前仍需完成五点标定、端到端遥测与静摩擦脱困验证。未经这些验证，不应把临时 `x_mm` 当作题目精度结果。

> 最后更新：2026-08-01 15:22
