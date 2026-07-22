# K230 同心圆靶识别

本项目用于实时识别同心圆靶的靶心，输出靶心像素坐标。黑框定位和靶心提取分两阶段：先通过四边形检测锁定靶面，再使用同心圆/红点精确定位靶心。

> 当前状态：黑框定位已完成，`black_frame_demo.py`（原生 find_rects 管线）和 `cvlite_demo.py`（cv_lite 加速版）均可稳定追踪靶纸。需刷 **CanMV v1.8+** 固件以使用 cv_lite。同心圆/红点检测（`main.py`）尚未升级。未接入 MSPM0 UART。

## 固件要求

- 原生版 `black_frame_demo.py`：CanMV v1.2+ 即可
- cv_lite 版 `cvlite_demo.py`：需要 **CanMV v1.8+**（ATK DNK230D 专用，含 `cv_lite` 模块）
- 固件烧录工具和镜像见厂商资料盘 `6，软件资料/`

## 目录

```text
K230_Target_Detection/
├── black_frame_demo.py # 原生 find_rects 黑框定位（稳定版）
├── cvlite_demo.py      # cv_lite 硬件加速黑框定位（实验版）
├── main.py             # 同心圆/红点靶心检测（待升级）
├── target_config.py    # 检测阈值参数
├── test_cvlite.py      # cv_lite 模块可用性测试脚本
├── 靶子.jpg            # 靶纸样张
├── log.md              # 运行日志
├── pic/                # 实拍帧（不纳入 git）
├── research/           # 调研文档 + 开发日志
└── README.md
```

## 黑框定位管线

### 原生版 (`black_frame_demo.py`) — 分支 `feat/k230-target-detection`

```
sensor QVGA(320×240) → to_grayscale → binary(0,80) → invert → find_rects()
```

配合时序保持（HOLD_FRAMES=5）+ 连续确认（detect_streak≥2）+ 距离门控（MAX_JUMP=60），圆形视觉指示。

### cv_lite 版 (`cvlite_demo.py`) — 分支 `feat/k230-cvlite`

```
sensor GRAYSCALE QVGA → cv_lite.grayscale_find_rectangles_with_corners()
```

单函数调用替代 3 步预处理，检测连续性和坐标稳定性显著优于原生版。

## 靶纸规格

209.5 × 296 mm（竖），含 18.5 mm 宽黑色外框。白纸区域 172.5 × 259 mm，长宽比 ≈ 0.666。内部有黑色同心圆（直径 40mm 整数倍）和红色中心点。

## 当前输出

控制台示例（cv_lite 版）：

```text
source=detect, roi=(131,76,55,74), center=(156,112), radius=40, corners=[...]
source=hold, roi=(130,76,55,74), center=(156,112), radius=40, corners=[...]
```

- `source`: detect=新检出, hold=时序保持, reject=距离门控拦截, none=未检出
- `center`: 对角线交点（透视中心），圆形视觉指示
- `radius`: 四角点到圆心平均距离
- 坐标均为 320×240 采集分辨率下的像素值

## 待办

- [ ] cv_lite 版补上距离门控+追踪逻辑（当前仅基础检测，`7cfbcd1`）
- [ ] `main.py` 同心圆/红点检测用 cv_lite 升级
- [ ] UART 输出对接 MSPM0（复用 `k210_link` 二进制协议）
- [ ] 添加 `clock.fps()` 实测帧率
- [ ] 评估 `rgb888_perspective_transform` 抗倾斜效果
