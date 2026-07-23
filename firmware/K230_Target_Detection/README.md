# K230 同心圆靶识别

本项目用于实时识别同心圆靶的靶心，输出靶心像素坐标。黑框定位和靶心提取分两阶段：先通过四边形检测锁定靶面，再使用同心圆/红点精确定位靶心。

> 当前状态：黑框定位已完成，双通道 cv_lite 版稳定运行 ~56-58 FPS。**已知限制：全画面矩形检测无法区分靶纸和同形背景（窗框等），靶纸身份校验（黑框/同心圆/红点）尚未接入。** 同心圆/红点检测（`main.py`）尚未升级。未接入 MSPM0 UART。

## 固件要求

- `cvlite_demo.py`：需要 **CanMV v1.8+**（ATK DNK230D 专用，含 `cv_lite` 模块）
- `black_frame_demo.py`：CanMV v1.2+ 即可
- `main.py` / `test_cvlite.py`：CanMV v1.2+
- 固件烧录工具和镜像见厂商资料盘 `6，软件资料/`

## 目录

```text
K230_Target_Detection/
├── cvlite_demo.py              # cv_lite 双通道黑框定位（主力）
├── black_frame_demo.py         # 原生 find_rects 黑框定位（对照）
├── main.py                     # 同心圆/红点靶心检测（待升级）
├── target_config.py            # 检测阈值参数
├── test_cvlite.py              # cv_lite 模块可用性测试
├── test_dual_channel.py        # bind_layer API 兼容性诊断
├── 靶子.jpg                    # 靶纸样张
├── log.md                      # 运行日志（不纳入 git）
├── docs/                       # 设计文档与评审
│   ├── cvlite_dual_channel.md
│   ├── cvlite_dual_channel_review.md
│   └── cvlite_demo_优化建议报告.md
├── research/                   # 调研文档
│   ├── 2025-电赛E题-视觉方案调研.md
│   └── 靶心识别方案调研.md
├── log/                        # 开发日志
└── pic/                        # 实拍帧（不纳入 git）
```

## 黑框定位管线

### cv_lite 版 (`cvlite_demo.py`) — 分支 `feat/k230-cvlite`

双通道架构，实测 ~56-58 FPS：

```
sensor 1280×960
  ├─ ch0: 640×480 RGB565 → snapshot → 绘制检测结果 → show_image（彩色预览）
  └─ ch1: 320×240 GRAYSCALE → cv_lite.grayscale_find_rectangles_with_corners()
```

检测坐标 ×2 映射到 640×480 显示。追踪逻辑：连续确认（detect_streak≥2）+ 时序保持（HOLD_FRAMES=5）+ 距离门控（MAX_JUMP=60）。候选按形状得分（面积 × 长宽比匹配度，TARGET=0.70）排序。

### 原生版 (`black_frame_demo.py`) — 分支 `feat/k230-target-detection`

```
sensor QVGA(320×240) → to_grayscale → binary(0,80) → invert → find_rects()
```

同样具备时序保持 + 连续确认 + 距离门控 + 形状得分。

## 靶纸规格

209.5 × 296 mm（竖），含 18.5 mm 宽黑色外框（长宽比 ≈ 0.708）。白纸区域 172.5 × 259 mm（长宽比 ≈ 0.666）。内部有黑色同心圆（直径 40mm 整数倍）和红色中心点。

## 当前输出

```text
source=detect, roi=(131,76,55,74), center=(156,112), radius=40, corners=[...]
fps=58.4
```

- `source`: detect=新检出, hold=时序保持, reject=距离门控拦截, none=未检出
- `center`: 对角线交点（透视中心），圆形 visual 指示
- `radius`: 四角点到圆心平均距离
- 坐标均为 320×240 检测分辨率下的像素值

## 待办

- [ ] **靶纸身份校验**（P0）：黑框 ROI → 四边线 → 透视矫正 → 同心圆/红点验证，解决窗框误检
- [ ] `main.py` 同心圆/红点检测用 cv_lite 升级
- [ ] UART 输出对接 MSPM0（复用 `k210_link` 二进制协议 `0xAA+x+y+area+xor`）
- [ ] 评估 `rgb888_perspective_transform` 抗倾斜效果
- [ ] v1.8 `bind_layer` 不可用——待固件更新后切换硬件直显 + 独立 OSD 层
