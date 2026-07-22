# K230 同心圆靶识别

本项目用于实时识别红色同心圆靶的靶心，输出靶心像素坐标。中心红点是首选特征；当其因距离、反光或遮挡而不可见时，使用同心圆的公共圆心兜底。

> 当前状态：完成 K230 CanMV 最小 Demo 骨架（IDE/LCD 显示）；尚未在目标板和实际靶距上验证。未接入 MSPM0 UART，不会影响现有小车工程。

## 目录

```text
K230_Target_Detection/
├── main.py             # CanMV 入口：取流、圆检测、同心筛选、OSD 显示
├── target_config.py    # 与相机视场和靶距有关的可调参数
├── 靶子.jpg            # 当前样张
└── README.md
```

## 检测定义

- 优先使用红色阈值提取中心红点，输出红点质心。
- 未检出可信红点时，对相机画面运行 Hough 圆检测；将圆心距离不超过 `CENTER_TOLERANCE_PX` 的圆归为同一靶子，取圆数最多的一组的平均圆心。
- 红点与同心圆均不可用时，本帧识别无效，不复用上一帧坐标。
- 圆直径的实际尺寸为 4 cm 的整数倍；初版不输出厘米值，圆环数量和像素半径只用于调试兜底结果。

样张中可见 5 条圆环线和中心红点。中心红点不是圆环，不计入 `ring_count`。

## 运行

1. 将 `main.py`、`target_config.py` 上传至 K230 CanMV 文件系统，或用 CanMV IDE 打开 `main.py` 运行。
2. 首次运行只观察 IDE/LCD 预览与串口日志。若提示不支持 `find_blobs()` 或 `find_circles()`，请保留完整日志；不同 CanMV 固件版本的图像 API 可能不同。
3. 将靶子放到实际使用距离，依次调整 `MIN_RADIUS`、`MAX_RADIUS`、`CIRCLE_THRESHOLD` 与 `CENTER_TOLERANCE_PX`，直到圆心稳定、圆环数正确。

## 当前输出

控制台示例：

```text
target: source=red_dot, center=(640, 361), rings=0, radii=[]
target: source=circles, center=(640, 361), rings=5, radii=[78, 157, 235, 314, 392]
```

圆心和半径均为相机像素坐标。初版仅在 K230 的 LCD/IDE 叠加显示并输出控制台日志。后续如需发送给 MSPM0，将保留 `valid, center_x, center_y` 三个字段并复用仓库既有的 UART 帧；本初版不擅自改变该协议。

## 验收顺序

1. K230 可打开相机并显示预览。
2. 靶子正对相机时，优先出现 `source=red_dot`，中心叠加标记稳定。
3. 将靶子拉远至红点不可见时，出现 `source=circles`，且中心仍正确。
4. 平移靶子时，中心坐标方向、数值变化正确。
5. 倾斜、强反光、部分遮挡时记录误检情况，再决定是否增加滤波或透视校正。
