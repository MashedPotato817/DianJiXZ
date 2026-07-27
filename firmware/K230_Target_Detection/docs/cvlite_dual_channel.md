# cvlite_demo 双通道显示方案

## 当前架构

v1.8 固件 `bind_layer` API 不兼容（三种调用形式全部失败），采用软件显示方案：

```
sensor 1280×960
  ├─ ch0: 640×480 RGB565 → snapshot → 绘制检测结果 → show_image（软件显示）
  └─ ch1: 320×240 GRAYSCALE → cv_lite 检测
```

坐标 ×2 映射，每帧绘制，实测 ~56-58 FPS。

## 硬件直显（待未来固件支持）

```
sensor 1280×960
  ├─ ch0: 640×480 YUV → bind_layer(VIDEO1) → LCD 彩色预览（零 CPU）
  └─ ch1: 320×240 GRAYSCALE → cv_lite 检测
  OSD: 640×480 ARGB → LAYER_OSD3（坐标 ×2，结果变化时刷新）
```

注：`Display.bind_layer(**info, dstlayer=...)` 在 v1.8 上不兼容（`extra keyword arguments given`），位置参数形式同样失败。需等待固件更新。

## 传感器配置

```python
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, PIXEL_FORMAT_YUV_SEMIPLANAR_420

sensor = Sensor(id=None, fps=30)
sensor.reset()

# ch0: 显示通道
sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

# ch1: 检测通道
sensor.set_framesize(w=320, h=240, chn=CAM_CHN_ID_1)
sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

# 显示
Display.init(Display.ST7701, width=640, height=480, to_ide=True)
MediaManager.init()
sensor.run()
```

## 主循环

```python
while True:
    display_img = sensor.snapshot(chn=CAM_CHN_ID_0)  # 640×480 RGB565
    img = sensor.snapshot(chn=CAM_CHN_ID_1)           # 320×240 GRAYSCALE
    result = find_target(img)

    if result:
        # 坐标 ×2 映射，画在 display_img 上
        draw_circle(display_img, cx * 2, cy * 2, radius * 2)
        draw_cross(display_img, cx * 2, cy * 2)
    Display.show_image(display_img, 0, 0)
```

## 关键点

1. **双通道独立分辨率**：ch0 640×480 显示，ch1 320×240 检测，硬件缩放
2. **每帧绘制**：snapshot 每次返回新图，检测结果必须每帧重绘
3. **坐标映射**：检测 320×240 → 显示 640×480，统一 ×2
4. **实测 ~56-58 FPS**（to_ide=True，含 IDE 传输开销）
