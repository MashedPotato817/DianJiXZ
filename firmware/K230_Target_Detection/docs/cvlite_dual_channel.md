# cvlite_demo 双通道显示方案

## 目标

将 cvlite_demo 从单通道 show_image 贴图改为双通道硬件直显架构，和 main.py 统一。

## 当前 vs 目标

```
当前（单通道）：
  sensor GRAYSCALE 320×240 → cv_lite 检测 → show_image(居中贴到640×480 LCD)
  问题：灰度预览、黑边、show_image 每帧占 ~5ms

目标（双通道）：
  sensor 1280×960
    ├─ ch0: 640×480 YUV → bind_layer(VIDEO1) → LCD 彩色预览（零 CPU）
    └─ ch1: 320×240 GRAYSCALE → cv_lite 检测
  OSD: 640×480 ARGB → LAYER_OSD3（坐标 ×2，结果变化时刷新）
```

## 传感器配置

```python
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, PIXEL_FORMAT_YUV_SEMIPLANAR_420

sensor = Sensor(id=None, fps=30)
sensor.reset()

# ch0: 显示通道
sensor.set_framesize(w=640, h=480, chn=CAM_CHN_ID_0)
sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420, chn=CAM_CHN_ID_0)

# ch1: 检测通道
sensor.set_framesize(w=320, h=240, chn=CAM_CHN_ID_1)
sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

# 显示
Display.init(Display.ST7701, osd_num=1, to_ide=True)
Display.bind_layer(**sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0),
                   dstlayer=Display.LAYER_VIDEO1)

# OSD 画布
osd = image.Image(640, 480, image.ARGB8888)
```

## 主循环

```python
while True:
    img = sensor.snapshot(chn=CAM_CHN_ID_1)  # 320×240 GRAYSCALE
    result = find_target(img)

    if result_changed:
        osd.clear()
        if result:
            # 坐标 ×2 映射到 640×480
            draw_cross(osd, cx * 2, cy * 2)
            draw_circle(osd, cx * 2, cy * 2, radius * 2)
            draw_corners(osd, [(x * 2, y * 2) for x, y in corners])
        Display.show_image(osd, 0, 0, Display.LAYER_OSD3)
```

## 关键点

1. **ch0 直显不走 CPU**：YUV 帧通过 bind_layer 硬件送到 LCD，不需要 show_image
2. **ch1 独立分辨率**：320×240 硬件缩放，检测不受显示分辨率影响
3. **OSD 按需刷新**：只在检测结果变化时 clear + 重绘，不每帧刷新。显示帧率 60fps，OSD 帧率 ~10fps
4. **to_ide=True**：IDE 预览看到的是 OSD 叠加后的 640×480 画面
5. **坐标映射**：检测在 320×240，显示在 640×480，坐标统一 ×2

## 参考

`main.py` 的 `create_camera()` — 完全相同的双通道 + OSD 模式。
