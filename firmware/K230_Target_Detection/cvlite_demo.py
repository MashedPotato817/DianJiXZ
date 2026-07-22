"""K230 CanMV 靶面定位：cv_lite 硬件加速版。

管线：sensor GRAYSCALE → cv_lite.grayscale_find_rectangles_with_corners
采集 320×240，显示居中在 640×480 LCD。
"""
import os
import time
import math

import cv_lite
import ulab.numpy as np
from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
LCD_WIDTH = 640
LCD_HEIGHT = 480

# ── cv_lite 参数 ──
CANNY_LO = 50
CANNY_HI = 150
APPROX_EPSILON = 0.04       # 多边形拟合精度，越小越精确
AREA_MIN_RATIO = 0.02       # 矩形最小面积比（相对全图）
MAX_ANGLE_COS = 0.3         # 角点直角容差，越小越严格
GAUSSIAN_BLUR = 5           # 高斯模糊核，必须奇数

# ── 筛选参数（320×240） ──
MIN_ASPECT = 0.52
MAX_ASPECT = 0.92

LOG_PERIOD_FRAMES = 15
HOLD_FRAMES = 5
MAX_JUMP = 60


def find_target(img):
    """cv_lite 矩形检测，返回 (corners, (rx,ry,rw,rh)) 或 None。"""
    img_np = img.to_numpy_ref()
    rects = cv_lite.grayscale_find_rectangles_with_corners(
        [FRAME_HEIGHT, FRAME_WIDTH], img_np,
        CANNY_LO, CANNY_HI,
        APPROX_EPSILON,
        AREA_MIN_RATIO,
        MAX_ANGLE_COS,
        GAUSSIAN_BLUR,
    )

    best = None
    best_area = 0
    for r in rects:
        if len(r) != 12:
            continue
        rx, ry, rw, rh = r[0], r[1], r[2], r[3]
        c1x, c1y, c2x, c2y, c3x, c3y, c4x, c4y = r[4:12]
        corners = [(c1x, c1y), (c2x, c2y), (c3x, c3y), (c4x, c4y)]

        if any(p[0] < 0 or p[1] < 0 for p in corners):
            continue
        area = rw * rh
        aspect = min(rw, rh) / max(rw, rh) if rw and rh else 0
        if not MIN_ASPECT <= aspect <= MAX_ASPECT:
            continue
        if area > best_area:
            best_area = area
            best = (corners, (rx, ry, rw, rh))
    return best


sensor = None
try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(w=FRAME_WIDTH, h=FRAME_HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)  # cv_lite grayscale 函数要求灰度图
    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("cvlite demo started: capture %dx%d GRAYSCALE, display %dx%d" %
          (FRAME_WIDTH, FRAME_HEIGHT, LCD_WIDTH, LCD_HEIGHT))

    frame_count = 0
    last_corners = None
    last_rect = None
    hold_count = 0
    detect_streak = 0
    track_cx = None
    track_cy = None

    while True:
        os.exitpoint()
        img = sensor.snapshot()
        result = find_target(img)

        if result:
            corners, rect = result
            detect_streak += 1
            if detect_streak >= 2:
                last_corners = corners
                last_rect = rect
                hold_count = 0
            source = "detect"
        else:
            detect_streak = 0
            if last_corners is not None and hold_count < HOLD_FRAMES:
                corners = last_corners
                rect = last_rect
                hold_count += 1
                source = "hold"
            else:
                corners = None
                rect = None
                source = "none"

        if corners is not None:
            rx, ry, rw, rh = rect
            cx = sum(p[0] for p in corners) // 4
            cy = sum(p[1] for p in corners) // 4

            # 距离门控
            if track_cx is not None:
                dx = cx - track_cx
                dy = cy - track_cy
                if dx * dx + dy * dy > MAX_JUMP * MAX_JUMP:
                    cx, cy = track_cx, track_cy
                    source = "reject"
                else:
                    track_cx, track_cy = cx, cy
            else:
                track_cx, track_cy = cx, cy

            radius = int(sum(math.sqrt((p[0] - cx) ** 2 + (p[1] - cy) ** 2)
                            for p in corners) / 4)
            img.draw_circle(cx, cy, radius, color=(255, 255, 0), thickness=2)
            for p in corners:
                img.draw_circle(p[0], p[1], 3, color=(0, 255, 0), thickness=2)
            img.draw_cross(cx, cy, color=(255, 0, 0), size=20, thickness=2)

            if frame_count % LOG_PERIOD_FRAMES == 0:
                corners_str = ", ".join("(%d,%d)" % (p[0], p[1]) for p in corners)
                print("source=%s, roi=(%d,%d,%d,%d), center=(%d,%d), radius=%d, corners=[%s]" %
                      (source, rx, ry, rw, rh, cx, cy, radius, corners_str))
        else:
            track_cx = None
            track_cy = None
            if frame_count % LOG_PERIOD_FRAMES == 0:
                print("source=none")

        frame_count += 1
        Display.show_image(img,
                           x=round((LCD_WIDTH - FRAME_WIDTH) / 2),
                           y=round((LCD_HEIGHT - FRAME_HEIGHT) / 2))

except KeyboardInterrupt as error:
    print("user stop:", error)
except BaseException as error:
    print("Exception:", error)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
