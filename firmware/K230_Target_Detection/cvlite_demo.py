"""K230 CanMV 靶面定位：cv_lite 硬件加速 + 双通道显示。

ch0: 640×480 YUV → bind_layer 硬件直显 LCD（零 CPU）
ch1: 320×240 GRAYSCALE → cv_lite 检测
OSD: 640×480 ARGB → LAYER_OSD3（坐标 ×2，结果变化时刷新）
"""
import os
import time
import math

import cv_lite
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, Sensor

LCD_WIDTH = 640
LCD_HEIGHT = 480
DETECT_WIDTH = 320
DETECT_HEIGHT = 240
SCALE = 2  # 检测→显示坐标映射

# ── cv_lite 参数 ──
CANNY_LO = 50
CANNY_HI = 150
APPROX_EPSILON = 0.04
AREA_MIN_RATIO = 0.02
MAX_ANGLE_COS = 0.3
GAUSSIAN_BLUR = 5

# ── 筛选参数（320×240） ──
MIN_AREA = 800
MAX_AREA = 72_000
MIN_ASPECT = 0.60
MAX_ASPECT = 0.80
TARGET_ASPECT = 0.70

LOG_PERIOD_FRAMES = 15
HOLD_FRAMES = 5
MAX_JUMP = 60


def find_target(img):
    """cv_lite 矩形检测，返回 (corners, (rx,ry,rw,rh)) 或 None。"""
    img_np = img.to_numpy_ref()
    rects = cv_lite.grayscale_find_rectangles_with_corners(
        [DETECT_HEIGHT, DETECT_WIDTH], img_np,
        CANNY_LO, CANNY_HI,
        APPROX_EPSILON,
        AREA_MIN_RATIO,
        MAX_ANGLE_COS,
        GAUSSIAN_BLUR,
    )

    best = None
    best_score = 0
    for r in rects:
        if len(r) != 12:
            continue
        rx, ry, rw, rh = r[0], r[1], r[2], r[3]
        c1x, c1y, c2x, c2y, c3x, c3y, c4x, c4y = r[4:12]
        corners = [(c1x, c1y), (c2x, c2y), (c3x, c3y), (c4x, c4y)]

        if any(p[0] < 0 or p[1] < 0 for p in corners):
            continue
        area = rw * rh
        if area < MIN_AREA or area > MAX_AREA:
            continue
        aspect = min(rw, rh) / max(rw, rh) if rw and rh else 0
        if not MIN_ASPECT <= aspect <= MAX_ASPECT:
            continue
        score = area * (1.0 - abs(aspect - TARGET_ASPECT) / TARGET_ASPECT)
        if score > best_score:
            best_score = score
            best = (corners, (rx, ry, rw, rh))
    return best


sensor = None
try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()

    # ch0: 640×480 RGB565 → show_image 显示
    sensor.set_framesize(w=LCD_WIDTH, h=LCD_HEIGHT, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    # ch1: 320×240 GRAYSCALE → cv_lite 检测
    sensor.set_framesize(w=DETECT_WIDTH, h=DETECT_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("cvlite dual-channel started: display %dx%d, detect %dx%d" %
          (LCD_WIDTH, LCD_HEIGHT, DETECT_WIDTH, DETECT_HEIGHT))

    clock = time.clock()
    frame_count = 0
    last_corners = None
    last_rect = None
    hold_count = 0
    detect_streak = 0
    track_cx = None
    track_cy = None

    while True:
        os.exitpoint()
        clock.tick()
        display_img = sensor.snapshot(chn=CAM_CHN_ID_0)
        img = sensor.snapshot(chn=CAM_CHN_ID_1)
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
                corners = None
                rect = None
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
            cx = sum(p[0] for p in corners) // 4
            cy = sum(p[1] for p in corners) // 4

            if track_cx is not None:
                dx = cx - track_cx
                dy = cy - track_cy
                if dx * dx + dy * dy > MAX_JUMP * MAX_JUMP:
                    cx, cy = track_cx, track_cy
                    corners = last_corners
                    rect = last_rect
                    source = "reject"
                else:
                    track_cx, track_cy = cx, cy
            else:
                track_cx, track_cy = cx, cy

            # 在 reject 之后解包 rect，保证日志 ROI 与图形一致
            rx, ry, rw, rh = rect
        else:
            track_cx = None
            track_cy = None

        # 在 ch0 显示帧上叠加检测结果（每帧画，因为 snapshot 每次都是新图）
        if corners is not None:
            radius = int(sum(math.sqrt((p[0] - cx) ** 2 + (p[1] - cy) ** 2)
                            for p in corners) / 4)
            # 坐标 ×2 映射到 640×480
            ox, oy = cx * SCALE, cy * SCALE
            oradius = radius * SCALE
            display_img.draw_circle(ox, oy, oradius, color=(255, 255, 0), thickness=2)
            for p in corners:
                display_img.draw_circle(p[0] * SCALE, p[1] * SCALE, 5,
                                        color=(0, 255, 0), thickness=2)
            display_img.draw_cross(ox, oy, color=(255, 0, 0), size=20, thickness=2)

        Display.show_image(display_img, 0, 0)

        if frame_count % LOG_PERIOD_FRAMES == 0:
            if corners is not None:
                corners_str = ", ".join("(%d,%d)" % (p[0], p[1]) for p in corners)
                print("source=%s, roi=(%d,%d,%d,%d), center=(%d,%d), radius=%d, corners=[%s]" %
                      (source, rx, ry, rw, rh, cx, cy, radius, corners_str))
            else:
                print("source=%s" % source)

        if frame_count % LOG_PERIOD_FRAMES == 0:
            print("fps=%.1f" % clock.fps())

        frame_count += 1

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
