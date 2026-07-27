"""K230 CanMV 靶面定位：预处理 + find_rects 检测黑框。

靶纸 209.5×296 mm（竖），含 18.5 mm 宽黑框，内部有同心圆。
管线：sensor QVGA → binary → invert → find_rects
采集 320×240（硬件缩放），显示居中在 640×480 LCD。
"""
import os
import time
import math

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
LCD_WIDTH = 640
LCD_HEIGHT = 480

# ── 预处理参数 ──
BINARY_LO = 0        # 二值化暗区下限
BINARY_HI = 80       # 二值化暗区上限（黑框落在此范围）

# ── find_rects 参数（320×240） ──
RECT_THRESHOLD = 2_000
X_GRADIENT = 10
Y_GRADIENT = 10

# ── 筛选参数（320×240，总像素 76,800） ──
# 靶纸长宽比 209.5/296 ≈ 0.708
MIN_AREA = 800
MAX_AREA = 72_000
MIN_ASPECT = 0.52
MAX_ASPECT = 0.92

LOG_PERIOD_FRAMES = 15
HOLD_FRAMES = 5
MAX_JUMP = 60  # 空间一致性门控：新检测距上次位置超过此值视为假阳性（320×240）


def find_target(img):
    """预处理 + find_rects，返回 (corners, rect_tuple) 或 None。"""
    # 1. 灰度化
    gray = img.to_grayscale(copy=True)

    # 2. 二值化：暗像素→白，其余→黑
    gray.binary([(BINARY_LO, BINARY_HI)])

    # 3. 反色：黑框变白色前景
    gray.invert()

    # 4. 矩形检测
    rects = gray.find_rects(threshold=RECT_THRESHOLD,
                            x_gradient=X_GRADIENT,
                            y_gradient=Y_GRADIENT)

    # 6. 筛选
    best = None
    best_area = 0
    for r in rects:
        corners = r.corners()
        if len(corners) != 4:
            continue
        if any(p[0] < 0 or p[1] < 0 for p in corners):
            continue
        rx, ry, rw, rh = r.rect()
        area = rw * rh
        if area < MIN_AREA or area > MAX_AREA:
            continue
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
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("find_rects demo started: capture %dx%d, display %dx%d" %
          (FRAME_WIDTH, FRAME_HEIGHT, LCD_WIDTH, LCD_HEIGHT))

    frame_count = 0
    last_corners = None
    last_rect = None
    hold_count = 0
    detect_streak = 0
    track_cx = None   # 已确认的追踪位置
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

            # 空间一致性门控：拒绝跳到远处背景物体的假阳性
            if track_cx is not None:
                dx = cx - track_cx
                dy = cy - track_cy
                if dx * dx + dy * dy > MAX_JUMP * MAX_JUMP:
                    # 跳变过大，视为假阳性，保持上一位置
                    cx, cy = track_cx, track_cy
                    source = "reject"
                else:
                    track_cx, track_cy = cx, cy
            else:
                track_cx, track_cy = cx, cy

            # 最小包围圆：圆心 = 对角线交点，半径 = 四角点到圆心的平均距离
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
        # 320×240 居中显示在 640×480 LCD 上
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
