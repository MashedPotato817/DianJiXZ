"""K230 CanMV 靶面定位：预处理 + find_rects 检测黑框。

靶纸 209.5×296 mm（竖），含 18.5 mm 宽黑框。
管线：灰度 → gaussian → binary → invert → erode → dilate → find_rects
"""
import os
import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ── 预处理参数 ──
BINARY_LO = 0        # 二值化暗区下限
BINARY_HI = 80       # 二值化暗区上限（黑框 + 同心圆落在此范围）
ERODE_K = 1          # 腐蚀核大小，去同心圆细线
DILATE_K = 1         # 膨胀核大小，连接黑框断边
GAUSSIAN_K = 1       # 高斯滤波核

# ── find_rects 参数 ──
RECT_THRESHOLD = 6_000    # 矩形质量门槛，低于默认 10000
X_GRADIENT = 10           # 水平梯度核
Y_GRADIENT = 10           # 垂直梯度核

# ── 筛选参数（基于靶纸尺寸） ──
# 靶纸长宽比 209.5/296 ≈ 0.708，白纸内部 0.666
MIN_AREA = 3_000
MAX_AREA = 250_000
MIN_ASPECT = 0.52
MAX_ASPECT = 0.92

LOG_PERIOD_FRAMES = 15


def find_target(img):
    """预处理 + find_rects，返回 (corners, rect_tuple) 或 None。"""
    # 1. 灰度化
    gray = img.to_grayscale(copy=True)

    # 2. 高斯去噪
    gray.gaussian(GAUSSIAN_K)

    # 3. 二值化：暗像素→白，其余→黑
    gray.binary([(BINARY_LO, BINARY_HI)])

    # 4. 反色：黑框变白色前景（find_rects 检测白色四边形）
    gray.invert()

    # 5. 腐蚀：去除同心圆细线，保留粗黑框
    gray.erode(ERODE_K)

    # 6. 膨胀：连接黑框断边
    gray.dilate(DILATE_K)

    # 7. 矩形检测
    rects = gray.find_rects(threshold=RECT_THRESHOLD,
                            x_gradient=X_GRADIENT,
                            y_gradient=Y_GRADIENT)

    # 8. 筛选：面积 + 长宽比
    best = None
    best_area = 0
    for r in rects:
        corners = r.corners()
        if len(corners) != 4:
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
    Display.init(Display.ST7701, width=FRAME_WIDTH, height=FRAME_HEIGHT, to_ide=True)
    MediaManager.init()
    sensor.run()
    print("find_rects demo started: %dx%d" % (FRAME_WIDTH, FRAME_HEIGHT))

    frame_count = 0
    while True:
        os.exitpoint()
        img = sensor.snapshot()
        result = find_target(img)

        if result:
            corners, (rx, ry, rw, rh) = result
            # 四角点求中心（比 bounding box 中心更精确，抗倾斜）
            cx = sum(p[0] for p in corners) // 4
            cy = sum(p[1] for p in corners) // 4

            # 绘制
            img.draw_rectangle(rx, ry, rw, rh, color=(255, 255, 0), thickness=2)
            for p in corners:
                img.draw_circle(p[0], p[1], 5, color=(0, 255, 0), thickness=2)
            img.draw_cross(cx, cy, color=(255, 0, 0), size=20, thickness=2)

            if frame_count % LOG_PERIOD_FRAMES == 0:
                corners_str = ", ".join("(%d,%d)" % (p[0], p[1]) for p in corners)
                print("selected=yes, roi=(%d,%d,%d,%d), center=(%d,%d), corners=[%s]" %
                      (rx, ry, rw, rh, cx, cy, corners_str))
        elif frame_count % LOG_PERIOD_FRAMES == 0:
            print("selected=no")

        frame_count += 1
        Display.show_image(img, 0, 0)

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
