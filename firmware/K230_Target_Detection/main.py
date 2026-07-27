"""K230 CanMV 红色同心圆靶识别（初版）。"""
import gc
import os
import utime

import image
from media.display import Display
from media.media import MediaManager
from media.sensor import (CAM_CHN_ID_0, CAM_CHN_ID_1, PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                          Sensor)

from target_config import (CENTER_TOLERANCE_PX, CIRCLE_THRESHOLD, DISPLAY_MODE,
                           LOG_PERIOD_FRAMES, MAX_RADIUS, MIN_RADIUS, SENSOR_ID,
                           RED_DOT_MAX_ASPECT, RED_DOT_MAX_PIXELS,
                           RED_DOT_MIN_ASPECT, RED_DOT_MIN_PIXELS,
                           RED_DOT_THRESHOLD, X_STRIDE, Y_STRIDE)


def distance_sq(circle_a, circle_b):
    dx = circle_a.x() - circle_b.x()
    dy = circle_a.y() - circle_b.y()
    return dx * dx + dy * dy


def select_target(circles):
    """从候选圆中找出圆心最一致的一组，返回 (center_x, center_y, circles)。"""
    if not circles:
        return None

    max_distance_sq = CENTER_TOLERANCE_PX * CENTER_TOLERANCE_PX
    best_group = []
    for seed in circles:
        group = [circle for circle in circles
                 if distance_sq(seed, circle) <= max_distance_sq]
        if len(group) > len(best_group):
            best_group = group

    center_x = sum(circle.x() for circle in best_group) // len(best_group)
    center_y = sum(circle.y() for circle in best_group) // len(best_group)
    best_group.sort(key=lambda circle: circle.r())
    return center_x, center_y, best_group


def find_red_dot(frame):
    """检测中心红点；红色圆环通常尺寸更大或长宽比不接近 1，会被过滤。"""
    if not hasattr(frame, "find_blobs"):
        return None

    candidates = []
    for blob in frame.find_blobs([RED_DOT_THRESHOLD], pixels_threshold=RED_DOT_MIN_PIXELS,
                                 area_threshold=RED_DOT_MIN_PIXELS):
        if blob.pixels() > RED_DOT_MAX_PIXELS:
            continue
        aspect = blob.w() / blob.h() if blob.h() else 0
        if RED_DOT_MIN_ASPECT <= aspect <= RED_DOT_MAX_ASPECT:
            candidates.append(blob)

    if not candidates:
        return None
    dot = max(candidates, key=lambda blob: blob.pixels())
    return dot.cx(), dot.cy()


def create_camera():
    board = os.uname()[-1]
    sensor = Sensor(id=SENSOR_ID, fps=30) if SENSOR_ID is not None else Sensor(fps=30)
    sensor.reset()

    display_type = Display.LT9611 if DISPLAY_MODE in ("hdmi", "lt9611") else Display.ST7701
    Display.init(display_type, osd_num=1, to_ide=True)
    display_size = (Display.width(), Display.height())

    sensor.set_framesize(w=display_size[0], h=display_size[1], chn=CAM_CHN_ID_0)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420, chn=CAM_CHN_ID_0)
    sensor.set_framesize(w=display_size[0], h=display_size[1], chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

    osd = image.Image(display_size[0], display_size[1], image.ARGB8888)
    Display.bind_layer(**sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0),
                       dstlayer=Display.LAYER_VIDEO1)
    MediaManager.init()
    sensor.run()
    print("camera started:", board, display_size)
    return sensor, osd


def main():
    sensor = None
    try:
        sensor, osd = create_camera()
        frame_count = 0
        while True:
            frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            red_dot = find_red_dot(frame)
            circle_target = None
            if not red_dot:
                if not hasattr(frame, "find_circles"):
                    raise RuntimeError("当前固件不支持 find_blobs() 或 find_circles()；请保留 CanMV 版本和错误日志。")
                circles = frame.find_circles(threshold=CIRCLE_THRESHOLD,
                                             x_stride=X_STRIDE, y_stride=Y_STRIDE,
                                             r_min=MIN_RADIUS, r_max=MAX_RADIUS,
                                             r_step=2)
                circle_target = select_target(circles)
            osd.clear()
            if red_dot:
                center_x, center_y = red_dot
                source = "red_dot"
                target_circles = []
            elif circle_target:
                center_x, center_y, target_circles = circle_target
                source = "circles"
            else:
                center_x = center_y = None
                source = "invalid"
                target_circles = []

            if center_x is not None:
                for circle in target_circles:
                    osd.draw_circle(circle.x(), circle.y(), circle.r(), color=(0, 255, 0), thickness=2)
                osd.draw_cross(center_x, center_y, color=(255, 0, 0), size=16, thickness=2)
                frame_count += 1
                if frame_count % LOG_PERIOD_FRAMES == 0:
                    radii = [circle.r() for circle in target_circles]
                    print("target: source=%s, center=(%d, %d), rings=%d, radii=%s" %
                          (source, center_x, center_y, len(radii), radii))
            Display.show_image(osd, 0, 0, Display.LAYER_OSD3)
            gc.collect()
    finally:
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        utime.sleep_ms(50)


if __name__ == "__main__":
    main()
