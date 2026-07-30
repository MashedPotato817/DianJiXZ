"""K230 小球运动检测预览，不使用 UART。

ch1: 320x240 GRAYSCALE，供 cv_lite 霍夫圆检测。
ch0: 640x480 RGB565，供 CanMV IDE/LCD 预览与结果叠加。
"""

import gc
import os
import time

import cv_lite
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, Sensor

from ball_detect_config import (BALL_ROI, CIRCLE_ACCUMULATOR,
                                CIRCLE_CANNY_HIGH, CIRCLE_DP,
                                CIRCLE_MIN_DISTANCE, CIRCLE_R_MAX,
                                CIRCLE_R_MIN, FRAME_HEIGHT, FRAME_WIDTH,
                                IMAGE_CENTER_X, LOG_PERIOD_FRAMES,
                                LOST_FRAMES, MAX_CENTER_JUMP_PX,
                                MAX_POSITION_MM, MM_PER_PIXEL,
                                STABLE_FRAMES, ENABLE_LOG_FILE,
                                LOG_FOLDER_PATH)

now = time.localtime()
LOG_FILE_PATH = LOG_FOLDER_PATH + "%04d%02d%02d_%02d%02d%02d.txt" % (now[0], now[1], now[2], now[3], now[4], now[5])
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_SCALE = DISPLAY_WIDTH // FRAME_WIDTH
CENTER_LINE_X = int(IMAGE_CENTER_X * DISPLAY_SCALE)
# 按当前像素—毫米映射绘制静态标定参考线，用于白色管壁实测。
MINUS_50_LINE_X = int((IMAGE_CENTER_X - 50.0 / MM_PER_PIXEL) *
                       DISPLAY_SCALE)
PLUS_50_LINE_X = int((IMAGE_CENTER_X + 50.0 / MM_PER_PIXEL) *
                      DISPLAY_SCALE)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def log_timestamp():
    """使用 K230 系统时钟；未校时设备上的日期仅表示日志顺序。"""
    now = time.localtime()
    return "%04d-%02d-%02d %02d:%02d:%02d" % (now[0], now[1], now[2], now[3], now[4], now[5])


def log_message(text, log_file):
    line = "[%s] %s" % (log_timestamp(), text)
    print(line)
    if log_file is not None:
        log_file.write(line + "\n")
        log_file.flush()


def find_circle(frame, previous):
    raw_circles = cv_lite.grayscale_find_circles(
        [FRAME_HEIGHT, FRAME_WIDTH], frame.to_numpy_ref(), CIRCLE_DP,
        CIRCLE_MIN_DISTANCE, CIRCLE_CANNY_HIGH, CIRCLE_ACCUMULATOR,
        CIRCLE_R_MIN, CIRCLE_R_MAX)
    roi_x, roi_y, roi_w, roi_h = BALL_ROI
    circles = []
    for index in range(0, len(raw_circles) - 2, 3):
        x = raw_circles[index]
        y = raw_circles[index + 1]
        radius = raw_circles[index + 2]
        if roi_x <= x < roi_x + roi_w and roi_y <= y < roi_y + roi_h:
            circles.append((x, y, radius))
    if not circles:
        return None

    if previous is None:
        reference_x, reference_y = IMAGE_CENTER_X, FRAME_HEIGHT // 2
    else:
        reference_x, reference_y = previous
    return min(circles, key=lambda circle:
               (circle[0] - reference_x) ** 2 +
               (circle[1] - reference_y) ** 2)


def main():
    sensor = None
    log_file = None
    try:
        if ENABLE_LOG_FILE:
            try:
                # /data 下的日志目录首次使用时不存在，先创建再打开本次运行的文件。
                try:
                    os.mkdir(LOG_FOLDER_PATH)
                except OSError:
                    pass
                log_file = open(LOG_FILE_PATH, "a")
                log_message("--- BALL preview start ---", log_file)
            except OSError as error:
                print("log file disabled:", error)
        sensor = Sensor(width=1280, height=960, fps=30)
        sensor.reset()
        sensor.set_framesize(w=DISPLAY_WIDTH, h=DISPLAY_HEIGHT,
                             chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        sensor.set_framesize(w=FRAME_WIDTH, h=FRAME_HEIGHT,
                             chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)

        Display.init(Display.ST7701, width=DISPLAY_WIDTH,
                     height=DISPLAY_HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()
        log_message("K230 BALL preview started: cv_lite circle detection",
                    log_file)

        previous = None
        filtered_center_x = None
        valid_streak = 0
        lost_streak = LOST_FRAMES
        last_valid_x_mm = 0.0
        frame_count = 0
        clock = time.clock()

        while True:
            os.exitpoint()
            clock.tick()
            display_frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            detect_frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            ball = find_circle(detect_frame, previous)
            center_x = None
            center_y = None

            if ball is not None and previous is not None:
                dx = ball[0] - previous[0]
                dy = ball[1] - previous[1]
                if dx * dx + dy * dy > MAX_CENTER_JUMP_PX * MAX_CENTER_JUMP_PX:
                    ball = None

            if ball is None:
                lost_streak += 1
                if lost_streak >= LOST_FRAMES:
                    valid_streak = 0
                    previous = None
                    filtered_center_x = None
                    x_mm = 0.0
                    valid = 0
                else:
                    # 短暂漏检时保持最近一次已确认位置，避免 valid 单帧抖动。
                    x_mm = last_valid_x_mm
                    valid = 1 if valid_streak >= STABLE_FRAMES else 0
            else:
                center_x, center_y, radius = ball
                previous = (center_x, center_y)
                if filtered_center_x is None:
                    filtered_center_x = float(center_x)
                else:
                    # 仅用于显示和静止标定的多帧平均，不改变原始圆心检测结果。
                    filtered_center_x = (0.2 * center_x +
                                         0.8 * filtered_center_x)
                valid_streak += 1
                lost_streak = 0
                x_mm = clamp((center_x - IMAGE_CENTER_X) * MM_PER_PIXEL,
                             -MAX_POSITION_MM, MAX_POSITION_MM)
                valid = 1 if valid_streak >= STABLE_FRAMES else 0
                if valid:
                    last_valid_x_mm = x_mm

                draw_x = center_x * DISPLAY_SCALE
                draw_y = center_y * DISPLAY_SCALE
                display_frame.draw_circle(draw_x, draw_y,
                                          radius * DISPLAY_SCALE,
                                          color=(0, 255, 0), thickness=2)
                display_frame.draw_cross(draw_x, draw_y,
                                         color=(255, 0, 0), size=20,
                                         thickness=2)

            # 蓝线为标定中心，青/紫线分别为 -50/+50 mm 参考位置。
            display_frame.draw_line(MINUS_50_LINE_X, 0,
                                    MINUS_50_LINE_X, DISPLAY_HEIGHT,
                                    color=(0, 255, 255), thickness=1)
            display_frame.draw_string_advanced(MINUS_50_LINE_X + 4, 36, 16,
                                               "-50", color=(0, 255, 255))
            display_frame.draw_line(CENTER_LINE_X, 0,
                                    CENTER_LINE_X,
                                    DISPLAY_HEIGHT, color=(0, 0, 255),
                                    thickness=1)
            display_frame.draw_string_advanced(CENTER_LINE_X + 4, 36, 16,
                                               "0", color=(0, 0, 255))
            display_frame.draw_line(PLUS_50_LINE_X, 0,
                                    PLUS_50_LINE_X, DISPLAY_HEIGHT,
                                    color=(255, 0, 255), thickness=1)
            display_frame.draw_string_advanced(PLUS_50_LINE_X + 4, 36, 16,
                                               "+50", color=(255, 0, 255))
            if center_x is None:
                if valid:
                    status = "BALL HD x=%+.2fmm" % x_mm
                else:
                    status = "BALL LS fps=%.1f" % clock.fps()
            else:
                status = "BALL %s px=%d avg=%.2f x=%+.2fmm" % (
                    "OK" if valid else "HD", center_x,
                    filtered_center_x, x_mm)
            display_frame.draw_string_advanced(8, 8, 24, status,
                                               color=(255, 255, 0))
            Display.show_image(display_frame, 0, 0)

            frame_count += 1
            if frame_count % LOG_PERIOD_FRAMES == 0:
                log_message("ball: center_px=%s center_px_avg=%s "
                            "reference_px=%.3f x_mm=%.2f valid=%d fps=%.1f" %
                            (str(center_x), str(filtered_center_x),
                             IMAGE_CENTER_X, x_mm, valid, clock.fps()),
                            log_file)
            gc.collect()
    finally:
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        if log_file is not None:
            log_message("--- BALL preview stop ---", log_file)
            log_file.close()


if __name__ == "__main__":
    main()
