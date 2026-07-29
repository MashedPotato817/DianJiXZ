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
                                STABLE_FRAMES)

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_SCALE = DISPLAY_WIDTH // FRAME_WIDTH


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


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
    try:
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
        print("K230 BALL preview started: cv_lite circle detection")

        previous = None
        valid_streak = 0
        lost_streak = LOST_FRAMES
        frame_count = 0
        clock = time.clock()

        while True:
            os.exitpoint()
            clock.tick()
            display_frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            detect_frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            ball = find_circle(detect_frame, previous)

            if ball is not None and previous is not None:
                dx = ball[0] - previous[0]
                dy = ball[1] - previous[1]
                if dx * dx + dy * dy > MAX_CENTER_JUMP_PX * MAX_CENTER_JUMP_PX:
                    ball = None

            if ball is None:
                valid_streak = 0
                lost_streak += 1
                if lost_streak >= LOST_FRAMES:
                    previous = None
                x_mm = 0.0
                valid = 0
            else:
                center_x, center_y, radius = ball
                previous = (center_x, center_y)
                valid_streak += 1
                lost_streak = 0
                x_mm = clamp((center_x - IMAGE_CENTER_X) * MM_PER_PIXEL,
                             -MAX_POSITION_MM, MAX_POSITION_MM)
                valid = 1 if valid_streak >= STABLE_FRAMES else 0

                draw_x = center_x * DISPLAY_SCALE
                draw_y = center_y * DISPLAY_SCALE
                display_frame.draw_circle(draw_x, draw_y,
                                          radius * DISPLAY_SCALE,
                                          color=(0, 255, 0), thickness=2)
                display_frame.draw_cross(draw_x, draw_y,
                                         color=(255, 0, 0), size=20,
                                         thickness=2)

            # 蓝线为标定的摆杆中心；绿圈/红十字为当前候选球。
            display_frame.draw_line(IMAGE_CENTER_X * DISPLAY_SCALE, 0,
                                    IMAGE_CENTER_X * DISPLAY_SCALE,
                                    DISPLAY_HEIGHT, color=(0, 0, 255),
                                    thickness=1)
            status = "BALL %s x=%+.1fmm fps=%.1f" % (
                "OK" if valid else "LOST", x_mm, clock.fps())
            display_frame.draw_string_advanced(8, 8, 24, status,
                                               color=(255, 255, 0))
            Display.show_image(display_frame, 0, 0)

            frame_count += 1
            if frame_count % LOG_PERIOD_FRAMES == 0:
                print("ball: x_mm=%.1f valid=%d fps=%.1f" %
                      (x_mm, valid, clock.fps()))
            gc.collect()
    finally:
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()


if __name__ == "__main__":
    main()
