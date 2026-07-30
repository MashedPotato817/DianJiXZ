"""K230 CanMV 小球位置检测并发送给 MSPM0 UART1。

接线：IO40/UART1_TX -> MSPM0 PB7/UART1_RX
      IO41/UART1_RX <- MSPM0 PB6/UART1_TX
帧格式：$K230,BALL,<x_mm>,<valid>,<seq>#\r\n

先在 K230 实机运行并标定 ball_detect_config.py；本文件不控制舵机或底盘。
"""

import gc
import os
import time

import cv_lite
from machine import FPIOA, UART
from media.display import Display
from media.media import MediaManager
from media.sensor import CAM_CHN_ID_0, CAM_CHN_ID_1, Sensor

from ball_detect_config import (BALL_LAB_THRESHOLD, BALL_MAX_PIXELS,
                                BALL_MIN_ASPECT, BALL_MIN_PIXELS,
                                BALL_MAX_ASPECT, BALL_ROI, CIRCLE_ACCUMULATOR,
                                CIRCLE_CANNY_HIGH, CIRCLE_DP,
                                CIRCLE_MIN_DISTANCE, CIRCLE_R_MAX,
                                CIRCLE_R_MIN, CIRCLE_USE_ROI_CROP,
                                DETECT_MODE, FRAME_HEIGHT,
                                FRAME_WIDTH, IMAGE_CENTER_X, LOG_PERIOD_FRAMES,
                                LOST_FRAMES, MAX_CENTER_JUMP_PX,
                                MAX_POSITION_MM, MM_PER_PIXEL,
                                SEND_PERIOD_MS, STABLE_FRAMES, UART_BAUDRATE,
                                CALIBRATION_READY, UART_ALLOW_UNCALIBRATED,
                                ENABLE_LOG_FILE, LOG_FOLDER_PATH)

now = time.localtime()
LOG_FILE_PATH = LOG_FOLDER_PATH + "%04d%02d%02d_%02d%02d%02d_uart.txt" % \
                (now[0], now[1], now[2], now[3], now[4], now[5])
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_SCALE = DISPLAY_WIDTH // FRAME_WIDTH
CENTER_LINE_X = int(IMAGE_CENTER_X * DISPLAY_SCALE)
MINUS_50_LINE_X = int((IMAGE_CENTER_X - 50.0 / MM_PER_PIXEL) *
                      DISPLAY_SCALE)
PLUS_50_LINE_X = int((IMAGE_CENTER_X + 50.0 / MM_PER_PIXEL) *
                     DISPLAY_SCALE)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def log_timestamp():
    """使用 K230 系统时钟；未校时设备上的日期仅表示日志顺序。"""
    now = time.localtime()
    return "%04d-%02d-%02d %02d:%02d:%02d" % \
           (now[0], now[1], now[2], now[3], now[4], now[5])


def log_message(text, log_file):
    """终端与文件输出保持一致，文件不可用时仍保留终端信息。"""
    line = "[%s] %s" % (log_timestamp(), text)
    print(line)
    if log_file is not None:
        log_file.write(line + "\n")
        log_file.flush()


def find_blob_center(frame):
    candidates = []
    for blob in frame.find_blobs([BALL_LAB_THRESHOLD], roi=BALL_ROI,
                                 pixels_threshold=BALL_MIN_PIXELS,
                                 area_threshold=BALL_MIN_PIXELS):
        if blob.pixels() > BALL_MAX_PIXELS:
            continue
        aspect = blob.w() / blob.h() if blob.h() else 0.0
        if BALL_MIN_ASPECT <= aspect <= BALL_MAX_ASPECT:
            candidates.append(blob)
    if not candidates:
        return None
    ball = max(candidates, key=lambda blob: blob.pixels())
    return ball.cx(), ball.cy(), "blob"


def find_circle_center(frame, previous):
    # cv_lite 仅接收灰度 ndarray，返回扁平列表 [x, y, r, ...]。
    roi_x, roi_y, roi_w, roi_h = BALL_ROI
    if CIRCLE_USE_ROI_CROP:
        circle_frame = frame.copy(roi=BALL_ROI)
        image_height, image_width = roi_h, roi_w
    else:
        circle_frame = frame
        image_height, image_width = FRAME_HEIGHT, FRAME_WIDTH
        roi_x, roi_y = 0, 0

    raw_circles = cv_lite.grayscale_find_circles(
        [image_height, image_width], circle_frame.to_numpy_ref(), CIRCLE_DP,
        CIRCLE_MIN_DISTANCE, CIRCLE_CANNY_HIGH, CIRCLE_ACCUMULATOR,
        CIRCLE_R_MIN, CIRCLE_R_MAX)
    circles = []
    for index in range(0, len(raw_circles) - 2, 3):
        center_x = raw_circles[index] + roi_x
        center_y = raw_circles[index + 1] + roi_y
        if (BALL_ROI[0] <= center_x < BALL_ROI[0] + BALL_ROI[2] and
                BALL_ROI[1] <= center_y < BALL_ROI[1] + BALL_ROI[3]):
            circles.append((center_x, center_y))
    if not circles:
        return None

    if previous is None:
        reference_x = IMAGE_CENTER_X
        reference_y = FRAME_HEIGHT // 2
    else:
        reference_x, reference_y = previous

    center_x, center_y = min(circles, key=lambda circle:
                              (circle[0] - reference_x) ** 2 +
                              (circle[1] - reference_y) ** 2)
    return center_x, center_y, "cvlite_circle"


def find_ball(frame, previous):
    if DETECT_MODE == "blob":
        return find_blob_center(frame)
    return find_circle_center(frame, previous)


def main():
    sensor = None
    log_file = None
    fpioa = FPIOA()
    fpioa.set_function(40, FPIOA.UART1_TXD)
    fpioa.set_function(41, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)

    try:
        if ENABLE_LOG_FILE:
            try:
                try:
                    os.mkdir(LOG_FOLDER_PATH)
                except OSError:
                    pass
                log_file = open(LOG_FILE_PATH, "a")
                log_message("--- BALL UART start ---", log_file)
            except OSError as error:
                print("log file disabled:", error)

        sensor = Sensor(width=1280, height=960, fps=30)
        sensor.reset()
        sensor.set_framesize(w=DISPLAY_WIDTH, h=DISPLAY_HEIGHT,
                             chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)
        sensor.set_framesize(w=FRAME_WIDTH, h=FRAME_HEIGHT, chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
        Display.init(Display.ST7701, width=DISPLAY_WIDTH,
                     height=DISPLAY_HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()

        log_message("K230 BALL detection started", log_file)
        log_message("mode=%s, UART1 IO40/IO41, %d baud" %
                    (DETECT_MODE, UART_BAUDRATE), log_file)
        if (CALIBRATION_READY == False) and UART_ALLOW_UNCALIBRATED:
            log_message("WARNING: temporary uncalibrated UART position mode",
                        log_file)

        sequence = 0
        valid_streak = 0
        lost_streak = LOST_FRAMES
        previous = None
        last_valid_x_mm = 0.0
        last_send_ms = time.ticks_ms()
        frame_count = 0
        clock = time.clock()

        while True:
            clock.tick()
            display_frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            detect_frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            detected = find_ball(detect_frame, previous)
            source = "none"
            center_x = None
            center_y = None

            if detected is not None:
                center_x, center_y, source = detected
                if previous is not None:
                    dx = center_x - previous[0]
                    dy = center_y - previous[1]
                    if dx * dx + dy * dy > MAX_CENTER_JUMP_PX * MAX_CENTER_JUMP_PX:
                        detected = None
                        center_x = None
                        center_y = None
                        source = "none"

            if detected is None:
                lost_streak += 1
                if lost_streak >= LOST_FRAMES:
                    valid_streak = 0
                    previous = None
                    x_mm = 0.0
                    valid = 0
                else:
                    x_mm = last_valid_x_mm
                    valid = 1 if valid_streak >= STABLE_FRAMES else 0
            else:
                center_x, center_y, source = detected
                previous = (center_x, center_y)
                lost_streak = 0
                valid_streak += 1
                x_mm = clamp((center_x - IMAGE_CENTER_X) * MM_PER_PIXEL,
                             -MAX_POSITION_MM, MAX_POSITION_MM)
                valid = 1 if valid_streak >= STABLE_FRAMES else 0
                if valid:
                    last_valid_x_mm = x_mm

            if center_x is not None:
                display_frame.draw_cross(center_x * DISPLAY_SCALE,
                                         center_y * DISPLAY_SCALE,
                                         color=(255, 0, 0), size=20,
                                         thickness=2)
            display_frame.draw_line(MINUS_50_LINE_X, 0,
                                    MINUS_50_LINE_X, DISPLAY_HEIGHT,
                                    color=(0, 255, 255), thickness=1)
            display_frame.draw_string_advanced(MINUS_50_LINE_X + 4, 36, 16,
                                               "-50", color=(0, 255, 255))
            display_frame.draw_line(CENTER_LINE_X, 0, CENTER_LINE_X,
                                    DISPLAY_HEIGHT, color=(0, 0, 255),
                                    thickness=1)
            display_frame.draw_string_advanced(CENTER_LINE_X + 4, 36, 16,
                                               "0", color=(0, 0, 255))
            display_frame.draw_line(PLUS_50_LINE_X, 0,
                                    PLUS_50_LINE_X, DISPLAY_HEIGHT,
                                    color=(255, 0, 255), thickness=1)
            display_frame.draw_string_advanced(PLUS_50_LINE_X + 4, 36, 16,
                                               "+50", color=(255, 0, 255))
            roi_x, roi_y, roi_w, roi_h = BALL_ROI
            display_frame.draw_rectangle(roi_x * DISPLAY_SCALE,
                                         roi_y * DISPLAY_SCALE,
                                         roi_w * DISPLAY_SCALE,
                                         roi_h * DISPLAY_SCALE,
                                         color=(255, 255, 0), thickness=1)
            if center_x is None:
                state = "HD" if valid else "LS"
                status = "BALL %s" % state
            else:
                state = "OK" if valid else "HD"
                status = "BALL %s px=%d x=%+.2fmm" % (state, center_x, x_mm)
            display_frame.draw_string_advanced(8, 8, 24, status,
                                               color=(255, 255, 0))
            display_frame.draw_string_advanced(DISPLAY_WIDTH - 168, 8, 24,
                                               "FPS:%.1f" % clock.fps(),
                                               color=(0, 255, 0))
            Display.show_image(display_frame, 0, 0)

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_PERIOD_MS:
                sequence += 1
                if CALIBRATION_READY or UART_ALLOW_UNCALIBRATED:
                    tx_x_mm = x_mm
                    tx_valid = valid
                else:
                    tx_x_mm = 0.0
                    tx_valid = 0
                uart.write(("$K230,BALL,%.1f,%d,%d#\r\n" %
                            (tx_x_mm, tx_valid, sequence)).encode())
                last_send_ms = now_ms

            # 清空 MSPM0 心跳与 ACK，避免 K230 接收 FIFO 积累。
            if uart.any():
                response = uart.read()
                if response:
                    log_message("RX: %s" % response, log_file)
                    if b"$MSPM0,ACK#" in response:
                        log_message("MSPM0 BALL frame acknowledged", log_file)
                    if b"$MSPM0,HELLO#" in response:
                        log_message("MSPM0 heartbeat received", log_file)

            frame_count += 1
            if frame_count % LOG_PERIOD_FRAMES == 0:
                log_message("vision: source=%s x_mm=%.1f valid=%d | "
                            "uart: x_mm=%.1f valid=%d seq=%d mode=%s" %
                            (source, x_mm, valid, tx_x_mm, tx_valid, sequence,
                             "cal" if CALIBRATION_READY else
                             ("temp" if UART_ALLOW_UNCALIBRATED else "safe")),
                            log_file)
            gc.collect()
    finally:
        if sensor is not None:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        if log_file is not None:
            log_message("--- BALL UART stop ---", log_file)
            log_file.close()


if __name__ == "__main__":
    main()
