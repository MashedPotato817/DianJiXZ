"""K230 CanMV 小球位置检测并发送给 MSPM0 UART2。

接线：IO40/UART1_TX -> MSPM0 PB18/UART2_RX
      IO41/UART1_RX <- MSPM0 PB17/UART2_TX
帧格式：$K230,BALL,<x_mm>,<valid>,<seq>#\r\n

先在 K230 实机运行并标定 ball_detect_config.py；本文件不控制舵机或底盘。
"""

import gc
import time

import cv_lite
from machine import FPIOA, UART
from media.sensor import CAM_CHN_ID_1, Sensor

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
                                CALIBRATION_READY)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


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
    fpioa = FPIOA()
    fpioa.set_function(40, FPIOA.UART1_TXD)
    fpioa.set_function(41, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)

    sensor = Sensor(width=1280, height=960, fps=30)
    sensor.reset()
    sensor.set_framesize(w=FRAME_WIDTH, h=FRAME_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_1)
    sensor.run()

    print("K230 BALL detection started")
    print("mode=%s, UART1 IO40/IO41, %d baud" %
          (DETECT_MODE, UART_BAUDRATE))

    sequence = 0
    valid_streak = 0
    lost_streak = LOST_FRAMES
    previous = None
    last_valid_x_mm = 0.0
    last_send_ms = time.ticks_ms()
    frame_count = 0

    try:
        while True:
            frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            detected = find_ball(frame, previous)
            source = "none"

            if detected is not None:
                center_x, center_y, source = detected
                if previous is not None:
                    dx = center_x - previous[0]
                    dy = center_y - previous[1]
                    if dx * dx + dy * dy > MAX_CENTER_JUMP_PX * MAX_CENTER_JUMP_PX:
                        detected = None

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

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_PERIOD_MS:
                sequence += 1
                if CALIBRATION_READY:
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
                    print("RX:", response)
                    if b"$MSPM0,ACK#" in response:
                        print("MSPM0 BALL frame acknowledged")
                    if b"$MSPM0,HELLO#" in response:
                        print("MSPM0 heartbeat received")

            frame_count += 1
            if frame_count % LOG_PERIOD_FRAMES == 0:
                print("vision: source=%s x_mm=%.1f valid=%d | "
                      "uart: x_mm=%.1f valid=%d seq=%d" %
                      (source, x_mm, valid, tx_x_mm, tx_valid, sequence))
            gc.collect()
    finally:
        sensor.stop()


if __name__ == "__main__":
    main()
