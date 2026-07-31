"""K230 CanMV 小球位置检测并发送给 MSPM0 UART1。

接线：IO40/UART1_TX -> MSPM0 PB7/UART1_RX
      IO41/UART1_RX <- MSPM0 PB6/UART1_TX
帧格式：
$K230,BALL,<x10>,<valid>,<seq>,<edge>*<crc8>#\r\n

x10 为毫米值乘 10 的有符号整数。crc8 使用 CRC-8/ATM，覆盖 '$' 后到
'*' 前的 ASCII 字节；MSPM0 不再接受无校验旧帧进入闭环。

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
                                DETECT_MODE, ENABLE_CONSOLE_SUMMARY,
                                ENABLE_LOG_SUMMARY,
                                ENABLE_STARTUP_CONFIG_LOG,
                                ENABLE_UART_RX_DEBUG, FRAME_HEIGHT,
                                FRAME_WIDTH, IMAGE_CENTER_X,
                                LOST_FRAMES, MAX_CENTER_JUMP_PX,
                                MAX_POSITION_MM, MM_PER_PIXEL,
                                EDGE_LOST_PIXEL_MARGIN,
                                GC_PERIOD_MS, LOG_SUMMARY_PERIOD_MS,
                                SEND_PERIOD_MS,
                                STABLE_FRAMES, UART_BAUDRATE,
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


def crc8_atm(payload):
    """CRC-8/ATM: poly=0x07, init=0x00，无反射、无异或输出。"""
    crc = 0
    for byte in payload.encode():
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def scale_x10(x_mm):
    if x_mm >= 0.0:
        return int(x_mm * 10.0 + 0.5)
    return int(x_mm * 10.0 - 0.5)


def build_ball_frame(x_mm, valid, sequence, edge_direction):
    x10_limit = int(MAX_POSITION_MM * 10.0 + 0.5)
    x10 = clamp(scale_x10(x_mm), -x10_limit, x10_limit)
    payload = "K230,BALL,%d,%d,%d,%d" % \
              (x10, valid, sequence, edge_direction)
    return "$%s*%02X#\r\n" % (payload, crc8_atm(payload))


def log_timestamp():
    """使用 K230 系统时钟；未校时设备上的日期仅表示日志顺序。"""
    now = time.localtime()
    return "%04d-%02d-%02d %02d:%02d:%02d" % \
           (now[0], now[1], now[2], now[3], now[4], now[5])


def log_message(text, log_file, to_console=True, to_file=True):
    """按需输出到终端和 TXT；常规运行避免将诊断流写入两端。"""
    line = "[%s] %s" % (log_timestamp(), text)
    if to_console:
        print(line)
    if to_file and log_file is not None:
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

        log_message("K230 BALL UART started", log_file)
        # 保留部署配置打印入口；日常运行不需要反复关注这些固定参数。
        if ENABLE_STARTUP_CONFIG_LOG:
            log_message("config: mode=%s UART1 IO40/IO41 %d baud" %
                        (DETECT_MODE, UART_BAUDRATE), log_file)
        if (CALIBRATION_READY == False) and UART_ALLOW_UNCALIBRATED:
            log_message("WARNING: temporary uncalibrated UART position mode",
                        log_file)

        sequence = 0
        valid_streak = 0
        lost_streak = LOST_FRAMES
        previous = None
        last_valid_x_mm = 0.0
        last_valid_center_x = None
        last_send_ms = time.ticks_ms()
        last_summary_ms = last_send_ms
        last_gc_ms = last_send_ms
        last_reported_state = None
        tx_x_mm = 0.0
        tx_valid = 0
        tx_edge_direction = 0
        clock = time.clock()

        while True:
            clock.tick()
            display_frame = sensor.snapshot(chn=CAM_CHN_ID_0)
            detect_frame = sensor.snapshot(chn=CAM_CHN_ID_1)
            frame_fps = clock.fps()
            detected = find_ball(detect_frame, previous)
            source = "none"
            center_x = None
            center_y = None
            edge_direction = 0

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
                    # 位置值保留到最后一次可靠检测，供 MSPM0 判断边缘恢复方向。
                    x_mm = last_valid_x_mm
                    valid = 0
                    if last_valid_center_x is not None:
                        if last_valid_center_x < EDGE_LOST_PIXEL_MARGIN:
                            edge_direction = -1
                        elif last_valid_center_x >= \
                                FRAME_WIDTH - EDGE_LOST_PIXEL_MARGIN:
                            edge_direction = 1
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
                    last_valid_center_x = center_x

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
                                               "FPS:%.1f" % frame_fps,
                                               color=(0, 255, 0))
            Display.show_image(display_frame, 0, 0)

            # 终端只在状态实际改变时提醒。运行中的连续数值保留给 TXT 摘要。
            if state != last_reported_state:
                log_message("STATE %s VIS=%+.1f valid=%d" %
                            (state, x_mm, valid), log_file)
                last_reported_state = state

            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_send_ms) >= SEND_PERIOD_MS:
                sequence = (sequence + 1) & 0xFFFFFFFF
                if sequence == 0:
                    sequence = 1
                if CALIBRATION_READY or UART_ALLOW_UNCALIBRATED:
                    tx_x_mm = x_mm
                    tx_valid = valid
                    tx_edge_direction = edge_direction
                else:
                    tx_x_mm = 0.0
                    tx_valid = 0
                    tx_edge_direction = 0
                uart.write(build_ball_frame(
                    tx_x_mm, tx_valid, sequence, tx_edge_direction).encode())
                last_send_ms = now_ms

            # 清空 MSPM0 心跳与 ACK，避免 K230 接收 FIFO 积累。
            if uart.any():
                response = uart.read()
                if response:
                    # 原始回包和心跳会显著降低终端可读性，仅在排查链路时打开。
                    if ENABLE_UART_RX_DEBUG:
                        log_message("RX: %s" % response, log_file)
                    if b"$MSPM0,ACK#" in response:
                        log_message("LINK ACK", log_file)

            # 视觉帧与 UART 发送周期不同。摘要明确区分 VIS（当前画面）和
            # TX（最近一次已发送帧），避免把二者的瞬时差异误判为串口错误。
            if ENABLE_LOG_SUMMARY and \
                    time.ticks_diff(now_ms, last_summary_ms) >= \
                    LOG_SUMMARY_PERIOD_MS:
                tx_age_ms = time.ticks_diff(now_ms, last_send_ms)
                summary_center_x = center_x if center_x is not None else -1
                log_message("BALL %s VIS=%+.1f/%d px=%d TX=%+.1f/%d "
                            "edge=%+d seq=%d age=%dms fps=%.1f" %
                            (state, x_mm, valid, summary_center_x,
                             tx_x_mm, tx_valid, tx_edge_direction, sequence,
                             tx_age_ms, frame_fps), log_file,
                            to_console=ENABLE_CONSOLE_SUMMARY)
                last_summary_ms = now_ms

            # 每帧回收会抢占检测和显示。定时回收仍可释放临时图像/候选对象，
            # 同时将开销移出高频帧路径。
            if time.ticks_diff(now_ms, last_gc_ms) >= GC_PERIOD_MS:
                gc.collect()
                last_gc_ms = now_ms
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
