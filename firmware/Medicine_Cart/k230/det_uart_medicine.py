# -*- coding: utf-8 -*-
'''
智能送药小车 — K230 端：数字识别 + UART 通信集成脚本。

功能：
  - 加载病房号 1-8 目标检测模型（mp_deployment_source/）
  - 实时采集摄像头画面，运行推理
  - 通过 UART1 (IO40/IO41) 与 MSPM0 通信：
      握手 → 心跳 → $K230,RESULT,<ward>,<conf># 识别结果
  - 屏幕显示检测结果 + 链路状态

部署：
  1. 将本脚本上传到 K230 开发板 /sdcard/
  2. 确保 /sdcard/mp_deployment_source/ 下有 deploy_config.json 和 .kmodel
  3. 确保 libs/ 目录可导入（PipeLine, PlatTasks, Utils）
  4. 确认 IO40/IO41 已接 MSPM0 PB7/PB6
  5. 在 CanMV IDE 运行本脚本
'''

import os, gc, time
from machine import Pin, FPIOA, UART
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *

# ========== 通信参数（与 MSPM0 k230_link.c 一致） ==========
K230_PORT      = 1
BAUDRATE       = 115200
HELLO_PERIOD   = 500       # ms
DATA_PERIOD    = 1000      # ms, 心跳间隔
LINK_TIMEOUT   = 3000      # ms, 断链超时
RESULT_MIN_INTERVAL = 200  # ms, 两次 RESULT 发送最小间隔（避免 flooding）

# ========== 模型参数 ==========
ROOT_PATH      = "/sdcard/mp_deployment_source/"
DISPLAY_MODE   = "lcd"
RGB888P_SIZE   = [640, 360]

# ========== 初始化 FPIOA + UART ==========
fpioa = FPIOA()
fpioa.set_function(40, FPIOA.UART1_TXD)
fpioa.set_function(41, FPIOA.UART1_RXD)

# K0 按键（保存截图）
fpioa.set_function(34, FPIOA.GPIO34)
key0 = Pin(34, Pin.IN, pull=Pin.PULL_UP, drive=7)
save_dir = "/data/pic"
try:
    os.mkdir(save_dir)
except OSError:
    pass

uart = UART(UART.UART1, baudrate=BAUDRATE,
            bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE)

# ========== UART 协议状态 ==========
rx_buffer   = b""
got_ack     = False
got_msp_hello = False
link_online = False
last_rx     = time.ticks_ms()
last_hello  = last_rx
last_data   = last_rx
last_result = 0

# ========== 识别结果稳定 ==========
STABLE_COUNT = 3          # 连续相同结果才视为稳定
pending_ward = 0
pending_count = 0
best_ward   = 0           # 当前帧最优结果
best_conf   = 0


def uart_send(text):
    '''发送 ASCII 帧，自动追加 \\r\\n'''
    uart.write(text + b"\r\n")
    print("TX:", text.decode())


def handle_rx_line(line):
    '''解析 $...# 帧，维护握手状态'''
    global got_ack, got_msp_hello, link_online, last_rx

    last_rx = time.ticks_ms()
    print("RX:", line.decode())

    if line == b"$MSPM0,HELLO#":
        got_msp_hello = True
        uart_send(b"$K230,ACK#")
    elif line == b"$MSPM0,ACK#":
        got_ack = True
    elif line == b"$CAR,HELLO#":
        # 兼容旧主板 k210_link.c 握手
        uart_send(b"$K210,OK#")
    elif line == b"$MSPM0,DATA#":
        uart_send(b"$K230,DATA_ACK#")
    elif line == b"$MSPM0,LINK_OK#":
        link_online = True
        print("LINK OK: 双方 HELLO/ACK 已确认")


def process_rx():
    '''从 UART FIFO 读取并提取 $...# 帧'''
    global rx_buffer
    data = uart.read()
    if not data:
        return
    rx_buffer += data
    while b"#" in rx_buffer:
        end = rx_buffer.find(b"#")
        line = rx_buffer[:end + 1]
        rx_buffer = rx_buffer[end + 1:].lstrip(b"\r\n")
        if line.startswith(b"$"):
            handle_rx_line(line)


def time_ms():
    return time.ticks_ms()


def time_diff(now, last):
    return time.ticks_diff(now, last)


def send_result(ward, confidence):
    '''发送识别结果帧：$K230,RESULT,<ward>,<conf>#'''
    msg = "$K230,RESULT,%d,%d#" % (ward, confidence)
    uart_send(msg.encode())


# ========== 加载模型 ==========
deploy_conf = read_json(ROOT_PATH + "deploy_config.json")
kmodel_path    = ROOT_PATH + deploy_conf["kmodel_path"]
labels         = deploy_conf["categories"]
conf_threshold = deploy_conf["confidence_threshold"]
nms_threshold  = deploy_conf["nms_threshold"]
model_input    = deploy_conf["img_size"]
nms_option     = deploy_conf["nms_option"]
model_type     = deploy_conf["model_type"]

anchors = []
if model_type == "AnchorBaseDet":
    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

# ========== 初始化摄像头管线 ==========
pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode=DISPLAY_MODE)
pl.create()
display_size = pl.get_display_size()

# ========== 初始化检测器 ==========
det_app = DetectionApp("video", kmodel_path, labels, model_input,
                        anchors, model_type, conf_threshold, nms_threshold,
                        RGB888P_SIZE, display_size, debug_mode=0)
det_app.config_preprocess()

# ========== 启动通告 ==========
print("MedCart K230: 数字识别 + UART 通信启动")
print("  模型: %s" % kmodel_path)
print("  类别: %s" % labels)
print("  UART: Port%d, %d baud" % (K230_PORT, BAUDRATE))

uart_send(b"$K230,BOOT#")
key0_last = key0.value()

# ========== 主循环 ==========
while True:
    now = time_ms()

    # 1. 处理 MSPM0 发来的帧
    process_rx()

    # 2. 握手：发 HELLO 直到收到 ACK
    if not got_ack and time_diff(now, last_hello) >= HELLO_PERIOD:
        last_hello = now
        uart_send(b"$K230,HELLO#")

    # 3. 链路就绪判定
    if not link_online and got_msp_hello and got_ack:
        link_online = True
        print("LINK OK: 双方 HELLO/ACK 已确认")

    # 4. 心跳
    if link_online and time_diff(now, last_data) >= DATA_PERIOD:
        last_data = now
        uart_send(b"$K230,DATA#")

    # 5. 超时断链
    if link_online and time_diff(now, last_rx) >= LINK_TIMEOUT:
        link_online = False
        got_msp_hello = False
        got_ack = False
        print("LINK LOST: 3 秒未收到 MSPM0 数据")

    # 6. 采集 + 推理
    with ScopedTiming("total", 0):
        img = pl.get_frame()
        res = det_app.run(img)
        det_app.draw_result(pl.osd_img, res)

    # 7. 解析检测结果，提取最优病房号
    best_ward = 0
    best_conf = 0

    if res:
        if isinstance(res, dict):
            # CanMV v1.8+ API: res = {'scores':array, 'idx':array, 'boxes':array}
            scores = res.get('scores', [])
            idxs   = res.get('idx', [])
            for i in range(len(scores)):
                score = float(scores[i])
                if score > best_conf:
                    best_conf = score
                    class_id = int(idxs[i])
                    # labels = ["4","5","3","6","7","8","1","2"]
                    best_ward = int(labels[class_id])
        else:
            # 旧 API: res = [[class_id, score, x1, y1, x2, y2], ...]
            for det in res:
                class_id = int(det[0])
                score    = float(det[1])
                if score > best_conf:
                    best_conf = score
                    best_ward = int(labels[class_id])

    # 8. 稳定性确认 + 发送
    if best_ward >= 1 and best_ward <= 8 and best_conf >= conf_threshold:
        if best_ward == pending_ward:
            pending_count += 1
            if pending_count >= STABLE_COUNT and link_online:
                if time_diff(now, last_result) >= RESULT_MIN_INTERVAL:
                    send_result(best_ward, int(best_conf * 100))
                    last_result = now
        else:
            pending_ward = best_ward
            pending_count = 1
    else:
        # 无有效检测 → 重置待确认
        pending_ward = 0
        pending_count = 0

    # 9. 显示链路状态到 OSD
    status_line = "K230"
    if link_online:
        status_line += " LINK"
        if best_ward > 0:
            status_line += " | Ward:%d %.0f%%" % (best_ward, best_conf * 100)
    else:
        status_line += " WAIT"
    # 覆盖左上角状态（如果需要，用 draw_result 已画检测框）

    # 10. K0 按键保存截图
    key0_now = key0.value()
    if key0_last and not key0_now:
        photo = pl.get_capture_frame()
        photo.draw_image(pl.osd_img, 0, 0)
        filename = save_dir + "/med_%d.jpg" % time.ticks_ms()
        photo.save(filename, quality=95)
        print("photo saved:", filename)
    key0_last = key0_now

    # 11. 屏幕刷新
    pl.show_image()
    gc.collect()

# Cleanup (unreachable under normal operation)
det_app.deinit()
pl.destroy()
