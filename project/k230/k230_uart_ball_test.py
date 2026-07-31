# K230 到 MSPM0 UART1 最小联调脚本（CanMV MicroPython）
# 接线：K230 IO40/TX -> MSPM0 PB7/UART1_RX
#       K230 IO41/RX <- MSPM0 PB6/UART1_TX

import time
from machine import FPIOA, UART

K230_UART_PORT = UART.UART1
K230_TX_IO = 40
K230_RX_IO = 41
BAUDRATE = 115200
FRAME_PERIOD_MS = 50
# 默认关闭。实测保护逻辑时每次只开启一项，例如改为 20。
INJECT_BAD_CRC_EVERY = 0
INJECT_RANGE_ERROR_EVERY = 0


def crc8_atm(payload):
    crc = 0
    for byte in payload.encode():
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_ball_frame(x10, valid, sequence, edge_direction):
    payload = "K230,BALL,%d,%d,%d,%d" % \
              (x10, valid, sequence, edge_direction)
    return "$%s*%02X#\r\n" % (payload, crc8_atm(payload))


fpioa = FPIOA()
fpioa.set_function(K230_TX_IO, FPIOA.UART1_TXD)
fpioa.set_function(K230_RX_IO, FPIOA.UART1_RXD)

uart = UART(K230_UART_PORT, baudrate=BAUDRATE,
            bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE)

sequence = 0
last_send_ms = time.ticks_ms()
last_report_seq = 0

print("K230 UART BALL test started")
print("TX: $K230,BALL,-250,1,<seq>,0*<crc8>#")

while True:
    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, last_send_ms) >= FRAME_PERIOD_MS:
        sequence = (sequence + 1) & 0xFFFFFFFF
        if sequence == 0:
            sequence = 1
        if (INJECT_RANGE_ERROR_EVERY > 0 and
                sequence % INJECT_RANGE_ERROR_EVERY == 0):
            # CRC 正确但超出 MSPM0 允许的 +/-150.0 mm。
            frame = build_ball_frame(2000, 1, sequence, 0)
        else:
            frame = build_ball_frame(-250, 1, sequence, 0)
        if INJECT_BAD_CRC_EVERY > 0 and \
                sequence % INJECT_BAD_CRC_EVERY == 0:
            marker = frame.index("*")
            old_crc = frame[marker + 1:marker + 3]
            bad_crc = "00" if old_crc != "00" else "FF"
            frame = frame[:marker + 1] + bad_crc + frame[marker + 3:]
        uart.write(frame.encode())
        last_send_ms = now_ms
        if sequence - last_report_seq >= 20:
            print("BALL frames sent, seq:", sequence)
            last_report_seq = sequence

    if uart.any():
        received = uart.read()
        print("RX:", received)
        if received and b"$MSPM0,HELLO#" in received:
            print("MSPM0 heartbeat received")
        if received and b"$MSPM0,ACK#" in received:
            print("MSPM0 BALL frame acknowledged")
