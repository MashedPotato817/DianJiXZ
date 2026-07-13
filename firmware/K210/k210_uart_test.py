from fpioa_manager import fm
from machine import UART
import time


K210_TX_PIN = 7
K210_RX_PIN = 6
BAUD = 115200


def xor7(data):
    value = 0
    for byte in data[:7]:
        value ^= byte
    return value


def u16le(value):
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def make_frame(x, y, area):
    frame = b"\xAA" + u16le(x) + u16le(y) + u16le(area)
    return frame + bytes((xor7(frame),))


fm.register(K210_TX_PIN, fm.fpioa.UART1_TX, force=True)
fm.register(K210_RX_PIN, fm.fpioa.UART1_RX, force=True)

uart = UART(UART.UART1, BAUD, 8, 0, 1, timeout=10, read_buf_len=256)

last_send = time.ticks_ms()
sequence = 0

while True:
    data = uart.read()
    if data and b"$CAR,HELLO#" in data:
        uart.write(b"$K210,OK#")

    now = time.ticks_ms()
    if time.ticks_diff(now, last_send) >= 100:
        last_send = now
        sequence += 1
        x = 160 + (sequence % 40)
        y = 120
        area = 1234
        uart.write(make_frame(x, y, area))

    time.sleep_ms(5)
