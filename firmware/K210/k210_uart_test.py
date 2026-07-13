from fpioa_manager import fm
from machine import UART
import time

try:
    from Maix import GPIO
    from board import board_info
except Exception:
    GPIO = None
    board_info = None


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


def init_led():
    if GPIO is None or board_info is None:
        print("status led unavailable")
        return None

    try:
        fm.register(board_info.LED_G, fm.fpioa.GPIO0, force=True)
        led = GPIO(GPIO.GPIO0, GPIO.OUT)
        led.value(1)
        return led
    except Exception as err:
        print("status led init failed:", err)
        return None


def set_led(led, on):
    if led is not None:
        led.value(0 if on else 1)


fm.register(K210_TX_PIN, fm.fpioa.UART1_TX, force=True)
fm.register(K210_RX_PIN, fm.fpioa.UART1_RX, force=True)

uart = UART(UART.UART1, BAUD, 8, 0, 1, timeout=10, read_buf_len=256)

status_led = init_led()
last_send = time.ticks_ms()
last_heartbeat = last_send
sequence = 0
led_on = False

print("K210 UART test started, tx=IO%d rx=IO%d baud=%d" % (K210_TX_PIN, K210_RX_PIN, BAUD))

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

    if time.ticks_diff(now, last_heartbeat) >= 500:
        last_heartbeat = now
        led_on = not led_on
        set_led(status_led, led_on)
        print("K210 alive seq=%d" % sequence)

    time.sleep_ms(5)
