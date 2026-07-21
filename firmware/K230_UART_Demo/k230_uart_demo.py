"""K230 CanMV 与 MSPM0 双向 UART 最小联调脚本。"""
from machine import FPIOA, UART
import time

K230_PORT = 1
BAUDRATE = 115200
HELLO_PERIOD_MS = 500
DATA_PERIOD_MS = 1000
LINK_TIMEOUT_MS = 3000

fpioa = FPIOA()
if K230_PORT == 1:
    fpioa.set_function(40, FPIOA.UART1_TXD)
    fpioa.set_function(41, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)
else:
    fpioa.set_function(44, FPIOA.UART2_TXD)
    fpioa.set_function(45, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, baudrate=BAUDRATE,
                bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE)

rx_buffer = b""
got_msp_hello = False
got_msp_ack = False
link_online = False
last_rx = time.ticks_ms()
last_hello = last_rx
last_data = last_rx


def send(text):
    uart.write(text + b"\r\n")
    print("TX:", text.decode())


def handle_line(line):
    global got_msp_hello, got_msp_ack, link_online, last_rx
    last_rx = time.ticks_ms()
    print("RX:", line.decode())

    if line == b"$MSPM0,HELLO#":
        got_msp_hello = True
        send(b"$K230,ACK#")
    elif line == b"$MSPM0,ACK#":
        got_msp_ack = True
    elif line == b"$MSPM0,DATA#":
        send(b"$K230,DATA_ACK#")
    elif line == b"$MSPM0,LINK_OK#":
        print("LINK OK: 双方 HELLO/ACK 已确认")
        link_online = True


def process_rx():
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
            handle_line(line)


print("K230 UART handshake demo started, Port%d, %d baud" % (K230_PORT, BAUDRATE))
send(b"$K230,BOOT#")

while True:
    process_rx()
    now = time.ticks_ms()

    if (not got_msp_ack and
            time.ticks_diff(now, last_hello) >= HELLO_PERIOD_MS):
        last_hello = now
        send(b"$K230,HELLO#")

    if not link_online and got_msp_hello and got_msp_ack:
        link_online = True
        print("LINK OK: 双方 HELLO/ACK 已确认")

    if link_online and time.ticks_diff(now, last_data) >= DATA_PERIOD_MS:
        last_data = now
        send(b"$K230,DATA#")

    if link_online and time.ticks_diff(now, last_rx) >= LINK_TIMEOUT_MS:
        link_online = False
        got_msp_hello = False
        got_msp_ack = False
        print("LINK LOST: 3 秒未收到 MSPM0 数据")

    time.sleep_ms(5)
