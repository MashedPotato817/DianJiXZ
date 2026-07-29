# K230 到 MSPM0 UART2 最小联调脚本（CanMV MicroPython）
# 接线：K230 IO40/TX -> MSPM0 PB18/UART2_RX
#       K230 IO41/RX <- MSPM0 PB17/UART2_TX

import time
from machine import FPIOA, UART

K230_UART_PORT = UART.UART1
K230_TX_IO = 40
K230_RX_IO = 41
BAUDRATE = 115200
FRAME_PERIOD_MS = 50

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
print("TX: $K230,BALL,-25.0,1,<seq>#")

while True:
    now_ms = time.ticks_ms()

    if time.ticks_diff(now_ms, last_send_ms) >= FRAME_PERIOD_MS:
        sequence += 1
        frame = "$K230,BALL,-25.0,1,{}#\r\n".format(sequence)
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
