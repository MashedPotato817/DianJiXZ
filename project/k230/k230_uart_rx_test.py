# K230 UART 回传线单向测试（CanMV MicroPython）
# 仅验证 MSPM0 PB6/UART1_TX -> K230 IO41/UART1_RX。
# 测试时不要连接 K230 IO40/TX，避免 K230 自发数据造成回灌干扰。

import time
from machine import FPIOA, UART

K230_RX_IO = 41
BAUDRATE = 115200

fpioa = FPIOA()
fpioa.set_function(K230_RX_IO, FPIOA.UART1_RXD)

uart = UART(UART.UART1, baudrate=BAUDRATE,
            bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE)

print("K230 UART RX test started")
print("Waiting for $MSPM0,HELLO# from MSPM0 PB6...")

while True:
    if uart.any():
        received = uart.read()
        print("RX:", received)
        if received and b"$MSPM0,HELLO#" in received:
            print("MSPM0 UART1 TX: PASS")
    time.sleep_ms(10)
