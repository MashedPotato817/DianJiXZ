"""
HC-06 Bluetooth Serial Terminal (PC)

用法:
  1. 先在 Windows 设置中配对 HC-06（蓝牙 → 添加设备 → HC-06，密码 1234）
  2. 在设备管理器中找到 HC-06 对应的 COM 口
  3. 运行本脚本:
       python hc06_terminal.py COM3
     如果不加 COM 口参数，脚本会列出所有可用串口让你选。
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports


def list_ports():
    """列出所有可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到任何串口！")
        print("请确认 HC-06 已在 Windows 蓝牙设置中配对。")
        return None

    print("\n可用串口:")
    print("-" * 60)
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  —  {p.description}")
    print("-" * 60)
    return ports


def reader_thread(ser, stop_event):
    """后台线程：持续读取串口数据并打印"""
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                text = data.decode('utf-8', errors='replace')
                sys.stdout.write(f"\r\033[K{text}")  # \033[K clears current line
                sys.stdout.write("\n> ")              # re-print prompt
                sys.stdout.flush()
        except (serial.SerialException, OSError):
            break
        time.sleep(0.05)


def main():
    port = None

    # 参数解析
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        ports = list_ports()
        if ports is None:
            return
        sel = input("\n选择串口编号 (或输入 COM 口名): ").strip()
        try:
            idx = int(sel)
            port = ports[idx].device
        except (ValueError, IndexError):
            port = sel

    # 打开串口
    print(f"\n连接 {port} @ 9600 ...", end=" ")
    ser = serial.Serial(port, baudrate=9600, timeout=0.1)
    print("OK!")

    print("=" * 50)
    print("  HC-06 Terminal — 等待数据...")
    print("  输入文字回车发送，输入 :quit 退出")
    print("=" * 50)

    stop_event = threading.Event()
    reader = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    reader.start()

    try:
        sys.stdout.write("> ")
        sys.stdout.flush()
        while True:
            line = sys.stdin.readline()
            if not line:
                break
            line = line.rstrip('\n\r')
            if line == ':quit':
                break
            if line:
                ser.write((line + '\r\n').encode('utf-8'))
            sys.stdout.write("> ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n断开连接...")
        stop_event.set()
        reader.join(timeout=1)
        ser.close()
        print("已退出。")


if __name__ == '__main__':
    main()
