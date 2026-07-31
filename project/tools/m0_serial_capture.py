#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MSPM0 遥测实时读取 + 交互式发命令（pyserial）
- 端口 COM7 / 115200 / 8N1
- 读 M0 UART0 的 $T/$E 行，解析关键字段实时打印，原始行存文件
- 同时启动后台线程接受命令：边看数据边调参，无需重编译。
- 用法：python m0_serial_capture.py [端口] [输出文件]

命令输入（在 "CMD> " 提示符下）：
  KP 0.30    ->  $SET,KP,0.30#     设置位置比例增益
  KD 0.05    ->  $SET,KD,0.05#     设置速度微分增益
  TRIM 122.4 ->  $SET,TRIM,122.4#  设置平衡角（trim）
  CAL        ->  $CAL#             触发自动扫掠标定
  $...       ->  原样透传任意命令（供扩展）
  help / quit

注意：要发命令，电脑 USB-TTL 的 TX 必须接到 MSPM0 PA11/UART0_RX（只读日志时可不接）。
"""
import serial
import sys
import threading

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD = 115200
OUTFILE = sys.argv[2] if len(sys.argv) > 2 else "m0_log.txt"

# 20 字段 TELEM_V3 格式
FIELDS = ["t_ms","dt_ms","rx_ms","rx_age_ms","seq","x10","valid","edge",
          "v10","toward_v10","stop10","trim10","servo_us","out10","ctrl",
          "parse_err","crc_err","seq_gap","target_x10","task"]

def parse_row(text):
    parts = text.split(",")
    if len(parts) < len(FIELDS):
        return None
    row = {}
    for i, name in enumerate(FIELDS):
        v = parts[i].strip()
        try:
            row[name] = int(v)
        except ValueError:
            row[name] = v
    return row

def fmt_row(r):
    return (f"t={r['t_ms']:>7} x={r['x10']/10:>6.1f} "
            f"v={r['v10']/10:>6.1f} trim={r['trim10']/10:>5.1f} "
            f"servo={r['servo_us']:>4} out={r['out10']/10:>5.1f} "
            f"ctrl={r['ctrl']:>2} tgt={r['target_x10']/10:>5.1f} task={r['task']}")

def user_input_thread(ser):
    """后台线程：阻塞读 stdin，把命令写成 MSPM0 UART0 命令帧。"""
    while True:
        try:
            line = input("CMD> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        low = line.lower()
        if low in ("q", "quit", "exit"):
            break
        if low == "help":
            print("命令: KP <值> | KD <值> | TRIM <值> | CAL | $原始帧 | quit",
                  flush=True)
            continue
        parts = line.split(None, 1)
        if low == "cal":
            frame = "$CAL#"
        elif (parts and parts[0].upper() in ("KP", "KD", "TRIM")
                and len(parts) == 2):
            frame = "$SET,%s,%s#" % (parts[0].upper(), parts[1].strip())
        elif line.startswith("$"):
            frame = line
        else:
            print("无法识别命令，输入 help 查看", flush=True)
            continue
        ser.write(frame.encode())
        print(">>> 已发送: %s" % frame, flush=True)


def main():
    print(f"打开 {PORT} @ {BAUD} ...（Ctrl+C 停止，日志存 {OUTFILE}）")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    threading.Thread(target=user_input_thread, args=(ser,), daemon=True).start()
    buf = b""
    last_t = None
    with open(OUTFILE, "wb") as f:
        try:
            while True:
                data = ser.read(512)
                if data:
                    f.write(data)
                    f.flush()
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        text = line.decode("gbk", errors="replace")
                        if text.startswith("$T,"):
                            r = parse_row(text[2:])
                            if r is not None:
                                print(fmt_row(r), flush=True)
                        elif text.startswith("$E,"):
                            print("EVENT:", text, flush=True)
                        elif text.startswith("#TELEM"):
                            print("HEADER:", text[:60], flush=True)
        except KeyboardInterrupt:
            print("\n停止。日志已存", OUTFILE)
        finally:
            ser.close()

if __name__ == "__main__":
    main()
