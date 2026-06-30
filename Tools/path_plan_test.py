"""
路径规划测试脚本 — 纯 ASCII 文本模式

用法:
    python path_plan_test.py COM3

功能:
    输入 x y (单位 m) → 下位机 A* 规划 (起点0,0) → 回传路点序列
    也支持二进制 0xA5 帧发送目标坐标

依赖: pyserial (pip install pyserial)
"""

import sys
import serial
import time


def ascii_mode(ser):
    """交互式 ASCII 输入测试"""
    print("=" * 50)
    print("  纯路径规划测试 — ASCII 模式")
    print("  起点固定 (0, 0)mm")
    print("  输入 x y (单位 m) → 规划 → 回传路点")
    print("  输入 q 退出")
    print("=" * 50)

    while True:
        try:
            inp = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if inp.lower() == 'q':
            break
        if not inp:
            continue

        # 发送: "x y\r"
        ser.write((inp + "\r").encode())
        time.sleep(0.8)

        # 读取回传
        while ser.in_waiting:
            data = ser.read(ser.in_waiting)
            try:
                print(data.decode(errors='replace'), end='', flush=True)
            except:
                pass

    print("\n退出")


def binary_mode(ser, x_m, y_m):
    """二进制 0xA5 帧发送目标坐标 (x,y 单位 m)"""
    import struct

    x_mm = int(x_m * 1000)
    y_mm = int(y_m * 1000)

    payload = struct.pack('<ii', x_mm, y_mm)  # 小端 int32×2

    cmd = 0x01
    length = 8
    checksum = cmd ^ length
    for b in payload:
        checksum ^= b

    frame = bytes([0xA5, cmd, length]) + payload + bytes([checksum & 0xFF])

    print(f"\n发送 0xA5 帧: 目标({x_mm}, {y_mm})mm")
    print(f"  原始: {frame.hex(' ').upper()}")

    ser.write(frame)
    time.sleep(1.0)

    # 读取 0x90 路点回复
    wp_count = 0
    buf = b""
    while ser.in_waiting or wp_count == 0:
        buf += ser.read(ser.in_waiting or 1)
        # 搜索 0xA5 0x90 帧头
        while len(buf) >= 14:  # 最小帧: 0xA5 + 0x90 + 0x0A + 10B + Xor = 14B
            if buf[0] == 0xA5 and buf[1] == 0x90 and buf[2] == 0x0A:
                idx = buf[3]
                total = buf[4]
                x = int.from_bytes(buf[5:9], 'little', signed=True)
                y = int.from_bytes(buf[9:13], 'little', signed=True)
                print(f"  路点 [{idx+1}/{total}]: ({x}, {y})mm = ({x/1000:.2f}, {y/1000:.2f})m")
                wp_count += 1
                buf = buf[14:]
                if idx + 1 >= total:
                    print(f"  → 共 {total} 个路点, 规划完成\n")
                    return
            else:
                buf = buf[1:]
        time.sleep(0.2)

    if wp_count == 0:
        print("  ⚠ 未收到路点回复 (路径不可达或超时)")


def main():
    if len(sys.argv) < 2:
        print("用法: python path_plan_test.py <COM端口> [x_m y_m]")
        print("  ASCII交互: python path_plan_test.py COM3")
        print("  二进制发送: python path_plan_test.py COM3 5 4")
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        print(f"已连接 {port} @ 115200 8N1")
    except Exception as e:
        print(f"无法打开 {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    if len(sys.argv) >= 4:
        # 二进制模式: 发一次目标坐标
        x_m = float(sys.argv[2])
        y_m = float(sys.argv[3])
        binary_mode(ser, x_m, y_m)
    else:
        # ASCII 交互模式
        ascii_mode(ser)

    ser.close()


if __name__ == "__main__":
    main()
