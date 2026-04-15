#!/usr/bin/env python3
import socket
import struct

HOST = "127.0.0.1"
PORT = 8080

def parse_pointcloud_packet(data):
    """解析单个 PointCloud TCP 数据包"""
    if len(data) < 8:  # 最小包长度不够
        return None

    if data[0] != 0xF5 or data[-1] != 0x5F:
        return None

    func_code = data[1]
    if func_code not in (0x02, 0x03):
        return None

    # 小端读取点数量 N
    N = struct.unpack_from('<H', data, 2)[0]

    expected_len = 1 + 1 + 2 + N*24 + 2 + 1
    if len(data) != expected_len:
        return None

    points = []
    offset = 4
    for _ in range(N):
        x = struct.unpack_from('<d', data, offset)[0]
        y = struct.unpack_from('<d', data, offset + 8)[0]
        z = struct.unpack_from('<d', data, offset + 16)[0]
        points.append((x, y, z))
        offset += 24

    return func_code, points

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print(f"Connected to {HOST}:{PORT}")

    buffer = b""

    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                print("Server closed connection.")
                break

            buffer += chunk

            # 循环解析完整包
            while True:
                start = buffer.find(b'\xF5')
                if start == -1:
                    buffer = b""  # 没找到包头，丢弃数据
                    break

                if len(buffer) < start + 8:
                    break  # 数据不够，等待下次 recv

                # 小端读取点数量 N
                N = struct.unpack_from('<H', buffer, start + 2)[0]
                total_len = 1 + 1 + 2 + N*24 + 2 + 1

                if len(buffer) < start + total_len:
                    break  # 包不完整，等待下次 recv

                packet = buffer[start:start+total_len]
                buffer = buffer[start+total_len:]  # 移除已解析数据

                result = parse_pointcloud_packet(packet)
                if result:
                    func_code, points = result
                    label = "Acquired" if func_code == 0x02 else "Unacquired"
                    print(f"[{label}] Received {len(points)} points:")
                    for i, (x, y, z) in enumerate(points[:5]):
                        print(f"  Point {i}: ({x:.3f}, {y:.3f}, {z:.3f})")
                    if len(points) > 5:
                        print(f"  ... and {len(points) - 5} more points")
                    print("-"*50)
                else:
                    print("Invalid or unsupported packet, skipped.")
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
