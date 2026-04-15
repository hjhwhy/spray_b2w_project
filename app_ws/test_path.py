import socket
import struct

HOST = '127.0.0.1'  # ROS2 节点所在机器 IP
PORT = 8080         # ROS2 节点 TCP 端口

def parse_path_packet(data):
    """
    解析轨迹包:
    包头 0xF5 | 功能码 0x04 | 点数量 N (1B) | N*24字节轨迹点 (double) | CRC低 | CRC高 | 包尾 0x5F
    """
    if len(data) < 5:
        return None

    if data[0] != 0xF5 or data[1] != 0x04 or data[-1] != 0x5F:
        print("包头/功能码/包尾不匹配")
        return None

    N = data[2]
    expected_len = 1 + 1 + 1 + N * 24 + 2 + 1
    if len(data) != expected_len:
        print(f"长度不匹配: 期望 {expected_len}, 实际 {len(data)}")
        return None

    points = []
    idx = 3
    for _ in range(N):
        x, y, z = struct.unpack('<ddd', data[idx:idx+24])  # 小端 double
        points.append((x, y, z))
        idx += 24

    crc_low = data[idx]
    crc_high = data[idx+1]

    return {
        'num_points': N,
        'points': points,
        'crc': (crc_low, crc_high)
    }

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT))
            print(f"已连接到 ROS2 节点 TCP 端口 {HOST}:{PORT}")
        except Exception as e:
            print(f"连接失败: {e}")
            return

        buffer = bytearray()
        while True:
            try:
                data = s.recv(1024)
                if not data:
                    print("服务器断开连接")
                    break
                buffer.extend(data)

                # 尝试解析完整的包
                while True:
                    if len(buffer) < 5:
                        break  # 不够最小包
                    try:
                        header_index = buffer.index(0xF5)
                    except ValueError:
                        buffer.clear()
                        break

                    if len(buffer) < header_index + 5:
                        break  # 不够最小包长度

                    func_code = buffer[header_index + 1]
                    if func_code != 0x04:
                        buffer = buffer[header_index + 1:]  # 跳过无效包
                        continue

                    N = buffer[header_index + 2]
                    total_len = 1 + 1 + 1 + N*24 + 2 + 1
                    if len(buffer) < header_index + total_len:
                        break  # 等待更多数据

                    packet = buffer[header_index:header_index + total_len]
                    parsed = parse_path_packet(packet)
                    if parsed:
                        print(f"\n收到轨迹包，点数: {parsed['num_points']}")
                        for i, pt in enumerate(parsed['points']):
                            print(f"  点 {i+1}: x={pt[0]:.3f}, y={pt[1]:.3f}, z={pt[2]:.3f}")
                        print(f"CRC: {parsed['crc']}")
                    buffer = buffer[header_index + total_len:]

            except Exception as e:
                print(f"接收数据异常: {e}")
                break

if __name__ == "__main__":
    main()
