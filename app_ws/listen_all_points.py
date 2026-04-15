import socket
import struct

def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def main():
    host = "127.0.0.1"
    port = 8080

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("Connected.")

    while True:
        head = recv_exact(sock, 1)
        if not head:
            break
        if head[0] != 0xF5:
            continue

        func = recv_exact(sock, 1)[0]

        if func != 0x01:
            # 跳过非 0x01 包（你也可以解析）
            print(f"收到其他功能码: 0x{func:02X}")
            continue

        # 读取点数量（不是 data_len）
        count_bytes = recv_exact(sock, 2)
        N = count_bytes[0] | (count_bytes[1] << 8)

        # 每个点 24 字节
        payload = recv_exact(sock, N * 24)

        # CRC + tail
        crc = recv_exact(sock, 2)
        tail = recv_exact(sock, 1)[0]

        if tail != 0x5F:
            print("包尾错误！收到:", hex(tail))
            continue

        print(f"收到 0x01 包，共 {N} 个点：")

        idx = 0
        for i in range(N):
            x = struct.unpack('<d', payload[idx:idx+8])[0]
            y = struct.unpack('<d', payload[idx+8:idx+16])[0]
            z = struct.unpack('<d', payload[idx+16:idx+24])[0]
            idx += 24

            print(f"Point {i}:  x={x:.3f}, y={y:.3f}, z={z:.3f}")


if __name__ == "__main__":
    main()
