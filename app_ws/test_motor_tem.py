import socket
import struct

SERVER_IP = "127.0.0.1"  # 如果服务端在远程修改为对应 IP
SERVER_PORT = 8080       # 和 RemoteControlNode 中的 listen_port 保持一致

def parse_packet(packet):
    """
    按协议解析包头 0xF5, 功能码, 温度值, CRC, 包尾 0x5F
    功能码 0x05 表示 motors_temperature
    """
    if len(packet) != 6:
        return None
    if packet[0] != 0xF5 or packet[1] != 0x05 or packet[5] != 0x5F:
        return None
    temp_value = packet[2]
    return temp_value

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
        s.connect((SERVER_IP, SERVER_PORT))
        print("Connected. Waiting for temperature data...")
        buffer = bytearray()
        while True:
            data = s.recv(1024)
            if not data:
                print("Server disconnected.")
                break
            buffer.extend(data)
            # 尝试解析每个完整包
            while len(buffer) >= 6:
                # 找到包头 0xF5
                if buffer[0] != 0xF5:
                    buffer.pop(0)
                    continue
                packet = buffer[:6]
                temp = parse_packet(packet)
                if temp is not None:
                    print(f"Max Motor Temperature: {temp}")
                # 移除已解析包
                buffer = buffer[6:]

if __name__ == "__main__":
    main()
