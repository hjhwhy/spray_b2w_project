import socket

def send_command(cmd_type):
    packet = bytearray()
    packet.append(0xF5)               # header
    packet.append(0x08)               # func_code
    packet.extend([0x01, 0x00])       # data_len = 1 byte (little-endian)
    packet.extend(cmd_type.to_bytes(1, 'little'))  # 1-byte instruction type
    packet.extend([0x00, 0x00])       # CRC ignored
    packet.append(0x5F)               # tail

    print("SEND:", packet.hex())

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('127.0.0.1', 8080))
    s.send(packet)
    s.close()


# 测试发送指令：例如发送 0x05（后退）
send_command(0x05)

# 指令表（1字节）
# 0x01 开始
# 0x02 暂停
# 0x03 结束
# 0x04 前进
# 0x05 后退
# 0x06 左移
# 0x07 右移
# 0x08 左转
# 0x09 右转
