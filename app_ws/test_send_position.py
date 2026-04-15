import socket
import struct

HOST = "127.0.0.1"
PORT = 8080

def parse_position_packet(packet):

    if packet[0] != 0xF5 or packet[-1] != 0x5F:
        print("Header or tail error")
        return

    func_code = packet[1]
    if func_code != 0x07:
        print(f"Ignored packet func_code={func_code}")
        return

    # 解析小端 double: X,Y,Z
    x = struct.unpack_from('<d', packet, 2)[0]
    y = struct.unpack_from('<d', packet, 10)[0]
    z = struct.unpack_from('<d', packet, 18)[0]

    crc_low = packet[26]
    crc_high = packet[27]

    print(f"Received Position (double):")
    print(f"    X = {x:.6f}")
    print(f"    Y = {y:.6f}")
    print(f"    Z = {z:.6f}")
    print(f"CRC: ({crc_low}, {crc_high})")
    print("-------------------------------------")


def start_client():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    print(f"Connected to {HOST}:{PORT}, waiting for position packets...")

    buffer = bytearray()

    while True:
        data = sock.recv(1024)
        if not data:
            print("Disconnected.")
            break
        
        buffer.extend(data)

        # 解析 29 字节固定包
        while len(buffer) >= 29:
            if buffer[0] != 0xF5:
                buffer.pop(0)
                continue

            packet = buffer[:29]
            parse_position_packet(packet)

            buffer = buffer[29:]  # 移除已解析的数据

    sock.close()


if __name__ == "__main__":
    start_client()
