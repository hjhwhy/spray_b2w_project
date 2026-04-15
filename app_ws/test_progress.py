#!/usr/bin/env python3
import socket
import time

HOST = "127.0.0.1"
PORT = 8080

def main():
    print(f"Connecting to {HOST}:{PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.connect((HOST, PORT))
        print("Connected! Waiting for data...\n")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # 持续接收 remote_control_node 的 sendProgress() 输出
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print("Server closed connection.")
                break

            print("Received (hex):", data.hex())
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("Read error:", e)
            break

    sock.close()


if __name__ == "__main__":
    main()
