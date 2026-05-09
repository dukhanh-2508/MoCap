import socket
import time
import json
import sys

UDP_IP = "127.0.0.1"
UDP_PORT = sys.argv[1:] if len(sys.argv) == 3 else [5050, 5050]

"""
AF_INET: option for address family, which is a way to define the type of address structure.
It specifies how to interpret network addresses.
AF_INET ==> use IPv4, network layer
SOCK_DIAGRAM ==> Socket Diagram, a type of socket based on UDP, transport layer, decide whether to use UDP or TCP.

UDP_IP 127.0.0.1: loopback address (localhost), points to the device running the code.
UDP_PORT 5005: free port not preoccupied by the system.


"""

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("--- HỆ THỐNG MÁY CHỦ ĐIỀU PHỐI ---")
print("Nhấn phím [ENTER] để phát lệnh chụp ảnh đồng loạt. Nhấn Ctrl+C để thoát.\n")

while True:
    input() # Đợi người dùng nhấn Enter
    
    # Lấy giờ hiện tại và hẹn lịch 3 giây sau
    target_time = time.time() + 3.0 
    
    message = {
        "command": "capture",
        "target_timestamp": target_time
    }
    
    # Bắn gói tin UDP đi
    for port in UDP_PORT:
        sock.sendto(json.dumps(message).encode('utf-8'), (UDP_IP, int(port)))
    print(f"[Server] Đã phát lệnh! Hẹn giờ kích hoạt: {target_time:.4f}")