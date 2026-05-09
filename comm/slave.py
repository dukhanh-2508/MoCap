import socket
import time
import json
import sys

"""
setsockopt(): config properties of a socket.
Level SOL_SOCKET: Socket Option Level, dictates the config to happen at the Socket level, not
other levels like IP or TCP/UDP.
Optname SO_REUSEPORT: Socket Option - Reuse Port: allow multiple programs to use a port.
value  = 1 ==> Turn the config on.

sock.bind(): bind a socket to a port.

recvfrom(): Receive data from the socket. Returns a pair of (bytes, address). Address is the address of the socket
sending the data.
Bufsize 1024 indicates the maximum amount of data to be received at once (in bytes)

sock.bind() binds the socket to an address (IP, PORT)
"""

# Lấy tên node từ dòng lệnh (Ví dụ: python3 slave_node.py Node_1)
node_name = sys.argv[1] if len(sys.argv) > 1 else "Unknown_Node"
node_port = int(sys.argv[2]) if len(sys.argv) > 2 else 5050

UDP_IP = "127.0.0.1" # Chạy local nội bộ
UDP_PORT = node_port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Tuyệt chiêu của Linux: Cho phép nhiều script cùng nghe trên 1 Port
# sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1) 
sock.bind((UDP_IP, UDP_PORT))

print(f"[{node_name}] Đã khởi động! Đang lắng nghe lệnh tại cổng {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)
    msg = json.loads(data.decode('utf-8'))
    
    if msg.get("command") == "capture":
        target_time = msg["target_timestamp"]
        print(f"[{node_name}] Đã nhận lịch hẹn! Chờ đến mốc: {target_time:.4f}")
        
        # VÒNG LẶP NÓNG (Spin-wait) - Giữ CPU thức để canh giờ
        while True:
            now = time.time()
            if now >= target_time:
                # KHOẢNH KHẮC BÓP CÒ (Giả lập lệnh cv2.grab() ở đây)
                capture_time = time.time() 
                break
                
        # Tính toán sai số thuật toán
        error_ms = (capture_time - target_time) * 1000
        print(f"[{node_name}] 📸 ĐÃ CHỤP! (Độ trễ logic: {error_ms:.3f} ms)\n")