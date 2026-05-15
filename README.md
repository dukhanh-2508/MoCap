Code phần network nằm ở trong dir src.
Không có code phần đồng bộ (SD chrony để đồng bộ NTP cho các máy).

Hệ thống phần mềm hiện có ba phần chính: Server, Slave, Hiển thị:
 - Server: là phần máy chủ kiểm soát việc đồng bộ các slave camera, nhận dữ liệu
   xử lý 2D từ các slave camera và thực hiện việc tái tạo tọa độ 3D. Server
   giao tiếp với phần hiển thị để hiển thị ra các marker 3D.
   Trong server được chia thành 3 phần:
	- Orchestrator: người điều phối, tập trung vào việc gửi các gói tin hẹn chụp
	  cho các slave. Phần này gửi cho slave camera các gói tin qua socket.
	- Receiver: nhận các gói tin từ slave camera qua socket và đẩy xuống 
	  cho phần tái tạo 3D ở dưới qua một queue.
	- Processor: Nhận dữ liệu từ Receiver bằng queue, tái tạo 3D và đẩy
	  qua cho mảng hiển thị dùng ZeroMQ.
 - Slave: là các slave camera. Có 3 phần:
	- Receiver: nhận lệnh từ orchestrator và đẩy lệnh xuống cho tầng xử lý
	  việc chụp và xử lý ảnh bên dưới.
	- Img Processing: nhận lệnh từ receiver, schedule lệnh chụp và xử lý tìm
	  tâm marker và theo dõi ID. Kết quả được đẩy cho tầng dưới Sender qua queue.
	- Sender: Gửi dữ liệu về cho Receiver của Server qua socket.
 - Hiển thị: giao tiếp với Processor của Server dùng ZeroMQ (cơ chế pub-sub)
   và hiển thị marker 3D. UI được tạo dùng PyQt6 và OpenGL.
   Flow chính của app:
	- Chọn data source, nếu chọn real-time thì phải nhập IP và port vào 
	  để kết nối. Nên dùng 127.0.0.1 và 5556.
	- Tạo marker, nhớ id gán khi tạo marker cần trùng với ID vật mà marker
	  đại diện.
	- Ấn play để chạy.

(*) Để compile chương trình C++: dùng CMake.
Để build server:
 - Vào src/buildServer. Có thể clear folder này trước để build cho gọn.
 - Gọi "cmake ../cmake/server" để nhận file CMakeLists.txt
 - Gọi "make"
Build slave cũng tương tự.

(*) Một số thư viện ngoài được dùng:
 - ZMQ.
 - spdlog (hiện đang dùng cái này để log, nhưng không ổn lắm)
 - Thư viện CLI11.hpp (trong src/lib) để làm app command line cho server và slave.

(*) Một số vấn đề hiện tại:
 - Log in ra quá nhanh, làm ko tương tác được với app command line của server
   và slave, mà tắt đi thì không theo dõi được. spdlog có chức năng giúp giới hạn
   số lần in ra của log, nhưng chưa đủ thông minh để xử lý trường hợp các msg
   chỉ khác nhau ít (VD giữa các log in giá trị).
   Có thể không in log ra nữa, mà lưu vào một file nào đó và chỉ in thông tin
   khi có lệnh.
 - Chưa tích hợp thuật toán tái tạo 3D và phần chụp ảnh và tìm tâm, track ID  marker.



