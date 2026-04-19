cmake_minimum_required(VERSION 3.10)

# Đặt tên dự án và chỉ định ngôn ngữ là C++
project(Thesis_1 CXX)

# Thiết lập sử dụng chuẩn C++14 (phù hợp và ổn định cho OpenCV 4)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tự động tìm đường dẫn cấu hình của OpenCV trên hệ thống
find_package(OpenCV REQUIRED)

# Tạo file thực thi tên là 'main' từ file mã nguồn trong thư mục source
add_executable(main source/main.cpp)

# Nạp đường dẫn header của OpenCV và thư mục 'include' nội bộ của dự án
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS} include)

# Liên kết file thực thi với các thư viện lõi của OpenCV
target_link_libraries(main ${OpenCV_LIBS})