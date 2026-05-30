#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

/**Định nghĩa Gói tin mạng (SlaveDataMessage): Định nghĩa một cấu trúc dữ liệu chuẩn để Slave đóng gói tọa độ 2D gửi đi,
 và Server dựa vào đúng cấu trúc đó để bóc tách dữ liệu ra xử lý.

Cung cấp Ma trận dùng chung (MocapConfig): Cả file sinh ảnh giả lập sil_camera_simulator.hpp (phía Slave) và file tính
toán processor_3d.hpp (phía Server) đều cần chung một bộ thông số hình học máy ảnh thì toán học giải ngược mới chính xác
. Việc gom ma trận về file này giúp bạn đổi cấu hình góc quay/khoảng cách camera một nơi là toàn hệ thống tự cập nhật.

Đóng gói truyền dữ liệu lên giao diện (MocapUtils): Cung cấp hàm chuyển đổi mảng số thực tọa độ 3D thành một chuỗi ký tự
 gọn nhẹ để đẩy qua cổng ZeroMQ (Cơ chế Pub-Sub) giúp cho giao diện Python PyQt6 đọc lên vẽ hình mượt mà nhất.*/
// --- 1. ĐỊNH NGHĨA CẤU TRÚC GÓI TIN MẠNG (MESSAGES) ---

// Gói tin Tọa độ 2D từ Slave gửi về Server qua UDP Socket
struct SlaveDataMessage {
    int cam_id;              // ID của camera gửi (1 hoặc 2)
    uint32_t frame_id;       // Thứ tự khung hình để Server đồng bộ
    double timestamp;        // Mốc thời gian chụp (ms)
    int marker_count;        // Số lượng marker tìm thấy

    // Danh sách tọa độ (X tương đương u, Y tương đương v) và ID của từng marker
    std::vector<int> marker_ids;
    std::vector<cv::Point2f> points_2d;
};

// --- 2. CẤU HÌNH THÔNG SỐ HÌNH HỌC CAMERA ẢO (MÔ PHỎNG SIL) ---
namespace MocapConfig {
    // Độ phân giải khung hình chuẩn
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;

    // Ma trận nội thông số K giả lập (Tiêu cự f=630, Tâm quang học cx=320, cy=240)
    inline cv::Mat get_init_K() {
        cv::Mat K = (cv::Mat_<double>(3,3) <<
            630.0,   0.0, 320.0,
              0.0, 630.0, 240.0,
              0.0,   0.0,   1.0);
        return K;
    }

    // Các thông số Ngoại hiệu chuẩn (Extrinsic) giả lập cho hệ thống 2 Camera

    // Camera 1 đặt làm gốc tọa độ thế giới
    inline cv::Mat get_R1() { return cv::Mat::eye(3, 3, CV_64F); }
    inline cv::Mat get_T1() { return cv::Mat::zeros(3, 1, CV_64F); }

    // Camera 2 dịch chuyển ngang 400mm theo trục X và hơi xoay hướng về tâm 5 độ
    inline cv::Mat get_R2() {
        double theta = -5.0 * M_PI / 180.0; // Chuyển sang Radian
        cv::Mat R2 = (cv::Mat_<double>(3,3) <<
            cos(theta),  0.0, sin(theta),
                   0.0,  1.0,        0.0,
           -sin(theta),  0.0, cos(theta));
        return R2;
    }

    inline cv::Mat get_T2() {
        // Vector tịnh tiến t: dịch chuyển -400mm theo trục X
        cv::Mat T2 = (cv::Mat_<double>(3,1) << -400.0, 0.0, 0.0);
        return T2;
    }
}

// --- 3. TIỆN ÍCH TRUYỀN TẢI DỮ LIỆU QUA ZERO MẠNG (ZMQ SERIALIZATION) ---
namespace MocapUtils {
    // Chuyển đổi dữ liệu ma trận điểm 3D thành chuỗi ký tự String để bắn qua ZeroMQ
    inline std::string serialize_3d_points(int frame_id, const std::vector<cv::Point3d>& points, const std::vector<int>& ids) {
        std::stringstream ss;
        // Cấu trúc chuỗi gửi đi: FRAME_ID;ID_1,X_1,Y_1,Z_1;ID_2,X_2,Y_2,Z_2;...
        ss << frame_id;
        for (size_t i = 0; i < points.size(); ++i) {
            ss << ";" << ids[i] << "," << points[i].x << "," << points[i].y << "," << points[i].z;
        }
        return ss.str();
    }
}