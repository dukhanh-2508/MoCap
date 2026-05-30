#pragma once
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>

/** 1. Quản lý hình học hệ thống (Stereo Projection)Hàm set_projection_matrices làm nhiệm vụ kết hợp thông số quang học
của ống kính (Ma trận nội $K$) với vị trí lắp đặt phần cứng của 2 chiếc Raspberry Pi trong phòng Lab (Ma trận xoay $R$,
vector tịnh tiến $T$).
2. Giao hội tia sáng tính tọa độ thực (Triangulation via DLT)
Một camera đơn chỉ biết Marker nằm trên một tia sáng nào đó chứ không biết nó cách camera bao xa. Khi có 2 camera cùng
chụp, hàm triangulate_dlt sẽ dựng lại 2 tia sáng này từ góc nhìn của Cam 1 và Cam 2. Giao điểm của 2 đường thẳng đó
trong không gian chính là vị trí thực tế của nắp chai hay Marker
3. Đối chiếu dữ liệu đa kênh (Data Matching)
Hàm reconstruct_scene nhận vào hai luồng dữ liệu riêng biệt từ 2 con Raspberry Pi qua giao thức UDP. Nó tự động kiểm tra
, so khớp các cặp Marker có cùng ID với nhau để thực hiện phép tính, đảm bảo tính đúng đắn dữ liệu trước khi bắn sang
ZeroMQ cho tầng hiển thị PyQt6 vẽ đồ thị.*/

// Struct một Marker sau khi đã được dựng thành tọa độ 3D thực tế
struct Point3D {
    int id;
    cv::Point3d coord;    // Tọa độ không gian 3 chiều thực tế (X, Y, Z)
};

class Processor3D {
private:
    cv::Mat P1; // Ma trận chiếu  của Camera 1 (3X4)
    cv::Mat P2; // Ma trận chiếu  của Camera 2 (3X4)

public:
    Processor3D() {}

    /**
     *  Khởi tạo và tính toán Ma trận chiếu P1, P2 từ thông số hình học hệ thống Stereo
     * Ma trận nội thông số Camera 1 (3x3)
     *  R1 Ma trận xoay Camera 1 (3x3) - Thường là ma trận đơn vị nếu chọn Cam 1 làm gốc
     * T1 Vector tịnh tiến Camera 1 (3x1) - Thường là vector [0, 0, 0] nếu chọn Cam 1 làm gốc
     *  K2 Ma trận nội thông số Camera 2 (3x3)
     *  R2 Ma trận xoay của Camera 2 so với Camera 1 (3x3)
     *  T2 Vector tịnh tiến của Camera 2 so với Camera 1 (3x1)
     */
    void set_projection_matrices(const cv::Mat& K1, const cv::Mat& R1, const cv::Mat& T1,
                                 const cv::Mat& K2, const cv::Mat& R2, const cv::Mat& T2) {
        // Đảm bảo dữ liệu ma trận đầu vào ở định dạng số thực Double (CV_64F)
        cv::Mat K1_64, R1_64, T1_64, K2_64, R2_64, T2_64;
        K1.convertTo(K1_64, CV_64F); R1.convertTo(R1_64, CV_64F); T1.convertTo(T1_64, CV_64F);
        K2.convertTo(K2_64, CV_64F); R2.convertTo(R2_64, CV_64F); T2.convertTo(T2_64, CV_64F);

        // Tạo ma trận [R1 | T1] kích thước 3x4 cho Camera 1
        cv::Mat Rt1 = cv::Mat::eye(3, 4, CV_64F);
        R1_64.copyTo(Rt1(cv::Rect(0, 0, 3, 3)));
        T1_64.copyTo(Rt1(cv::Rect(3, 0, 1, 3)));
        P1 = K1_64 * Rt1; // P1 = K1 * [R1 | T1]

        // Tạo ma trận [R2 | T2] kích thước 3x4 cho Camera 2
        cv::Mat Rt2(3, 4, CV_64F);
        R2_64.copyTo(Rt2(cv::Rect(0, 0, 3, 3)));
        T2_64.copyTo(Rt2(cv::Rect(3, 0, 1, 3)));
        P2 = K2_64 * Rt2; // P2 = K2 * [R2 | T2]
    }

    /**
     * Giải thuật toán DLT sử dụng phân rã SVD để tính tọa độ 3D từ 1 cặp điểm ảnh 2D
     * pt1 Tọa độ (u1, v1) của Marker trên ảnh Camera 1
     *  pt2 Tọa độ (u2, v2) của Marker trên ảnh Camera 2
     */
    cv::Point3d triangulate_dlt(cv::Point2f pt1, cv::Point2f pt2) {
        // Thiết lập ma trận hệ số A kích thước 4x4 từ phương trình hình học lỗ kim
        cv::Mat A(4, 4, CV_64F);

        // Hàng 0 và 1: Ràng buộc hình học từ góc nhìn Camera 1
        A.row(0) = pt1.x * P1.row(2) - P1.row(0);
        A.row(1) = pt1.y * P1.row(2) - P1.row(1);

        // Hàng 2 và 3: Ràng buộc hình học từ góc nhìn Camera 2
        A.row(2) = pt2.x * P2.row(2) - P2.row(0);
        A.row(3) = pt2.y * P2.row(2) - P2.row(1);

        // Thực hiện giải hệ phương trình tuyến tính thuần nhất A * X = 0 bằng phân rã SVD
        cv::Mat w, u, vt;
        cv::SVD::compute(A, w, u, vt);

        // Nghiệm tối ưu nằm ở hàng cuối cùng của ma trận Vt (dạng tọa độ đồng nhất [X, Y, Z, W])
        double W = vt.at<double>(3, 3);
        double X = vt.at<double>(3, 0) / W;
        double Y = vt.at<double>(3, 1) / W;
        double Z = vt.at<double>(3, 2) / W;

        return cv::Point3d(X, Y, Z);
    }

    /**
     * Hàm xử lý hàng loạt: Đối chiếu ID và tái tạo toàn bộ không gian 3D của các Marker
     * cam1_data Map chứa dữ liệu của Cam 1: Key là ID, Value là tọa độ 2D (u, v)
     * cam2_data Map chứa dữ liệu của Cam 2: Key là ID, Value là tọa độ 2D (u, v)
     */
    std::vector<Point3D> reconstruct_scene(const std::map<int, cv::Point2f>& cam1_data,
                                           const std::map<int, cv::Point2f>& cam2_data) {
        std::vector<Point3D> markers_3d;

        // Vòng lặp quét qua toàn bộ Marker của Cam 1
        for (const auto& item : cam1_data) {
            int marker_id = item.first;

            // Nếu Cam 2 cũng tìm thấy một Marker có trùng ID (Nhờ bộ logic đồng bộ)
            if (cam2_data.count(marker_id)) {
                // Tiến hành giao hội tia sáng để dựng tọa độ 3D thực tế
                cv::Point3d coord_3d = triangulate_dlt(item.second, cam2_data.at(marker_id));
                markers_3d.push_back({marker_id, coord_3d});
            }
        }
        return markers_3d;
    }
};