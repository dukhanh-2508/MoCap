#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <cstdint>
#include <opencv2/opencv.hpp>

using namespace std;
// sử dụng thuật toán DLT mở rộng cho nhiều Camera (Generalized Multi-view DLT) để gom toàn bộ ma trận chiếu
//Pi và tọa độ 2D (ui, vi) của N camera vào một hệ phương trình tuyến tính tổng quát duy nhất A.X = 0, sau đó
//giải SVD một lần để tìm ra tọa độ 3D tối ưu nhất.
//Thuật toán này tự động quét xem có bao nhiêu camera nhìn thấy cùng một ID Marker (tối thiểu là 2) để lập ma trận hệ số A có kích thước (2N x 4)
// Struct lưu trữ thông tin điểm ảnh từ các máy trạm Slave gửi về
struct CameraObservation {
    int camera_id;
    cv::Point2f pt_2d;
};

class MultiViewProcessor3D {
private:
    // Lưu trữ ma trận chiếu P của TẤT CẢ camera trong hệ thống (Key: camera_id)
    std::map<int, cv::Mat> camera_projections;

public:
    MultiViewProcessor3D() = default;

    // Hàm đăng ký ma trận chiếu P = K * [R|T] cho từng Camera khi cấu hình hệ thống
    void register_camera(int cam_id, const cv::Mat& K, const cv::Mat& R, const cv::Mat& T) {
        cv::Mat Rt;
        cv::hconcat(R, T, Rt);
        camera_projections[cam_id] = K * Rt;
    }

    // THUẬT TOÁN LÕI: N-View Triangulation qua phương pháp Bình phương tối tiểu SVD
    cv::Point3d triangulate_multi_view(const std::vector<CameraObservation>& observations) {
        // Cần tối thiểu dữ liệu từ 2 góc camera để giao hội tia sáng trong không gian
        if (observations.size() < 2) return cv::Point3d(0, 0, 0);

        int N = observations.size();
        // Mỗi camera đóng góp 2 hàng ràng buộc đường cực, ma trận A có kích thước (2N x 4)
        cv::Mat A = cv::Mat::zeros(2 * N, 4, CV_64F);

        for (int i = 0; i < N; ++i) {
            int cam_id = observations[i].camera_id;
            double u = observations[i].pt_2d.x;
            double v = observations[i].pt_2d.y;

            // Lấy ma trận chiếu P tương ứng của Camera đó
            cv::Mat P = camera_projections[cam_id];

            // Thiết lập hệ phương trình ràng buộc hình học tia sáng
            A.row(2 * i)     = u * P.row(2) - P.row(0);
            A.row(2 * i + 1) = v * P.row(2) - P.row(1);
        }

        // Tiến hành phân rã giá trị đơn lẻ SVD giải hệ thuần nhất
        cv::SVD svd(A, cv::SVD::MODIFY_A);
        cv::Mat X = svd.vt.row(3); // Nghiệm là hàng cuối cùng của ma trận Vt

        double w = X.at<double>(0, 3);
        if (std::abs(w) < 1e-5) w = 1e-5;

        // Trả về tọa độ Euclid thực tế (đơn vị: mm)
        return cv::Point3d(X.at<double>(0, 0) / w,
                           X.at<double>(0, 1) / w,
                           X.at<double>(0, 2) / w);
    }

    // Hàm quét đồng bộ dữ liệu theo ID Marker từ gói mạng đồng bộ n-camera đẩy xuống
    void process_mocap_frame(const std::map<int, std::vector<CameraObservation>>& frame_data,
                             std::map<int, cv::Point3d>& output_3d) {
        // frame_data sắp xếp theo cấu trúc: Key = Marker_ID -> Danh sách các Cam nhìn thấy nó
        for (const auto& item : frame_data) {
            int marker_id = item.first;
            const std::vector<CameraObservation>& obs = item.second;

            if (obs.size() >= 2) { // Chỉ giải nếu có từ 2 camera trở lên bắt trúng tâm
                output_3d[marker_id] = triangulate_multi_view(obs);
            }
        }
    }
};