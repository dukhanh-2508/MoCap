#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>

//File này thay thế cho camera để phục vụ mô phỏng SIL.
class SilCameraSimulator {
private:
    double focal_length = 630.0;
    double cx = 320.0;
    double cy = 240.0;
    double t = 0.0;

public:
    cv::Mat generate_virtual_frame(int cam_id) {
        cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
        t += 0.05; // dịch chuyển tịnh tiến

        // Giả lập 3 Marker thực quay quanh một tâm chung ở khoảng cách Z = 3 mét (3000mm)
        double centerX = 100.0, centerY = 0.0, centerZ = 3000.0;
        double radius = 150.0;

        std::vector<cv::Point3d> markers_3d(3);
        markers_3d[0] = cv::Point3d(centerX + radius * cos(t),             centerY + radius * sin(t),             centerZ);
        markers_3d[1] = cv::Point3d(centerX + radius * cos(t + 2*M_PI/3), centerY + radius * sin(t + 2*M_PI/3), centerZ);
        markers_3d[2] = cv::Point3d(centerX + radius * cos(t + 4*M_PI/3), centerY + radius * sin(t + 4*M_PI/3), centerZ);

        for (const auto& pt3d : markers_3d) {
            double X = pt3d.x;
            double Y = pt3d.y;
            double Z = pt3d.z;

            if (cam_id == 2) {
                X -= 400.0; // Cam 2 đặt cách Cam 1 đoạn d = 400mm theo trục X
            }

            int u = static_cast<int>((focal_length * X / Z) + cx);
            int v = static_cast<int>((focal_length * Y / Z) + cy);

            // Vẽ đốm sáng trắng tượng trưng cho nắp chai phản quang đường kính 15mm thực tế
            if (u >= 0 && u < 640 && v >= 0 && v < 480) {
                cv::circle(frame, cv::Point(u, v), 8, cv::Scalar(255, 255, 255), -1);
            }
        }
        return frame;
    }
};