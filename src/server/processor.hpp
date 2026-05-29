#pragma once

#include <iostream>
#include <mutex>
#include <map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>

#include "../lib/lib.hpp"
#include "../lib/frameQueue.hpp"

using namespace std;

// Ép C++ đóng gói chặt chẽ cấu trúc bộ nhớ phẳng, không chèn byte rác
#pragma pack(push, 1)
struct MoCapBinaryPoint {
    int32_t id;      // 4 bytes
    float x;         // 4 bytes
    float y;         // 4 bytes
    float z;         // 4 bytes
};                   // Tổng cộng: Đúng 16 bytes vật lý chuẩn mã hóa mạng
#pragma pack(pop)

// ============================================================================
// I. LỚP CƠ SỞ TOÁN HỌC: PROCESSOR3D (SVD CHUẨN ĐÉT)
// ============================================================================
class Processor3D {
    private:
        cv::Mat P1, P2;

        cv::Point3d triangulate_dlt_point(const cv::Point2f& pt1, const cv::Point2f& pt2) {
            cv::Mat A(4, 4, CV_64F);
            A.row(0) = pt1.x * P1.row(2) - P1.row(0);
            A.row(1) = pt1.y * P1.row(2) - P1.row(1);
            A.row(2) = pt2.x * P2.row(2) - P2.row(0);
            A.row(3) = pt2.y * P2.row(2) - P2.row(1);

            cv::SVD svd(A, cv::SVD::MODIFY_A);
            cv::Mat X = svd.vt.row(3);

            double w = X.at<double>(0, 3);
            if (std::abs(w) < 1e-5) w = 1e-5;

            return cv::Point3d(X.at<double>(0, 0) / w,
                               X.at<double>(0, 1) / w,
                               X.at<double>(0, 2) / w);
        }

    public:
        Processor3D() {
            init_geometry();
        }

        void init_geometry() {
            cv::Mat K = (cv::Mat_<double>(3,3) << 630.0,   0.0, 320.0,
                                                   0.0, 630.0, 240.0,
                                                   0.0,   0.0,   1.0);

            cv::Mat R1 = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat T1 = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat Rt1;
            cv::hconcat(R1, T1, Rt1);
            P1 = K * Rt1;

            cv::Mat R2 = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat T2 = (cv::Mat_<double>(3,1) << -400.0, 0.0, 0.0);
            cv::Mat Rt2;
            cv::hconcat(R2, T2, Rt2);
            P2 = K * Rt2;
        }

        vector<ProcessedPoint> triangulate(const AlignedFrame& frame) {
            vector<ProcessedPoint> results;
            for (size_t i = 0; i < frame.marker_ids.size(); ++i) {
                int id = frame.marker_ids[i];
                cv::Point2f pt1 = frame.cam1_points[i];
                cv::Point2f pt2 = frame.cam2_points[i];

                cv::Point3d pt3d = triangulate_dlt_point(pt1, pt2);

                ProcessedPoint p_out;
                p_out.id = id;
                p_out.x = pt3d.x;
                p_out.y = pt3d.y;
                p_out.z = pt3d.z;
                results.push_back(p_out);
            }
            return results;
        }
};

// ============================================================================
// II. LỚP ĐIỀU PHỐI LUỒNG MẠNG: PROCESSORFUNCTOR (BẮN MULTIPART ĐỒNG BỘ PYTHON)
// ============================================================================
class ProcessorFunctor {
    private:
        bool* isRunning;
        int sendPort;
        string sendIP;
        bool needsUpdate;
        mutex configMtx;

        zmq::context_t* zmq_ctx;
        zmq::socket_t* zmq_pub;

    public:
        ProcessorFunctor(ProcessorConfig& cfg) {
            isRunning = cfg.isRunning;
            sendPort = cfg.port;
            sendIP = cfg.ip;
            needsUpdate = false;

            zmq_ctx = new zmq::context_t(1);
            zmq_pub = new zmq::socket_t(*zmq_ctx, zmq::socket_type::pub);
            string address = "tcp://" + sendIP + ":" + to_string(sendPort);
            zmq_pub->bind(address);
        }

        ~ProcessorFunctor() {
            zmq_pub->close();
            delete zmq_pub;
            delete zmq_ctx;
        }

        bool operator()(DataQueue<AlignedFrame>& queue) {
            Processor3D triangulator;

            while (*isRunning) {
                AlignedFrame frame;

                if (needsUpdate) {
                    lock_guard<mutex> lock(configMtx);
                    zmq_pub->disconnect("tcp://" + sendIP + ":" + to_string(sendPort));
                    string new_address = "tcp://" + sendIP + ":" + to_string(sendPort);
                    zmq_pub->bind(new_address);
                    needsUpdate = false;
                }

                if (queue.pop(frame)) {
                    // 1. Giải toán tái tạo hình học 3D ra vector tọa độ thực
                    vector<ProcessedPoint> points_3d = triangulator.triangulate(frame);

                    // 2. DUYỆT QUA TỪNG POINT ĐỂ BẮN MULTIPART CHUẨN BINARY SANG PYTHON
                    for (const auto& p : points_3d) {

                        // PHẦN 1: Bắn Topic Byte (Khớp chuẩn MOCAP_CHANNEL = b'\x01' của Python)
                        uint8_t topic_byte = 0x01;
                        zmq::message_t msg_topic(sizeof(topic_byte));
                        memcpy(msg_topic.data(), &topic_byte, sizeof(topic_byte));

                        // Bật cờ ZMQ_SNDMORE báo cho mạng biết: "Còn phần Payload nhị phân đi sau"
                        zmq_pub->send(msg_topic, zmq::send_flags::sndmore);

                        // PHẦN 2: Đóng gói đúng Struct nhị phân phẳng 16 bytes
                        MoCapBinaryPoint bin_pt;
                        bin_pt.id = static_cast<int32_t>(p.id);
                        bin_pt.x  = static_cast<float>(p.x);
                        bin_pt.y  = static_cast<float>(p.y);
                        bin_pt.z  = static_cast<float>(p.z);

                        zmq::message_t msg_payload(sizeof(MoCapBinaryPoint));
                        memcpy(msg_payload.data(), &bin_pt, sizeof(MoCapBinaryPoint));

                        // Gửi nốt phần Payload, kết thúc một gói Multipart tin nhắn mạng
                        zmq_pub->send(msg_payload, zmq::send_flags::none);
                    }
                }
            }
            return true;
        }

        void changeConnection(ProcessorConfig& prs_cfg) {
            lock_guard<mutex> lock(configMtx);
            sendIP = prs_cfg.ip;
            sendPort = prs_cfg.port;
            needsUpdate = true;
        }
};