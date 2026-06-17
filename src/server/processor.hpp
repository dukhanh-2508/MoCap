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

        vector<ProcessedPoint> triangulate(const AlignedFrame& frame);
};


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
        ProcessorFunctor(ProcessorConfig& prs_cfg);

        ~ProcessorFunctor();

        void changeConnection(ProcessorConfig& prs_cfg);

        bool operator()(DataQueue<AlignedFrame>& queue);
};