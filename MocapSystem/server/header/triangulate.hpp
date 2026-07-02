#pragma once

#include "network.hpp"
#include "../../lib/header/lib.hpp"

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <map>
#include <vector>
#include <chrono>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

template <typename I>
class TriangulateModule : public IModule<I, NetworkOutputResult> {
    private:
        map<int, Mat> camera_projections;
        unordered_map<uint32_t, AlignedFrame> alignment_buffer;
        uint32_t last_dispatched_frame = 0;
        int TOTAL_CAMERAS = 2;
        ofstream out_log; // ofstream - Output File Stream ==> Used to write logs to a log file

        void check_timeouts();
        void process_aligned_frame(const AlignedFrame& frame);
        Point3d triangulate_multi_view(const vector<CameraObservation>& observations);
    public:
        TriangulateModule();
        ~TriangulateModule();
        
        void register_camera(int cam_id, const Mat& K, const Mat& R, const Mat& T);
        void setState(int newState) override;
        void runModule() override;
        bool parseNetworkInput(const NetworkOutputResult inputData);
};


template <typename I>
TriangulateModule<I>::TriangulateModule() : IModule<I, NetworkOutputResult>(20) {
    out_log.open("/tmp/mocap_3d.log", ios::out | ios::trunc);
    
    system("gnome-terminal --title='[MOCAP] 3D Triangulation Output' -- bash -c 'tail -f /tmp/mocap_3d.log' &");
}

template <typename I>
TriangulateModule<I>::~TriangulateModule() {
    if(out_log.is_open()) {
        out_log.close();
    }
}

template <typename I>
void TriangulateModule<I>::register_camera(int cam_id, const Mat& K, const Mat& R, const Mat& T) {
    Mat Rt;
    hconcat(R, T, Rt);
    camera_projections[cam_id] = K * Rt;
}

template <typename I>
void TriangulateModule<I>::setState(int newState) {
    this->moduleState = newState;
}

template <typename I>
bool TriangulateModule<I>::parseNetworkInput(const NetworkOutputResult inputData) {
    if (inputData.origin == TRACKING_COORD) { // Input data must be marker tracking data
        CameraPacket pkt = get<CameraPacket>(inputData.result);
        
        if (pkt.header.frame_id <= last_dispatched_frame && last_dispatched_frame > 0) {
            return true; 
        }

        if (alignment_buffer.find(pkt.header.frame_id) == alignment_buffer.end()) { // Frame with a new ID received, make a new AlignedFrame slot
            alignment_buffer[pkt.header.frame_id] = AlignedFrame{
                pkt.header.frame_id, 
                chrono::steady_clock::now(), 
                {}
            };
        }

        // Put camera packet to the correct camera id in the correct AlignedFrame slot
        alignment_buffer[pkt.header.frame_id].camera_data[pkt.header.camera_id] = pkt;

        if (alignment_buffer[pkt.header.frame_id].camera_data.size() == TOTAL_CAMERAS) {
            process_aligned_frame(alignment_buffer[pkt.header.frame_id]);
            
            last_dispatched_frame = max(last_dispatched_frame, pkt.header.frame_id);
            alignment_buffer.erase(pkt.header.frame_id);
        }

        return true;
    } 
    return false;
}

template <typename I>
void TriangulateModule<I>::check_timeouts() {
    auto now = chrono::steady_clock::now();
    for (auto it = alignment_buffer.begin(); it != alignment_buffer.end(); ) {
        auto duration = chrono::duration_cast<chrono::milliseconds>(now - it->second.creation_time).count();
        if (duration > 100) {
            it = alignment_buffer.erase(it);
        } else {
            it++;
        }
    }
}

template <typename I>
Point3d TriangulateModule<I>::triangulate_multi_view(const vector<CameraObservation>& observations) {
    if (observations.size() < 2) return Point3d(0, 0, 0);

    int N = observations.size();
    Mat A = Mat::zeros(2 * N, 4, CV_64F);

    for (int i = 0; i < N; ++i) {
        int cam_id = observations[i].camera_id;
        double u = observations[i].pt_2d.x;
        double v = observations[i].pt_2d.y;

        if (camera_projections.find(cam_id) == camera_projections.end()) continue;
        Mat P = camera_projections[cam_id];

        A.row(2 * i)     = u * P.row(2) - P.row(0);
        A.row(2 * i + 1) = v * P.row(2) - P.row(1);
    }

    SVD svd(A, SVD::MODIFY_A);
    Mat X = svd.vt.row(3);

    double w = X.at<double>(0, 3);
    if (abs(w) < 1e-5) w = 1e-5;

    return Point3d(X.at<double>(0, 0) / w,
                       X.at<double>(0, 1) / w,
                       X.at<double>(0, 2) / w);
}

template <typename I>
void TriangulateModule<I>::process_aligned_frame(const AlignedFrame& frame) {
    map<int, vector<CameraObservation>> frame_data_3d;
    
    // From AlignedFrame, construct a map with the key as a marker's ID and the value as pairs of camera ID and marker coordinate from that camera
    for (const auto& pair : frame.camera_data) {
        int cam_id = pair.first;
        const CameraPacket& pkt = pair.second;
        
        for (const auto& center : pkt.centers) {
            frame_data_3d[center.object_id].push_back(
                CameraObservation{cam_id, Point2f(center.x, center.y)}
            );
        }
    }

    out_log << "========== Frame: " << frame.frame_id << " ==========" << endl;
    for (const auto& item : frame_data_3d) {
        int marker_id = item.first;
        const vector<CameraObservation>& obs = item.second;
        
        if (obs.size() >= 2) {
            Point3d pt3d = triangulate_multi_view(obs);
            
            out_log << "[ID: " << marker_id << "] -> " 
                    << "X: " << pt3d.x << " | "
                    << "Y: " << pt3d.y << " | "
                    << "Z: " << pt3d.z << " (mm)" << endl;
        }
    }
    out_log << flush;
}

template <typename I>
void TriangulateModule<I>::runModule() {
    while(this->moduleState == TRIANGULATE_IDLE || this->moduleState == TRIANGULATE_RUNNING) {
        I inputData = this->inputData->pop();

        bool dataParsed = true;
        if(this->parseNetworkInput(inputData) == false) {
            printf("[TRIANGULATE] Failed to parse input from Network\n");
            dataParsed = false;
        }

        if (dataParsed == true) {
            this->check_timeouts();
        }
    }
}
