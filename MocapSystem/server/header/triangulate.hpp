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
        string calibResultFolder;

        map<int, Mat> camera_projections;
        map<int, Mat> camera_K_matrices; 
        map<int, Mat> camera_D_coeffs;

        unordered_map<uint32_t, AlignedFrame> alignment_buffer;
        unordered_map<int, string> availableCamID; // key is camera id and string is ip of the slave camera with that id
        uint32_t last_dispatched_frame = 0;
        int TOTAL_CAMERAS = 2;
        ofstream out_log; // ofstream - Output File Stream ==> Used to write logs to a log file

        void check_timeouts();
        void process_aligned_frame(const AlignedFrame& frame);
        Point3d triangulate_multi_view(const vector<CameraObservation>& observations);
    public:
        TriangulateModule();
        ~TriangulateModule();
        
        void assignCalibResultFolder(string resultFolder);
        void loadCalibrationData(const string& calibFolder);
        void register_camera(int cam_id, const Mat& K, const Mat& D, const Mat& R, const Mat& T);
        void setState(int newState) override;
        void runModule() override;
        bool parseNetworkInput(const NetworkOutputResult inputData);
        bool updateAvailableCamID(int oldCamID, int newCamID, string ip); // Called from main, update available ids when the user set a new id or modify an existing id
};

template <typename I>
bool TriangulateModule<I>::updateAvailableCamID(int oldCamID, int newCamID, string ip) {
    int key = -999;
    bool isItemAlreadyAvailable = false;

    if (oldCamID == -999 && !ip.empty()) { // Update ID using only IP
        for (const auto& it : this->availableCamID) {
            if (it.second == ip) {
                key = it.first;   
                isItemAlreadyAvailable = true;
                break; 
            }
        }
    } 
    else if (oldCamID != -999) { // Update using old ID or old ID and IP
        auto findID = this->availableCamID.find(oldCamID);
        if (findID != this->availableCamID.end()) {
            key = findID->first;
            isItemAlreadyAvailable = true;
        }
    }

    if (isItemAlreadyAvailable) {
        this->availableCamID.erase(key);
        this->availableCamID[newCamID] = ip;
    } else {
        this->availableCamID[newCamID] = ip;
    }

    return true;
}

template <typename I>
void TriangulateModule<I>::assignCalibResultFolder(string resultFolder) {
    this->calibResultFolder = resultFolder;
}

template <typename I>
void TriangulateModule<I>::loadCalibrationData(const string& calibFolder) {
    printf("[TRIANGULATE] Loading calibration data from: %s\n", calibFolder.c_str());
    
    for (auto item : this->availableCamID) {
        int cam_id = item.first;
        Mat K, dist, R, T;
        bool intrinsicLoaded = false;
        bool extrinsicLoaded = false;

        // Read intrinsic
        string inPath = calibFolder + "/intrinsic_result_cam_" + to_string(cam_id) + "/camera_params_cam_" + to_string(cam_id) + ".yml";
        FileStorage fsIn(inPath, FileStorage::READ);
        
        if (fsIn.isOpened()) {
            fsIn["K"] >> K;
            fsIn["dist"] >> dist;
            if (!K.empty() && !dist.empty()) intrinsicLoaded = true;
            fsIn.release();
        } else {
            printf("[TRIANGULATE] Warning: Cannot open Intrinsic file %s\n", inPath.c_str());
        }

        // Read extrinsic
        string exPath = calibFolder + "/extrinsic_result_cam_" + to_string(cam_id) + "/extrinsic_params_cam_" + to_string(cam_id) + ".yml";
        FileStorage fsEx(exPath, FileStorage::READ);
        
        if (fsEx.isOpened()) {
            FileNode root = fsEx.root();
            
            for (auto it = root.begin(); it != root.end(); ++it) {
                FileNode item = *it;
                if (item.type() == FileNode::MAP && !item["R_Matrix"].empty() && !item["T_Vector"].empty()) {
                    item["R_Matrix"] >> R;
                    item["T_Vector"] >> T;
                    extrinsicLoaded = true;
                    
                    break; 
                }
            }
            fsEx.release();
        } else {
            printf("[TRIANGULATE] Warning: Cannot open Extrinsic file %s\n", exPath.c_str());
        }

        if (intrinsicLoaded && extrinsicLoaded) {
            register_camera(cam_id, K, dist, R, T);
            printf("[TRIANGULATE] Successfully loaded full params for Camera %d\n", cam_id);
        } else {
            printf("[TRIANGULATE] Error: Incomplete calibration data for Camera %d\n", cam_id);
        }
    }
}

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
void TriangulateModule<I>::register_camera(int cam_id, const Mat& K, const Mat& D, const Mat& R, const Mat& T) {
    camera_K_matrices[cam_id] = K.clone();
    camera_D_coeffs[cam_id] = D.clone();
    
    Mat Rt;
    hconcat(R, T, Rt);
    camera_projections[cam_id] = K * Rt;
}

template <typename I>
void TriangulateModule<I>::setState(int newState) {
    if (newState == TRIANGULATE_RUNNING && this->moduleState != TRIANGULATE_RUNNING) {
        if (!this->calibResultFolder.empty()) {
            loadCalibrationData(this->calibResultFolder); 
        } else {
            printf("[TRIANGULATE] Calib result folder not found. Failed to load calib result");
        }
    }

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

        // Undistort the images
        vector<Point2f> raw_pts, ideal_pts;
        vector<int> obj_ids;
        
        for (const auto& center : pkt.centers) {
            raw_pts.push_back(Point2f(center.x, center.y));
            obj_ids.push_back(center.object_id);
        }

        if (!raw_pts.empty() && camera_K_matrices.count(cam_id) && camera_D_coeffs.count(cam_id)) {
            // Deal with distortion
            undistortPoints(raw_pts, ideal_pts, 
                                camera_K_matrices[cam_id], 
                                camera_D_coeffs[cam_id], 
                                cv::noArray(), 
                                camera_K_matrices[cam_id]);
                                
            for (size_t i = 0; i < ideal_pts.size(); ++i) {
                int forced_id = 0;
                frame_data_3d[forced_id].push_back( // Replace obj_ids[i] by forced_id
                    CameraObservation{cam_id, ideal_pts[i]}
                );
            }
        } else {
            for (const auto& center : pkt.centers) {
                int forced_id = 0;
                frame_data_3d[forced_id].push_back( // Replace center.object_id by forced_id
                    CameraObservation{cam_id, Point2f(center.x, center.y)}
                );
                printf("[TRIANGULATION] Observation added: %d (obj id) - %d (cam id) - (%f, %f)\n", center.object_id, cam_id, center.x, center.y);
            }
        }
    }

    out_log << "========== Frame: " << frame.frame_id << " ==========" << endl;
    for (const auto& item : frame_data_3d) {
        int marker_id = item.first;
        const vector<CameraObservation>& obs = item.second;
        printf("[TRIANGULATION] Processing observation...\n");
        if (obs.size() >= 2) {
            Point3d pt3d = triangulate_multi_view(obs);
            
            out_log << "[ID: " << marker_id << "] -> " 
                    << "X: " << pt3d.x << " | "
                    << "Y: " << pt3d.y << " | "
                    << "Z: " << pt3d.z << " (cm)" << endl;
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
