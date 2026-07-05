#pragma once

#include <opencv2/opencv.hpp>
#include <map>
#include <cstdint>

#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/lib_slave.hpp"
#include "../../lib/header/module.hpp"

using namespace std;
using namespace cv;

// Output data queue (O) is set to a dummy 'int' since this module
// pushes its results directly to the ontroller via outNoti queue.
template <typename I>
class ProcessImgModule : public IModule<I, int> {
    private:
        int slave_id = 0; // Needed to pack into CameraPacketHeader
        
        // Tuning parameters for CV algorithm
        int thresh_value = 245;
        int max_disappeared = 30;
        float tracking_dist_threshold = 50.0f;
        double minArea = 50.0;
        double minCircularity = 0.6;
        uint8_t next_id = 0;
        
        // Memory for tracking
        map<uint8_t, Point2f> tracked_markers;
        map<uint8_t, int> disappeared_frames;

        // Internal CV pipeline function
        vector<CenterPacket> extractAndTrack(const Mat& frame);

    public:
        ProcessImgModule();
        ~ProcessImgModule();

        void setConfig(int s_id);
        void setState(int newState) override;
        void runModule() override;
        void setConfig(string paramName, float value);
};

template <typename I>
void ProcessImgModule<I>::setConfig(string paramName, float value) {
    if (paramName == "ID") {
        this->slave_id = (int)value;
    } else if (paramName == "THRESH") {
        this->thresh_value = (int)value;
    } else if (paramName == "MAX_DIS") {
        this->max_disappeared = (int)value;
    } else if (paramName == "TRACK_DIST") {
        this->tracking_dist_threshold = value;
    } else if (paramName == "AREA") {
        this->minArea = value;
    } else if (paramName == "CIR") {
        this->minCircularity = value;
    }
}

template <typename I>
ProcessImgModule<I>::ProcessImgModule() : IModule<I, int>(20) {
    this->moduleState = PROCESS_IDLE;
}

template <typename I>
ProcessImgModule<I>::~ProcessImgModule() {}

template <typename I>
void ProcessImgModule<I>::setConfig(int s_id) {
    this->slave_id = s_id;
}

template <typename I>
void ProcessImgModule<I>::setState(int newState) {
    this->moduleState = newState;
    
    // Clear tracking history if system switches back to IDLE
    if (newState == PROCESS_IDLE) {
        tracked_markers.clear();
        disappeared_frames.clear();
        next_id = 0;
        printf("[PROCESS] Tracking history cleared.\n");
    }
}

template <typename I>
vector<CenterPacket> ProcessImgModule<I>::extractAndTrack(const Mat& frame) {
    Mat gray, blurred, thresh;
    
    // Pre-processing
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blurred, Size(7, 7), 0);
    threshold(blurred, thresh, thresh_value, 255, THRESH_BINARY);

    // Contour Extraction
    vector<vector<Point>> contours;    
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    printf("\n[PROCESSOR] Number of contours found: %d\n", contours.size());

    vector<Point2f> current_centers;
    
    for (const auto& c : contours) {
        double area = contourArea(c);
        if (area < this->minArea) {
            printf("[PROCESSOR] Too small: %f < %f\n", area, this->minArea);
            continue; // Filter small noise
        }

        Point2f center;
        float radius;
        minEnclosingCircle(c, center, radius);
        
        double circularity = area / (M_PI * radius * radius);
        if (circularity < this->minCircularity) {
            printf("[PROCESSOR] Not round enough: %f < %f\n", circularity, this->minCircularity);
            continue; // Filter non-circular blobs
        }

        Moments m = moments(c);
        if (m.m00 == 0) continue;

        float cX = static_cast<float>(m.m10 / m.m00);
        float cY = static_cast<float>(m.m01 / m.m00);
        current_centers.push_back(Point2f(cX, cY));
    }
    printf("[PROCESSOR] Amount of centers found: %d\n", current_centers.size());

    // ID Tracking Logic
    vector<CenterPacket> output_markers;
    set<int> matched_indices;

    for (auto& track : tracked_markers) {
        uint8_t track_id = track.first;
        Point2f track_pos = track.second;
        
        float min_dist = 99999.0f;
        int best_idx = -1;

        // Find the nearest new center for this existing track ID
        for (size_t i = 0; i < current_centers.size(); ++i) {
            if (matched_indices.count(i)) continue;
            
            float dist = sqrt(pow(current_centers[i].x - track_pos.x, 2) +
                              pow(current_centers[i].y - track_pos.y, 2));
            if (dist < min_dist) {
                min_dist = dist;
                best_idx = i;
            }
        }

        // Check if the nearest center is within the threshold limit
        if (best_idx != -1 && min_dist < tracking_dist_threshold) {
            tracked_markers[track_id] = current_centers[best_idx];
            disappeared_frames[track_id] = 0; // Reset missing counter
            matched_indices.insert(best_idx);
            
            output_markers.push_back(CenterPacket{
                track_id,
                current_centers[best_idx].x,
                current_centers[best_idx].y
            });
        } else {
            // Marker lost in this frame
            disappeared_frames[track_id]++;
        }
    }

    // Clean up lost tracks
    for (auto it = disappeared_frames.begin(); it != disappeared_frames.end();) {
        if (it->second > max_disappeared) {
            tracked_markers.erase(it->first);
            it = disappeared_frames.erase(it);
        } else {
            ++it;
        }
    }

    // Register new tracks for unmatched centers
    for (size_t i = 0; i < current_centers.size(); ++i) {
        if (matched_indices.count(i)) continue;
        
        uint8_t new_id = next_id++;
        tracked_markers[new_id] = current_centers[i];
        disappeared_frames[new_id] = 0;
        
        output_markers.push_back(CenterPacket{
            new_id,
            current_centers[i].x,
            current_centers[i].y
        });
    }

    // Debug, print out markers
    for (auto it : output_markers) {
        printf("Detected marker: %d (id) - (%f, %f)\n", it.object_id, it.x, it.y);
    }

    return output_markers;
}

template <typename I>
void ProcessImgModule<I>::runModule() {
    while (this->moduleState == PROCESS_IDLE || this->moduleState == PROCESS_RUNNING) {
        // Wait and pull frame from Capture Module
        I inputData = this->inputData->pop();
        
        // Skip processing if system is not in TRACKING mode
        if (this->moduleState != PROCESS_RUNNING) continue;

        FramePacket* packet = (FramePacket*)&inputData;
        if (packet->frame.empty()) continue;

        // Extract 2D coords and maintain IDs
        vector<CenterPacket> tracked_centers = extractAndTrack(packet->frame);

        // Pack data into standard MoCap struct
        CameraPacketHeader header = {
            packet->capture_time_us,
            packet->frame_id,
            (uint8_t)this->slave_id,
            (uint8_t)tracked_centers.size()
        };

        CameraPacket camPacket = {
            header,
            tracked_centers
        };

        // Notify Controller FSM and push results to FSM
        SystemNotification noti;
        noti.origin = PROCESS_BLOCK;
        
        ProcessNotiPayload payload = {PROCESS_TRACKING_DONE, camPacket};
        noti.payload = payload;

        this->outNoti->push(noti);
    }
}

