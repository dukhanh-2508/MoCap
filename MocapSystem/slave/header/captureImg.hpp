#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

#include "../../lib/header/module.hpp"

using namespace std;
using namespace cv;

template <typename I>
class CaptureImgModule : public IModule<I, FramePacket> {
    private:
        VideoCapture camera;
        
        int frameWidth = 1280;
        int frameHeight = 720;
        int cameraFPS = 30;
        int slave_id = 0;

        bool openCamera();
        void closeCamera();
        string saveFrameToRAM(const Mat& frame, uint32_t frame_id);

    public:
        CaptureImgModule();
        ~CaptureImgModule();

        void setCameraConfig(int width, int height, int fps, int s_id);
        void setState(int newState) override;
        void runModule() override;
};

namespace fs = std::filesystem;

template <typename I>
CaptureImgModule<I>::CaptureImgModule() : IModule<I, FramePacket>(20) {
    this->moduleState = CAPTURE_IDLE;
    
    string ram_dir = "/dev/shm/mocap_frames";
    if (!fs::exists(ram_dir)) {
        fs::create_directory(ram_dir);
    }
}

template <typename I>
CaptureImgModule<I>::~CaptureImgModule() {
    closeCamera();
}

template <typename I>
void CaptureImgModule<I>::setCameraConfig(int width, int height, int fps, int s_id) {
    this->frameWidth = width;
    this->frameHeight = height;
    this->cameraFPS = fps;
    this->slave_id = s_id;
}

template <typename I>
void CaptureImgModule<I>::setState(int newState) {
    this->moduleState = newState;
    if (newState == CAPTURE_RUNNING && !camera.isOpened()) {
        openCamera();
    } else if (newState == CAPTURE_IDLE && camera.isOpened()) {
    } else if (newState == CAPTURE_STOP) {
        closeCamera();
    }
}

// Controll camera HW
template <typename I>
bool CaptureImgModule<I>::openCamera() { // Open camerconfig camera HW parameters 
    camera.open(0, CAP_V4L2);
    
    if (!camera.isOpened()) {
        printf("[CAPTURE] FATAL: Cannot open camera hardware!\n");
        return false;
    }

    camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(CAP_PROP_FRAME_WIDTH, frameWidth);
    camera.set(CAP_PROP_FRAME_HEIGHT, frameHeight);
    camera.set(CAP_PROP_FPS, cameraFPS);
    
    camera.set(CAP_PROP_AUTO_EXPOSURE, 1);
    camera.set(CAP_PROP_AUTO_WB, 0);

    printf("[CAPTURE] Camera opened successfully at %dx%d @ %dfps.\n", frameWidth, frameHeight, cameraFPS);
    return true;
}

template <typename I>
void CaptureImgModule<I>::closeCamera() {
    if (camera.isOpened()) {
        camera.release();
        printf("[CAPTURE] Camera released.\n");
    }
}

template <typename I>
string CaptureImgModule<I>::saveFrameToRAM(const Mat& frame, uint32_t frame_id) {
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "/dev/shm/mocap_frames/frame_%06u_cam_%d.jpg", frame_id, this->slave_id);

    vector<int> compression_params;
    compression_params.push_back(IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100); 

    imwrite(filepath, frame, compression_params);
    return string(filepath);
}

template <typename I>
void CaptureImgModule<I>::runModule() {
    Mat capturedImg;

    while (this->moduleState == CAPTURE_IDLE || this->moduleState == CAPTURE_RUNNING) {
        I inputData = this->inputData->pop();
        CaptureTrigger* trigger = (CaptureTrigger*)&inputData;

        if (!camera.isOpened()) continue;

        uint64_t actual_capture_time = 0;

        // Timing logic
        if (trigger->target_time_us > 0 && !trigger->is_manual_req) {
            auto target_tp = chrono::time_point<chrono::system_clock>(chrono::microseconds(trigger->target_time_us));
            auto now = chrono::system_clock::now();

            if (target_tp > now) {
                auto wait_duration = target_tp - now;

                // Coarse wait: Yield CPU if there's plenty of time (> 2ms)
                if (wait_duration > chrono::milliseconds(2)) {
                    this_thread::sleep_until(target_tp - chrono::milliseconds(1));
                }

                // Precision busy-wait: Poll clock rapidly for the last millisecond
                while (this->moduleState == CAPTURE_RUNNING) {
                    now = chrono::system_clock::now();
                    uint64_t now_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count(); 

                    if (now_us >= trigger->target_time_us) {
                        if (camera.grab()) {
                            actual_capture_time = now_us;
                            camera.retrieve(capturedImg);
                        } else {
                            printf("[CAPTURE] Warning: Failed to grab frame at %lu\n", now_us);
                        }
                        break; // Exit busy-wait
                    }
                }
            } else {
                // Already late, capture immediately
                if (camera.grab()) {
                    actual_capture_time = chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now().time_since_epoch()).count();
                    camera.retrieve(capturedImg);
                    printf("[CAPTURE] Warning: Missed target time. Late by %lu us.\n", actual_capture_time - trigger->target_time_us);
                }
            }
        } 
        else if (trigger->is_manual_req) {
            // Immediate manual capture requested by TCP
            if (camera.read(capturedImg)) {
                actual_capture_time = chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now().time_since_epoch()).count();
            }
        }

        if (capturedImg.empty()) continue;

        // Routing output
        if (trigger->is_manual_req) {
            // Manual mode: Compress to RAM and tell FSM to send it via TCP
            string savedPath = saveFrameToRAM(capturedImg, trigger->frame_id);
            
            SystemNotification noti;
            noti.origin = CAPTURE_BLOCK;
            CaptureNotiPayload payload = {CAPTURE_SAVED_TO_RAM, savedPath};
            noti.payload = payload;
            
            this->outNoti->push(noti);
            printf("[CAPTURE] Manual frame captured and saved to RAM.\n");
        } 
        else {
            // Tracking mode: Push raw Mat directly to process image module
            FramePacket outPacket;
            outPacket.frame_id = trigger->frame_id;
            outPacket.capture_time_us = actual_capture_time;
            
            outPacket.frame = capturedImg.clone(); 
            
            this->outputData->push(outPacket);
        }
    }
}
