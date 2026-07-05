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
        
        int frameWidth = 640;
        int frameHeight = 480;
        int cameraFPS = 15;
        float brightness = -999.0f;
        float gain = -999.0f;
        float exposure = -999.0f;
        int slave_id = 0;

        string saveFrameToRAM(const Mat& frame, uint32_t frame_id);
        

    public:
        CaptureImgModule();
        ~CaptureImgModule();

        void setCameraConfig(int width, int height, int fps, int s_id);
        void setState(int newState) override;
        void runModule() override;
        void setConfig(string paramName, float value);
        bool openCamera();
        void closeCamera();
};

namespace fs = std::filesystem;

template <typename I>
void CaptureImgModule<I>::setConfig(string paramName, float value) {
    if (paramName == "ID") this->slave_id = (int)value;
    else if (paramName == "BRIGHTNESS") { this->brightness = value; if (camera.isOpened()) camera.set(CAP_PROP_BRIGHTNESS, value); }
    else if (paramName == "GAIN") { this->gain = value; if (camera.isOpened()) camera.set(CAP_PROP_GAIN, value); }
    else if (paramName == "EXPOSURE") { this->exposure = value; if (camera.isOpened()) camera.set(CAP_PROP_EXPOSURE, value); }
    else if (paramName == "RESW") { 
        this->frameWidth = value; 
        if (camera.isOpened()) {
            camera.set(CAP_PROP_FRAME_WIDTH, value);
            printf("[CAPTURE] Updated %s to %f\n", paramName, value);
        }
    }
    else if (paramName == "RESH") { 
        this->frameWidth = value; 
        if (camera.isOpened()) {
            camera.set(CAP_PROP_FRAME_HEIGHT, value);
            printf("[CAPTURE] Updated %s to %f\n", paramName, value);
        }
    }
    else if (paramName == "FPS") { this->cameraFPS = value; if (camera.isOpened()) camera.set(CAP_PROP_FPS, value); }
    
}


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

    camera.set(CAP_PROP_BUFFERSIZE, 1);

    camera.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(CAP_PROP_FRAME_WIDTH, frameWidth);
    camera.set(CAP_PROP_FRAME_HEIGHT, frameHeight);
    camera.set(CAP_PROP_FPS, cameraFPS);
    
    camera.set(CAP_PROP_AUTO_EXPOSURE, 1);
    camera.set(CAP_PROP_AUTO_WB, 0);
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
        /*
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
                    uint64_t current_time_us = chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now().time_since_epoch()).count();
                    uint64_t latency = current_time_us - trigger->target_time_us;

                    // If late by over 50ms, this timing trigger command is useless ==> Skip
                    if (latency > 50000) { 
                        printf("[CAPTURE] Dropped frame %u to relieve queue. Too late by %lu us.\n", trigger->frame_id, latency);
                        continue;
                    }

                    actual_capture_time = chrono::duration_cast<chrono::microseconds>(chrono::system_clock::now().time_since_epoch()).count();
                    camera.retrieve(capturedImg);
                    printf("[CAPTURE] Warning: Missed target time. Late by %lu us.\n", actual_capture_time - trigger->target_time_us);
                }
            }
        } 
        */
        if (trigger->target_time_us > 0 && !trigger->is_manual_req) {
                auto target_tp = chrono::time_point<chrono::system_clock>(chrono::microseconds(trigger->target_time_us));
                auto now = chrono::system_clock::now();

                if (target_tp > now) {
                    auto wait_duration = target_tp - now;
                    if (wait_duration > chrono::milliseconds(2)) {
                        this_thread::sleep_until(target_tp - chrono::milliseconds(1));
                    }
                    while (this->moduleState == CAPTURE_RUNNING) {
                        now = chrono::system_clock::now();
                        uint64_t now_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count(); 

                        if (now_us >= trigger->target_time_us) {
                            auto startShot = chrono::system_clock::now();
                            if (camera.grab()) {
                                auto endGrab = chrono::system_clock::now();
                                actual_capture_time = now_us;
                                camera.retrieve(capturedImg);
                                auto endShot = chrono::system_clock::now();
                                // printf("[CAPTURE] Camera shot takes (end grab - end shot): %u - %lu (us)\n", chrono::duration_cast<chrono::microseconds>(endGrab - startShot).count(), chrono::duration_cast<chrono::microseconds>(endShot - startShot).count());
                                // printf("[CAPTURE] Captured tracking image in time: %lu - delay: %lu\n", now_us, now_us - trigger->target_time_us);
                            }
                            break;
                        }
                    }
                } else {
                    uint64_t current_time_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count();
                    uint64_t latency = current_time_us - trigger->target_time_us;

                    if (latency > 50000) { 
                        // printf("[CAPTURE] Dropped frame %u to relieve queue. Too late by %lu us.\n", trigger->frame_id, latency);
                        continue;
                    } 
                    else {
                        auto startShot = chrono::system_clock::now();
                        if (camera.grab()) {
                            auto endGrab = chrono::system_clock::now();
                            actual_capture_time = current_time_us;
                            camera.retrieve(capturedImg);
                            auto endShot = chrono::system_clock::now();
                            // printf("[CAPTURE] Camera shot takes (end grab - end shot): %u - %lu (us)\n", chrono::duration_cast<chrono::microseconds>(endGrab - startShot).count(), chrono::duration_cast<chrono::microseconds>(endShot - startShot).count());
                            // printf("[CAPTURE] Warning: Missed target time. Late by %lu us.\n", latency);
                        }
                    }
                }
            }
        else if (trigger->is_manual_req) {
            for (int i = 0; i < 5; i++) {
                camera.grab();
            }
            
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
