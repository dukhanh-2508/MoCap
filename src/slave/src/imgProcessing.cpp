#include <chrono>
#include <cstring>
#include <thread>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>
#include <cstdio>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include "../imgProcessing.hpp"
#include "../locateMarker.hpp"

namespace fs = std::filesystem;
using namespace std;

MarkerDetectorFunctor::MarkerDetectorFunctor(CameraConfig& cfg, CameraParameters& paras) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->module_id = cfg.glcfg.module_id;
    this->cameraFPS = &(paras.cameraFPS);
    this->frameWidth = &(paras.frameWidth);
    this->frameHeight = &(paras.frameHeight);
}

void saveCompressedFrameToRAM(const cv::Mat& frame, uint32_t frame_id, uint8_t camera_id) {
    // 1. Khai báo đường dẫn đích trên RAM
    const string ram_dir = "/dev/shm/mocap_frames";

    // 2. Tạo thư mục nếu chưa tồn tại (chỉ chạy 1 lần lúc bắt đầu)
    if (!fs::exists(ram_dir)) {
        fs::create_directory(ram_dir);
    }

    // 3. Định dạng tên file đồng nhất để dễ quản lý: frame_00123_cam_1.jpg
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "%s/frame_%06u_cam_%d.jpg", ram_dir.c_str(), frame_id, camera_id);

    // 4. Cấu hình thông số nén JPEG
    // Chất lượng chạy từ 0 - 100. (Mặc định của OpenCV là 95).
    // Với ảnh MoCap (thường chỉ lấy luồng LED sáng hoặc marker), mức 75-80 là đủ rõ và siêu nhẹ.
    vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(100); 

    // 5. Ghi ảnh trực tiếp vào ổ RAM
    cv::imwrite(filepath, frame, compression_params);
}

bool MarkerDetectorFunctor::operator()(DataQueue<FutureTriggerPacket>& commandQueue,
                                        DataQueue<CameraPacket>& resultQueue) {
    MarkerDetector detector;
    cv::Mat capturedImg(*(this->frameWidth), *(this->frameHeight), CV_8UC3);
    cv::VideoCapture camera(0, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        spdlog::error("[SLAVE PROCESSOR] FATAL: Cannot open camera 0!");
        return false; // Thoát luôn luồng nếu không mở được cam
    }
    // Config camera
    camera.set(cv::CAP_PROP_FRAME_WIDTH, *(this->frameWidth));
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, *(this->frameHeight));
    camera.set(cv::CAP_PROP_FPS, *(this->cameraFPS));

    ImgProcessor processor;

    while (((this->isRunning != NULL) && *(this->isRunning))) {
        FutureTriggerPacket packet;
        if (commandQueue.pop(packet)) {
            spdlog::info("[SLAVE PROCESSOR] Received command, preparing");
            char command[4];
            memcpy(command, packet.header, 4);
            uint32_t frame_id = packet.frame_id;
            uint64_t targetTime = packet.target_time_us;
            uint64_t captureTime = 0;

            auto target_tp = chrono::time_point<chrono::system_clock>(chrono::microseconds(targetTime));
            auto now = chrono::system_clock::now();

            if (target_tp > now) {
                spdlog::info("[SLAVE PROCESSOR] Timing");
                auto wait_duration = target_tp - now;

                // Sleep when there's plenty of time before targetTime
                if (wait_duration > chrono::milliseconds(2)) {
                    this_thread::sleep_until(target_tp - chrono::milliseconds(1));
                }

                // Continously poll time when close to targetTime
                while (((this->isRunning != NULL) && *(this->isRunning))) {
                    now = chrono::system_clock::now();
                    uint64_t now_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count(); 

                    if (now_us >= targetTime) { // Capture image
                        if(camera.grab()) {
                            captureTime = now_us;
                            spdlog::info("[SLAVE PROCESSOR] Set to capture at {}\nCapture at {}", targetTime, now_us);

                            camera.retrieve(capturedImg);
                        } else {
                            spdlog::error("[SLAVE PROCESSOR] Failed to capture image");
                            continue;
                        }

                        break;
                    }
                }
            } else { // Capture image
                if(camera.grab()) {
                    now = chrono::system_clock::now();
                    captureTime = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count();
                    spdlog::info("[SLAVE PROCESSOR] Set to capture at {}\nCapture at {}", targetTime, captureTime);

                    camera.retrieve(capturedImg);
                } else {
                    spdlog::error("[SLAVE PROCESSOR] Failed to capture image");
                    continue;
                }
            }

            saveCompressedFrameToRAM(capturedImg, packet.frame_id, this->module_id);


            if (!*(this->isRunning)) break;
            /*
            vector<CenterPacket> centers = processor.process_frame(capturedImg);

            // Dummy data gen
            */
            /*
            const uint8_t NUM_MARKERS = 3;        
            for (uint8_t i = 0; i < NUM_MARKERS; i++) {
                CenterPacket pt;
                pt.object_id = i;
                
                float base_x = 20.0f; 
                float base_y = 20.0f;
                float speed = 0.1f; 
                
                // Dùng module_id làm độ lệch pha để các Cam chạy so le nhau
                // Thay vì cộng dồn tọa độ làm tràn giới hạn < 40
                float phase = this->module_id * 1.5f; 

                if (i == 0) {
                    // Marker 0: Quay tròn quanh tâm
                    // Bán kính 15 -> x, y sẽ dao động từ 5 đến 35 (luôn < 40)
                    pt.x = base_x + 15.0f * cos(frame_id * speed + phase);
                    pt.y = base_y + 15.0f * sin(frame_id * speed + phase);
                } 
                else if (i == 1) {
                    // Marker 1: Chạy qua lại theo chiều ngang
                    // Biên độ 15 -> x dao động từ 5 đến 35 (luôn < 40)
                    pt.x = base_x + 15.0f * sin(frame_id * speed * 0.5f + phase);
                    pt.y = base_y - 10.0f; // Cố định y ở mức 10
                } 
                else {
                    // Marker 2: Chạy lên xuống theo chiều dọc
                    // Biên độ 15 -> y dao động từ 5 đến 35 (luôn < 40)
                    pt.x = base_x - 10.0f; // Cố định x ở mức 10
                    pt.y = base_y + 15.0f * cos(frame_id * speed * 0.5f + phase);
                }
                
                centers.push_back(pt);
            }
            */
            /*
            CameraPacketHeader header = {
                captureTime,
                frame_id,
                this->module_id,
                static_cast<uint8_t>(centers.size())
            };

            CameraPacket camPacket = {
                header,
                centers
            };
            
            resultQueue.push(move(camPacket));

            spdlog::info("[SLAVE PROCESSOR] Pushed data to sender: cap time {} - fid {} - mid {}", header.captureTime, header.frame_id, header.camera_id);
        } else {
            this_thread::sleep_for(chrono::microseconds(100));
        }
        */
            this_thread::sleep_for(chrono::microseconds(100));
        }        
    }
    return true;
}