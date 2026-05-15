#include <chrono>
#include <cstring>
#include <thread>
#include <cmath>
#include <spdlog/spdlog.h>

#include "../imgProcessing.hpp"

MarkerDetectorFunctor::MarkerDetectorFunctor(CameraConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->module_id = cfg.glcfg.module_id;
}

bool MarkerDetectorFunctor::operator()(DataQueue<FutureTriggerPacket>& commandQueue,
                                        DataQueue<CameraPacket>& resultQueue) {
    MarkerDetector detector;

    while (((this->isRunning != NULL) && *(this->isRunning))) {
        FutureTriggerPacket packet;
        if (commandQueue.pop(packet)) {
            char command[4];
            memcpy(command, packet.header, 4);
            uint32_t frame_id = packet.frame_id;
            uint64_t targetTime = packet.target_time_us;
            uint64_t captureTime = 0;

            auto target_tp = chrono::time_point<chrono::system_clock>(chrono::microseconds(targetTime));
            auto now = chrono::system_clock::now();

            if (target_tp > now) {
                auto wait_duration = target_tp - now;

                if (wait_duration > chrono::milliseconds(2)) {
                    this_thread::sleep_until(target_tp - chrono::milliseconds(1));
                }

                while (((this->isRunning != NULL) && *(this->isRunning))) {
                    now = chrono::system_clock::now();
                    uint64_t now_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count(); 

                    if (now_us >= targetTime) {
                        captureTime = now_us;
                        
                        spdlog::info("[SLAVE PROCESSOR] Set to capture at {}\nCapture at {}", targetTime, now_us);

                        break;
                    }
                }
            } else {
                now = chrono::system_clock::now();
                captureTime = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count();
            }

            if (!*(this->isRunning)) break;

            vector<CenterPacket> centers;

            // Dummy data gen
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
    }

    return true;
}