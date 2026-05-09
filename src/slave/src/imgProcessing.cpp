#include <chrono>
#include <cstring>
#include <thread>

#include "imgProcessing.hpp"

MarkerDetectorFunctor::MarkerDetectorFunctor(CameraConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->module_id = cfg.glcfg.module_id;
}

bool MarkerDetectorFunctor::operator()(DataQueue<FutureTriggerPacket>& commandQueue,
                                        DataQueue<CameraPacket>& resultQueue) {
    MarkerDetector detector;

    while (*(this->isRunning)) {
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

                while (*(this->isRunning)) {
                    now = chrono::system_clock::now();
                    uint64_t now_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count(); 

                    if (now_us >= targetTime) {
                        captureTime = now_us;
                        break;
                    }
                }
            } else {
                now = chrono::system_clock::now();
                captureTime = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch()).count();
            }

            if (!*(this->isRunning)) break;

            vector<CenterPacket> centers;

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
        } else {
            this_thread::sleep_for(chrono::microseconds(100));
        }
    }

    return true;
}