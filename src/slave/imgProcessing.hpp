#pragma once
 //Khai báo thêm OpenCv để giả lập
#include <opencv2/opencv.hpp> // Thêm OpenCV để xử lý ảnh C++

#include <arpa/inet.h>
#include <vector>
#include <cstdint>

#include "../lib/frameQueue.hpp"
#include "receiver.hpp"

using namespace std;

class MarkerDetector {
    public:
};

class MarkerDetectorFunctor {
private:
    bool* isRunning;
    uint8_t module_id;

    // --- KHAI BÁO THÊM CHO LUỒNG MÔ PHỎNG SIL ---
    uint32_t local_frame_counter = 0;
    // Hàm giả lập tạo đốm sáng trắng của Marker dựa trên hình học Cam ảo
    cv::Mat generate_sil_frame(uint32_t frame_id);

public:
    MarkerDetectorFunctor(CameraConfig& cfg);

    bool operator()(DataQueue<FutureTriggerPacket>& commandQueue,
                    DataQueue<CameraPacket>& resultQueue);
};