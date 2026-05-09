#pragma once

#include <arpa/inet.h>
#include <vector>
#include <cstdint>

#include "frameQueue.hpp"
#include "receiver.hpp"

using namespace std;

#pragma pack(push, 1)
typedef struct {
    uint8_t object_id;
    float x;                
    float y; 
} CenterPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint64_t captureTime;
    uint32_t frame_id;       
    uint8_t camera_id;                
    uint8_t centerCount;
} CameraPacketHeader;
#pragma pack(pop)

typedef struct {
    CameraPacketHeader header;
    vector<CenterPacket> centers;
} CameraPacket;

class MarkerDetector {
    public:
};

class MarkerDetectorFunctor {
    private:
        bool* isRunning;
        uint8_t module_id;
    public:
        MarkerDetectorFunctor(CameraConfig& cfg);

        bool operator()(DataQueue<FutureTriggerPacket>& commandQueue,
                        DataQueue<CameraPacket>& resultQueue);
};