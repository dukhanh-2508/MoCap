#pragma once

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
    public:
        MarkerDetectorFunctor(CameraConfig& cfg);

        bool operator()(DataQueue<FutureTriggerPacket>& commandQueue,
                        DataQueue<CameraPacket>& resultQueue);
};