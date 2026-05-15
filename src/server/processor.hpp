#pragma once

#include <iostream>
#include <mutex>

#include "../lib/lib.hpp"
#include "../lib/frameQueue.hpp"

using namespace std;

class Processor3D {
    public:
        Processor3D() = default;
        
        // Hàm nhận frame 2D và trả ra các điểm 3D
        vector<ProcessedPoint> triangulate(const AlignedFrame& frame);
};

class ProcessorFunctor {
    private:
        bool* isRunning;
        int sendPort;
        string sendIP;
        bool needsUpdate;
        mutex configMtx;

    public:
        ProcessorFunctor(ProcessorConfig& cfg);

        bool operator()(DataQueue<AlignedFrame>& queue);

        void changeConnection(ProcessorConfig& prs_cfg);
};
