#pragma once

#include <iostream>
#include <mutex>

#include "lib.hpp"
#include "frameQueue.hpp"

using namespace std;

#pragma pack(push, 1)
typedef struct {
    int32_t m_id;
    float x;       
    float y;    
    float z;
} ProcessedPoint;
#pragma pack(pop)

class Processor3D {
    private:
        DataQueue<AlignedFrame>& commQueue;

        void triangulate(const AlignedFrame& frame);
    public:
        Processor3D(DataQueue<AlignedFrame>& queue) : commQueue (queue) {};
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
