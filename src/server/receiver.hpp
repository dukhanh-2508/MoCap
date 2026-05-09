#pragma once

#include <iostream>

#include "frameQueue.hpp"
#include "lib.hpp"

using namespace std;

class IngestionBuffer {
    private:
        DataQueue<AlignedFrame>& commQueue; 
        unordered_map<uint32_t, AlignedFrame> buffer;
        mutex mtx;
        uint32_t last_dispatched_frame = 0;
        const int TOTAL_CAMERAS = 4;
        const int TIMEOUT_MS = 15;
    public:
        IngestionBuffer(DataQueue<AlignedFrame>& queue) : commQueue(queue) {}
        void add_packet(const CameraPacket& pkt);
        void check_timeouts();
    private:
        void dispatch_frame(uint32_t frame_id);
};

class ReceiverFunctor {
    private:
        bool* isRunning;
        int receivePort;
        string receiveIP;
        bool needsUpdate;
        mutex configMtx;

        int sock;

        bool createSocket();

    public:
        ReceiverFunctor(ReceiverConfig& cfg);

        bool operator()(DataQueue<AlignedFrame>& queue);

        void changeSocket(ReceiverConfig& rcv_cfg);
};