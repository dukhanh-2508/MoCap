#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>
#include <string>

#include "frameQueue.hpp"
#include "lib.hpp"

using namespace std;

#pragma pack(push, 1)
struct FutureTriggerPacket {
    char header[4];          
    uint32_t frame_id;          
    uint64_t target_time_us;
};
#pragma pack(pop)

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

        bool operator()(DataQueue<FutureTriggerPacket>& queue);

        void changeSocket(ReceiverConfig& rcv_cfg);
};