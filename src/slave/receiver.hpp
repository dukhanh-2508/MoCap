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

#include "../lib/frameQueue.hpp"
#include "../lib/lib.hpp"

using namespace std;

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