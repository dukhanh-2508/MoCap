#pragma once

#include <iostream>
#include <string>
#include <mutex>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "../lib/lib.hpp"
#include "../lib/frameQueue.hpp"

using namespace std;

class ResultSenderFunctor {
    private:
        bool* isRunning;
        int sendPort;
        string sendIP;
        bool needsUpdate;
        mutex configMtx;
        sockaddr_in boardcast_addr;

        int sock;

        bool createSocket();

    public:
        ResultSenderFunctor(SenderConfig& cfg);

        bool operator()(DataQueue<CameraPacket>& resultQueue);

        void changeSocket(SenderConfig& snd_cfg);
};