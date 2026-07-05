#pragma once

#include <iostream>
#include <arpa/inet.h>
#include <mutex>

#include "../lib/lib.hpp"

#define SEND_INFINITE_TIMES -1

class OrchestratorFunctor {
    private:
        bool* isRunning;
        int* timesSendPacket;
        int orchPort;
        string orchIP;
        bool needsUpdate;
        mutex configMtx;
        sockaddr_in boardcast_addr;

        int sock;

        bool createSocket();

    public:
        int currentTimesSendPacket;
        OrchestratorFunctor(OrchestratorConfig& cfg);

        bool operator()();

        void changeSocket(OrchestratorConfig& orch_cfg);
};