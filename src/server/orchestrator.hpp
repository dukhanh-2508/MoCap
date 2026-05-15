#pragma once

#include <iostream>
#include <arpa/inet.h>
#include <mutex>

#include "../lib/lib.hpp"

class OrchestratorFunctor {
    private:
        bool* isRunning;
        int orchPort;
        string orchIP;
        bool needsUpdate;
        mutex configMtx;
        sockaddr_in boardcast_addr;

        int sock;

        bool createSocket();

    public:
        OrchestratorFunctor(OrchestratorConfig& cfg);

        bool operator()();

        void changeSocket(OrchestratorConfig& orch_cfg);
};