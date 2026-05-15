#include <cstring>
#include <unistd.h>
#include <thread>
#include <spdlog/spdlog.h>

#include "../orchestrator.hpp"
#include "../../lib/log.hpp"

using namespace std;

OrchestratorFunctor::OrchestratorFunctor(OrchestratorConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->orchIP = cfg.orch_ip;
    this->orchPort = cfg.orch_port;
    this->sock = -1;
    memset(&(this->boardcast_addr), 0, sizeof(this->boardcast_addr));

    this->needsUpdate = false;
}

void OrchestratorFunctor::changeSocket(OrchestratorConfig& orch_cfg) {
    std::lock_guard<std::mutex> lock(configMtx);
    
    this->orchPort = orch_cfg.orch_port;
    this->orchIP = orch_cfg.orch_ip;
    this->needsUpdate = true;
}

bool OrchestratorFunctor::createSocket() {
    int new_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (new_sock < 0) {
        cerr << "[ORCHESTRATOR] Error: Failed to create socket" << endl;
        return false;
    }

    int broadcastEnable = 1;
    if (setsockopt(new_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        cerr << "[ORCHESTRATOR] Failed to config boardcast" << endl;
    }
    
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 100000; 
    setsockopt(new_sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    memset(&(this->boardcast_addr), 0, sizeof(this->boardcast_addr));
    (this->boardcast_addr).sin_family = AF_INET;
    (this->boardcast_addr).sin_port = htons(this->orchPort);
    if (inet_pton(AF_INET, this->orchIP.c_str(), &(this->boardcast_addr).sin_addr) <= 0) {
        cerr << "[ORCHESTRATOR] Invalid IP. Default to INADDR_BROADCAST" << endl;
        (this->boardcast_addr).sin_addr.s_addr = htonl(INADDR_BROADCAST);
        this->orchIP = "255.255.255.255";
    }

    this->sock = new_sock;

    cout << "[ORCHESTRATOR] Socket created. Address: " << this->orchIP << " - " << this->orchPort << endl;
    cout << "[ORCHESTRATOR] Started. Sending commands..." << endl;
    
    return true;
}

bool OrchestratorFunctor::operator()() {
    createSocket();

    uint32_t current_frame = 0;
    const uint64_t FUTURE_OFFSET_US = 15000; 
    const float FPS = 1.0/7;
    float loopDelay = 1000 / FPS;
    const chrono::milliseconds loop_delay((int) loopDelay);

    while (((this->isRunning != NULL) && *(this->isRunning))) {
        if (this->needsUpdate) {
            close(sock);
            createSocket();
            this->needsUpdate = false;
        }

        auto now = chrono::system_clock::now();
        uint64_t now_us = chrono::duration_cast<chrono::microseconds>(
                            now.time_since_epoch()).count();

        uint64_t target_time = now_us + FUTURE_OFFSET_US;

        FutureTriggerPacket packet;
        strncpy(packet.header, "TRIG", 4);
        packet.frame_id = current_frame++;
        packet.target_time_us = target_time;

        if (sendto(sock, &packet, sizeof(packet), 0, 
                  (struct sockaddr*)&(this->boardcast_addr), sizeof(this->boardcast_addr)) < 0) {
            // cerr << "[ORCHESTRATOR] Warning: Unable to send command!" << endl;
            // cerr << "[ORCHESTRATOR] Errno: " << errno << " - " << strerror(errno) << endl;
            spdlog::error(
                "[ORCHESTRATOR] Warning: Unable to send command!\nErrno: {} - {}", errno, strerror(errno)
            );
        } else {
            spdlog::info("[ORCHESTRATOR] Set time to {}", target_time);
        }

        this_thread::sleep_for(loop_delay);
    }

    close(sock);
    return true;
}