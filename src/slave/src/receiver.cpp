#include <fcntl.h>
#include <iostream>

#include "receiver.hpp"

using namespace std;

ReceiverFunctor::ReceiverFunctor(ReceiverConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->receiveIP = cfg.rcv_ip;
    this->receivePort = cfg.rcv_port;
    this->sock = -1;

    this->needsUpdate = false;
}

void ReceiverFunctor::changeSocket(ReceiverConfig& rcv_cfg) {
    std::lock_guard<std::mutex> lock(configMtx);
    
    this->receivePort = rcv_cfg.rcv_port;
    this->receiveIP = rcv_cfg.rcv_ip;
    this->needsUpdate = true;
}

bool ReceiverFunctor::createSocket() {
    int new_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (new_sock < 0) {
        cerr << "[SLAVE RECEIVER] Error: Failed to create socket" << endl;
        return false;
    }

    int broadcastEnable = 1;
    if (setsockopt(new_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        cerr << "[SLAVE RECEIVER] Error: Failed to set socket option (SO_BROADCAST)" << endl;
        close(new_sock);
        return false;
    }

    int reuseAddrEnable = 1;
    if (setsockopt(new_sock, SOL_SOCKET, SO_REUSEADDR, &reuseAddrEnable, sizeof(reuseAddrEnable)) < 0) {
        cerr << "[SLAVE RECEIVER] Failed to config reuse socket" << endl;
    }

    struct sockaddr_in recv_addr;
    memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(this->receivePort);
    
    if (inet_pton(AF_INET, this->receiveIP.c_str(), &recv_addr.sin_addr) <= 0) {
        cerr << "[SLAVE RECEIVER] Invalid IP. Default to INADDR_ANY" << endl;
        recv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    
    if (bind(new_sock, (const struct sockaddr*) &recv_addr, sizeof(recv_addr)) < 0) {
        cerr << "[SLAVE RECEIVER] Error: Failed to bind socket" << endl;
        close(new_sock);
        return false;
    }
    fcntl(new_sock, F_SETFL, O_NONBLOCK);

    cout << "[SLAVE RECEIVER] Socket is listening for broadcasts on port " << this->receivePort << " with IP " << this->receiveIP << endl;

    this->sock = new_sock;

    return true;
}

bool ReceiverFunctor::operator()(DataQueue<FutureTriggerPacket>& queue) {
    createSocket();

    while (*(this->isRunning)) {
        if (this->needsUpdate) {
            close(this->sock);
            createSocket();
            this->needsUpdate = false;
        }

        FutureTriggerPacket packet;
        int bytes = recvfrom(this->sock, &packet, sizeof(packet), 0, NULL, NULL);

        if (bytes < 0) {
            if (!(errno == EAGAIN || errno == EWOULDBLOCK)) {
                cerr << "[SLAVE RECEIVER] Unable to receive packet. Connection Error" << endl;
            }
            this_thread::sleep_for(chrono::microseconds(100));
        } else {
            queue.push(move(packet));
        }
    }

    close(this->sock);
    cout << "[SLAVE RECEIVER] Socket closed" << endl;

    return true;
}