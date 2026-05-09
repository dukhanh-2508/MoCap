#include <unistd.h>
#include "sender.hpp"

ResultSenderFunctor::ResultSenderFunctor(SenderConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->sendIP = cfg.snd_ip;
    this->sendPort = cfg.snd_port;
    this->sock = -1;
    memset(&(this->boardcast_addr), 0, sizeof(this->boardcast_addr));

    this->needsUpdate = false;
}

void ResultSenderFunctor::changeSocket(SenderConfig& snd_cfg) {
    std::lock_guard<std::mutex> lock(configMtx);
    
    this->sendPort = snd_cfg.snd_port;
    this->sendIP = snd_cfg.snd_ip;
    this->needsUpdate = true;
}

bool ResultSenderFunctor::createSocket() {
    int new_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (new_sock < 0) {
        cerr << "[SLAVE SENDER] Error: Failed to create socket" << endl;
        return false;
    }

    int broadcastEnable = 1;
    if (setsockopt(new_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        cerr << "[SLAVE SENDER] Failed to config boardcast" << endl;
        close(new_sock);
        return false;
    }    

    memset(&(this->boardcast_addr), 0, sizeof(this->boardcast_addr));
    (this->boardcast_addr).sin_family = AF_INET;
    (this->boardcast_addr).sin_port = htons(this->sendPort);
    
    if (inet_pton(AF_INET, this->sendIP.c_str(), &(this->boardcast_addr).sin_addr) <= 0) {
        cerr << "[SLAVE SENDER] Invalid IP. Default to INADDR_BROADCAST" << endl;
        (this->boardcast_addr).sin_addr.s_addr = htonl(INADDR_BROADCAST);
    }

    this->sock = new_sock;

    cout << "[SLAVE SENDER] Started. Sending data..." << endl;

    return true;
}

bool ResultSenderFunctor::operator()(DataQueue<CameraPacket>& resultQueue) {
    createSocket();

    while (*(this->isRunning)) {
        if (this->needsUpdate) {
            close(this->sock);
            createSocket();
            this->needsUpdate = false;
        }

        CameraPacket packet;
        resultQueue.pop(packet);

        uint8_t buffer[1024];
        int offset = 0;

        memcpy(buffer + offset, &(packet.header), sizeof(CameraPacketHeader));
        offset += sizeof(CameraPacketHeader);

        for (const auto& center : packet.centers) {
            if (offset + sizeof(CenterPacket) > sizeof(buffer)) {
                break;
            }
            memcpy(buffer + offset, &center, sizeof(CenterPacket));
            offset += sizeof(CenterPacket);
        }

        if (sendto(this->sock, buffer, offset, 0, 
                  (struct sockaddr*)&(this->boardcast_addr), sizeof(this->boardcast_addr)) < 0) {
            cerr << "[SLAVE SENDER] Warning: Unable to send data!" << endl;
        }
    }

    close(this->sock);
    return true;
}