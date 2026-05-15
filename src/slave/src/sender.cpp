#include <unistd.h>
#include <spdlog/spdlog.h>

#include "../sender.hpp"

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
    
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 100000; 
    setsockopt(new_sock, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    memset(&(this->boardcast_addr), 0, sizeof(this->boardcast_addr));
    (this->boardcast_addr).sin_family = AF_INET;
    (this->boardcast_addr).sin_port = htons(this->sendPort);
    
    if (inet_pton(AF_INET, this->sendIP.c_str(), &(this->boardcast_addr).sin_addr) <= 0) {
        cerr << "[SLAVE SENDER] Invalid IP. Default to INADDR_BROADCAST" << endl;
        (this->boardcast_addr).sin_addr.s_addr = htonl(INADDR_BROADCAST);
        this->sendIP = "255.255.255.255";
    }

    this->sock = new_sock;

    cout << "[SLAVE SENDER] Started. Sending data..." << endl;

    return true;
}

bool ResultSenderFunctor::operator()(DataQueue<CameraPacket>& resultQueue) {
    createSocket();
    int frame_count = 0;
    auto last_fps_time = std::chrono::steady_clock::now();

    while (((this->isRunning != NULL) && *(this->isRunning))) {
        if (this->needsUpdate) {
            close(this->sock);
            createSocket();
            this->needsUpdate = false;
        }

        CameraPacket packet;
        if (!resultQueue.pop(packet)) {
            break;
        }

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
        } else {
            spdlog::info("[SLAVE SENDER] Packet sent");
            frame_count++;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();

        if (elapsed_ms >= 1000) {
            spdlog::info("[SLAVE SENDER] Sent FPS: {}", frame_count);
            
            frame_count = 0;
            last_fps_time = now;
        }
    }

    close(this->sock);
    return true;
}