#include <mutex>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>

#include "receiver.hpp"

void IngestionBuffer::add_packet(const CameraPacket& pkt) {
    lock_guard<mutex> lock(mtx);

    if (pkt.header.frame_id <= last_dispatched_frame && last_dispatched_frame > 0) {
        cout << "[RECEIVER] Package of camera " << (int)pkt.header.camera_id 
                << " of frame " << pkt.header.frame_id << " came late\n";
        return;
    }

    if (buffer.find(pkt.header.frame_id) == buffer.end()) {
        buffer[pkt.header.frame_id] = AlignedFrame{
            pkt.header.frame_id, 
            chrono::steady_clock::now(), 
            {}
        };
    }

    buffer[pkt.header.frame_id].camera_data[pkt.header.camera_id] = pkt;

    if (buffer[pkt.header.frame_id].camera_data.size() == TOTAL_CAMERAS) {
        dispatch_frame(pkt.header.frame_id);
    }
}

void IngestionBuffer::check_timeouts() {
    lock_guard<mutex> lock(mtx);
    auto now = chrono::steady_clock::now();

    for (auto it = buffer.begin(); it != buffer.end(); ) {
        auto duration = chrono::duration_cast<chrono::milliseconds>(now - it->second.creation_time).count();
                
        if (duration > TIMEOUT_MS) {
            it = buffer.erase(it);
        } else {
            it++;
        }
    }
}

void IngestionBuffer::dispatch_frame(uint32_t frame_id) {
    auto& frame = buffer[frame_id];

    commQueue.push(move(frame));
    buffer.erase(frame_id);

    last_dispatched_frame = max(last_dispatched_frame, frame_id);
}

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
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        cerr << "[RECEIVER] Error: Failed to create socket" << endl;
        return false;
    }

    int broadcastEnable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0) {
        cerr << "[RECEIVER] Error: Failed to set socket option (SO_BROADCAST)" << endl;
        return false;
    }

    int reuseAddrEnable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuseAddrEnable, sizeof(reuseAddrEnable)) < 0) {
        cerr << "[RECEIVER] Failed to config reuse socket" << endl;
    }

    struct sockaddr_in recv_addr;
    memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(this->receivePort);
    
    if (inet_pton(AF_INET, this->receiveIP.c_str(), &recv_addr.sin_addr) <= 0) {
        cerr << "[RECEIVER] Invalid IP. Default to INADDR_ANY" << endl;
        recv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    if (bind(sock, (const struct sockaddr*) &recv_addr, sizeof(recv_addr)) < 0) {
        cerr << "[RECEIVER] Error: Failed to bind socket" << endl;
        return false;
    }
    fcntl(sock, F_SETFL, O_NONBLOCK);

    cout << "[RECEIVER] Socket is listening for broadcasts on port " << this->receivePort << endl;

    this->sock = sock;

    return true;
}

bool ReceiverFunctor::operator()(DataQueue<AlignedFrame>& queue) {
    IngestionBuffer align_buffer(queue);

    createSocket();

    CameraPacket pkt;
    uint8_t rawBuffer[1500];

    while (*(this->isRunning)) {
        if (this->needsUpdate) {
            close(sock);
            createSocket();
            this->needsUpdate = false;
        }

        int bytes_read = recvfrom(sock, rawBuffer, sizeof(rawBuffer), 0, NULL, NULL);
        
        if (bytes_read > 0) {
            CameraPacketHeader* pktHeader = reinterpret_cast<CameraPacketHeader*>(rawBuffer);
            int expectedSize = sizeof(CameraPacketHeader) + sizeof(CenterPacket) * pktHeader->centerCount;
            if (expectedSize == bytes_read) {
                pkt.header.captureTime = pktHeader->captureTime;
                pkt.header.frame_id = pktHeader->frame_id;
                pkt.header.camera_id = pktHeader->camera_id;

                CenterPacket* arr = reinterpret_cast<CenterPacket*>(rawBuffer + sizeof(CameraPacketHeader));

                pkt.centers.clear();
                pkt.centers.assign(arr, arr + pktHeader->centerCount);
                
                align_buffer.add_packet(pkt);
            }
        }

        align_buffer.check_timeouts();
        this_thread::sleep_for(chrono::microseconds(100));
    }

    close(sock);
    return true;
}