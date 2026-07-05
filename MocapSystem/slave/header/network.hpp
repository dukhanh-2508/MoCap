#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>

#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/module.hpp"
#include "../../lib/header/lib.hpp"

using namespace std;

template <typename I>
class NetworkModule : public IModule<I, CaptureTrigger> {
    private:
        int tcp_client_fd = -1;
        int udp_listen_fd = -1;
        int udp_tx_fd = -1;
        struct sockaddr_in server_udp_addr;
    
        string server_ip = "127.0.0.1";
        int tcp_port = 8080;
        int udp_port = 8081;
        int slave_id = 1;

        atomic<bool> is_running{false};
        thread tcp_rx_thread;
        thread udp_rx_thread;
        mutex tcp_tx_mtx;

        void initNetwork();
        void closeNetwork();
        void tcpRxLoop();
        void udpRxLoop();

        void sendTCPPacket(TCP_PACKET_TYPE type, const void* payload, size_t size);
        void sendFileTCP(const string& filepath);

    public:
        NetworkModule();
        ~NetworkModule();

        void setState(int newState) override;
        void runModule() override;
        void setConfig(int new_id) { this->slave_id = new_id; };
        void setInitialConfig(string ip, int port, int id);
};

template <typename I>
void NetworkModule<I>::setInitialConfig(string ip, int port, int id) {
    this->server_ip = ip;
    this->tcp_port = port;
    this->slave_id = id;
    this->udp_port = port + 1;
}

template <typename I>
NetworkModule<I>::NetworkModule() : IModule<I, CaptureTrigger>(20) {
    this->moduleState = NETWORK_IDLE;
}

template <typename I>
NetworkModule<I>::~NetworkModule() {
    closeNetwork();
}

template <typename I>
void NetworkModule<I>::setState(int newState) {
    this->moduleState = newState;
    if (newState == NETWORK_RUNNING && !is_running) {
        initNetwork();
    } else if (newState == NETWORK_STOP && is_running) {
        closeNetwork();
    }
}

// Socket setup
template <typename I>
void NetworkModule<I>::initNetwork() {
    is_running = true;

    // Setup UDP boardcast listener
    udp_listen_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    setsockopt(udp_listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in udp_addr;
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(udp_port);
    udp_addr.sin_addr.s_addr = INADDR_ANY;

    bind(udp_listen_fd, (struct sockaddr*)&udp_addr, sizeof(udp_addr));

    // Setup TCP client
    tcp_client_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port);
    inet_pton(AF_INET, server_ip.c_str(), &serv_addr.sin_addr);

    if (connect(tcp_client_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("[SLAVE NETWORK] Failed to connect to TCP Server %s:%d\n", server_ip.c_str(), tcp_port);
        is_running = false;
        return;
    }

    printf("[SLAVE NETWORK] Connected to Server %s:%d\n", server_ip.c_str(), tcp_port);

    // Send Handshake packet to register slave_id with server
    sendTCPPacket(PKT_HANDSHAKE, nullptr, 0);

    // Create socket UDP
    udp_tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&server_udp_addr, 0, sizeof(server_udp_addr));
    server_udp_addr.sin_family = AF_INET;
    server_udp_addr.sin_port = htons(tcp_port + 2);
    inet_pton(AF_INET, server_ip.c_str(), &server_udp_addr.sin_addr);

    // Start background threads
    udp_rx_thread = thread(&NetworkModule::udpRxLoop, this);
    tcp_rx_thread = thread(&NetworkModule::tcpRxLoop, this);
}

template <typename I>
void NetworkModule<I>::closeNetwork() {
    is_running = false;
    if (udp_listen_fd >= 0) {
        close(udp_listen_fd);
        udp_listen_fd = -1;
    }
    if (tcp_client_fd >= 0) {
        close(tcp_client_fd);
        tcp_client_fd = -1;
    }
    if (udp_rx_thread.joinable()) udp_rx_thread.join();
    if (tcp_rx_thread.joinable()) tcp_rx_thread.join();
}

// Background receive loop
// UDP Loop: Listen for rapid MoCap synchronization triggers
template <typename I>
void NetworkModule<I>::udpRxLoop() {
    FutureTriggerPacket pkt;
    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    while (is_running) {
        int bytes = recvfrom(udp_listen_fd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&sender_addr, &addr_len);
        if (bytes == sizeof(FutureTriggerPacket)) {
            if (strncmp(pkt.header, "SYNC", 4) == 0) {
                // Construct CaptureTrigger and send directly to Capture Module
                CaptureTrigger trigger;
                trigger.frame_id = pkt.frame_id;
                trigger.target_time_us = pkt.target_time_us;
                trigger.is_manual_req = false;
                
                this->outputData->push(trigger);
            }
        }
    }
}

// TCP Loop: Listen for Server Commands
template <typename I>
void NetworkModule<I>::tcpRxLoop() {
    while (is_running) {
        TCPHeader header;
        int bytes_read = read(tcp_client_fd, &header, sizeof(TCPHeader));
        
        if (bytes_read <= 0) {
            printf("[SLAVE NETWORK] Server disconnected.\n");
            break;
        }

        if (header.type == PKT_CMD_REQ_IMG) {
            // Server wants a manual picture
            printf("[SLAVE NETWORK] Server requested an image capture.\n");
            CaptureTrigger trigger;
            trigger.frame_id = 0;
            trigger.target_time_us = 0;
            trigger.is_manual_req = true; 
            
            this->outputData->push(trigger);
        }
        else if (header.type == PKT_CMD_SET_PARAM) {
            std::vector<char> buffer(header.payload_size);
            int bytes_read = read(tcp_client_fd, buffer.data(), header.payload_size);
            
            if (bytes_read == header.payload_size) {
                string payload_str(buffer.data(), header.payload_size);
                size_t colon_pos = payload_str.find(':');
                
                if (colon_pos != string::npos) {
                    SystemNotification noti;
                    noti.origin = NETWORK;
                    
                    NetworkNotiPayloadSlave p;
                    p.cmdOrigin = NET_SERVER_CMD_CONFIG;
                    p.paramName = payload_str.substr(0, colon_pos);
                    
                    p.paramValue = stof(payload_str.substr(colon_pos + 1));
                    
                    noti.payload = p;
                    this->outNoti->push(noti);
                }
            } else {
                printf("[NETWORK] Error: Incomplete param payload received.\n");
            }
        }
        else if (header.type == PKT_CMD_REQ_INFO) {
            SystemNotification noti;
            noti.origin = NETWORK;
            NetworkNotiPayloadSlave payload;
            payload.cmdOrigin = NET_SERVER_CMD_INFO;
            noti.payload = payload;
            this->outNoti->push(noti);
        }
    }
}

// Send functions
template <typename I>
void NetworkModule<I>::sendTCPPacket(TCP_PACKET_TYPE type, const void* payload, size_t size) {
    lock_guard<mutex> lock(tcp_tx_mtx);
    if (tcp_client_fd < 0) return;

    TCPHeader header = {type, (uint32_t)size, this->slave_id};
    write(tcp_client_fd, &header, sizeof(TCPHeader));
    if (size > 0 && payload != nullptr) {
        write(tcp_client_fd, payload, size);
    }
}

template <typename I>
void NetworkModule<I>::sendFileTCP(const string& filepath) {
    FILE* img_file = fopen(filepath.c_str(), "rb");
    if (!img_file) {
        printf("[SLAVE NETWORK] Failed to open file for sending: %s\n", filepath.c_str());
        return;
    }

    fseek(img_file, 0, SEEK_END);
    long filesize = ftell(img_file);
    fseek(img_file, 0, SEEK_SET);

    lock_guard<mutex> lock(tcp_tx_mtx);
    TCPHeader header = {PKT_IMAGE_DATA, (uint32_t)filesize, this->slave_id};
    write(tcp_client_fd, &header, sizeof(TCPHeader));

    uint8_t buffer[4096];
    long remaining = filesize;
    while (remaining > 0) {
        size_t to_read = (remaining < sizeof(buffer)) ? remaining : sizeof(buffer);
        size_t bytes_read = fread(buffer, 1, to_read, img_file);
        if (bytes_read > 0) {
            write(tcp_client_fd, buffer, bytes_read);
            remaining -= bytes_read;
        } else break;
    }
    fclose(img_file);
    printf("[SLAVE NETWORK] Image file sent to server: %s\n", filepath.c_str());

    remove(filepath.c_str()); // Delete file from RAM
}

template <typename I>
void NetworkModule<I>::runModule() {
    while (this->moduleState == NETWORK_IDLE || this->moduleState == NETWORK_RUNNING) {
        I inputData = this->inputData->pop();
        
        SlaveNetworkInput* netIn = (SlaveNetworkInput*)&inputData;

        if (netIn->type == SLAVE_NET_SEND_INFO) {
            sendTCPPacket(PKT_INFO_RESPONSE, netIn->infoMsg.c_str(), netIn->infoMsg.size());
        } 
        else if (netIn->type == SLAVE_NET_SEND_FILE) {
            sendFileTCP(netIn->filepath);
        } 
        else if (netIn->type == SLAVE_NET_SEND_TRACKING) {
            size_t centers_size = netIn->cameraData.centers.size() * sizeof(CenterPacket);
            size_t total_payload_size = sizeof(CameraPacketHeader) + centers_size;

            vector<uint8_t> udp_buffer(total_payload_size);
            
            memcpy(udp_buffer.data(), &netIn->cameraData.header, sizeof(CameraPacketHeader));
            memcpy(udp_buffer.data() + sizeof(CameraPacketHeader), netIn->cameraData.centers.data(), centers_size);

            sendto(udp_tx_fd, udp_buffer.data(), total_payload_size, 0, 
                (struct sockaddr*)&server_udp_addr, sizeof(server_udp_addr));
        }
    }
}


