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
};

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
            char* param_buf = new char[header.payload_size + 1];
            read(tcp_client_fd, param_buf, header.payload_size);
            param_buf[header.payload_size] = '\0';
            
            string param_str(param_buf);
            delete[] param_buf;

            // Example Parsing "TOGGLE:1.0" or "ID:2"
            size_t delim = param_str.find(':');
            if (delim != string::npos) {
                string key = param_str.substr(0, delim);
                float val = stof(param_str.substr(delim + 1));

                SystemNotification noti;
                noti.origin = NETWORK;
                NetworkNotiPayloadSlave payload;

                if (key == "TOGGLE") {
                    payload.cmdOrigin = NET_SERVER_CMD_TRACKING;
                    payload.toggle = (val > 0.0f) ? "on" : "off";
                } else if (key == "ID") {
                    payload.cmdOrigin = NET_SERVER_CMD_CONFIG;
                    payload.slave_id = (int)val;
                    this->slave_id = (int)val; // Update internally
                }
                
                noti.payload = payload;
                this->outNoti->push(noti);
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
            // Construct payload: Header + Centers array
            size_t centers_size = netIn->cameraData.centers.size() * sizeof(CenterPacket);
            size_t total_payload_size = sizeof(CameraPacketHeader) + centers_size;

            lock_guard<mutex> lock(tcp_tx_mtx);
            TCPHeader tcp_head = {PKT_CAMERA_DATA, (uint32_t)total_payload_size, this->slave_id};
            write(tcp_client_fd, &tcp_head, sizeof(TCPHeader));
            
            // Send CameraPacket parts
            write(tcp_client_fd, &netIn->cameraData.header, sizeof(CameraPacketHeader));
            write(tcp_client_fd, netIn->cameraData.centers.data(), centers_size);
        }
    }
}


