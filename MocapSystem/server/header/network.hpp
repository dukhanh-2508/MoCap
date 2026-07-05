#pragma once

#include <variant>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/module.hpp"
#include "../../lib/header/module.hpp"

using namespace std;

/*
TCP: used to send commands, status info
UDP: used to send marker tracking data
*/

template <typename I>
class NetworkModule : public IModule<I, NetworkOutputResult> {
    private:
        int tcp_server_fd = -1; // File descriptor for the TCP server listening socket
        int udp_broadcast_fd = -1; // File descriptor for the UDP broadcast socket
        int udp_port = 8081; // Port used for broadcasting UDP timing triggers
        int tcp_port = 8080; // Port used for listening to incoming TCP connections
        int udp_recv_fd = -1;
        int udp_recv_port = 8082;
        string bind_ip = "10.42.0.1"; // IP address to bind the server to (empty means INADDR_ANY)
        unordered_map<string, int> ip_to_fd;
        
        unordered_map<int, int> slave_sockets; // K: slave_id, V: socket_fd
        unordered_map<int, string> pending_img_paths; // K: slave_id, V: save folder

        string udpTriggerIPBand = "255.255.255.255";
        
        atomic<bool> is_running{false};
        thread rx_thread;
        mutex sock_mtx; // Prevent data races between TX and RX threads

        void initNetwork();
        void closeNetwork();
        void rxLoop();

        void handleTCPClient(int client_fd);
        void sendUDPTrigger(const NetCmdTrigger& data);
        void sendTCPRequest(int slave_id, TCP_PACKET_TYPE type, const void* payload, size_t size);
        

    public:
        NetworkModule();
        ~NetworkModule();

        void setState(int newState) override;
        void runModule() override;
        void processInput(const NetworkInput& input);
        void setudpTriggerIPBand(string band);
};

template <typename I>
void NetworkModule<I>::setudpTriggerIPBand(string band) {
    this->udpTriggerIPBand = band;
}

template <typename I>
NetworkModule<I>::NetworkModule() : IModule<I, NetworkOutputResult>(20) {
    this->moduleState = NETWORK_IDLE;
}

template <typename I>
NetworkModule<I>::~NetworkModule() {
    closeNetwork();
}

template <typename I>
void NetworkModule<I>::setState(int newState) {
    this->moduleState = newState;
    if ((newState == NETWORK_RUNNING || newState == NETWORK_IDLE) && !is_running) {
        initNetwork();
    } else if (newState == NETWORK_STOP && is_running) {
        closeNetwork();
    }
}

// Create socket for TCP/UDP communication
template <typename I>
void NetworkModule<I>::initNetwork() {
    is_running = true;

    // Setup UDP boardcast
    udp_broadcast_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int broadcastEnable = 1;
    setsockopt(udp_broadcast_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    // Setup TCP server
    tcp_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int reuse = 1;
    setsockopt(tcp_server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    // Non-blocking socket để dùng với select()
    // In blocking mode, read() or access() functions called by select() will block this thread if the socket has no data
    // In non-blocking mode, those functions will immediately return if socket has no data
    fcntl(tcp_server_fd, F_SETFL, O_NONBLOCK);

    // Set up UDP socket for mocap tracking data
    udp_recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in recv_addr;
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_addr.s_addr = bind_ip.empty() ? INADDR_ANY : inet_addr(bind_ip.c_str());
    recv_addr.sin_port = htons(udp_recv_port > 0 ? udp_recv_port : 8082);

    if (bind(udp_recv_fd, (struct sockaddr*)&recv_addr, sizeof(recv_addr)) < 0) {
        perror("[NETWORK] Failed to bind UDP Receive Port");
    } else {
        printf("[NETWORK] Listening for UDP Tracking on Port %d\n", udp_recv_port);
    }
    fcntl(udp_recv_fd, F_SETFL, O_NONBLOCK);
    ////////

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port > 0 ? tcp_port : 8080); // Cổng mặc định 8080 nếu chưa config
    
    if (bind_ip == "") serv_addr.sin_addr.s_addr = INADDR_ANY;
    else inet_pton(AF_INET, bind_ip.c_str(), &serv_addr.sin_addr);

    bind(tcp_server_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    listen(tcp_server_fd, 10);


    rx_thread = thread(&NetworkModule::rxLoop, this);
    
    printf("[NETWORK] Server started. TCP Port: %d. Ready for connections.\n", ntohs(serv_addr.sin_port));
}

template <typename I>
void NetworkModule<I>::closeNetwork() {
    is_running = false;
    if (rx_thread.joinable()) rx_thread.join();
    
    if (tcp_server_fd >= 0) close(tcp_server_fd);
    if (udp_broadcast_fd >= 0) close(udp_broadcast_fd);
    if (udp_recv_fd >= 0) { close(udp_recv_fd); udp_recv_fd = -1; }
    
    lock_guard<mutex> lock(sock_mtx);
    for (auto const& [id, fd] : slave_sockets) {
        close(fd);
    }
    slave_sockets.clear();
}

// RX thread to handle TCP incoming requests
template <typename I>
void NetworkModule<I>::rxLoop() {
    // fd_set is a bit array used by select() to keep track of multiple file descriptors (sockets)
    fd_set readfds;
    struct timeval tv; // Max time select() should wait for an event

    while (is_running) {
        FD_ZERO(&readfds); // Clear sockets
        FD_SET(tcp_server_fd, &readfds); // Add the main TCP server socket
        FD_SET(udp_recv_fd, &readfds);
        int max_sd = tcp_server_fd;
        if (udp_recv_fd > max_sd) max_sd = udp_recv_fd;

        sock_mtx.lock();
        for (auto const& [id, fd] : slave_sockets) {
            FD_SET(fd, &readfds);
            if (fd > max_sd) max_sd = fd;
        }
        sock_mtx.unlock();

        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms timeout

        int activity = select(max_sd + 1, &readfds, NULL, NULL, &tv);
        if (activity < 0) continue;

        // Process received TCP packets
        if (FD_ISSET(tcp_server_fd, &readfds)) {
            struct sockaddr_in client_addr;
            socklen_t addrlen = sizeof(client_addr); // Create a new file descriptor from new slave 
            int new_socket = accept(tcp_server_fd, (struct sockaddr*)&client_addr, &addrlen); // Accept new slave connection
            
            if (new_socket >= 0) {
                string client_ip = inet_ntoa(client_addr.sin_addr);
                ip_to_fd[client_ip] = new_socket;
                sock_mtx.lock();
                slave_sockets[new_socket] = new_socket; 
                sock_mtx.unlock();
                printf("[NETWORK] New slave connected. FD: %d\n", new_socket);
            }
        }

        // Process received UDP packets
        if (FD_ISSET(udp_recv_fd, &readfds)) {
            uint8_t buffer[2048];
            struct sockaddr_in slave_addr;
            socklen_t addr_len = sizeof(slave_addr);
            
            int bytes_read = recvfrom(udp_recv_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&slave_addr, &addr_len);
            
            if (bytes_read > 0 && bytes_read >= (int)sizeof(CameraPacketHeader)) {
                CameraPacket pkt;
                memcpy(&pkt.header, buffer, sizeof(CameraPacketHeader));
                
                size_t expected_size = sizeof(CameraPacketHeader) + pkt.header.centerCount * sizeof(CenterPacket);
                if (bytes_read >= expected_size) {
                    pkt.centers.resize(pkt.header.centerCount);
                    memcpy(pkt.centers.data(), buffer + sizeof(CameraPacketHeader), pkt.header.centerCount * sizeof(CenterPacket));
                    
                    NetworkOutputResult outResult;
                    outResult.origin = TRACKING_COORD;
                    outResult.result = pkt;
                    this->outputData->push(outResult);
                }
            }
        }

        sock_mtx.lock();
        vector<int> disconnected_slaves;
        
        for (auto const& [id, fd] : slave_sockets) {
            if (FD_ISSET(fd, &readfds)) {
                // Process data sent back from connected slave
                TCPHeader header;
                int bytes_read = read(fd, &header, sizeof(TCPHeader));
                
                if (bytes_read <= 0) {
                    disconnected_slaves.push_back(id);
                } else {
                    if (header.type == PKT_HANDSHAKE) {
                        printf("[NETWORK] Handshake from Slave ID: %d\n", header.slave_id);
                        // Update the correct slave id
                        int temp_fd = fd;
                        slave_sockets.erase(id);
                        slave_sockets[header.slave_id] = temp_fd;
                    }
                    else if (header.type == PKT_IMAGE_DATA) {
                        // Save image request
                        string folder = pending_img_paths.count(header.slave_id) ? pending_img_paths[header.slave_id] : ".";
                        string filepath = folder + "/img_slave_" + to_string(header.slave_id) + "_" + to_string(chrono::steady_clock::now().time_since_epoch().count()) + ".jpg";
                        
                        FILE* img_file = fopen(filepath.c_str(), "wb");
                        uint8_t buffer[4096];
                        uint32_t remaining = header.payload_size;
                        
                        while (remaining > 0) {
                            int to_read = (remaining < sizeof(buffer)) ? remaining : sizeof(buffer);
                            int r = read(fd, buffer, to_read);
                            if (r <= 0) break;
                            if (img_file) fwrite(buffer, 1, r, img_file);
                            remaining -= r;
                        }
                        if (img_file) fclose(img_file);
                        printf("[NETWORK] Image received and saved to: %s\n", filepath.c_str());
                    }
                    else if (header.type == PKT_INFO_RESPONSE) {
                        char* info_buf = new char[header.payload_size + 1];
                        read(fd, info_buf, header.payload_size);
                        info_buf[header.payload_size] = '\0';
                        printf("[NETWORK] INFO from Slave %d: %s\n", header.slave_id, info_buf);
                        delete[] info_buf;
                    }
                }
            }
        }
        
        for (int id : disconnected_slaves) {
            close(slave_sockets[id]);
            slave_sockets.erase(id);
            printf("[NETWORK] Slave ID %d disconnected.\n", id);
        }
        sock_mtx.unlock();
    }
}

// TX thread 
template <typename I>
void NetworkModule<I>::sendUDPTrigger(const NetCmdTrigger& data) {
    if (udp_broadcast_fd < 0) return;

    FutureTriggerPacket pkt;
    memcpy(pkt.header, "SYNC", 4);
    pkt.frame_id = data.frame_id;
    pkt.target_time_us = data.target_time_us;

    struct sockaddr_in bcast_addr;
    memset(&bcast_addr, 0, sizeof(bcast_addr));
    bcast_addr.sin_family = AF_INET;
    bcast_addr.sin_port = htons(udp_port > 0 ? udp_port : 9090);
    bcast_addr.sin_addr.s_addr = inet_addr(this->udpTriggerIPBand.c_str());

    sendto(udp_broadcast_fd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&bcast_addr, sizeof(bcast_addr));
}

template <typename I>
void NetworkModule<I>::sendTCPRequest(int slave_id, TCP_PACKET_TYPE type, const void* payload, size_t size) {
    lock_guard<mutex> lock(sock_mtx);
    if (slave_sockets.find(slave_id) == slave_sockets.end()) {
        printf("[NETWORK] Cannot send TCP: Slave %d not found.\n", slave_id);
        return;
    }
    
    int fd = slave_sockets[slave_id];
    TCPHeader header = {type, (uint32_t)size, -1};
    
    write(fd, &header, sizeof(TCPHeader));
    if (size > 0 && payload != nullptr) {
        write(fd, payload, size);
    }
}

template <typename I>
void NetworkModule<I>::processInput(const NetworkInput& input) {
    switch (input.cmd) {
        case NET_CMD_CONFIG: { // Config IP and port before initiating network connection
            auto cfg = get<NetCmdConfig>(input.payload);
            this->tcp_port = cfg.tcp_port;
            this->udp_port = cfg.udp_port;
            this->udp_recv_port = cfg.udp_recv_port;
            this->bind_ip = cfg.bind_ip;
            printf("\n[NETWORK] Set network to: %s - %d (tcp) - %d (udp trig) - %d (udp track)\n", cfg.bind_ip.c_str(), cfg.tcp_port, cfg.udp_port, cfg.udp_recv_port);

            if (is_running) {
                printf("[NETWORK] Restarting sockets to apply new configurations...\n");
                closeNetwork();
                initNetwork();
            }
            break;
        }
        case NET_CMD_BROADCAST_TRIGGER: {
            sendUDPTrigger(get<NetCmdTrigger>(input.payload));
            break;
        }
        case NET_CMD_REQ_IMAGE: {
            auto req = get<NetCmdReqImage>(input.payload);
            pending_img_paths[req.slave_id] = req.saveFolder;
            sendTCPRequest(req.slave_id, PKT_CMD_REQ_IMG, nullptr, 0);
            printf("\n[NETWORK] Sent Capture Request to Slave %d\n", req.slave_id);
            break;
        }
        case NET_CMD_SET_PARAM: {
            auto req = get<NetCmdSetParam>(input.payload);
            int target_fd = -1;
            
            if (req.target_ip != "" && ip_to_fd.count(req.target_ip)) {
                target_fd = ip_to_fd[req.target_ip];
            } 
            else if (req.slave_id != -999 && slave_sockets.count(req.slave_id)) {
                target_fd = slave_sockets[req.slave_id];
            }

            if (target_fd != -1) {
                string param_str = req.param_name + ":" + to_string(req.value);
                size_t total_payload_size = param_str.size();
                
                TCPHeader tcp_head = {PKT_CMD_SET_PARAM, (uint32_t)total_payload_size, 0};
                
                write(target_fd, &tcp_head, sizeof(TCPHeader));
                write(target_fd, param_str.c_str(), total_payload_size);
                
                printf("\n[NETWORK] Sent Param Set (%s) to FD %d\n", param_str.c_str(), target_fd);

                // Update new slave ID for slave_sockets
                if (req.param_name == "ID") {
                    int new_id = (int)req.value;
                    for (auto it = slave_sockets.begin(); it != slave_sockets.end(); ) {
                        if (it->second == target_fd) {
                            it = slave_sockets.erase(it);
                        } else {
                            ++it;
                        }
                    }
                    slave_sockets[new_id] = target_fd;
                    printf("[NETWORK] Server 'slave_sockets' map updated! FD %d is now ID %d.\n", target_fd, new_id);
                }
            } else {
                printf("\n[NETWORK] Failed to find target Slave for param config (IP: %s, ID: %d).\n", 
                        req.target_ip.c_str(), req.slave_id);
            }
            break;
        }
        case NET_CMD_QUERY_INFO: {
            auto req = get<NetCmdQueryInfo>(input.payload);
            sendTCPRequest(req.slave_id, PKT_CMD_REQ_INFO, nullptr, 0);
            printf("\n[NETWORK] Sent Info Query to Slave %d\n", req.slave_id);
            break;
        }
    }
}

template <typename I>
void NetworkModule<I>::runModule() {
    while (this->moduleState == NETWORK_IDLE || this->moduleState == NETWORK_RUNNING) {
        I inputData = this->inputData->pop();
        // Have to call setState() first to establish network server
        processInput(inputData);
    }
}
