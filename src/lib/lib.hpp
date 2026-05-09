#pragma once

#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

typedef struct {
    bool is_running;
    int module_id;
} GlobalConfig;

typedef struct {
    GlobalConfig glcfg;
    int rcv_port;  
    string rcv_ip;
} ReceiverConfig;

typedef struct {
    GlobalConfig glcfg;
    int orch_port;  
    string orch_ip;
} OrchestratorConfig;

typedef struct {
    GlobalConfig glcfg;
    int snd_port;
    string snd_ip;
} SenderConfig;

typedef struct {
    GlobalConfig glcfg;
} CameraConfig;

typedef struct {
    GlobalConfig glcfg;
    int prs_port;
    string prs_ip;
} ProcessorConfig;


#pragma pack(push, 1)
typedef struct {
    uint8_t object_id;
    float x;                
    float y; 
} CenterPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint64_t captureTime;
    uint32_t frame_id;       
    uint8_t camera_id;                
    uint8_t centerCount;
} CameraPacketHeader;
#pragma pack(pop)

typedef struct {
    CameraPacketHeader header;
    vector<CenterPacket> centers; 
} CameraPacket;

typedef struct {
    uint32_t frame_id;
    chrono::steady_clock::time_point creation_time;
    unordered_map<uint8_t, CameraPacket> camera_data;
} AlignedFrame;


#pragma pack(push, 1)
struct FutureTriggerPacket {
    char header[4];          
    uint32_t frame_id;          
    uint64_t target_time_us;
};
#pragma pack(pop)
