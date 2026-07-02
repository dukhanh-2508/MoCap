#pragma once

#include "lib_slave.hpp"

#include <variant>
#include <string>
#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// ========= Server calib block =========== //
enum CALIB_BLOCK_STATE {
    CALIB_IDLE = 0,
    CALIB_RUNNING = 1,
    CALIB_STOP = 2,
    CALIB_END = 3,
    CALIB_UNKNOWN = 4,
    CALIB_START = 5,
};

enum CALIB_MODE {
    CALIB_MODE_IN = 0,
    CALIB_MODE_EX = 1,
};

// Struct to describe the checker board used in calib
typedef struct {
    int row; // amount of rows of the board, in checker square amount
    int col; // amount of cols of the board, in checker square amount
    int checkSize; // Length of a side of a checker square, in mm
} CalibBoardDesc;

typedef struct {

} CalibNotiPayload;

typedef struct {
    
} CalibOutputResult;

// ========== Server CLI block =============== //
enum CLI_STATE {
    CLI_IDLE = 0,
    CLI_RUNNING = 1,
    CLI_STOP = 2,
    CLI_END = 3,
    CLI_UNKNOWN = 4,
};

enum CMD_SET {
    CLI_SYSTEM_MODE_SET = 0,
    CLI_CALIB_SET = 1,
    CLI_TRIANGULATE_SET = 2,
    CLI_NETWORK_SET = 3,
    CLI_MANAGE_SLAVE_SET = 4,
    CLI_OTHER_SET = 5,
    CLI_EXIT = 6
};

// Structs for notification msgs
typedef struct {
    string mainMode = "";
    string subMode = "";
} systemMode_Info;

typedef struct {
    string imgSrc = "";
    bool startCalibCalc = false;
    string calibMode = "";
} calib_Info;

typedef struct {
    string triangulateMode = "";
} triangulate_Info;

typedef struct {
    bool confNetworkMaster = false;
    bool confNetworkSlave = false;
    bool confTX = false;
    bool confRX = false;
    int port = 0;
    string ip = "";
} network_Info;

typedef struct {
    string slave_ip = "";
    bool getID = false;
    int newSlaveID = 0;
    string toggle = "";
} manageSlave_Info;

typedef struct {
    int takePicAmount = 0;
    int imgIdx = 0; // Onlu used when in manual mode, otherwise it should remain at 0
    bool isManualMode = true;
    string targetCamID = 0;
    string saveFolder = ".";
} other_Info;

typedef struct {
} exit_Info;

typedef struct {
    CMD_SET cmdOrigin;
    variant<systemMode_Info, calib_Info, triangulate_Info, network_Info, manageSlave_Info, other_Info, exit_Info> info;
} CLINotiPayload;

typedef struct {
    CALIB_BLOCK_STATE calibState;
    CALIB_MODE calibMode;
    CalibBoardDesc desc;
    string dataFolderIn;
    string dataFolderEx;
} CalibSettings;

typedef struct {
    CMD_SET cmdOrigin;
    variant<CalibSettings> result;
} CLIOutputResult;

// ============ Network block server ================== //
// ==========================================
// THÔNG TIN GÓI TIN & PAYLOAD 
// ==========================================
#pragma pack(push, 1)
typedef struct {
    uint8_t object_id;
    float x;                
    float y; 
} CenterPacket;

typedef struct {
    uint64_t captureTime;
    uint32_t frame_id;       
    uint8_t camera_id;                
    uint8_t centerCount;
} CameraPacketHeader;

typedef struct {
    char header[4]; // "SYNC"          
    uint32_t frame_id;          
    uint64_t target_time_us;
} FutureTriggerPacket;
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

enum NETWORK_OUTPUT_ORIGIN {
    TRACKING_COORD = 0, 
};

typedef struct {
    NETWORK_OUTPUT_ORIGIN origin;
    variant<CameraPacket> result;
} NetworkOutputResult;

// ==========================================
// GIAO THỨC TRUYỀN THÔNG TCP NỘI BỘ
// ==========================================
enum TCP_PACKET_TYPE {
    PKT_HANDSHAKE = 0,
    PKT_CAMERA_DATA = 1,
    PKT_IMAGE_DATA = 2,
    PKT_CMD_REQ_IMG = 3,
    PKT_CMD_SET_PARAM = 4,
    PKT_INFO_RESPONSE = 5,
    PKT_CMD_REQ_INFO = 6
};

#pragma pack(push, 1)
typedef struct {
    TCP_PACKET_TYPE type;
    uint32_t payload_size;
    int slave_id;
} TCPHeader;
#pragma pack(pop)

// ==========================================
// ĐẦU VÀO CỦA MODULE NETWORK (Từ Controller/CLI)
// ==========================================
enum NETWORK_BLOCK_STATE {
    NETWORK_IDLE = 0,
    NETWORK_RUNNING = 1,
    NETWORK_STOP = 2,
    NETWORK_END = 3,
    NETWORK_UNKNOWN = 4,
    NETWORK_START = 5,
};

enum NETWORK_CMD {
    NET_CMD_CONFIG = 0,
    NET_CMD_BROADCAST_TRIGGER = 1,
    NET_CMD_REQ_IMAGE = 2,
    NET_CMD_SET_PARAM = 3,
    NET_CMD_QUERY_INFO = 4
};

struct NetCmdConfig { int tcp_port; int udp_port; int udp_recv_port; string bind_ip; };
struct NetCmdTrigger { uint64_t target_time_us; uint32_t frame_id; };
struct NetCmdReqImage { int slave_id; string saveFolder; };
struct NetCmdSetParam { int slave_id; string param_name; float value; };
struct NetCmdQueryInfo { int slave_id; };

typedef struct {
    NETWORK_CMD cmd;
    variant<NetCmdConfig, NetCmdTrigger, NetCmdReqImage, NetCmdSetParam, NetCmdQueryInfo> payload;
} NetworkInput;

typedef struct {
} NetworkNotiPayload;

// ============ Server triangulation block ============== //
enum TRIANGULATE_BLOCK_STATE {
    TRIANGULATE_IDLE = 0,
    TRIANGULATE_RUNNING = 1,
    TRIANGULATE_STOP = 2,
    TRIANGULATE_END = 3,
    TRIANGULATE_UNKNOWN = 4,
    TRIANGULATE_START = 5,
};

typedef struct {
    int camera_id;
    Point2f pt_2d;
} CameraObservation;

typedef struct {

} TriangulateNotiPayload;

// ============ Controller block server =============== //
// System states
enum SYSTEM_STATE {
    UNKNOWN_STATE = 0,
    CALIB_STATE = 1,
    TRACK = 2,
    EVAL = 3,
    START = 4,
    STOP = 5,
    IDLE = 6
};

// Sub-systems
enum SUB_SYSTEM {
    UNKNOWN_BLOCK = 7,
    CLI_BLOCK = 8, // The command line interface
    CONTROLLER = 9, // FSM of server, control system and handle routing of data
    NETWORK = 10, // Handle two-way communication with slave
    CALIB_BLOCK = 11,
    TRIANGULATE = 12,
    CAPTURE_BLOCK = 13, // Image capturing block on slave
    PROCESS_BLOCK = 14, // Img processing block on slave
};

typedef struct {
    PROCESS_STATUS status;
    CameraPacket cameraData;
} ProcessNotiPayload;

typedef struct {
    SLAVE_NET_INPUT_TYPE type;
    string infoMsg;
    string filepath;
    CameraPacket cameraData;
} SlaveNetworkInput;

// Notification / control / event packet to the controller
typedef struct {
    SUB_SYSTEM origin;
    variant<CLINotiPayload, NetworkNotiPayload, CalibNotiPayload, TriangulateNotiPayload, NetworkNotiPayloadSlave, CaptureNotiPayload, ProcessNotiPayload> payload;
} SystemNotification;



