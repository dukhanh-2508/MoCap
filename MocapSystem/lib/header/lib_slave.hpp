#pragma once

// #include "../../lib/header/lib.hpp"

#include <cstdint>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// ================= Slave Image Capturing Block =============== //
// ==========================================
// CAPTURE STRUCTS & ENUMS
// ==========================================
enum CAPTURE_BLOCK_STATE {
    CAPTURE_IDLE = 0,
    CAPTURE_RUNNING = 1,
    CAPTURE_STOP = 2,
    CAPTURE_END = 3,
    CAPTURE_UNKNOWN = 4,
    CAPTURE_START = 5,
};

// Data passed from Capture to Process block during Tracking
typedef struct {
    uint32_t frame_id;
    uint64_t capture_time_us;
    Mat frame; 
} FramePacket;

// Notification to FSM (e.g., when a manual picture is saved to RAM)
enum CAPTURE_STATUS {
    CAPTURE_SAVED_TO_RAM = 0,
    CAPTURE_FAILED = 1
};

typedef struct {
    CAPTURE_STATUS status;
    string filepath;
} CaptureNotiPayload;

// ============ Slave network block =============== //

// Output of Network block -> Input of Capture block
typedef struct {
    uint32_t frame_id;
    uint64_t target_time_us;
    bool is_manual_req; // True if requested via TCP, False if via UDP broadcast
} CaptureTrigger;

// Input to Network block (from FSM)
enum SLAVE_NET_INPUT_TYPE {
    SLAVE_NET_SEND_INFO = 0,
    SLAVE_NET_SEND_FILE = 1,
    SLAVE_NET_SEND_TRACKING = 2
};

// Notification sent back to FSM
enum NET_SERVER_CMD {
    NET_SERVER_CMD_TRACKING = 0,
    NET_SERVER_CMD_CONFIG = 1,
    NET_SERVER_CMD_INFO = 2
};

typedef struct {
    NET_SERVER_CMD cmdOrigin;
    string paramName;
    float paramValue;
} NetworkNotiPayloadSlave;

// ============= Slave Img Processing Block ================ //

enum PROCESS_BLOCK_STATE {
    PROCESS_IDLE = 0,
    PROCESS_RUNNING = 1,
    PROCESS_STOP = 2,
    PROCESS_END = 3,
    PROCESS_UNKNOWN = 4,
    PROCESS_START = 5,
};

enum PROCESS_STATUS {
    PROCESS_TRACKING_DONE = 0,
    PROCESS_FAILED = 1
};


