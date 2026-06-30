#pragma once

#include <variant>

#include "main.hpp"
#include "calib.hpp"

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
    std::string mainMode = "";
    std::string subMode = "";
} systemMode_Info;

typedef struct {
    std::string imgSrc = "";
    bool startCalibCalc = false;
    std::string calibMode = "";
} calib_Info;

typedef struct {
    std::string triangulateMode = "";
} triangulate_Info;

typedef struct {
    bool confNetworkMaster = false;
    bool confNetworkSlave = false;
    bool confTX = false;
    bool confRX = false;
    int port = 0;
    std::string ip = "";
} network_Info;

typedef struct {
    std::string slave_ip = "";
    bool getID = false;
    int newSlaveID = 0;
    std::string toggle = "";
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

// Structs for output

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

template <typename I>
class CLIModule : public IModule<I, CLIOutputResult> {
    public:
        void setState(CLI_STATE newState) override;
        void runModule() override;
};
