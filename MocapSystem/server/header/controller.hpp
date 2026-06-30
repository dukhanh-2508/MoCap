#pragma once

#include <queue>
#include <variant>

#include "../../lib/header/ringBuffer.hpp"
#include "../header/calib.hpp"
#include "../header/cmdI.hpp"
#include "../header/network.hpp"
#include "../header/triangulate.hpp"

using namespace std;

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
    CLI = 8, // The command line interface
    CONTROLLER = 9, // FSM of server, control system and handle routing of data
    NETWORK = 10, // Handle two-way communication with slave
    CALIB_BLOCK = 11,
    TRIANGULATE = 12,
};

// Notification / control / event packet to the controller
typedef struct {
    SUB_SYSTEM origin;
    variant<CLINotiPayload, NetworkNotiPayload, CalibNotiPayload, TriangulateNotiPayload> payload;
} SystemNotification;

class MocapController {
    private:
        SYSTEM_STATE currentState;

    public:
        MocapController();
        SYSTEM_STATE getCurrentState();
        SYSTEM_STATE changeState(const SYSTEM_STATE newState);
};
