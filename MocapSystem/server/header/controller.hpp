#pragma once

#include <queue>
#include <variant>

#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/lib.hpp"

using namespace std;

class MocapController {
    private:
        SYSTEM_STATE currentState;
        int systemFPS = 15; // Control how quickly trigger messages are boardcasted and camera FPS

    public:
        MocapController();
        SYSTEM_STATE getCurrentState();
        SYSTEM_STATE changeState(const SYSTEM_STATE newState);
        int getFPS();
        void changeFPS(int newFPS);
};

int MocapController::getFPS() {
    return this->systemFPS;
}

void MocapController::changeFPS(int newFPS) {
    this->systemFPS = newFPS;
}

MocapController::MocapController() {
    this->currentState = IDLE;
}

SYSTEM_STATE MocapController::getCurrentState() {
    return this->currentState;
}

SYSTEM_STATE MocapController::changeState(const SYSTEM_STATE newState) {
    this->currentState = newState;
    return this->currentState;
}
