#pragma once

#include <queue>
#include <variant>

#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/lib.hpp"

using namespace std;

class MocapController {
    private:
        SYSTEM_STATE currentState;

    public:
        MocapController();
        SYSTEM_STATE getCurrentState();
        SYSTEM_STATE changeState(const SYSTEM_STATE newState);
};

MocapController::MocapController() {

}

SYSTEM_STATE MocapController::getCurrentState() {
    return this->currentState;
}

SYSTEM_STATE MocapController::changeState(const SYSTEM_STATE newState) {

}
