#pragma once

#include "network.hpp"
#include "main.hpp"

enum TRIANGULATE_BLOCK_STATE {
    TRIANGULATE_IDLE = 0,
    TRIANGULATE_RUNNING = 1,
    TRIANGULATE_STOP = 2,
    TRIANGULATE_END = 3,
    TRIANGULATE_UNKNOWN = 4,
    TRIANGULATE_START = 5,
};

typedef struct {

} TriangulateNotiPayload;


template <typename I>
class TriangulateModule : public IModule<I, NetworkOutputResult> {
    private:
    public:
        void setState(TRIANGULATE_BLOCK_STATE newState) override;
        void runModule() override;
        bool parseNetworkInput(const NetworkOutputResult inputData);
};
