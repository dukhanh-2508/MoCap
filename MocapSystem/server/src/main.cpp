#include <thread>

#include "../header/main.hpp"
#include "../header/controller.hpp"
#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/threadQueue.hpp"

/*
enum SUB_SYSTEM {
    UNKNOWN = 7,
    CLI = 8, // The command line interface
    CONTROLLER = 9, // FSM of server, control system and handle routing of data
    NETWORK = 10, // Handle two-way communication with slave
    CALIB_BLOCK = 11,
    TRIANGULATE = 12,
};
*/
template <typename I, typename O>
IModule<I, O>::IModule(ThreadSafeRingBuffer<SystemNotification> noti, ThreadSafeRingBuffer<I> input, int state) {
    this->outNoti = noti;
    this->inputData = input;
    this->outputData = ThreadSafeRingBuffer<O>(20);

    this->moduleState = state;
}

template <typename I, typename O>
ThreadSafeRingBuffer<O> IModule<I, O>::getResultQueue() {
    return this->outputData;
}

template <typename I, typename O>
void IModule<I, O>::assignInputQueue(ThreadSafeRingBuffer<I> inputQueue) {
    this->inputData = inputQueue
}

template <typename I, typename O>
void IModule<I, O>::assignOuNotiQueue(ThreadSafeRingBuffer<SystemNotification> notiQueue) {
    this->outNoti = notiQueue;
}

int main() {
    MocapController controller;
    ThreadSafeRingBuffer<SystemNotification> notiQueue(20);

    while(true) {
        SystemNotification noti = notiQueue.pop();

        switch (noti.origin) {
        case CLI:
            break;
        case NETWORK:
            break;
        case CALIB_BLOCK:
            break;
        case TRIANGULATE:
            break;
        case UNKNOWN_BLOCK:
            break;
        
        default:
            break;
        }
    }

    return 0;
}
