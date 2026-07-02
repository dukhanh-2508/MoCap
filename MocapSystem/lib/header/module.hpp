#pragma once

#include "../../lib/header/ringBuffer.hpp"
#include "../../server/header/controller.hpp"
#include "../../lib/header/lib.hpp"

#include <memory>

using namespace std;

// Use smart pointer (shared_ptr) to avoid dangling pointer problems

template <typename I, typename O>
class IModule {
    protected:
        shared_ptr<ThreadSafeRingBuffer<SystemNotification>> outNoti;// Receive a queue from the main thread to output notifications from this thread to the main thread
        shared_ptr<ThreadSafeRingBuffer<I>> inputData; // Receive a queue from another thread to receive data from that thread
        shared_ptr<ThreadSafeRingBuffer<O>> outputData; // A queue that can be given to another thread to push data to that thread
        int moduleState;

    public:
        virtual ~IModule() = default;
        IModule(shared_ptr<ThreadSafeRingBuffer<SystemNotification>> noti, shared_ptr<ThreadSafeRingBuffer<I>> input, int state, int outputBufferSize);
        IModule(int outputBufferSize);

        shared_ptr<ThreadSafeRingBuffer<O>> getResultQueue();
        void assignInputQueue(shared_ptr<ThreadSafeRingBuffer<I>> inputQueue);
        void assignOuNotiQueue(shared_ptr<ThreadSafeRingBuffer<SystemNotification>> notiQueue);
        void assignOutDataQueue(shared_ptr<ThreadSafeRingBuffer<O>> outQueue);

        virtual void setState(int newState) = 0;
        int getModuleState();
        virtual void runModule() = 0;
};

template <typename I, typename O>
IModule<I, O>::IModule(shared_ptr<ThreadSafeRingBuffer<SystemNotification>> noti, shared_ptr<ThreadSafeRingBuffer<I>> input, int state, int outputBufferSize) {
    this->outNoti = noti;
    this->inputData = input;
    this->outputData = make_shared<ThreadSafeRingBuffer<O>>(outputBufferSize);

    this->moduleState = state;
}

template <typename I, typename O>
IModule<I, O>::IModule(int outputBufferSize) {
    this->outputData = std::make_shared<ThreadSafeRingBuffer<O>>(outputBufferSize);
    this->moduleState = 0;
}

template <typename I, typename O>
shared_ptr<ThreadSafeRingBuffer<O>> IModule<I, O>::getResultQueue() {
    return this->outputData;
}

template <typename I, typename O>
void IModule<I, O>::assignInputQueue(shared_ptr<ThreadSafeRingBuffer<I>> inputQueue) {
    this->inputData = inputQueue;
}

template <typename I, typename O>
void IModule<I, O>::assignOuNotiQueue(shared_ptr<ThreadSafeRingBuffer<SystemNotification>> notiQueue) {
    this->outNoti = notiQueue;
}

template <typename I, typename O>
void IModule<I, O>::assignOutDataQueue(shared_ptr<ThreadSafeRingBuffer<O>> outQueue) {
    this->outputData = outQueue;
}

template <typename I, typename O>
int IModule<I, O>::getModuleState() {
    return this->moduleState;
}


