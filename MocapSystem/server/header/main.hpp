#pragma once

#include "../../lib/header/ringBuffer.hpp"
#include "../header/controller.hpp"

template <typename I, typename O>
class IModule {
    private:
        ThreadSafeRingBuffer<SystemNotification> outNoti; // Receive a queue from the main thread to output notifications from this thread to the main thread
        ThreadSafeRingBuffer<I> inputData; // Receive a queue from another thread to receive data from that thread
        ThreadSafeRingBuffer<O> outputData; // A queue that can be given to another thread to push data to that thread
        int moduleState;

    public:
        virtual ~IModule() = default;

        IModule(ThreadSafeRingBuffer<SystemNotification> noti, ThreadSafeRingBuffer<I> input, int state);

        ThreadSafeRingBuffer<O> getResultQueue();
        void assignInputQueue(ThreadSafeRingBuffer<I> inputQueue);
        void assignOuNotiQueue(ThreadSafeRingBuffer<SystemNotification> notiQueue);

        virtual void setState(int newState) = 0;

        virtual void runModule() = 0;
};
