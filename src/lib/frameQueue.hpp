#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

#include "receiver.hpp"

using namespace std;

template <typename T>
class DataQueue {
    private:
        std::queue<T> queue;
        std::mutex mtx;
        std::condition_variable cv;
        bool stop = false;

    public:
        void push(T&& frame);
        bool pop(T& frame);
        void shutdown();
};