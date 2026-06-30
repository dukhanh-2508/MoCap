#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

using namespace std;

template <typename T>
class ThreadQueue {
    private:
        std::queue<T> queue;
        std::mutex mtx;
        std::condition_variable cv;
        bool stop = false;

    public:
        void push(T&& frame) {}

        bool pop(T& frame) {}

        bool pop_for(T& frame, int timeout_ms) {}

        void shutdown() {}
};
