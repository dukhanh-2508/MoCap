#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

using namespace std;

template <typename T>
class DataQueue {
    private:
        std::queue<T> queue;
        std::mutex mtx;
        std::condition_variable cv;
        bool stop = false;

    public:
        void push(T&& frame) {
            {
                lock_guard<mutex> lock(mtx);
                queue.push(std::move(frame));
            }
            cv.notify_one();
        }

        bool pop(T& frame) {
            unique_lock<mutex> lock(mtx);
            cv.wait(lock, [this] { return !queue.empty() || stop; });
                
            if (stop && queue.empty()) return false;
                
            frame = move(queue.front());
            queue.pop();
            return true;
        }

        void shutdown() {
            {
                lock_guard<std::mutex> lock(mtx);
                stop = true;
            }
            cv.notify_all();
        }
};
