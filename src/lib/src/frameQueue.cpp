#include "../frameQueue.hpp"

template <typename T>
void DataQueue<T>::push(T&& frame) {
    {
        lock_guard<mutex> lock(mtx);
        queue.push(std::move(frame));
    }
    cv.notify_one();
}

template <typename T>
bool DataQueue<T>::pop(T& frame) {
    unique_lock<mutex> lock(mtx);
    cv.wait(lock, [this] { return !queue.empty() || stop; });
        
    if (stop && queue.empty()) return false;
        
    frame = move(queue.front());
    queue.pop();
    return true;
}

template <typename T>
void DataQueue<T>::shutdown() {
    {
        lock_guard<std::mutex> lock(mtx);
        stop = true;
    }
    cv.notify_all();
}
