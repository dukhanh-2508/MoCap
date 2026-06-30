#include "../header/ringBuffer.hpp"

using namespace std;

template <typename T>
void ThreadSafeRingBuffer<T>::push(const T& item) {
    std::lock_guard<std::mutex> lock(mtx);
        
    buffer[head] = item;
    head = (head + 1) % capacity;
        
    if (count == capacity) {
        tail = (tail + 1) % capacity;
    } else {
        count++;
    }
        
    cv.notify_one();
}

template <typename T>
T ThreadSafeRingBuffer<T>::pop() {
    // Create a mutex lock
    std::unique_lock<std::mutex> lock(mtx);
        
    // If count > 0: msg available in the queue -> move down to the code below
    // If not, unlock mutex, and the thread this function's called in is suspended and goes into a conditional variable wait queue
    // When another code calls notify (notify_all, notify_one) -> re-acquire lock, re-check condition
    cv.wait(lock, [this]() { return count > 0; });
        
    T item = buffer[tail];
    tail = (tail + 1) % capacity;
    count--;
        
    return item;
}

template <typename T>
vector<T> ThreadSafeRingBuffer<T>::drain() {
    lock_guard<std::mutex> lock(mtx);
        
    vector<T> result;
    result.reserve(count); // Cấp phát 1 lần tối ưu tốc độ
        
    while (count > 0) {
        result.push_back(buffer[tail]);
        tail = (tail + 1) % capacity;
        count--;
    }
        
    // Trả về mảng chứa toàn bộ dữ liệu, hàng đợi lúc này count = 0
    return result;
}

template <typename T>
size_t ThreadSafeRingBuffer<T>::size() {
    std::lock_guard<std::mutex> lock(mtx);
    return count;
}