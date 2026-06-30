#pragma once

#include <vector>
#include <mutex>
#include <condition_variable>

template <typename T>
class ThreadSafeRingBuffer {
private:
    std::vector<T> buffer;
    size_t capacity;
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;

    std::mutex mtx;
    std::condition_variable cv;

public:
    explicit ThreadSafeRingBuffer(size_t cap) : capacity(cap), buffer(cap) {}

    void push(const T& item) {}

    T pop() {}

    std::vector<T> drain() {}

    size_t size() {}
};