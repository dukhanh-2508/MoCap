#include <iostream>
#include <string>
#include <chrono>
#include <sstream>

class ThrottledLogger {
private:
    std::string lastMessage;
    std::chrono::steady_clock::time_point lastTime;
    int intervalSec;

    void buildString(std::stringstream&) {}

    // Hàm bổ trợ: Đệ quy để đẩy từng đối số vào stringstream
    template<typename T, typename... Args>
    void buildString(std::stringstream& ss, T first, Args... args) {
        ss << first;             // Đẩy đối số hiện tại vào stream (giống cout)
        buildString(ss, args...); // Đệ quy cho các đối số còn lại
    }

public:
    ThrottledLogger(int seconds = 2) : intervalSec(seconds), lastTime(std::chrono::steady_clock::now()) {}

    template<typename... Args>
    void log(Args... args) {
        std::stringstream ss;
        buildString(ss, args...); // Gom tất cả biến thành 1 chuỗi duy nhất
        
        std::string msg = ss.str();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastTime).count();

        // Kiểm tra logic: Nếu nội dung khác bản tin cũ HOẶC đã quá thời gian chờ
        if (msg != lastMessage || elapsed >= intervalSec) {
            std::cerr << msg << std::endl;
            lastMessage = msg;
            lastTime = now;
        }
    }
};