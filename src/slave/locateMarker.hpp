#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <set>
#include <cmath>

#include "../lib/lib.hpp"

using namespace std;

class ImgProcessor {
    private:
        int thresh_value = 245;
        uint8_t next_id = 0;
        int max_disappeared = 30;
        map<uint8_t, cv::Point2f> tracked_markers;
        map<int, int> disappeared_frames;

    public:
        ImgProcessor() {};

        vector<CenterPacket> process_frame(const cv::Mat& frame);
};