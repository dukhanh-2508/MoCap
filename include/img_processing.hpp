#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <set>
#include <cmath>


// File này chứa bộ xử lý ảnh 2D, tìm tâm và tracking ID
struct Marker2D {
    int id;
    cv::Point2f center;
};

class ImgProcessor {
private:
    int thresh_value = 245;
    int next_id = 0;
    int max_disappeared = 30;
    std::map<int, cv::Point2f> tracked_markers;
    std::map<int, int> disappeared_frames;

public:
    ImgProcessor() {}

    std::vector<Marker2D> process_frame(const cv::Mat& frame) {
        cv::Mat gray, blurred, thresh;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(7, 7), 0);
        cv::threshold(blurred, thresh, thresh_value, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point2f> current_centers;
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 50) continue;

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(c, center, radius);
            double circularity = area / (M_PI * radius * radius);
            if (circularity < 0.6) continue;

            cv::Moments m = cv::moments(c);
            if (m.m00 == 0) continue;

            float cX = static_cast<float>(m.m10 / m.m00);
            float cY = static_cast<float>(m.m01 / m.m00);
            current_centers.push_back(cv::Point2f(cX, cY));
        }

        std::vector<Marker2D> output_markers;
        std::set<int> matched_indices;

        for (auto& track : tracked_markers) {
            int track_id = track.first;
            cv::Point2f track_pos = track.second;
            float min_dist = 99999.0f;
            int best_idx = -1;

            for (size_t i = 0; i < current_centers.size(); ++i) {
                if (matched_indices.count(i)) continue;
                float dist = std::sqrt(std::pow(current_centers[i].x - track_pos.x, 2) +
                                       std::pow(current_centers[i].y - track_pos.y, 2));
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = i;
                }
            }

            if (best_idx != -1 && min_dist < 50.0f) {
                tracked_markers[track_id] = current_centers[best_idx];
                disappeared_frames[track_id] = 0;
                matched_indices.insert(best_idx);
                output_markers.push_back({track_id, current_centers[best_idx]});
            } else {
                disappeared_frames[track_id]++;
            }
        }

        for (auto it = disappeared_frames.begin(); it != disappeared_frames.end();) {
            if (it->second > max_disappeared) {
                tracked_markers.erase(it->first);
                it = disappeared_frames.erase(it);
            } else {
                ++it;
            }
        }

        for (size_t i = 0; i < current_centers.size(); ++i) {
            if (matched_indices.count(i)) continue;
            int new_id = next_id++;
            tracked_markers[new_id] = current_centers[i];
            disappeared_frames[new_id] = 0;
            output_markers.push_back({new_id, current_centers[i]});
        }

        return output_markers;
    }
};