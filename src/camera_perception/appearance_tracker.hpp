#pragma once
#include "vision_core.hpp"
namespace grasp {
// Short-lived 2D appearance continuity, never a measured depth/grasp candidate.
class AppearanceTracker {
    struct Track { Detection detection; double seededMs; };
    std::vector<Track> tracks;
    cv::Mat previous;
    double previousMs=0;
public:
    std::vector<Detection> update(const cv::Mat& color,double timestampMs,
                                 const std::vector<Detection>& fresh,double maximumAgeMs);
};
}
