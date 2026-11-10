#pragma once
#include "vision_io.hpp"
namespace grasp {
class RTDetector {
    struct Impl;
    std::unique_ptr<Impl> impl;
public:
    explicit RTDetector(const Config& config);
    ~RTDetector();
    std::vector<Detection> run(const cv::Mat& image,Result& timing);
};
}
