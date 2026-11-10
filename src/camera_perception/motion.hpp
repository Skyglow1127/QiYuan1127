#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <deque>
namespace grasp {
struct MotionEvidence {
    bool enabled=false;
    std::string state="not_checked";
    double flowP90=0,depthP75=0,stableMs=0,costMs=0;
    int tracks=0,cells=0;
};
// Visual stability evidence, not robot velocity or proof of exposure synchronization.
class MotionMonitor {
    struct Sample { cv::Mat gray,depth; double time; };
    std::deque<Sample> history;
    double previousTime=0,stableSince=-1;
    double flowLimit,settleMs;
public:
    explicit MotionMonitor(double flowPixelsPerSecond=8,double settleMilliseconds=600)
        :flowLimit(flowPixelsPerSecond),settleMs(settleMilliseconds) {}
    MotionEvidence update(const cv::Mat& bgr,const cv::Mat& depth,double timestampMs);
};
}
