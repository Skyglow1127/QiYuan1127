#include "motion.hpp"
#include "vision_core.hpp"
#include <cmath>
namespace grasp {
MotionEvidence MotionMonitor::update(const cv::Mat& bgr,const cv::Mat& depth,double timestampMs) {
    auto start=cv::getTickCount(); MotionEvidence out; out.enabled=true; out.state="warming_up";
    cv::Mat gray,smallDepth;
    cv::resize(bgr,gray,{320,240},0,0,cv::INTER_AREA);
    cv::cvtColor(gray,gray,cv::COLOR_BGR2GRAY);
    cv::resize(depth,smallDepth,{80,60},0,0,cv::INTER_NEAREST);
    double gap=(timestampMs-previousTime)/1000.;
    if(gap<=0 || gap>.2) history.clear();
    // ~100 ms baseline suppresses subpixel sensor noise while checking every new
    // frame. A large displacement still revokes stability on its first frame.
    while(history.size()>1 && timestampMs-history.front().time>130) history.pop_front();
    if(!history.empty()) {
        const auto& reference=history.front();
        const auto& previousGray=reference.gray;
        const auto& previousDepth=reference.depth;
        double dt=(timestampMs-reference.time)/1000.;
        // Spread features across the view: the camera-mounted gripper alone cannot
        // establish scene stability. Re-seed every pair to avoid drifting tracks.
        std::vector<cv::Point2f> a,b,back;
        cv::Mat quality; cv::cornerMinEigenVal(previousGray,quality,3);
        for(int y=0;y<3;++y) for(int x=0;x<4;++x) {
            cv::Rect tile(x*80,y*80,80,80); std::vector<cv::Point2f> points;
            cv::goodFeaturesToTrack(previousGray(tile),points,12,.02,8);
            for(auto p:points) {
                p+=cv::Point2f(float(tile.x),float(tile.y));
                if(quality.at<float>(int(p.y),int(p.x))>=.0004f) a.push_back(p);
            }
        }
        std::vector<double> speeds,depthChanges;
        bool occupied[12]={false};
        if(!a.empty()) {
            std::vector<uchar> ok,reverse; std::vector<float> error,backError;
            cv::calcOpticalFlowPyrLK(previousGray,gray,a,b,ok,error,{15,15},2);
            cv::calcOpticalFlowPyrLK(gray,previousGray,b,back,reverse,backError,{15,15},2);
            for(size_t i=0;i<a.size();++i) if(ok[i] && reverse[i] && error[i]<25 &&
                cv::norm(a[i]-back[i])<1 && b[i].x>=0 && b[i].y>=0 && b[i].x<320 && b[i].y<240) {
                speeds.push_back(cv::norm(a[i]-b[i])/dt);
                occupied[int(a[i].y)/80*4+int(a[i].x)/80]=true;
            }
        }
        for(int y=0;y<60;++y) for(int x=0;x<80;++x) {
            float aZ=previousDepth.at<float>(y,x),bZ=smallDepth.at<float>(y,x);
            if(std::isfinite(aZ) && std::isfinite(bZ) && aZ>=.15 && bZ>=.15 && aZ<=1.5 && bZ<=1.5)
                depthChanges.push_back(std::abs(double(aZ)-bZ));
        }
        out.tracks=int(speeds.size()); for(bool cell:occupied) out.cells+=cell;
        if(!speeds.empty()) out.flowP90=percentile(speeds,.90);
        if(!depthChanges.empty()) out.depthP75=percentile(depthChanges,.75);
        // Depth change is supplemental; untextured RGB never becomes 'still'.
        if(out.tracks<30 || out.cells<6) { out.state="uncertain"; stableSince=-1; }
        else if(out.flowP90>flowLimit || out.depthP75>.0035) { out.state="moving"; stableSince=-1; }
        else {
            if(stableSince<0) stableSince=timestampMs;
            out.stableMs=timestampMs-stableSince;
            out.state=out.stableMs>=settleMs?"still":"settling";
        }
    } else stableSince=-1;
    history.push_back({gray,smallDepth,timestampMs}); previousTime=timestampMs;
    out.costMs=(cv::getTickCount()-start)*1000./cv::getTickFrequency();
    return out;
}
}
