#include "appearance_tracker.hpp"
#include <cmath>
namespace grasp {
std::vector<Detection> AppearanceTracker::update(const cv::Mat& color,double time,
        const std::vector<Detection>& fresh,double maximumAgeMs) {
    cv::Mat gray; cv::cvtColor(color,gray,cv::COLOR_BGR2GRAY);
    std::vector<Track> next; std::vector<Detection> propagated;
    if(!previous.empty() && previous.size()==gray.size() && time>previousMs && time-previousMs<=200) {
        for(const auto& track:tracks) {
            if(time-track.seededMs>maximumAgeMs || track.detection.mask.empty()) continue;
            // Current-frame evidence supersedes a historical contour. Avoid
            // optical flow work on every stationary re-detection.
            bool replaced=false;
            for(const auto& d:fresh) if((d.box & track.detection.box).area()>0) { replaced=true; break; }
            if(replaced) continue;
            cv::Mat featureRegion;
            cv::dilate(track.detection.mask,featureRegion,cv::getStructuringElement(cv::MORPH_ELLIPSE,{11,11}));
            std::vector<cv::Point2f> a,b,back,src,dst;
            cv::goodFeaturesToTrack(previous,a,50,.025,4,featureRegion);
            if(a.size()<6) continue;
            std::vector<uchar> ok,reverse; std::vector<float> error,backError;
            cv::calcOpticalFlowPyrLK(previous,gray,a,b,ok,error,{21,21},2);
            cv::calcOpticalFlowPyrLK(gray,previous,b,back,reverse,backError,{21,21},2);
            for(size_t i=0;i<a.size();++i) if(ok[i] && reverse[i] && error[i]<20 && cv::norm(back[i]-a[i])<1)
                { src.push_back(a[i]); dst.push_back(b[i]); }
            if(src.size()<6) continue;
            cv::Mat inliers,affine=cv::estimateAffinePartial2D(src,dst,inliers,cv::RANSAC,1.5);
            if(affine.empty() || cv::countNonZero(inliers)<6 || double(cv::countNonZero(inliers))/a.size()<.65) continue;
            double scale=std::hypot(affine.at<double>(0,0),affine.at<double>(1,0));
            if(scale<.85 || scale>1.18 || std::abs(std::atan2(affine.at<double>(1,0),affine.at<double>(0,0)))>.35) continue;
            cv::Mat mask,warped;
            cv::warpAffine(track.detection.mask,mask,affine,gray.size(),cv::INTER_NEAREST);
            if(cv::countNonZero(mask)<40) continue;
            cv::Rect box=cv::boundingRect(mask);
            if(box.x<=0 || box.y<=0 || box.br().x>=gray.cols || box.br().y>=gray.rows) continue;
            cv::warpAffine(previous,warped,affine,gray.size());
            cv::Mat difference; cv::absdiff(warped,gray,difference);
            if(cv::mean(difference,mask)[0]>20) continue;
            bool covered=false;
            for(const auto& detection:fresh) if((detection.box & box).area()>0) { covered=true; break; }
            if(covered) continue;
            Detection detection=track.detection; detection.box=box; detection.mask=mask;
            detection.source="temporal_appearance"; detection.verification="tracked_2d_unverified";
            next.push_back({detection,track.seededMs}); propagated.push_back(detection);
        }
    }
    for(const auto& detection:fresh) if(detection.source=="depth_component" && !detection.mask.empty() && next.size()<8)
        next.push_back({detection,time});
    tracks=std::move(next); previous=gray; previousMs=time;
    return propagated;
}
}
