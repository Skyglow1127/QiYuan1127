#include "motion.hpp"
#include "appearance_tracker.hpp"
#include "vision_io.hpp"
#include <iostream>
#include <stdexcept>
using namespace grasp;
static void check(bool value,const char* reason) { if(!value) throw std::runtime_error(reason); }
int main() {
 try {
    cv::setNumThreads(2);
    cv::Mat texture(240,320,CV_8UC3),depth(240,320,CV_32F,cv::Scalar(.5));
    cv::RNG rng(4); rng.fill(texture,cv::RNG::UNIFORM,0,255); cv::GaussianBlur(texture,texture,{3,3},.8);
    MotionMonitor monitor; auto m=monitor.update(texture,depth,1000);
    check(m.state=="warming_up","first frame cannot establish stability");
    for(int i=1;i<=23;++i) m=monitor.update(texture,depth,1000+i*33);
    check(m.state=="still" && m.cells>=6,"textured still scene recovers after settle interval");
    cv::Mat moved; cv::Mat affine=(cv::Mat_<double>(2,3)<<1,0,3,0,1,2);
    cv::warpAffine(texture,moved,affine,texture.size());
    m=monitor.update(moved,depth,1792);
    check(m.state=="moving" && m.flowP90>20,"camera translation immediately revokes stability");
    for(int i=1;i<=10;++i) m=monitor.update(moved,depth,1792+i*33);
    check(m.state=="settling","brief pause cannot accept old moving observation");
    for(int i=11;i<=27;++i) m=monitor.update(moved,depth,1792+i*33);
    check(m.state=="still","new stable pose recovers");
    m=monitor.update(moved,depth,4000); check(m.state=="warming_up","frame gap resets history");
    m=monitor.update(moved,depth,3900); check(m.state=="warming_up","clock rollback resets history");
    MotionMonitor blank; cv::Mat plain(texture.size(),CV_8UC3,cv::Scalar(120,120,120));
    for(int i=0;i<30;++i) m=blank.update(plain,depth,1000+i*33);
    check(m.state=="uncertain","textureless view must not be called stationary");
    MotionMonitor noise;
    for(int i=0;i<30;++i) {
        cv::Mat perturb(texture.size(),CV_8SC3),noisy;
        rng.fill(perturb,cv::RNG::NORMAL,0,2);
        cv::add(texture,perturb,noisy,cv::noArray(),CV_8UC3);
        m=noise.update(noisy,depth,1000+i*33);
    }
    check(m.state=="still","small independent image noise must not prevent recovery");
    MotionMonitor z; z.update(texture,depth,1000);
    m=z.update(texture,depth+.02,1033); check(m.state=="moving","depth discontinuity also revokes stability");
    AppearanceTracker tracker;
    cv::Mat mask=cv::Mat::zeros(texture.size(),CV_8U); mask(cv::Rect(50,50,100,90)).setTo(255);
    Detection seed{0,1.f,{50,50,100,90},mask,"depth_component"};
    check(tracker.update(texture,1000,{seed},2000).empty(),"fresh observation does not duplicate a contour");
    auto tracked=tracker.update(moved,1033,{},2000);
    check(tracked.size()==1 && tracked[0].source=="temporal_appearance","translate unknown object contour through RGB correspondence");
    check(std::abs(tracked[0].box.x-53)<=1 && std::abs(tracked[0].box.y-52)<=1,"tracked contour follows observed translation");
    check(tracker.update(plain,1066,{},2000).empty(),"occlusion/feature loss removes contour");
    tracker.update(texture,1100,{seed},50);
    check(tracker.update(moved,1200,{},50).empty(),"expired appearance never persists indefinitely");
    Packet p; p.synchronized=true; p.motion=m;
    check(!depthUsable(p),"accepted receipt pairing cannot bypass motion guard");
    Config config; Camera camera; camera.size=texture.size(); camera.K={300,0,160,0,300,120,0,0,1};
    p.color=texture; p.depth=depth; p.deviceColorUs=1000000; p.deviceDepthUs=1000000;
    makeDirectory("motion_test_artifacts"); savePacket("motion_test_artifacts/moving",p,camera);
    auto restored=loadPacket("motion_test_artifacts/moving.yml",camera,config);
    check(restored->motion.enabled && !depthUsable(*restored),"moving snapshot cannot evade guard via legacy config");
    p.motion.enabled=false; p.motion.state="not_checked"; savePacket("motion_test_artifacts/legacy",p,camera);
    config.motionGuard=1; restored=loadPacket("motion_test_artifacts/legacy.yml",camera,config);
    check(!depthUsable(*restored),"single old photo cannot prove temporal stability");
    std::cout<<"motion stability, replay and validity checks passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
