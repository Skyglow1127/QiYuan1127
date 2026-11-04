// Read-only subscriptions to discovered D1 topics. No publisher, writes, or motion.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/SetServoAngle_.hpp"
#include "msg/SetServoDumping_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "msg/ArmString_.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
using namespace unitree::robot;
using namespace unitree_arm::msg::dds_;
int main() {
    ChannelFactory::Instance()->Init(0,"enp99s0");
    std::mutex lock;
    std::atomic<int> angle{0}, control{0}, dumping{0}, zero{0}, feedback{0}, servo{0};
    const auto start=std::chrono::steady_clock::now();
    auto stamp=[&] {return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();};
    auto angleCallback=[&](const char* name,std::atomic<int>& count,const void* raw) {
        const auto& m=*static_cast<const SetServoAngle_*>(raw);
        std::lock_guard<std::mutex> guard(lock); ++count;
        std::cout<<stamp()<<" "<<name<<" seq="<<m.seq_()<<" id="<<int(m.id_())
                 <<" angle="<<m.angle_()<<" delay_ms="<<m.delay_ms_()<<std::endl;
    };
    ChannelSubscriber<SetServoAngle_> a("set_servo_angle"), c("set_servo_angle_control");
    ChannelSubscriber<SetServoDumping_> d("set_servo_dumping");
    ChannelSubscriber<ArmString_> z("arm_zero"), f("rt/arm_Feedback");
    ChannelSubscriber<PubServoInfo_> s("current_servo_angle");
    a.InitChannel([&](const void* m){angleCallback("set_servo_angle",angle,m);},16);
    c.InitChannel([&](const void* m){angleCallback("set_servo_angle_control",control,m);},16);
    d.InitChannel([&](const void* raw){
        const auto& m=*static_cast<const SetServoDumping_*>(raw);
        std::lock_guard<std::mutex> guard(lock); ++dumping;
        std::cout<<stamp()<<" set_servo_dumping seq="<<m.seq_()<<" id="<<int(m.id_())<<" power_raw="<<m.power_()<<std::endl;
    },16);
    z.InitChannel([&](const void* raw){
        std::lock_guard<std::mutex> guard(lock); ++zero;
        std::cout<<stamp()<<" arm_zero "<<static_cast<const ArmString_*>(raw)->data_()<<std::endl;
    },16);
    f.InitChannel([&](const void* raw){
        std::lock_guard<std::mutex> guard(lock); ++feedback;
        std::cout<<stamp()<<" rt/arm_Feedback "<<static_cast<const ArmString_*>(raw)->data_()<<std::endl;
    },16);
    s.InitChannel([&](const void* raw){
        const auto& m=*static_cast<const PubServoInfo_*>(raw);
        std::lock_guard<std::mutex> guard(lock); ++servo;
        std::cout<<stamp()<<" current_servo_angle servo4_raw="<<m.servo4_data_()<<std::endl;
    },16);
    std::this_thread::sleep_for(std::chrono::seconds(12));
    a.CloseChannel();c.CloseChannel();d.CloseChannel();z.CloseChannel();f.CloseChannel();s.CloseChannel();
    std::cout<<"COUNTS set_servo_angle="<<angle.load()<<" set_servo_angle_control="<<control.load()
             <<" set_servo_dumping="<<dumping.load()<<" arm_zero="<<zero.load()
             <<" feedback="<<feedback.load()<<" servo="<<servo.load()<<std::endl;
}
