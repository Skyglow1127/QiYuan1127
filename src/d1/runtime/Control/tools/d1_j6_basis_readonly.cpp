// Two feedback-only subscriptions. No publisher, motor command, power, enable,
// zero operation, normalization or writes to robot topics are present.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

using namespace unitree::robot;
using namespace unitree_arm::msg::dds_;

int main() {
    ChannelFactory::Instance()->Init(0, "enp99s0");
    std::mutex mutex;
    std::atomic<int> feedback{0}, servo{0};
    const auto started = std::chrono::steady_clock::now();
    auto elapsed = [&] {
        return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    };
    ChannelSubscriber<ArmString_> f("rt/arm_Feedback");
    ChannelSubscriber<PubServoInfo_> s("current_servo_angle");
    f.InitChannel([&](const void* raw) {
        const auto& message = *static_cast<const ArmString_*>(raw);
        std::lock_guard<std::mutex> guard(mutex);
        ++feedback;
        // Print the received JSON string verbatim; no controller parser involved.
        std::cout << std::setprecision(12) << elapsed() << " rt/arm_Feedback "
                  << message.data_() << std::endl;
    }, 64);
    s.InitChannel([&](const void* raw) {
        const auto& m = *static_cast<const PubServoInfo_*>(raw);
        std::lock_guard<std::mutex> guard(mutex);
        ++servo;
        std::cout << std::setprecision(12) << elapsed() << " current_servo_angle "
                  << "{\"servo0_data_raw\":" << m.servo0_data_()
                  << ",\"servo1_data_raw\":" << m.servo1_data_()
                  << ",\"servo2_data_raw\":" << m.servo2_data_()
                  << ",\"servo3_data_raw\":" << m.servo3_data_()
                  << ",\"servo4_data_raw\":" << m.servo4_data_()
                  << ",\"servo5_data_raw\":" << m.servo5_data_()
                  << ",\"servo6_data_raw\":" << m.servo6_data_() << "}" << std::endl;
    }, 64);
    std::this_thread::sleep_for(std::chrono::seconds(8));
    f.CloseChannel();
    s.CloseChannel();
    std::cout << "COUNTS feedback=" << feedback.load() << " servo=" << servo.load() << std::endl;
}
