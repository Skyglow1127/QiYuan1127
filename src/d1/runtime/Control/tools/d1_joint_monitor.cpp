// Read-only D1 feedback subscriber for the display panel. No control channel.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/ArmString_.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: d1_joint_monitor <network-interface>\n";
        return 2;
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        std::mutex output;
        unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> feedback("rt/arm_Feedback");
        feedback.InitChannel([&](const void* raw) {
            if (!raw) return;
            const auto& message = *static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw);
            const double received = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            std::lock_guard<std::mutex> guard(output);
            // The raw JSON is validated by the HTTP process. Timestamps are
            // recorded here, so delayed stdout cannot make old data fresh.
            std::cout << std::setprecision(17) << "{\"received_monotonic_s\":"
                      << received << ",\"message\":" << message.data_() << "}" << std::endl;
        }, 1);
        while (!stopping) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        feedback.CloseChannel();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
