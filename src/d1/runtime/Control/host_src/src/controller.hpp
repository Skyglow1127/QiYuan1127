#pragma once
#include "protocol.hpp"
#include "output.hpp"
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <csignal>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <iostream>
#include <atomic>
#include <thread>
#include <dds/dds.hpp>

namespace d1 {
extern volatile std::sig_atomic_t interrupted;
struct Options {
    std::string nic, topic = "rt/arm_Command", feedbackTopic = "rt/arm_Feedback";
    std::string servoTopic = "current_servo_angle";
    int domain = 0, timeoutMs = 3000, motionTimeoutMs = 8000;
    int feedbackRecoveryTimeoutMs = 2000;
    int sequenceStart = -1;
    std::optional<double> j4BiasDeg;
    // Explicit opt-in: maximum configured arm speed in degrees per second.
    // J6 is a position unit, so it is not subject to the angular jump gate.
    std::optional<double> feedbackMaxSpeedDegS;
    std::array<bool, 7> allowed{false, false, false, false, true, true, true};
    std::string ackMode = "position";
    bool dryRun = false, verbose = false, readOnly = false;
    // Explicit per-process permission for the approved first nonzero position target.
    bool bootstrapPositionOnce = false;
};
std::string detectNic();
void checkScope(const Command& command, const Options& options);
// 本机 Cyclone DDS 的监听器销毁路径存在退出竞态。
// 使用独立轮询线程收取已配置 domain 的数据，退出时先 join，再销毁 reader。
// 不修改官方 SDK；发布端仍使用 Unitree ChannelPublisher。
template<class Message> class FeedbackChannel {
public:
    FeedbackChannel(int domain, const std::string& name, std::function<void(const void*)> callback)
        : participant_(domain), subscriber_(participant_), topic_(participant_, name), reader_(dds::core::null) {
        auto qos = subscriber_.default_datareader_qos();
        qos << dds::core::policy::Reliability::BestEffort() << dds::core::policy::History::KeepLast(64);
        reader_ = dds::sub::DataReader<Message>(subscriber_, topic_, qos);
        worker_ = std::thread([this, callback]() {
            while (!stop_) {
                try {
                    auto samples = reader_.take();
                    for (const auto& sample : samples) if (sample.info().valid()) callback(&sample.data());
                } catch (const std::exception& e) {
                    err() << "[DDS读取] " << e.what() << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }
    ~FeedbackChannel() { stop_ = true; if (worker_.joinable()) worker_.join(); }
    FeedbackChannel(const FeedbackChannel&) = delete;
    FeedbackChannel& operator=(const FeedbackChannel&) = delete;
private:
    dds::domain::DomainParticipant participant_;
    dds::sub::Subscriber subscriber_;
    dds::topic::Topic<Message> topic_;
    dds::sub::DataReader<Message> reader_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
};
class Controller {
public:
    explicit Controller(Options options);
    ~Controller();
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    bool execute(Command command);
    static void help();
private:
    State snapshot();
    bool waitAngles(bool includeStatus = false);
    bool printStatus();
    bool checkCurrentHold(const Command& command);
    bool send(const Command& command, bool forcePosition = false, const Angles* guardBaseline = nullptr,
              bool permitBootstrap = false);
    bool motionReady(int joint = -1);
    bool waitFeedbackGate(std::unique_lock<std::mutex>& lock, bool allowUnstartedPosition = false);
    void reportJointFeedbackStaleness(Clock::time_point now);
    void onFeedback(const void* raw);
    void onServo(const void* raw);
    Options options_;
    std::mutex mutex_;
    std::condition_variable changed_;
    State state_;
    Ack ack_;
    int sequence_;
    // A raw damping ACK is never evidence of position mode. Once requested in
    // this process, normal position commands for those joints remain blocked.
    std::array<bool,7> dampingRequested_{};
    bool bootstrapUsed_ = false, bootstrapFailed_ = false;
    // 通道最后声明：回调资源先销毁，mutex/state 的生命周期覆盖回调。
    std::unique_ptr<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> publisher_;
    std::unique_ptr<FeedbackChannel<unitree_arm::msg::dds_::ArmString_>> subscriber_;
    std::unique_ptr<FeedbackChannel<unitree_arm::msg::dds_::PubServoInfo_>> servo_;
};
} // namespace d1
