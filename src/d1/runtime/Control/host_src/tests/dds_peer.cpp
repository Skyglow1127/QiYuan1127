// 仅用于 loopback / domain 231 / 专用测试话题，不连接机械臂。
#include "protocol.hpp"
#include "msg/ArmString_.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <atomic>
#include <iostream>
#include <thread>
using namespace d1;
using Message = unitree_arm::msg::dds_::ArmString_;
int main(int argc, char** argv) {
    const std::string scenario = argc > 1 ? argv[1] : "success";
    using namespace unitree::robot;
    ChannelFactory::Instance()->Init(231, "lo");
    ChannelPublisher<Message> publisher("d1_test_feedback"); publisher.InitChannel();
    std::atomic<int> seq{-1};
    std::atomic<int> power{1}, enabled{1};
    if (scenario == "hold_check" || scenario.rfind("bootstrap", 0) == 0) enabled = 0;
    if (scenario == "bootstrap_power_off") power = 0;
    if (scenario == "bootstrap_initial_enabled_drop") enabled = 1;
    std::array<std::atomic<double>,7> actual{};
    for (auto& angle : actual) angle = 0;
    if (scenario == "invalid_baseline" || scenario == "bootstrap_invalid_baseline") actual[5] = 135.5;
    if (scenario == "reconnect_out_of_range") { actual[3] = 164.4; actual[5] = -168.3; }
    if (scenario == "hold_gripper_clamp") actual[6] = 50.1;
    if (scenario == "bootstrap_quantized_half") actual[6] = 6.5;
    if (scenario == "bootstrap_jitter" || scenario == "normal_jitter") actual[1] = -90.1;
    if (scenario == "bootstrap_jitter_boundary" || scenario == "normal_jitter_boundary") actual[1] = static_cast<float>(-90.2);
    if (scenario == "bootstrap_jitter_excess" || scenario == "normal_jitter_excess") actual[1] = -90.201;
    if (scenario == "normal_j6_excess") actual[6] = 65.01;
    ChannelSubscriber<Message> subscriber("d1_test_command");
    subscriber.InitChannel([&](const void* raw) {
        auto c = json::parse(static_cast<const Message*>(raw)->data_()).as_object();
        if (scenario != "state_mismatch" && c.at("funcode") == 6)
            power = static_cast<int>(c.at("data").as_object().at("power").as_int64());
        if (c.at("funcode") == 4 || c.at("funcode") == 5)
            enabled = c.at("data").as_object().at("mode").as_int64() >= 1000 ? 1 : 0;
        if ((c.at("funcode") == 1 || c.at("funcode") == 2) && scenario != "bootstrap_no_enable") enabled = 1;
        if (c.at("funcode") == 1 && scenario != "stalled")
            actual[c.at("data").as_object().at("id").as_int64()] = c.at("data").as_object().at("angle").to_number<double>();
        if (c.at("funcode") == 2 && scenario != "stalled")
            for (int i = 0; i < 7; ++i)
                actual[i] = c.at("data").as_object().at("angle" + std::to_string(i)).to_number<double>();
        if (c.at("funcode") == 1 && scenario == "j4_bias") actual[4] = actual[4].load() + .4;
        if (scenario == "protected_drift") actual[0] = 1;
        if (scenario == "bootstrap_initial_enabled_drop") enabled = 0;
        if (scenario == "normal_postsend_jitter") actual[1] = static_cast<float>(-90.2);
        if (scenario == "normal_postsend_excess") actual[1] = -90.201;
        seq = static_cast<int>(c.at("seq").as_int64());
    }, 16);
    auto publish = [&](const json::object& obj) {
        Message m; m.data_() = json::serialize(obj); publisher.Write(m);
    };
    std::cout << "READY" << std::endl;
    for (;;) {
        if (scenario == "hold_moving") actual[0] = actual[0].load() + .12;
        if (scenario != "no_feedback") {
            json::object angles;
            for (int i=0;i<7;++i) angles["angle" + std::to_string(i)] = actual[i].load();
            publish({{"seq",10},{"address",2},{"funcode",1},{"data",angles}});
            json::object status{{"power_status",power.load()},{"enable_status",enabled.load()},{"error_status",(scenario == "fault" || scenario == "bootstrap_fault") ? 1 : 0}};
            if (scenario == "bootstrap_partial_status") status.erase("error_status");
            publish({{"seq",10},{"address",2},{"funcode",3},{"data",status}});
        }
        int id = seq.exchange(-1);
        if (id > 0 && scenario != "bootstrap_no_ack") {
            // 插入其他发送者的成功回执，验证 seq 关联。
            publish({{"seq",id+1},{"address",3},{"funcode",2},{"data",json::object{{"exec_status",1}}}});
            publish({{"seq",id},{"address",3},{"funcode",1},{"data",json::object{{"recv_status",scenario == "reject" ? 0 : 1}}}});
            if (scenario == "success" || scenario == "single_enable") publish({{"seq",id},{"address",3},{"funcode",2},{"data",json::object{{"exec_status",1}}}});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
