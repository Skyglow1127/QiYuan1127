#include "src/controller.hpp"
#include <csignal>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace {
void signalHandler(int) { d1::interrupted = 1; }
void usage() {
    d1::out() <<
        "Unitree D1-T 机械臂终端控制\n"
        "用法: arm_control [选项] [命令及参数]\n"
        "  无命令进入交互控制台；也可从标准输入逐行读命令\n"
        "  --nic <网卡>            默认寻找唯一 192.168.123.x 网卡\n"
        "  --domain <0..232>       DDS domain，默认 0\n"
        "  --topic <话题>          默认 rt/arm_Command\n"
        "  --feedback-topic <话题> 默认 rt/arm_Feedback\n"
        "  --servo-topic <话题>    默认 current_servo_angle\n"
        "  --no-servo              不订阅辅助舵机话题\n"
        "  --timeout-ms <100..60000> 等待反馈/回执超时，默认 3000\n"
        "  --ack position|exec|recv 默认运动位置闭环；exec 执行回执，recv 接收回执\n"
        "  --joints <列表|all>      默认仅允许 4,5,6；保护其余关节\n"
        "  --j4-bias-deg <度>      J4 有符号误差补偿：下发=目标-偏差；默认关闭\n"
        "  --feedback-max-speed <deg/s> 逐轴拒绝变化大于 3×速度×接收间隔的读数；默认关闭，仅 J0–J5\n"
        "  --feedback-recovery-timeout-ms <601..60000> 单轴过期暂停，全部轴过期才超时；默认 2000 ms\n"
        "  --motion-timeout-ms <ms> 位置闭环超时，默认 8000\n"
        "  --seq-start <1..65535>   指定首次序号（协议诊断）\n"
        "  --exec '命令; 命令'     批处理，失败立即停止\n"
        "  --bootstrap-position-once 允许已批准的首个非零位置目标从 enable=0 启动一次，并回读确认\n"
        "  --dry-run               离线校验并打印 JSON，无 DDS 通信\n"
        "  --verbose               输出原始反馈 JSON\n"
        "  --watch                 相当于 watch 命令\n"
        "  --help                  帮助\n"
        "示例: arm_control --nic enp100s0 status\n"
        "      arm_control --dry-run --joints all --exec 'damping 5 1'\n"
        "返回码: 0=完成所选确认级别；1=设备/通信/状态未确认；2=参数错误；130=中断\n\n";
    d1::Controller::help();
}
}
int main(int argc, char** argv) {
    struct sigaction action{};
    action.sa_handler = signalHandler;
    sigemptyset(&action.sa_mask);
    // 不使用 SA_RESTART，让 Ctrl+C 也能打断终端 getline。
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    d1::Options options;
    std::vector<d1::Command> commands;
    bool batch = false;
    try {
        std::string exec, direct;
        bool execSet = false, watch = false;
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            auto value = [&]() -> std::string {
                if (++i >= argc || std::string(argv[i]).empty()) throw std::invalid_argument("选项缺少值: " + arg);
                return argv[i];
            };
            if (arg == "--help" || arg == "-h") { usage(); return 0; }
            if (arg == "--nic") options.nic = value();
            else if (arg == "--topic") options.topic = value();
            else if (arg == "--feedback-topic") options.feedbackTopic = value();
            else if (arg == "--servo-topic") options.servoTopic = value();
            else if (arg == "--no-servo") options.servoTopic.clear();
            else if (arg == "--domain") options.domain = d1::integer(value(), 0, 232);
            else if (arg == "--timeout-ms") options.timeoutMs = d1::integer(value(), 100, 60000);
            else if (arg == "--seq-start") options.sequenceStart = d1::integer(value(), 1, 65535);
            else if (arg == "--j4-bias-deg") {
                options.j4BiasDeg = d1::number(value());
                d1::compensateJ4(d1::Command{}, options.j4BiasDeg);
            }
            else if (arg == "--feedback-max-speed") {
                options.feedbackMaxSpeedDegS = d1::number(value());
                d1::checkFeedbackMaxSpeed(*options.feedbackMaxSpeedDegS);
            }
            else if (arg == "--feedback-recovery-timeout-ms")
                options.feedbackRecoveryTimeoutMs = d1::integer(value(), 601, 60000);
            else if (arg == "--motion-timeout-ms") options.motionTimeoutMs = d1::integer(value(), 1000, 60000);
            else if (arg == "--joints") {
                auto list = value(); options.allowed.fill(false);
                if (list == "all") options.allowed.fill(true);
                else {
                    std::istringstream ids(list);
                    for (std::string id; std::getline(ids, id, ',');) options.allowed[d1::integer(id,0,6)] = true;
                    if (list.back() == ',') throw std::invalid_argument("关节列表不能以逗号结束");
                }
            }
            else if (arg == "--ack") {
                auto mode = value();
                if (mode != "recv" && mode != "exec" && mode != "position") throw std::invalid_argument("--ack 只接受 position/recv/exec");
                options.ackMode = mode;
            } else if (arg == "--exec") {
                if (execSet) throw std::invalid_argument("--exec 不可重复");
                exec = value(); execSet = true;
            } else if (arg == "--bootstrap-position-once") options.bootstrapPositionOnce = true;
            else if (arg == "--dry-run") options.dryRun = true;
            else if (arg == "--verbose") options.verbose = true;
            else if (arg == "--watch") watch = true;
            else if (arg.rfind("-", 0) == 0) throw std::invalid_argument("未知选项: " + arg);
            else {
                direct = arg;
                while (++i < argc) direct += " " + std::string(argv[i]);
            }
        }
        if ((execSet ? 1 : 0) + (!direct.empty() ? 1 : 0) + (watch ? 1 : 0) > 1)
            throw std::invalid_argument("--exec、--watch 和直接命令不可混用；选项放在命令之前");
        batch = execSet || !direct.empty() || watch;
        if (execSet) {
            std::istringstream lines(exec);
            for (std::string line; std::getline(lines, line, ';');)
                if (!d1::words(line).empty()) commands.push_back(d1::parseCommand(line));
            if (commands.empty()) throw std::invalid_argument("--exec 没有命令");
        } else if (watch) commands.push_back(d1::parseCommand("watch"));
        else if (!direct.empty()) commands.push_back(d1::parseCommand(direct));
        for (const auto& c : commands) {
            d1::checkScope(c, options);
            d1::compensateJ4(c, options.j4BiasDeg);
        }
        // 纯帮助和限位查询同样不触发网络。
        bool offlineOnly = !commands.empty();
        for (const auto& c : commands) offlineOnly &= c.kind == d1::Kind::Help || c.kind == d1::Kind::Limits || c.kind == d1::Kind::Quit;
        if (offlineOnly) options.dryRun = true;
        // A read-only command batch has no control publisher, including the
        // deliberately disabled hold-current entry point.
        options.readOnly = !commands.empty();
        for (const auto& c : commands) options.readOnly &=
            c.kind != d1::Kind::Send && c.kind != d1::Kind::Nudge && c.kind != d1::Kind::TestJoint;
    } catch (const std::exception& e) {
        d1::err() << "[参数错误] " << e.what() << "\n"; return 2;
    }
    try {
        d1::Controller arm(options);
        if (batch) {
            for (const auto& command : commands) {
                if (d1::interrupted) return 130;
                if (command.kind == d1::Kind::Quit) break;
                if (!arm.execute(command)) return d1::interrupted ? 130 : 1;
            }
        } else {
            const bool interactive = isatty(STDIN_FILENO);
            if (interactive) d1::out() << "D1-T 控制台：help 查看命令；quit 退出。不会自动上电或使能。\n";
            std::string line;
            while (!d1::interrupted) {
                if (interactive) d1::out() << "arm> \n";
                if (!std::getline(std::cin, line)) break;
                if (d1::words(line).empty()) continue;
                try {
                    auto command = d1::parseCommand(line);
                    if (command.kind == d1::Kind::Quit) break;
                    if (!arm.execute(command) && !interactive) return d1::interrupted ? 130 : 1;
                } catch (const std::invalid_argument& e) {
                    d1::err() << "[参数错误] " << e.what() << "\n";
                    if (!interactive) return 2;
                }
            }
        }
        return d1::interrupted ? 130 : 0;
    } catch (const std::invalid_argument& e) {
        d1::err() << "[参数错误] " << e.what() << "\n"; return 2;
    } catch (const std::exception& e) {
        d1::err() << "[失败] " << e.what() << "\n"; return 1;
    }
}
