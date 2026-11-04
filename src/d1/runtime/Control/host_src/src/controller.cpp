#include "controller.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <unistd.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <thread>

namespace d1 {
volatile std::sig_atomic_t interrupted = 0;
using namespace std::chrono_literals;
namespace {
std::string statusValue(std::optional<int> v) { return v ? std::to_string(*v) : "未知"; }
long age(Clock::time_point time) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - time).count();
}
bool pause(double seconds) {
    auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (!interrupted && Clock::now() < end) std::this_thread::sleep_for(20ms);
    return !interrupted;
}
}
std::string detectNic() {
    ifaddrs* list = nullptr;
    if (getifaddrs(&list) != 0) throw std::runtime_error("读取网卡失败，请使用 --nic 指定网卡");
    std::set<std::string> matches;
    for (auto* p = list; p; p = p->ifa_next) {
        if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET || !(p->ifa_flags & IFF_UP) || (p->ifa_flags & IFF_LOOPBACK)) continue;
        auto ip = ntohl(reinterpret_cast<sockaddr_in*>(p->ifa_addr)->sin_addr.s_addr);
        if ((ip & 0xffffff00u) == 0xc0a87b00u) matches.insert(p->ifa_name);
    }
    freeifaddrs(list);
    if (matches.size() != 1) throw std::runtime_error("未找到唯一的 192.168.123.x 网卡，请用 ip -br address 检查并使用 --nic 指定");
    return *matches.begin();
}
void checkScope(const Command& c, const Options& options) {
    const bool hold = c.kind == Kind::HoldCurrent || c.kind == Kind::HoldCurrentCheck;
    if (c.kind != Kind::Send && c.kind != Kind::Nudge && c.kind != Kind::TestJoint && !hold) return;
    if ((hold && c.id >= 0) || (c.kind == Kind::Nudge || c.kind == Kind::TestJoint) || c.function == 1 || c.function == 4) {
        int id = (hold || c.kind == Kind::Nudge || c.kind == Kind::TestJoint) ? c.id : static_cast<int>(c.data.at("id").as_int64());
        if (!options.allowed.at(id)) throw std::invalid_argument("J" + std::to_string(id) + " 已受保护，未发送指令");
    } else {
        for (bool allowed : options.allowed) if (!allowed)
            throw std::invalid_argument("当前仅允许选定关节；禁止全关节目标、归零、整体电源协议/阻尼请求/同位恢复");
    }
}
Controller::Controller(Options options) : options_(std::move(options)),
    sequence_(static_cast<int>((Clock::now().time_since_epoch().count() ^ getpid()) & 0x3fffffff) + 100) {
    if (options_.sequenceStart >= 0) sequence_ = options_.sequenceStart - 1;
    if (options_.feedbackMaxSpeedDegS) {
        checkFeedbackMaxSpeed(*options_.feedbackMaxSpeedDegS);
        if (options_.feedbackRecoveryTimeoutMs <= 600 || options_.feedbackRecoveryTimeoutMs > 60000)
            throw std::invalid_argument("feedback recovery timeout must be in 601..60000 ms");
        out() << "[feedback-filter] " << json::serialize(json::object{
            {"enabled", true}, {"max_speed_deg_s", *options_.feedbackMaxSpeedDegS},
            {"multiplier", 3}, {"joints", "J0-J5"}, {"soft_freshness_ms", 600},
            {"recovery_timeout_ms", options_.feedbackRecoveryTimeoutMs}}) << std::endl;
    }
    if (options_.dryRun) {
        if (options_.sequenceStart < 0) sequence_ = 100;
        out() << "[dry-run] 离线模式，不初始化 DDS，不模拟设备成功反馈\n";
        return;
    }
    if (options_.nic.empty()) options_.nic = detectNic();
    if (if_nametoindex(options_.nic.c_str()) == 0) throw std::runtime_error("网卡不存在: " + options_.nic);
    using namespace unitree::robot;
    ChannelFactory::Instance()->Init(options_.domain, options_.nic);
    if (!options_.readOnly) {
        publisher_ = std::make_unique<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(options_.topic);
        publisher_->InitChannel();
    }
    subscriber_ = std::make_unique<FeedbackChannel<unitree_arm::msg::dds_::ArmString_>>(options_.domain, options_.feedbackTopic, [this](const void* msg) { onFeedback(msg); });
    if (!options_.servoTopic.empty()) {
        servo_ = std::make_unique<FeedbackChannel<unitree_arm::msg::dds_::PubServoInfo_>>(options_.domain, options_.servoTopic, [this](const void* msg) { onServo(msg); });
    }
    out() << "[DDS" << (options_.readOnly ? "只读" : "") << "] nic=" << options_.nic << " domain=" << options_.domain
              << (options_.readOnly ? "\n  无控制 publisher" : "\n  TX " + options_.topic)
              << "\n  RX " << options_.feedbackTopic << std::endl;
}
Controller::~Controller() {
    servo_.reset();
    subscriber_.reset();
    if (publisher_) publisher_->CloseChannel();
}
void Controller::onFeedback(const void* raw) {
    if (!raw) return;
    const auto receivedAt = Clock::now();
    try {
        const auto& text = static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw)->data_();
        if (options_.verbose || options_.feedbackMaxSpeedDegS) out() << "[rx] " << text << std::endl;
        auto feedback = parseFeedback(text, bool(options_.feedbackMaxSpeedDegS));
        if (!feedback) {
            if (options_.feedbackMaxSpeedDegS) {
                std::lock_guard<std::mutex> lock(mutex_);
                state_.jointRecoveryCandidates.fill(std::nullopt);
            }
            return;
        }
        std::optional<json::object> rejected;
        json::object marker;
        bool anyAccepted = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rejected = updateFilteredFeedback(state_, *feedback, receivedAt, options_.feedbackMaxSpeedDegS,
                options_.feedbackRecoveryTimeoutMs);
            ack_.update(*feedback);
            if (options_.feedbackMaxSpeedDegS && feedback->angles) {
                anyAccepted = std::any_of(state_.acceptedJoints.begin(), state_.acceptedJoints.end(), [](bool v) { return v; });
                marker = makeFeedbackMarker(state_, json::parse(text), receivedAt, rejected);
            }
            if (options_.feedbackMaxSpeedDegS) reportJointFeedbackStaleness(Clock::now());
        }
        if (options_.feedbackMaxSpeedDegS && feedback->angles)
            out() << (anyAccepted ? "[feedback-accepted] " : "[feedback-rejected] ")
                  << json::serialize(marker) << std::endl;
        changed_.notify_all();
    } catch (const std::exception& e) {
        err() << "[反馈解析] " << e.what() << std::endl;
    }
}
void Controller::onServo(const void* raw) {
    if (!raw) return;
    const auto& m = *static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);
    Angles a{m.servo0_data_(),m.servo1_data_(),m.servo2_data_(),m.servo3_data_(),m.servo4_data_(),m.servo5_data_(),m.servo6_data_()};
    for (auto value : a) if (!std::isfinite(value)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    state_.servo = a; state_.servoTime = Clock::now();
}
State Controller::snapshot() { std::lock_guard<std::mutex> lock(mutex_); return state_; }
bool Controller::waitAngles(bool includeStatus) {
    if (options_.dryRun) return false;
    auto deadline = Clock::now() + std::chrono::milliseconds(options_.timeoutMs);
    std::unique_lock<std::mutex> lock(mutex_);
    auto ready = [&]() {
        auto now = Clock::now();
        return state_.freshAngles(now) && (!includeStatus ||
            (state_.power && state_.enable && now - state_.powerTime <= 2s && now - state_.enableTime <= 2s));
    };
    while (!interrupted && !ready() && Clock::now() < deadline)
        changed_.wait_for(lock, 50ms);
    return !interrupted && ready();
}
bool Controller::printStatus() {
    auto s = snapshot();
    auto output = out();
    output << std::fixed << std::setprecision(2);
    if (s.angles) {
        for (int i = 0; i < 7; ++i) output << "J" << i << "=" << (*s.angles)[i] << (i == 6 ? " SDK单位(未标定)" : " deg") << "  ";
        output << "\n角度反馈年龄: " << age(s.angleTime) << " ms" << (s.freshAngles(Clock::now()) ? "" : " [已过期]") << "\n";
        output << "有效角度帧=" << s.angleFrames << " 最近帧间隔=" << s.angleIntervalMs << " ms（-1 为样本不足）\n";
    } else output << "关节角度: 未收到完整反馈\n";
    output << "enable_status=" << statusValue(s.enable) << " power_status=" << statusValue(s.power)
              << " error_status=" << statusValue(s.error) << "（软件状态原值；不是硬件供电/扭矩/位置模式的独立确认）\n";
    if (s.enable) output << "enable 软件状态年龄: " << age(s.enableTime) << " ms\n";
    if (s.power) output << "power 软件状态年龄: " << age(s.powerTime) << " ms\n";
    if (s.error) output << "错误码反馈年龄: " << age(s.errorTime) << " ms\n";
    if (s.servo) {
        output << "current_servo_angle 已收到，年龄 " << age(s.servoTime) << " ms\n";
        if (options_.verbose) {
            output << "辅助舵机原始值（单位/零偏尚未独立验证）: ";
            for (int i = 0; i < 7; ++i) output << "servo" << i << "=" << (*s.servo)[i] << " ";
            output << "\n";
        }
    }
    return s.freshAngles(Clock::now());
}
bool Controller::motionReady(int joint) {
    if (options_.dryRun) return true;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!waitFeedbackGate(lock)) return false;
    }
    waitAngles(true);
    auto s = snapshot();
    auto now = Clock::now();
    const bool dampingPending = joint >= 0 ? dampingRequested_[joint] :
        std::any_of(dampingRequested_.begin(), dampingRequested_.end(), [](bool v) { return v; });
    if (!s.freshAngles(now) || s.power != 1 || s.enable != 1 || s.error != 0 || dampingPending ||
        now - s.powerTime > 2s || now - s.enableTime > 2s || now - s.errorTime > 2s) {
        err() << "[未发送] 运动需要新鲜角度及软件 power=1/enable=1/error=0，且本进程没有未恢复的原始阻尼请求。"
                     "软件状态和阻尼 ACK 都不证明硬件位置模式；enable/release 别名已停用。\n";
        return false;
    }
    for (int i = 0; i < 7; ++i) checkFeedbackAngle(i, (*s.angles)[i]);
    return true;
}
bool Controller::waitFeedbackGate(std::unique_lock<std::mutex>& lock, bool allowUnstartedPosition) {
    if (!options_.feedbackMaxSpeedDegS) return true;
    const auto enteredAt = Clock::now();
    const auto deadline = state_.feedbackPendingSince.value_or(enteredAt) +
        std::chrono::milliseconds(options_.feedbackRecoveryTimeoutMs);
    const auto gapCountAtEntry = state_.allFeedbackGapCount;
    bool waiting = false;
    auto diagnostic = [&] {
        const auto now = Clock::now();
        json::object event{{"event_monotonic_s", std::chrono::duration<double>(now.time_since_epoch()).count()},
            {"timeout_policy", "all_seven_joint_ages"},
            {"timeout_ms", options_.feedbackRecoveryTimeoutMs}, {"feedback_pending", bool(state_.feedbackPendingSince)},
            {"wait_s", std::chrono::duration<double>(now - enteredAt).count()}};
        if (state_.angles) {
            event["last_valid_monotonic_s"] = std::chrono::duration<double>(state_.angleTime.time_since_epoch()).count();
            event["last_valid_age_s"] = std::chrono::duration<double>(now - state_.angleTime).count();
        }
        for (const auto& field : jointFeedbackMetadata(state_)) event[field.key()] = field.value();
        return event;
    };
    while (!interrupted) {
        if (ack_.rejected()) {
            err() << "[设备拒绝] 反馈等待期间收到当前 seq 的否定回执；停止后续运动\n";
            return false;
        }
        reportJointFeedbackStaleness(Clock::now());
        // A real observed limit violation must not be hidden behind another
        // axis waiting for recovery. The filtering policy never relaxes limits.
        for (int joint = 0; joint < 7; ++joint)
            if (state_.jointAngles[joint]) checkFeedbackAngle(joint, *state_.jointAngles[joint]);
        const auto decision = feedbackRecoveryState(state_, Clock::now(), deadline,
            allowUnstartedPosition, options_.feedbackRecoveryTimeoutMs, gapCountAtEntry);
        if (decision == FeedbackRecoveryState::Ready) {
            if (waiting) out() << "[feedback-gap-recovered] " << json::serialize(diagnostic()) << std::endl;
            return true;
        }
        if (decision == FeedbackRecoveryState::UnsafeStatus) {
            err() << "[未发送] 反馈等待期间收到不允许的 power/enable/error 状态；停止后续运动\n";
            return false;
        }
        if (decision == FeedbackRecoveryState::TimedOut) {
            err() << "[feedback-timeout] " << json::serialize(diagnostic()) << std::endl;
            return false;
        }
        if (!waiting) {
            out() << "[feedback-gap-wait] " << json::serialize(diagnostic()) << std::endl;
            waiting = true;
        }
        changed_.wait_for(lock, 20ms);
    }
    return false;
}
void Controller::reportJointFeedbackStaleness(Clock::time_point now) {
    for (const auto& event : updateJointFeedbackStaleness(state_, now)) {
        const bool stale = event.as_object().at("state") == "stale";
        out() << (stale ? "[joint-feedback-stale] " : "[joint-feedback-recovered] ")
              << json::serialize(event) << std::endl;
    }
}
bool Controller::checkCurrentHold(const Command& c) {
    if (options_.dryRun) {
        err() << "[未发送] hold-current 需要实际反馈；离线模式不假定零位、不生成可执行恢复目标\n";
        return false;
    }
    if (!waitAngles(true)) {
        err() << "[未发送] hold-current 没有新鲜完整反馈/软件状态\n";
        return false;
    }
    auto first = snapshot();
    checkHoldState(first, Clock::now());
    State latest = first;
    int samples = 1;
    auto deadline = Clock::now() + std::chrono::milliseconds(options_.timeoutMs);
    // Three distinct frames spanning at least 100 ms; no reset-and-retry if
    // the pose moves. The limits are recovery checks, not motor speed ratings.
    while (!interrupted && Clock::now() < deadline &&
           (samples < 3 || latest.angleTime - first.angleTime < 100ms)) {
        if (!pause(0.02)) break;
        auto next = snapshot();
        checkHoldState(next, Clock::now());
        if (next.angleTime > latest.angleTime) {
            checkHoldDrift(*first.angles, *next.angles);
            latest = next;
            ++samples;
        }
    }
    if (interrupted || samples < 3 || latest.angleTime - first.angleTime < 100ms) {
        err() << "[未发送] 同位恢复未取得连续稳定的新反馈\n";
        return false;
    }
    for (int id = 0; id < 7; ++id) if (c.id < 0 || c.id == id) {
        const auto candidate = makeHoldCurrent(latest, Clock::now(), id);
        out() << "[hold-candidate-only] J" << id << " measured=" << std::setprecision(9)
              << candidate.data.at("angle") << " requested_delay_ms=1000；未下发\n";
    }
    out() << "[hold-check] " << samples << " 帧通过新鲜度/范围/稳定检查；J6 同位恢复目标须为 0..50。"
          << "这只证明采样位置，不证明硬件扭矩/位置模式。\n";
    if (c.kind == Kind::HoldCurrentCheck) return true;
    // No runtime boolean override. Enabling this path requires an independently
    // verified deployed controller contract and a subsequent reviewed change.
    err() << "[未发送] hold-current 暂停：已保存控制器零位移分支存在 0/0，且未确认现役机载修复。"
             "请完成机载零距离修复与部署核验；不能用 damping 1000、回零或角度取模代替。\n";
    return false;
}
bool Controller::send(const Command& c, bool forcePosition, const Angles* guardBaseline, bool permitBootstrap) {
    if (interrupted) return false;
    if (options_.readOnly) {
        err() << "[未发送] 只读模式没有控制 publisher\n"; return false;
    }
    const auto wire = compensateJ4(c, options_.j4BiasDeg);
    // 首次发指令前确保反馈通道已实际收到数据，避免命令抵达但瞬时回执丢在订阅发现窗口。
    if (!options_.dryRun && !c.motion && !waitAngles()) {
        err() << "[连接就绪超时] 尚未收到完整角度反馈，未发送指令\n"; return false;
    }
    bool bootstrap = false;
    if (c.motion && bootstrapFailed_) {
        err() << "[未发送] 首次位置启动未完成确认，本进程停止后续运动；不会自动重试\n";
        return false;
    }
    if (c.motion && !options_.dryRun) {
        const bool allowUnstarted = options_.bootstrapPositionOnce && !bootstrapUsed_ && permitBootstrap &&
            c.kind == Kind::Send && c.function == 1;
        std::unique_lock<std::mutex> lock(mutex_);
        if (!waitFeedbackGate(lock, allowUnstarted)) return false;
    }
    if (c.motion && !options_.dryRun && options_.bootstrapPositionOnce && !bootstrapUsed_ &&
        permitBootstrap && c.kind == Kind::Send && (c.function == 1 || c.function == 2)) {
        waitAngles(true);
        auto initial = snapshot();
        if (initial.enable == 0) {
            if (std::any_of(dampingRequested_.begin(), dampingRequested_.end(), [](bool v) { return v; })) {
                err() << "[未发送] 本进程已有原始阻尼请求，禁止使用位置启动标志\n";
                return false;
            }
            checkPositionBootstrap(wire, initial, Clock::now());
            bootstrap = true;
        }
    }
    if (c.motion && !bootstrap && !motionReady(c.function == 1 ? static_cast<int>(c.data.at("id").as_int64()) : -1)) return false;
    if (++sequence_ >= 2000000000) sequence_ = 100;
    if (sequence_ == 10) ++sequence_;
    auto payload = encode(wire, sequence_);
    if (wire.data != c.data) {
        const auto key = c.function == 1 ? "angle" : "angle4";
        out() << "[compensation] J4 desired=" << c.data.at(key)
                  << " bias=" << *options_.j4BiasDeg << " wire=" << wire.data.at(key) << "\n";
    }
    if (options_.dryRun) {
        out() << "[tx] " << payload << std::endl;
        return true;
    }
    Clock::time_point sentAt;
    const bool positionMode = (forcePosition || options_.ackMode == "position") && c.motion && (c.function == 1 || c.function == 2);
    PositionCheck position;
    position.allowed = options_.allowed;
    if (c.motion) {
        if (c.function == 1) {
            const auto id = c.data.at("id").as_int64();
            position.targets[id] = c.data.at("angle").to_number<double>();
            // test-joint passes forcePosition=true. During that safety test, every
            // non-target joint must stay at its measured baseline, including the
            // other joints that are normally in the operator's allowed set.
            if (forcePosition) {
                position.allowed.fill(false);
                position.allowed[id] = true;
            }
        }
        if (c.function == 2) for (int i = 0; i < 7; ++i) position.targets[i] = c.data.at("angle" + std::to_string(i)).to_number<double>();
    }
    unitree_arm::msg::dds_::ArmString_ msg;
    msg.data_() = payload;
    if (c.function == 4 || c.function == 5) {
        if (c.function == 4) dampingRequested_[c.data.at("id").as_int64()] = true;
        else dampingRequested_.fill(true);
        out() << "[原始阻尼请求] mode=" << c.data.at("mode")
              << "；ACK 只确认协议请求，绝不确认位置保持或硬件扭矩状态\n";
    }
    {
        // Serialize the last pending-frame check with the actual publish. A
        // rejected sample processed after motionReady must not permit another
        // target to slip through before the next accepted sample.
        std::unique_lock<std::mutex> publishLock(mutex_);
        if (c.motion && !waitFeedbackGate(publishLock, bootstrap)) return false;
        const auto now = Clock::now();
        if (c.motion && options_.feedbackMaxSpeedDegS) {
            if (feedbackRecoveryState(state_, now, now + std::chrono::milliseconds(options_.feedbackRecoveryTimeoutMs),
                    bootstrap, options_.feedbackRecoveryTimeoutMs) != FeedbackRecoveryState::Ready) {
                err() << "[feedback-timeout] 实际发布前缺少 0.6 秒内的有效角度；未发送运动目标\n";
                return false;
            }
            for (int joint = 0; joint < 7; ++joint) checkFeedbackAngle(joint, (*state_.angles)[joint]);
            if (bootstrap) checkPositionBootstrap(wire, state_, now);
            else if (state_.power != 1 || state_.enable != 1 || state_.error != 0 ||
                     now - state_.powerTime > 2s || now - state_.enableTime > 2s || now - state_.errorTime > 2s) {
                err() << "[未发送] 反馈恢复后的软件状态不满足运动条件\n";
                return false;
            }
        }
        sentAt = now;
        position.sentAt = sentAt;
        if (c.motion) position.baseline = guardBaseline ? *guardBaseline : *state_.angles;
        ack_ = Ack{}; ack_.seq = sequence_;
        // Consume permission only for an actual publish, never for a paused
        // command. No same-position hold is sent while feedback is rejected.
        if (options_.bootstrapPositionOnce && permitBootstrap && c.kind == Kind::Send &&
            c.motion && (c.function == 1 || c.function == 2)) bootstrapUsed_ = true;
        if (bootstrap) {
            bootstrapFailed_ = true;
            out() << "[position-bootstrap] 已批准路径首个非零位置目标；等待新 enable=1 状态及角度反馈，不自动重发\n";
        }
        out() << "[tx] " << payload << std::endl;
        if (!publisher_->Write(msg, static_cast<int64_t>(options_.timeoutMs) * 1000)) {
            err() << "[DDS写入失败] 本地发布失败，尚未确认设备接收\n"; return false;
        }
    }
    auto receiveDeadline = Clock::now() + std::chrono::milliseconds(options_.timeoutMs);
    auto motionDeadline = Clock::now() + std::chrono::milliseconds(options_.motionTimeoutMs);
    bool verifyPower = c.function == 6;
    std::unique_lock<std::mutex> lock(mutex_);
    bool receiptReported = false;
    while (!interrupted) {
        if (c.motion && options_.feedbackMaxSpeedDegS) {
            const auto checkedAt = Clock::now();
            const bool blocked = feedbackRecoveryState(state_, checkedAt,
                checkedAt + std::chrono::milliseconds(options_.feedbackRecoveryTimeoutMs), bootstrap,
                options_.feedbackRecoveryTimeoutMs) != FeedbackRecoveryState::Ready;
            if (!waitFeedbackGate(lock, bootstrap)) return false;
            if (blocked) {
                const auto waited = Clock::now() - checkedAt;
                receiveDeadline += waited;
                motionDeadline += waited;
            }
        }
        if (ack_.rejected()) {
            err() << "[设备拒绝] seq=" << sequence_ << " recv=" << statusValue(ack_.received)
                      << " exec=" << statusValue(ack_.executed) << "\n"; return false;
        }
        if (ack_.accepted(false) && !receiptReported) {
            out() << "[接收确认] seq=" << sequence_ << " 耗时=" << age(sentAt) << " ms\n";
            receiptReported = true;
        }
        if (c.motion) {
            position.observe(state_);
            if (position.violated) {
                err() << "[保护停止] 未授权关节反馈偏移超过 0.5°；停止后续操作，不向受保护关节发送纠正指令\n";
                return false;
            }
            if (!state_.freshAngles(Clock::now())) {
                err() << "[反馈中断] 已发送目标，但角度反馈超过 2 秒未更新；停止后续操作\n";
                return false;
            }
        }
        bool stateVerified = false;
        if (verifyPower) stateVerified = state_.powerTime > sentAt && state_.power == c.data.at("power").as_int64();
        bool bootstrapVerified = !bootstrap ||
            (state_.enable == 1 && state_.power == 1 && state_.error == 0 &&
             state_.enableTime > sentAt && state_.powerTime > sentAt && state_.errorTime > sentAt &&
             state_.angleTime > sentAt && state_.freshAngles(Clock::now()) &&
             Clock::now() - state_.enableTime <= 250ms && Clock::now() - state_.powerTime <= 250ms &&
             Clock::now() - state_.errorTime <= 250ms);
        if (bootstrap && state_.errorTime > sentAt && state_.error != 0) {
            err() << "[启动失败] 首个位置目标后收到 error!=0，停止后续操作\n";
            return false;
        }
        bool complete = bootstrapVerified && (positionMode ? position.settled >= 3 && ack_.accepted(false) :
            verifyPower ? stateVerified && ack_.accepted(false) : ack_.accepted(options_.ackMode != "recv"));
        if (complete) {
            if (bootstrap) {
                bootstrapFailed_ = false;
                out() << "[position-bootstrap-ok] 已收到命令发送后的新 enable=1/power=1/error=0 和实际角度反馈；不是独立硬件扭矩证明\n";
            }
            auto output = out();
            output << "[ok] seq=" << sequence_;
            if (positionMode) {
                output << " 位置闭环通过，连续=" << position.settled << " 帧；目标误差=" << position.maxError
                          << "；受保护关节最大偏移=" << position.protectedDrift
                          << "；exec=" << statusValue(ack_.executed);
                for (int i = 0; i < 7; ++i) if (position.targets[i])
                    output << " J" << i << "=" << (*state_.angles)[i] << " target=" << *position.targets[i];
            } else output << (stateVerified ? " 接收回执及软件状态回读确认；不证明硬件电源/扭矩状态" :
                         ack_.executed == 1 ? " 协议执行回执确认；不证明硬件位置模式或动作完成" : " 设备已接收；尚未确认执行完成");
            output << std::endl;
            return true;
        }
        if (!ack_.accepted(false) && Clock::now() >= receiveDeadline) {
            err() << "[接收超时] 未收到本 seq 的回执；角度反馈="
                      << (state_.freshAngles(Clock::now()) ? "正常，检查指令话题/服务" : "中断，检查链路") << "\n";
            break;
        }
        if (Clock::now() >= (positionMode ? motionDeadline : receiveDeadline)) {
            if (bootstrap && !bootstrapVerified) {
                err() << "[启动未确认] 首个位置目标未获得新 enable=1/power=1/error=0 及角度反馈；停止后续操作\n";
                break;
            }
            if (positionMode) {
                auto output = err();
                output << "[位置超时] 设备已接收，但反馈未连续到达目标；不是连接超时。";
                for (int i = 0; i < 7; ++i) if (position.targets[i])
                    output << " J" << i << "=" << (*state_.angles)[i] << " target=" << *position.targets[i];
                output << "\n";
            } else err() << "[执行/状态未确认] 已接收，但未获得所需执行回执或状态；不是连接超时\n";
            break;
        }
        changed_.wait_for(lock, 50ms);
    }
    err() << "[未确认] seq=" << sequence_ << " recv=" << statusValue(ack_.received)
              << " exec=" << statusValue(ack_.executed)
              << (interrupted ? "，用户中断" : "") << "。不自动重发；停止后续操作。\n";
    return false;
}
void Controller::help() {
    out() <<
        "命令（J0..J5 单位度；J6 为 SDK 原始单位，尚未标定为实际开口 mm）:\n"
        "  status / doctor         状态 / 链路诊断（不发送控制指令）\n"
        "  watch [秒数]            每 0.5 秒显示；默认持续到 Ctrl+C\n"
        "  limits                  本地输入限位\n"
        "  power on|off            原始 funcode 6 请求；回执/软件状态不证明物理供电或断扭矩\n"
        "  enable / release        已拒绝：旧别名误把原始阻尼称作位置使能/卸力\n"
        "  damping <id|all> <值>   显式原始阻尼，0..80000；非位置使能，勿凭状态位推断扭矩\n"
        "  hold-current-check <id|all> 只读检查新鲜/稳定/范围内反馈并显示同位候选\n"
        "  hold-current <id|all>   同位恢复接口暂时拒绝发送：机载零距离修复尚未核验\n"
        "  j <id> <角度> [delay_ms] 单关节绝对目标\n"
        "  nudge <id> <增量>       按新鲜反馈相对移动，单次最多 ±10\n"
        "  test-joint <id> <增量>  小幅往返闭环测试，增量绝对值 0.8..2\n"
        "  grip <0..65>            夹爪绝对目标\n"
        "  ja <a0> ... <a6> [0|1]  全关节目标，默认 mode=1\n"
        "  move <a0> ... <a6>      固件 mode=1 插补，同 ja 默认模式\n"
        "  zero                    固件归零（可能产生大幅动作）\n"
        "  dumping <0..80000>      damping all 的原始阻尼兼容名，不是使能\n"
        "  sleep <秒数>            等待；help / quit / exit\n"
        "默认仅允许 J4/J5/J6；启用其他关节需要显式 --joints。\n"
        "Ctrl+C 退出程序；不会停止已被固件接收的运动，不是急停。\n";
}
bool Controller::execute(Command c) {
    checkScope(c, options_);
    compensateJ4(c, options_.j4BiasDeg);
    switch (c.kind) {
    case Kind::Help: help(); return true;
    case Kind::Quit: return true;
    case Kind::Limits:
        for (int i = 0; i < 7; ++i) out() << "J" << i << ": [" << limits[i].min << ", " << limits[i].max << "] " << (i == 6 ? "SDK单位(未标定)" : "deg") << "\n";
        out() << "限位沿用原程序；不代表碰撞保护。\n"; return true;
    case Kind::Sleep: return pause(c.value);
    case Kind::HoldCurrent: case Kind::HoldCurrentCheck: return checkCurrentHold(c);
    case Kind::Status: case Kind::Doctor: {
        waitAngles(true);
        bool ok = printStatus();
        if (c.kind == Kind::Doctor) out() << (ok ? "[链路] 收到新鲜完整角度反馈。\n" : "[链路] 无新鲜完整反馈；检查网卡、DDS domain、话题、网线和机械臂服务。\n");
        return ok;
    }
    case Kind::Watch: {
        if (options_.dryRun) { err() << "离线模式无法 watch\n"; return false; }
        waitAngles();
        auto start = Clock::now();
        bool ok = false;
        do {
            ok = printStatus();
            if (!pause(0.5)) break;
        } while (!interrupted && (c.value == 0 || std::chrono::duration<double>(Clock::now() - start).count() < c.value));
        return ok && !interrupted;
    }
    case Kind::TestJoint: {
        if (options_.dryRun) { err() << "test-joint 需要实测基线，离线模式不发送测试目标\n"; return false; }
        if (!motionReady(c.id)) return false;
        auto baseline = *snapshot().angles;
        const double target = baseline[c.id] + c.value;
        checkAngle(c.id, baseline[c.id]); // 返程目标也必须满足同一限位
        checkAngle(c.id, target);
        out() << "[test] J" << c.id << " baseline=" << baseline[c.id] << " target=" << target
                  << "；到位后返回实测起点；两段均要求位置闭环\n";
        Command leg; leg.kind = Kind::Send; leg.function = 1; leg.motion = true;
        leg.data = {{"id",c.id},{"angle",target},{"delay_ms",0}};
        Command returnLeg = leg;
        returnLeg.data["angle"] = baseline[c.id];
        compensateJ4(returnLeg, options_.j4BiasDeg); // 发送去程之前同时校验返程
        if (!send(leg, true, &baseline)) return false;
        leg.data["angle"] = baseline[c.id];
        if (!send(leg, true, &baseline)) return false;
        out() << "[test-ok] J" << c.id << " 往返闭环验证完成，起点=" << baseline[c.id]
                  << " 回读=" << (*snapshot().angles)[c.id] << "\n";
        return true;
    }
    case Kind::Nudge: {
        if (!waitAngles()) { err() << "[未发送] nudge 必须读取实时角度，离线模式也不能假定零位\n"; return false; }
        auto s = snapshot();
        double target = (*s.angles)[c.id] + c.value;
        checkAngle(c.id, target);
        c.kind = Kind::Send; c.function = 1;
        c.data = {{"id",c.id},{"angle",target},{"delay_ms",0}};
        return send(c);
    }
    case Kind::Send: return send(c, false, nullptr, true);
    }
    return false;
}
} // namespace d1
