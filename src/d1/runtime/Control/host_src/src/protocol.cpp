#include "protocol.hpp"
#include <boost/json/src.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace d1 {
namespace {
void require(bool ok, const std::string& message) {
    if (!ok) throw std::invalid_argument(message);
}
void count(const std::vector<std::string>& t, size_t lo, size_t hi, const char* usage) {
    require(t.size() >= lo && t.size() <= hi, std::string("用法: ") + usage);
}
std::optional<int> field(const json::object& obj, const char* name) {
    const auto* v = obj.if_contains(name);
    if (!v || !v->is_int64()) return {};
    const auto n = v->as_int64();
    if (n < 0 || n > std::numeric_limits<int>::max()) return {};
    return static_cast<int>(n);
}
std::optional<int> bit(const json::object& obj, const char* name) {
    auto n = field(obj, name);
    return n && *n <= 1 ? n : std::nullopt;
}
}
int integer(const std::string& text, int min, int max) {
    size_t end = 0;
    long long value;
    try { value = std::stoll(text, &end); }
    catch (...) { throw std::invalid_argument("整数无效: " + text); }
    require(end == text.size() && value >= min && value <= max,
            "整数超出范围或含多余字符: " + text + " [" + std::to_string(min) + "," + std::to_string(max) + "]");
    return static_cast<int>(value);
}
double number(const std::string& text) {
    size_t end = 0;
    double value;
    try { value = std::stod(text, &end); }
    catch (...) { throw std::invalid_argument("数值无效: " + text); }
    require(end == text.size() && std::isfinite(value), "需要有限数值: " + text);
    return value;
}
void checkAngle(int id, double angle) {
    require(id >= 0 && id < 7, "关节编号必须为 0..6");
    require(std::isfinite(angle) && angle >= limits[id].min && angle <= limits[id].max,
            "J" + std::to_string(id) + " 目标超出本地限位 [" +
            std::to_string(limits[id].min) + "," + std::to_string(limits[id].max) + "]；未发送");
}
void checkFeedbackAngle(int id, double angle) {
    require(id >= 0 && id < 7, "关节编号必须为 0..6");
    const double tolerance = id < 6 ? armFeedbackToleranceDeg : 0.;
    const double roundoff = id < 6 ? std::numeric_limits<float>::epsilon() *
        std::max({1.0, std::abs(limits[id].min), std::abs(limits[id].max)}) : 0.;
    std::ostringstream message;
    message << std::setprecision(10) << "J" << id << " 实际反馈=" << angle
            << (id < 6 ? "°" : " SDK") << " 超出反馈允许范围：目标限位=["
            << limits[id].min << ", " << limits[id].max << "]，反馈抖动容差="
            << tolerance << (id < 6 ? "°" : " SDK")
            << "；停止后续操作，不修改反馈或扩大目标限位";
    require(std::isfinite(angle) && angle >= limits[id].min - tolerance - roundoff &&
            angle <= limits[id].max + tolerance + roundoff, message.str());
}
std::vector<std::string> words(const std::string& line) {
    std::istringstream stream(line);
    std::vector<std::string> out;
    for (std::string word; stream >> word;) out.push_back(word);
    return out;
}
Command compensateJ4(const Command& command, std::optional<double> bias) {
    Command wire = command;
    if (!bias) return wire;
    require(std::isfinite(*bias) && std::abs(*bias) <= 1, "J4 补偿偏差必须为有限值且不超过 ±1 度");
    require(command.kind != Kind::HoldCurrent && command.kind != Kind::HoldCurrentCheck,
            "hold-current 必须关闭 J4 补偿；禁止将实测同位目标改成偏置目标");
    if (command.kind != Kind::Send) return wire;
    std::string key;
    if (command.function == 1 && command.data.at("id").as_int64() == 4) key = "angle";
    if (command.function == 2) key = "angle4";
    if (key.empty()) return wire;
    const double desired = command.data.at(key).to_number<double>();
    checkAngle(4, desired);
    // 与协议三位小数一致；校验实际下发值，不裁剪越界目标。
    const double target = std::round((desired - *bias) * 1000.) / 1000.;
    checkAngle(4, target);
    require(std::abs(desired) <= 85 && std::abs(target) <= 85,
            "J4 补偿模式要求原目标及下发角均在已测试的 ±85 度范围内；未发送");
    wire.data[key] = target;
    return wire;
}
Command parseCommand(const std::string& line) {
    auto t = words(line);
    require(!t.empty(), "命令为空");
    Command c;
    const auto& name = t[0];
    if (name == "help" || name == "limits" || name == "status" || name == "doctor" || name == "quit" || name == "exit") {
        count(t, 1, 1, "help | limits | status | doctor | quit");
        c.kind = name == "help" ? Kind::Help : name == "limits" ? Kind::Limits :
                 name == "status" ? Kind::Status : name == "doctor" ? Kind::Doctor : Kind::Quit;
    } else if (name == "watch" || name == "sleep") {
        count(t, name == "sleep" ? 2 : 1, 2, "watch [秒数] | sleep <秒数>");
        c.kind = name == "watch" ? Kind::Watch : Kind::Sleep;
        c.value = t.size() == 2 ? number(t[1]) : 0;
        require(c.value >= 0 && c.value <= 3600, "秒数必须为 0..3600；watch 0 持续显示直到 Ctrl+C");
    } else if (name == "hold-current" || name == "hold-current-check") {
        count(t, 2, 2, "hold-current[-check] <id|all>（同位恢复待机载修复核验；check 仅只读检查）");
        c.kind = name == "hold-current" ? Kind::HoldCurrent : Kind::HoldCurrentCheck;
        c.motion = name == "hold-current";
        c.id = t[1] == "all" ? -1 : integer(t[1], 0, 6);
    } else if (name == "nudge" || name == "test-joint") {
        count(t, 3, 3, "nudge <id> <增量>（度，J6 为未标定 SDK 单位）");
        c.kind = name == "nudge" ? Kind::Nudge : Kind::TestJoint; c.motion = true;
        c.id = integer(t[1], 0, 6); c.value = number(t[2]);
        if (name == "test-joint") require(std::abs(c.value) >= 0.8 && std::abs(c.value) <= 2, "test-joint 增量绝对值必须在 0.8..2 之间");
        require(std::abs(c.value) <= 10, "nudge 单次增量最多 ±10");
    } else {
        c.kind = Kind::Send;
        if (name == "power") {
            count(t, 2, 2, "power on|off");
            require(t[1] == "on" || t[1] == "off" || t[1] == "1" || t[1] == "0", "power 只接受 on/off/1/0");
            c.function = 6; c.data["power"] = (t[1] == "on" || t[1] == "1") ? 1 : 0;
        } else if (name == "enable" || name == "release") {
            throw std::invalid_argument("enable/release 已停用：funcode 4/5 是原始阻尼，不是位置使能/硬件卸力证明。明确阻尼用 damping <id|all> <value>；同位恢复用 hold-current <id|all>，须先核对机载零距离处理");
        } else if (name == "damping") {
            count(t, 3, 3, "damping <id|all> <0..80000>（原始阻尼；值与硬件扭矩模式未独立确认）");
            c.function = t[1] == "all" ? 5 : 4;
            if (c.function == 4) c.data["id"] = integer(t[1], 0, 6);
            c.data["mode"] = integer(t[2], 0, 80000);
        } else if (name == "dumping") {
            count(t, 2, 2, "dumping <0..80000>（原始固件 mode，非通用使能值）");
            c.function = 5; c.data["mode"] = integer(t[1], 0, 80000);
        } else if (name == "zero") {
            count(t, 1, 1, "zero"); c.function = 7; c.motion = true;
        } else if (name == "j" || name == "grip") {
            bool grip = name == "grip";
            count(t, grip ? 2 : 3, grip ? 2 : 4, "j <id> <角度> [delay_ms] | grip <0..65>");
            int id = grip ? 6 : integer(t[1], 0, 6);
            double angle = number(t[grip ? 1 : 2]); checkAngle(id, angle);
            c.function = 1; c.motion = true;
            c.data = {{"id", id}, {"angle", angle}, {"delay_ms", t.size() == 4 ? integer(t[3], 0, 32767) : 0}};
        } else if (name == "ja" || name == "move") {
            count(t, 8, name == "ja" ? 9 : 8, "ja <a0> ... <a6> [mode 0|1] | move <a0> ... <a6>");
            c.function = 2; c.motion = true;
            c.data["mode"] = t.size() == 9 ? integer(t[8], 0, 1) : 1;
            for (int i = 0; i < 7; ++i) {
                double angle = number(t[i + 1]); checkAngle(i, angle);
                c.data["angle" + std::to_string(i)] = angle;
            }
        } else if (name == "raw" || name == "demo") {
            throw std::invalid_argument("已移除 raw/demo；请使用有参数校验的 j、ja、nudge 或 --exec 命令序列");
        } else throw std::invalid_argument("未知命令: " + name + "（help 查看帮助）");
    }
    return c;
}
std::string encode(const Command& c, int seq) {
    require(c.kind == Kind::Send, "仅发送命令可编码");
    require(seq > 0 && seq != 10, "seq 必须为正整数且避开主动反馈序号 10");
    // 固件兼容：采用 SDK 示例的普通十进制，避免 Boost.JSON 将 double 写成 1.1E1。
    std::ostringstream wire;
    wire.imbue(std::locale::classic());
    wire << "{\"seq\":" << seq << ",\"address\":1,\"funcode\":" << c.function;
    if (!c.data.empty()) {
        wire << ",\"data\":{";
        bool first = true;
        for (const auto& item : c.data) {
            if (!first) wire << ',';
            first = false;
            wire << json::serialize(json::value(item.key())) << ':';
            if (item.value().is_double()) {
                double n = item.value().as_double();
                if (std::floor(n) == n) wire << static_cast<long long>(n);
                else wire << std::fixed << std::setprecision(3) << n;
            } else wire << json::serialize(item.value());
        }
        wire << '}';
    }
    wire << '}';
    return wire.str();
}
std::optional<Feedback> parseFeedback(const std::string& text, bool allowPartialAngles) {
    boost::system::error_code ec;
    auto value = json::parse(text, ec);
    if (ec || !value.is_object()) return {};
    auto& obj = value.as_object();
    auto address = field(obj, "address");
    auto function = field(obj, "funcode");
    if (!function) function = field(obj, "code");
    auto* data = obj.if_contains("data");
    if (!address || !function || !data || !data->is_object()) return {};
    auto& d = data->as_object();
    Feedback f; f.address = *address; f.function = *function;
    f.seq = field(obj, "seq").value_or(-1);
    if (f.address == 2 && f.function == 1) {
        Angles a{};
        for (int i = 0; i < 7; ++i) {
            auto* v = d.if_contains("angle" + std::to_string(i));
            if (!v || !v->is_number()) {
                if (!allowPartialAngles) return {};
                a[i] = std::numeric_limits<double>::quiet_NaN();
                continue;
            }
            a[i] = v->to_number<double>();
            if (!std::isfinite(a[i]) && !allowPartialAngles) return {};
        }
        f.angles = a;
    } else if (f.address == 2 && f.function == 3) {
        f.enable = bit(d, "enable_status"); f.power = bit(d, "power_status");
        f.error = field(d, "error_status");
        if (!f.enable && !f.power && !f.error) return {};
    } else if (f.address == 3 && f.seq > 0) {
        if (f.function == 1) f.received = bit(d, "recv_status");
        if (f.function == 2) f.executed = bit(d, "exec_status");
        if (!f.received && !f.executed) return {};
    } else return {};
    return f;
}
void State::update(const Feedback& f, Clock::time_point now) {
    if (f.angles) {
        if (angles) angleIntervalMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - angleTime).count();
        ++angleFrames; angles = f.angles; angleTime = now;
        acceptedJoints.fill(true); rejectedJoints.fill(false);
        for (int joint = 0; joint < 7; ++joint) {
            jointAngles[joint] = (*f.angles)[joint]; jointTimes[joint] = now;
            ++jointSequences[joint];
        }
    }
    if (f.enable) { enable = f.enable; enableTime = now; }
    if (f.power) { power = f.power; powerTime = now; }
    if (f.error) { error = f.error; errorTime = now; }
}
bool State::freshAngles(Clock::time_point now) const {
    return angles && now - angleTime <= std::chrono::seconds(2);
}
void checkFeedbackMaxSpeed(double degreesPerSecond) {
    require(std::isfinite(degreesPerSecond) && degreesPerSecond > 0,
            "--feedback-max-speed 需要正有限数值（deg/s）");
}
std::optional<json::object> updateFilteredFeedback(State& state, const Feedback& feedback,
    Clock::time_point receivedAt, std::optional<double> maxSpeedDegS, int recoveryTimeoutMs) {
    if (!maxSpeedDegS || !feedback.angles) {
        state.update(feedback, receivedAt);
        return {};
    }
    checkFeedbackMaxSpeed(*maxSpeedDegS);
    const auto& incoming = *feedback.angles;
    const auto previousPending = state.feedbackPendingSince;
    state.acceptedJoints.fill(false); state.rejectedJoints.fill(false);
    json::array rejections;
    bool anyAccepted = false;
    state.feedbackAcceptanceMethod = "last_accepted_cone";
    for (int joint = 0; joint < 7; ++joint) {
        const double value = incoming[joint];
        const bool finite = std::isfinite(value);
        bool bounded = finite;
        if (bounded) {
            try { checkFeedbackAngle(joint, value); }
            catch (const std::invalid_argument&) { bounded = false; }
        }
        const bool known = bool(state.jointAngles[joint]);
        bool accepted = false;
        std::string method = known ? "last_accepted_cone" : "initial_valid";
        json::object reason{{"joint", joint}};
        if (!finite) {
            reason["reason"] = "nonfinite_or_missing_angle";
        } else if (!known) {
            accepted = bounded;
            if (!bounded) {
                reason["reason"] = "invalid_initial_angle";
                reason["incoming"] = value;
            }
        } else if (joint == 6) {
            // J6 is not degrees. Preserve existing hard-limit behavior: expose
            // a finite actual value to checkFeedbackAngle/healthy, never hide
            // a true J6 limit violation behind an indefinite pending state.
            accepted = true;
            method = "j6_finite_feedback";
        } else {
            const double dt = std::chrono::duration<double>(receivedAt - state.jointTimes[joint]).count();
            const double delta = std::abs(value - *state.jointAngles[joint]);
            const double threshold = 3. * *maxSpeedDegS * dt;
            accepted = dt >= 0 && delta <= threshold;
            if (!accepted) {
                reason["reason"] = "angular_jump";
                reason["previous_deg"] = *state.jointAngles[joint];
                reason["incoming_deg"] = value;
                reason["delta_deg"] = delta;
                reason["dt_s"] = dt;
                reason["threshold_deg"] = threshold;
                if (bounded && state.jointPendingSince[joint] && state.jointRecoveryCandidates[joint]) {
                    const double rawDt = std::chrono::duration<double>(
                        receivedAt - state.jointRecoveryCandidateTimes[joint]).count();
                    if (rawDt > 0 && std::abs(value - *state.jointRecoveryCandidates[joint]) <=
                                      3. * *maxSpeedDegS * rawDt) {
                        accepted = true;
                        method = "two_valid_raw_frames";
                    }
                }
            }
        }
        if (accepted) {
            anyAccepted = true;
            state.acceptedJoints[joint] = true;
            state.jointAngles[joint] = value;
            state.jointTimes[joint] = receivedAt;
            ++state.jointSequences[joint];
            state.jointPendingSince[joint].reset();
            state.jointRecoveryCandidates[joint].reset();
            state.jointAcceptanceMethods[joint] = method;
            if (method == "two_valid_raw_frames" || !known) state.feedbackAcceptanceMethod = method;
        } else {
            state.rejectedJoints[joint] = true;
            if (!state.jointPendingSince[joint]) state.jointPendingSince[joint] = receivedAt;
            if (bounded && joint < 6) {
                state.jointRecoveryCandidates[joint] = value;
                state.jointRecoveryCandidateTimes[joint] = receivedAt;
            } else state.jointRecoveryCandidates[joint].reset();
            reason["source_seq"] = feedback.seq;
            reason["max_speed_deg_s"] = *maxSpeedDegS;
            reason["multiplier"] = 3;
            rejections.push_back(reason);
        }
    }
    const bool anyPending = std::any_of(state.jointPendingSince.begin(), state.jointPendingSince.end(),
        [](const auto& pending) { return bool(pending); });
    if (anyPending) {
        if (!state.feedbackPendingSince) state.feedbackPendingSince = receivedAt;
    } else {
        if (previousPending) state.feedbackPauseTotalSeconds +=
            std::chrono::duration<double>(receivedAt - *previousPending).count();
        state.feedbackPendingSince.reset();
    }
    if (anyAccepted) {
        if (state.angleFrames && receivedAt - state.angleTime > std::chrono::milliseconds(recoveryTimeoutMs)) {
            ++state.allFeedbackGapCount;
            state.lastAllFeedbackGapStart = state.angleTime;
            state.lastAllFeedbackGapEnd = receivedAt;
        }
        if (state.angleFrames) state.angleIntervalMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(receivedAt - state.angleTime).count();
        ++state.angleFrames;
        state.angleTime = receivedAt;
    }
    if (std::all_of(state.jointAngles.begin(), state.jointAngles.end(),
                    [](const auto& value) { return bool(value); })) {
        Angles merged;
        for (int joint = 0; joint < 7; ++joint) merged[joint] = *state.jointAngles[joint];
        state.angles = merged;
    } else state.angles.reset();
    auto statuses = feedback; statuses.angles.reset();
    state.update(statuses, receivedAt);
    if (rejections.empty()) return {};
    auto result = rejections.front().as_object();
    result["joint_rejections"] = rejections;
    return result;
}
json::object jointFeedbackMetadata(const State& state) {
    json::array times, sequences, pending, accepted, rejected, merged, methods;
    for (int joint = 0; joint < 7; ++joint) {
        if (state.jointAngles[joint]) {
            times.push_back(std::chrono::duration<double>(state.jointTimes[joint].time_since_epoch()).count());
            merged.push_back(*state.jointAngles[joint]);
        } else { times.push_back(nullptr); merged.push_back(nullptr); }
        sequences.push_back(state.jointSequences[joint]);
        pending.push_back(bool(state.jointPendingSince[joint]));
        methods.emplace_back(state.jointAcceptanceMethods[joint]);
        if (state.acceptedJoints[joint]) accepted.push_back(joint);
        if (state.rejectedJoints[joint]) rejected.push_back(joint);
    }
    json::object metadata{{"joint_times_s", times}, {"joint_sequences", sequences}, {"joint_pending", pending},
        {"accepted_joints", accepted}, {"rejected_joints", rejected}, {"merged_angles", merged},
        {"joint_acceptance_methods", methods}, {"feedback_pending", bool(state.feedbackPendingSince)},
        {"all_feedback_gap_count", state.allFeedbackGapCount}};
    metadata["last_all_feedback_gap_start_s"] = state.lastAllFeedbackGapStart ?
        json::value(std::chrono::duration<double>(state.lastAllFeedbackGapStart->time_since_epoch()).count()) : json::value(nullptr);
    metadata["last_all_feedback_gap_end_s"] = state.lastAllFeedbackGapEnd ?
        json::value(std::chrono::duration<double>(state.lastAllFeedbackGapEnd->time_since_epoch()).count()) : json::value(nullptr);
    return metadata;
}
json::object makeFeedbackMarker(const State& state, const json::value& rawMessage,
    Clock::time_point receivedAt, const std::optional<json::object>& rejection) {
    json::object marker = rejection.value_or(json::object{{"joint_rejections", json::array{}}});
    for (const auto& field : jointFeedbackMetadata(state)) marker[field.key()] = field.value();
    marker["acceptance_method"] = state.feedbackAcceptanceMethod;
    marker["message"] = rawMessage;
    marker["received_monotonic_s"] = std::chrono::duration<double>(receivedAt.time_since_epoch()).count();
    marker["pause_total_s"] = state.feedbackPauseTotalSeconds;
    return marker;
}
json::array updateJointFeedbackStaleness(State& state, Clock::time_point now) {
    json::array events;
    for (int joint = 0; joint < 7; ++joint) {
        const bool known = bool(state.jointAngles[joint]);
        const bool stale = known ? now - state.jointTimes[joint] > std::chrono::milliseconds(600) :
                                   state.angleFrames > 0;
        if (stale == state.jointStaleReported[joint]) continue;
        state.jointStaleReported[joint] = stale;
        json::object event{{"joint", joint}, {"state", stale ? "stale" : "recovered"},
            {"event_monotonic_s", std::chrono::duration<double>(now.time_since_epoch()).count()}};
        event["age_s"] = known ? json::value(std::chrono::duration<double>(now - state.jointTimes[joint]).count()) :
                                  json::value(nullptr);
        events.push_back(event);
    }
    return events;
}
FeedbackRecoveryState feedbackRecoveryState(const State& state, Clock::time_point now,
    Clock::time_point initialDeadline, bool allowUnstartedPosition, int timeoutMs,
    std::optional<unsigned long> gapCountAtEntry) {
    using namespace std::chrono;
    if ((state.error && *state.error != 0) || (state.power && *state.power != 1) ||
        (state.enable && *state.enable != 1 && !(allowUnstartedPosition && *state.enable == 0)))
        return FeedbackRecoveryState::UnsafeStatus;
    if (gapCountAtEntry && state.allFeedbackGapCount > *gapCountAtEntry)
        return FeedbackRecoveryState::TimedOut;
    bool ready = bool(state.angles), anyKnown = false, allHardStale = true;
    for (int joint = 0; joint < 7; ++joint) {
        if (!state.jointAngles[joint]) { ready = false; continue; }
        anyKnown = true;
        if (state.jointTimes[joint] > now) return FeedbackRecoveryState::UnsafeStatus;
        const auto age = now - state.jointTimes[joint];
        if (age < milliseconds(timeoutMs)) allHardStale = false;
        if (age > milliseconds(600) || state.jointPendingSince[joint]) ready = false;
    }
    if (ready) return FeedbackRecoveryState::Ready;
    if ((anyKnown && allHardStale) || (!anyKnown && now >= initialDeadline))
        return FeedbackRecoveryState::TimedOut;
    return FeedbackRecoveryState::Waiting;
}
void checkPositionBootstrap(const Command& wire, const State& s, Clock::time_point now) {
    using namespace std::chrono;
    require(wire.kind == Kind::Send && wire.motion && wire.function == 1,
            "位置启动只允许正常单轴 funcode 1 非零目标；不允许全轴同位分量、回零、阻尼、同位恢复或相对测试");
    auto fresh = [&](Clock::time_point t) { return now >= t && now - t <= milliseconds(250); };
    require(s.angles && fresh(s.angleTime) && fresh(s.powerTime) && fresh(s.enableTime) && fresh(s.errorTime),
            "位置启动需要 250 ms 内完整实际角度及 power/enable/error 状态反馈；未发送");
    for (int joint = 0; joint < 7; ++joint)
        require(s.jointAngles[joint] && fresh(s.jointTimes[joint]),
                "位置启动需要每轴 250 ms 内实际角度反馈；J" + std::to_string(joint) + " 未满足，未发送");
    require(s.power == 1 && s.enable == 0 && s.error == 0,
            "位置启动需要 power=1/enable=0/error=0；未发送");
    for (int i = 0; i < 7; ++i) checkFeedbackAngle(i, (*s.angles)[i]);
    require((*s.angles)[6] <= 50, "位置启动 J6 实际反馈超过机载目标上限 50；未发送");
    bool nonzero = false;
    for (int i = 0; i < 7; ++i) {
        if (wire.function == 1 && i != wire.data.at("id").as_int64()) continue;
        double target = wire.data.at(wire.function == 1 ? "angle" : "angle" + std::to_string(i)).to_number<double>();
        checkAngle(i, target);
        require(i != 6 || target <= 50, "位置启动 J6 目标超过机载上限 50；未发送");
        // Use the encoder itself: std::round(x*1000) can disagree with iostream
        // formatting at binary floating half-boundaries (e.g. 6.5005 -> 6.500).
        const auto key = wire.function == 1 ? "angle" : "angle" + std::to_string(i);
        auto current = wire;
        current.data[key] = (*s.angles)[i];
        const auto encodedTarget = json::parse(encode(wire, 1)).as_object()
            .at("data").as_object().at(key).to_number<double>();
        const auto encodedCurrent = json::parse(encode(current, 1)).as_object()
            .at("data").as_object().at(key).to_number<double>();
        nonzero |= encodedTarget != encodedCurrent;
    }
    require(nonzero, "位置启动拒绝同位/量化后零位移目标；必须使用已批准路径中的首个非零目标");
}
void checkHoldState(const State& s, Clock::time_point now) {
    using namespace std::chrono;
    require(s.angles && now >= s.angleTime && now - s.angleTime <= milliseconds(250),
            "hold-current 需要年龄不超过 250 ms 的完整实际角度反馈；未发送");
    for (int joint = 0; joint < 7; ++joint)
        require(s.jointAngles[joint] && now >= s.jointTimes[joint] &&
                now - s.jointTimes[joint] <= milliseconds(250),
                "hold-current 需要每轴 250 ms 内实际角度反馈；J" + std::to_string(joint) + " 未满足，未发送");
    require(s.power == 1 && now >= s.powerTime && now - s.powerTime <= seconds(2),
            "hold-current 需要新鲜 power_status=1（仅软件状态，非物理供电证明）；未发送");
    require(s.error == 0 && now >= s.errorTime && now - s.errorTime <= seconds(2),
            "hold-current 需要新鲜 error_status=0；未发送");
    for (int i = 0; i < 7; ++i) checkFeedbackAngle(i, (*s.angles)[i]);
    require((*s.angles)[6] <= 50,
            "hold-current 的 J6 反馈超过已保存机载目标上限 50；禁止依赖固件钳位，未发送");
}
void checkHoldDrift(const Angles& baseline, const Angles& current) {
    for (int i = 0; i < 7; ++i) {
        require(std::isfinite(baseline[i]) && std::isfinite(current[i]) &&
                std::abs(current[i] - baseline[i]) <= (i == 6 ? 0.3 : 0.2),
                "hold-current 基线未稳定：J" + std::to_string(i) + " 变化超出同位恢复窗口；未发送后续目标");
    }
}
Command makeHoldCurrent(const State& s, Clock::time_point now, int id) {
    require(id >= 0 && id < 7, "hold-current 关节编号必须为 0..6");
    checkHoldState(s, now);
    // Feedback tolerance never authorizes an out-of-range same-position target.
    checkAngle(id, (*s.angles)[id]);
    Command c; c.kind = Kind::Send; c.function = 1; c.motion = true;
    // No position offset, wrapping or clamping. The wire encoder rounds to
    // 0.001 SDK unit. This requested delay is not a measured hardware speed.
    c.data = {{"id",id},{"angle",(*s.angles)[id]},{"delay_ms",1000}};
    return c;
}
bool PositionCheck::observe(const State& s) {
    if (!s.angles) return settled >= 3;
    for (int i = 0; i < 7; ++i) checkFeedbackAngle(i, (*s.angles)[i]);
    bool atTarget = true;
    maxError = 0;
    for (int i = 0; i < 7; ++i) {
        if (!allowed[i]) {
            protectedDrift = std::max(protectedDrift, std::abs((*s.angles)[i] - baseline[i]));
            if (protectedDrift > 0.5) violated = true;
        }
        if (targets[i]) {
            double error = std::abs((*s.angles)[i] - *targets[i]);
            maxError = std::max(maxError, error);
            // Feedback originates as float32. For example -74.7f at a -75
            // target differs by 0.300003, although the intended boundary is
            // 0.3 degrees. Allow only representation-scale rounding, not an
            // additional physical positioning tolerance.
            const double roundoff = std::numeric_limits<float>::epsilon() *
                std::max({1.0, std::abs((*s.angles)[i]), std::abs(*targets[i])});
            if (error > (i == 6 ? 0.5 : 0.3) + roundoff) atTarget = false;
        }
    }
    // Inspect actual drift on every read, but consume an arrival sample only
    // once all seven axes have independently advanced after this command.
    // A fresh partial frame cannot count another axis's held value again.
    if (!atTarget || violated) settled = 0;
    for (int joint = 0; joint < 7; ++joint)
        if (!s.jointAngles[joint] || s.jointTimes[joint] <= sentAt ||
            s.jointTimes[joint] <= lastJointSamples[joint]) return settled >= 3;
    lastSample = s.angleTime;
    lastJointSamples = s.jointTimes;
    settled = atTarget && !violated ? settled + 1 : 0;
    return settled >= 3;
}
void Ack::update(const Feedback& f) {
    if (f.address != 3 || f.seq != seq) return;
    if (f.received && received != 0) received = f.received;
    if (f.executed && executed != 0) executed = f.executed;
}
bool Ack::rejected() const { return received == 0 || executed == 0; }
bool Ack::accepted(bool executionRequired) const {
    return !rejected() && (executed == 1 || (!executionRequired && received == 1));
}
} // namespace d1
