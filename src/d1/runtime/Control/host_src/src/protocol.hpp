#pragma once
#include <boost/json.hpp>
#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace d1 {
namespace json = boost::json;
using Clock = std::chrono::steady_clock;
using Angles = std::array<double, 7>;
struct Limit { double min, max; };
// 保留原程序的本地限位；它们不是碰撞检测，也不是固件型号鉴别。
inline constexpr std::array<Limit, 7> limits{{
    {-135,135}, {-90,90}, {-90,90}, {-135,135}, {-90,90}, {-135,135}, {0,65}}};
int integer(const std::string& text, int min, int max);
double number(const std::string& text);
void checkAngle(int id, double angle);
// User-authorized measurement jitter allowance, not a command/physical limit change.
inline constexpr double armFeedbackToleranceDeg = 0.2;
void checkFeedbackAngle(int id, double angle);
std::vector<std::string> words(const std::string& line);

enum class Kind { Help, Limits, Status, Doctor, Watch, Sleep, Quit, Send, Nudge, TestJoint, HoldCurrent, HoldCurrentCheck };
struct Command {
    Kind kind = Kind::Help;
    int function = 0;
    json::object data;
    bool motion = false;
    double value = 0;
    int id = 0;
};
Command parseCommand(const std::string& line);
// 输入为期望物理角度；仅返回下发副本，位置判定必须继续使用原目标。
Command compensateJ4(const Command& command, std::optional<double> bias);
std::string encode(const Command& command, int seq);

struct Feedback {
    int address = 0, function = 0, seq = -1;
    std::optional<Angles> angles;
    std::optional<int> enable, power, error, received, executed;
};
// Default parsing requires seven finite angles. Filter mode retains valid axes
// and represents missing/invalid numeric axes as NaN for independent rejection.
std::optional<Feedback> parseFeedback(const std::string& text, bool allowPartialAngles = false);
struct State {
    std::optional<Angles> angles, servo;
    std::optional<int> enable, power, error;
    unsigned long angleFrames = 0;
    long angleIntervalMs = -1;
    Clock::time_point angleTime{}, servoTime{}, enableTime{}, powerTime{}, errorTime{};
    std::optional<Clock::time_point> feedbackPendingSince;
    double feedbackPauseTotalSeconds = 0;
    std::array<std::optional<double>, 7> jointAngles{};
    std::array<Clock::time_point, 7> jointTimes{}, jointRecoveryCandidateTimes{};
    std::array<unsigned long, 7> jointSequences{};
    std::array<std::optional<Clock::time_point>, 7> jointPendingSince{};
    std::array<std::optional<double>, 7> jointRecoveryCandidates{};
    std::array<bool, 7> acceptedJoints{}, rejectedJoints{}, jointStaleReported{};
    std::array<std::string, 7> jointAcceptanceMethods{};
    unsigned long allFeedbackGapCount = 0;
    std::optional<Clock::time_point> lastAllFeedbackGapStart, lastAllFeedbackGapEnd;
    std::string feedbackAcceptanceMethod;
    void update(const Feedback& feedback, Clock::time_point now);
    bool freshAngles(Clock::time_point now) const;
};
void checkFeedbackMaxSpeed(double degreesPerSecond);
// Update each accepted axis independently. A rejected axis preserves its own
// angle, sequence and receive time; other axes and status fields still update.
// A returned diagnostic is not a terminal fault. Callers retain raw wire logs.
std::optional<json::object> updateFilteredFeedback(State& state, const Feedback& feedback,
    Clock::time_point receivedAt, std::optional<double> maxSpeedDegS, int recoveryTimeoutMs = 2000);
enum class FeedbackRecoveryState { Ready, Waiting, TimedOut, UnsafeStatus };
// Motion waits until all seven axes are fresh and nonpending. Timeout requires
// every axis to exceed its own hard age; a single stale axis is diagnostic only.
// initialDeadline bounds startup when no axis has ever been accepted.
// A gap counter captured at gate entry also detects an overdue source frame
// that arrived before the waiting thread could observe the hard-stale state.
FeedbackRecoveryState feedbackRecoveryState(const State& state, Clock::time_point now,
    Clock::time_point initialDeadline, bool allowUnstartedPosition = false, int timeoutMs = 2000,
    std::optional<unsigned long> gapCountAtEntry = {});
json::object jointFeedbackMetadata(const State& state);
json::object makeFeedbackMarker(const State& state, const json::value& rawMessage,
    Clock::time_point receivedAt, const std::optional<json::object>& rejection = {});
json::array updateJointFeedbackStaleness(State& state, Clock::time_point now);
// Same-position recovery uses the intersection of the CLI bounds and the saved
// 2026-10-23 controller target bounds. In particular J6 must be <=50 here:
// the controller silently clips a larger target. Never clip/wrap feedback.
// These checks validate an observed position, not hardware torque/mode status.
// A bootstrap is a real approved position movement, never a same-position enable command.
void checkPositionBootstrap(const Command& wire, const State& state, Clock::time_point now);
void checkHoldState(const State& state, Clock::time_point now);
void checkHoldDrift(const Angles& baseline, const Angles& current);
Command makeHoldCurrent(const State& state, Clock::time_point now, int id);
struct PositionCheck {
    Angles baseline{};
    std::array<std::optional<double>, 7> targets{};
    std::array<bool, 7> allowed{};
    Clock::time_point sentAt{}, lastSample{};
    std::array<Clock::time_point, 7> lastJointSamples{};
    int settled = 0;
    double protectedDrift = 0, maxError = 0;
    bool violated = false;
    bool observe(const State& state);
};
struct Ack {
    int seq = -1;
    std::optional<int> received, executed;
    void update(const Feedback& feedback);
    bool rejected() const;
    bool accepted(bool executionRequired) const;
};
} // namespace d1
