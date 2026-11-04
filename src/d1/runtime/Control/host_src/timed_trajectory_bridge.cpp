// JSON-lines transport only. No homing, power, enable, fallback or implicit
// cancellation. "published" confirms DDS Write, never motor adoption/arrival.
#include "src/protocol.hpp"
#include "msg/ArmString_.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <boost/json.hpp>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
namespace json = boost::json;
using Message = unitree_arm::msg::dds_::ArmString_;
using namespace unitree::robot;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }

class JsonOutput {
public:
    JsonOutput() {
        // SDK printf/cout/log output belongs on stderr. Reserve the original
        // stdout descriptor for our complete, mutex-protected JSON envelopes.
        fd_ = ::dup(STDOUT_FILENO);
        if (fd_ < 0 || ::dup2(STDERR_FILENO, STDOUT_FILENO) < 0)
            throw std::runtime_error("Cannot reserve JSON stdout descriptor");
    }
    ~JsonOutput() { if (fd_ >= 0) ::close(fd_); }
    bool emit(const char* kind, json::value message) noexcept {
        try {
            const auto now = std::chrono::steady_clock::now().time_since_epoch();
            json::object envelope;
            envelope["kind"] = kind;
            envelope["host_monotonic_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            envelope["message"] = std::move(message);
            const std::string line = json::serialize(envelope) + '\n';
            std::lock_guard<std::mutex> lock(mutex_);
            std::size_t done = 0;
            while (done < line.size()) {
                const auto written = ::write(fd_, line.data()+done, line.size()-done);
                if (written < 0 && errno == EINTR) continue;
                if (written <= 0) { failed_ = true; return false; }
                done += static_cast<std::size_t>(written);
            }
            return true;
        } catch (...) { failed_ = true; return false; }
    }
    bool error(const char* where, const std::string& reason, const std::string& raw = "") noexcept {
        try {
            json::object message{{"where",where},{"error",reason}};
            if (!raw.empty()) message["raw"] = raw;
            return emit("error", std::move(message));
        } catch (...) { failed_ = true; return false; }
    }
    bool failed() const { return failed_.load(); }
private:
    int fd_ = -1;
    std::mutex mutex_;
    std::atomic<bool> failed_{false};
};

void validate(const json::value& message) {
    if (!message.is_object()) throw std::invalid_argument("stdin message must be a JSON object");
    const auto& object = message.as_object();
    const auto* schema = object.if_contains("schema");
    const auto* type = object.if_contains("type");
    if (!schema || !schema->is_string() || schema->as_string() != "d1_timed_trajectory_v1")
        throw std::invalid_argument("schema must be d1_timed_trajectory_v1");
    if (!type || !type->is_string()) throw std::invalid_argument("type is required");
    const auto& kind = type->as_string();
    if (kind != "chunk" && kind != "cancel" && kind != "pause" && kind != "resume")
        throw std::invalid_argument("type must be chunk, cancel, pause or resume");
    // Point shape, limits, sequencing, digest, gripper preservation and timing
    // admission are authoritative on the receiving controller. All fields,
    // including message_seq, are forwarded unchanged; no alternate seq is made.
}

class Bridge {
public:
    explicit Bridge(JsonOutput& output) : output_(output),
        publisher_("rt/d1_TimedTrajectory"), state_("rt/d1_TrajectoryState"),
        feedback_("rt/arm_Feedback") {
        publisher_.InitChannel();
        state_.InitChannel([this](const void* raw) { received("state", raw); }, 32);
        feedback_.InitChannel([this](const void* raw) { received("feedback", raw); }, 32);
    }
    ~Bridge() {
        try { state_.CloseChannel(); } catch (...) {}
        try { feedback_.CloseChannel(); } catch (...) {}
        try { publisher_.CloseChannel(); } catch (...) {}
    }
    bool publish(const json::value& value) {
        Message message;
        message.data_() = json::serialize(value);
        if (!publisher_.Write(message, 0)) {
            output_.error("publisher", "DDS Write returned false; no further stdin messages published");
            return false;
        }
        return output_.emit("published", value);
    }
private:
    void received(const char* kind, const void* raw) noexcept {
        try {
            const auto& payload = static_cast<const Message*>(raw)->data_();
            boost::system::error_code error;
            auto value = json::parse(payload, error);
            if (error) { output_.error(kind, error.message(), payload); return; }
            output_.emit(kind, std::move(value));
        } catch (const std::exception& error) { output_.error(kind,error.what()); }
        catch (...) { output_.error(kind,"Unknown receive callback exception"); }
    }
    JsonOutput& output_;
    ChannelPublisher<Message> publisher_;
    ChannelSubscriber<Message> state_, feedback_;
};
}

int main(int argc, char** argv) {
    std::signal(SIGPIPE, SIG_IGN);
    struct sigaction action{};
    action.sa_handler = onSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    try {
        JsonOutput output;
        try {
            std::string nic;
            int domain = 0;
            bool nicSeen = false, domainSeen = false;
            for (int i=1; i<argc; ++i) {
                const std::string argument = argv[i];
                if (argument == "--help" || argument == "-h") {
                    std::cerr << "d1_timed_bridge --nic enp99s0 [--domain 0..232]\n"
                                 "stdin: one d1_timed_trajectory_v1 chunk/cancel/pause/resume JSON object per line.\n";
                    return 0;
                }
                if (argument != "--nic" && argument != "--domain")
                    throw std::invalid_argument("Unknown option: "+argument);
                if (++i >= argc) throw std::invalid_argument("Missing value for "+argument);
                if (argument == "--nic") {
                    if (nicSeen) throw std::invalid_argument("--nic cannot be repeated");
                    nic = argv[i]; nicSeen = true;
                } else {
                    if (domainSeen) throw std::invalid_argument("--domain cannot be repeated");
                    domain = d1::integer(argv[i],0,232); domainSeen = true;
                }
            }
            if (!nicSeen || nic != "enp99s0")
                throw std::invalid_argument("--nic enp99s0 is required; no automatic interface selection");
            ChannelFactory::Instance()->Init(domain,nic);
            Bridge bridge(output);
            std::string line;
            while (!interrupted && !output.failed() && std::getline(std::cin,line)) {
                if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
                boost::system::error_code error;
                auto message = json::parse(line,error);
                if (error) { output.error("stdin",error.message(),line); return 2; }
                validate(message);
                if (!bridge.publish(message)) return 1;
            }
            // EOF closes DDS channels. It never generates cancel, pause, a
            // position command, power or enable. The sender owns lifecycle.
            return interrupted ? 130 : (output.failed() ? 1 : 0);
        } catch (const std::invalid_argument& error) {
            output.error("validation",error.what());return 2;
        } catch (const std::exception& error) {
            output.error("bridge",error.what());return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "d1_timed_bridge: " << error.what() << '\n';return 1;
    }
}
