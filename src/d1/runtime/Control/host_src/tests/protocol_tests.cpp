#include "protocol.hpp"
#include <iostream>
#include <stdexcept>
using namespace d1;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main() {
    try {
        auto c = parseCommand("ja 0 -60 60 0 30 0 0");
        auto wire = json::parse(encode(c, 123)).as_object();
        check(wire.at("address") == 1 && wire.at("funcode") == 2, "SDK command envelope");
        check(wire.at("data").as_object().at("angle1").to_number<double>() == -60, "SDK angle mapping");
        check(!json::parse(encode(parseCommand("zero"),124)).as_object().contains("data"), "SDK zero has no data");
        for (auto text : {"j 5 nan", "j 2 91", "j 7 1", "j 1 1junk", "j 1 0 -1", "power typo", "enable", "enable 5", "release", "release 5", "enable 1 extra", "damping all -1", "damping 7 1", "damping all 80001", "hold-current 7", "hold-current", "ja 0 0 0 0 0 0 0 2", "move 0 0 0 0 0 0 0 40", "nudge 5 11"}) {
            bool rejected = false;
            try { parseCommand(text); } catch (const std::invalid_argument&) { rejected = true; }
            check(rejected, text);
        }
        State state;
        auto now = Clock::now();
        auto angle = parseFeedback(R"({"seq":10,"address":2,"funcode":1,"data":{"angle0":13,"angle1":-90,"angle2":88.2,"angle3":1.8,"angle4":11.6,"angle5":-3.5,"angle6":0.1}})");
        check(angle.has_value(), "valid feedback");
        state.update(*angle, now);
        check(state.freshAngles(now), "fresh complete angles");
        check(!state.freshAngles(now + std::chrono::seconds(3)), "stale angles rejected");
        check(!parseFeedback(R"({"address":2,"funcode":1,"data":{"angle0":1}})"), "partial angles cannot refresh");
        check(!parseFeedback(R"({"address":2,"funcode":1,"data":{},"angle0":1})"), "respect nested JSON");
        check(!parseFeedback("{broken}"), "malformed feedback");
        auto status = parseFeedback(R"({"address":2,"funcode":3,"data":{"power_status":1}})");
        state.update(*status, now);
        auto partial = parseFeedback(R"({"address":2,"funcode":3,"data":{"enable_status":1}})");
        state.update(*partial, now + std::chrono::seconds(3));
        check(state.powerTime == now, "partial status cannot refresh power timestamp");
        Ack ack; ack.seq = 123;
        auto recv = parseFeedback(R"({"seq":122,"address":3,"funcode":1,"data":{"recv_status":1}})");
        ack.update(*recv); check(!ack.accepted(false), "unrelated seq ignored");
        recv->seq = 123; ack.update(*recv);
        check(ack.accepted(false) && !ack.accepted(true), "receipt is not execution");
        recv->received = 0; ack.update(*recv);
        check(ack.rejected() && !ack.accepted(false), "negative receipt is failure");
        PositionCheck position; position.sentAt = now; position.baseline.fill(0);
        position.allowed = {false,false,false,false,true,true,true}; position.targets[5] = 1;
        State measured; Feedback measuredFeedback; measuredFeedback.angles = Angles{0,0,0,0,0,1,0};
        measured.update(measuredFeedback, now); check(!position.observe(measured), "pre-send sample cannot confirm");
        measured.update(measuredFeedback, now + std::chrono::milliseconds(100));
        check(!position.observe(measured) && position.settled == 1, "first matching sample");
        position.observe(measured); check(position.settled == 1, "same frame must not count twice");
        measured.update(measuredFeedback, now + std::chrono::milliseconds(200)); position.observe(measured);
        measured.update(measuredFeedback, now + std::chrono::milliseconds(300));
        check(position.observe(measured), "three distinct matching frames confirm target");
        (*measuredFeedback.angles)[0] = 0.6; measured.update(measuredFeedback, now + std::chrono::milliseconds(400));
        check(!position.observe(measured) && position.violated, "protected drift latches failure");
        for (double target : {-85., -75., 75., 85.}) {
            for (double sign : {-1., 1.}) {
                for (double error : {.3, .301, .4}) {
                    PositionCheck boundary;
                    boundary.sentAt = now; boundary.allowed[4] = true;
                    boundary.targets[4] = target;
                    State sample; Feedback sampleFeedback; sampleFeedback.angles = Angles{};
                    (*sampleFeedback.angles)[4] = static_cast<double>(static_cast<float>(target + sign * error));
                    bool confirmed = false;
                    for (int frame = 1; frame <= 3; ++frame) {
                        sample.update(sampleFeedback, now + std::chrono::milliseconds(100 * frame));
                        confirmed = boundary.observe(sample);
                    }
                    check(confirmed == (error == .3), "float32 boundary accepted; real excess still rejected");
                }
            }
        }
        check(encode(parseCommand("j 5 11"),4).find("\"angle\":11,") != std::string::npos, "plain decimal SDK wire format");
        auto damping = parseCommand("damping 5 1000");
        check(damping.function == 4 && !damping.motion && damping.data.at("mode") == 1000 && damping.data.at("id") == 5,
              "explicit raw damping is not a position command");
        check(parseCommand("damping all 1").function == 5, "raw all-joint damping");
        check(parseCommand("dumping 1").data == parseCommand("damping all 1").data, "legacy dumping keeps explicit raw semantics");
        check(parseCommand("hold-current all").kind == Kind::HoldCurrent, "hold-current parsed independently of send");
        check(parseCommand("hold-current-check 5").kind == Kind::HoldCurrentCheck, "hold-current-check parsed as read-only");
        auto rejects = [](auto action) {
            try { action(); } catch (const std::invalid_argument&) { return true; }
            return false;
        };
        check(rejects([&] { compensateJ4(parseCommand("hold-current all"), 0.); }), "even zero compensation flag rejected for hold");
        check(rejects([&] { compensateJ4(parseCommand("hold-current-check 5"), .4); }), "hold check compensation rejected");
        State h; Feedback hFeedback; hFeedback.angles = Angles{0,-89,86,4,11,-5,35.9};
        h.update(hFeedback, now);
        h.angleTime = h.powerTime = h.errorTime = now; h.power = 1; h.error = 0; h.enable = 0;
        auto candidate = makeHoldCurrent(h, now, 5);
        check(candidate.data.at("angle").to_number<double>() == -5 && candidate.data.at("delay_ms") == 1000,
              "hold uses measured target, explicit delay and no enable-state prerequisite");
        check(rejects([&] { makeHoldCurrent(h, now + std::chrono::milliseconds(251), 5); }), "stale hold angles rejected");
        auto partialOld = h; partialOld.jointTimes[4] -= std::chrono::milliseconds(251);
        check(rejects([&] { checkHoldState(partialOld, now); }), "hold rejects one old axis despite a fresh aggregate timestamp");
        auto bad = h; (*bad.angles)[3] = 164.4; (*bad.angles)[5] = -168.3;
        check(rejects([&] { makeHoldCurrent(bad, now, 5); }), "actual reconnect over-range posture rejected, never wrapped");
        check((*bad.angles)[3] == 164.4 && (*bad.angles)[5] == -168.3, "rejected feedback not normalized");
        bad = h; (*bad.angles)[6] = 50.1;
        check(rejects([&] { makeHoldCurrent(bad, now, 5); }), "other-joint controller-clipped feedback rejects hold");
        bad = h; (*bad.angles)[6] = 50; makeHoldCurrent(bad, now, 6);
        bad = h; bad.angles.reset();
        check(rejects([&] { makeHoldCurrent(bad, now, 5); }), "missing feedback rejected");
        bad = h; bad.power = 0;
        check(rejects([&] { makeHoldCurrent(bad, now, 5); }), "power software gate retained");
        bad = h; bad.error = 1;
        check(rejects([&] { makeHoldCurrent(bad, now, 5); }), "fault rejects hold");
        bad = h; bad.errorTime -= std::chrono::seconds(3);
        check(rejects([&] { makeHoldCurrent(bad, now, 5); }), "stale fault status rejected");
        auto drift = *h.angles; drift[0] += .21;
        check(rejects([&] { checkHoldDrift(*h.angles, drift); }), "any arm drift rejects hold");
        drift = *h.angles; drift[6] += .31;
        check(rejects([&] { checkHoldDrift(*h.angles, drift); }), "gripper drift rejects hold");
        checkHoldDrift(*h.angles, *h.angles);
        State boot = h; boot.enableTime = now;
        checkPositionBootstrap(parseCommand("j 6 36"), boot, now);
        check(rejects([&] { checkPositionBootstrap(parseCommand("ja 0 -89 86 4 11 -5 36"), boot, now); }), "all-axis bootstrap refused");
        for (const auto& command : {"j 6 35.9", "j 6 35.9001", "ja 0 -89 86 4 11 -5 35.9",
                                   "zero", "damping all 1000", "hold-current all", "nudge 6 1", "test-joint 5 1", "j 6 50.1"})
            check(rejects([&] { checkPositionBootstrap(parseCommand(command), boot, now); }),
                  "bootstrap rejects holds, non-position commands, quantized-zero and clamped targets");
        auto bootBad = boot; bootBad.enable = 1;
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }), "bootstrap requires observed enable zero");
        bootBad = boot; bootBad.jointTimes[4] -= std::chrono::milliseconds(250);
        checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now);
        bootBad.jointTimes[4] -= std::chrono::milliseconds(1);
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }),
              "bootstrap rejects a single old axis even when aggregate angle time and statuses remain fresh");
        bootBad = boot; bootBad.power = 0;
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }), "bootstrap power gate");
        bootBad = boot; bootBad.error = 1;
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }), "bootstrap fault gate");
        for (auto member : {&State::angleTime, &State::powerTime, &State::enableTime, &State::errorTime}) {
            bootBad = boot; bootBad.*member -= std::chrono::milliseconds(251);
            check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }), "bootstrap stale channel gate");
            bootBad = boot; bootBad.*member += std::chrono::milliseconds(1);
            check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }), "bootstrap future timestamp gate");
        }
        bootBad = boot; (*bootBad.angles)[1] = -90.1;
        checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now);
        check((*bootBad.angles)[1] == -90.1, "accepted jitter preserves raw feedback");
        (*bootBad.angles)[1] = -90.201;
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 36"), bootBad, now); }), "bootstrap rejects beyond authorized feedback jitter");
        bootBad = boot; (*bootBad.angles)[6] = 50.1;
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 5 -4"), bootBad, now); }), "bootstrap rejects clamp-range feedback");
        bootBad = boot; (*bootBad.angles)[6] = 6.5;
        check(encode(parseCommand("j 6 6.5005"),1).find("\"angle\":6.500,") != std::string::npos,
              "half-boundary regression has actual same-position wire value");
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 6 6.5005"), bootBad, now); }),
              "bootstrap uses actual wire encoder at positive half-boundary");
        (*bootBad.angles)[5] = -6.5;
        check(rejects([&] { checkPositionBootstrap(parseCommand("j 5 -6.5005"), bootBad, now); }),
              "bootstrap uses actual wire encoder at negative half-boundary");
        checkPositionBootstrap(parseCommand("j 6 6.501"), bootBad, now);
        for (int joint = 0; joint < 6; ++joint) {
            for (double boundary : {limits[joint].min - .2, limits[joint].max + .2})
                checkFeedbackAngle(joint, static_cast<float>(boundary));
            for (double outside : {limits[joint].min - .201, limits[joint].max + .201})
                check(rejects([&] { checkFeedbackAngle(joint, outside); }), "feedback beyond 0.2 rejected");
            check(rejects([&] { checkAngle(joint, limits[joint].min - .001); }), "lower command limit unchanged");
            check(rejects([&] { checkAngle(joint, limits[joint].max + .001); }), "upper command limit unchanged");
        }
        for (double outside : {-.001, 65.001})
            check(rejects([&] { checkFeedbackAngle(6, outside); }), "J6 receives no arm jitter tolerance");
        bootBad = boot; (*bootBad.angles)[1] = -90.1;
        check(rejects([&] { makeHoldCurrent(bootBad, now, 1); }), "jitter must not become an out-of-range hold target");
        check(rejects([&] { parseCommand("j 1 -90.1"); }), "wire target limits remain strict");
        std::cout << "Protocol and state tests passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
