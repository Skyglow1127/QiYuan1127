#include "protocol.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace d1;
using namespace std::chrono_literals;
namespace {
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
Feedback frame(double j4 = 0) {
    Feedback f; f.address=2; f.function=1; f.seq=10;
    f.angles=Angles{0,0,0,0,j4,0,36}; return f;
}
template<class Action> bool throws(Action action) {
    try { action(); } catch (const std::invalid_argument&) { return true; } return false;
}
}
int main() {
    try {
        const auto start=Clock::time_point{}+1s;
        for (double speed : {0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
            check(throws([&]{checkFeedbackMaxSpeed(speed);}), "positive finite speed required");
        State state;
        check(!updateFilteredFeedback(state,frame(39.4),start,.5), "initial seven axes accepted");
        auto bad=frame(-165.3); bad.power=1;bad.enable=1;bad.error=0;
        auto rejected=updateFilteredFeedback(state,bad,start+312ms,.5);
        check(rejected && rejected->at("reason")=="angular_jump", "historical J4 jump rejected");
        check(state.angles && (*state.angles)[4]==39.4 && state.jointTimes[4]==start &&
              state.jointSequences[4]==1 && state.jointTimes[0]==start+312ms &&
              state.jointSequences[0]==2 && state.angleTime==start+312ms && state.angleFrames==2,
              "J4 rejection preserves only J4 while six valid axes update");
        check(state.power==1 && state.powerTime==start+312ms && state.feedbackPendingSince,
              "status and aggregate pending remain independent");
        auto metadata=jointFeedbackMetadata(state);
        check(metadata.at("accepted_joints").as_array().size()==6 &&
              metadata.at("rejected_joints").as_array()==json::array{4} &&
              metadata.at("joint_pending").as_array()[4]==true &&
              metadata.at("merged_angles").as_array()[4]==39.4,
              "mixed frame metadata carries merged values and explicit masks");
        const auto rawMessage=json::parse(R"({"seq":10,"address":2,"funcode":1,"data":{"angle0":0,"angle1":0,"angle2":0,"angle3":0,"angle4":-165.3,"angle5":0,"angle6":36}})");
        const auto marker=makeFeedbackMarker(state,rawMessage,start+312ms,rejected);
        check(marker.at("message")==rawMessage && marker.at("joint_rejections").as_array().size()==1 &&
              marker.at("received_monotonic_s")==1.312 &&
              marker.at("joint_times_s").as_array()[0]==marker.at("received_monotonic_s") &&
              marker.at("joint_sequences").as_array()[0]==2 &&
              marker.at("joint_times_s").as_array()[4]==1. &&
              marker.at("joint_sequences").as_array()[4]==1 &&
              marker.at("merged_angles").as_array()[4]==39.4 &&
              marker.at("joint_pending").as_array()[4]==true,
              "production marker preserves raw data and exact independent accepted/rejected metadata");
        Ack ack;ack.seq=123;Feedback receipt;receipt.address=3;receipt.seq=123;receipt.received=1;
        ack.update(receipt);check(ack.accepted(false), "bad angle does not affect ACK");
        check(feedbackRecoveryState(state,start+400ms,start+2s)==FeedbackRecoveryState::Waiting,
              "one pending axis pauses every new trajectory target");
        check(!updateFilteredFeedback(state,frame(39.5),start+500ms,.5) &&
              state.jointAcceptanceMethods[4]=="last_accepted_cone" && !state.feedbackPendingSince &&
              std::abs(state.feedbackPauseTotalSeconds-.188)<1e-12,
              "next J4 value in original cone restores this axis and closes pause once");
        check(feedbackRecoveryState(state,start+501ms,start+2s)==FeedbackRecoveryState::Ready,
              "all seven fresh axes restore motion readiness");

        for (int joint=0;joint<6;++joint) {
            State boundary;updateFilteredFeedback(boundary,frame(),start,1.);
            auto exact=frame();(*exact.angles)[joint]=3.;
            check(!updateFilteredFeedback(boundary,exact,start+1s,1.), "exact angular threshold accepted per axis");
            (*exact.angles)[joint]=std::nextafter(6.,7.);
            check(updateFilteredFeedback(boundary,exact,start+2s,1.).has_value() &&
                  boundary.jointTimes[joint]==start+1s, "strict angular excess rejected per axis");
        }
        auto j1=[](double v) {auto f=frame();(*f.angles)[1]=v;return f;};
        State observed;updateFilteredFeedback(observed,j1(-86.2),start,.5);
        check(updateFilteredFeedback(observed,j1(-85.5),start+111ms,.5).has_value(), "real J1 first jump rejected");
        check(updateFilteredFeedback(observed,j1(-85.3),start+222ms,.5).has_value(), "raw pair still too fast");
        check(!updateFilteredFeedback(observed,j1(-85.3),start+333ms,.5) &&
              observed.jointAcceptanceMethods[1]=="two_valid_raw_frames" &&
              observed.jointSequences[1]==2 && (*observed.angles)[1]==-85.3 &&
              std::abs(observed.feedbackPauseTotalSeconds-.222)<1e-12,
              "bounded consecutive raw pair restores actual J1 independently");

        State singleLost;updateFilteredFeedback(singleLost,frame(39.4),start,.5);
        for (int index=1;index<=40;++index) {
            updateFilteredFeedback(singleLost,frame(index<20?-165.:-166.),start+100ms*index,.5);
            check(!singleLost.jointRecoveryCandidates[4] && singleLost.jointTimes[4]==start &&
                  singleLost.jointSequences[4]==1, "stable out-of-range J4 never becomes valid");
            check(feedbackRecoveryState(singleLost,start+100ms*index,start+2s)==FeedbackRecoveryState::Waiting,
                  "single stale J4 is recorded but cannot time out six live axes");
        }
        auto events=updateJointFeedbackStaleness(singleLost,start+4s);
        check(events.size()==1 && events[0].as_object().at("joint")==4 &&
              events[0].as_object().at("state")=="stale", "only lost joint emits stale transition");
        check(updateJointFeedbackStaleness(singleLost,start+4100ms).empty(), "stale diagnostic not emitted per frame");
        updateFilteredFeedback(singleLost,frame(39.5),start+4200ms,.5);
        events=updateJointFeedbackStaleness(singleLost,start+4200ms);
        check(events.size()==1 && events[0].as_object().at("state")=="recovered", "per-axis fresh recovery emitted once");

        State reset;updateFilteredFeedback(reset,frame(),start,.5);
        updateFilteredFeedback(reset,frame(10),start+100ms,.5);
        auto invalid=frame(10);(*invalid.angles)[4]=std::numeric_limits<double>::quiet_NaN();
        updateFilteredFeedback(reset,invalid,start+150ms,.5);
        check(!reset.jointRecoveryCandidates[4], "invalid axis clears its raw candidate");
        check(updateFilteredFeedback(reset,frame(10.1),start+200ms,.5).has_value(), "first bounded raw after invalid remains candidate");
        check(updateFilteredFeedback(reset,frame(10.1),start+200ms,.5).has_value(), "zero raw dt cannot recover");
        check(!updateFilteredFeedback(reset,frame(10.1),start+300ms,.5), "positive raw dt restores axis");
        PositionCheck arrival;arrival.sentAt=start;arrival.allowed[4]=true;arrival.targets[4]=0;
        check(!arrival.observe(reset) && arrival.maxError>3, "restored actual value still reaches tracking/arrival protection");
        State protectedAxis;updateFilteredFeedback(protectedAxis,frame(),start,.5);
        auto jump=frame();(*jump.angles)[0]=1.;updateFilteredFeedback(protectedAxis,jump,start+100ms,.5);
        (*jump.angles)[0]=1.1;updateFilteredFeedback(protectedAxis,jump,start+200ms,.5);
        PositionCheck drift;drift.sentAt=start;drift.allowed[5]=true;
        check(!drift.observe(protectedAxis) && drift.violated, "per-axis recovery never suppresses protected drift");

        State partialSamples;updateFilteredFeedback(partialSamples,frame(),start,.5);
        PositionCheck coherent;coherent.sentAt=start;coherent.baseline=*partialSamples.angles;
        coherent.allowed[6]=true;coherent.targets[6]=36.;
        auto missingSample=frame();(*missingSample.angles)[4]=std::numeric_limits<double>::quiet_NaN();
        for (int index=1;index<=3;++index) {
            updateFilteredFeedback(partialSamples,missingSample,start+100ms*index,.5);
            check(!coherent.observe(partialSamples) && coherent.settled==0,
                  "partial frames cannot consume a protected axis's pre-command value as a new arrival sample");
        }
        updateFilteredFeedback(partialSamples,frame(),start+400ms,.5);
        check(!coherent.observe(partialSamples) && coherent.settled==1, "first complete per-axis update consumes one sample");
        for (int index=5;index<=7;++index) {
            updateFilteredFeedback(partialSamples,missingSample,start+100ms*index,.5);
            check(!coherent.observe(partialSamples) && coherent.settled==1,
                  "partial frames cannot reuse one accepted per-axis sample to accumulate settled confirmations");
        }
        updateFilteredFeedback(partialSamples,frame(),start+800ms,.5);coherent.observe(partialSamples);
        updateFilteredFeedback(partialSamples,frame(),start+900ms,.5);
        check(coherent.observe(partialSamples) && coherent.settled==3,
              "three independently advanced seven-axis samples still confirm arrival");
        auto partialDrift=missingSample;(*partialDrift.angles)[0]=.6;
        updateFilteredFeedback(partialSamples,partialDrift,start+1s,10.);
        check(!coherent.observe(partialSamples) && coherent.violated,
              "protected drift is checked on a partial update even without a consumable complete sample");

        auto partialText=R"({"seq":10,"address":2,"funcode":1,"data":{"angle0":0,"angle1":0,"angle2":0,"angle3":0,"angle4":null,"angle5":0,"angle6":36}})";
        check(!parseFeedback(partialText), "default parser remains strict");
        auto partial=parseFeedback(partialText,true);
        check(partial && std::isnan((*partial->angles)[4]), "filter-only parser retains six valid axes");
        State initialPartial;updateFilteredFeedback(initialPartial,*partial,start,.5);
        check(!initialPartial.angles && initialPartial.jointSequences[0]==1 && initialPartial.jointSequences[4]==0,
              "complete angles cannot initialize from a missing axis");
        metadata=jointFeedbackMetadata(initialPartial);
        check(metadata.at("joint_times_s").as_array()[4].is_null() &&
              metadata.at("merged_angles").as_array()[4].is_null(), "unknown metadata is null, never a fake zero");
        for (int index=1;index<=30;++index) updateFilteredFeedback(initialPartial,*partial,start+100ms*index,.5);
        check(feedbackRecoveryState(initialPartial,start+3s,start+2s)==FeedbackRecoveryState::Waiting,
              "healthy axes prevent global timeout despite one uninitialized axis");
        updateFilteredFeedback(initialPartial,frame(),start+3100ms,.5);
        check(initialPartial.angles && feedbackRecoveryState(initialPartial,start+3100ms,start+2s)==FeedbackRecoveryState::Ready,
              "first actual valid missing axis completes baseline");

        State hardLimits;updateFilteredFeedback(hardLimits,frame(90.1),start,1.);
        updateFilteredFeedback(hardLimits,frame(90.3),start+1s,1.);
        check(throws([&]{checkFeedbackAngle(4,*hardLimits.jointAngles[4]);}), "slow arm limit breach remains a real hard failure");
        auto gripBad=frame();(*gripBad.angles)[6]=65.1;
        State grip;check(updateFilteredFeedback(grip,gripBad,start,.5).has_value() && !grip.jointAngles[6],
                         "initial bad J6 cannot initialize this axis");
        updateFilteredFeedback(grip,frame(),start+100ms,.5);
        updateFilteredFeedback(grip,gripBad,start+200ms,.5);
        check(*grip.jointAngles[6]==65.1 && !grip.jointPendingSince[6] &&
              throws([&]{checkFeedbackAngle(6,*grip.jointAngles[6]);}), "finite J6 range breach stays visible to hard limit checks");

        State gaps;updateFilteredFeedback(gaps,frame(),start,.5);gaps.power=gaps.enable=1;gaps.error=0;
        check(feedbackRecoveryState(gaps,start+600ms,start+2s)==FeedbackRecoveryState::Ready, "soft boundary accepted");
        check(feedbackRecoveryState(gaps,start+604ms,start+2s)==FeedbackRecoveryState::Waiting, "all stale at .604 waits");
        Feedback status;status.power=1;status.enable=1;status.error=0;
        updateFilteredFeedback(gaps,status,start+1500ms,.5);
        check(feedbackRecoveryState(gaps,start+2s,start+2s)==FeedbackRecoveryState::TimedOut,
              "only all seven accepted ages reaching two seconds times out; status cannot refresh them");
        for (auto arrival : {1990ms,2000ms,2010ms}) {
            State late;updateFilteredFeedback(late,frame(),start,.5);
            const auto countAtEntry=late.allFeedbackGapCount;
            updateFilteredFeedback(late,frame(),start+arrival,.5);
            const bool overdue=arrival>2000ms;
            check(late.allFeedbackGapCount==(overdue?1u:0u) &&
                  feedbackRecoveryState(late,start+2020ms,start+2s,false,2000,countAtEntry)==
                      (overdue?FeedbackRecoveryState::TimedOut:FeedbackRecoveryState::Ready),
                  "source-time all-axis gap evidence distinguishes late arrival from late waiter wakeup");
            auto evidence=jointFeedbackMetadata(late);
            if (overdue) {
                check(evidence.at("last_all_feedback_gap_start_s")==1. &&
                      evidence.at("last_all_feedback_gap_end_s")==3.01,
                      "persistent all-axis gap evidence retains source start and end");
                updateFilteredFeedback(late,frame(),start+2110ms,.5);
                check(late.allFeedbackGapCount==1 &&
                      feedbackRecoveryState(late,start+2120ms,start+2s,false,2000,countAtEntry)==FeedbackRecoveryState::TimedOut,
                      "subsequent healthy frames cannot erase timeout during this wait");
            } else check(evidence.at("last_all_feedback_gap_start_s").is_null() &&
                         evidence.at("last_all_feedback_gap_end_s").is_null(),
                         "no gap evidence is represented by null times");
        }
        State busyCpu;updateFilteredFeedback(busyCpu,frame(),start,.5);
        for (int index=1;index<=30;++index) updateFilteredFeedback(busyCpu,frame(),start+100ms*index,.5);
        check(busyCpu.allFeedbackGapCount==0 &&
              feedbackRecoveryState(busyCpu,start+3020ms,start+2s,false,2000,0)==FeedbackRecoveryState::Ready,
              "continuous valid reception remains ready when the gate thread wakes after its old deadline");
        State configured;updateFilteredFeedback(configured,frame(),start,.5,1000);
        updateFilteredFeedback(configured,frame(),start+1010ms,.5,1000);
        check(configured.allFeedbackGapCount==1,"gap evidence uses the configured recovery timeout");
        gaps.error=1;check(feedbackRecoveryState(gaps,start+700ms,start+2s)==FeedbackRecoveryState::UnsafeStatus,"explicit error remains immediate");
        gaps.error=0;gaps.enable=0;
        check(feedbackRecoveryState(gaps,start+700ms,start+2s)==FeedbackRecoveryState::UnsafeStatus &&
              feedbackRecoveryState(gaps,start+700ms,start+2s,true)==FeedbackRecoveryState::Waiting,"bootstrap-only enable-zero exception");
        State empty;
        check(feedbackRecoveryState(empty,start+1s,start+2s)==FeedbackRecoveryState::Waiting &&
              feedbackRecoveryState(empty,start+2s,start+2s)==FeedbackRecoveryState::TimedOut,"all missing startup is bounded");
        State disabled;updateFilteredFeedback(disabled,frame(39.4),start,{});
        check(!updateFilteredFeedback(disabled,frame(-165.3),start+100ms,{}) &&
              (*disabled.angles)[4]==-165.3,"default disabled observation unchanged");
        std::cout<<"Per-axis feedback filtering and recovery tests passed\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
