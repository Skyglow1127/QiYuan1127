// D1 UART integrity repair. Startup opens the port and queries only: no clear,
// home, target, damping, power or automatic recovery command is sent.
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "msg/SetServoAngle_.hpp"
#include "msg/SetServoDumping_.hpp"
#include "FashionStar/UServo/FashionStar_UartServo.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include "position_timing.hpp"
#include "timed_trajectory.hpp"
using namespace unitree::robot;
using namespace fsuservo;
using namespace rapidjson;
using Clock = std::chrono::steady_clock;
namespace {
constexpr const char* buildId = "d1-timed-groups-20261027-v1";
std::atomic<bool> running{true};
std::mutex busMutex, publishMutex;
std::unique_ptr<FSUS_Protocol> protocol;
std::array<std::unique_ptr<FSUS_Servo>,7> servo;
std::array<float,7> angles{};
std::array<Clock::time_point,7> measured{};
std::array<bool,7> valid{};
std::array<unsigned,256> invalidByStatus{};
uint64_t queries=0, invalidQueries=0, validFrames=0, droppedFrames=0;
ChannelPublisherPtr<unitree_arm::msg::dds_::ArmString_> feedback, trajectoryState;
std::unique_ptr<d1_timed::Trajectory> timed;
uint64_t feedbackSequence=0,stateSequence=0;
std::array<int,6> lastTimedEncoded{};std::atomic<bool> timedEncodingInitialized{false};
ChannelPublisherPtr<unitree_arm::msg::dds_::PubServoInfo_> servoFeedback;
constexpr std::array<float,7> minimum{-135,-90,-90,-135,-90,-135,-20};
constexpr std::array<float,7> maximum{135,90,90,135,90,135,50};
int64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}
void publish(Document& doc) {
    StringBuffer buffer; Writer<StringBuffer> writer(buffer); doc.Accept(writer);
    unitree_arm::msg::dds_::ArmString_ msg{}; msg.data_()=buffer.GetString();
    std::lock_guard<std::mutex> lock(publishMutex);
    feedback->Write(msg,0);
    std::cout << buffer.GetString() << std::endl;
}
Document diagnostic(const char* event) {
    Document d;d.SetObject();auto& a=d.GetAllocator();
    d.AddMember("seq",0,a);d.AddMember("address",2,a);d.AddMember("funcode",90,a);
    Value data(kObjectType);
    data.AddMember("schema","d1_uart_integrity_v1",a);
    data.AddMember("build_id",Value(buildId,a),a);
    data.AddMember("event",Value(event,a),a);
    data.AddMember("board_monotonic_ns",ns(Clock::now()),a);
    d.AddMember("data",data,a);return d;
}
// The same UART owner interleaves at most one bounded query between group
// deadlines. A feedback query cannot reserve seven 100 ms waits ahead of motion.
struct Scan {
    int next=0;unsigned mask=0;Clock::time_point begin{};
    std::array<float,7> sample{};std::array<unsigned,7> status{};
    std::array<Clock::time_point,7> stamps{};
} scanState;
void queryOne(unsigned timeoutMs) {
    std::lock_guard<std::mutex> lock(busMutex);
    auto& scan=scanState;const int id=scan.next;
    if(id==0){scan.mask=0;scan.begin=Clock::now();scan.stamps={};}
    ++queries;
    try {
        const auto result=servo[id]->queryRawAngleChecked(timeoutMs);scan.status[id]=result.status;
        if(result.valid&&result.status==FSUS_STATUS_SUCCESS&&result.servo_id==id&&std::isfinite(result.angle)) {
            scan.sample[id]=result.angle;scan.stamps[id]=Clock::now();scan.mask|=1u<<id;
            angles[id]=result.angle;measured[id]=scan.stamps[id];valid[id]=true;
        } else {valid[id]=false;++invalidQueries;++invalidByStatus[scan.status[id]];}
    } catch(const std::exception& error) {
        scan.status[id]=FSUS_STATUS_FAIL;valid[id]=false;++invalidQueries;++invalidByStatus[FSUS_STATUS_FAIL];
        std::cerr<<"query_write_error joint="<<id<<" reason="<<error.what()<<std::endl;
    }
    if(++scan.next!=7)return;
    scan.next=0;const auto end=Clock::now();bool frameOk=scan.mask==127;
    for(int j=0;j<7;++j)if(end-scan.stamps[j]>std::chrono::milliseconds(250))frameOk=false;
    if(frameOk) {
        ++validFrames;++feedbackSequence;
        Document d;d.SetObject();auto& a=d.GetAllocator();Value data(kObjectType);
        for(int j=0;j<7;++j){const std::string key="angle"+std::to_string(j);data.AddMember(Value(key.c_str(),a),scan.sample[j],a);}
        d.AddMember("seq",10,a);d.AddMember("address",2,a);d.AddMember("funcode",1,a);d.AddMember("data",data,a);
        unitree_arm::msg::dds_::PubServoInfo_ info{};
        info.servo0_data_()=scan.sample[0];info.servo1_data_()=scan.sample[1];info.servo2_data_()=scan.sample[2];
        info.servo3_data_()=scan.sample[3];info.servo4_data_()=scan.sample[4];info.servo5_data_()=scan.sample[5];info.servo6_data_()=scan.sample[6];
        servoFeedback->Write(info,0);publish(d);
    } else ++droppedFrames;
    auto d=diagnostic("feedback_scan");auto& data=d["data"];auto& a=d.GetAllocator();
    data.AddMember("queries_total",queries,a);data.AddMember("queries_invalid",invalidQueries,a);
    data.AddMember("valid_frames",validFrames,a);data.AddMember("dropped_frames",droppedFrames,a);
    data.AddMember("valid_mask",scan.mask,a);data.AddMember("scan_begin_monotonic_ns",ns(scan.begin),a);data.AddMember("scan_end_monotonic_ns",ns(end),a);
    Value ss(kArrayType),ts(kArrayType),counts(kObjectType);
    for(int j=0;j<7;++j){ss.PushBack(scan.status[j],a);ts.PushBack(scan.mask&(1u<<j)?ns(scan.stamps[j]):int64_t(0),a);}
    for(unsigned i=0;i<256;++i)if(invalidByStatus[i]){const std::string key=std::to_string(i);counts.AddMember(Value(key.c_str(),a),invalidByStatus[i],a);}
    data.AddMember("query_status",ss,a);data.AddMember("axis_sample_monotonic_ns",ts,a);data.AddMember("invalid_by_status",counts,a);publish(d);
}
void commandDiagnostic(const char* event,int seq,int id,int requested,int effective,bool ok,const char* reason) {
    auto d=diagnostic(event);auto& data=d["data"];auto& a=d.GetAllocator();
    data.AddMember("command_seq",seq,a);data.AddMember("joint",id,a);
    data.AddMember("requested_interval_ms",requested,a);data.AddMember("effective_interval_ms",effective,a);
    data.AddMember("write_ok",ok,a);data.AddMember("reason",Value(reason,a),a);
    data.AddMember("confirmation","serial write only; no motor adoption or arrival confirmation",a);publish(d);
}
void target(const void* raw) {
    const auto& m=*static_cast<const unitree_arm::msg::dds_::SetServoAngle_*>(raw);
    const int id=m.id_(), seq=m.seq_(),requested=m.delay_ms_();const float angle=m.angle_();
    std::lock_guard<std::mutex> lock(busMutex);
    int effective=0;
    if(timed&&timed->active()){commandDiagnostic("command_rejected",seq,id,requested,0,false,"timed trajectory owns the arm; legacy target rejected");return;}
    if(id<0 || id>=7 || !std::isfinite(angle) || angle<minimum[id] || angle>maximum[id]) {
        commandDiagnostic("command_rejected",seq,id,requested,0,false,"invalid id/target; no clipping or automatic damping");return;
    }
    if(!valid[id] || Clock::now()-measured[id]>std::chrono::milliseconds(250)) {
        commandDiagnostic("command_rejected",seq,id,requested,0,false,"no valid measured angle within 250 ms");return;
    }
    if(!d1_timing_fix::delay_ms(angles[id],angle,requested,effective)) {
        commandDiagnostic("command_rejected",seq,id,requested,0,false,"invalid/non-finite/out-of-range interval");return;
    }
    try {
        servo[id]->setRawAngle(angle,effective);
        commandDiagnostic("command_write",seq,id,requested,effective,true,"complete serial packet write returned successfully");
    } catch(const std::exception& error) {
        commandDiagnostic("command_rejected",seq,id,requested,effective,false,error.what());
    }
}
void damping(const void* raw) {
    const auto& m=*static_cast<const unitree_arm::msg::dds_::SetServoDumping_*>(raw);
    const int id=m.id_();std::lock_guard<std::mutex> lock(busMutex);
    if(timed&&timed->active()){commandDiagnostic("command_rejected",m.seq_(),id,0,0,false,"timed trajectory owns the arm; damping rejected");return;}
    if(id<0||id>=7) {commandDiagnostic("command_rejected",m.seq_(),id,0,0,false,"invalid explicit damping id");return;}
    try {servo[id]->setDamping(m.power_());}
    catch(const std::exception& error) {commandDiagnostic("command_rejected",m.seq_(),id,0,0,false,error.what());}
}
d1_timed::Feedback timedFeedback() {
    std::lock_guard<std::mutex> l(busMutex);d1_timed::Feedback result{};
    for(int j=0;j<7;++j){result.angles[j]=angles[j];result.measured[j]=measured[j];result.valid[j]=valid[j];}
    result.sequence=feedbackSequence;
    if(!timedEncodingInitialized){for(int j=0;j<6;++j)lastTimedEncoded[j]=static_cast<int>(std::lround(angles[j]*10.));timedEncodingInitialized=true;}
    return result;
}
d1_timed::WriteResult timedWrite(const d1_timed::Q& q,int interval,Clock::time_point due,double maxLate) {
    std::lock_guard<std::mutex> l(busMutex);
    // UART packets remain sequential. Only the board dispatch clock is common;
    // measured first/last write times expose this residual hardware skew.
    const auto first=Clock::now();int count=0;
    try {
        for(int j=0;j<6;++j){
            if(std::chrono::duration<double,std::milli>(Clock::now()-due).count()>maxLate)
                throw std::runtime_error("serial dispatch deadline exceeded; remaining axes not sent");
            const int encoded=static_cast<int>(std::lround(q[j]*10.));
            if(encoded==lastTimedEncoded[j])continue;
            servo[j]->setRawAngle(static_cast<float>(encoded)/10.f,interval);
            lastTimedEncoded[j]=encoded;++count;
        }
    }catch(const std::exception& e){
        auto d=diagnostic("group_dispatch_failure");auto& data=d["data"];auto& a=d.GetAllocator();
        data.AddMember("commands_completed",count,a);data.AddMember("current_packet_delivery_unknown",true,a);
        data.AddMember("reason",Value(e.what(),a),a);publish(d);throw;
    }
    const auto last=Clock::now();
    auto d=diagnostic("group_dispatch");auto& data=d["data"];auto& a=d.GetAllocator();
    data.AddMember("first_write_monotonic_ns",ns(first),a);data.AddMember("last_write_monotonic_ns",ns(last),a);
    data.AddMember("interval_ms",interval,a);data.AddMember("commands",count,a);
    data.AddMember("hardware_atomic",false,a);publish(d);
    return {std::chrono::duration<double,std::milli>(last-first).count(),count,ns(first),ns(last)};
}
void timedState(const std::string& id,const std::string& state,const std::string& reason,int64_t epoch,double late,double span,int commands) {
    Document d;d.SetObject();auto& a=d.GetAllocator();
    d.AddMember("schema","d1_trajectory_state_v1",a);d.AddMember("trajectory_id",Value(id.c_str(),a),a);
    d.AddMember("message_seq",++stateSequence,a);d.AddMember("state",Value(state.c_str(),a),a);d.AddMember("reason",Value(reason.c_str(),a),a);
    d.AddMember("epoch_monotonic_ns",epoch,a);d.AddMember("board_monotonic_ns",ns(Clock::now()),a);
    d.AddMember("dispatch_lateness_ms",late,a);d.AddMember("group_serial_span_ms",span,a);d.AddMember("serial_commands",commands,a);
    d.AddMember("hardware_atomic",false,a);d.AddMember("physical_accuracy_verified",false,a);
    if(state=="accepted")timedEncodingInitialized=false;
    StringBuffer buffer;Writer<StringBuffer> w(buffer);d.Accept(w);
    unitree_arm::msg::dds_::ArmString_ m{};m.data_()=buffer.GetString();
    std::lock_guard<std::mutex> l(publishMutex);trajectoryState->Write(m,0);std::cout<<buffer.GetString()<<std::endl;
}
void timedReceive(const void* raw){timed->receive(static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw)->data_());}
void stop(int) {running=false;}
}
int main(int argc,char** argv) {
    if(argc==2 && std::string(argv[1])=="--version") {std::cout<<buildId<<std::endl;return 0;}
    std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    ChannelFactory::Instance()->Init(0);
    feedback.reset(new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>("rt/arm_Feedback"));feedback->InitChannel();
    servoFeedback.reset(new ChannelPublisher<unitree_arm::msg::dds_::PubServoInfo_>("current_servo_angle"));servoFeedback->InitChannel();
    protocol=std::make_unique<FSUS_Protocol>("/dev/ttyS4",FSUS_DEFAULT_BAUDRATE);
    for(int id=0;id<7;++id) servo[id]=std::make_unique<FSUS_Servo>(id,protocol.get());
    // No clearAngle at startup: preserve present coordinate definitions.
    for(int j=0;j<7;++j)queryOne(100);
    trajectoryState.reset(new ChannelPublisher<unitree_arm::msg::dds_::ArmString_>("rt/d1_TrajectoryState"));trajectoryState->InitChannel();
    timed=std::make_unique<d1_timed::Trajectory>(timedFeedback,timedWrite,timedState);
    ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> trajectories("rt/d1_TimedTrajectory");trajectories.InitChannel(timedReceive,32);
    ChannelSubscriber<unitree_arm::msg::dds_::SetServoAngle_> positions("set_servo_angle");positions.InitChannel(target,10);
    ChannelSubscriber<unitree_arm::msg::dds_::SetServoDumping_> dampings("set_servo_dumping");dampings.InitChannel(damping,10);
    auto nextScan=Clock::now();
    while(running) {
        timed->tick();const auto now=Clock::now();const auto due=timed->nextDue();
        if(scanState.next!=0||now>=nextScan) {
            const double budget=due==Clock::time_point::max()?100.:std::chrono::duration<double,std::milli>(due-now).count()-3.;
            if(budget>=1.) {
                const unsigned timeout=static_cast<unsigned>(std::min(timed->active()?20.:100.,std::floor(budget)));
                queryOne(std::max(1u,timeout));if(scanState.next==0)nextScan=Clock::now()+std::chrono::milliseconds(100);
                continue;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    trajectories.CloseChannel();positions.CloseChannel();dampings.CloseChannel();return 0;
}
