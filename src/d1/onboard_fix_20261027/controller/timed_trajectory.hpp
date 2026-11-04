#pragma once
// One board clock for the whole arm. Points carry intended arrival times;
// each segment endpoint is written at its preceding point's time, with that
// segment's interval. No receipt wait, per-axis feedback wait, catch-up burst,
// or silent clock re-anchoring occurs inside normal trajectory execution.
#include <rapidjson/document.h>
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <stdexcept>
namespace d1_timed {
using Clock=std::chrono::steady_clock;
using Q=std::array<double,7>;
struct Feedback {Q angles;std::array<Clock::time_point,7> measured;std::array<bool,7> valid;uint64_t sequence;};
struct Point {double ms;Q q;};
struct WriteResult {double span_ms;int commands;int64_t first_ns,last_ns;};
inline int64_t ns(Clock::time_point t){return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();}
class Trajectory {
public:
 using Read=std::function<Feedback()>;
 using Write=std::function<WriteResult(const Q&,int,Clock::time_point,double)>;
 using Emit=std::function<void(const std::string&,const std::string&,const std::string&,int64_t,double,double,int)>;
 Trajectory(Read read,Write write,Emit emit):read_(read),write_(write),emit_(emit){}
 bool active()const{return busy_.load();}
 Clock::time_point nextDue(){std::lock_guard<std::mutex> l(lock_);return settleDeadline_!=Clock::time_point{}?Clock::time_point::max():dueLocked();}
 void receive(const std::string& raw){
  std::lock_guard<std::mutex> l(lock_);
  rapidjson::Document d;d.Parse(raw.c_str());
  std::string incoming;
  try{
   require(d.IsObject(),"JSON object required");
   require(text(d,"schema")=="d1_timed_trajectory_v1","unknown schema");incoming=text(d,"trajectory_id");
   require(!incoming.empty()&&incoming.size()<=128,"invalid trajectory id");const auto type=text(d,"type");
   const int requestSeq=integer(d,"message_seq",1,2147483647);
   if(type!="chunk"){
    require(incoming==id_&&busy_,"no matching active trajectory");
    if(requestSeq<=lastRequestSeq_){emitState(paused_?"paused":"resumed","duplicate or reordered control ignored");return;}
    lastRequestSeq_=requestSeq;
    if(type=="cancel"){finish("cancelled","future groups cancelled; in-flight motor segment may finish");return;}
    if(type=="pause"){pauseRequested_=true;return;}
    if(type=="resume"){
     if(!paused_){pauseRequested_=false;emitState("resumed","already running; duplicate resume ignored");return;}
     epoch_+=Clock::now()-pauseAt_;paused_=false;pauseRequested_=false;
     emitState("resumed","whole-arm clock resumed");return;
    }
    throw std::runtime_error("unsupported control type");
   }
   const int chunk=integer(d,"chunk_index",0,4095);
   require(d.HasMember("final")&&d["final"].IsBool(),"final flag required");
   const bool final=d["final"].GetBool();
   require(text(d,"gripper_mode")=="preserve","timed arm mode must preserve gripper");
   if(!busy_){
    require(chunk==0&&!seen_.count(incoming),"new trajectory must begin with unused id and chunk zero");
    require(seen_.size()<4096,"trajectory id history exhausted; explicit process maintenance required");
    const double lead=number(d,"start_after_ms",100,5000);
    maxLate_=number(d,"max_lateness_ms",1,50);maxError_=number(d,"max_tracking_error_deg",.1,3);
    speed_=number(d,"velocity_limit_deg_s",.01,5);acceleration_=number(d,"acceleration_limit_deg_s2",.01,10);
    id_=incoming;points_.clear();chunks_.clear();next_=1;loaded_=started_=paused_=pauseRequested_=false;
    settled_=0;lastFeedback_=0;lastRequestSeq_=0;epoch_=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double,std::milli>(lead));
    seen_.insert(id_);busy_=true;
   }
   require(incoming==id_,"another trajectory is already active");
   if(chunk<static_cast<int>(chunks_.size())){
    require(chunks_[chunk]==raw,"conflicting duplicate chunk");emitState("buffered","identical duplicate ignored");return;
   }
   require(!loaded_&&chunk==static_cast<int>(chunks_.size()),"chunk gap or data after final");
   require(requestSeq>lastRequestSeq_,"nonadvancing chunk message sequence");
   require(number(d,"max_lateness_ms",1,50)==maxLate_&&number(d,"max_tracking_error_deg",.1,3)==maxError_
       &&number(d,"velocity_limit_deg_s",.01,5)==speed_&&number(d,"acceleration_limit_deg_s2",.01,10)==acceleration_,"trajectory parameters changed between chunks");
   require(d.HasMember("points")&&d["points"].IsArray()&&!d["points"].Empty(),"nonempty points required");
   auto candidate=points_;
   for(const auto& p:d["points"].GetArray()){
    require(p.IsObject(),"point object required");Point point;point.ms=number(p,"time_from_start_ms",0,120000);
    require(p.HasMember("sdk_angles")&&p["sdk_angles"].IsArray()&&p["sdk_angles"].Size()==7,"seven sdk angles required");
    for(int j=0;j<7;++j){require(p["sdk_angles"][j].IsNumber(),"numeric angles required");point.q[j]=p["sdk_angles"][j].GetDouble();
     static constexpr double lo[7]={-135,-90,-90,-135,-90,-135,0},hi[7]={135,90,90,135,90,135,65};
     require(std::isfinite(point.q[j])&&point.q[j]>=lo[j]&&point.q[j]<=hi[j],"joint target outside approved host envelope; J6 is snapshot-only");}
    if(candidate.empty())require(point.ms==0,"first point must be at time zero");
    else {const auto& before=candidate.back();const double dt=(point.ms-before.ms)/1000.;require(dt>=.02&&dt<=65.535,"segment interval out of range");
     require(std::abs(point.q[6]-candidate.front().q[6])<1e-6,"timed arm trajectory must preserve gripper");
     for(int j=0;j<6;++j){const double v=(point.q[j]-before.q[j])/dt;require(std::abs(v)<=speed_+1e-5,"reference velocity limit exceeded");
      if(candidate.size()>1){const auto& prev=candidate[candidate.size()-2];const double prevDt=(before.ms-prev.ms)/1000.;
       const double prevV=(before.q[j]-prev.q[j])/prevDt;
       require(std::abs(v-prevV)/((dt+prevDt)/2)<=acceleration_+1e-4,"reference acceleration limit exceeded");}}
    }
    candidate.push_back(point);require(candidate.size()<=4096,"trajectory buffer full");
   }
   require(!final||candidate.size()>=2,"final trajectory needs two points");
   require(Clock::now()<epoch_,"buffer arrived after trajectory start deadline");
   points_=std::move(candidate);chunks_.push_back(raw);loaded_=final;lastRequestSeq_=requestSeq;emitState(chunk==0?"accepted":"buffered",final?"complete trajectory buffered":"awaiting remaining chunks");
  }catch(const std::exception& e){
   if(busy_&&incoming==id_)finish("fault",e.what());else emit_(incoming,"fault",e.what(),0,0,0,0);
  }
 }
 // Called by the single UART owner before each feedback transaction.
 void tick(){
  std::lock_guard<std::mutex> l(lock_);if(!busy_||paused_)return;
  try{
   auto now=Clock::now();
   if(!loaded_){if(now>=epoch_)finish("fault","trajectory buffer incomplete at start deadline");return;}
   const auto due=dueLocked();if(now<due)return;
   const double late=std::chrono::duration<double,std::milli>(now-due).count();
   if(settleDeadline_==Clock::time_point{})require(late<=maxLate_,"group dispatch deadline missed; no backlog replay");
   const auto f=read_();
   for(int j=0;j<7;++j){require(f.valid[j]&&now>=f.measured[j]&&now-f.measured[j]<=std::chrono::milliseconds(250),"valid encoder feedback older than 250 ms");
    static constexpr double lo[7]={-135,-90,-90,-135,-90,-135,0},hi[7]={135,90,90,135,90,135,65};
    const double tolerance=j<6?.2:0.;require(std::isfinite(f.angles[j])&&f.angles[j]>=lo[j]-tolerance-1e-5&&f.angles[j]<=hi[j]+tolerance+1e-5,"joint feedback outside approved host envelope");}
   if(!started_){
    for(int j=0;j<7;++j)require(std::abs(f.angles[j]-points_.front().q[j])<=.3+1e-5,"measured start differs from buffered start");
    started_=true;emitState("started","board monotonic trajectory epoch reached");
   }
   if(next_>=points_.size()){
    if(settleDeadline_==Clock::time_point{})settleDeadline_=now+std::chrono::seconds(5);
    require(now<=settleDeadline_,"endpoint did not settle within five seconds");
    if(f.sequence!=lastFeedback_){
     lastFeedback_=f.sequence;bool arrived=true;
     for(int j=0;j<7;++j){const double tolerance=j<6?1.:.5;if(std::abs(f.angles[j]-points_.back().q[j])>tolerance+1e-5)arrived=false;}
     if(!arrived)settled_=0;
     else {
      if(settled_==0)settledLow_=settledHigh_=f.angles;
      bool spanOk=true;
      for(int j=0;j<6;++j){settledLow_[j]=std::min(settledLow_[j],f.angles[j]);settledHigh_[j]=std::max(settledHigh_[j],f.angles[j]);
       if(settledHigh_[j]-settledLow_[j]>.3+1e-5)spanOk=false;}
      if(spanOk)++settled_;else {settled_=1;settledLow_=settledHigh_=f.angles;}
     }
     if(settled_>=3)finish("completed","endpoint confirmed from three new valid encoder frames; physical accuracy unverified");
    }return;
   }
   const auto& from=points_[next_-1];
   for(int j=0;j<6;++j)require(std::abs(f.angles[j]-from.q[j])<=maxError_+1e-5,"whole-arm tracking error exceeded limit");
   require(std::abs(f.angles[6]-points_.front().q[6])<=.5+1e-5,"gripper drift during arm trajectory");
   if(pauseRequested_){paused_=true;pauseAt_=due;emitState("paused","whole-arm phase paused at segment boundary; physical stop unverified");return;}
   const auto& to=points_[next_];const int interval=static_cast<int>(std::lround(to.ms-from.ms));
   require(std::chrono::duration<double,std::milli>(Clock::now()-due).count()<=maxLate_,"group deadline expired during admission");
   const auto result=write_(to.q,interval,due,maxLate_);
   require(result.span_ms<=maxLate_,"serial group span exceeded dispatch budget; no catch-up");
   lastSpan_=result.span_ms;lastLate_=static_cast<double>(result.first_ns-ns(due))/1.e6;lastCommands_=result.commands;++next_;
   emitState("dispatched","segment endpoint written as a group; motor application unconfirmed");
  }catch(const std::exception& e){finish("fault",e.what());}
 }
private:
 static void require(bool ok,const char* s){if(!ok)throw std::runtime_error(s);}
 static std::string text(const rapidjson::Value& d,const char* k){require(d.HasMember(k)&&d[k].IsString(),"missing string field");return d[k].GetString();}
 static double number(const rapidjson::Value& d,const char* k,double lo,double hi){require(d.HasMember(k)&&d[k].IsNumber(),"missing numeric field");double n=d[k].GetDouble();require(std::isfinite(n)&&n>=lo&&n<=hi,"numeric field out of bounds");return n;}
 static int integer(const rapidjson::Value& d,const char* k,int lo,int hi){require(d.HasMember(k)&&d[k].IsInt(),"integer field required");int n=d[k].GetInt();require(n>=lo&&n<=hi,"integer field out of bounds");return n;}
 Clock::time_point dueLocked()const{
  if(!busy_||paused_)return Clock::time_point::max();
  const double ms=next_<=points_.size()&&next_>0&&!points_.empty()?points_[next_-1].ms:0;
  return epoch_+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double,std::milli>(ms));
 }
 void emitState(const std::string& s,const std::string& why){emit_(id_,s,why,ns(epoch_),lastLate_,lastSpan_,lastCommands_);}
 void finish(const std::string& s,const std::string& why){emitState(s,why);busy_=false;paused_=false;pauseRequested_=false;settleDeadline_={};}
 Read read_;Write write_;Emit emit_;std::mutex lock_;std::atomic<bool>busy_{false};
 std::set<std::string> seen_;std::string id_;std::vector<Point> points_;std::vector<std::string> chunks_;
 size_t next_=1;bool loaded_=false,started_=false,paused_=false,pauseRequested_=false;
 Clock::time_point epoch_{},pauseAt_{},settleDeadline_{};uint64_t lastFeedback_=0;int settled_=0,lastRequestSeq_=0;
 Q settledLow_{},settledHigh_{};double maxLate_=20,maxError_=3,speed_=5,acceleration_=10,lastLate_=0,lastSpan_=0;int lastCommands_=0;
};
}
