#include "depth_gate.h"
#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <stdexcept>
void check(const std::vector<uint16_t>& x,uint16_t lo,uint16_t hi) {
 std::vector<uint16_t> out(x.size()+2,12345),expected(x.size(),0);
 std::array<uint64_t,5> stats{99,99,99,99,99},want{0,0,0,0,0};
 if(lo>hi || x.size()>307200) {want[4]=1; std::fill(expected.begin(),expected.end(),12345);}
 else for(size_t i=0;i<x.size();++i) {
  auto d=x[i];if(d==0||d==65535||d<lo||d>hi)continue;
  expected[i]=d;want[1]+=d;
  if(want[0]++==0)want[2]=d;
  want[2]=std::min(want[2],uint64_t(d));want[3]=std::max(want[3],uint64_t(d));
 }
 depth_gate(x.data(),out.data()+1,stats.data(),x.size(),lo,hi);
 if(stats!=want || !std::equal(expected.begin(),expected.end(),out.begin()+1)
    ||out.front()!=12345||out.back()!=12345)throw std::runtime_error("Mismatch/buffer overrun");
}
int main(int argc,char**argv) {
 check({},0,65535);check({0,1,299,300,1000,1001,65534,65535},300,1000);
 check({0,65535},0,65535);check({10,20},20,10);
 std::mt19937 gen(123);
 std::vector<uint16_t>x(640*480);
 for(auto&d:x) { d=uint16_t(gen()); }
 check(x,300,1500);check(x,0,65535);
 std::fill(x.begin(),x.end(),65534);check(x,1,65534);
 x.push_back(3);check(x,0,65535);
 if(argc>1) {
  std::ifstream f(argv[1],std::ios::binary|std::ios::ate);if(!f)throw std::runtime_error("Input missing");
  auto bytes=f.tellg();
  if(bytes<=0||bytes>307200*2||bytes%2!=0)throw std::runtime_error("Invalid depth byte count");
  f.seekg(0);std::vector<uint16_t> real(size_t(bytes)/2);f.read(reinterpret_cast<char*>(real.data()),real.size()*2);
  if(f.gcount()!=std::streamsize(real.size()*2))throw std::runtime_error("Truncated raw depth");
  check(real,300,1500);check(real,0,65535);
 }
 std::cout<<"PASS depth gate: boundary, invalid, randomized, full frame and overflow cases"<<std::endl;
}
