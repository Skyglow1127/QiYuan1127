#include "depth_gate.h"
#include <vector>
#include <iostream>
#include <algorithm>
int main() {
 const unsigned N=4096;
 std::vector<uint16_t> in(640*480),out(640*480);
 uint64_t stats[5];
 for(unsigned round=0;round<2;++round) {
  uint64_t count=0,sum=0,lo=65535,hi=0;
  for(unsigned i=0;i<N;++i)in[i]=round ? 65534 : uint16_t((i*37U)%2500);
  uint16_t lower=round?1:300,upper=round?65534:1500;
  depth_gate(in.data(),out.data(),stats,N,lower,upper);
  for(unsigned i=0;i<N;++i) {
   uint16_t d=in[i],expected=(d!=0&&d!=65535&&d>=lower&&d<=upper)?d:0;
   if(out[i]!=expected){std::cerr<<"Pixel mismatch "<<i<<"\n";return 1;}
   if(expected){++count;sum+=d;lo=std::min(lo,uint64_t(d));hi=std::max(hi,uint64_t(d));}
  }
  if(stats[0]!=count||stats[1]!=sum||stats[2]!=(count?lo:0)||stats[3]!=hi||stats[4]!=0)return 2;
 }
 std::cout<<"RTL TEST PASS: two 4096-pixel blocks; full-frame arithmetic checked separately in C simulation"<<std::endl;
 return 0;
}
