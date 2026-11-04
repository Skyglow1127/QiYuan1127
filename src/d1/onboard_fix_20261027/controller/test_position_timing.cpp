#include "position_timing.hpp"
#include <cassert>
#include <limits>
int main() {
 int interval=-1;
 assert(d1_timing_fix::delay_ms(-4.8,-4.8,1000,interval) && interval==1000);
 assert(d1_timing_fix::delay_ms(-4.8,-4.8,0,interval) && interval==1);
 assert(d1_timing_fix::delay_ms(-4.8,-2.8,1000,interval) && interval==1317);
 assert(d1_timing_fix::delay_ms(-2.8,-4.8,1000,interval) && interval==1317);
 assert(!d1_timing_fix::delay_ms(0,0,-1,interval));
 assert(!d1_timing_fix::delay_ms(0,0,65536,interval));
 assert(!d1_timing_fix::delay_ms(std::numeric_limits<double>::quiet_NaN(),0,1000,interval));
 assert(!d1_timing_fix::delay_ms(0,std::numeric_limits<double>::infinity(),1000,interval));
}
