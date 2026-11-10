#include "depth_gate.h"
void depth_gate(const uint16_t *input, uint16_t *output, uint64_t *stats,
                uint32_t count, uint16_t lower, uint16_t upper) {
#pragma HLS INTERFACE m_axi port=input offset=slave bundle=gmem0 depth=307200
#pragma HLS INTERFACE m_axi port=output offset=slave bundle=gmem1 depth=307200
#pragma HLS INTERFACE m_axi port=stats offset=slave bundle=gmem2 depth=5
#pragma HLS INTERFACE s_axilite port=input bundle=control
#pragma HLS INTERFACE s_axilite port=output bundle=control
#pragma HLS INTERFACE s_axilite port=stats bundle=control
#pragma HLS INTERFACE s_axilite port=count bundle=control
#pragma HLS INTERFACE s_axilite port=lower bundle=control
#pragma HLS INTERFACE s_axilite port=upper bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
    if (count > 307200 || lower > upper) {
        for (int j=0;j<4;++j) stats[j]=0;
        stats[4]=1;
        return;
    }
    uint32_t valid=0;
    uint64_t sum=0;
    uint16_t minimum=65535, maximum=0;
    for (uint32_t i=0;i<count;++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=0 max=307200 avg=307200
        const uint16_t d=input[i];
        const bool keep=(d!=0 && d!=65535 && d>=lower && d<=upper);
        output[i]=keep ? d : 0;
        if (keep) {
            ++valid;
            sum+=d;
            if (d<minimum) minimum=d;
            if (d>maximum) maximum=d;
        }
    }
    stats[0]=valid; stats[1]=sum;
    stats[2]=valid ? minimum : 0; stats[3]=maximum; stats[4]=0;
}
