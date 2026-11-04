#pragma once
#include <ap_axi_sdata.h>
#include <hls_stream.h>
using Pixel=ap_axiu<32,0,0,0>;
void vpr2_letterbox(hls::stream<Pixel>& input,hls::stream<Pixel>& output,
                   unsigned short ih,unsigned short iw,unsigned short rh,unsigned short rw,
                   unsigned short oh,unsigned short ow,unsigned char pad);
