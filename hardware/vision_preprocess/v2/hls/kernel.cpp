#include "kernel.hpp"
#include "common/xf_common.hpp"
#include "imgproc/xf_resize.hpp"
#include "dnn/xf_insertBorder.hpp"
using Input=xf::cv::Mat<XF_8UC3,1080,1920,XF_NPPC1,2>;
using Canvas=xf::cv::Mat<XF_8UC3,640,640,XF_NPPC1,2>;
static void load(hls::stream<Pixel>& input,Input& image,int count){
    for(int i=0;i<count;++i){
#pragma HLS PIPELINE II=1
        Pixel p=input.read();image.write(i,p.data.range(23,0));
    }
}
static void store(Canvas& image,hls::stream<Pixel>& output,int count){
    for(int i=0;i<count;++i){
#pragma HLS PIPELINE II=1
        Pixel p;p.data=0;p.data.range(23,0)=image.read(i);p.keep=15;p.strb=15;p.last=(i==count-1);output.write(p);
    }
}
void vpr2_letterbox(hls::stream<Pixel>& input,hls::stream<Pixel>& output,
                   unsigned short ih,unsigned short iw,unsigned short rh,unsigned short rw,
                   unsigned short oh,unsigned short ow,unsigned char pad){
#pragma HLS INTERFACE axis port=input
#pragma HLS INTERFACE axis port=output
#pragma HLS INTERFACE ap_none port=ih
#pragma HLS INTERFACE ap_none port=iw
#pragma HLS INTERFACE ap_none port=rh
#pragma HLS INTERFACE ap_none port=rw
#pragma HLS INTERFACE ap_none port=oh
#pragma HLS INTERFACE ap_none port=ow
#pragma HLS INTERFACE ap_none port=pad
#pragma HLS INTERFACE ap_ctrl_hs port=return
    Input source(ih,iw);Canvas resized(rh,rw),canvas(oh,ow);
#pragma HLS DATAFLOW
    load(input,source,int(ih)*iw);
    xf::cv::resize<XF_INTERPOLATION_BILINEAR,XF_8UC3,1080,1920,640,640,XF_NPPC1,false,4,2,2>(source,resized);
    xf::cv::insertBorder<XF_8UC3,640,640,640,640,XF_NPPC1,2,2>(resized,canvas,pad);
    store(canvas,output,int(oh)*ow);
}
