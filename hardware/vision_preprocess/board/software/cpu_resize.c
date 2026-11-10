// Exact VPR1 floor-nearest CPU baseline, RGB888 or RGBX8888, one thread.
#include <stdint.h>
#include <string.h>
int vpr_cpu_resize(const uint8_t *src, uint8_t *dst, int iw,int ih,int ow,int oh,int channels) {
    if(!src || !dst || iw<1 || ih<1 || ow<1 || oh<1 || ow>iw || oh>ih || iw>1920 || ih>1080 || (channels!=3 && channels!=4)) return -1;
    if(iw==ow && ih==oh) {memcpy(dst,src,(size_t)iw*ih*channels);return 0;}
    int xs[1920];
    for(int x=0;x<ow;x++) xs[x]=(int)((int64_t)x*iw/ow)*channels;
    for(int y=0;y<oh;y++) {
        const uint8_t *row=src+(size_t)((int64_t)y*ih/oh)*iw*channels;
        uint8_t *out=dst+(size_t)y*ow*channels;
        if(channels==4) for(int x=0;x<ow;x++) memcpy(out+4*x,row+xs[x],4);
        else for(int x=0;x<ow;x++) {const uint8_t *p=row+xs[x];out[3*x]=p[0];out[3*x+1]=p[1];out[3*x+2]=p[2];}
    }
    return 0;
}
