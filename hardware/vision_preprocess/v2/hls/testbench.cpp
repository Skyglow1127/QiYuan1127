#include "kernel.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <stdexcept>
static long checks=0;static int cases=0;
static void check(bool value,const char* what){++checks;if(!value)throw std::runtime_error(what);}
static std::vector<unsigned char> read(const std::string& name,size_t count){
    std::ifstream f(name,std::ios::binary);check(bool(f),"Missing frozen vector");std::vector<unsigned char> result(count);
    f.read((char*)result.data(),count);check(f.gcount()==count,"Incomplete vector");return result;
}
int main(int argc,char** argv){
    try{
        check(argc==2,"Vector directory argument required");std::string folder=argv[1];
        std::ifstream manifest(folder+"/cases.txt");check(bool(manifest),"Missing manifest");
        std::string name;int iw,ih,rw,rh,ow,oh,pad;
        while(manifest>>name>>iw>>ih>>rw>>rh>>ow>>oh>>pad){
            auto source=read(folder+"/"+name+".input.rgb",iw*ih*3);auto expected=read(folder+"/"+name+".expected.rgb",ow*oh*3);
            hls::stream<Pixel> input,output;
            for(int i=0;i<iw*ih;++i){Pixel p;p.data=unsigned(source[3*i])|(unsigned(source[3*i+1])<<8)|(unsigned(source[3*i+2])<<16);p.keep=15;p.strb=15;p.last=i==iw*ih-1;input.write(p);}
            vpr2_letterbox(input,output,ih,iw,rh,rw,oh,ow,pad);
            int maxerr=0;long sum=0;int changed=0;int left=(ow-rw)/2,top=(oh-rh)/2;std::vector<unsigned char> actual(ow*oh*3);
            for(int y=0;y<oh;++y)for(int x=0;x<ow;++x){
                check(!output.empty(),"Missing output pixel");Pixel p=output.read();
                check(p.keep==15&&p.strb==15,"Output byte qualifiers");check(bool(p.last)==(y==oh-1&&x==ow-1),"Output EOF TLAST");check(p.data.range(31,24)==0,"RGBX high byte");
                for(int c=0;c<3;++c){int i=(y*ow+x)*3+c,value=p.data.range(c*8+7,c*8);actual[i]=value;int d=std::abs(value-int(expected[i]));maxerr=std::max(maxerr,d);sum+=d;changed+=(d!=0);if(y<top||y>=top+rh||x<left||x>=left+rw)check(value==pad,"Exact padding");}
            }
            check(input.empty()&&output.empty(),"Packet count mismatch");
            std::cout<<"CASE "<<cases<<" "<<iw<<"x"<<ih<<" -> "<<ow<<"x"<<oh<<" content="<<rw<<"x"<<rh<<" max="<<maxerr<<" mae="<<double(sum)/(ow*oh*3)<<" changed="<<changed<<std::endl;
            std::ofstream f(folder+"/"+name+".actual.rgb",std::ios::binary);f.write((char*)actual.data(),actual.size());
            check(maxerr<=2,"Bilinear maximum byte error > 2");check(double(sum)/(ow*oh*3)<=0.5,"Bilinear MAE > .5");++cases;
        }
        check(cases==14,"Required case count");std::cout<<"TEST_PASS VPR2_CSIM cases="<<cases<<" checks="<<checks<<std::endl;return 0;
    }catch(const std::exception& e){std::cerr<<"TEST_FAIL "<<e.what()<<std::endl;return 1;}
}
