// Diagnostic only: forward the installed runtime conversion without changing it.
#include <dlfcn.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
static void* real(const char* name) {
  void* h=dlopen("libflexmlrt.so",RTLD_NOW|RTLD_NOLOAD);
  void* p=h?dlsym(h,name):nullptr;
  if(!p){fprintf(stderr,"trace lookup failed %s\n",name);abort();} return p;
}
extern "C" void trace_bf(const uint16_t*,float*,size_t) asm("_Z33bfloat16_to_float_avx512_unrolledPKtPfm");
extern "C" void trace_bf(const uint16_t* s,float* d,size_t n){
  static auto f=(void(*)(const uint16_t*,float*,size_t))real("_Z33bfloat16_to_float_avx512_unrolledPKtPfm");
  static unsigned id=0;unsigned k=id++;
  size_t z=0;for(size_t i=0;i<n;i++)z+=s[i]==0;
  fprintf(stderr,"TRACE_BF16_TO_F32 id=%u n=%zu zeros=%zu\n",k,n,z);
  const char* dir=getenv("PI05_TRACE_DIR");
  if(dir){auto path=std::string(dir)+"/raw_bf16_"+std::to_string(k)+".bin";auto fp=fopen(path.c_str(),"wb");if(fp){fwrite(s,2,n,fp);fclose(fp);}}
  f(s,d,n);
}
extern "C" void trace_fp(const float*,uint16_t*,size_t,int) asm("_ZN8flexmlrt3Qdq19convert_f32_to_bf16EPKfPtmi");
extern "C" void trace_fp(const float* s,uint16_t* d,size_t n,int mode){
 static auto f=(void(*)(const float*,uint16_t*,size_t,int))real("_ZN8flexmlrt3Qdq19convert_f32_to_bf16EPKfPtmi");
 fprintf(stderr,"TRACE_F32_TO_BF16 n=%zu mode=%d\n",n,mode);f(s,d,n,mode);
}
