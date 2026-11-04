#include <stdint.h>
#include <stddef.h>
#if !defined(__aarch64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Requires little-endian AArch64"
#endif
static inline void fence(void){__asm__ volatile("dsb sy" ::: "memory");}
uint32_t mmio_read32(void *p,size_t o){fence();uint32_t v=*(volatile uint32_t*)((char*)p+o);fence();return v;}
void mmio_write32(void *p,size_t o,uint32_t v){fence();*(volatile uint32_t*)((char*)p+o)=v;fence();}
void mmio_fence(void){fence();}
