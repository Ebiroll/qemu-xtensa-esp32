#ifndef __NRF51_HELPER_H__
#define __NRF51_HELPER_H__

#include "qemu/osdep.h"

const char * eptostr(uint32_t ep_addr);
const char * regtostr(uint32_t reg);


#define GENERICDP(pref, ...) do{ \
    fputs(pref, stdout); \
    printf(__VA_ARGS__); \
    fputs("\n", stdout); \
}while(0)

#define NODP() do{}while(0)

#if 0
#define CCMDP(...) GENERICDP("[CCM] ", __VA_ARGS__)
#else
#define CCMDP(...) NODP()
#endif

#if 0
#define PPIDP(...) GENERICDP("[PPI] ", __VA_ARGS__)
#else
#define PPIDP(...) NODP()
#endif

#endif
