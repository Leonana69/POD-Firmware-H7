#ifndef __UTILS_H__
#define __UTILS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void sensorsDelayUs(uint32_t period, void *intf_ptr);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_H__