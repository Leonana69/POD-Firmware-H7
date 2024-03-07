#ifndef __STABILIZER_H__
#define __STABILIZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t stabilizerInit(void);
void stabilizerReset(void);

#ifdef __cplusplus
}
#endif

#endif // __STABILIZER_H__