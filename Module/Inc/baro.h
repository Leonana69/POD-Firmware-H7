#ifndef __BARO_H__
#define __BARO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t baroInit(void);
float pressureToAltitude(float pressure);

#ifdef __cplusplus
}
#endif

#endif // __BARO_H__