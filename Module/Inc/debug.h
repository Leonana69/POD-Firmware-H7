#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "_usart.h"
#include "eprintf.h"

#ifndef DEBUG_FMT
#define DEBUG_FMT(FMT) FMT
#endif

#define DEBUG_WRITE(FMT, ...) eprintf(debugUartPutChar, DEBUG_FMT(FMT), ## __VA_ARGS__)

#ifdef MODULE_NAME
#define DEBUG_PRINT(FMT, ...) DEBUG_WRITE("%s: "FMT, MODULE_NAME, ## __VA_ARGS__)
#else
#define DEBUG_PRINT(FMT, ...) DEBUG_WRITE(FMT, ## __VA_ARGS__)
#endif

#ifdef __cplusplus
}
#endif
#endif // __DEBUG_H__