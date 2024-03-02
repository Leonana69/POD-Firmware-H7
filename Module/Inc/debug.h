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

#define DEBUG_PRINT(FMT, ...) eprintf(debugUartPutChar, FMT, ## __VA_ARGS__)

#ifdef __cplusplus
}
#endif
#endif