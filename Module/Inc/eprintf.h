#ifndef	__EPRINTF_H__
#define __EPRINTF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <stdint.h>

/**
 * putc function pointer definition
 */
typedef void (*transmit)(const uint8_t *data, uint16_t len);

/**
 * Light printf implementation
 * @param[in] putcf Putchar function to be used by Printf
 * @param[in] fmt Format string
 * @param[in] ... Parameters to print
 * @return the number of character printed
 */
int eprintf(transmit transmit, const char * fmt, ...)
    __attribute__ (( format(printf, 2, 3) ));

/**
 * Light printf implementation
 * @param[in] fmt Format string
 * @param[in] ap Parameters to print
 * @return the number of character printed
 */
void evprintf(const char * fmt, va_list ap);
#ifdef __cplusplus
}
#endif
#endif //__EPRINTF_H__
