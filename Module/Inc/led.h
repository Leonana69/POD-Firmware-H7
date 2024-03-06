#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "_config.h"
#include <stdint.h>

#define LED_COM 0
#define LED_R_O 1
#define LED_R_B 2
#define LED_L_O 3
#define LED_L_B 4

#define LED_ON  1
#define LED_OFF 0

#define ERR_LED LED_L_O

uint32_t ledInit(void);
void ledToggle(uint8_t id);
void ledSet(uint8_t id, uint8_t state);
void ledClearAll(void);

#ifdef __cplusplus
}
#endif

#endif // __LED_H__