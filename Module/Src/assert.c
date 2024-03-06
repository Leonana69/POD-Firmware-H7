#include "assert.h"
#include "debug.h"
#include "led.h"
#include "FreeRTOS.h"
#include "main.h"

void assertFail(char *exp, char *file, int line) {
    portDISABLE_INTERRUPTS();
    DEBUG_PRINT("Assert failed %s:%d\n", file, line);
    ledClearAll();
    ledSet(ERR_LED, LED_ON);
    // powerStop();
    HAL_NVIC_SystemReset();
}