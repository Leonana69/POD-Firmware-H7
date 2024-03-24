#include "led.h"
#include "gpio.h"
#include "config.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint16_t polarity;
} led_t;

static uint8_t ledCount;
static led_t led[] = {
    { .port = LED_COM_GPIO_PORT, .pin = LED_COM_GPIO_PIN },
    { .port = LED_R_O_GPIO_PORT, .pin = LED_R_O_GPIO_PIN },
    { .port = LED_R_B_GPIO_PORT, .pin = LED_R_B_GPIO_PIN },
    { .port = LED_L_O_GPIO_PORT, .pin = LED_L_O_GPIO_PIN },
    { .port = LED_L_B_GPIO_PORT, .pin = LED_L_B_GPIO_PIN }
};

void ledInit(void) {
    ledCount = sizeof(led) / sizeof(led_t);
    ledClearAll();
}

uint8_t ledIdGet(uint8_t collection) {
    uint8_t id = 31 - __builtin_clz(collection);
    return id >= ledCount ? 0 : id;
}

void ledToggle(uint8_t collection) {
    while (collection) {
        uint8_t id = ledIdGet(collection);
        HAL_GPIO_TogglePin(led[id].port, led[id].pin);
        collection &= ~(1 << id);
    }
}

void ledSet(uint8_t collection, uint8_t state) {
    while (collection) {
        uint8_t id = ledIdGet(collection);
        HAL_GPIO_WritePin(led[id].port, led[id].pin, state);
        collection &= ~(1 << id);
    }
}

void ledClearAll(void) {
    for (uint8_t i = 0; i < ledCount; i++) {
        HAL_GPIO_WritePin(led[i].port, led[i].pin, led[i].polarity);
    }
}