#ifndef __LED_SEQ_H__
#define __LED_SEQ_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "led.h"

typedef struct {
    uint8_t led;
    uint8_t action;
    uint16_t delay;
} ledAction_t;

typedef struct {
    uint8_t count;
    ledAction_t *action;
} ledSeq_t;

uint32_t ledSeqInit();
void ledSeqSend(ledSeq_t *ledSeq);

#define LED_SEQ_PROTECT(...) __VA_ARGS__ 
#define LED_SEQ_DEF(NAME, LED_ACTION) \
    ledAction_t ledAction##NAME[] = LED_ACTION; \
    ledSeq_t ledSeq##NAME = { \
        .count = sizeof(ledAction##NAME) / sizeof(ledAction_t), \
        .action = ledAction##NAME \
    }; \
    void ledSeqSend##NAME() { ledSeqSend(&ledSeq##NAME); }
#define LED_SEQ_DECL(NAME) \
    extern ledSeq_t ledSeq##NAME; \
    void ledSeqSend##NAME();
#define LED_SEQ_CALL(NAME) \
    ledSeqSend##NAME()

LED_SEQ_DECL(Test);
LED_SEQ_DECL(TakeOff);
LED_SEQ_DECL(Land);
LED_SEQ_DECL(Link);

#ifdef __cplusplus
}
#endif

#endif // __LED_SEQ_H__