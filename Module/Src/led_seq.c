#include "led_seq.h"
#include "freeRTOS_helper.h"
#include "config.h"
#include "system.h"

STATIC_QUEUE_DEF(ledSeqQueue, 10, ledSeq_t);
STATIC_TASK_DEF(ledSeqTask, LED_SEQ_TASK_PRIORITY, LED_SEQ_TASK_STACK_SIZE);

LED_SEQ_DEF(Test, LED_SEQ_PROTECT({
    { LED_COM | LED_R_O | LED_R_B | LED_L_O | LED_L_B, LED_ON, 500 },
    { LED_COM | LED_R_O | LED_R_B | LED_L_O | LED_L_B, LED_OFF, 500 },
    { LED_COM | LED_R_O | LED_R_B | LED_L_O | LED_L_B, LED_ON, 500 },
    { LED_COM | LED_R_O | LED_R_B | LED_L_O | LED_L_B, LED_OFF, 0 },
}));

LED_SEQ_DEF(TakeOff, LED_SEQ_PROTECT({
    { LED_R_O | LED_L_O, LED_ON, 200 },
    { LED_R_O | LED_L_O, LED_OFF, 200 },
}));

LED_SEQ_DEF(Land, LED_SEQ_PROTECT({
    { LED_R_O | LED_L_O, LED_ON, 200 },
    { LED_R_O | LED_L_O, LED_OFF, 200 },
}));

LED_SEQ_DEF(Link, LED_SEQ_PROTECT({
    { LED_COM, LED_ON, 200 },
    { LED_COM, LED_OFF, 200 },
    { LED_COM, LED_ON, 200 },
    { LED_COM, LED_OFF, 0 },
}));

uint32_t ledSeqInit() {
    ledInit();
    STATIC_QUEUE_INIT(ledSeqQueue);
    STATIC_QUEUE_SEND(ledSeqQueue, &ledSeqTest, 0);
    STATIC_TASK_INIT(ledSeqTask, NULL);
    return TASK_INIT_SUCCESS;
}

void ledSeqSend(ledSeq_t *ledSeq) {
    STATIC_QUEUE_SEND(ledSeqQueue, ledSeq, 0);
}

void ledSeqTask(void *pvParameters) {
    ledSeq_t ledSeq;
    systemWaitStart();
    while (1) {
        STATIC_QUEUE_RECEIVE(ledSeqQueue, &ledSeq, osWaitForever);
        for (int i = 0; i < ledSeq.count; i++) {
            ledSet(ledSeq.action[i].led, ledSeq.action[i].action);
            osDelay(ledSeq.action[i].delay);
        }
    }
}