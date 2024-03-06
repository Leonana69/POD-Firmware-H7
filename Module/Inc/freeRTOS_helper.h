#ifndef __FREERTOS_HELPER_H__
#define __FREERTOS_HELPER_H__

#ifdef  __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "cmsis_os2.h"

// Static Queue
#define STATIC_QUEUE_DEF(NAME, SIZE, TYPE) \
    static osMessageQueueId_t NAME##_id; \
    static TYPE NAME##_buffer[SIZE]; \
    static StaticQueue_t NAME##_queue_cb; \
    static const osMessageQueueAttr_t NAME##_queue_attr = { \
        .name = #NAME, \
        .cb_mem = NAME##_queue_cb, \
        .cb_size = sizeof(NAME##_queue_cb), \
        .mq_mem = NAME##_buffer, \
        .mq_size = sizeof(NAME##_buffer), \
    }; \
    static void NAME##_queue_init(void) { \
        NAME##_id = osMessageQueueNew(SIZE, sizeof(TYPE), &NAME##_queue_attr); \
    }

#define STATIC_QUEUE_INIT(NAME) \
    NAME##_queue_init()

#define STATIC_QUEUE_SEND(NAME, DATA, TIMEOUT) \
    osMessageQueuePut(NAME##_id, &DATA, 0, TIMEOUT)

#define STATIC_QUEUE_RECEIVE(NAME, DATA, TIMEOUT) \
    osMessageQueueGet(NAME##_id, &DATA, NULL, TIMEOUT)

// Static Task
#define STATIC_TASK_DEF(NAME, PRIORITY, STACK_SIZE) \
    void NAME(void *argument); \
    static osThreadId_t NAME##_id; \
    static uint32_t NAME##_stack[STACK_SIZE]; \
    static StaticTask_t NAME##_task_cb; \
    static const osThreadAttr_t NAME##_task_attr = { \
        .name = #NAME, \
        .cb_mem = &NAME##_task_cb, \
        .cb_size = sizeof(NAME##_task_cb), \
        .stack_mem = NAME##_stack, \
        .stack_size = sizeof(NAME##_stack), \
        .priority = (osPriority_t)PRIORITY, \
    }; \
    static void NAME##_task_init(void *argument) { \
        NAME##_id = osThreadNew(NAME, argument, &NAME##_task_attr); \
    }

#define STATIC_TASK_INIT(NAME, ARGUMENT) \
    NAME##_task_init(ARGUMENT)

// Static Mutex
#define STATIC_MUTEX_DEF(NAME) \
    static osMutexId_t NAME##_id; \
    static StaticMutex_t NAME##_mutex_cb; \
    static const osMutexAttr_t NAME##_mutex_attr = { \
        .name = #NAME, \
        .cb_mem = &NAME##_mutex_cb, \
        .cb_size = sizeof(NAME##_mutex_cb), \
    }; \
    static void NAME##_mutex_init(void) { \
        NAME##_id = osMutexNew(&NAME##_mutex_attr); \
    }

#define STATIC_MUTEX_INIT(NAME) \
    NAME##_mutex_init()

#define STATIC_MUTEX_LOCK(NAME, TIMEOUT) \
    osMutexAcquire(NAME##_id, TIMEOUT)

#define STATIC_MUTEX_UNLOCK(NAME) \
    osMutexRelease(NAME##_id)

// Static Semaphore
#define STATIC_SEMAPHORE_DEF(NAME) \
    static osSemaphoreId_t NAME##_id; \
    static StaticSemaphore_t NAME##_semaphore_cb; \
    static const osSemaphoreAttr_t NAME##_semaphore_attr = { \
        .name = #NAME, \
        .cb_mem = &NAME##_semaphore_cb, \
        .cb_size = sizeof(NAME##_semaphore_cb), \
    }; \
    static void NAME##_semaphore_init(uint32_t max_count, uint32_t init_count) { \
        NAME##_id = osSemaphoreNew(max_count, init_count, &NAME##_semaphore_attr); \
    }

#define STATIC_SEMAPHORE_INIT(NAME, MAX_COUNT, INIT_COUNT) \
    NAME##_semaphore_init(MAX_COUNT, INIT_COUNT)

#define STATIC_SEMAPHORE_WAIT(NAME, TIMEOUT) \
    osSemaphoreAcquire(NAME##_id, TIMEOUT)

#define STATIC_SEMAPHORE_RELEASE(NAME) \
    osSemaphoreRelease(NAME##_id)

#ifdef  __cplusplus
}
#endif

#endif // __FREERTOS_HELPER_H__