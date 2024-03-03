#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG

#define STABILIZER_TASK_PRIORITY    osPriorityRealtime
#define FLOW_TASK_PRIORITY          osPriorityNormal

#define STABILIZER_TASK_STACK_SIZE  configMINIMAL_STACK_SIZE
#define FLOW_TASK_STACK_SIZE        configMINIMAL_STACK_SIZE

#ifdef __cplusplus
}
#endif
#endif