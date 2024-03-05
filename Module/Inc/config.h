#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG

#define STABILIZER_TASK_PRIORITY    osPriorityRealtime
#define IMU_TASK_PRIORITY           osPriorityNormal
#define FLOW_TASK_PRIORITY          osPriorityNormal
#define SYSTEM_TASK_PRIORITY        osPriorityNormal
#define TOF_TASK_PRIORITY           osPriorityNormal

/*
 * Make sure the stack size is large enough to handle the task.
 */
#define STABILIZER_TASK_STACK_SIZE  configMINIMAL_STACK_SIZE
#define IMU_TASK_STACK_SIZE         (2 * configMINIMAL_STACK_SIZE)
#define FLOW_TASK_STACK_SIZE        configMINIMAL_STACK_SIZE
#define SYSTEM_TASK_STACK_SIZE      (2 * configMINIMAL_STACK_SIZE)
#define TOF_TASK_STACK_SIZE         configMINIMAL_STACK_SIZE

#ifdef __cplusplus
}
#endif
#endif