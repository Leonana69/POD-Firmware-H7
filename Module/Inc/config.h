#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG

#define SYSTEM_TASK_PRIORITY        osPriorityNormal

#define STABILIZER_TASK_PRIORITY    osPriorityRealtime
#define IMU_TASK_PRIORITY           osPriorityNormal
#define BARO_TASK_PRIORITY          osPriorityNormal
#define FLOW_TASK_PRIORITY          osPriorityNormal
#define TOF_TASK_PRIORITY           osPriorityNormal
#define ESTIMATOR_TASK_PRIORITY     osPriorityNormal

/*
 * Make sure the stack size is large enough to handle the task.
 */
#define SYSTEM_TASK_STACK_SIZE      (2 * configMINIMAL_STACK_SIZE)

#define STABILIZER_TASK_STACK_SIZE  (2 * configMINIMAL_STACK_SIZE)
#define IMU_TASK_STACK_SIZE         (2 * configMINIMAL_STACK_SIZE)
#define BARO_TASK_STACK_SIZE        (2 * configMINIMAL_STACK_SIZE)
#define FLOW_TASK_STACK_SIZE        (2 * configMINIMAL_STACK_SIZE)
#define TOF_TASK_STACK_SIZE         (2 * configMINIMAL_STACK_SIZE)
#define ESTIMATOR_TASK_STACK_SIZE   (4 * configMINIMAL_STACK_SIZE)

#define TASK_INIT_SUCCESS           0
#define TASK_INIT_FAILED(index)     (1 << index)

#define STABILIZER_TASK_INDEX       0
#define IMU_TASK_INDEX              1
#define FLOW_TASK_INDEX             2
#define TOF_TASK_INDEX              3
#define ESTIMATOR_TASK_INDEX        4
#define BARO_TASK_INDEX             5

#ifdef __cplusplus
}
#endif
#endif // __CONFIG_H__