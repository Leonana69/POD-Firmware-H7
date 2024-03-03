#include "utils.h"
#include "cmsis_os.h"

/*! @brief Sensor delay_us function */
void sensorsDelayUs(uint32_t period, void *intf_ptr) {
  uint32_t period_ms = (period + 500) / 1000;
  if (period_ms == 0) {
    osDelay(1);
  } else
    osDelay(period_ms);
}