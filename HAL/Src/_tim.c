#include "_tim.h"
#include "_config.h"

void _TIM_Init() {
    HAL_TIM_Base_Start(&DELAY_US_TIM);
}