//
// Created by zyfdx on 2026/10/19.
//
#include "user.h"
#include "main.h"
extern volatile uint32_t tick;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        tick++;
    }
}