//
// Created by zyfdx on 2026/10/19.
//
#include "user.h"
#include "main.h"
IMU imu;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        imu.acc_calculate();
        imu.gyro_calculate();
    }
}