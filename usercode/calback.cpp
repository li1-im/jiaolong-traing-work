//
// Created by zyfdx on 2026/10/19.
//
#include "user.h"
#include "main.h"
IMU imu;
//  volatile uint32_t last_tick=0;
// volatile int8_t delta=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // uint32_t tick=HAL_GetTick();
        imu.acc_calculate();
        imu.gyro_calculate();
        imu.sensorDataTransform();
        // delta=tick-last_tick;
        imu.updateAttitude(0.001f);
        // last_tick=tick;
    }
}
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Instance == TIM6) {
//         imu.acc_calculate();
//         imu.gyro_calculate();
//         imu.sensorDataTransform();
//         // uint32_t tick=HAL_GetTick();
//         imu.updateAttitude(0.002f);
//     }
// }