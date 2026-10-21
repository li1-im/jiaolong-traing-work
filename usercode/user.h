//
// Created by zyfdx on 2026/10/18.
//
#include "stm32f4xx_hal.h"
#ifndef SPI_USER_H
#define SPI_USER_H
const float R_imu[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, -1.0f, 0.0f},
                         {0.0f, 0.0f, 1.0f}};
class IMU
{
    public:
        IMU():fourdimen{1.0f,0,0,0},acc{0, 0, 0}, gyro{0, 0, 0} ,gyro_bias_{0,0,0},accel_bias_{0,0,0}{}
        void acc_calculate();
        void gyro_calculate();
        void read();
        void sensorDataTransform();
        void calibrate_gyro(uint32_t samples, TIM_HandleTypeDef *htim);
        void updateAttitude(float dt);
        float wrapPi(float a);
        float roll_  = 0.0f;
        float pitch_ = 0.0f;
        float yaw_   = 0.0f;
    private:
        float fourdimen[4];
         int16_t acc[3];
         int16_t gyro[3];
         float  acc_scale;
         float gyro_scale;
         float acc_x, acc_y, acc_z;
         float gyro_x, gyro_y, gyro_z;
        float  gyro_sensor_[3];
        float  gyro_bias_[3];
        float accel_sensor_[3];
        float accel_bias_[3];


};
void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t *rx_data,uint8_t length);
void bmi088_write_reg(uint8_t reg,uint8_t tx_data);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data);
uint8_t bmi088_init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
#endif //SPI_USER_H
