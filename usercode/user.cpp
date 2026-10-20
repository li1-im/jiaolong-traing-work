//
// Created by zyfdx on 2026/10/18.
//
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "user.h"
constexpr float G = 9.80665f;
constexpr float PI = 3.14159f;
// void IMU::read()
// {
//     uint8_t tmp[6];
//     bmi088_accel_read_reg(0x12, tmp, 6);
//     acc[0]=tmp[1]<<8 | tmp[0];
//     acc[1]=tmp[3]<<8 | tmp[2];
//     acc[2]=tmp[5]<<8 | tmp[4];
//     bmi088_gyro_read_reg(0x02, tmp, 6);
//     gyro[0]=tmp[1]<<8 | tmp[0];
//     gyro[1]=tmp[3]<<8 | tmp[2];
//     gyro[2]=tmp[5]<<8 | tmp[4];
//
// }
void IMU::acc_calculate()
{
    uint8_t raw_range=0;
    uint8_t rx_acc_data[6];
    float range_g=6.0f;
    bmi088_accel_read_reg(0x41, &raw_range, 1);
    if (raw_range == 0x00) range_g = 3.0f;
    else if (raw_range == 0x01) range_g = 6.0f;
    else if (raw_range == 0x02) range_g = 12.0f;
    else if (raw_range == 0x03) range_g = 24.0f;
    acc_scale = (range_g * G) / 32768.0f;

    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);
    acc[0]=rx_acc_data[1]<<8 | rx_acc_data[0];
    acc[1]=rx_acc_data[3]<<8 | rx_acc_data[2];
    acc[2]=rx_acc_data[5]<<8 | rx_acc_data[4];
    acc_x=acc[0]*acc_scale;
    acc_y=acc[1]*acc_scale;
    acc_z=acc[2]*acc_scale;
}
void IMU::gyro_calculate()
{
    uint8_t raw_range=0;
    uint8_t rx_gyro_data[6];
    float range_dps=2000.0f;
    bmi088_gyro_read_reg(0x0F, &raw_range, 1);
    if(raw_range == 0x00) range_dps = 2000.0f;
    else if(raw_range == 0x01) range_dps = 1000.0f;
    else if(raw_range == 0x02) range_dps = 500.0f;
    else if(raw_range == 0x03) range_dps = 250.0f;
    else if(raw_range == 0x04) range_dps = 125.0f;
    gyro_scale =range_dps*PI/180/32768.0f;
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
    gyro[0]=rx_gyro_data[1]<<8 | rx_gyro_data[0];
    gyro[1]=rx_gyro_data[3]<<8 | rx_gyro_data[2];
    gyro[2]=rx_gyro_data[5]<<8 | rx_gyro_data[4];
    gyro_x=gyro[0]*gyro_scale;
    gyro_y=gyro[1]*gyro_scale;
    gyro_z=gyro[2]*gyro_scale;
}
void bmi088_write_byte(uint8_t tx_data)
{
    HAL_SPI_Transmit(&hspi1,&tx_data, 1, 10);
}
void bmi088_read_byte(uint8_t *rx_data,uint8_t length)
{
    HAL_SPI_Receive(&hspi1,rx_data,length,10);
}
void bmi088_write_reg(uint8_t reg,uint8_t tx_data)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), tx_data};
    HAL_SPI_Transmit(&hspi1, tx, 2, 10);
}
void BMI088_ACCEL_NS_L(void) {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin,GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}
// void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
//     BMI088_GYRO_NS_H();
//     BMI088_ACCEL_NS_L();
//
//     bmi088_write_byte((reg & 0x7F) << 1);//0b 0111 1111
//     bmi088_write_byte(data);
//     BMI088_ACCEL_NS_H();
//     uint8_t tmp[2];
//     bmi088_read_byte(tmp, 2);
//
// }
// void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length)
// {
//     uint8_t tmp;
//     BMI088_GYRO_NS_H();
//     BMI088_ACCEL_NS_L();
//
//     bmi088_write_byte(((reg & 0x7F) << 1) | 0x01);
//     bmi088_read_byte(&tmp,1);
//     bmi088_read_byte(&tmp,1);
//     bmi088_read_byte(rx_data,length);
//     BMI088_ACCEL_NS_H();
// }
// void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length)
// {
//     uint8_t tmp;
//     BMI088_ACCEL_NS_H();
//     BMI088_GYRO_NS_L();
//
//     bmi088_write_byte(reg |0x80);
//     bmi088_read_byte(&tmp,1);
//     bmi088_read_byte(rx_data,length);
//     BMI088_GYRO_NS_H();
//
// }
// void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data)
// {
//     BMI088_ACCEL_NS_H();
//     BMI088_GYRO_NS_L();
//
//     bmi088_write_byte(reg & 0x7F);//0b 0111 1111
//     bmi088_write_byte(tx_data);
//
//     BMI088_GYRO_NS_H();
//     uint8_t tmp[2];
//     bmi088_read_byte(tmp, 2);
// }
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);
    BMI088_ACCEL_NS_H();
}

void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    uint8_t dummy;
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();
    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(&dummy, 1);          // 加速度计：丢 1 个 dummy
    bmi088_read_byte(rx_data, length);
    BMI088_ACCEL_NS_H();
}

void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();
    bmi088_write_byte(reg | 0x80);
    bmi088_read_byte(rx_data, length);    // 陀螺仪：没有 dummy
    BMI088_GYRO_NS_H();
}

void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(tx_data);
    BMI088_GYRO_NS_H();
}
uint8_t bmi088_init(void) {
    uint8_t dummy = 0;
    uint8_t acc_id = 0;
    uint8_t gyro_id = 0;
    uint8_t acc_power[2] = {0};
    uint8_t acc_config[2] = {0};
    uint8_t gyro_config[3] = {0};

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    HAL_Delay(1);

    // Switch ACCEL to SPI，第一次读到的值不用
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Soft Reset ACCEL
    bmi088_accel_write_single_reg(0x7E, 0xB6);
    HAL_Delay(1);

    // Soft Reset GYRO
    bmi088_gyro_write_single_reg(0x14, 0xB6);
    HAL_Delay(30);

    // ACCEL 软复位后再做一次虚拟读取
    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(1);

    // Check Chip ID
    bmi088_accel_read_reg(0x00, &acc_id, 1);
    if (acc_id != 0x1E) {
        return 1;
    }
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) {
        return 2;
    }

    // Switch ACCEL to Active Mode
    bmi088_accel_write_single_reg(0x7C, 0x00); // ACC_PWR_CONF
    HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7D, 0x04); // ACC_PWR_CTRL
    HAL_Delay(5);

    // ACCEL Range / ODR / BW
    bmi088_accel_write_single_reg(0x41, 0x01); // ACC_RANGE
    HAL_Delay(1);
    bmi088_accel_write_single_reg(0x40, 0xAB); // ACC_CONF
    HAL_Delay(40);

    // GYRO Power / Range / ODR / BW
    bmi088_gyro_write_single_reg(0x11, 0x00); // GYRO_LPM1
    HAL_Delay(30);
    bmi088_gyro_write_single_reg(0x0F, 0x00); // GYRO_RANGE
    HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x10, 0x02); // GYRO_BANDWIDTH
    HAL_Delay(10);

    // Read back，连续读取相邻的配置寄存器
    bmi088_accel_read_reg(0x7C, acc_power, 2);
    if (acc_power[0] != 0x00 || acc_power[1] != 0x04) {
        return 3;
    }
    bmi088_accel_read_reg(0x40, acc_config, 2);
    if (acc_config[0] != 0xAB || (acc_config[1] & 0x03) != 0x01) {
        return 4;
    }
    bmi088_gyro_read_reg(0x0F, gyro_config, 3);
    if ((gyro_config[0] & 0x07) != 0x00 ||
        (gyro_config[1] & 0x07) != 0x02 || gyro_config[2] != 0x00) {
        return 5;
        }

    return 6;
}
