/**
 ******************************************************************************
 * @file    imu_monitor.cpp/h
 * @brief   IMU管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/imu_monitor.h"
#include "base/common/math.h"
#include "base/imu/driver/bmi088.h"
#include "base/imu/driver/ist8310.h"
#include "base/imu/driver/mpu6500.h"

namespace boardimu {

void initSensor(imu::RawData_t& raw_data);
void readSensor(imu::RawData_t& raw_data);

const bool bmi088_enabled = true;
const bool ist8310_enabled = false;
const bool mpu6500_enabled = true;

struct Param {
  const float dt = 1e-3f;
  const float kg = 5e-3f;
  const float km = 0;
  const float g_thres = 0.3f;

  const float R_imu[3][3] = {{1.0f, 0.0f, 0.0f},
                             {0.0f, 1.0f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};
  const float R_mag[3][3] = {{0.93f, 0.0f, 0.0f},
                             {0.0f, 1.03f, 0.0f},
                             {0.0f, 0.0f, 1.0f}};

  const float gyro_bias[3] = {2.6e-3f, 3e-4f, 3.5e-4f};
  const float accel_bias[3] = {0, 0, 0};
  const float mag_bias[3] = {-13.0f, -0.5f, 30.0f};
  const float accel_sen = 1;
} param;

imu::TempControl temp(BOARD_IMU_HEAT_TIM, BOARD_IMU_HEAT_CHANNEL,
                      PID(5000, 0, 0, 500, 5000));

}

IMU board_imu(boardimu::param.dt, boardimu::param.kg, boardimu::param.km,
              boardimu::param.g_thres, boardimu::param.R_imu,
              boardimu::param.R_mag, boardimu::param.gyro_bias,
              boardimu::param.accel_bias, boardimu::param.mag_bias,
              boardimu::param.accel_sen, &boardimu::readSensor);

namespace extimu {
}

void boardimu::initSensor(imu::RawData_t& raw_data) {
#ifdef DBC
  if (boardimu::bmi088_enabled) {
    imu::delay(100);
    while (BMI088_init()) {
      imu::delay(10);
    }
    BMI088_read(raw_data.gyro, raw_data.accel, raw_data.temp);
  }
  if (boardimu::ist8310_enabled) {
    while (ist8310_init()) {
      imu::delay(10);
    }
    ist8310_read_mag(raw_data.mag);
  }
  board_imu.init(EulerAngle_t(0, 0, 0), true);
#elif defined DBA
  if (boardimu::mpu6500_enabled) {
    mpu_device_init();
  }
  board_imu.init(EulerAngle_t(0, 0, 0), false, 500);
#endif
  boardimu::temp.init();
}

void boardimu::readSensor(imu::RawData_t& raw_data) {
#ifdef DBC
  if (boardimu::bmi088_enabled) {
    BMI088_read(raw_data.gyro, raw_data.accel, raw_data.temp);
  }
  if (boardimu::ist8310_enabled) {
    ist8310_read_mag(raw_data.mag);
  }
#elif defined DBA
  if (boardimu::mpu6500_enabled) {
    mpu_get_data(raw_data.gyro, raw_data.accel, raw_data.mag, raw_data.temp);
  }
#endif
}

void imu::TempControl::init(void) {
  HAL_TIM_Base_Start(htim_);
  HAL_TIM_PWM_Start(htim_, tim_channel_);
}

void imu::TempControl::handle(float temp_ref, float temp_fdb) {
  pwm_ = (uint16_t)math::limitMin(pid_.calc(temp_ref, temp_fdb), 0);
  __HAL_TIM_SetCompare(htim_, tim_channel_, pwm_);
}

void imu::initAll(void) {
  boardimu::initSensor(board_imu.raw_data_);
}

void imu::handleAll(void) {
  board_imu.update();
  boardimu::temp.handle(40, board_imu.raw_data_.temp[0]);
}
