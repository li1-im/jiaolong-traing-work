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

#ifndef IMU_MONITOR_H
#define IMU_MONITOR_H

#include <stdint.h>
#include "base/common/pid.h"
#include "base/imu/imu.h"
#include "hardware_config.h"
#include "tim.h"

namespace imu {

class TempControl {
 public:
  TempControl(TIM_HandleTypeDef* htim, uint32_t tim_channel, PID pid)
      : pid_(pid), pwm_(0), htim_(htim), tim_channel_(tim_channel) {}

  void init(void);
  void handle(float temp_ref, float temp_fdb);

 private:
  PID pid_;
  uint16_t pwm_;
  TIM_HandleTypeDef* htim_;
  uint32_t tim_channel_;
};

void initAll(void);

void handleAll(void);

}

extern IMU board_imu;

#endif
