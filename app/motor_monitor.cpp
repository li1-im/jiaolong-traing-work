/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/motor_monitor.h"
#include "lib/arm_math/arm_math.h"
#include<cmath>






const Motor::KFParam_t joint_kf_param(5.f, 6e5f, 1.f);

constexpr float kJ0Ratio = 0.826f;
constexpr float kJ1Ratio = 1.92f;

volatile float g_J0_kp_pp = 40.f, g_J0_ki_pp = 0.02f, g_J0_kd_pp = 60.f;
volatile float g_J0_imax_pp = 300.f, g_J0_outmax_pp = 1440.f,
               g_J0_dfilt_pp = 0.5f;
volatile float g_J0_kp_sp = 35.f, g_J0_ki_sp = 0.f, g_J0_kd_sp = 0.f;
volatile float g_J0_imax_sp = 60000.f, g_J0_outmax_sp = 4914.f,
               g_J0_dfilt_sp = 1.0f;
volatile float g_J0_ratio = kJ0Ratio;
volatile float g_J1_kp_pp = 40.f, g_J1_ki_pp = 0.f, g_J1_kd_pp = 100.f;
volatile float g_J1_imax_pp = 0.f, g_J1_outmax_pp = 1440.f,
               g_J1_dfilt_pp = 1.0f;
volatile float g_J1_kp_sp = 10.f, g_J1_ki_sp = 0.f, g_J1_kd_sp = 40.f;
volatile float g_J1_imax_sp = 60000.f, g_J1_outmax_sp = 4914.f,
               g_J1_dfilt_sp = 1.0f;
volatile float g_J1_ratio = kJ1Ratio;


// 每毫秒把上面的可调参数写进电机里，否则要重烧太麻烦了
static void motorsPidRefresh(void) {
  J0.ppid_.kp_ = g_J0_kp_pp;  J0.ppid_.ki_ = g_J0_ki_pp;
  J0.ppid_.kd_ = g_J0_kd_pp;  J0.ppid_.i_max_ = g_J0_imax_pp;
  J0.ppid_.out_max_ = g_J0_outmax_pp;
  J0.ppid_.d_filter_.setK(g_J0_dfilt_pp);
  J0.spid_.kp_ = g_J0_kp_sp;  J0.spid_.ki_ = g_J0_ki_sp;
  J0.spid_.kd_ = g_J0_kd_sp;  J0.spid_.i_max_ = g_J0_imax_sp;
  J0.spid_.out_max_ = g_J0_outmax_sp;
  J0.spid_.d_filter_.setK(g_J0_dfilt_sp);
  J0.ratio_ = g_J0_ratio;
  J1.ppid_.kp_ = g_J1_kp_pp;  J1.ppid_.ki_ = g_J1_ki_pp;
  J1.ppid_.kd_ = g_J1_kd_pp;  J1.ppid_.i_max_ = g_J1_imax_pp;
  J1.ppid_.out_max_ = g_J1_outmax_pp;
  J1.ppid_.d_filter_.setK(g_J1_dfilt_pp);
  J1.spid_.kp_ = g_J1_kp_sp;  J1.spid_.ki_ = g_J1_ki_sp;
  J1.spid_.kd_ = g_J1_kd_sp;  J1.spid_.i_max_ = g_J1_imax_sp;
  J1.spid_.out_max_ = g_J1_outmax_sp;
  J1.spid_.d_filter_.setK(g_J1_dfilt_sp);
  J1.ratio_ = g_J1_ratio;
}

const PID J0_spid(30,0.01,0,0,16384,1);
const PID J0_ppid(75,0,400,0,1440,1);
Motor J0(Motor::M3508, kJ0Ratio, Motor::POSITION_SPEED,
         J0_ppid,
         J0_spid,
         true, joint_kf_param);

const PID J1_spid(12,0,50,0,16384,1);
const PID J1_ppid( 50,0,100,0,1440,1);
Motor J1(Motor::M3508, kJ1Ratio, Motor::POSITION_SPEED,
         J1_ppid,
         J1_spid,
         true, joint_kf_param);

Motor* can1_dji_motor[11] = {
    nullptr,
    nullptr,
    nullptr,
    &J0,
    nullptr,
    nullptr,
    nullptr,
    &J1,
    nullptr,
    nullptr,
    nullptr
};
Motor* can2_dji_motor[11] = {
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

MITMotorDriver mit_motor_driver[] = {};

const float motor_stop_rotate_speed_thres = 1200;

void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::POWEROFF;
  }
}

void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::STOP;
  }
}

void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->mode_ = Motor::WORKING;
  }
}

void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
  for (auto & i : mit_motor_driver) {
    i.motor_->reset();
    i.setCmd(mitmotor::MOTOR_MODE);
  }
  J0.setFdbSrc(&J0.kfAngle(), &J0.kfSpeed());
  J1.setFdbSrc(&J1.kfAngle(), &J1.kfSpeed());
}

void allMotorsHandle(void) {
  motorsPidRefresh();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
  for (auto & i : mit_motor_driver) {
    i.handle();
  }
}

void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
  for (auto & i : mit_motor_driver) {
    if (i.canRxMsgCheck(hcan, rx_header)) {
      i.canRxMsgCallback(hcan, rx_header, rx_data);
    }
  }
}
