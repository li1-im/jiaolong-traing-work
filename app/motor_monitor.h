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

#ifndef MOTOR_MONITOR_H
#define MOTOR_MONITOR_H

#include "base/motor/driver/dji_motor_driver.h"
#include "base/motor/driver/mit_motor_driver.h"
#include "base/motor/motor.h"

void allMotorsStopShutoff(void);
void allMotorsShutOff(void);
void allMotorsStop(void);
void allMotorsOn(void);

void allMotorsInit(void);

void allMotorsHandle(void);

void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data);

extern Motor J0, J1;
extern Motor* can1_dji_motor[11];
extern Motor* can2_dji_motor[11];

#endif
