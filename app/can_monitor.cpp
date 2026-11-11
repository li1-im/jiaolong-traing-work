/**
 ******************************************************************************
 * @file    can_monitor.cpp/h
 * @brief   CAN communication transmit manage. CAN通信发送管理
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/can_monitor.h"

#include "can.h"
#include "cmsis_os.h"

#include "app/motor_monitor.h"

extern DJIMotorDriver dji_motor_driver;
extern MITMotorDriver mit_motor_driver[];

void canFilterInit(void) {
  CAN_FilterTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.SlaveStartFilterBank = 14;

  filter.FilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan1, &filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void canTxMonitor(void) {
  dji_motor_driver.canTxMsg(1, djimotor::ID_1_4);
  dji_motor_driver.canTxMsg(1, djimotor::ID_5_8);


  osDelay(1);



}
