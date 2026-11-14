/**
 *******************************************************************************
 * @file      :HW_can.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"
#include "stdint.h"

#include "gimbal_runtime.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t can1_rx_data[8], can2_rx_data[8];
uint32_t pTxMailbox;

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef canfilter = {};

  canfilter.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilter.FilterScale = CAN_FILTERSCALE_16BIT;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;
  if (hcan == &hcan1) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterBank = 0;
    canfilter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  } else if (hcan == &hcan2) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;
    canfilter.FilterActivation = ENABLE;
    canfilter.FilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK) {
      Error_Handler();
    }
  }
}

uint32_t can_rec_times = 0;
uint32_t can_success_times = 0;
uint32_t can_receive_data = 0;

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan != &hcan1) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0u) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) != HAL_OK) {
      GimbalOnCanError(hcan,true);
      break;
    }
    GimbalOnCanRx(hcan, rx_header1, can1_rx_data);
  }
}

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan != &hcan2) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) != 0u) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) != HAL_OK) {
      GimbalOnCanError(hcan,true);
      break;
    }
    GimbalOnCanRx(hcan, rx_header2, can2_rx_data);
  }
}

/**
 * @brief   向can总线发送数据，抄官方的
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文
 * @param	len为发送数据长度（字节数）
 * @retval  none
 * @note    主控发送都是len=8字节，再加上帧间隔3位，理论上can总线1ms最多传输9帧
 **/
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len) {
  CAN_TrySend(hcan, msg, id, len);
}

HAL_StatusTypeDef CAN_TrySend(CAN_HandleTypeDef *hcan, const uint8_t *msg,
                             uint32_t id, uint8_t len) {
  if (!hcan || !msg || id > 0x7ff || len > 8) return HAL_ERROR;
  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return HAL_BUSY;
  CAN_TxHeaderTypeDef header = {};
  header.StdId = id; header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA; header.DLC = len;
  uint32_t mailbox = 0;
  uint8_t data[8]{};
  for (uint8_t i=0; i<len; ++i) data[i] = msg[i];
  return HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox);
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  GimbalOnCanError(hcan);
}
