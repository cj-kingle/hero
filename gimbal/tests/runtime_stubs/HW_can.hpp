#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };
struct CAN_HandleTypeDef { unsigned index; };
struct CAN_RxHeaderTypeDef { uint32_t StdId=0, IDE=0, RTR=0, DLC=8; };
inline constexpr uint32_t CAN_ID_STD=0, CAN_RTR_DATA=0, CAN_TX_MAILBOX0=1,
    CAN_TX_MAILBOX1=2, CAN_TX_MAILBOX2=4, CAN_IT_BUSOFF=1, CAN_IT_ERROR_PASSIVE=2,
    CAN_IT_ERROR_WARNING=4, CAN_IT_ERROR=8, CAN_IT_RX_FIFO0_MSG_PENDING=16,
    CAN_IT_RX_FIFO1_MSG_PENDING=32, HAL_CAN_ERROR_NONE=0;
extern CAN_HandleTypeDef hcan1,hcan2;
inline constexpr uint32_t HAL_CAN_ERROR_EWG=0x1u,HAL_CAN_ERROR_EPV=0x2u,
    HAL_CAN_ERROR_BOF=0x4u,HAL_CAN_ERROR_ACK=0x20u,HAL_CAN_ERROR_TX_ALST0=0x800u,
    HAL_CAN_ERROR_TIMEOUT=0x20000u,HAL_CAN_ERROR_NOT_INITIALIZED=0x40000u,
    HAL_CAN_ERROR_NOT_READY=0x80000u,HAL_CAN_ERROR_NOT_STARTED=0x100000u,
    HAL_CAN_ERROR_PARAM=0x200000u,HAL_CAN_ERROR_INTERNAL=0x800000u;
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
uint32_t HAL_GetTick();
void CanFilter_Init(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef*,uint32_t);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef*);
HAL_StatusTypeDef CAN_TrySend(CAN_HandleTypeDef*,const uint8_t*,uint32_t,uint8_t);
void CAN_Send_Msg(CAN_HandleTypeDef*,uint8_t*,uint32_t,uint8_t);
