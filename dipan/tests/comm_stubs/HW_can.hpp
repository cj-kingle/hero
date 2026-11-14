#pragma once
#include <cstdint>
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY };
constexpr uint32_t ENABLE=1, DISABLE=0, CAN_ID_STD=0, CAN_RTR_DATA=0;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING=1, CAN_IT_RX_FIFO1_MSG_PENDING=2;
constexpr uint32_t DMA_CIRCULAR=1, HAL_UART_RXEVENT_HT=1;
constexpr uint32_t CAN_TX_MAILBOX0=1,CAN_TX_MAILBOX1=2,CAN_TX_MAILBOX2=4,HAL_CAN_ERROR_NONE=0;
constexpr uint32_t HAL_CAN_ERROR_EWG=1,HAL_CAN_ERROR_ACK=0x20,HAL_CAN_ERROR_BOF=4;
constexpr uint32_t HAL_CAN_ERROR_TIMEOUT=0x20000,HAL_CAN_ERROR_NOT_INITIALIZED=0x40000;
constexpr uint32_t HAL_CAN_ERROR_NOT_READY=0x80000,HAL_CAN_ERROR_NOT_STARTED=0x100000;
constexpr uint32_t HAL_CAN_ERROR_PARAM=0x200000,HAL_CAN_ERROR_INTERNAL=0x800000;
struct CAN_HandleTypeDef {
  struct { uint32_t AutoBusOff=0, AutoRetransmission=1; } Init;
};
struct CAN_RxHeaderTypeDef { uint32_t StdId=0, IDE=0, RTR=0, DLC=8; };
struct DMA_HandleTypeDef { struct { uint32_t Mode=DMA_CIRCULAR; } Init; };
struct UART_HandleTypeDef { DMA_HandleTypeDef* hdmarx=nullptr; };
extern CAN_HandleTypeDef hcan1, hcan2;
extern UART_HandleTypeDef huart3;
uint32_t HAL_GetTick();
inline uint32_t __get_PRIMASK() { return 0; }
inline void __disable_irq() {}
inline void __set_PRIMASK(uint32_t) {}
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*, uint32_t);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef*);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t);
void CanFilter_Init(CAN_HandleTypeDef*);
HAL_StatusTypeDef CAN_TrySend(CAN_HandleTypeDef*,const uint8_t*,uint32_t,uint8_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*,uint8_t*,uint16_t);
uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*);
