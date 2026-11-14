#pragma once
#include "arm_math.h"
inline void* GPIOA=nullptr;
inline void* GPIOB=nullptr;
inline constexpr uint16_t GPIO_PIN_0=1,GPIO_PIN_4=16;
uint32_t HAL_GetTick();
