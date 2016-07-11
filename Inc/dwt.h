/*
 * dwt.h
 *
 */

#ifndef __DWT_H_
#define __DWT_H_

#include "stm32f4xx.h"

extern uint32_t DWT_Init(void);

static inline uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}

static inline uint8_t DWT_Compare(int32_t tp)
{
  return (((int32_t)DWT_Get() - tp) < 0);
}

/**
  * @brief  microseconds
  */
static inline void DWT_Delay(uint32_t us)
{
  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}

#endif /* __DWT_H_ */
