/*
 * dwt.c
 *
 */

#include "dwt.h"

uint32_t DWT_Init(void)
{
	uint32_t c;

    /* Enable TRC */
	CoreDebug->DEMCR &= ~0x01000000;
	CoreDebug->DEMCR |=  0x01000000;

	/* Enable counter */
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |=  0x00000001;

	/* Reset counter */
	DWT->CYCCNT = 0;

	/* Check if DWT has started */
	c = DWT->CYCCNT;

	/* 2 dummys */
	__ASM volatile ("NOP");
	__ASM volatile ("NOP");

	/* Return difference, if result is zero, DWT has not started */
	return (DWT->CYCCNT - c);
}
