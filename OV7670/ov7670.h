/*
 * ov7670.h
 */

#ifndef OV7670_OV7670_H_
#define OV7670_OV7670_H_

#include "ov7670_regs.h"

#define OV7670_ADDRESS 0x42

#define OV7670_OK 0
#define OV7670_ERROR 1

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t addr;
	uint32_t timeout;
} OV7670_HandleTypeDef;

int OV7670_Reset(OV7670_HandleTypeDef *hov);
int OV7670_WriteReg(OV7670_HandleTypeDef *hov, uint8_t regAddr, const uint8_t *pData);
int OV7670_ReadReg(OV7670_HandleTypeDef *hov, uint8_t regAddr, uint8_t *pData);
int OV7670_WriteRegList(OV7670_HandleTypeDef *hov, const struct regval_t *reg_list);

#endif /* OV7670_OV7670_H_ */
