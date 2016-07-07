
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "ov7670.h"

uint8_t OV7670_Debug_ReadReg(OV7670_HandleTypeDef *hov, uint8_t reg_addr)
{
	uint32_t i2c2_cr1 = hov->hi2c->Instance->CR1;
	uint32_t i2c2_cr2 = hov->hi2c->Instance->CR2;
	uint32_t i2c2_oar1 = hov->hi2c->Instance->OAR1;
	uint32_t i2c2_oar2 = hov->hi2c->Instance->OAR2;
	uint32_t i2c2_ccr = hov->hi2c->Instance->CCR;
	uint32_t i2c2_sr1 = hov->hi2c->Instance->SR1;
	uint32_t i2c2_sr2 = hov->hi2c->Instance->SR2;
	uint32_t i2c2_trise = hov->hi2c->Instance->TRISE;

	uint8_t data = 0xCC;

	while (__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY));

	hov->hi2c->Instance->CR1 |= I2C_CR1_START;

	while (!__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_MSL)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_SB));

	hov->hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(hov->addr);

	while (!__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY)
	    || !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_MSL)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_ADDR)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_TXE)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_TRA));

	hov->hi2c->Instance->DR = reg_addr;

	while (!__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_MSL)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BTF)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_TXE)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_TRA));

	hov->hi2c->Instance->CR1 |= I2C_CR1_STOP;

	hov->hi2c->Instance->CR1 |= I2C_CR1_START;

	while (!__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_MSL)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_SB));

	hov->hi2c->Instance->DR = I2C_7BIT_ADD_READ(hov->addr);

	while (!__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_MSL)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_ADDR));

	hov->hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

	while (!__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_MSL)
		|| !__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_RXNE));

	data = hov->hi2c->Instance->DR;

	hov->hi2c->Instance->CR1 |= I2C_CR1_STOP;

	while (__HAL_I2C_GET_FLAG(hov->hi2c, I2C_FLAG_BUSY));

	hov->hi2c->Instance->CR1 |= I2C_CR1_ACK;

	return data;
}

int OV7670_WriteReg(OV7670_HandleTypeDef *hov, uint8_t regAddr, uint8_t *pData)
{
	if (HAL_I2C_Mem_Write(hov->hi2c, hov->addr, regAddr, I2C_MEMADD_SIZE_8BIT, pData, 1, hov->timeout) == HAL_OK)
	{
		return OV7670_OK;
	}
	else
	{
		return OV7670_ERROR;
	}
}

int OV7670_ReadReg(OV7670_HandleTypeDef *hov, uint8_t regAddr, uint8_t *pData)
{
	//*pData = OV7670_Debug_ReadReg(hov, regAddr);
	//return OV7670_ERROR;

	if (HAL_I2C_Mem_Read(hov->hi2c, hov->addr, regAddr, I2C_MEMADD_SIZE_8BIT, pData, 1, hov->timeout) == HAL_OK)
	{
		return OV7670_OK;
	}
	else
	{
		return OV7670_ERROR;
	}
}

int OV7670_Reset(OV7670_HandleTypeDef *hov)
{
	uint8_t data = COM7_RESET;
	if (OV7670_WriteReg(hov, REG_COM7, &data) != OV7670_OK)
	{
		return OV7670_ERROR;
	}
	HAL_Delay(100);
	return OV7670_OK;
}
