/*
 * usbd_camera.c
 *
 */

#include "usbd_camera.h"

void Camera_On();
void Camera_Off();
uint8_t *Camera_GetFrame(uint32_t *pFrameLength);
void Camera_FreeFrame(uint8_t *frame);
void OV7670_Init(void);
int OV7670_Config(void);

USBD_UVC_CameraTypeDef USBD_Camera =
{
	Camera_On,
	Camera_Off,
	Camera_GetFrame,
	Camera_FreeFrame
};

static DCMI_HandleTypeDef *camera_hdcmi;
static DMA_HandleTypeDef *camera_hdma;
static I2C_HandleTypeDef *camera_hi2c;
static OV7670_HandleTypeDef hov;
static GPIO_TypeDef *camera_error_led_port;
static uint16_t camera_error_led_pin;

static uint8_t cameraOn;
static uint8_t cameraBuffer1[160 * 120 * 2];
static uint8_t cameraBuffer2[160 * 120 * 2];
static uint8_t *cameraBuffers[2];
volatile uint8_t cameraBufferState[2];
static uint8_t currentCameraBuffer;
static int cameraFrameCount;

void Camera_SignalError()
{
	if (camera_error_led_port == NULL)
	{
		return;
	}
	for (int i = 0; i < 3; i++)
	{
		HAL_GPIO_WritePin(camera_error_led_port, camera_error_led_pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(camera_error_led_port, camera_error_led_pin, GPIO_PIN_RESET);
		HAL_Delay(100);
	}
	HAL_GPIO_WritePin(camera_error_led_port, camera_error_led_pin, GPIO_PIN_RESET);
}

void Camera_Init(I2C_HandleTypeDef *hi2c, DCMI_HandleTypeDef *hdcmi, DMA_HandleTypeDef *hdma_dcmi, GPIO_TypeDef *errorLedPort, uint16_t errorLedPin)
{
	camera_hi2c  = hi2c;
	camera_hdcmi = hdcmi;
	camera_hdma  = hdma_dcmi;
	camera_error_led_port = errorLedPort;
	camera_error_led_pin = errorLedPin;

	cameraBuffers[0] = cameraBuffer1;
	cameraBuffers[1] = cameraBuffer2;
	cameraBufferState[0] = CAMERA_BUFFER_STATE_FREE;
	cameraBufferState[1] = CAMERA_BUFFER_STATE_FREE;

	cameraOn = 0;
	OV7670_Init();
	int ov_config_result = OV7670_Config();
	if (ov_config_result != OV7670_OK)
	{
		Camera_SignalError();
	}
}

void Camera_Fill_Buffer(int bufferIndex)
{
	HAL_DCMI_StateTypeDef dcmi_state = camera_hdcmi->State;
	HAL_DMA_StateTypeDef dma_state = camera_hdma->State;

	if (camera_hdcmi->State != HAL_DCMI_STATE_READY)
	{
		return;
	}
	if (camera_hdma->State != HAL_DMA_STATE_READY && camera_hdma->State != HAL_DMA_STATE_READY_MEM0)
	{
		return;
	}
	if (cameraBufferState[bufferIndex] != CAMERA_BUFFER_STATE_FREE)
	{
		return;
	}

	cameraBufferState[bufferIndex] = CAMERA_BUFFER_STATE_LOCKED_BY_DMA;
	currentCameraBuffer = bufferIndex;
	HAL_DCMI_Start_DMA(camera_hdcmi, DCMI_MODE_SNAPSHOT, cameraBuffers[bufferIndex], 160 * 120 * 2 / 4);
}

void Camera_On()
{
	if (cameraOn == 1)
	{
		return;
	}

	cameraBufferState[0] = CAMERA_BUFFER_STATE_FREE;
	cameraBufferState[1] = CAMERA_BUFFER_STATE_FREE;

	cameraFrameCount = 0;
	cameraOn = 1;
}

void Camera_Off()
{
	cameraOn = 0;
}

void Camera_Loop()
{

}

uint8_t *Camera_GetFrame(uint32_t *pFrameLength)
{
	if (camera_hdcmi->State == HAL_DCMI_STATE_READY
		  && (camera_hdma->State == HAL_DMA_STATE_READY || camera_hdma->State == HAL_DMA_STATE_READY_MEM0))
	{
		for (int i = 0; i < 2; i++)
		{
			if (cameraBufferState[i] == CAMERA_BUFFER_STATE_LOCKED_BY_DMA)
			{
				cameraBufferState[i] = CAMERA_BUFFER_STATE_READY;
			}
		}
		for (int i = 0; i < 2; i++)
		{
			if (cameraBufferState[i] == CAMERA_BUFFER_STATE_FREE)
			{
				Camera_Fill_Buffer(i);
				break;
			}
		}
	}

	for (int i = 0; i < 2; i++)
	{
		if (cameraBufferState[i] == CAMERA_BUFFER_STATE_READY)
		{
			cameraBufferState[i] = CAMERA_BUFFER_STATE_LOCKED_BY_UVC;
			*pFrameLength = 160 * 120 * 2;
			return cameraBuffers[i];
		}
	}

	*pFrameLength = 0;
	return NULL;
}

void Camera_FreeFrame(uint8_t *frame)
{
	for (int i = 0; i < 2; i++)
	{
		if (cameraBuffers[i] == frame && cameraBufferState[i] == CAMERA_BUFFER_STATE_LOCKED_BY_UVC)
		{
			cameraBufferState[i] = CAMERA_BUFFER_STATE_FREE;
		}
	}
	cameraFrameCount++;
}

void OV7670_Init(void)
{
	hov.hi2c = camera_hi2c;
	hov.addr = OV7670_ADDRESS;
	hov.timeout = 100;
}

int OV7670_Config()
{
	int ov_reset_result = OV7670_Reset(&hov);
	if (ov_reset_result != OV7670_OK)
	{
		return ov_reset_result;
	}
	int ov_read_reg_result = OV7670_ERROR;
	int ov_write_reg_result = OV7670_ERROR;

	ov_write_reg_result = OV7670_WriteRegList(&hov, ov7670_default_regs);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	ov_write_reg_result = OV7670_WriteRegList(&hov, qqvga_ov7670);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	uint8_t ov_com3 = 4; // REG_COM3 enable scaling
	ov_write_reg_result = OV7670_WriteReg(&hov, REG_COM3, &ov_com3);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	ov_write_reg_result = OV7670_WriteRegList(&hov, yuv422_ov7670);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	// configure PCLK to 24MHz

	uint8_t ov_clk_rc = 0;
	ov_read_reg_result = OV7670_ReadReg(&hov, REG_CLKRC, &ov_clk_rc);
	if (ov_read_reg_result != OV7670_OK)
	{
		return ov_read_reg_result;
	}
	ov_clk_rc = (ov_clk_rc & 0x80) | 0x01; // to enable prescaler by 2
	ov_write_reg_result = OV7670_WriteReg(&hov, REG_CLKRC, &ov_clk_rc);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}

	uint8_t ov_dblv = 0;
	ov_read_reg_result = OV7670_ReadReg(&hov, REG_DBLV, &ov_dblv);
	if (ov_read_reg_result != OV7670_OK)
	{
		return ov_read_reg_result;
	}
	ov_dblv = (ov_dblv & 0x3F) | DBLV_PLL6; // to enable PLL x6
	ov_write_reg_result = OV7670_WriteReg(&hov, REG_DBLV, &ov_dblv);
	if (ov_write_reg_result != OV7670_OK)
	{
		return ov_write_reg_result;
	}
	HAL_Delay(100);

	return OV7670_OK;
}
