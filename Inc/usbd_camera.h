/*
 * usbd_camera.h
 *
 */

#ifndef INC_USBD_CAMERA_H_
#define INC_USBD_CAMERA_H_

#include "stm32f4xx_hal.h"
#include "usbd_uvc.h"
#include "ov7670.h"

extern USBD_UVC_CameraTypeDef USBD_Camera;

void Camera_Init(I2C_HandleTypeDef *hi2c, DCMI_HandleTypeDef *hdcmi, DMA_HandleTypeDef *hdma_dcmi, GPIO_TypeDef *errorLedPort, uint16_t errorLedPin);
void Camera_Loop();

#define CAMERA_BUFFER_STATE_FREE 1
#define CAMERA_BUFFER_STATE_LOCKED_BY_DMA 2
#define CAMERA_BUFFER_STATE_READY 3
#define CAMERA_BUFFER_STATE_LOCKED_BY_UVC 4

#endif /* INC_USBD_CAMERA_H_ */
