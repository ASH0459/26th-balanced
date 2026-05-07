/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart10;

/* USER CODE BEGIN Private defines */

/* STP-23 激光雷达协议定义 */
#define LIDAR_HEADER        0x54
#define LIDAR_VERLEN        0x2C
#define LIDAR_POINT_NUM     12
#define LIDAR_FRAME_LEN     47

typedef struct {
    uint16_t distance;
    uint8_t  intensity;
} LiDARPointTypeDef;

typedef struct {
    uint8_t  header;
    uint8_t  ver_len;
    uint16_t temperature;
    uint16_t start_angle;
    LiDARPointTypeDef point[LIDAR_POINT_NUM];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t  crc8;
} LiDARFrameTypeDef;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART10_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

