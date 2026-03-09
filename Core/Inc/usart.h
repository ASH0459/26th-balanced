/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart10;

/* USER CODE BEGIN Private defines */

/* STP-23 激光雷达协议定义 */
#define LIDAR_HEADER        0x54    // 帧头
#define LIDAR_VERLEN        0x2C    // 固定版本+点数标识
#define LIDAR_POINT_NUM     12      // 每帧测量点数
#define LIDAR_FRAME_LEN     47      // 一帧总字节数

/* 激光雷达单点数据结构 */
typedef struct {
    uint16_t distance;      // 距离值 (mm)
    uint8_t  intensity;     // 置信度
} LiDARPointTypeDef;

/* 激光雷达一帧数据结构 */
typedef struct {
    uint8_t  header;                            // 帧头 0x54
    uint8_t  ver_len;                           // 版本+点数
    uint16_t temperature;                       // 温度 (ADC原始值)
    uint16_t start_angle;                       // 起始角度 (放大100倍)
    LiDARPointTypeDef point[LIDAR_POINT_NUM];   // 12个测量点
    uint16_t end_angle;                         // 结束角度 (放大100倍)
    uint16_t timestamp;                         // 时间戳
    uint8_t  crc8;                              // CRC校验
} LiDARFrameTypeDef;

/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART10_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

