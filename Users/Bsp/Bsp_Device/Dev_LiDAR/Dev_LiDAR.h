/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Dev_LiDAR.h
  * @brief      STP-23激光雷达设备驱动
  * @note       Device Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-05-2026                      1. done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#pragma once

#include "main.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 激光雷达DMA接收缓冲区大小 (>= LIDAR_FRAME_LEN * 2，确保能接收完整帧) */
#define LIDAR_RX_BUF_SIZE   128

/* 激光雷达相关变量外部声明 */
extern LiDARFrameTypeDef LiDAR_Data;        // 雷达帧数据
extern uint16_t lidar_distance;             // 雷达计算后的距离值 (mm)
extern uint16_t lidar_receive_cnt;          // 雷达接收正确帧计数

/**
  * @brief  激光雷达初始化（开启DMA不定长接收）
  */
extern void LiDAR_Init(void);

extern void UART1_Error_Handler(void);

/**
  * @brief  串口1激光雷达中断服务函数（在DMA不定长回调中调用）
  * @param  huart: 串口句柄
  * @param  Size:  本次接收到的数据长度
  */
extern void USART1_LiDAR_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif
