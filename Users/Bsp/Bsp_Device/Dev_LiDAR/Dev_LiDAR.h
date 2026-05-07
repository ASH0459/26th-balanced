/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Dev_LiDAR.h
  * @brief      STP-23 / STP-23L 激光雷达设备驱动
  * @note       Device Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-05-2026                      1. done
  *  V2.0.0     May-07-2026                      2. 新增STP-23L支持，双协议切换
  *
  @verbatim
  ==============================================================================
  * 修改 LIDAR_TYPE_SELECT 宏即可切换雷达型号：
  *   LIDAR_TYPE_STP23  — STP-23  (0x54头, 47B/帧, CRC8, 921600)
  *   LIDAR_TYPE_STP23L — STP-23L (0xAA头, 195B/帧, 累加和, 115200)
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

/*============================================================================
 *  雷达型号选择（修改此宏切换 STP-23 / STP-23L）
 *============================================================================*/
#define LIDAR_TYPE_STP23    0
#define LIDAR_TYPE_STP23L   1

#define LIDAR_TYPE_SELECT LIDAR_TYPE_STP23L /* <-- 在这里切换 */

/*============================================================================
 *  STP-23 协议定义
 *============================================================================*/
#if (LIDAR_TYPE_SELECT == LIDAR_TYPE_STP23)

#define LIDAR_HEADER        0x54
#define LIDAR_VERLEN        0x2C
#define LIDAR_POINT_NUM     12
#define LIDAR_FRAME_LEN     47
#define LIDAR_RX_BUF_SIZE   128
#define LIDAR_BAUDRATE      921600

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

/*============================================================================
 *  STP-23L 协议定义
 *============================================================================*/
#elif (LIDAR_TYPE_SELECT == LIDAR_TYPE_STP23L)

#define LIDAR23L_HEADER         0xAA
#define LIDAR23L_HEADER_LEN     4
#define LIDAR23L_POINT_NUM      12
#define LIDAR23L_POINT_SIZE     15      /* 每个测量点15字节 */
#define LIDAR23L_DATA_LEN       (LIDAR23L_POINT_NUM * LIDAR23L_POINT_SIZE)  /* 180 */
#define LIDAR23L_FRAME_LEN      (10 + LIDAR23L_DATA_LEN + 4 + 1)           /* 195 */
#define LIDAR_RX_BUF_SIZE       512
#define LIDAR_BAUDRATE          230400

/* STP-23L 命令字 */
#define PACK_GET_DISTANCE       0x02
#define PACK_RESET_SYSTEM       0x0D
#define PACK_STOP               0x0F
#define PACK_ACK                0x10
#define PACK_VERSION            0x14

typedef struct {
    uint16_t distance;
    uint16_t noise;
    uint32_t peak;
    uint8_t  confidence;
    uint32_t intg;
    int16_t  reftof;
} LiDAR23LPointTypeDef;

typedef struct {
    uint8_t  header[4];                         /* 4×0xAA */
    uint8_t  device_addr;
    uint8_t  command;
    uint16_t chunk_offset;
    uint16_t data_len;
    LiDAR23LPointTypeDef point[LIDAR23L_POINT_NUM];
    uint32_t timestamp;
    uint8_t  crc;
} LiDAR23LFrameTypeDef;

#else
#error "LIDAR_TYPE_SELECT must be LIDAR_TYPE_STP23 or LIDAR_TYPE_STP23L"
#endif

/*============================================================================
 *  公共接口
 *============================================================================*/

/**
  * @brief  激光雷达初始化（开启DMA不定长接收）
  */
extern void LiDAR_Init(void);

/**
  * @brief  获取激光雷达距离值 (mm)
  * @retval 距离值，无效时返回 0
  */
extern uint16_t LiDAR_Get_Distance(void);

/**
  * @brief  获取激光雷达接收帧计数
  */
extern uint16_t LiDAR_Get_Receive_Count(void);

/**
  * @brief  串口错误处理（在 HAL_UART_ErrorCallback 中调用）
  */
extern void UART1_Error_Handler(void);

/**
  * @brief  串口1激光雷达中断服务函数（在 HAL_UARTEx_RxEventCallback 中调用）
  */
extern void USART1_LiDAR_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif
