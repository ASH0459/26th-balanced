/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Dev_LiDAR.c
  * @brief      STP-23激光雷达设备驱动
  * @note       Device Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-05-2026                      1. done
  *
  @verbatim
  ==============================================================================
  * STP-23激光雷达协议：
  *   帧格式：Header(0x54) + VerLen(0x2C) + Temp(2B) + StartAngle(2B)
  *           + 12×(Distance(2B)+Intensity(1B)) + EndAngle(2B) + Timestamp(2B) + CRC8
  *   波特率：921600
  *   每帧47字节
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "Dev_LiDAR.h"
#include <string.h>

/* CRC8 查表，用于STP-23激光雷达数据校验 */
static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

/* 激光雷达全局变量 */
LiDARFrameTypeDef LiDAR_Data;              // 雷达帧数据
uint16_t lidar_distance = 0;               // 雷达计算后的距离值 (mm)
uint16_t lidar_receive_cnt = 0;            // 雷达接收正确帧计数
__attribute__((section(".ram_d2"), aligned(4))) uint8_t lidar_rx_buf[LIDAR_RX_BUF_SIZE];

/**
  * @brief  激光雷达初始化，开启串口1 DMA不定长接收
  *         本函数将会在FreeRTOS中的初始化环节被调用
  */
void LiDAR_Init(void)
{
    /* 串口不定长接收+DMA初始化 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, LIDAR_RX_BUF_SIZE);
}

/**
  * @brief  CRC8校验计算
  * @param  data: 数据指针
  * @param  len:  数据长度
  * @retval CRC8校验值
  */
static uint8_t lidar_crc8_calc(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        crc = CrcTable[(crc ^ data[i]) & 0xff];
    }
    return crc;
}

/**
  * @brief  激光雷达数据处理函数，每100帧计算一次平均距离
  */
static void lidar_data_process(void)
{
    static uint8_t frame_cnt = 0;
    static uint16_t valid_count = 0;
    static uint32_t sum = 0;

    for (uint8_t i = 0; i < LIDAR_POINT_NUM; i++)
    {
        if (LiDAR_Data.point[i].distance != 0) // 去除距离为0的无效点
        {
            valid_count++;
            sum += LiDAR_Data.point[i].distance;
        }
    }

    if (++frame_cnt >= 100) // 每100帧计算一次平均距离
    {
        if (valid_count > 0)
        {
            lidar_distance = sum / valid_count;
        }
        sum = 0;
        valid_count = 0;
        frame_cnt = 0;
    }
}

/**
  * @brief  从DMA接收缓冲区中解析激光雷达帧
  *         在缓冲区中搜索帧头，找到完整帧后进行CRC校验和数据处理
  * @param  data: DMA接收缓冲区指针
  * @param  len:  本次接收到的数据长度
  */
static void lidar_parse_buffer(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;

    while (i + LIDAR_FRAME_LEN <= len)
    {
        // 搜索帧头
        if (data[i] == LIDAR_HEADER && data[i + 1] == LIDAR_VERLEN)
        {
            // 校验CRC8（前46字节计算CRC，第47字节为CRC值）
            uint8_t crc = lidar_crc8_calc(&data[i], LIDAR_FRAME_LEN - 1);

            if (crc == data[i + LIDAR_FRAME_LEN - 1])
            {
                // CRC校验通过，解析帧数据
                uint8_t *p = &data[i];

                LiDAR_Data.header      = p[0];
                LiDAR_Data.ver_len     = p[1];
                LiDAR_Data.temperature = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
                LiDAR_Data.start_angle = (uint16_t)p[4] | ((uint16_t)p[5] << 8);

                for (uint8_t j = 0; j < LIDAR_POINT_NUM; j++)
                {
                    uint8_t offset = 6 + j * 3;
                    LiDAR_Data.point[j].distance  = (uint16_t)p[offset] | ((uint16_t)p[offset + 1] << 8);
                    LiDAR_Data.point[j].intensity  = p[offset + 2];
                }

                LiDAR_Data.end_angle  = (uint16_t)p[42] | ((uint16_t)p[43] << 8);
                LiDAR_Data.timestamp  = (uint16_t)p[44] | ((uint16_t)p[45] << 8);
                LiDAR_Data.crc8       = p[46];

                lidar_data_process();
                lidar_receive_cnt++;

                i += LIDAR_FRAME_LEN; // 跳过已解析的帧
            }
            else
            {
                i++; // CRC失败，后移一字节继续搜索
            }
        }
        else
        {
            i++; // 非帧头，后移一字节
        }
    }
}

/**
  * @brief  串口1激光雷达DMA不定长接收中断服务函数
  *         将在HAL_UARTEx_RxEventCallback中被调用，参考UART10裁判系统写法
  * @param  huart: 串口句柄
  * @param  Size:  本次接收到的数据长度
  */
void USART1_LiDAR_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        static uint16_t this_time_rx_len = 0;
        this_time_rx_len = LIDAR_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        if (Size <= LIDAR_RX_BUF_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, LIDAR_RX_BUF_SIZE);

            // 解析DMA缓冲区数据
            lidar_parse_buffer(lidar_rx_buf, this_time_rx_len);
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, LIDAR_RX_BUF_SIZE);
            memset(lidar_rx_buf, 0, LIDAR_RX_BUF_SIZE);
        }
    }
}


/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口1的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
void UART1_Error_Handler(void)
{
    __HAL_UNLOCK(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, sizeof(lidar_rx_buf));
    memset(lidar_rx_buf, 0, sizeof(lidar_rx_buf));
}
