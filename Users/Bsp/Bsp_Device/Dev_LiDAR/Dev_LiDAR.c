/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Dev_LiDAR.c
  * @brief      STP-23 / STP-23L 激光雷达设备驱动
  * @note       Device Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-05-2026                      1. done
  *  V2.0.0     May-07-2026                      2. 新增STP-23L支持，双协议切换
  *
  @verbatim
  ==============================================================================
  * 修改 Dev_LiDAR.h 中的 LIDAR_TYPE_SELECT 宏即可切换雷达型号
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "Dev_LiDAR.h"
#include <string.h>

/*============================================================================
 *  公共变量
 *============================================================================*/
uint16_t lidar_distance = 0;               // 雷达计算后的距离值 (mm)
uint16_t lidar_receive_cnt = 0;            // 雷达接收正确帧计数
__attribute__((section(".ram_d2"), aligned(4))) uint8_t lidar_rx_buf[LIDAR_RX_BUF_SIZE];

/*============================================================================
 *  STP-23 驱动
 *============================================================================*/
#if (LIDAR_TYPE_SELECT == LIDAR_TYPE_STP23)

LiDARFrameTypeDef LiDAR_Data;

/* CRC8 查表 */
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

static uint8_t lidar_crc8_calc(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc = CrcTable[(crc ^ data[i]) & 0xff];
    return crc;
}

static void lidar_data_process(void)
{
    uint16_t valid_count = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < LIDAR_POINT_NUM; i++)
    {
        if (LiDAR_Data.point[i].distance != 0)
        {
            valid_count++;
            sum += LiDAR_Data.point[i].distance;
        }
    }

    if (valid_count > 0)
        lidar_distance = sum / valid_count;
}

static void lidar_parse_buffer(uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    while (i + LIDAR_FRAME_LEN <= len)
    {
        if (data[i] == LIDAR_HEADER && data[i + 1] == LIDAR_VERLEN)
        {
            uint8_t crc = lidar_crc8_calc(&data[i], LIDAR_FRAME_LEN - 1);
            if (crc == data[i + LIDAR_FRAME_LEN - 1])
            {
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
                i += LIDAR_FRAME_LEN;
            }
            else
            {
                i++;
            }
        }
        else
        {
            i++;
        }
    }
}

/*============================================================================
 *  STP-23L 驱动
 *============================================================================*/
#elif (LIDAR_TYPE_SELECT == LIDAR_TYPE_STP23L)

LiDAR23LFrameTypeDef LiDAR23L_Data;

static uint8_t lidar23l_crc_calc(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
        crc += data[i];
    return crc;
}

static void lidar23l_data_process(void)
{
    uint16_t valid_count = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < LIDAR23L_POINT_NUM; i++)
    {
        if (LiDAR23L_Data.point[i].distance != 0)
        {
            valid_count++;
            sum += LiDAR23L_Data.point[i].distance;
        }
    }

    if (valid_count > 0)
        lidar_distance = sum / valid_count;
}

/**
  * @brief  在DMA缓冲区中查找连续4个0xAA的帧头位置
  * @param  data: 缓冲区指针
  * @param  len:  缓冲区长度
  * @retval 帧头起始索引，未找到返回 -1
  */
static int16_t lidar23l_find_header(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i + 3 < len; i++)
    {
        if (data[i] == 0xAA && data[i + 1] == 0xAA &&
            data[i + 2] == 0xAA && data[i + 3] == 0xAA)
        {
            return (int16_t)i;
        }
    }
    return -1;
}

/**
  * @brief  从DMA缓冲区中解析STP-23L帧
  *         帧格式：4×0xAA + DevAddr + Cmd + ChunkOff(2B) + DataLen(2B)
  *               + 12×(Dist(2B)+Noise(2B)+Peak(4B)+Conf(1B)+Intg(4B)+Reftof(2B))
  *               + Timestamp(4B) + CRC(1B)
  *         总计195字节，CRC为前面所有字节累加和
  */
static void lidar23l_parse_buffer(uint8_t *data, uint16_t len)
{
    uint16_t pos = 0;

    while (pos + LIDAR23L_FRAME_LEN <= len)
    {
        int16_t hdr = lidar23l_find_header(&data[pos], len - pos);
        if (hdr < 0)
            break;

        pos += (uint16_t)hdr;

        if (pos + LIDAR23L_FRAME_LEN > len)
            break;

        uint8_t *p = &data[pos];

        /* 校验：从device_addr到timestamp末尾（索引4~193）累加和 == 索引194 */
        uint8_t crc = lidar23l_crc_calc(&p[4], LIDAR23L_FRAME_LEN - 1 - 4);
        if (crc == p[LIDAR23L_FRAME_LEN - 1])
        {
            /* 解析帧头 */
            LiDAR23L_Data.header[0]    = p[0];
            LiDAR23L_Data.header[1]    = p[1];
            LiDAR23L_Data.header[2]    = p[2];
            LiDAR23L_Data.header[3]    = p[3];
            LiDAR23L_Data.device_addr  = p[4];
            LiDAR23L_Data.command      = p[5];
            LiDAR23L_Data.chunk_offset = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
            LiDAR23L_Data.data_len     = (uint16_t)p[8] | ((uint16_t)p[9] << 8);

            /* 解析12个测量点，每点15字节 */
            for (uint8_t i = 0; i < LIDAR23L_POINT_NUM; i++)
            {
                uint16_t offset = 10 + i * 15;
                LiDAR23L_Data.point[i].distance   = (uint16_t)p[offset]      | ((uint16_t)p[offset + 1] << 8);
                LiDAR23L_Data.point[i].noise       = (uint16_t)p[offset + 2] | ((uint16_t)p[offset + 3] << 8);
                LiDAR23L_Data.point[i].peak        = (uint32_t)p[offset + 4] | ((uint32_t)p[offset + 5] << 8)
                                                    | ((uint32_t)p[offset + 6] << 16) | ((uint32_t)p[offset + 7] << 24);
                LiDAR23L_Data.point[i].confidence  = p[offset + 8];
                LiDAR23L_Data.point[i].intg        = (uint32_t)p[offset + 9]  | ((uint32_t)p[offset + 10] << 8)
                                                    | ((uint32_t)p[offset + 11] << 16) | ((uint32_t)p[offset + 12] << 24);
                LiDAR23L_Data.point[i].reftof      = (int16_t)((uint16_t)p[offset + 13] | ((uint16_t)p[offset + 14] << 8));
            }

            /* 解析时间戳 */
            uint16_t ts_offset = 10 + LIDAR23L_DATA_LEN;  /* 190 */
            LiDAR23L_Data.timestamp = (uint32_t)p[ts_offset]     | ((uint32_t)p[ts_offset + 1] << 8)
                                    | ((uint32_t)p[ts_offset + 2] << 16) | ((uint32_t)p[ts_offset + 3] << 24);
            LiDAR23L_Data.crc = p[LIDAR23L_FRAME_LEN - 1];

            lidar23l_data_process();
            lidar_receive_cnt++;

            pos += LIDAR23L_FRAME_LEN;
        }
        else
        {
            /* CRC失败，跳过这个假帧头，从下一字节继续 */
            pos++;
        }
    }
}

#endif /* LIDAR_TYPE_SELECT */

/*============================================================================
 *  公共接口实现
 *============================================================================*/

void LiDAR_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, LIDAR_RX_BUF_SIZE);
}

uint16_t LiDAR_Get_Distance(void)
{
    return lidar_distance;
}

uint16_t LiDAR_Get_Receive_Count(void)
{
    return lidar_receive_cnt;
}

void USART1_LiDAR_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        uint16_t this_time_rx_len = LIDAR_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        if (Size <= LIDAR_RX_BUF_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, LIDAR_RX_BUF_SIZE);

#if (LIDAR_TYPE_SELECT == LIDAR_TYPE_STP23)
            lidar_parse_buffer(lidar_rx_buf, this_time_rx_len);
#elif (LIDAR_TYPE_SELECT == LIDAR_TYPE_STP23L)
            lidar23l_parse_buffer(lidar_rx_buf, this_time_rx_len);
#endif
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, LIDAR_RX_BUF_SIZE);
            memset(lidar_rx_buf, 0, LIDAR_RX_BUF_SIZE);
        }
    }
}

void UART1_Error_Handler(void)
{
    __HAL_UNLOCK(&huart1);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, lidar_rx_buf, sizeof(lidar_rx_buf));
    memset(lidar_rx_buf, 0, sizeof(lidar_rx_buf));
}
