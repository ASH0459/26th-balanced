/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  *             详细编写较为考验对串口寄存器和DMA的掌握，建议查看安富莱V7和STM32手册
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *  V1.1.0     Sep-27-2024     Light
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Dev_Remote_Control.h"

#include "main.h"
#include "App_Detect_Task.h"
#include "APL_RC_Hub.h"

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;


//取正函数
static int16_t RC_abs(int16_t value);

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, dr16_control_t *rc_ctrl);


//遥控器控制变量
dr16_control_t rc_ctrl;

// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
//static __attribute__((section(".ram_d2_data"))) uint8_t sbus_rx_buf[2][RC_FRAME_LENGTH];
static uint8_t sbus_rx_buf[2][RC_FRAME_LENGTH];

/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void Remote_Control_Init(void)
{
    RC_Init(&huart5, sbus_rx_buf[0], sbus_rx_buf[1], RC_FRAME_LENGTH);
}


/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const dr16_control_t *Get_Remote_Control_Point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_Data_Is_Error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[4]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

    error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void Slove_RC_Lost(void)
{
    RC_Restart(SBUS_RX_BUF_NUM);
}
void Slove_Data_Error(void)
{
    RC_Restart(SBUS_RX_BUF_NUM);
}

/** @note   1.判断接收数据来源，此处使用DMA+串口完成不定长数据的接收，因此只处理IDLE空闲总线数据，判断完成之后必须手动清除挂起标志位
  *         2.判断当前DMA缓冲区是0还是1
  *         3.失能DMA禁止数据流（使能时为NDTR只读）之后，读取NDTR（Number of Data Register）寄存器获取接收数据长度
  *         4.计算数据长度之后，重新设定NDTR接收数据长度与DMA目标缓冲区（双缓冲区交替），使能DMA进行下一次数据接收
  *         5.判断数据长度是否为18字节（DT7与DR16数据帧长度），满足则进行数据解析
  *
  * 可优化方向: 本赛季对于全向轮优化需求不大，同时DT7发送频率有限制 但优化方向可部署在类似的外设上 如：CAN、FDCAN、SPI等
  *         1.使用DMA的FIFO（F4系列的串口无FIFO）
  *         2.利用RTOS的消息队列将数据解析任务与中断剥离
  * */
void USER_USART5_IRQHandler(UART_HandleTypeDef *huart ,uint16_t Size)
{
    /* 接收数据非空 */
    // if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) // Read Data Register Not Empty
    // {
    //     __HAL_UART_CLEAR_PEFLAG(huart); // 清除中断标志位
    // }
    // /* 总线空闲 */
    // else if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // IDLE line detected
    // {
        static uint16_t this_time_rx_len = 0;

        //__HAL_UART_CLEAR_PEFLAG(huart);

        /* Current memory buffer used is Memory 0 */
        if ((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            // 失效DMA
            __HAL_DMA_DISABLE(huart->hdmarx);

            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR;

            // 重新设定数据长度
            //((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = SBUS_RX_BUF_NUM;

            // 设定缓冲区1
            ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

            __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

            if(Size == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

                // 记录数据接收时间
                detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[0]);
            }
            // 使能DMA
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
            // 失效DMA
            __HAL_DMA_DISABLE(huart->hdmarx);

            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            //this_time_rx_len = SBUS_RX_BUF_NUM - ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR;

            // 重新设定数据长度
           //((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = SBUS_RX_BUF_NUM;

            // 设定缓冲区0
            //DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

            __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

            if(Size == RC_FRAME_LENGTH)
            {
                // 处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

                // 记录数据接收时间
                detect_hook(DBUS_TOE);
                //sbus_to_usart1(sbus_rx_buf[1]);
            }
            // 使能DMA
            __HAL_DMA_ENABLE(huart->hdmarx);
        }
    __HAL_DMA_ENABLE(huart->hdmarx);
    // }

}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, dr16_control_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->key.last_v = rc_ctrl->key.v;

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  			//!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

///**
//  * @brief          send sbus data by usart1, called in usart5_IRQHandle
//  * @param[in]      sbus: sbus data, 18 bytes
//  * @retval         none
//  */
///**
//  * @brief          通过usart1发送sbus数据,在usart5_IRQHandle调用
//  * @param[in]      sbus: sbus数据, 18字节
//  * @retval         none
//  */
//void Sbus_to_Usart1(uint8_t *sbus)
//{
//    static uint8_t usart_tx_buf[20];
//    static uint8_t i =0;
//    usart_tx_buf[0] = 0xA6;
//    memcpy(usart_tx_buf + 1, sbus, 18);
//    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
//    {
//        usart_tx_buf[19] += usart_tx_buf[i];
//    }
//    usart1_tx_dma_enable(usart_tx_buf, 20);
//}

