/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       APL_USART.cpp
  * @brief      串口回调函数
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-29-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "APL_UASRT.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "HDL_VT.h"
#include "HAL_USART.h"
#include "Dev_Remote_Control.h"

/**
 * @brief          不定长接收中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart->Instance == USART1)
    {
       //图传链路
        VT_Module.VT_Data_Processing(UART1_Manage_Object.rx_buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.rx_buffer, UART1_Manage_Object.rx_data_size);
    }
    else if(huart->Instance == UART5)
    {
        USER_USART5_IRQHandler(huart ,Size);
    }
    else if(huart->Instance == USART6)
    {

    }
}


/**
 * @brief          错误处理中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(UART1_Manage_Object.huart, UART1_Manage_Object.rx_buffer, CUSTOM_QUEUE_ITEM_SIZE);

    }
    else if (huart->Instance == USART3)
    {

    }
}