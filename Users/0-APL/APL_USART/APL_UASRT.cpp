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
#include "HAL_USART.h"
#include "Dev_Referee.h"
#include "App_Detect_Task.h"
#include "Dev_LiDAR.h"

/**
 * @brief          不定长接收中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart == &huart10)
    {
        // 用作接收裁判系统常规链路
        UART10_ISR_Handler(huart, Size);
        detect_hook(Referee_TOE);
    }
    else if(huart->Instance == UART5)
    {
        USER_USART5_IRQHandler(huart ,Size);
    }
    else if(huart->Instance == USART1)
    {
        // 用作接收STP-23激光雷达数据
        USART1_LiDAR_ISR_Handler(huart, Size);
    }
}


/**
 * @brief          错误处理中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART10)
    {
        UART10_Error_Handler();
    }
    else if (huart->Instance == USART1) {
        UART1_Error_Handler();
    }
}

