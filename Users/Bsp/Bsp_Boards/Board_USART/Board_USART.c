#include "Board_USART.h"
#include "App_Detect_Task.h"
#include "Dev_Referee.h"
#include "Dev_LiDAR.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

/**
 * @brief          不定长接收中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        // 激光雷达
        USART1_LiDAR_ISR_Handler(huart, Size);
    }
    else if (huart == &huart10)
    {
        // 用作接收裁判系统常规链路
        UART10_ISR_Handler(huart, Size);
        detect_hook(Referee_TOE);
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
        UART1_Error_Handler();
    }
    else if (huart->Instance == USART10)
    {
        UART10_Error_Handler();
    }
}
