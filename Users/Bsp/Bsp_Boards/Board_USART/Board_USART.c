#include "Board_USART.h"
#include "App_Detect_Task.h"
#include "Dev_Referee.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

/**
 * @brief          不定长接收中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart10)
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
    if (huart->Instance == USART10)
    {
        UART10_Error_Handler();
    }
}

/**
 * @brief          裁判系统串口发送函数
 * @param[in]      none
 * @retval         none
 */

void Referee_Tx_Dma_Enable(uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
    {
        return;
    }

    // if (HAL_UART_GetState(&huart10) != HAL_UART_STATE_READY)
    // {
    //     HAL_UART_AbortTransmit(&huart10);
    // }

    HAL_UART_Transmit_DMA(&huart10, data, len);
}
