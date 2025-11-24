#include "Board_RC.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef  hdma_usart5_rx;

/**
  * @brief          片上串口DMA初始化 双缓冲区接收数据
  * @param[in]      rx1_buf         内存缓冲区1
  *                 rx2_buf         内存缓冲区2
  *                 dma_buf_num     数据长度
  *                 指定将DMA传输的数据存入rx1_buf与rx2_buf中，之后直接读取rx1_buf/rx2_buf即可
  * @retval         none
  */
void RC_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxEventType = HAL_UART_RXEVENT_IDLE;

    huart->RxXferSize    = DataLength;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

    SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(huart->hdmarx);


}

void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxEventType = HAL_UART_RXEVENT_IDLE;

    huart->RxXferSize    = DataLength;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&huart->Instance->RDR,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
}

/**
  * @brief          失能串口
  * @param[in]      none
  * @retval         none
  */
void RC_Disable(void)
{
    __HAL_UART_DISABLE(&huart5);
}

/**
  * @brief          重启串口，以保证热插拔的稳定性
  * @param[in]      none
  * @retval         none
  */
void RC_Restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_usart5_rx);
    ((DMA_Stream_TypeDef *)huart5.hdmarx->Instance)->NDTR = dma_buf_num;
    __HAL_DMA_ENABLE(&hdma_usart5_rx);
    __HAL_UART_ENABLE(&huart5);
}







