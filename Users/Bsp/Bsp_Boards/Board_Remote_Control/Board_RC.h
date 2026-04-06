#ifndef BSP_RC_H
#define BSP_RC_H

#include "main.h"

#ifdef __cplusplus
extern "C"{
#endif

extern void RC_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength);

extern void RC_Disable(void);

extern void RC_Restart(uint16_t dma_buf_num);

extern void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength);
#ifdef __cplusplus
}
#endif

#endif
