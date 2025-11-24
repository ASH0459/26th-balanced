#include "bsp_led.h"
#include "main.h"

#define WS2812_LowLevel    0xC0
#define WS2812_HighLevel   0xF0

extern SPI_HandleTypeDef hspi6;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&hspi6, &res, 0, 0xFFFF);
    while (hspi6.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&hspi6, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&hspi6, &res, 1, 0xFFFF);
    }
}
