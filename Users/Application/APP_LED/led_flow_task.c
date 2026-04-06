#include "led_flow_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"

void led_RGB_flow_task(void * argument)
{
    uint8_t red = 1;
		uint8_t green = 1;
		uint8_t blue = 1;
	WS2812_Ctrl(red, green, blue);

    while(1)
    {
				WS2812_Ctrl(red, green, blue);
				green += 100 - red;
				blue += 100 - red - green;
				vTaskDelay(LED_DELAY_TIME);
				red++;
				green++;
				blue++;
    }
}
