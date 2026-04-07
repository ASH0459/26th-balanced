#include "App_UI_Task.h"
#include "ui.h"
#include "main.h"
#include "cmsis_os.h"
#include "App_Chassis_Task.h"
#include "ui_interface.h"
#include "UC_Referee.h"
#include "Dev_Can_Receive.h"
#include "HDL_SuperCap.hpp"
#include <string.h>
#include "Board_USART.h"



void UI_Task(void *argument)
{
  osDelay(1000);


  while (1)
  {
    osDelay(1000);
    // HAL_UART_Transmit_DMA(&huart10, (uint8_t *)"Hello, UI!", 11);
  }
}
