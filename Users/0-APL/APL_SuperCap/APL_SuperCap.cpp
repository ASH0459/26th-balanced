/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_SuperCap.cpp
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-2-2       FenZhong        1. 建立超电发送线程
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "APL_SuperCap.hpp"
#include "UC_Referee.h"

/**
 * @brief 超级电容任务句柄
 */
osThreadId_t SuperCap_TaskHandle;

/**
 * @brief 超级电容任务属性配置
 */
const osThreadAttr_t SuperCap_Task_attributes = {
  .name = "SuperCap_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/**
************************************************************************
* @brief:      	SuperCap_Task: 超级电容通信任务
* @param[in]:   pvParameters: 任务参数(未使用)
* @retval:     	void
* @details:    	FreeRTOS任务，周期性向超级电容主控发送控制数据
************************************************************************
**/
void SuperCap_Task(void *pvParameters) {

  /* 初始化超电线程 */
  SuperCap_Init();

  /* 进入 [SuperCap_Task] 线程 */
  while (1) {

      /*编辑控制值(待改，输入裁判系统给的功率值)*/
      SuperCap_TX_Config(&SuperCap_Data,
                           1,
                           robot_state.chassis_power_limit,
                           power_heat_data_t.chassis_power_buffer,
                           1,
                           0,
                           0
                           );

    /* 发送数据给超电主控 */
    STM32_to_SuperCap();

    // 切出时间片，将 CPU 资源让给其他线程
    osDelay(SuperCap_Task_Time);
  }
}

