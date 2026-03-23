/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_SuperCap.hpp
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
#pragma once

/**
 * @brief 头文件
 */
#include "cmsis_os.h"
#include "HDL_SuperCap.hpp"


/**
 * @brief 宏定义
 */
#define SuperCap_Task_Time 100

/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */

/**
 * @brief 超级电容任务句柄
 */
extern osThreadId_t SuperCap_TaskHandle;

/**
 * @brief 超级电容任务属性配置
 */
extern const osThreadAttr_t SuperCap_Task_attributes;

/**
 * @brief CPP部分
 */
#ifdef __cplusplus

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
************************************************************************
* @brief:      	SuperCap_Task: 超级电容通信任务
* @param[in]:   pvParameters: 任务参数(未使用)
* @retval:     	void
* @details:    	FreeRTOS任务，周期性向超级电容主控发送控制数据
************************************************************************
**/
extern void SuperCap_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif