/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_PowerLimitator.hpp
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     ${MONTH}-${DAY}-${YEAR}     Light            1. done
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
#include "main.h"
#include "cmsis_os.h"
#include "HDL_PowerLimitator.hpp"
#include "HDL_SuperCap.hpp"
/**
 * @brief 宏定义
 */

/**
 * @brief 结构体
 */

/**
 * @brief 变量外部声明
 */

/**
 * @brief 功率控制任务句柄
 */
extern osThreadId_t PowerLimitatorHandle;

/**
 * @brief 功率控制任务属性配置
 */
extern const osThreadAttr_t PowerLimitator_attributes;

/**
 * @brief CPP部分
 */
#ifdef __cplusplus
/* 轮腿功率控制器类 */
class WheelLeggedPowerLimitator {
public:
 enum MODE : uint8_t {
  BATTERY = 0,
  CAP
};

 // 初始化参数
 void Init();

 // 更新能量环和最大允许功率 (放置在 FreeRTOS 任务中定期调用)
 void Update_Power_Loop(float referee_limit, float cap_energy, bool is_cap_online);

 // 核心算法：输入 LQR 的各个控制分量，计算出衰减后的 Uspeed 和 Uyaw
 void Calculate_Decay(float Uspeed, float Uyaw,
                      float leftUelse, float rightUelse,
                      float wLeftWheel, float wRightWheel);

 float getDecayUspeed() const { return decayUspeed; }
 float getDecayUyaw() const { return decayUyaw; }
 float getConfiguredMaxPower() const { return configuredMaxPower; }

 void setMode(MODE m) { mode = m; }

private:
 // 电机拟合参数 (M3508)
 const float k0 = 0.2463f;     // 机械功率系数
 const float k1 = 0.168286f;   // 反电动势/转速线性损耗
 const float k2 = 2.641787f;   // 焦耳发热损耗
 const float k3 = 0.625f;      // 常态静态损耗

 float configuredMaxPower; // 能量环算出的当前周期最大允许功率

 // 能量环 PID
 PidTypeDef_t energy_pid;

 float decayUspeed = 1.0f;
 float decayUyaw = 1.0f;

 MODE mode = BATTERY;
};

/**
 * @brief 功率控制的底盘全局数据变量
 */
extern WheelLeggedPowerLimitator WL_PowerManager;

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
************************************************************************
* @brief:      	PowerLimitator_Task: 功率控制任务
* @param[in]:   pvParameters: 任务参数(未使用)
* @retval:     	void
* @details:    	FreeRTOS任务，周期性执行底盘功率控制逻辑
************************************************************************
**/
extern void PowerLimitator_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif