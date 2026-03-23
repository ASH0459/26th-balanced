/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_PowerLimitator.hpp
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0
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
#include "Alg_PID.h"
#include "Alg_UserLib.h"
/**
 * @brief 宏定义
 */
/* 功率控制任务执行间隔 */
#define PowerLimitator_Task_Time 10

/* 用于功率控制的电机数量 */
#define PowerLimitator_Motor_Number 1

/* RPM 转化 RAD 常数 */
#define RPM_TO_RAD 0.10471975511965977f

/* 功率控制能量环 PID 控制器参数设定 */
#define PowerLimitator_Energy_PID_Kp       0.0f
#define PowerLimitator_Energy_PID_Ki       0.0f
#define PowerLimitator_Energy_PID_Kd       0.0f
#define PowerLimitator_Energy_PID_Maxout   0.0f
#define PowerLimitator_Energy_PID_iMaxout  0.0f

/**
 * @brief 结构体
 */
// 连接状态
typedef enum {
 Connected,
 Disconnected,
} Connect_Status_t;

// 底盘功率控制状态
typedef enum {
 NOT_RESTRICTED,   // 底盘功率不受限
 RESTRICTED        // 底盘功率受限
} PowerControlStatus;

// 电机的状态数据
typedef struct
{

 float k0;                        // 电机系数
 float k1;
 float k2;
 float k3;
 float torqueConst;               // 电机力矩常数

 float currentPower;              // 电机实际的电流
 float getRadFeedback;            // 电机实际的转速
 float getTorqueFeedback;         // 电机实际输出力矩
 float pids_out;                  // 电机实际 PID 输出
 float cmdPower;
 float decayCurrent;              // 电机的衰减电流
 float finalcurrent;              // 最终的电机电流输出

 float chassisTargetSpeed;        // 底盘目标速度
 float chassisCurrentSpeed;       // 底盘当前速度
 float Uspeed;                    // 轮子速度扭矩
 float Uyaw;                      // 轮子转向扭矩
 float wLeftWheel;                // 左边轮子转速
 float leftUelse;                 // 左边轮子维持平衡扭矩
 float wRightWheel;               // 右边轮子转速
 float rightUelse;                // 右边轮子维持平衡扭矩

 float restrictedUspeed;          // 功率控制后的速度扭矩
 float restrictedUyaw;            // 功率控制后的方向扭矩
 float decayUspeed;               // 速度扭矩放缩比
 float decayUyaw;                 // 方向扭矩放缩比

 float K;                         // Uspeed = K * Uyaw
 float A;
 float B;
 float C;
 float Delta;
} Motor_PowerLimitator_t;

// 超级电容的状态数据
typedef struct
{

 float capacity;

} Cap_PowerLimitator_t;

// 裁判系统的状态数据
typedef struct
{

 float refereeMaxPower;           // 底盘功率测量值


} Referee_PowerLimitator_t;

// 功率模型的状态数据
typedef struct
{

 float measuredPower;             // 底盘功率测量值
 float maxPower;                  // 底盘最大总功率限制值
 float estimatedPower;            // 估计的总功率
 float commandPower;              // 命令总功率
 float powerControlStatus;        // 底盘功率控制状态

} PowerLimitator_t;



// 机器人的状态数据
typedef struct
{

 Connect_Status_t Motor_Status;   // 电机连接状态
 Connect_Status_t Cap_Status;     // 超级电容连接状态
 Connect_Status_t Referee_Status; // 裁判系统连接状态

} Robot_Status_t;



/**
 * @brief 变量外部声明
 */
extern Cap_PowerLimitator_t Cap_PowerLimitator;
extern Motor_PowerLimitator_t Motor_PowerLimitator[PowerLimitator_Motor_Number];

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
* @brief:      	PowerLimitator_Status_Update: 功率限制器状态更新
* @param[in]:   updatedMaxPower: 更新的最大功率限制值
* @retval:     	void
* @details:    	更新功率限制器的最大功率限制状态
************************************************************************
**/
extern float PowerLimitator_Status_Update(float chassisTargetSpeed,
                                          float chassisCurrentSpeed,
                                          float Uspeed,
                                          float Uyaw,
                                          float wLeftWheel,
                                          float leftUelse,
                                          float wRightWheel,
                                          float rightUelse);

/**
************************************************************************
* @brief:      	PowerLimitator_Motor_Update: 功率限制器电机更新
* @param[in]:   void
* @retval:     	void
* @details:    	更新所有受功率限制控制的电机状态
************************************************************************
**/
extern void PowerLimitator_Motor_Update(void);

/**
************************************************************************
* @brief:      	PowerLimitator_Init: 功率限制器初始化
* @param[in]:   void
* @retval:     	void
* @details:    	初始化功率限制器相关的参数和状态
************************************************************************
**/
extern void PowerLimitator_Init(void);

/**
************************************************************************
* @brief:      	rpm2rad: RPM转弧度/秒
* @param[in]:   rpm: 转速(转/分钟)
* @retval:     	float: 角速度(弧度/秒)
* @details:    	将转速从RPM转换为弧度/秒
************************************************************************
**/
extern float rpm2rad(float rpm);

/**
************************************************************************
* @brief:      	current2Torque: 电流转扭矩
* @param[in]:   motor_number: 电机编号
* @param[in]:   current: 电流值
* @retval:     	float: 扭矩值
* @details:    	根据电机编号和电流值计算对应的扭矩
************************************************************************
**/
extern float current2Torque(int motor_number, float current);

/**
************************************************************************
* @brief:      	PowerLimitator_Motor_Status_Init: 功率限制器电机状态初始化
* @param[in]:   Motor_Number: 电机编号
* @param[in]:   k0: 电机系数k0
* @param[in]:   k1: 电机系数k1
* @param[in]:   k2: 电机系数k2
* @param[in]:   k3: 电机系数k3
* @param[in]:   torqueConst: 电机力矩常数
* @retval:     	void
* @details:    	初始化指定电机的功率模型参数
************************************************************************
**/
extern void PowerLimitator_Motor_Status_Init(int Motor_Number,
                                             float k0,
                                             float k1,
                                             float k2,
                                             float k3,
                                             float torqueConst);

/**
************************************************************************
* @brief:      	PowerLimitator_Motor_Status_Update: 功率限制器电机状态更新
* @param[in]:   Motor_Number: 电机编号
* @param[in]:   getRPMFeedback: 反馈转速(RPM)
* @param[in]:   getTorqueFeedback: 反馈扭矩
* @param[in]:   pids_out: PID输出值
* @retval:     	void
* @details:    	更新指定电机的实时状态数据
************************************************************************
**/
extern void PowerLimitator_Motor_Status_Update(uint8_t Motor_Number,
                                         float getRPMFeedback,
                                         float getTorqueFeedback,
                                         float pids_out
                                         );

/**
************************************************************************
* @brief:      	PowerLimitator_getDecayCurrent: 获取衰减电流
* @param[in]:   void
* @retval:     	void
* @details:    	计算功率受限时的电机衰减电流
************************************************************************
**/
extern void PowerLimitator_getDecayCurrent(void);

/**
************************************************************************
* @brief:      	PowerLimitator_Energy_PID_Update: 能量环PID更新
* @param[in]:   void
* @retval:     	void
* @details:    	更新能量环PID控制器
************************************************************************
**/
extern void PowerLimitator_Energy_PID_Update(void);

/**
************************************************************************
* @brief:      	floatEqual: 浮点数相等判断
* @param[in]:   a: 浮点数a
* @param[in]:   b: 浮点数b
* @retval:     	bool: 是否相等
* @details:    	判断两个浮点数是否在误差范围内相等
************************************************************************
**/
extern bool_t floatEqual(float a, float b);

/**
************************************************************************
* @brief:      	PowerLimitator_Energy_Capacity_Set: 设置能量容量
* @param[in]:   capacity: 容量值
* @retval:     	void
* @details:    	设置超级电容的当前容量
************************************************************************
**/
extern void PowerLimitator_Energy_Capacity_Set(float capacity);


#ifdef __cplusplus
}
#endif