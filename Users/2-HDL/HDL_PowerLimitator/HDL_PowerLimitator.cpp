/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_PowerLimitator.cpp
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

/**
 * @brief 头文件
 */
#include "HDL_PowerLimitator.hpp"
#include "App_Chassis_Task.h"
#include "Dev_Can_Receive.h"
/* 用于对电机扭矩做低通滤波 */
first_order_filter_type_t PowerLimitator_LowPassFilter[PowerLimitator_Motor_Number];

// 定义滤波参数数组
float power_filter_param[1] = {0.0f};

/* 用于功率控制的的电机数据全局变量 */
Motor_PowerLimitator_t Motor_PowerLimitator[PowerLimitator_Motor_Number];

/* 用于功率控制的的超级电容数据全局变量 */
Cap_PowerLimitator_t Cap_PowerLimitator;

/* 用于功率控制的的裁判系统数据全局变量 */
Referee_PowerLimitator_t Referee_PowerLimitator;

/* 功率控制的底盘全局数据变量 */
PowerLimitator_t PowerLimitator_Chassis;

/* 能量环 PID 控制器 */
PidTypeDef_t PowerLimitator_Energy_PID;
// 输入：     实际给的最大功率的开平方（这里不是裁判系统给的最大功率）
// 输出：     基于裁判系统给的底盘最高功率的功率调整量
// 反馈数据：  超级电容留存的电量值的开平方
// 为什么要开平方：这是贴合能量与功率非线性特性的优化手段，能让 PID 控制器在调节时更线性、更稳定

/* 能量环 PID 控制器参数 */
float PowerLimitator_Energy_PID_Param[3] = {PowerLimitator_Energy_PID_Kp,   // Kp
                                            PowerLimitator_Energy_PID_Ki,   // Ki
                                            PowerLimitator_Energy_PID_Kd};  // Kd

/**
************************************************************************
* @brief:      	PowerLimitator_Init: 功率限制器初始化
* @param[in]:   void
* @retval:     	void
* @details:    	初始化低通滤波器和能量环PID控制器
************************************************************************
**/
void PowerLimitator_Init() {

    /* 初始化低通滤波器 */
    for (int i = 0; i < PowerLimitator_Motor_Number; i++) {
        first_order_filter_init(&PowerLimitator_LowPassFilter[i], PowerLimitator_Task_Time / 1000.0f, power_filter_param);
    }

    /* 初始化能量环 PID 控制器 */
    PID_init(&PowerLimitator_Energy_PID, PID_POSITION, PowerLimitator_Energy_PID_Param, PowerLimitator_Energy_PID_Maxout, PowerLimitator_Energy_PID_iMaxout);

    // /* 初始化功率模型电机参数 */
    PowerLimitator_Motor_Status_Init(0,                 // Motor_Number);
                                     0.005f,             // k0
                                     0.055293495677826994f,     // k1
                                     7.1416853145f,              // k2
                                     0.570335448f,       // k3
                                     0.005f              // torqueConst
                                     );
}

/**
************************************************************************
* @brief:      	rpm2rad: 将转速转换为角速
* @param[in]:   rpm: 输入的转速值，单位为RPM（转每分钟）
* @retval:     	float: 转换后的角速度值，单位为弧度/秒
* @details:    	根据固定转换系数RPM_TO_RAD（2π/60，约0.1047）将输入的转速值转换为角速度值
************************************************************************
**/
float rpm2rad(float rpm) {
    return rpm * RPM_TO_RAD;
}

/**
************************************************************************
* @brief:      	current2Torque: 将电流转换为扭矩
* @param[in]:   motor_number: 电机编号（0-3），用于索引获取对应电机的k0系数
* @param[in]:   current: 输入的电流值，单位根据系统定义
* @retval:     	float: 转换后的扭矩值，单位根据系统定义
* @details:    	根据指定电机的扭矩系数k0，将输入的电流值转换为对应的扭矩值，计算公式为：扭矩 = 电流 × 电机k0系数
************************************************************************
**/
float current2Torque(int motor_number, float current) {
    return current * Motor_PowerLimitator[motor_number].k0;
}

/**
************************************************************************
* @brief:      	PowerLimitator_Motor_Status_Init: 电机数据初始化
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
void PowerLimitator_Motor_Status_Init(int Motor_Number,
                                      float k0,
                                      float k1,
                                      float k2,
                                      float k3,
                                      float torqueConst)
{
    /* 初始化电机参数 */
    Motor_PowerLimitator[Motor_Number].k0 = k0;
    Motor_PowerLimitator[Motor_Number].k1 = k1;
    Motor_PowerLimitator[Motor_Number].k2 = k2;
    Motor_PowerLimitator[Motor_Number].k3 = k3;
    Motor_PowerLimitator[Motor_Number].torqueConst = torqueConst;
}

/**
************************************************************************
* @brief:      	PowerLimitator_Motor_Status_Update: 更新电机数据函数接口
* @param[in]:   Motor_Number: 电机编号
* @param[in]:   getRadFeedback: 反馈角速度
* @param[in]:   getTorqueFeedback: 反馈扭矩
* @param[in]:   pids_out: PID输出值
* @retval:     	void
* @details:    	更新指定电机的实时状态数据
************************************************************************
**/
void PowerLimitator_Motor_Status_Update(uint8_t Motor_Number,
                                        float getRadFeedback,
                                        float getTorqueFeedback,
                                        float pids_out
                                        )
{
    /* 更新电机参数 */
    Motor_PowerLimitator[Motor_Number].getRadFeedback = getRadFeedback;
    Motor_PowerLimitator[Motor_Number].getTorqueFeedback = getTorqueFeedback;
    Motor_PowerLimitator[Motor_Number].pids_out = pids_out;
}

/**
************************************************************************
* @brief:      	PowerLimitator_Motor_Update: 更新电机数据函数
* @param[in]:   void
* @retval:     	void
* @details:    	对电机扭矩做滤波计算并更新电机参数
************************************************************************
**/
void PowerLimitator_Motor_Update()
{
    /* 对轮电机电流做滤波计算 */
    for (int i = 0; i < PowerLimitator_Motor_Number; i++) {
        first_order_filter_cali(&PowerLimitator_LowPassFilter[i], chassis_wheel[i].given_current);
    }

    // /* 更新电机参数 */
    // PowerLimitator_Motor_Status_Update(0,
    //                                  rpm2rad(motor3508_chassis[1].speed_rpm),
    //                                  // 0.0f,
    //                                  ((PowerLimitator_LowPassFilter[0].out / 1000.0f) * Motor_PowerLimitator[0].torqueConst),
    //                                  (M2006_Speed_PID.out / 1000.0f)
    //                                  );


}

/**
************************************************************************
* @brief:      	PowerLimitator_Status_Update: 更新功率限制器状态
* @param[in]:   updatedMaxPower: 更新后的最大功率限制值
* @retval:     	void
* @details:    	函数接收更新后的最大功率值并设置，随后遍历电机，计算各电机的当前转速与扭矩
*               据此求解每个电机的当前功率和基于PID输出的命令功率，最后累加得到总估计功率和总命令功
************************************************************************
**/

float PowerLimitator_Status_Update(float chassisTargetSpeed,
                                   float chassisCurrentSpeed,
                                   float Uspeed,
                                   float Uyaw,
                                   float wLeftWheel,
                                   float leftUelse,
                                   float wRightWheel,
                                   float rightUelse)
{
    Motor_PowerLimitator->chassisTargetSpeed  = chassisTargetSpeed;
    Motor_PowerLimitator->chassisCurrentSpeed = chassisCurrentSpeed;
    Motor_PowerLimitator->Uspeed              = Uspeed;
    Motor_PowerLimitator->Uyaw                = Uyaw;
    Motor_PowerLimitator->wLeftWheel          = wLeftWheel;
    Motor_PowerLimitator->leftUelse           = leftUelse;
    Motor_PowerLimitator->wRightWheel         = wRightWheel;
    Motor_PowerLimitator->rightUelse          = rightUelse;

    if (floatEqual(Uyaw, 0.0f))
    {
        if (Uspeed * Uyaw > 0.0f)
            Motor_PowerLimitator->K = 10000.0f;
        else
            Motor_PowerLimitator->K = -10000.0f;
    }
    else
        Motor_PowerLimitator->K = Uspeed / Uyaw;

    // 计算出当前需要的总扭矩（Uspeed + Uyaw + Uelse）和目标功率（此处需要修改为新版功率模型）
    float leftTotalTorque  = Uspeed - Uyaw + leftUelse;
    float rightTotalTorque = Uspeed + Uyaw + rightUelse;

    Motor_PowerLimitator->cmdPower = (Motor_PowerLimitator->k1 * (fabs(wLeftWheel) + fabs(wRightWheel))) + (Motor_PowerLimitator->k2 * (powf(leftTotalTorque, 2) + powf(rightTotalTorque, 2))) +
                       Motor_PowerLimitator->k3 + (wLeftWheel * leftTotalTorque) + (wRightWheel * rightTotalTorque);

    // 如果目标功率小于目前超电预设的最大功率，则保留当前
    if (Motor_PowerLimitator->cmdPower <= PowerLimitator_Chassis.maxPower)
    {
        Motor_PowerLimitator->restrictedUspeed = Motor_PowerLimitator->Uspeed;
        Motor_PowerLimitator->restrictedUyaw   = Motor_PowerLimitator->Uyaw;

        Motor_PowerLimitator->decayUspeed = 1.0f;
        Motor_PowerLimitator->decayUyaw   = 1.0f;
    }
    // 如果目标功率大于可以使用的最大功率，则根据当前车速和目标车速的关系来判断是使用大P分配还是等比分配法，并计算出受限的 Uspeed 和 Uyaw
    else
    {
        float speedError = Motor_PowerLimitator->chassisTargetSpeed - Motor_PowerLimitator->chassisCurrentSpeed;

        if (fabs(speedError) > 0.5f && fabs(Motor_PowerLimitator->chassisCurrentSpeed) > 0.5f &&
            ((Motor_PowerLimitator->chassisTargetSpeed * Motor_PowerLimitator->chassisCurrentSpeed < 0.0f) || (Motor_PowerLimitator->chassisTargetSpeed >= 0.0f && speedError < 0.0f) ||
             (Motor_PowerLimitator->chassisTargetSpeed <= 0.0f && speedError > 0.0f)))
        {
            Motor_PowerLimitator->decayUspeed = Motor_PowerLimitator->decayUspeed * 0.97f + 1.0f * 0.03f;
            Motor_PowerLimitator->decayUyaw   = Motor_PowerLimitator->decayUyaw * 0.97f + 1.0f * 0.03f;
        }
        else
        {
            // 推导计算出受限的 Uspeed 和 Uyaw，首先根据功率模型的公式，列出关于 Uyaw 的二次方程，并计算出 A、B、C 和 Delta 的值
            Motor_PowerLimitator->A = (Motor_PowerLimitator->k2 * (2.0f * powf(Motor_PowerLimitator->K, 2) + 2.0f));
            Motor_PowerLimitator->B = (2.0f * Motor_PowerLimitator->k2 * (Motor_PowerLimitator->K - 1.0f) * Motor_PowerLimitator->leftUelse) + (2.0f * Motor_PowerLimitator->k2 * (Motor_PowerLimitator->K + 1.0f) * Motor_PowerLimitator->rightUelse) +
                        (Motor_PowerLimitator->wLeftWheel * (Motor_PowerLimitator->K - 1.0f)) + (Motor_PowerLimitator->wRightWheel * (Motor_PowerLimitator->K + 1.0f));
            Motor_PowerLimitator->C = (Motor_PowerLimitator->wLeftWheel * Motor_PowerLimitator->leftUelse) + (Motor_PowerLimitator->wRightWheel * Motor_PowerLimitator->rightUelse) +
                        (Motor_PowerLimitator->k1 * (fabs(Motor_PowerLimitator->wLeftWheel) + fabs(Motor_PowerLimitator->wRightWheel))) +
                        (Motor_PowerLimitator->k2 * (powf(Motor_PowerLimitator->leftUelse, 2) + powf(Motor_PowerLimitator->rightUelse, 2))) + Motor_PowerLimitator->k3 - PowerLimitator_Chassis.maxPower;

            Motor_PowerLimitator->Delta = powf(Motor_PowerLimitator->B, 2) - 4 * Motor_PowerLimitator->A * Motor_PowerLimitator->C;

            // 根据 Delta 的值来判断二次方程的根的情况，并计算出受限的 Uyaw 和 Uspeed
            // 单一解，
            if (floatEqual(Motor_PowerLimitator->Delta, 0.0f))  // repeat roots
            {
                float tempUyaw = (-Motor_PowerLimitator->B) / (2.0f * Motor_PowerLimitator->A);

                if (tempUyaw * Motor_PowerLimitator->Uyaw > 0.0f)
                    Motor_PowerLimitator->restrictedUyaw = tempUyaw;
                else
                    Motor_PowerLimitator->restrictedUyaw = 0.0f;

                Motor_PowerLimitator->restrictedUspeed = Motor_PowerLimitator->K * Motor_PowerLimitator->restrictedUyaw;
            }
            // 两个解都不为零，并且与当前的 Uyaw 同号，此时说明可以通过调整 Uyaw 来满足功率限制的要求，因此选择其中较大的一个（如果 Uyaw > 0 的话）或者较小的一个（如果 Uyaw < 0 的话）作为受限的 Uyaw，并计算出受限的 Uspeed
            else if (Motor_PowerLimitator->Delta > 0.0f)  // distinct roots
            {
                float tempUyaw1 = (-Motor_PowerLimitator->B - sqrtf(Motor_PowerLimitator->Delta)) / (2.0f * Motor_PowerLimitator->A);
                float tempUyaw2 = (-Motor_PowerLimitator->B + sqrtf(Motor_PowerLimitator->Delta)) / (2.0f * Motor_PowerLimitator->A);

                if (tempUyaw1 * Motor_PowerLimitator->Uyaw > 0.0f && tempUyaw2 * Motor_PowerLimitator->Uyaw > 0.0f)
                {
                    if (Motor_PowerLimitator->Uyaw > 0.0f)
                        Motor_PowerLimitator->restrictedUyaw = fmax(tempUyaw1, tempUyaw2);
                    else
                        Motor_PowerLimitator->restrictedUyaw = fmin(tempUyaw1, tempUyaw2);
                }
                else if (tempUyaw1 * Motor_PowerLimitator->Uyaw > 0.0f)
                    Motor_PowerLimitator->restrictedUyaw = tempUyaw1;
                else if (tempUyaw2 * Motor_PowerLimitator->Uyaw > 0.0f)
                    Motor_PowerLimitator->restrictedUyaw = tempUyaw2;
                else
                    Motor_PowerLimitator->restrictedUyaw = 0.0f;

                Motor_PowerLimitator->restrictedUspeed = Motor_PowerLimitator->K * Motor_PowerLimitator->restrictedUyaw;
            }
            // 无实数解，此时说明在当前的功率限制下，无法通过调整 Uyaw 来满足功率限制的要求，因此只能将 Uyaw 限制为 0，并将 Uspeed 限制为 0（如果 K 不等于 0 的话）
            else if (Motor_PowerLimitator->Delta < 0.0f)  // imaginary roots
            {
                float tempUyaw = (-Motor_PowerLimitator->B) / (2.0f * Motor_PowerLimitator->A);

                if (tempUyaw * Motor_PowerLimitator->Uyaw > 0.0f)
                    Motor_PowerLimitator->restrictedUyaw = tempUyaw;
                else
                    Motor_PowerLimitator->restrictedUyaw = 0.0f;

                Motor_PowerLimitator->restrictedUspeed = Motor_PowerLimitator->K * Motor_PowerLimitator->restrictedUyaw;
            }

            float tempDecayUspeed = Motor_PowerLimitator->restrictedUspeed / Uspeed;
            float tempDecayUyaw   = Motor_PowerLimitator->restrictedUyaw / Uyaw;

            tempDecayUspeed = float_constrain(tempDecayUspeed, 0.01f, 1.00f);
            tempDecayUyaw   = float_constrain(tempDecayUyaw, 0.01f, 1.00f);

            Motor_PowerLimitator->decayUspeed = tempDecayUspeed * 0.1f + Motor_PowerLimitator->decayUspeed * 0.9f;
            Motor_PowerLimitator->decayUyaw   = tempDecayUyaw * 0.1f + Motor_PowerLimitator->decayUyaw * 0.9f;
        }
    }

    // 根据受限的 Uspeed 和 Uyaw 以及原来的 Uelse，计算出受限的总扭矩
    float restrictedLeftTotalTorque  = Motor_PowerLimitator->restrictedUspeed - Motor_PowerLimitator->restrictedUyaw + leftUelse;
    float restrictedRightTotalTorque = Motor_PowerLimitator->restrictedUspeed + Motor_PowerLimitator->restrictedUyaw + rightUelse;

    // 根据受限的 Uspeed 和 Uyaw 以及原来的 Uelse，计算出受限的总扭矩，并根据功率模型的公式计算出受限的功率值(要修改为新模型)
    PowerLimitator_Chassis.estimatedPower = (Motor_PowerLimitator->k1 * (fabs(wLeftWheel) + fabs(wRightWheel))) +
                              (Motor_PowerLimitator->k2 * (powf(restrictedLeftTotalTorque, 2) + powf(restrictedRightTotalTorque, 2))) + Motor_PowerLimitator->k3 +
                              (wLeftWheel * restrictedLeftTotalTorque) + (wRightWheel * restrictedRightTotalTorque);

    return restrictedLeftTotalTorque, restrictedRightTotalTorque;
}


/**
************************************************************************
* @brief:      	PowerLimitator_getDecayCurrent: 计算功率限制下的衰减电流
* @param[in]:   void
* @retval:     	void
* @details:    	函数先初始化可分配功率和所需功率总和，遍历 PowerLimitator_Motor_Number 个电机计算两者数值
*               当命令功率超过最大功率限制时，设置功率控制状态为受限，对命令功率为正的电机
*               通过转速转换、功率权重计算及二次方程求解得到衰减电流；未超过限制时，设置状态为非受限
************************************************************************
**/
void PowerLimitator_getDecayCurrent()
{
    float allocatablePower = PowerLimitator_Chassis.maxPower, powerSumRequired = 0;

    for (int i = 0; i < PowerLimitator_Motor_Number; i++)
    {
        if (Motor_PowerLimitator[i].cmdPower > 0.0f)
        {
            powerSumRequired += Motor_PowerLimitator[i].cmdPower;
        }
        else
        {
            allocatablePower -= Motor_PowerLimitator[i].cmdPower;
        }
    }

    // Start to calculate the decay current, if the power is over the limit
    if (PowerLimitator_Chassis.commandPower > PowerLimitator_Chassis.maxPower)
    {
        PowerLimitator_Chassis.powerControlStatus = RESTRICTED;
        for (int i = 0; i < PowerLimitator_Motor_Number; i++)
        {
            if ( floatEqual(Motor_PowerLimitator[i].cmdPower, 0.0f) || Motor_PowerLimitator[i].cmdPower < 0.0f )
            {
                continue;
            }
            float curAv       = rpm2rad(Motor_PowerLimitator[i].getRadFeedback);
            float powerWeight = Motor_PowerLimitator[i].cmdPower / powerSumRequired;
            float delta       = curAv * curAv - 4.0f * Motor_PowerLimitator[i].k2 * (Motor_PowerLimitator[i].k1 * fabs(curAv) + Motor_PowerLimitator[i].k3 / (float)PowerLimitator_Motor_Number - powerWeight * allocatablePower);
            if ( floatEqual(delta, 0.0f) )  // repeat roots
            {
                Motor_PowerLimitator[i].decayCurrent = -curAv / (2.0f * Motor_PowerLimitator[i].k2) / Motor_PowerLimitator[i].k0;
            }
            else if (delta > 0.0f)  // distinct roots
            {
                Motor_PowerLimitator[i].decayCurrent = Motor_PowerLimitator[i].decayCurrent > 0.0f ? (-curAv + sqrtf(delta)) / (2.0f * Motor_PowerLimitator[i].k2) / Motor_PowerLimitator[i].k0 : (-curAv - sqrtf(delta)) / (2.0f * Motor_PowerLimitator[i].k2) / Motor_PowerLimitator[i].k0;
            }
            else  // imaginary roots
            {
                Motor_PowerLimitator[i].decayCurrent = -curAv / (2.0f * Motor_PowerLimitator[i].k2) / Motor_PowerLimitator[i].k0;
            }

            // 根据功率状态选择电流
            Motor_PowerLimitator[i].finalcurrent = Motor_PowerLimitator[i].decayCurrent;

        }
    }
    else
    {
        PowerLimitator_Chassis.powerControlStatus = NOT_RESTRICTED;

        // 根据功率状态选择电流
        for (int i = 0; i < PowerLimitator_Motor_Number; i++) {
            Motor_PowerLimitator[i].finalcurrent = Motor_PowerLimitator[i].pids_out;
        }
    }


}

/**
************************************************************************
* @brief:      	floatEqual: 判断两个浮点数是否相等
* @param[in]:   a: 第一个浮点数
* @param[in]:   b: 第二个浮点数
* @retval:     	bool: 若两浮点数相等则返回true，否则返回false
* @details:    	考虑浮点数精度问题，先处理NaN（任一为NaN则不相等）和无穷大（同符号无穷大则相等）情况
*               再通过绝对误差（小于1e-6）或相对误差（绝对差值与两数中绝对值较大者的比值小于1e-6）判断相等性
************************************************************************
**/
bool_t floatEqual(float a, float b) {

    if (isnan(a) || isnan(b)) {
        return false;
    }

    if (isinf(a) && isinf(b)) {
        return (a > 0 && b > 0) || (a < 0 && b < 0);
    }

    float abs_diff = fabs(a - b);

    if (abs_diff < 1e-6f) {
        return true;
    }

    float abs_a = fabs(a);
    float abs_b = fabs(b);
    float max_abs = (abs_a > abs_b) ? abs_a : abs_b;

    return (abs_diff / max_abs) < 1e-6f;
}

/**
************************************************************************
* @brief:      	PowerLimitator_Energy_PID_Update: 能量环PID更新
* @param[in]:   void
* @retval:     	void
* @details:    	计算能量环PID并更新底盘最大功率限制
************************************************************************
**/
void PowerLimitator_Energy_PID_Update() {

    /* 能量环 PID 控制器计算 */
    PID_Calc(&PowerLimitator_Energy_PID, sqrtf(Cap_PowerLimitator.capacity), sqrtf(PowerLimitator_Energy_PID.set));

    /* 应用能量环 PID 控制器计算结果 */
    PowerLimitator_Chassis.maxPower =  Referee_PowerLimitator.refereeMaxPower - PowerLimitator_Energy_PID.out;

}

/**
************************************************************************
* @brief:      	PowerLimitator_Energy_Capacity_Set: 设置能量容量
* @param[in]:   capacity: 容量值
* @retval:     	void
* @details:    	设置能量环PID的目标容量
************************************************************************
**/
void PowerLimitator_Energy_Capacity_Set(float capacity) {
    PowerLimitator_Energy_PID.set = capacity;
}