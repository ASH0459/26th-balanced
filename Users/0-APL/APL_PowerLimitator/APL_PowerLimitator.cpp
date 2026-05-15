/**
****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_PowerLimitator.cpp
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

/**
 * @brief 头文件
 */
#include "APL_PowerLimitator.hpp"
#include "App_Detect_Task.h"
#include "UC_Referee.h"
#include "math.h"

/* 英雄机器人类型定义：0为近战优先，1为远程优先 */
#define HERO_TYPE_MELEE 0
#define HERO_TYPE_RANGED 1
#define INFANTRY_TYPE_POWER 2
#define INFANTRY_TYPE_HP 3

#define CURRENT_HERO_TYPE HERO_TYPE_MELEE

/* 历史等级记录 */
static uint8_t historical_robot_level = 1;

/* 离线电机幽灵功率缓冲时间记录 */
static uint32_t motor_offline_time[PowerLimitator_Motor_Number] = {0};

/**
 * @brief 获取英雄机器人底盘功率上限
 * @param level 机器人等级 (1-10)
 * @param type 机器人类型 (HERO_TYPE_MELEE 或 HERO_TYPE_RANGED)
 * @return 功率上限 (W)
 */
static float Get_Infantry_Power_Limit(uint8_t level, uint8_t type) {
    if (level < 1) level = 1;
    if (level > 10) level = 10;
    
    if (type == INFANTRY_TYPE_POWER) {
        const float melee_power[10] = {60, 65, 70, 75, 80, 95, 90, 95, 100, 100};
        return melee_power[level - 1];
    } else if(type == INFANTRY_TYPE_HP) {
        const float ranged_power[10] = {45, 50, 55, 60, 65, 70, 75, 80, 90, 100};
        return ranged_power[level - 1];
    }
    else {
        return 0;
    }
}

/*===================[ 创建任务属性 ]===================*/

WheelLeggedPowerLimitator WL_PowerManager;

/**
 * @brief 功率控制任务句柄
 */
osThreadId_t PowerLimitatorHandle;

/**
 * @brief 功率控制任务属性配置
 */
const osThreadAttr_t PowerLimitator_attributes = {
    .name = "PowerLimitator",
    .stack_size = 128 * 8,
    .priority = (osPriority_t) osPriorityAboveNormal,
  };

static inline bool float_Equal(float a, float b) { return fabs(a - b) < 1e-5f; }

// 将宏定义限制在合理范围内
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void WheelLeggedPowerLimitator::Init() {
    // 初始化能量环 PID (PD 控制, 参数需根据实车调节)
    const float energy_pid_params[3] = {30.0f, 0.0f, 0.1f}; // Kp, Ki, Kd
    PID_init(&energy_pid, PID_POSITION, energy_pid_params, 300.0f, 0.0f);

    decayUspeed = 1.0f;
    decayUyaw = 1.0f;
    configuredMaxPower = 60.0f; // 默认安全值
}

void WheelLeggedPowerLimitator::Update_Power_Loop(float referee_limit, float cap_energy, bool is_cap_online) {
    if (is_cap_online) {
        // 定义期望剩余能量 E_target
        float buffSet = (mode == BATTERY) ? (100.0f * 0.4f) : (100.0f * 0.4f);

        // P_max = P_r - PD(sqrt(E_target), sqrt(E_current))
        // 使用能量的平方根进行PID计算以防止非线性突变
        float pid_out = -PID_Calc(&energy_pid, sqrtf(cap_energy), sqrtf(buffSet));

        configuredMaxPower = referee_limit + pid_out;

        // 限制在最小安全功率 35W 以上，防止软腿
        if(configuredMaxPower < 35.0f) configuredMaxPower = 35.0f;
    } else {
        // 超电掉线，直接使用裁判系统限制 (扣除一点余量防超)
        configuredMaxPower = referee_limit * 0.9f;
    }
}

void WheelLeggedPowerLimitator::Calculate_Decay(float Uspeed, float Uyaw,
                                                float leftUelse, float rightUelse,
                                                float wLeftWheel, float wRightWheel) {

    float K;
    // 约束条件 Uspeed = K * Uyaw
    if (floatEqual(Uyaw, 0.0f)) {
        K = (Uspeed * Uyaw > 0.0f) ? 10000.0f : -10000.0f;
    } else {
        K = Uspeed / Uyaw;
    }

    // 计算当前需求扭矩 (注意正负号：Tl = Uspeed - Uyaw + Uelse)
    float leftTotalTorque  = Uspeed + Uyaw + leftUelse;
    float rightTotalTorque = Uspeed - Uyaw + rightUelse;

    // 根据电机模型预测指令总功率
    float cmdPower = k0 * (wLeftWheel * leftTotalTorque + wRightWheel * rightTotalTorque) +
                     k1 * (fabs(wLeftWheel) + fabs(wRightWheel)) +
                     k2 * (leftTotalTorque * leftTotalTorque + rightTotalTorque * rightTotalTorque) +
                     k3;

    if (cmdPower <= configuredMaxPower) {
        // 功率充裕，不衰减
        decayUspeed = 1.0f;
        decayUyaw   = 1.0f;
    } else {
        // 功率超出，试图解出限制后的 Uyaw_restricted
        float restrictedUyaw = 0.0f;
        float restrictedUspeed = 0.0f;

        // 构建一元二次方程 Ax^2 + Bx + C = 0，其中 x = Uyaw_restricted
        float A = k2 * (2.0f * K * K + 2.0f);
        float B = k0 * (wLeftWheel * (K - 1.0f) + wRightWheel * (K + 1.0f)) +
                  2.0f * k2 * ((K - 1.0f) * leftUelse + (K + 1.0f) * rightUelse);
        float C = k0 * (wLeftWheel * leftUelse + wRightWheel * rightUelse) +
                  k1 * (fabs(wLeftWheel) + fabs(wRightWheel)) +
                  k2 * (leftUelse * leftUelse + rightUelse * rightUelse) +
                  k3 - configuredMaxPower;

        float Delta = B * B - 4.0f * A * C;

        if (floatEqual(Delta, 0.0f)) {
            // 单一解
            float tempUyaw = (-B) / (2.0f * A);
            restrictedUyaw = (tempUyaw * Uyaw > 0.0f) ? tempUyaw : 0.0f;
        }
        else if (Delta > 0.0f) {
            // 两个实根，选取与原 Uyaw 符号相同且绝对值较大的有效解
            float tempUyaw1 = (-B - sqrtf(Delta)) / (2.0f * A);
            float tempUyaw2 = (-B + sqrtf(Delta)) / (2.0f * A);

            if (tempUyaw1 * Uyaw > 0.0f && tempUyaw2 * Uyaw > 0.0f) {
                restrictedUyaw = (Uyaw > 0.0f) ? fmax(tempUyaw1, tempUyaw2) : fmin(tempUyaw1, tempUyaw2);
            } else if (tempUyaw1 * Uyaw > 0.0f) {
                restrictedUyaw = tempUyaw1;
            } else if (tempUyaw2 * Uyaw > 0.0f) {
                restrictedUyaw = tempUyaw2;
            } else {
                restrictedUyaw = 0.0f;
            }
        }
        else if (Delta < 0.0f) {
            // 无实数解 (Delta < 0)，取顶点
            float tempUyaw = (-B) / (2.0f * A);
            restrictedUyaw = (tempUyaw * Uyaw > 0.0f) ? tempUyaw : 0.0f;
        }

        restrictedUspeed = K * restrictedUyaw;

        // 计算衰减比例并限幅

        float tempDecayUspeed = (fabs(Uspeed) > 1e-5f) ? (restrictedUspeed / Uspeed) : 0.01f;
        float tempDecayUyaw   = (fabs(Uyaw) > 1e-5f)   ? (restrictedUyaw / Uyaw) : 0.8f;

        tempDecayUspeed = CLAMP(tempDecayUspeed, 0.01f, 1.0f);
        tempDecayUyaw   = CLAMP(tempDecayUyaw, 1.0f, 1.0f);

        // 低通滤波平滑输出衰减因子，防止控制突变导致机体抖动
        decayUspeed = tempDecayUspeed * 0.1f + decayUspeed * 0.9f;
        decayUyaw   = tempDecayUyaw * 0.1f + decayUyaw * 0.9f;
        // decayUspeed = 1.0f;
        // decayUyaw = 1.0f;
    }
}

/**
************************************************************************
* @brief:      	PowerLimitator_Task: 功率控制任务
* @param[in]:   pvParameters: 任务参数(未使用)
* @retval:     	void
* @details:    	FreeRTOS任务，周期性执行底盘功率控制逻辑，包括更新电机数据、
*               计算能量环PID、更新最大功率限制并计算衰减电流。
************************************************************************
**/
/**
 * @brief 功率控制任务
 */
void PowerLimitator_Task(void *pvParameters) {
    WL_PowerManager.Init();

    while (1) {
        float target_power_limit = 0.0f;
        float cap_energy = 0.0f;
        bool is_cap_online = false;

        // 裁判系统逻辑 (提取限制功率)
        if (!toe_is_error(Referee_TOE)) {
            target_power_limit = robot_state.chassis_power_limit;
        } else {
            target_power_limit = 60.0f * 0.85f; // 离线保守值
        }

        // 超级电容逻辑
        if (!toe_is_error(SuperCap_TOE)) {
             cap_energy = SuperCap_Data.Data.capEnergy; // 填入你的实际获取代码
            //cap_energy = 100.0f;
            is_cap_online = true;
        }

        // 1. 更新当前允许的最大功率 (包含能量环PID补偿)
        WL_PowerManager.Update_Power_Loop(target_power_limit, cap_energy, is_cap_online);

        // 2. 提取 LQR 各项分量进行功率解算
        // 在实际工程中，你需要在 App_Chassis_Task 中将 LQR 算出的分量传进来
        // float U_speed = ... ;
        // float U_yaw = ... ;
        // float U_else_L = ... ;
        // float U_else_R = ... ;
        // float w_L = chassis_move.chassis_wheel[0].vel;
        // float w_R = chassis_move.chassis_wheel[1].vel;

        // WL_PowerManager.Calculate_Decay(U_speed, U_yaw, U_else_L, U_else_R, w_L, w_R);

        osDelay(1); // 2ms 周期，与底盘控制任务同频
    }
}