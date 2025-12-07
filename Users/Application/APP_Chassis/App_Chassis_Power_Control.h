#include "App_Chassis_Task.h"
#include "Alg_RLS.hpp"
#include "App_Chassis_Task.h"
#include "main.h"

#define USE_SUPER_CAPACITOR 1

#pragma once

namespace Core
{
    namespace Control {
        namespace Power {
#ifdef __cplusplus
            extern "C"
            {
#endif

                //宏定义
                constexpr static float motor_3508_torque_const     = 0.3f; //3508电机扭矩常数 单位NM/A
                constexpr static float motor_3508_reduction_ratio  = 15.7647059f; // 3508的减速比
                constexpr static float motor_3508_currentlimit     = 20.0f; // c620电调最大电流
                constexpr static float motor_3508_output_limit     = 16384.0f; // c620控制输出电流最大值

                constexpr static float motor_6020_torque_const     = 0.741f; //6020电机扭矩常数 单位NM/A
                constexpr static float motor_6020_currentlimit     = 3.0f; // 6020电调最大电流
                constexpr static float motor_6020_output_limit     = 16384.0f; // 6020控制输出电流最大值

                constexpr static float refereeFullBuffSet          = 60.0f;//裁判系统最大能量缓冲值
                constexpr static float refereeBaseBuffSet          = 50.0f;//裁判系统最小能量缓冲值
                constexpr static float capFullBuffSet              = 230.0f;//超级电容最大能量值
                constexpr static float capBaseBuffSet              = 30.0f;//超级电容能够被使用到的最低能量值，小于这个值就需要充电了
                constexpr static float error_powerDistribution_set = 20.0f;//决定用哪种功率分配方法的误差阈值
                constexpr static float prop_powerDistribution_set  = 15.0f;

                // constexpr float MIN_MAXPOWER_CONFIGURED                   = 15.0f;
                constexpr float MAX_CAP_POWER_OUT                         = 300.0f;//最大电容输出
                constexpr float CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD = 43.0f;//电容离线时，判断是否有多余功率给超电充电的阈值
                constexpr float CAP_OFFLINE_ENERGY_TARGET_POWER           = 37.0f;//电容离线能量目标功率
                constexpr float MAX_POEWR_REFEREE_BUFF                    = 60.0f;//裁判系统能量环
                constexpr float REFEREE_GG_COE                            = 0.95f;//裁判系统断开系数？
                constexpr float CAP_REFEREE_BOTH_GG_COE                   = 0.85f;//裁判系统和超级电容通信同时断开的功率使用系数

                // maxLevel: 兵种最大级别，值为 10U。
                // HeroChassisPowerLimit_Melee_FIRST[maxLevel]: 英雄底盘不同级别的最大功率限制数组，适用于 近战优先 类型，值分别为 {55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U}。
                // HeroChassisPowerLimit_Remote_FIRST[maxLevel]: 英雄底盘不同级别的最大功率限制数组，适用于 远程优先 类型，值分别为 {55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U}。
                // InfantryChassisPowerLimit_HP_FIRST[maxLevel]: 步兵底盘在不同级别的最大功率限制数组，适用于 血量优先 类型，值分别为 {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U}。
                // InfantryChassisPowerLimit_PW_FIRST[maxLevel]: 步兵底盘在不同级别的最大功率限制数组，适用于 功率优先 类型，值分别为 {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U}。
                // SentryChassisPowerLimit: 哨兵底盘的最大功率限制，值为 120U。
                constexpr static uint8_t maxLevel                                     = 10U;
                constexpr static uint8_t HeroChassisPowerLimit_Melee_FIRST[maxLevel]  = {70U, 75U, 80U, 85U, 90U, 95U, 100U, 105U, 110U, 120U};
                constexpr static uint8_t HeroChassisPowerLimit_Remote_FIRST[maxLevel] = {50U, 55U, 60U, 60U, 70U, 75U, 80U, 85U, 90U, 100U};
                constexpr static uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel] = {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};
                constexpr static uint8_t InfantryChassisPowerLimit_PW_FIRST[maxLevel] = {60U, 65U, 70U, 75U, 80U, 85U, 90U, 95U, 100U, 100U};
                constexpr static uint8_t SentryChassisPowerLimit                      = 120U;

                enum class Division
                {
                    INFANTRY = 0,
                    HERO,
                    SENTRY
                  };

                struct Manager
                {
                    enum RLSEnabled : bool  //是否启动RLS
                    {
                        Disable = 0,
                        Enable  = 1
                    } rlsEnabled;

                    enum ErrorFlags //错误标志
                    {
                        MotorDisconnect   = 1U,
                        RefereeDisConnect = 2U,
                        CAPDisConnect     = 4U
                    };


                    uint8_t error;

                    /**
                     * @remark In case of initialization without explicit datas
                     */
                    Manager() = delete;

                    /**
                     * @brief The constructor of the power manager object
                     * @param motors_               The motor objects
                     * @todo                        This will change to the type of "AbstractFeedbackMotor*"
                     * @param division_             机器人的类型
                     * @param rlsEnabled_           启用或禁用 RLS 自适应参数模式，默认值为启用
                     * @param torqueConst_          电机的扭矩常数 (KA)，单位为 (N.m / A)，默认值为 0.22f。
                     * @param k1_                   功率估算电机的频率耗散参数 k1，默认值为 0.22f。
                     * @param k2_                   功率估算电机的电流耗散参数 k2，默认值为 1.2f。
                     * @param k3_                   常数功率损失参数 k3，默认值为 2.78f。
                     * @param lambda_               RLS 更新遗忘因子，默认值为 0.9999f。
                     */
                    /*

                    */
                    //默认初始化对象
                    // Manager(const Chassis_Motor motors_[],
                    //         const Division division_,
                    //         RLSEnabled rlsEnabled_ = Enable,
                    //         const float k1_        = 0.22f,
                    //         const float k2_        = 1.2f,
                    //         const float k3_        = 2.78f,
                    //         const float lambda_    = 0.9999f);

                    Division division;

                    float powerBuff; //能量缓冲值
                    float fullBuffSet;//能量上限值
                    float baseBuffSet;//能量下限值
                    float fullMaxPower;//功率环配合能量环自动算出来的功率上限
                    float baseMaxPower;//功率环配合能量环自动算出来的功率下限

                    float powerUpperLimit;//设定功率硬上限，也就是当前功率能够使用到的最大值，无论是自动分配功率，还是user自己设置功率，都不能超过这个阈值，会导致机器人超功率
                    float refereeMaxPower;//裁判系统规定最大功率

                    float userConfiguredMaxPower;//用户配置的最大功率值
                    float (*callback)(void);

                    float measuredPower;//测量的当前功率
                    float estimatedPower;//估计的当前功率
                    float estimatedCapEnergy;//估计的电容能量值

                    float torqueConst;//电机扭矩常数

                    float k1;//转速绝对值系数
                    float k2;//扭矩平方系数
                    float k3;//常数功率损失参数k3

                    TickType_t lastUpdateTick;//上次更新时间戳
                    Math::RLS<2> rls;//自适应参数估计
                };

                //电机相关参数，动力学解算后电机当前转速，目标转速，根据两者转速计算的pid输出值，pid输出最大值
                struct PowerObj
                {
                public:
                    float pidOutput;     // PID 控制器的输出命令，表示扭矩电流命令，范围在 [-maxOutput, maxOutput] 之间，没有单位
                    float curAv;         // 当前测量的角速度，范围在 [-maxAv, maxAv] 之间，单位为 rad/s
                    float setAv;         // 目标角速度，范围在 [-maxAv, maxAv] 之间，单位为 rad/s。
                    float pidMaxOutput;  // PID 控制器的最大输出值
                };

                /**
                * @brief Storing the power status of the chassis
                 */
                struct PowerStatus
                {
                public:
                    float userConfiguredMaxPower; //使用自定义配置的最大功率
                    float maxPowerLimited;//最大功率限制
                    float sumPowerCmd_before_clamp;//限制功率前的总功率命令
                    float effectivePower;//有效功率
                    float powerLoss;//功率损耗
                    float efficiency;//效率
                    uint8_t estimatedCapEnergy;//估计电容能量值
                    Manager::ErrorFlags error;
                };

                // return the latest feedback referee power limit(before referee disconnected), according to the robot level
                float getLatestFeedbackJudgePowerLimit();

                /**
                * @brief Get the controlled output torque current based on current model
                * @param objs The collections of power objects from four wheels, recording the necessary data from the PID controller
                * @retval The controlled output torque current
                */
                float *getControlledOutput(PowerObj *objs[4]);

                /**
                * @brief return the power status of the chassis
                * @retval The power status object
                */
                const volatile PowerStatus &getPowerStatus();

                /**
                * @brief The power controller module initialization function
                * @param manager The manager object
                * @note This function should be called before the scheduler starts
                */
                void init(const Manager &manager);

                /**
                * @brief set the user configured max power
                * @param maxPower The max power value
                * @note The max power configured by this function will compete with the basic energy limitation, to ensure system does not die
                */
                void setMaxPowerConfigured(float maxPower);

                void setMode(uint8_t mode);

                void registerPowerCallbackFunc(float (*callback)(void));

                /**
                * @brief Enable for disable the automatically parameters update process
                * @param isUpdate disable with 0, enable with 1
                * @note  The system will automatically disable the update when both referee system and cap is disconnect from the power module
                * @retval None
                */
                void setRLSEnabled(uint8_t isUpdate);

            }
        }
    }
#ifdef __cplusplus
}
#endif

//extern sup_measure_t sup_cap[1];

