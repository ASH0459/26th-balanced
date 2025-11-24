// #include "App_Chassis_Power_Control.h"
// #include "UC_Referee.h"
// #include "arm_math.h"
// #include "App_Detect_Task.h"
// #include "Dev_Remote_Control.h"
// #include "Alg_PID.h"
// #include "App_Chassis_Task.h"
// #include "Dev_CAN_Receive.h"
// #include "Alg_RLS.hpp"
// #include "Alg_UserLib.h"
// #include "APL_RC_Hub.h"
//
// namespace Core
// {
// namespace Control
// {
// namespace Power
// {
//
//
// #define POWER_PD_KP 50.0f
//
// Manager manager(chassis_move.motor_chassis, Division::INFANTRY); // 初始化manager对象，输入传入的底盘数据以及兵种类型
// PowerStatus powerStatus;
// static uint8_t LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL;
// static bool isCapEnergyOut                = false;  // 超级电容充电标志位
// static uint16_t motorDisconnectCounter[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};   // 电机连接情况标志位
// static float MIN_MAXPOWER_CONFIGURED      = 30.0f;
//
// static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }
//
// static inline float rpm2av(float rpm) { return rpm * (float)M_PI / 30.0f; }
//
// static inline float av2rpm(float av) { return av * 30.0f / (float)M_PI; }
//
// static inline void setErrorFlag(uint8_t &curFlag, Manager::ErrorFlags setFlag) { curFlag |= static_cast<uint8_t>(setFlag); }
//
// static inline void clearErrorFlag(uint8_t &curFlag, Manager::ErrorFlags clearFlag) { curFlag &= (~static_cast<uint8_t>(clearFlag)); }
//
// static inline bool isFlagged(uint8_t &curFlag, Manager::ErrorFlags flag) { return (curFlag & static_cast<uint8_t>(flag)) != 0; }
//
// //static inline bool isMotorConnected(const Chassis_Motor *motor) { return motor != nullptr && motor->getDisconnectCounter() < 10; }
//
// /*检测是否所有电机都已连接*/
// // static inline bool isAllMotorConnected()
// // {
// //     for (int i = 0; i < 8; i++)
// //         if (not isMotorConnected(manager.Motors[i]))
// //             return false;
// //     return true;
// // }
//
// Manager::Manager(
//     const Chassis_Motor motors_[], const Division division_, RLSEnabled rlsEnabled_, const float k1_, const float k2_, const float k3_, const float lambda_)
//
//     : rlsEnabled(rlsEnabled_),
//       error(0UL),
//       motors(motors_),
//       division(division_),
//       powerBuff(0.0f),
//       fullBuffSet(0.0f),
//       baseBuffSet(0.0f),
//       fullMaxPower(0.0f),
//       baseMaxPower(0.0f),
//       powerUpperLimit(0.0f),
//       refereeMaxPower(0.0f),
//       userConfiguredMaxPower(0.0f),
//       callback(nullptr),
//       torqueConst(0.0f),
//       k1(k1_),
//       k2(k2_),
//       k3(k3_),
//       lastUpdateTick(0),
//       rls(1e-5f, 0.99999f)
// {
//     configASSERT(k1_ >= 0);
//     configASSERT(k2_ >= 0);
//     configASSERT(k3_ >= 0);
//
//     //初始化两个系数加入到自定义调参功能中
//     float initParams[2] = {k1_, k2_};
//     rls.setParamVector(Matrixf<2, 1>(initParams));
// }
//
// static bool isInitialized;//初始化标志位
// StackType_t xPowerTaskStack[1024];//功率控制任务堆栈
// StaticTask_t uxPowerTaskTCB;//功率控制任务控制块
//
//     //裁判系统信息，待改
// using namespace Core::Communication;
// RefereeSystem::RefereeSystemMessageReceive<RefereeSystem::RefereePowerHeatMessageData> powerMessage; //裁判系统功率信息
// RefereeSystem::RefereeSystemMessageReceive<RefereeSystem::RefereeRobotStatusMessageData> robotMessage; //裁判系统机器人状态信息
//
// #if USE_SUPER_CAPACITOR
// const Core::Control::SuperCapacitor::CapacitorStatus &capStatus = Core::Control::SuperCapacitor::getStatus();
// #endif
//
// // POWER_PD_KP: 比例增益。
// // 0.0f: 积分增益（KI）设置为 0.0。
// // 0.2f: 微分增益 (KD)。
// // 0.0f: 积分部分的最小值。
// // MAX_CAP_POWER_OUT: 最大输出值。
// // 0.0f: 微分部分的最小值。?
// // 0.0f: 微分部分的最大值。？
// // 0.2f: 积分部分的最大值。?
// // 100UL: 微分部分的时间常数。?
// Core::Control::PID::Param powerPDParam(POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.0f, 0.0f, 0.2f, 100UL); //超级电容功率控制PID参数
// Core::Control::PID powerPD_base(powerPDParam);  //定义了一个名为 powerPD_base 的 PID 控制器对象，用于确保系统不会把电容的能量用尽。
// Core::Control::PID powerPD_full(powerPDParam);  //定义了一个名为 powerPD_full 的 PID 控制器对象，用于确保电容的能量保持在 90% 以下。
//
// /**
//  * @implements 设定用户最大功率值
//  */
// void setMaxPowerConfigured(float maxPower)
// {
//     //输入maxpower的值限制在MIN_MAXPOWER_CONFIGURED与powerUpperLimit之间
//     manager.userConfiguredMaxPower = float_constrain(maxPower , MIN_MAXPOWER_CONFIGURED , manager.powerUpperLimit);
// }
//
// void setMode(uint8_t mode) { setMaxPowerConfigured(mode == 1 ? manager.powerUpperLimit : manager.refereeMaxPower); }
//
// void registerPowerCallbackFunc(float (*callback)(void)) { manager.callback = callback; }
//
// /**
//  * @implements 使能或禁用 RLS 最小二乘法调参
//  */
// void setRLSEnabled(uint8_t enable) { manager.rlsEnabled = static_cast<Manager::RLSEnabled>(enable); }
//
// const volatile PowerStatus &getPowerStatus() { return powerStatus; }
//
// float getLatestFeedbackJudgePowerLimit() { return manager.refereeMaxPower; } //获取裁判系统功率上限
//
// /**
//  * @implements获取功率控制后的输出电流
//  */
//
// float *getControlledOutput(PowerObj *objs[4])
// {
//     //由于电机给的电流和扭矩成线性关系，所以这里计算扭矩电流率k0,用k0乘以电机电流就能得对应的电机扭矩
//     const float k0 = motor_3508_torque_const * manager.motors[0]->getCurrentLimit() /
//                      manager.motors[0]->getOutputLimit();  // torque current rate of the motor, defined as Nm/Output
//
//     static float newTorqueCurrent[4];//新的扭矩电流
//
//     float sumCmdPower = 0.0f;//总功率命令
//     float cmdPower[4];//单个电机功率命令
//
//     float sumError = 0.0f;//总电机转速误差
//     float error[4];//单个电机转速误差
//
//     //获得最大功率值userConfiguredMaxPower，如果超过了fullMaxPower取较小值，如果小于baseMaxPower则取其较大
//     float maxPower = Utils::Math::clamp(manager.userConfiguredMaxPower, manager.fullMaxPower, manager.baseMaxPower);
//
//     float allocatablePower = maxPower;
//     float sumPowerRequired = 0.0f;
// #if USE_DEBUG
//     static float newCmdPower;
// #endif
//
//     //计算每个电机在功率约束前的单个功率以及底盘需求的总功率
//     for (int i = 0; i < 4; i++)
//     {
//         if (isMotorConnected(manager.motors[i]))
//         {
//             PowerObj *p = objs[i];
//             //计算单个电机pid输出后功率约束前的输出，公式：单个电机功率 = 扭矩 * 转速 + k1 * |转速| + k2 * 扭矩^2 + k3 / 4
//             cmdPower[i] = p->pidOutput * k0 * p->curAv + fabs(p->curAv) * manager.k1 + p->pidOutput * k0 * p->pidOutput * k0 * manager.k2 +
//                           manager.k3 / static_cast<float>(4);
//
//             //计算底盘需要的总功率
//             sumCmdPower += cmdPower[i];
//             //计算单个电机转速误差，判断是否使用大P分配还是等比分配法
//             error[i] = fabs(p->setAv - p->curAv);
//
//             //由于功率是一个正标量，所以需要取绝对值来算总需求功率
//             if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
//             {
//                 allocatablePower += -cmdPower[i];
//             }
//             else
//             {
//                 //计算底盘电机总误差
//                 sumError += error[i];
//                 //计算底盘电机总功率需求
//                 sumPowerRequired += cmdPower[i];
//             }
//         }
//         //电机未连接处理
//         else if (motorDisconnectCounter[i] < 1000U)
//         {
//             cmdPower[i] = manager.motors[i]->getTorqueFeedback() * rpm2av(manager.motors[i]->getRPMFeedback()) +
//                           fabs(rpm2av(manager.motors[i]->getRPMFeedback())) * manager.k1 +
//                           manager.motors[i]->getTorqueFeedback() * manager.motors[i]->getTorqueFeedback() * manager.k2 + manager.k3 / 4.0f;
//             error[i] = 0.0f;
//         }
//         else
//         {
//             cmdPower[i] = 0.0f;
//             error[i]    = 0.0f;
//         }
//     }
//
//     powerStatus.maxPowerLimited          = maxPower; //最大功率限制
//     powerStatus.sumPowerCmd_before_clamp = sumCmdPower; //约束前的总功率
//
//     //如果底盘需要的功率大于最大功率
//     if (sumCmdPower > maxPower)
//     {
//         float errorConfidence;
//         if (sumError > error_powerDistribution_set)
//         {
//             //如果误差过大使用大P分配
//             errorConfidence = 1.0f;
//         }
//         else if (sumError > prop_powerDistribution_set)
//         {
//             //误差中等使用混合分配K = 0~1,具体K值由误差更接近哪一侧决定
//             errorConfidence =
//                 Utils::Math::clamp((sumError - prop_powerDistribution_set) / (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
//         }
//         else
//         {
//             //误差过小使用等比分配
//             errorConfidence = 0.0f;
//         }
//         //决定好分配方法后别等了开始分配吧
//         for (int i = 0; i < 4; i++)
//         {
//             PowerObj *p = objs[i];
//             //依旧检测电机是否连接成功
//             if (isMotorConnected(manager.motors[i]))
//             {
//                 //如果电机功率需求小于等于0则不调整，直接用输出值
//                 if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
//                 {
//                     newTorqueCurrent[i] = p->pidOutput;
//                     continue;
//                 }
//
//                 float powerWeight_Error = fabs(p->setAv - p->curAv) / sumError;//计算单个电机误差权重
//                 float powerWeight_Prop  = cmdPower[i] / sumPowerRequired;//计算单个电机功率需求权重
//                 float powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;//计算分配权重
//                 //t = (-B ± sqrt(▲)) / 2A  2A在下面判断方向的时候除
//                 float delta = p->curAv * p->curAv - 4.0f * manager.k2 * (manager.k1 * fabs(p->curAv) + manager.k3 / static_cast<float>(4) - powerWeight * allocatablePower);
//                 if (floatEqual(delta, 0.0f))  // repeat roots
//                 {
//                     newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
//                 }
//                 else if (delta > 0.0f)  // distinct roots
//                 {
//                     //根据pid输出决定输出电流方向以及计算对应电流
//                     newTorqueCurrent[i] = p->pidOutput > 0.0f ? (-p->curAv + sqrtf(delta)) / (2.0f * manager.k2) / k0
//                                                               : (-p->curAv - sqrtf(delta)) / (2.0f * manager.k2) / k0;
//                 }
//                 else  // imaginary roots
//                 {
//                     newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
//                 }
//                 //输出扭矩电流限幅
//                 newTorqueCurrent[i] = Utils::Math::clamp(newTorqueCurrent[i], p->pidMaxOutput);
//             }
//             else
//             {
//                 newTorqueCurrent[i] = 0.0f;
//             }
//         }
//     }
//     else
//     {
//         for (int i = 0; i < 4; i++)
//         {
//             if (isMotorConnected(manager.motors[i]))
//             {
//                 newTorqueCurrent[i] = objs[i]->pidOutput;
//             }
//             else
//             {
//                 newTorqueCurrent[i] = 0.0f;
//             }
//         }
//     }
//
// #if USE_DEBUG
//     newCmdPower = 0.0f;
//     for (int i = 0; i < 4; i++)
//     {
//         PowerObj *p = objs[i];
//         newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabs(p->curAv) * manager.k1 +
//                        newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * manager.k2 + manager.k3 / 4.0f;
//     }
// #endif
//
//     return newTorqueCurrent;//返回解算好的目标电流数组
//     //正常功率解算到此结束
// }
//
// //设置错误标志位
// static inline void setErrorFlag()
// {
//     /*Judge the error status*/
//     //使用超级电容
// #if USE_SUPER_CAPACITOR
//     if (not capStatus.isConnected || not capStatus.capacitorTx.enableDCDC || not capStatus.capacitorRx.errorCode == 0)
//         setErrorFlag(manager.error, Manager::CAPDisConnect);
//     else
//         clearErrorFlag(manager.error, Manager::CAPDisConnect);
// #else
//     setErrorFlag(manager.error, Manager::CAPDisConnect);
// #endif
//
//     //判断裁判系统是否连接
//     if (not RefereeSystem::isConnected())
//         setErrorFlag(manager.error, Manager::RefereeDisConnect);
//     else
//         clearErrorFlag(manager.error, Manager::RefereeDisConnect);
//
//     //判断电机是否连接
//     if (not isAllMotorConnected())
//         setErrorFlag(manager.error, Manager::MotorDisconnect);
//     else
//         clearErrorFlag(manager.error, Manager::MotorDisconnect);
// }
//
// //功率控制任务
// void powerDaemon [[noreturn]] (void *pvParam)
// {
//     static Matrixf<2, 1> samples;
//     static Matrixf<2, 1> params;
//     static float effectivePower = 0;
//
//     //获取电机扭矩常数 = 手册电机常数值×减速比
//     manager.torqueConst = manager.motors[0]->getKA() * manager.motors[0]->getReductionRatio();
//     //初始化标志位
//     isInitialized       = true;
//
//     vTaskDelay(1000);
//
//     //获取当前时间戳
//     manager.lastUpdateTick = xTaskGetTickCount();
//
//     while (true)
//     {
//         //判断是否出错
//         setErrorFlag();
//         TickType_t now = xTaskGetTickCount();
//
//         // update rls state and check whether cap energy is out even when cap disconnect to utilize credible data from referee system f0.or the rls
//         // model
//         // estimate the cap energy if cap disconnect
//         // estimated cap energy = cap energy feedback when cap is connected
// #if USE_SUPER_CAPACITOR
//         // If super capacitor is disconnected from the circuit, disable the rls update
//
//         /*****************************************此处代码是根据超级电容通信，裁判系统通信掉线与否计算超级电容电量剩余情况以及是否应该给超级电容充电**************************************/
//         // 如果超级电容掉线，则禁用RLS自动更新
//         if (isFlagged(manager.error, Manager::CAPDisConnect))
//
//
//         {
//             // Judge whether the cap energy is used-up
//             // 如果超级电容掉线，但是裁判系统在线
//             if (not isFlagged(manager.error, Manager::RefereeDisConnect))
//             {
//                 //如果裁判系统能量环小于60，说明缓冲能量已经被消耗，说明电容并非满电 && 裁判系统允许的底盘底盘功率 - 机器人当前使用功率 > 允许给超电充电的多余功率
//                 if (powerMessage.getData().bufferEnergy < MAX_POEWR_REFEREE_BUFF &&
//                     powerMessage.getData().chassisPower > CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD)
//                 {
//                     //可以给超级电容充电
//                     isCapEnergyOut             = true;
//                     //估计的超级电容能量为0，因为超电掉线了通信不了，直接先默认为09
//                     manager.estimatedCapEnergy = 0.0f;
//                 }
//                 else
//                 {
//                     //此时有两种情况
//                     //1.认为裁判系统能量没有被消耗（即超电充满）
//                     //2.裁判系统的功率只能勉强够机器人运动（裁判系统允许功率 - 机器人当前使用功率 < 允许给超电充电的多余功率CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD）
//
//                     isCapEnergyOut     = false;//关闭超电充电
//                     manager.rlsEnabled = Manager::Disable;//关闭rls自动调参
//                     if (powerMessage.getData().chassisPower < MIN_MAXPOWER_CONFIGURED && powerMessage.getData().bufferEnergy == 60U)//此时认为超电已经充满
//                     {
//                         manager.estimatedCapEnergy = 2100.0f;
//                     }
//                     else // 此时认为超电没充满，但是多余功率又没到达能够给超电充能的程度
//                     {
//                         // 按照一定计算方式计算当前超电电量
//                         manager.estimatedCapEnergy += (powerMessage.getData().chassisPower - manager.estimatedPower) *
//                                                       static_cast<float>((now - manager.lastUpdateTick) / configTICK_RATE_HZ);
//                         manager.estimatedCapEnergy = Utils::Math::clamp(manager.estimatedCapEnergy, 0.0f, 2100.0f);
//                     }
//                 }
//             }
//             else //要是裁判系统也掉线了
//             {
//                 //和上面else处理方法一样，但是无法再用裁判系统来判断超电是否充满了
//                 isCapEnergyOut     = false;
//                 manager.rlsEnabled = Manager::Disable;
//                 manager.estimatedCapEnergy += (CAP_OFFLINE_ENERGY_TARGET_POWER - manager.estimatedPower) *
//                                               static_cast<float>((now - manager.lastUpdateTick) / configTICK_RATE_HZ);
//                 manager.estimatedCapEnergy = Utils::Math::clamp(manager.estimatedCapEnergy, 0.0f, 2100.0f);
//             }
//         }
//         else // 如果超电没掉线，预测超级电容值直接等于超电回传的能量值
//         {
//             isCapEnergyOut             = false;
//             manager.estimatedCapEnergy = capStatus.capacitorRx.capEnergy / 255.0f * 2100.0f;
//         }
// #else  // Only Use Referee System, disable the rls update if referee data is invalid
//         if (isFlagged(manager.error, Manager::RefereeDisConnect))
//         {
//             manager.rlsEnabled = Manager::Disable;
//         }
//         isCapEnergyOut             = false;
//         manager.estimatedCapEnergy = 0.0f
// #endif
//
//         /****************************************此处代码是获取当前可用能量大小**************************************/
// #if USE_SUPER_CAPACITOR
//         //裁判系统缓冲能量和超级电容剩余能量是互补的
//         //这里优先使用超级电容的能量信息
//         if (not isFlagged(manager.error, Manager::CAPDisConnect))
//             manager.powerBuff = capStatus.capacitorRx.capEnergy;//使用超级电容剩余能量作为当前能量
//         else if (not isFlagged(manager.error, Manager::RefereeDisConnect))
//             manager.powerBuff = powerMessage.getData().bufferEnergy;//使用裁判系统缓冲能量作为当前能量
// #else
//                                      if (not isFlagged(manager.error, Manager::RefereeDisConnect))
//                                          manager.powerBuff = powerMessage.getData().bufferEnergy;
// #endif
// // Set the energy target based on the current error status
// #if USE_SUPER_CAPACITOR
//         /*****************************************此处代码是设置缓冲能量环的最大最小值**************************************/
//         //如果超级电容在线
//         if (not isFlagged(manager.error, Manager::CAPDisConnect))
//         {
//             manager.fullBuffSet = capFullBuffSet;//设置超级电容最大能量
//             manager.baseBuffSet = capBaseBuffSet;//设置超级电容能够被使用到的最低能量值
//         }
//         else
//         {
//             // 如果超级电容断线，则使用裁判系统的缓冲能量控制，参数和上面同理
//             manager.fullBuffSet = refereeFullBuffSet;
//             manager.baseBuffSet = refereeBaseBuffSet;
//         }
// #else
//         // Only Use Referee System
//         // if referee data is not valid, we do not enable the energy loop, so that we do not have to updating fullbuffset and basebuffset
//         manager.fullBuffSet = refereeFullBuffSet;
//         manager.baseBuffSet = refereeBaseBuffSet;
// #endif
//
//
//         /************************************这段是根据裁判系统和超电是否断线来根据不同策略给机器人可以使用的功率硬上限***************************************/
//
//         if (not isFlagged(manager.error, Manager::RefereeDisConnect))//如果裁判系统在线，根据裁判系统反馈等级采用不同的功率上限
//         {
//             //获取裁判系统允许使用的最大功率（1级步兵的功率也有45W，这里是何意味）
//             get_chassis_power_and_buffer(manager.refereeMaxPower, )
//             manager.refereeMaxPower = fmax(manager.refereeMaxPower, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
//             //如果裁判系统传回来的兵种等级大于10,则认为兵种是1级（没看懂，裁判系统应该不会传大于10级，因为10级满级，这里可能是认为裁判系统有时候会传错等级？）
//             if (robotMessage.getData().robot_level > 10U)
//                 LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
//             else //使用裁判系统传回来的兵种等级
//                 LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = fmax(1U, robotMessage.getData().robot_level);
// #if USE_SUPER_CAPACITOR
//             //如果超电通信断线
//             if (isFlagged(manager.error, Manager::CAPDisConnect))
//                 //机器人能使用的功率硬上限为裁判系统最大功率+裁判系统能量环计算输出的功率
//                 manager.powerUpperLimit = manager.refereeMaxPower + POWER_PD_KP * (sqrtf(refereeFullBuffSet) - sqrtf(refereeBaseBuffSet));
//             //如果超电通信在线
//             else
//                 //机器人能使用的功率硬上限为裁判系统最大功率+超电能量环最大输出的功率
//                 manager.powerUpperLimit = manager.refereeMaxPower + MAX_CAP_POWER_OUT;
// #else
//             manager.powerUpperLimit = manager.refereeMaxPower;
// #endif
//         }
//         else//如果裁判系统离线，使用更保守的功率控制策略
//         {
//             switch (manager.division)
//             {
//                 //裁判系统功率分配以当前基础再低一级
//             case Division::HERO:
//                 manager.refereeMaxPower = HeroChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U];
//                 break;
//             case Division::INFANTRY:
//                 manager.refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U];
//                 break;
//             case Division::SENTRY:
//                 manager.refereeMaxPower = SentryChassisPowerLimit;
//                 break;
//             default:
//                 configASSERT(0) break;
//             }
// // Since we have less available feedback, we constrain the power conservatively
// #if USE_SUPER_CAPACITOR
//             if (isFlagged(manager.error, Manager::CAPDisConnect)) //如果裁判系统和超电同时断线
//             {
//                 manager.powerUpperLimit = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE; //机器人能使用的功率硬上限为当前等级-1对应的裁判系统功率 * 保险系数
//             }
//             else //如裁判系统断线，超电在线
//             {
//                 manager.powerUpperLimit = manager.refereeMaxPower + MAX_CAP_POWER_OUT; //机器人能使用的功率硬上限为当前等级-1对应的裁判系统功率 + 超级电容最大功率输出
//             }
// #else
//             manager.powerUpperLimit = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
// #endif
//         }
//
//
//         /************************************这段是根据裁判系统和超电是否断线来根据不同策略给机器人可以使用的自动功率分配上下限***************************************/
//         MIN_MAXPOWER_CONFIGURED = manager.refereeMaxPower * 0.8f;
//         // 能量环计算
//         // 判断如果电容和裁判系统通信都断线（如果电容裁判系统都gg了，回去拉火车得了）
//         if (isFlagged(manager.error, Manager::CAPDisConnect) && isFlagged(manager.error, Manager::RefereeDisConnect))
//         {
//             //机器人最大可用功率为裁判系统最大功率*保险系数,防止超功率
//             manager.baseMaxPower = manager.fullMaxPower = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
//             //同时关闭两个能量环
//             powerPD_base.reset();
//             powerPD_full.reset();
//         }
//         else //只要裁判系统或者超电有一个没掉线，都使用能量环来控制功率(此处为所有都正常的时候最基本能量环控制功率方案，后续可以优化这里以及之前的功率分配系统)
//         {
//             manager.baseMaxPower =
//                 fmax(manager.refereeMaxPower - powerPD_base(sqrtf(manager.baseBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
//             manager.fullMaxPower =
//                 fmax(manager.refereeMaxPower - powerPD_full(sqrtf(manager.fullBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
//         }
//
//         // if user has self defined power curve, use it
//         // 如果有用户自定义的功率软上限，则使用用户自定义的,一般就是不在平地，过障碍的时候，或者你某种时候需要功率拉满的，就会用到用户自定义的功率软上限，此时不受超电自动分配功率控制
//         if (manager.callback != nullptr)
//             setMaxPowerConfigured(manager.callback());
//
//
//         /************************************这段是根据电机，裁判系统，超电来计算当前底盘功率*****************************************/
//         effectivePower = 0;
//         samples[0][0]  = 0;
//         samples[1][0]  = 0;
//         for (int i = 0; i < 4; i++)
//         {
//             //统计电机是否离线，离线的话统计离线时间，超过一定时间判断为电机真正离线
//             if (isMotorConnected(manager.motors[i]))
//             {
//                 motorDisconnectCounter[i] = 0U;
//             }
//             else
//             {
//                 motorDisconnectCounter[i]++;
//             }
//             if (motorDisconnectCounter[i] < 1000U)  // We consider motor that is just disconnected as still using power, by assuming the motor keep
//                                                     // latest output and rpm by 1 second, otherwise it is not safe if we only use energy loop
//             {
//                 effectivePower += manager.motors[i]->getTorqueFeedback() * rpm2av(manager.motors[i]->getRPMFeedback());
//                 samples[0][0] += fabsf(rpm2av(manager.motors[i]->getRPMFeedback()));
//                 samples[1][0] += manager.motors[i]->getTorqueFeedback() * manager.motors[i]->getTorqueFeedback();
//             }
//             else
//             {
//                 motorDisconnectCounter[i] = 1000U;
//             }
//         }
//         //计算电机估计功率
//         manager.estimatedPower = manager.k1 * samples[0][0] + manager.k2 * samples[1][0] + effectivePower + manager.k3;
//
// // Get the measured power from cap
// // If cap is disconnected, get measured power from referee feedback if cap energy is out
// // Otherwise, set it to estimated power
// #if USE_SUPER_CAPACITOR
//         if (not isFlagged(manager.error, Manager::CAPDisConnect))
//         {
//             // 获取超级电容测量功率
//             manager.measuredPower = capStatus.capacitorRx.chassisPower;
//         }
//         else if (not isFlagged(manager.error, Manager::RefereeDisConnect) && isCapEnergyOut)
//         {
//             // If the capacitor energy is used up, we could trust the data from the referee system
//             // 如果裁判系统在线且电容能量用尽，则使用裁判系统功率反馈
//             manager.measuredPower = powerMessage.getData().chassisPower;
//         }
//         else
//         {
//             // 如果裁判系统和超电都掉线，使用估计的功率控制，不使用能量环（回家拉火车）
//             manager.measuredPower = manager.estimatedPower;
//         }
// #else
//         if (not isFlagged(manager.error, Manager::RefereeDisConnect))
//         {
//             manager.measuredPower = powerMessage.getData().chassisPower;
//         }
//         else
//         {
//             manager.measuredPower = manager.estimatedPower;
//         }
// #endif
//
//         // update power status
//         powerStatus.userConfiguredMaxPower = manager.userConfiguredMaxPower;
//         powerStatus.effectivePower         = effectivePower;
//         powerStatus.powerLoss              = manager.measuredPower - effectivePower;
//         powerStatus.efficiency             = Utils::Math::clamp(effectivePower / manager.measuredPower, 0.0f, 1.0f);
//         powerStatus.estimatedCapEnergy     = static_cast<uint8_t>(manager.estimatedCapEnergy / 2100.0f * 255.0f);
//         powerStatus.error                  = static_cast<Manager::ErrorFlags>(manager.error);
//
//         // Update the RLS parameters AND
//         // Add dead zone AND
//         // The Referee System could not detect negative power, leading to failure of real measurement.
//         // So use estimated power to evaluate this situtation
//         // 开启参数自更新
//         if (manager.rlsEnabled == Manager::Enable && fabs(manager.measuredPower) > 5.0f &&
//             not(isFlagged(manager.error, Manager::CAPDisConnect) && manager.estimatedPower < 0))
//         {
//             params     = manager.rls.update(samples, manager.measuredPower - effectivePower - manager.k3);
//             manager.k1 = fmax(params[0][0], 1e-5f);  // In case the k1 diverge to negative number
//             manager.k2 = fmax(params[1][0], 1e-5f);  // In case the k2 diverge to negative number
//         }
//
//         manager.lastUpdateTick = now;
//
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }
//
// /**
//  * @implements
//  */
// void init(const Manager &mana)
// {
//     if (isInitialized)
//         return;
//     manager = mana;
//
//     // default value
//     LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = manager.division == Division::SENTRY ? 10U : 1U;
//     MIN_MAXPOWER_CONFIGURED           = 30.0f;
//     manager.powerUpperLimit           = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD + MAX_CAP_POWER_OUT;
//
//     Core::Communication::RefereeSystem::subscribeMessage(&powerMessage);
//     Core::Communication::RefereeSystem::subscribeMessage(&robotMessage);
//
//     xTaskCreateStatic(powerDaemon, "power", 1024, nullptr, 10, xPowerTaskStack, &uxPowerTaskTCB);
// }
//
// }  // namespace Power
// }  // namespace Control
// }  // namespace Core

