#ifndef INFANTRY_ROBOT_APP_CHASSIS_SAFETY_H
#define INFANTRY_ROBOT_APP_CHASSIS_SAFETY_H

#include "App_Chassis_Task.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief  检测 theta_l 失控并触发 INIT/FLIP 恢复。
     * @param  chassis_move: 底盘控制结构体指针
     * @retval 1 = 触发了恢复，0 = 正常
     *
     * 当处于平衡状态（NORMAL/LEG_1/LEG_2）且任一腿 |theta_l| 超过阈值时，
     * 将 state 切换到 INIT（posture DOWN 时走 FLIP）。
     */
    bool_t chassis_theta_loss_of_control_check(Chassis_Move *chassis_move);

    /**
     * @brief  检测 yaw 轴持续跟踪失败并释放 yaw 跟踪。
     * @param  chassis_move: 底盘控制结构体指针
     * @retval 1 = 触发了释放，0 = 正常
     *
     * 当 |chassis_yaw_err| 持续超过阈值达一定周期后，
     * 将 chassis_yaw_set 重置为当前 chassis_yaw，消除误差源。
     * 适用于底盘被卡住或被撞击导致 yaw 无法跟踪的场景。
     */
    bool_t chassis_yaw_stuck_check(Chassis_Move *chassis_move);

#ifdef __cplusplus
}
#endif

#endif
