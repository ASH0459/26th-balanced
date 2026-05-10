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
     * 在 NORMAL / LEG_1 / LEG_2 状态下生效，但上台阶活跃阶段
     *（CONTACT / RETRACT / STAND / EXTEND 等）跳过检测，
     * DETECT / DONE 阶段仍正常检测。任一腿 |theta_l| 超过阈值时
     * 将 state 切换到 INIT（posture DOWN 时走 FLIP）。
     */
    bool_t chassis_theta_loss_of_control_check(Chassis_Move *chassis_move);

    /**
     * @brief  检测 yaw 轴持续跟踪失败并释放 yaw 跟踪。
     * @param  chassis_move: 底盘控制结构体指针
     * @retval 1 = 触发了释放，0 = 正常
     *
     * 当 |chassis_yaw_err| 持续超过阈值达一定周期后，
     * 冻结 yaw：清零 yaw_err 和 d_yaw_set，保持 yaw_set 不变。
     * 适用于底盘被卡住或被撞击导致 yaw 无法跟踪的场景。
     */
    bool_t chassis_yaw_stuck_check(Chassis_Move *chassis_move);

    /**
     * @brief  检测卡墙角/障碍物导致的"推墙"正反馈并切断。
     * @param  chassis_move: 底盘控制结构体指针
     * @retval 1 = 触发了切断，0 = 正常
     *
     * 核心判据：(|theta_l_avg| > 阈值 或 |pitch| > 阈值) 且 |v_filter| ≈ 0。
     * theta_l 反映腿-机体相对角，pitch 反映机体绝对俯仰角，任一超限 +
     * v_filter ≈ 0 即判定被卡住。触发后清零 v_set 和位移积分 x_set 并进入冷却期。
     */
    bool_t chassis_stuck_push_check(Chassis_Move *chassis_move);

#ifdef __cplusplus
}
#endif

#endif
