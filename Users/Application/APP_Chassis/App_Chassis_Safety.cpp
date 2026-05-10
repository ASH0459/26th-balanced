#include "App_Chassis_Safety.h"
#include "App_Chassis_Task_Helpers.h"
#include "math.h"

// theta_l 失控阈值 (rad)，任一腿超过则触发 INIT/FLIP 恢复。
#define CHASSIS_THETA_LOSS_THRESHOLD 1.0f

bool_t chassis_theta_loss_of_control_check(Chassis_Move *chassis_move)
{
    if (chassis_move == NULL)
    {
        return 0;
    }

    if (!chassis_is_balancing_state(chassis_move->state))
    {
        return 0;
    }

    if (fabsf(chassis_move->chassis_left_control.theta_l) <= CHASSIS_THETA_LOSS_THRESHOLD &&
        fabsf(chassis_move->chassis_right_control.theta_l) <= CHASSIS_THETA_LOSS_THRESHOLD)
    {
        return 0;
    }

    // theta_l 失控，触发恢复
    chassis_move->state = (chassis_move->posture == CHASSIS_POSTURE_DOWN)
                              ? CHASSIS_FLIP
                              : CHASSIS_INIT;
    chassis_move->init_phase = CHASSIS_INIT_FOLD;
    chassis_move->pending_state = CHASSIS_NORMAL;

    return 1;
}

// ---- yaw 轴持续跟踪失败检测 ----

// yaw 误差阈值 (rad)，超过此值视为跟踪异常。
#define CHASSIS_YAW_STUCK_ERR_THRESHOLD 0.5f

// 持续超限周期数（@ 1kHz），超过则释放 yaw 跟踪。
#define CHASSIS_YAW_STUCK_TICKS 500U

bool_t chassis_yaw_stuck_check(Chassis_Move *chassis_move)
{
    if (chassis_move == NULL)
    {
        return 0;
    }

    static uint32_t yaw_stuck_ticks = 0;

    if (!chassis_is_balancing_state(chassis_move->state))
    {
        yaw_stuck_ticks = 0;
        return 0;
    }

    if (fabsf(chassis_move->chassis_yaw_err) > CHASSIS_YAW_STUCK_ERR_THRESHOLD)
    {
        yaw_stuck_ticks++;
        if (yaw_stuck_ticks >= CHASSIS_YAW_STUCK_TICKS)
        {
            // 释放 yaw 跟踪：将 set 重置为当前角度，消除误差源
            chassis_move->chassis_yaw_set = chassis_move->chassis_yaw;
            chassis_move->chassis_yaw_err = 0.0f;
            chassis_move->chassis_d_yaw_set = 0.0f;
            yaw_stuck_ticks = 0;
            return 1;
        }
    }
    else
    {
        yaw_stuck_ticks = 0;
    }

    return 0;
}
