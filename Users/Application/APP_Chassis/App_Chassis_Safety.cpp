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

    // 上台阶活跃阶段（CONTACT / RETRACT / STAND / EXTEND 及二级对应阶段）
    // theta_l 会正常大幅度变化，跳过检测避免误触发。
    // DETECT / DETECT_2ND / DONE 阶段仍正常检测。
    {
        const Chassis_StepUp_Phase_e phase = chassis_move->step_up_phase;
        if (phase == STEP_UP_CONTACT || phase == STEP_UP_RETRACT ||
            phase == STEP_UP_STAND || phase == STEP_UP_EXTEND ||
            phase == STEP_UP_CONTACT_2ND || phase == STEP_UP_RETRACT_2ND ||
            phase == STEP_UP_STAND_2ND)
        {
            return 0;
        }
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
#define CHASSIS_YAW_STUCK_ERR_THRESHOLD 0.35f

// 持续超限周期数（@ 1kHz），超过则释放 yaw 跟踪。
#define CHASSIS_YAW_STUCK_TICKS 300U

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
            // 重置 yaw_set 为当前相对角，清零误差和角速度
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

// ---- 卡墙角/障碍物检测 ----

// 卡住判定：|v_filter| 接近 0（机体无法位移），且满足以下任一条件：
//   - |theta_l_avg| 超阈值（腿-机体相对角偏离）
//   - |pitch| 超阈值（机体绝对俯仰角偏离）
//   - |x_err| = |x_set - x_filter| 超阈值（位置误差积分累积）
// 正常平衡时身体倾斜会伴随速度变化；卡住时 v_filter ≈ 0 且角度/位置误差持续不消散。
// 覆盖：遥控前推+卡墙、遥控不动+身体被推歪+卡住。
#define CHASSIS_STUCK_ANGLE_THRESHOLD 0.2f
#define CHASSIS_STUCK_V_FILTER_THRESHOLD 0.15f
#define CHASSIS_STUCK_X_ERR_THRESHOLD 3.0f

// 持续超限周期数（@ 1kHz），超过则触发释放。
#define CHASSIS_STUCK_TICKS 500U

// 触发后冷却周期数，期间持续清零 v_set，暂时忽略遥控输入以打破正反馈。
#define CHASSIS_STUCK_COOLDOWN_TICKS 500U

bool_t chassis_stuck_push_check(Chassis_Move *chassis_move)
{
    if (chassis_move == NULL)
    {
        return 0;
    }

    static uint32_t stuck_ticks = 0;
    static uint32_t cooldown_ticks = 0;

    if (!chassis_is_balancing_state(chassis_move->state))
    {
        stuck_ticks = 0;
        cooldown_ticks = 0;
        return 0;
    }

    // 上台阶活跃阶段 skip，同 theta_l 检测
    {
        const Chassis_StepUp_Phase_e phase = chassis_move->step_up_phase;
        if (phase == STEP_UP_CONTACT || phase == STEP_UP_RETRACT ||
            phase == STEP_UP_STAND || phase == STEP_UP_EXTEND ||
            phase == STEP_UP_CONTACT_2ND || phase == STEP_UP_RETRACT_2ND ||
            phase == STEP_UP_STAND_2ND)
        {
            stuck_ticks = 0;
            cooldown_ticks = 0;
            return 0;
        }
    }

    // 冷却期内持续清零 v_set 和位移积分，避免刚释放又触发
    if (cooldown_ticks > 0U)
    {
        cooldown_ticks--;
        chassis_move->chassis_v_set = 0.0f;
        chassis_move->chassis_x_set = chassis_move->x_filter;
        return 1;
    }

    const fp32 theta_l_avg = 0.5f * (chassis_move->chassis_left_control.theta_l +
                                      chassis_move->chassis_right_control.theta_l);
    const fp32 pitch = chassis_move->chassis_pitch;
    const fp32 v_actual = chassis_move->v_filter;
    const fp32 x_err = chassis_move->chassis_x_set - chassis_move->x_filter;

    // 无法位移 +（角度偏离 或 位置误差累积）= 被卡住
    const bool_t stuck_condition =
        (fabsf(v_actual) < CHASSIS_STUCK_V_FILTER_THRESHOLD) &&
        ((fabsf(theta_l_avg) > CHASSIS_STUCK_ANGLE_THRESHOLD) ||
         (fabsf(pitch) > CHASSIS_STUCK_ANGLE_THRESHOLD) ||
         (fabsf(x_err) > CHASSIS_STUCK_X_ERR_THRESHOLD));

    if (stuck_condition)
    {
        stuck_ticks++;
        if (stuck_ticks >= CHASSIS_STUCK_TICKS)
        {
            chassis_move->chassis_v_set = 0.0f;
            chassis_move->chassis_x_set = chassis_move->x_filter;
            cooldown_ticks = CHASSIS_STUCK_COOLDOWN_TICKS;
            stuck_ticks = 0;
            return 1;
        }
    }
    else
    {
        stuck_ticks = 0;
    }

    return 0;
}
