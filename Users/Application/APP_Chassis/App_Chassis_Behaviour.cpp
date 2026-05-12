#include "App_Chassis_Behaviour.h"
#include "App_Chassis_Safety.h"
#include "App_Chassis_Task_Helpers.h"

#include "Alg_UserLib.h"
#include "arm_math.h"
#include "UC_Referee.h"

// 按对称限幅裁剪标量。
static fp32 clamp_abs_fp32(fp32 value, fp32 limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

// 斜坡函数：current 向 target 以 max_step 的速率变化，不能超过 target。
static fp32 chassis_ramp_to_target(fp32 current, fp32 target, fp32 max_step)
{
    if (max_step <= 0.0f)
    {
        return target;
    }

    if (current < target)
    {
        current += max_step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= max_step;
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

// 将角度包到 [-pi, pi]。
static fp32 wrap_to_pi(fp32 angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

// 将腿长约束到机械允许范围。
static fp32 clamp_leg_length(fp32 leg_set)
{
    if (leg_set > CHASSIS_LEG_MAX)
    {
        return CHASSIS_LEG_MAX;
    }
    if (leg_set < CHASSIS_LEG_MIN)
    {
        return CHASSIS_LEG_MIN;
    }
    return leg_set;
}

// 按斜坡更新腿长目标，并缓存到 leg_filter_set.out。
static fp32 chassis_ramp_leg_target(Chassis_Move *chassis_move, fp32 target_leg_length, fp32 ramp_speed)
{
    const fp32 target = clamp_leg_length(target_leg_length);
    fp32 current = clamp_leg_length(chassis_move->chassis_leg_filter_set.out);
    const fp32 max_step = fabsf(ramp_speed) * CHASSIS_CONTROL_TIME;
    const fp32 error = target - current;

    if (max_step <= 1e-6f || fabsf(error) <= max_step)
    {
        current = target;
    }
    else if (error > 0.0f)
    {
        current += max_step;
    }
    else
    {
        current -= max_step;
    }

    chassis_move->chassis_leg_filter_set.out = current;
    return current;
}

// 检测某个行为模式请求的上升沿。
static bool_t chassis_is_request_rising_edge(const Chassis_Move *chassis_move_mode, chassis_mode_e target_mode)
{
    return chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode == target_mode &&
           chassis_move_mode->last_request_mode != target_mode;
}

// 将外部行为模式映射成底盘内部待切换状态。
static Chassis_State_e chassis_requested_mode_to_pending_state(chassis_mode_e requested_mode)
{
    switch (requested_mode)
    {
    case CHASSIS_MODE_STEP_1:
        return CHASSIS_LEG_1;
    case CHASSIS_MODE_STEP_2:
        return CHASSIS_LEG_2;
    case CHASSIS_MODE_JUMP:
        return CHASSIS_JUMP;
    case CHASSIS_MODE_NORMAL:
    default:
        return CHASSIS_NORMAL;
    }
}

#if CHASSIS_BYPASS_INIT_MODE
// 绕过 INIT 时，只允许切到可运行的常规状态。
static Chassis_State_e chassis_sanitize_pending_state(Chassis_State_e pending_state)
{
    if (pending_state == CHASSIS_LEG_1 ||
        pending_state == CHASSIS_LEG_2 ||
        pending_state == CHASSIS_NORMAL)
    {
        return pending_state;
    }
    return CHASSIS_NORMAL;
}
#endif

// 计算 vx 加速阶段本拍允许增长的步长。
static fp32 chassis_calc_vx_accel_step(fp32 current_vx, fp32 target_vx)
{
    const fp32 accel_base_step = CHASSIS_DIRECTION_VX_ACCEL * CHASSIS_CONTROL_TIME;
    const fp32 abs_target = fabsf(target_vx);

    if (accel_base_step <= 1e-6f || abs_target <= 1e-6f)
    {
        return accel_base_step;
    }

    fp32 progress = fabsf(current_vx) / abs_target;
    progress = float_constrain(progress, 0.0f, 1.0f);

    const fp32 fast_gain = float_constrain(CHASSIS_DIRECTION_VX_ACCEL_FAST_GAIN, 0.0f, 10.0f);
    const fp32 slow_gain = float_constrain(CHASSIS_DIRECTION_VX_ACCEL_SLOW_GAIN, 0.0f, 10.0f);
    const fp32 gain = slow_gain + (1.0f - progress) * (fast_gain - slow_gain);

    return accel_base_step * gain;
}

// 更新 vx 斜坡，负责起步、刹车和反向切换。
static fp32 chassis_update_vx_ramp(fp32 target_vx, bool_t hard_reset)
{
    static ramp_function_source_t vx_ramp;
    static uint8_t ramp_init_flag = 0;

    if (ramp_init_flag == 0)
    {
        ramp_init(&vx_ramp, CHASSIS_CONTROL_TIME, CHASSIS_KEY_MAX_SPEED, -CHASSIS_KEY_MAX_SPEED);
        ramp_init_flag = 1;
    }

    const fp32 target = clamp_abs_fp32(target_vx, CHASSIS_KEY_MAX_SPEED);
    const fp32 brake_step = CHASSIS_DIRECTION_VX_BRAKE_ACCEL * CHASSIS_CONTROL_TIME;
    const fp32 current = vx_ramp.out;

    if (hard_reset)
    {
        vx_ramp.out = target;
        return vx_ramp.out;
    }

    if (fabsf(target) < 1e-6f)
    {
        vx_ramp.out = chassis_ramp_to_target(vx_ramp.out, 0.0f, brake_step);
    }
    else if (current * target < 0.0f)
    {
        // 方向反转时，先用刹车斜坡减到0，再用加速斜坡进入反向目标，避免“反向起步过猛”.
        vx_ramp.out = chassis_ramp_to_target(current, 0.0f, brake_step);

        if (fabsf(vx_ramp.out) <= 1e-6f)
        {
            const fp32 accel_step = chassis_calc_vx_accel_step(vx_ramp.out, target);
            vx_ramp.out = chassis_ramp_to_target(vx_ramp.out, target, accel_step);
        }
    }
    else
    {
        const fp32 step = (fabsf(target) < fabsf(current)) ? brake_step : chassis_calc_vx_accel_step(current, target);
        vx_ramp.out = chassis_ramp_to_target(vx_ramp.out, target, step);

        if (fabsf(vx_ramp.out - target) <= 1e-6f)
        {
            vx_ramp.out = target;
        }
    }

    return vx_ramp.out;
}

// 更新小陀螺目标角速度，必要时硬复位内部状态。
static fp32 chassis_update_small_gyro_d_yaw(bool_t small_gyro_enable, bool_t hard_reset)
{
    static fp32 small_gyro_d_yaw = 0.0f;
    fp32 chassis_small_gyro_d_yaw_set = robot_state.chassis_power_limit * CHASSIS_POWER_D_YAW_RATE;
    const fp32 target_d_yaw = clamp_abs_fp32(chassis_small_gyro_d_yaw_set, CHASSIS_D_YAW_MAX);

    if (hard_reset)
    {
        small_gyro_d_yaw = 0.0f;
        return small_gyro_d_yaw;
    }

    const fp32 ramp_rate = fabsf(small_gyro_enable ? CHASSIS_SMALL_GYRO_RAMP_UP_RATE
                                                   : CHASSIS_SMALL_GYRO_RAMP_DOWN_RATE);
    const fp32 max_step = ramp_rate * CHASSIS_CONTROL_TIME;
    const fp32 target = small_gyro_enable ? target_d_yaw : 0.0f;

    small_gyro_d_yaw = chassis_ramp_to_target(small_gyro_d_yaw, target, max_step);
    return small_gyro_d_yaw;
}

// 停机态输出：速度和角速度清零，腿长回 normal。
static void chassis_zero_force_control(fp32 *v_set, fp32 *d_yaw_set, fp32 *leg_set)
{
    *v_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_NORMAL_LEG_TARGET;
}

// 根据云台相对角生成底盘 yaw 跟随输出。
static fp32 chassis_follow_yaw_control(const fp32 target_relative_yaw,
                                       Chassis_Move *chassis_move_rc_to_vector)
{
    const fp32 current_relative_yaw = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle;
    const fp32 body_yaw_err = shortest_angle_error(target_relative_yaw, current_relative_yaw);

    return PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, body_yaw_err);
}

// 保持态输出：速度回零，保持 yaw 跟随，腿长向目标缓慢逼近。
static void chassis_action_hold_control(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                        Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    *vx_set = chassis_update_vx_ramp(0.0f, 0);
    const fp32 yaw_target = wrap_to_pi(chassis_move_rc_to_vector->chassis_gimbal_data->yaw_set);
    const fp32 yaw_pid = chassis_follow_yaw_control(yaw_target, chassis_move_rc_to_vector);
    *yaw_set = yaw_target;
    *d_yaw_set = clamp_abs_fp32(yaw_pid, CHASSIS_D_YAW_MAX);
    *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector, target_leg_length, CHASSIS_LEG_STEP_RAMP_SPEED);
}

// 正常运动输出：处理线速度、yaw 跟随/小陀螺和目标腿长。
static void chassis_normal_control(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                   Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    const fp32 raw_target_vx = chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp;
    fp32 target_vx;

    if (chassis_move_rc_to_vector->state == CHASSIS_LEG_2 || chassis_move_rc_to_vector->state == CHASSIS_LEG_1)
    {
        // 腿长最长时限速到 0.9m/s
        const fp32 leg_avg = 0.5f * (chassis_move_rc_to_vector->chassis_left_control.wbr_control.L +
                                      chassis_move_rc_to_vector->chassis_right_control.wbr_control.L);
        const fp32 speed_limit = (leg_avg >= CHASSIS_LEG_MAX - 0.01f) ? 0.9f : CHASSIS_LEG_MAX_SPEED;
        target_vx = clamp_abs_fp32(raw_target_vx, speed_limit);
    }
    else
    {
        if (raw_target_vx > 0.0f)
        {
            target_vx = clamp_abs_fp32(raw_target_vx, CHASSIS_KEY_MAX_SPEED);
        }
        else
        {
            target_vx = clamp_abs_fp32(raw_target_vx, CHASSIS_KEY_BACK_MAX_SPEED);
        }
    }

    //*vx_set = chassis_update_vx_ramp(target_vx, 0);
    *vx_set = target_vx;

    const bool_t small_gyro_enable =
        (chassis_move_rc_to_vector->chassis_gimbal_data != NULL) &&
        (chassis_move_rc_to_vector->chassis_gimbal_data->gyro_enable != 0U);
    const fp32 small_gyro_d_yaw = chassis_update_small_gyro_d_yaw(small_gyro_enable, 0);

    if (small_gyro_enable)
    {
        // 小陀螺模式：锁定当前相对角，使用斜坡后的角速度旋转。
        *yaw_set = wrap_to_pi(chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle);
        *d_yaw_set = small_gyro_d_yaw;
    }
    else
    {
        const fp32 yaw_target = wrap_to_pi(chassis_move_rc_to_vector->chassis_gimbal_data->yaw_set);
        const fp32 yaw_pid = chassis_follow_yaw_control(yaw_target, chassis_move_rc_to_vector);
        *yaw_set = yaw_target;
        *d_yaw_set = clamp_abs_fp32(yaw_pid + small_gyro_d_yaw, CHASSIS_D_YAW_MAX);
    }
    *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector, target_leg_length, CHASSIS_LEG_STEP_RAMP_SPEED);
}

// INIT 行为输出：按 init_phase 决定腿长目标，速度和 yaw 保持为零。
static void chassis_init_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    *vx_set = 0.0f;
    *d_yaw_set = 0.0f;

    if (chassis_move_rc_to_vector->init_phase == CHASSIS_INIT_RETRACT)
    {
        // chassis_move_rc_to_vector->chassis_leg_filter_set.num[0] = CHASSIS_INIT_RETRACT_LEG_NUM;
        // first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, CHASSIS_INIT_RETRACT_LEG_TARGET);
        // *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;

        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                               CHASSIS_INIT_RETRACT_LEG_TARGET,
                                               CHASSIS_LEG_STEP_RAMP_SPEED);
    }
    else if (chassis_move_rc_to_vector->init_phase == CHASSIS_INIT_STAND)
    {
        chassis_move_rc_to_vector->chassis_leg_filter_set.num[0] = CHASSIS_ACCEL_LEG_NUM;
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, CHASSIS_NORMAL_LEG_TARGET);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
    else if (chassis_move_rc_to_vector->init_phase == CHASSIS_INIT_FOLD)
    {
        chassis_move_rc_to_vector->chassis_leg_filter_set.num[0] = CHASSIS_ACCEL_LEG_NUM;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_LEG_MAX,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
    }
    else
    {
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
}


static void chassis_jump_control(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                 Chassis_Move *chassis_move_rc_to_vector)
{
    *vx_set = chassis_update_vx_ramp(0.0f, 0);
    const fp32 yaw_target = wrap_to_pi(chassis_move_rc_to_vector->chassis_gimbal_data->yaw_set);
    const fp32 yaw_pid = chassis_follow_yaw_control(yaw_target, chassis_move_rc_to_vector);
    *yaw_set = yaw_target;
    *d_yaw_set = clamp_abs_fp32(yaw_pid, CHASSIS_D_YAW_MAX);

    switch (chassis_move_rc_to_vector->jump_phase)
    {
    case CHASSIS_JUMP_PRELOAD:
        *leg_set = CHASSIS_NORMAL_LEG_TARGET;
        break;
    case CHASSIS_JUMP_TAKEOFF:
        chassis_move_rc_to_vector->chassis_leg_filter_set.out = CHASSIS_JUMP_TAKEOFF_TARGET;
        *leg_set = CHASSIS_JUMP_TAKEOFF_TARGET;
        break;
    case CHASSIS_JUMP_READYLAND:
        // 空中瞬间收到最短腿长，不用斜坡，避免空中伸腿
        chassis_move_rc_to_vector->chassis_leg_filter_set.out = CHASSIS_LEG_MIN;
        chassis_move_rc_to_vector->chassis_left_leg_set = CHASSIS_LEG_MIN;
        chassis_move_rc_to_vector->chassis_right_leg_set = CHASSIS_LEG_MIN;
        *leg_set = CHASSIS_LEG_MIN;
        break;
    case CHASSIS_JUMP_DONE:
    default:
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;
    }
}

// LEG_1/LEG_2 下的上台阶行为输出：按 step_up_phase 分派速度和腿长策略。
static void chassis_control_leg_step_up(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                        Chassis_Move *chassis_move_rc_to_vector, fp32 default_leg_target)
{
    const Chassis_StepUp_Phase_e phase = chassis_move_rc_to_vector->step_up_phase;
    const fp32 yaw_target = wrap_to_pi(chassis_move_rc_to_vector->chassis_gimbal_data->yaw_set);
    const fp32 yaw_pid = chassis_follow_yaw_control(yaw_target, chassis_move_rc_to_vector);
    *yaw_set = yaw_target;
    *d_yaw_set = clamp_abs_fp32(yaw_pid, CHASSIS_D_YAW_MAX);

    switch (phase)
    {
    // 第一级台阶
    case STEP_UP_DETECT:
    case STEP_UP_DONE:
        // 正常受控，按默认 LEG 目标输出
        chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                               chassis_move_rc_to_vector, default_leg_target);
        break;

    case STEP_UP_CONTACT:
        // 撞台阶：速度归零，腿长维持当前 LEG 目标
        *vx_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           default_leg_target,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;

    case STEP_UP_RETRACT:
        // 收腿到最小腿长
        *vx_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;

    case STEP_UP_STAND:
        // 站稳，速度回零
        *vx_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;

    case STEP_UP_EXTEND:
        // 伸腿，正常遥控速度 + 固定前向偏移，x_set 由 task 层冻结不积分
        chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                               chassis_move_rc_to_vector, STEP_UP_EXTEND_LEG_TARGET);
        *vx_set += STEP_UP_EXTEND_VX_OFFSET;
        break;

    // 第二级台阶
    case STEP_UP_DETECT_2ND:
        // 检测第二级，正常受控，腿长维持当前目标
        if (default_leg_target == CHASSIS_LEG_2_TARGET)
        {
            chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                               chassis_move_rc_to_vector, STEP_UP_EXTEND_LEG_TARGET);
        }
        else if (default_leg_target == CHASSIS_LEG_1_TARGET)
        {
            chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                               chassis_move_rc_to_vector, default_leg_target);
        }

        break;

    case STEP_UP_CONTACT_2ND:
        // 撞第二级：速度归零，腿长维持当前目标
        *vx_set = 0.0f;
        if (default_leg_target == CHASSIS_LEG_2_TARGET)
        {
            *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                               STEP_UP_EXTEND_LEG_TARGET,
                                               CHASSIS_LEG_STEP_RAMP_SPEED);
        }
        else if (default_leg_target == CHASSIS_LEG_1_TARGET)
        {
            *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                               default_leg_target,
                                               CHASSIS_LEG_STEP_RAMP_SPEED);
        }
        break;

    case STEP_UP_RETRACT_2ND:
        // 收腿
        *vx_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;

    case STEP_UP_STAND_2ND:
        // 站稳
        *vx_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;

    default:
        chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                               chassis_move_rc_to_vector, default_leg_target);
        break;
    }
}

// LEG_1_STEP_DOWN 下的下台阶行为输出：按 step_down_phase 分派速度和腿长策略。
static void chassis_control_leg_step_down(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                          Chassis_Move *chassis_move_rc_to_vector)
{
    const Chassis_StepDown_Phase_e phase = chassis_move_rc_to_vector->step_down_phase;
    const fp32 yaw_target = wrap_to_pi(chassis_move_rc_to_vector->chassis_gimbal_data->yaw_set);
    const fp32 yaw_pid = chassis_follow_yaw_control(yaw_target, chassis_move_rc_to_vector);
    *yaw_set = yaw_target;
    *d_yaw_set = clamp_abs_fp32(yaw_pid, CHASSIS_D_YAW_MAX);

    switch (phase)
    {
    case STEP_DOWN_FREEFALL:
        // 取消 Tbl + 收腿到正常腿长
        *vx_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;

    case STEP_DOWN_LANDING:
        // 离地检测阶段：速度归零，维持短腿
        *vx_set = 0.0f;
        *leg_set = CHASSIS_NORMAL_LEG_TARGET;
        break;

    case STEP_DOWN_DONE:
    default:
        // 完成：正常控制，恢复腿长
        chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                               chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        break;
    }
}

// 更新底盘行为状态机：根据外部请求和当前姿态决定 state / pending_state。
void Chassis_Behaviour_Mode_Set(Chassis_Move *chassis_move_mode)
{
    if (chassis_move_mode == NULL || chassis_move_mode->chassis_gimbal_data == NULL)
    {
        return;
    }

    // theta_l 失控保护：优先于一切外部请求
    if (chassis_theta_loss_of_control_check(chassis_move_mode))
    {
        chassis_move_mode->last_request_mode = chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode;
        return;
    }

    const chassis_mode_e requested_mode = chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode;
    const bool_t step1_edge = chassis_is_request_rising_edge(chassis_move_mode, CHASSIS_MODE_STEP_1);
    const bool_t step2_edge = chassis_is_request_rising_edge(chassis_move_mode, CHASSIS_MODE_STEP_2);
    const bool_t jump_edge = chassis_is_request_rising_edge(chassis_move_mode, CHASSIS_MODE_JUMP);

    if (requested_mode == CHASSIS_MODE_NO_FORCE)
    {
        if (chassis_move_mode->chassis_gimbal_data->chassis_reset)
        {
            NVIC_SystemReset();
        }
        chassis_move_mode->pending_state = CHASSIS_NORMAL;
        chassis_move_mode->state = CHASSIS_STOP;
        chassis_move_mode->init_phase = CHASSIS_INIT_FOLD;
        chassis_move_mode->jump_phase = CHASSIS_JUMP_DONE;
        chassis_move_mode->reserved_angle_init_done = 0;
    }
    else if (requested_mode == CHASSIS_MODE_RESERVED)
    {
        // RESERVED 模式：趴地模式，不走 LQR，用 PID 控制速度和腿角度。
        chassis_move_mode->pending_state = CHASSIS_NORMAL;
        chassis_move_mode->state = CHASSIS_RESERVED;
        chassis_move_mode->init_phase = CHASSIS_INIT_FOLD;
        chassis_move_mode->jump_phase = CHASSIS_JUMP_DONE;
    }
    else
    {
        switch (chassis_move_mode->state)
        {
        case CHASSIS_STOP:
            if (requested_mode == CHASSIS_MODE_NORMAL)
            {
                chassis_move_mode->pending_state = chassis_requested_mode_to_pending_state(requested_mode);
#if CHASSIS_BYPASS_INIT_MODE
                if (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN)
                {
                    chassis_move_mode->state = CHASSIS_FLIP;
                }
                else
                {
                    chassis_move_mode->state = chassis_sanitize_pending_state(chassis_move_mode->pending_state);
                    chassis_move_mode->pending_state = CHASSIS_NORMAL;
                }
#else
                chassis_move_mode->state = (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN) ? CHASSIS_FLIP : CHASSIS_INIT;
#endif
            }
            break;

        case CHASSIS_FLIP:
            if (chassis_move_mode->posture == CHASSIS_POSTURE_UP)
            {
#if CHASSIS_BYPASS_INIT_MODE
                chassis_move_mode->state = chassis_sanitize_pending_state(chassis_move_mode->pending_state);
                chassis_move_mode->pending_state = CHASSIS_NORMAL;
#else
                chassis_move_mode->state = CHASSIS_INIT;
#endif
            }
            break;

        case CHASSIS_INIT:
            if (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN)
            {
                chassis_move_mode->state = CHASSIS_FLIP;
            }
#if CHASSIS_BYPASS_INIT_MODE
            else
            {
                chassis_move_mode->state = chassis_sanitize_pending_state(chassis_move_mode->pending_state);
                chassis_move_mode->pending_state = CHASSIS_NORMAL;
            }
#else
            else if (chassis_move_mode->init_phase == CHASSIS_INIT_DONE)
            {
                chassis_move_mode->state = chassis_move_mode->pending_state;
                if (chassis_move_mode->state != CHASSIS_NORMAL &&
                    chassis_move_mode->state != CHASSIS_LEG_1 &&
                    chassis_move_mode->state != CHASSIS_LEG_2)
                {
                    chassis_move_mode->state = CHASSIS_NORMAL;
                }
                chassis_move_mode->pending_state = CHASSIS_NORMAL;
            }
#endif
            break;

        case CHASSIS_RESERVED:
            // 从 RESERVED 切到其他模式时，走 INIT/FLIP 流程起身。
            chassis_move_mode->pending_state = chassis_requested_mode_to_pending_state(requested_mode);
#if CHASSIS_BYPASS_INIT_MODE
            if (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN)
            {
                chassis_move_mode->state = CHASSIS_FLIP;
            }
            else
            {
                chassis_move_mode->state = chassis_sanitize_pending_state(chassis_move_mode->pending_state);
                chassis_move_mode->pending_state = CHASSIS_NORMAL;
            }
#else
            chassis_move_mode->state = (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN) ? CHASSIS_FLIP : CHASSIS_INIT;
#endif
            break;

        case CHASSIS_NORMAL:
            if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_1;
                chassis_move_mode->step_up_phase = STEP_UP_DONE; // LEG_1 不做上台阶，由下台阶检测接管
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_2;
                chassis_move_mode->step_up_phase = STEP_UP_DETECT; // 高腿长进一级检测
            }
            else if (jump_edge)
            {
                chassis_move_mode->state = CHASSIS_JUMP;
            }
            break;

        case CHASSIS_LEG_1:
            if (chassis_move_mode->step_up_phase == STEP_UP_DONE)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (chassis_move_mode->step_up_phase != STEP_UP_DETECT &&
                     chassis_move_mode->step_up_phase != STEP_UP_DETECT_2ND &&
                     chassis_move_mode->step_up_phase != STEP_UP_DONE)
            {
                // 上台阶进行中，阻塞模式切换
            }
            else if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_2;
                chassis_move_mode->step_up_phase = STEP_UP_DETECT;
            }
            break;

        case CHASSIS_LEG_2:
            if (chassis_move_mode->step_up_phase == STEP_UP_DONE)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (chassis_move_mode->step_up_phase != STEP_UP_DETECT &&
                     chassis_move_mode->step_up_phase != STEP_UP_DETECT_2ND &&
                     chassis_move_mode->step_up_phase != STEP_UP_DONE)
            {
                // 上台阶进行中，阻塞模式切换
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_1;
                chassis_move_mode->step_up_phase = STEP_UP_DONE;
            }
            break;

        case CHASSIS_JUMP:
            if (chassis_move_mode->jump_phase == CHASSIS_JUMP_DONE)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            break;

        case CHASSIS_LEG_1_STEP_DOWN:
            if (chassis_move_mode->step_down_phase == STEP_DOWN_DONE)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (chassis_move_mode->step_down_phase != STEP_DOWN_FREEFALL &&
                     chassis_move_mode->step_down_phase != STEP_DOWN_LANDING)
            {
                // 未知阶段，回 NORMAL
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            // FREEFALL / LANDING 进行中：阻塞模式切换（step1/step2 不响应）
            break;

        default:
            chassis_move_mode->state = CHASSIS_STOP;
            break;
        }
    }

    chassis_move_mode->last_request_mode = requested_mode;
}

// RESERVED 模式行为层输出：根据标志位生成速度、转向和腿长目标，腿角度由控制循环处理。
static void chassis_reserved_behaviour_control(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                                Chassis_Move *chassis_move_rc_to_vector)
{
    const uint16_t flags = chassis_move_rc_to_vector->chassis_gimbal_data->reserved_flags;
    const uint8_t hi = static_cast<uint8_t>(flags >> 8);   // Byte0
    const uint8_t lo = static_cast<uint8_t>(flags & 0xFF); // Byte1

    // ---- 高字节 bit0/1: 前进/后退 ----
    fp32 target_vx = 0.0f;
    if ((hi & 0x01U) != 0U)
    {
        target_vx = CHASSIS_RESERVED_FWD_SPEED;
    }
    else if ((hi & 0x02U) != 0U)
    {
        target_vx = -CHASSIS_RESERVED_BWD_SPEED;
    }
    *vx_set = target_vx;

    // ---- 低字节 bit0/1: 左转/右转（差速转向）----
    fp32 turn = 0.0f;
    if ((lo & 0x01U) != 0U)
    {
        turn = CHASSIS_RESERVED_TURN_SPEED;
    }
    else if ((lo & 0x02U) != 0U)
    {
        turn = -CHASSIS_RESERVED_TURN_SPEED;
    }
    *d_yaw_set = turn;

    // ---- 低字节 bit6/7: 左腿伸长/缩短 ----
    fp32 left_leg_offset = 0.0f;
    if ((lo & 0x40U) != 0U)
    {
        left_leg_offset = CHASSIS_RESERVED_LEG_INC_STEP;
    }
    else if ((lo & 0x80U) != 0U)
    {
        left_leg_offset = -CHASSIS_RESERVED_LEG_INC_STEP;
    }

    // ---- 低字节 bit2/3: 右腿伸长/缩短 ----
    fp32 right_leg_offset = 0.0f;
    if ((lo & 0x04U) != 0U)
    {
        right_leg_offset = CHASSIS_RESERVED_LEG_INC_STEP;
    }
    else if ((lo & 0x08U) != 0U)
    {
        right_leg_offset = -CHASSIS_RESERVED_LEG_INC_STEP;
    }

    if (left_leg_offset != 0.0f)
    {
        chassis_move_rc_to_vector->chassis_left_leg_set =
            clamp_leg_length(chassis_move_rc_to_vector->chassis_left_leg_set + left_leg_offset);
    }
    else
    {
        chassis_move_rc_to_vector->chassis_left_leg_set =
            clamp_leg_length(chassis_move_rc_to_vector->chassis_left_control.wbr_control.L);
    }

    if (right_leg_offset != 0.0f)
    {
        chassis_move_rc_to_vector->chassis_right_leg_set =
            clamp_leg_length(chassis_move_rc_to_vector->chassis_right_leg_set + right_leg_offset);
    }
    else
    {
        chassis_move_rc_to_vector->chassis_right_leg_set =
            clamp_leg_length(chassis_move_rc_to_vector->chassis_right_control.wbr_control.L);
    }

    // RESERVED 模式下左右腿独立控制，chassis_leg_set 保留 0，控制循环只读 left/right_leg_set。
    *leg_set = 0.0f;

    *yaw_set = 0.0f;
}

// 根据当前 state 生成本拍行为层目标：vx_set / yaw_set / d_yaw_set / leg_set。.
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                   Chassis_Move *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || yaw_set == NULL || d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = 0.0f;
    *yaw_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_NORMAL_LEG_TARGET;

    const chassis_mode_e requested_mode =
        (chassis_move_rc_to_vector->chassis_gimbal_data != NULL)
            ? chassis_move_rc_to_vector->chassis_gimbal_data->chassis_behaviour_mode
            : CHASSIS_MODE_NO_FORCE;
    const bool_t protocol_valid =
        (chassis_move_rc_to_vector->chassis_gimbal_data != NULL) &&
        (chassis_move_rc_to_vector->chassis_gimbal_data->protocol_valid != 0U);

    switch (chassis_move_rc_to_vector->state)
    {
    case CHASSIS_STOP:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        chassis_update_vx_ramp(0.0f, 1);
        chassis_update_small_gyro_d_yaw(0, 1);
        break;

    case CHASSIS_FLIP:
        *vx_set = 0.0f;
        *d_yaw_set = 0.0f;
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_LEG_MAX,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        chassis_update_vx_ramp(0.0f, 1);
        chassis_update_small_gyro_d_yaw(0, 1);
        break;

    case CHASSIS_INIT:
        chassis_init_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        chassis_update_vx_ramp(0.0f, 1);
        chassis_update_small_gyro_d_yaw(0, 1);
        break;

    case CHASSIS_NORMAL:
        if (requested_mode == CHASSIS_MODE_NORMAL && protocol_valid)
        {
            // 正常模式：协议有效时按 normal control 输出。
            chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                   chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        else
        {
            // 退出正常控制：清小陀螺并退回 hold。
            chassis_update_small_gyro_d_yaw(0, 1);
            chassis_action_hold_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        break;
    case CHASSIS_JUMP:
        chassis_jump_control(vx_set, yaw_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        break;
    case CHASSIS_LEG_1_STEP_DOWN:
        if (protocol_valid)
        {
            chassis_control_leg_step_down(vx_set, yaw_set, d_yaw_set, leg_set,
                                          chassis_move_rc_to_vector);
        }
        else
        {
            // 协议无效：回 NORMAL
            chassis_move_rc_to_vector->step_down_phase = STEP_DOWN_DONE;
            chassis_move_rc_to_vector->state = CHASSIS_NORMAL;
        }
        break;
    case CHASSIS_LEG_1:
        if (protocol_valid)
        {
            chassis_control_leg_step_up(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        }
        else
        {
            // 协议无效：保持姿态和 LEG_1 目标腿长。
            chassis_update_small_gyro_d_yaw(0, 1);
            chassis_action_hold_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        }
        break;

    case CHASSIS_LEG_2:
        if (protocol_valid)
        {
            chassis_control_leg_step_up(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        }
        else
        {
            // 协议无效：保持姿态和 LEG_2 目标腿长。
            chassis_update_small_gyro_d_yaw(0, 1);
            chassis_action_hold_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        }
        break;

    case CHASSIS_RESERVED:
        chassis_update_small_gyro_d_yaw(0, 1);
        chassis_reserved_behaviour_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                           chassis_move_rc_to_vector);
        break;

    default:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        chassis_update_small_gyro_d_yaw(0, 1);
        break;
    }
}
