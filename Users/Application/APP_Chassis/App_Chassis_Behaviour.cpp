#include "App_Chassis_Behaviour.h"

#include "Alg_UserLib.h"
#include "arm_math.h"

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

static fp32 clamp_leg_length(fp32 leg_set)
{
    if (leg_set > CHASSIS_LEG_MAX)
    {
        return CHASSIS_LEG_MAX;
    }
    if (leg_set < 0.17f)
    {
        return 0.17f;
    }
    return leg_set;
}

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

static bool_t chassis_is_request_rising_edge(const Chassis_Move *chassis_move_mode, chassis_mode_e target_mode)
{
    return chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode == target_mode &&
           chassis_move_mode->last_request_mode != target_mode;
}

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
    case CHASSIS_MODE_UI_RESET:
    case CHASSIS_MODE_NORMAL:
    default:
        return CHASSIS_NORMAL;
    }
}

#if CHASSIS_BYPASS_INIT_MODE
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
        // 方向反转时，先用刹车斜坡减到0，再用加速斜坡进入反向目标，避免“反向起步过猛”。
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

static fp32 chassis_update_small_gyro_d_yaw(bool_t small_gyro_enable, bool_t hard_reset)
{
    static fp32 small_gyro_d_yaw = 0.0f;
    const fp32 target_d_yaw = clamp_abs_fp32(CHASSIS_SMALL_GYRO_D_YAW_SET, CHASSIS_D_YAW_MAX);

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

static void chassis_zero_force_control(fp32 *v_set, fp32 *d_yaw_set, fp32 *leg_set)
{
    *v_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_LEG_MAX;
}

static fp32 chassis_follow_yaw_control(const fp32 target_relative_yaw,
                                       Chassis_Move *chassis_move_rc_to_vector)
{
    const fp32 current_relative_yaw = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle;
    const fp32 body_yaw_err = shortest_angle_error(target_relative_yaw, current_relative_yaw);

    return PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, body_yaw_err);
}

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

static void chassis_normal_control(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                   Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    fp32 target_vx = 0;

    if (target_vx > 0.0f)
    {
        target_vx = clamp_abs_fp32(chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp,
                                   CHASSIS_KEY_MAX_SPEED);
    }
    else
    {
        target_vx = clamp_abs_fp32(chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp,
                                   CHASSIS_KEY_BACK_MAX_SPEED);
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

static void chassis_init_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    *vx_set = 0.0f;
    *d_yaw_set = 0.0f;

    if (chassis_move_rc_to_vector->init_phase == CHASSIS_INIT_RETRACT)
    {
        chassis_move_rc_to_vector->chassis_leg_filter_set.num[0] = CHASSIS_INIT_RETRACT_LEG_NUM;
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, CHASSIS_INIT_RETRACT_LEG_TARGET);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
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
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_JUMP_AIRBORNE_TARGET,
                                           CHASSIS_JUMP_AIRBORNE_LEG_RAMP_SPEED);
        break;
    case CHASSIS_JUMP_DONE:
    default:
        *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector,
                                           CHASSIS_NORMAL_LEG_TARGET,
                                           CHASSIS_LEG_STEP_RAMP_SPEED);
        break;
    }
}

void Chassis_Behaviour_Mode_Set(Chassis_Move *chassis_move_mode)
{
    if (chassis_move_mode == NULL || chassis_move_mode->chassis_gimbal_data == NULL)
    {
        return;
    }

    const chassis_mode_e requested_mode = chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode;
    const bool_t step1_edge = chassis_is_request_rising_edge(chassis_move_mode, CHASSIS_MODE_STEP_1);
    const bool_t step2_edge = chassis_is_request_rising_edge(chassis_move_mode, CHASSIS_MODE_STEP_2);
    const bool_t jump_edge = chassis_is_request_rising_edge(chassis_move_mode, CHASSIS_MODE_JUMP);

    if (requested_mode == CHASSIS_MODE_NO_FORCE ||
        requested_mode == CHASSIS_MODE_RESERVED ||
        requested_mode == CHASSIS_MODE_RESERVED_2)
    {
        chassis_move_mode->pending_state = CHASSIS_NORMAL;
        chassis_move_mode->state = CHASSIS_STOP;
        chassis_move_mode->init_phase = CHASSIS_INIT_FOLD;
        chassis_move_mode->jump_phase = CHASSIS_JUMP_DONE;
    }
    else
    {
        switch (chassis_move_mode->state)
        {
        case CHASSIS_STOP:
            if (requested_mode == CHASSIS_MODE_NORMAL ||
                requested_mode == CHASSIS_MODE_UI_RESET)
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

        case CHASSIS_NORMAL:
            if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_1;
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_2;
            }
            else if (jump_edge)
            {
                chassis_move_mode->state = CHASSIS_JUMP;
            }
            break;

        case CHASSIS_LEG_1:
            if (jump_edge)
            {
                chassis_move_mode->state = CHASSIS_JUMP;
            }
            else if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_2;
            }
            break;

        case CHASSIS_LEG_2:
            if (jump_edge)
            {
                chassis_move_mode->state = CHASSIS_JUMP;
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_1;
            }
            break;

        case CHASSIS_JUMP:
            if (chassis_move_mode->jump_phase == CHASSIS_JUMP_DONE)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            break;

        default:
            chassis_move_mode->state = CHASSIS_STOP;
            break;
        }
    }

    chassis_move_mode->last_request_mode = requested_mode;
}

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
            chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                   chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        else
        {
            chassis_update_small_gyro_d_yaw(0, 1);
            chassis_action_hold_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        break;

    case CHASSIS_LEG_1:
        if (protocol_valid)
        {
            chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                   chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        }
        else
        {
            chassis_update_small_gyro_d_yaw(0, 1);
            chassis_action_hold_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        }
        break;

    case CHASSIS_LEG_2:
        if (protocol_valid)
        {
            chassis_normal_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                   chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        }
        else
        {
            chassis_update_small_gyro_d_yaw(0, 1);
            chassis_action_hold_control(vx_set, yaw_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        }
        break;

    case CHASSIS_JUMP:
        chassis_update_small_gyro_d_yaw(0, 1);
        chassis_jump_control(vx_set, yaw_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        break;

    default:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        chassis_update_small_gyro_d_yaw(0, 1);
        break;
    }
}
