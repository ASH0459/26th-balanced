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

static bool_t chassis_is_request_rising_edge(const Chassis_Move *chassis_move_mode, chassis_mode_e target_mode)
{
    return chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode == target_mode &&
           chassis_move_mode->last_request_mode != target_mode;
}

static void chassis_zero_force_control(fp32 *v_set, fp32 *d_yaw_set, fp32 *leg_set)
{
    *v_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_LEG_MAX;
}

static void chassis_follow_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, fp32 *yaw_set,
                                   Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    const fp32 target_vx = clamp_abs_fp32(chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp, CHASSIS_KEY_MAX_SPEED);
    const fp32 target_yaw_angle = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_yaw_set;
    const fp32 current_yaw_angle = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle;
    const fp32 angle_error = shortest_angle_error(target_yaw_angle, current_yaw_angle);
    const fp32 target_d_yaw = PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, -angle_error);

    static ramp_function_source_t vx_ramp;
    static uint8_t ramp_init_flag = 0;
    if (ramp_init_flag == 0)
    {
        ramp_init(&vx_ramp, 0.002f, CHASSIS_KEY_MAX_SPEED, -CHASSIS_KEY_MAX_SPEED);
        ramp_init_flag = 1;
    }

    if (fabsf(target_vx) < 1e-6f)
    {
        vx_ramp.out = 0.0f;
    }
    else if (vx_ramp.out < target_vx)
    {
        ramp_calc(&vx_ramp, CHASSIS_KEY_ACCEL);
        if (vx_ramp.out > target_vx)
        {
            vx_ramp.out = target_vx;
        }
    }
    else if (vx_ramp.out > target_vx)
    {
        ramp_calc(&vx_ramp, -CHASSIS_KEY_ACCEL);
        if (vx_ramp.out < target_vx)
        {
            vx_ramp.out = target_vx;
        }
    }

    *vx_set = vx_ramp.out;
    *d_yaw_set = target_d_yaw;
    *leg_set = clamp_leg_length(target_leg_length);
    *yaw_set = angle_error;
}

static void chassis_init_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    *vx_set = 0.0f;
    *d_yaw_set = 0.0f;

    if (chassis_move_rc_to_vector->init_phase == CHASSIS_INIT_RETRACT)
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, CHASSIS_JUMP_PRELOAD_TARGET);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
    else if (chassis_move_rc_to_vector->init_phase == CHASSIS_INIT_STAND)
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, CHASSIS_NORMAL_LEG_TARGET);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
    else
    {
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
}

static void chassis_jump_control(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set,
                                 Chassis_Move *chassis_move_rc_to_vector)
{
    *vx_set = 0.0f;
    *yaw_set = 0.0f;
    *d_yaw_set = 0.0f;

    switch (chassis_move_rc_to_vector->jump_phase)
    {
    case CHASSIS_JUMP_PRELOAD:
        *leg_set = CHASSIS_JUMP_PRELOAD_TARGET;
        break;
    case CHASSIS_JUMP_TAKEOFF:
        *leg_set = CHASSIS_JUMP_TAKEOFF_TARGET;
        break;
    case CHASSIS_JUMP_AIRBORNE:
        *leg_set = CHASSIS_JUMP_TUCK_TARGET;
        break;
    case CHASSIS_JUMP_LAND:
    case CHASSIS_JUMP_DONE:
    default:
        *leg_set = CHASSIS_JUMP_LAND_TARGET;
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

    if (requested_mode == CHASSIS_MODE_NO_FORCE)
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
            if (requested_mode == CHASSIS_MODE_NORMAL)
            {
                chassis_move_mode->pending_state = CHASSIS_NORMAL;
                chassis_move_mode->state = (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN) ? CHASSIS_FLIP : CHASSIS_INIT;
            }
            break;

        case CHASSIS_FLIP:
            if (chassis_move_mode->posture == CHASSIS_POSTURE_UP)
            {
                chassis_move_mode->state = CHASSIS_INIT;
            }
            break;

        case CHASSIS_INIT:
            if (chassis_move_mode->posture == CHASSIS_POSTURE_DOWN)
            {
                chassis_move_mode->state = CHASSIS_FLIP;
            }
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
            if (step1_edge)
            {
                chassis_move_mode->state = CHASSIS_NORMAL;
            }
            else if (step2_edge)
            {
                chassis_move_mode->state = CHASSIS_LEG_2;
            }
            break;

        case CHASSIS_LEG_2:
            if (step2_edge)
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

    switch (chassis_move_rc_to_vector->state)
    {
    case CHASSIS_STOP:
    case CHASSIS_FLIP:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        break;

    case CHASSIS_INIT:
        chassis_init_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        break;

    case CHASSIS_NORMAL:
        chassis_follow_control(vx_set, d_yaw_set, leg_set, yaw_set, chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        break;

    case CHASSIS_LEG_1:
        chassis_follow_control(vx_set, d_yaw_set, leg_set, yaw_set, chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        break;

    case CHASSIS_LEG_2:
        chassis_follow_control(vx_set, d_yaw_set, leg_set, yaw_set, chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        break;

    case CHASSIS_JUMP:
        chassis_jump_control(vx_set, yaw_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        break;

    default:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        break;
    }
}
