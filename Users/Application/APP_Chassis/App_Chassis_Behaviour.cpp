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

static constexpr fp32 CHASSIS_SMALL_GYRO_D_YAW_MAX = 1000.0f;

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

static void chassis_zero_force_control(fp32 *v_set, fp32 *d_yaw_set, fp32 *leg_set)
{
    *v_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_LEG_MAX;
}

static void chassis_follow_yaw_control(fp32 *body_yaw_err_set, fp32 *d_yaw_set,
                                       Chassis_Move *chassis_move_rc_to_vector)
{
    const fp32 target_relative_yaw = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_yaw_set;
    const fp32 current_relative_yaw = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle;
    const fp32 body_yaw_err = shortest_angle_error(target_relative_yaw, current_relative_yaw);

    *body_yaw_err_set = body_yaw_err;
    *d_yaw_set = PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, -body_yaw_err);
}

static void chassis_follow_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, fp32 *body_yaw_err_set,
                                   Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    const fp32 target_vx = clamp_abs_fp32(chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp, CHASSIS_KEY_MAX_SPEED);

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
    chassis_follow_yaw_control(body_yaw_err_set, d_yaw_set, chassis_move_rc_to_vector);
    *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector, target_leg_length, CHASSIS_LEG_STEP_RAMP_SPEED);
}

static void chassis_small_gyro_control(fp32 *vx_set, fp32 *body_yaw_err_set, fp32 *d_yaw_set, fp32 *leg_set,
                                       Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    // 小陀螺只按角速度控制，不再用云台相对角做角度 PID。
    // CAN 接收层为了普通角度模式会把 turn_set 取反存入 chassis_yaw_set；
    // 在小陀螺模式下 turn_set 表示角速度命令，所以这里取反还原云台端发送的方向。
    const fp32 target_d_yaw = -chassis_move_rc_to_vector->chassis_gimbal_data->chassis_yaw_set;

    *vx_set = 0.0f;
    *body_yaw_err_set = 0.0f;
    *d_yaw_set = clamp_abs_fp32(target_d_yaw, CHASSIS_SMALL_GYRO_D_YAW_MAX);
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
    else
    {
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
}

static void chassis_jump_control(fp32 *vx_set, fp32 *body_yaw_err_set, fp32 *d_yaw_set, fp32 *leg_set,
                                 Chassis_Move *chassis_move_rc_to_vector)
{
    *vx_set = 0.0f;
    chassis_follow_yaw_control(body_yaw_err_set, d_yaw_set, chassis_move_rc_to_vector);

    switch (chassis_move_rc_to_vector->jump_phase)
    {
    case CHASSIS_JUMP_PRELOAD:
        *leg_set = CHASSIS_NORMAL_LEG_TARGET;
        break;
    case CHASSIS_JUMP_TAKEOFF:
        chassis_move_rc_to_vector->chassis_leg_filter_set.out = CHASSIS_JUMP_TAKEOFF_TARGET;
        *leg_set = CHASSIS_JUMP_TAKEOFF_TARGET;
        break;
    case CHASSIS_JUMP_AIRBORNE:
    case CHASSIS_JUMP_LAND:
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

    if (requested_mode == CHASSIS_MODE_NO_FORCE || requested_mode == CHASSIS_MODE_RESERVED)
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
            if (requested_mode == CHASSIS_MODE_NORMAL || requested_mode == CHASSIS_MODE_SMALL_GYRO)
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

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *body_yaw_err_set, fp32 *d_yaw_set, fp32 *leg_set,
                                   Chassis_Move *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || body_yaw_err_set == NULL || d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = 0.0f;
    *body_yaw_err_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_NORMAL_LEG_TARGET;

    const bool_t small_gyro_requested =
        chassis_move_rc_to_vector->chassis_gimbal_data != NULL &&
        chassis_move_rc_to_vector->chassis_gimbal_data->chassis_behaviour_mode == CHASSIS_MODE_SMALL_GYRO;

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
        if (small_gyro_requested)
        {
            chassis_small_gyro_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                       chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        else
        {
            chassis_follow_control(vx_set, d_yaw_set, leg_set, body_yaw_err_set,
                                   chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        break;

    case CHASSIS_LEG_1:
        if (small_gyro_requested)
        {
            chassis_small_gyro_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                       chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        }
        else
        {
            chassis_follow_control(vx_set, d_yaw_set, leg_set, body_yaw_err_set,
                                   chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        }
        break;

    case CHASSIS_LEG_2:
        if (small_gyro_requested)
        {
            chassis_small_gyro_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                       chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        }
        else
        {
            chassis_follow_control(vx_set, d_yaw_set, leg_set, body_yaw_err_set,
                                   chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        }
        break;

    case CHASSIS_JUMP:
        chassis_jump_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        break;

    default:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        break;
    }
}
