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
static constexpr fp32 CHASSIS_SMALL_GYRO_RAMP_UP_RATE = 24.0f;
static constexpr fp32 CHASSIS_SMALL_GYRO_RAMP_DOWN_RATE = 18.0f;
static constexpr fp32 CHASSIS_DIRECTION_EPSILON = 1e-6f;

typedef enum
{
    CHASSIS_SMALL_GYRO_DISABLED = 0,
    CHASSIS_SMALL_GYRO_RAMP_UP,
    CHASSIS_SMALL_GYRO_ACTIVE,
    CHASSIS_SMALL_GYRO_RAMP_DOWN,
} chassis_small_gyro_state_e;

typedef struct
{
    fp32 heading;
    fp32 speed_scale;
    bool_t moving;
} chassis_motion_intent_t;

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

static void chassis_direction_to_vector(chassis_direction_intent_e direction_intent, fp32 *x, fp32 *y)
{
    *x = 0.0f;
    *y = 0.0f;

    switch (direction_intent)
    {
    case CHASSIS_DIRECTION_FRONT:
        *x = 1.0f;
        break;
    case CHASSIS_DIRECTION_BACK:
        *x = -1.0f;
        break;
    case CHASSIS_DIRECTION_LEFT:
        *y = 1.0f;
        break;
    case CHASSIS_DIRECTION_RIGHT:
        *y = -1.0f;
        break;
    case CHASSIS_DIRECTION_FRONT_LEFT:
        *x = 1.0f;
        *y = 1.0f;
        break;
    case CHASSIS_DIRECTION_FRONT_RIGHT:
        *x = 1.0f;
        *y = -1.0f;
        break;
    case CHASSIS_DIRECTION_BACK_LEFT:
        *x = -1.0f;
        *y = 1.0f;
        break;
    case CHASSIS_DIRECTION_BACK_RIGHT:
        *x = -1.0f;
        *y = -1.0f;
        break;
    case CHASSIS_DIRECTION_STOP:
    default:
        break;
    }
}

static chassis_motion_intent_t chassis_build_motion_intent(chassis_direction_intent_e direction_intent)
{
    chassis_motion_intent_t intent{};

    fp32 raw_x = 0.0f;
    fp32 raw_y = 0.0f;
    chassis_direction_to_vector(direction_intent, &raw_x, &raw_y);

    const fp32 norm = sqrtf(raw_x * raw_x + raw_y * raw_y);
    if (norm <= CHASSIS_DIRECTION_EPSILON)
    {
        intent.heading = 0.0f;
        intent.speed_scale = 0.0f;
        intent.moving = 0;
        return intent;
    }

    const fp32 unit_x = raw_x / norm;
    const fp32 unit_y = raw_y / norm;

    intent.heading = atan2f(unit_y, unit_x);
    intent.speed_scale = sqrtf(unit_x * unit_x + unit_y * unit_y);
    intent.moving = 1;
    return intent;
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

    if (hard_reset)
    {
        vx_ramp.out = target;
        return vx_ramp.out;
    }

    if (fabsf(target) < 1e-6f)
    {
        vx_ramp.out = chassis_ramp_to_target(vx_ramp.out, 0.0f, CHASSIS_KEY_ACCEL * CHASSIS_CONTROL_TIME);
    }
    else if (vx_ramp.out < target)
    {
        ramp_calc(&vx_ramp, CHASSIS_KEY_ACCEL);
        if (vx_ramp.out > target)
        {
            vx_ramp.out = target;
        }
    }
    else if (vx_ramp.out > target)
    {
        ramp_calc(&vx_ramp, -CHASSIS_KEY_ACCEL);
        if (vx_ramp.out < target)
        {
            vx_ramp.out = target;
        }
    }

    return vx_ramp.out;
}

static fp32 chassis_calc_spin_target(fp32 vx_set)
{
    fp32 spin_target = CHASSIS_SPIN_MAIN_SPEED - CHASSIS_SPIN_LOW_SEN * fabsf(vx_set);
    if (spin_target < CHASSIS_SPIN_LOW_SPEED)
    {
        spin_target = CHASSIS_SPIN_LOW_SPEED;
    }
    return clamp_abs_fp32(spin_target, CHASSIS_SMALL_GYRO_D_YAW_MAX);
}

static fp32 chassis_update_small_gyro_state_machine(bool_t gyro_enable, fp32 spin_target, bool_t hard_reset,
                                                    Chassis_Move *chassis_move)
{
    static fp32 spin_output = 0.0f;
    static chassis_small_gyro_state_e gyro_state = CHASSIS_SMALL_GYRO_DISABLED;

    if (hard_reset)
    {
        spin_output = 0.0f;
        gyro_state = CHASSIS_SMALL_GYRO_DISABLED;
        chassis_move->chassis_spin_change_sen = 0.0f;
        return 0.0f;
    }

    const fp32 target = gyro_enable ? spin_target : 0.0f;
    const fp32 ramp_rate = (fabsf(target) > fabsf(spin_output)) ? CHASSIS_SMALL_GYRO_RAMP_UP_RATE : CHASSIS_SMALL_GYRO_RAMP_DOWN_RATE;
    spin_output = chassis_ramp_to_target(spin_output, target, ramp_rate * CHASSIS_CONTROL_TIME);

    if (fabsf(spin_output) <= 1e-3f)
    {
        gyro_state = CHASSIS_SMALL_GYRO_DISABLED;
    }
    else if (gyro_enable)
    {
        gyro_state = (fabsf(spin_output - target) <= 1e-3f) ? CHASSIS_SMALL_GYRO_ACTIVE : CHASSIS_SMALL_GYRO_RAMP_UP;
    }
    else
    {
        gyro_state = CHASSIS_SMALL_GYRO_RAMP_DOWN;
    }

    (void)gyro_state;
    chassis_move->chassis_spin_change_sen = fabsf(spin_output);
    return spin_output;
}

static void chassis_zero_force_control(fp32 *v_set, fp32 *d_yaw_set, fp32 *leg_set)
{
    *v_set = 0.0f;
    *d_yaw_set = 0.0f;
    *leg_set = CHASSIS_LEG_MAX;
}

static fp32 chassis_follow_yaw_control(const fp32 target_relative_yaw, fp32 *body_yaw_err_set,
                                       Chassis_Move *chassis_move_rc_to_vector)
{
    const fp32 current_relative_yaw = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle;
    const fp32 body_yaw_err = shortest_angle_error(target_relative_yaw, current_relative_yaw);

    *body_yaw_err_set = body_yaw_err;
    return PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, -body_yaw_err);
}

static void chassis_action_hold_control(fp32 *vx_set, fp32 *body_yaw_err_set, fp32 *d_yaw_set, fp32 *leg_set,
                                        Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    *vx_set = chassis_update_vx_ramp(0.0f, 0);
    const fp32 yaw_pid = chassis_follow_yaw_control(0.0f, body_yaw_err_set, chassis_move_rc_to_vector);
    const fp32 spin_cmd = chassis_update_small_gyro_state_machine(0, 0.0f, 0, chassis_move_rc_to_vector);
    *d_yaw_set = clamp_abs_fp32(yaw_pid + spin_cmd, CHASSIS_SMALL_GYRO_D_YAW_MAX);
    *leg_set = chassis_ramp_leg_target(chassis_move_rc_to_vector, target_leg_length, CHASSIS_LEG_STEP_RAMP_SPEED);
}

static void chassis_normal_control(fp32 *vx_set, fp32 *body_yaw_err_set, fp32 *d_yaw_set, fp32 *leg_set,
                                   Chassis_Move *chassis_move_rc_to_vector, fp32 target_leg_length)
{
    const chassis_motion_intent_t motion_intent =
        chassis_build_motion_intent(chassis_move_rc_to_vector->chassis_gimbal_data->direction_intent);
    const fp32 speed_amplitude = fabsf(clamp_abs_fp32(chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp,
                                                      CHASSIS_KEY_MAX_SPEED));
    const fp32 target_vx = speed_amplitude * motion_intent.speed_scale;

    *vx_set = chassis_update_vx_ramp(target_vx, 0);

    const bool_t gyro_requested = chassis_move_rc_to_vector->chassis_gimbal_data->gyro_enable == 1U;
    fp32 yaw_pid = 0.0f;
    fp32 yaw_err = 0.0f;

    if (!gyro_requested)
    {
        yaw_pid = chassis_follow_yaw_control(motion_intent.heading, &yaw_err, chassis_move_rc_to_vector);
    }

    const fp32 spin_target = chassis_calc_spin_target(*vx_set);
    const fp32 spin_cmd = chassis_update_small_gyro_state_machine(gyro_requested, spin_target, 0, chassis_move_rc_to_vector);

    *body_yaw_err_set = gyro_requested ? 0.0f : yaw_err;
    *d_yaw_set = clamp_abs_fp32(yaw_pid + spin_cmd, CHASSIS_SMALL_GYRO_D_YAW_MAX);
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
    *vx_set = chassis_update_vx_ramp(0.0f, 0);
    const fp32 yaw_pid = chassis_follow_yaw_control(0.0f, body_yaw_err_set, chassis_move_rc_to_vector);
    const fp32 spin_cmd = chassis_update_small_gyro_state_machine(0, 0.0f, 0, chassis_move_rc_to_vector);
    *d_yaw_set = clamp_abs_fp32(yaw_pid + spin_cmd, CHASSIS_SMALL_GYRO_D_YAW_MAX);

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
            if (requested_mode == CHASSIS_MODE_NORMAL ||
                requested_mode == CHASSIS_MODE_STEP_1 ||
                requested_mode == CHASSIS_MODE_STEP_2 ||
                requested_mode == CHASSIS_MODE_JUMP ||
                requested_mode == CHASSIS_MODE_UI_RESET)
            {
                chassis_move_mode->pending_state = chassis_requested_mode_to_pending_state(requested_mode);
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
    case CHASSIS_FLIP:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        chassis_update_vx_ramp(0.0f, 1);
        (void)chassis_update_small_gyro_state_machine(0, 0.0f, 1, chassis_move_rc_to_vector);
        break;

    case CHASSIS_INIT:
        chassis_init_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        chassis_update_vx_ramp(0.0f, 1);
        (void)chassis_update_small_gyro_state_machine(0, 0.0f, 1, chassis_move_rc_to_vector);
        break;

    case CHASSIS_NORMAL:
        if (requested_mode == CHASSIS_MODE_NORMAL && protocol_valid)
        {
            chassis_normal_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                   chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        else
        {
            chassis_action_hold_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                        chassis_move_rc_to_vector, CHASSIS_NORMAL_LEG_TARGET);
        }
        break;

    case CHASSIS_LEG_1:
        chassis_action_hold_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                    chassis_move_rc_to_vector, CHASSIS_LEG_1_TARGET);
        break;

    case CHASSIS_LEG_2:
        chassis_action_hold_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set,
                                    chassis_move_rc_to_vector, CHASSIS_LEG_2_TARGET);
        break;

    case CHASSIS_JUMP:
        chassis_jump_control(vx_set, body_yaw_err_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
        break;

    default:
        chassis_zero_force_control(vx_set, d_yaw_set, leg_set);
        break;
    }
}
