#include "App_Chassis_Task_Helpers.h"

#include <math.h>
#include <string.h>

#include "INS_Task.h"
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

bool_t chassis_init_legs_retracted(const Chassis_Move *chassis_move_control_loop) {
    if (chassis_move_control_loop == NULL) {
        return 0;
    }

    return (chassis_move_control_loop->chassis_left_control.wbr_control.L <= CHASSIS_INIT_RETRACT_LEG_TARGET + 0.005f &&
            chassis_move_control_loop->chassis_right_control.wbr_control.L <= CHASSIS_INIT_RETRACT_LEG_TARGET + 0.005f);
}

fp32 chassis_calc_init_wheel_theta_scale(const Chassis_Move *chassis_move_control_loop) {
    // 取两条腿里更差的一侧作为轮子输出门控，避免一边没摆正时轮子提前发力。
    const fp32 theta_err = fmaxf(
        fabsf(chassis_move_control_loop->chassis_left_control.theta_l_ctrl - CHASSIS_INIT_WHEEL_TARGET_THETA),
        fabsf(chassis_move_control_loop->chassis_right_control.theta_l_ctrl - CHASSIS_INIT_WHEEL_TARGET_THETA));
    const fp32 span = CHASSIS_INIT_WHEEL_ZERO_OUTPUT_THETA_ERR - CHASSIS_INIT_WHEEL_FULL_OUTPUT_THETA_ERR;

    if (span <= 1e-6f) {
        return (theta_err <= CHASSIS_INIT_WHEEL_FULL_OUTPUT_THETA_ERR) ? 1.0f : 0.0f;
    }

    return float_constrain((CHASSIS_INIT_WHEEL_ZERO_OUTPUT_THETA_ERR - theta_err) / span, 0.0f, 1.0f);
}

void chassis_apply_init_wheel_theta_gate(Chassis_Move *chassis_move_control_loop) {
    // 这个最终阶段的轮子门控默认 wheel_T 仍然是本周期基础 LQR 的输出，
    // 中间没有被别的阶段逻辑提前改写。
    if (chassis_move_control_loop->state != CHASSIS_INIT) {
        return;
    }

    if (chassis_move_control_loop->init_phase != CHASSIS_INIT_STAND ||
        chassis_init_legs_retracted(chassis_move_control_loop) == 0) {
        // INIT 只有在“已收腿且开始站起”后才允许轮子输出。
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
        return;
    }

    const fp32 theta_scale = chassis_calc_init_wheel_theta_scale(chassis_move_control_loop);
    chassis_move_control_loop->chassis_wheel[0].wheel_T *= theta_scale;
    chassis_move_control_loop->chassis_wheel[1].wheel_T *= theta_scale;
}

fp32 chassis_limit_init_leg_rotate_torque(fp32 torque, fp32 d_theta) {
    torque = float_constrain(torque, -CHASSIS_INIT_LEVEL_TORQUE_LIMIT, CHASSIS_INIT_LEVEL_TORQUE_LIMIT);

    if (fabsf(d_theta) > CHASSIS_INIT_LEVEL_SPEED_LIMIT &&
        torque * d_theta > 0.0f) {
        return 0.0f;
    }

    return torque;
}

fp32 chassis_sanitize_dd_L(fp32 dd_L) {
    if ((dd_L != dd_L) || (dd_L > 1e5f) || (dd_L < -1e5f)) {
        return 0.0f;
    }
    return dd_L;
}

fp32 chassis_calc_leg_extend_force(leg_control *leg, fp32 target_leg_length) {
    return -Leg_PID_Calc(&leg->leg_pid_control, leg->wbr_control.L, target_leg_length);
}

void chassis_update_leg_angle_signals(leg_control *leg, const fp32 *f_theta, uint8_t theta_filter_source, uint8_t d_theta_filter_source) {
    // 腿角信号同时保留原始值、低通值和卡尔曼值，同时在 feedback 阶段就缓存出本周期控制实际采用的一路。
    memcpy(leg->chassis_vaestimatekf_theta.F_data, f_theta, 4 * sizeof(fp32));

    leg->theta_l   = leg->wbr_control.theta_l;
    leg->d_theta_l = leg->wbr_control.d_theta_l;

    first_order_filter_cali(&leg->theta_l_lowpass_filter, leg->theta_l);
    first_order_filter_cali(&leg->d_theta_l_lowpass_filter, leg->d_theta_l);
    leg->theta_l_lowpass   = leg->theta_l_lowpass_filter.out;
    leg->d_theta_l_lowpass = leg->d_theta_l_lowpass_filter.out;

    leg->chassis_vaestimatekf_theta.MeasuredVector[0] = leg->theta_l;
    leg->chassis_vaestimatekf_theta.MeasuredVector[1] = leg->d_theta_l;

    Kalman_Filter_Update(&leg->chassis_vaestimatekf_theta);
    leg->theta_l_filter   = leg->chassis_vaestimatekf_theta.FilteredValue[0];
    leg->d_theta_l_filter = leg->chassis_vaestimatekf_theta.FilteredValue[1];
    leg->theta_l_ctrl     = (theta_filter_source == CHASSIS_FILTER_LOWPASS) ? leg->theta_l_lowpass : leg->theta_l_filter;
    leg->d_theta_l_ctrl   = (d_theta_filter_source == CHASSIS_FILTER_LOWPASS) ? leg->d_theta_l_lowpass : leg->d_theta_l_filter;
}

bool_t chassis_is_balancing_state(Chassis_State_e state) {
    return state == CHASSIS_NORMAL || state == CHASSIS_LEG_1 || state == CHASSIS_LEG_2;
}

bool_t chassis_is_step_mode_state(Chassis_State_e state) {
    return state == CHASSIS_LEG_1 || state == CHASSIS_LEG_2;
}

bool_t chassis_is_step_up_active(const Chassis_Move *chassis_move_control) {
    if (chassis_move_control == NULL) {
        return 0;
    }

    return chassis_is_step_mode_state(chassis_move_control->state) &&
           chassis_move_control->step_up_phase != STEP_UP_DONE;
}

bool_t chassis_is_step_up_before_standup(const Chassis_Move *chassis_move_control) {
    if (chassis_is_step_up_active(chassis_move_control) == 0) {
        return 0;
    }

    // 这几段仍属于“台阶子流程未完成”，主控制要继续抑制正常前进逻辑。
    return (chassis_move_control->step_up_phase == STEP_UP_SWING ||
            chassis_move_control->step_up_phase == STEP_UP_HOLD ||
            chassis_move_control->step_up_phase == STEP_UP_RETRACT ||
            chassis_move_control->step_up_phase == STEP_UP_STAND);
}

bool_t chassis_is_init_like_state(Chassis_State_e state) {
    return state == CHASSIS_INIT || state == CHASSIS_FLIP;
}

bool_t chassis_should_reset_small_gyro_translation(const Chassis_Move *chassis_move_control) {
    if (chassis_move_control == NULL || chassis_move_control->chassis_gimbal_data == NULL) {
        return 0;
    }

    if (chassis_is_balancing_state(chassis_move_control->state) == 0) {
        return 0;
    }

    if (chassis_move_control->chassis_gimbal_data->gyro_enable == 0U) {
        return 0;
    }

    return (fabsf(chassis_move_control->chassis_gimbal_data->v_tmp) <= CHASSIS_SMALL_GYRO_ZERO_V_INPUT_EPS);
}

bool_t chassis_is_small_gyro_active(const Chassis_Move *chassis_move_control) {
    if (chassis_move_control == NULL || chassis_move_control->chassis_gimbal_data == NULL) {
        return 0;
    }

    if (chassis_is_balancing_state(chassis_move_control->state) == 0) {
        return 0;
    }

    return (chassis_move_control->chassis_gimbal_data->gyro_enable != 0U);
}

fp32 chassis_calc_small_gyro_v_compensation(const Chassis_Move *chassis_move_control) {
    if (chassis_move_control == NULL) {
        return 0.0f;
    }

    // 小陀螺下用少量角度前馈 + 速度负反馈，抵消原地旋转时的平移漂移。
    fp32 compensation =
        CHASSIS_SMALL_GYRO_V_ANGLE_FEEDFORWARD_GAIN *
            arm_sin_f32(((chassis_move_control->chassis_gimbal_data != NULL)
                             ? chassis_move_control->chassis_gimbal_data->chassis_relative_angle
                             : 0.0f) +
                        CHASSIS_SMALL_GYRO_V_ANGLE_FEEDFORWARD_PHASE) -
        chassis_move_control->v_filter * CHASSIS_SMALL_GYRO_V_COMPENSATION_GAIN;

    return float_constrain(
        compensation,
        -fabsf(CHASSIS_SMALL_GYRO_V_COMPENSATION_MAX),
        fabsf(CHASSIS_SMALL_GYRO_V_COMPENSATION_MAX));
}

fp32 chassis_calc_small_gyro_move_v_set(const Chassis_Move *chassis_move_control, fp32 move_speed_cmd) {
    if (chassis_move_control == NULL || chassis_move_control->chassis_gimbal_data == NULL) {
        return move_speed_cmd;
    }

    // 把标量平移指令投影到当前车体相对朝向，再叠加小陀螺漂移补偿。
    const fp32 speed_limit = CHASSIS_KEY_MAX_SPEED * CHASSIS_SMALL_GYRO_MOVE_SPEED_SCALE;

    return float_constrain(
        move_speed_cmd * CHASSIS_SMALL_GYRO_MOVE_GAIN * CHASSIS_SMALL_GYRO_MOVE_SPEED_SCALE *
                arm_cos_f32(chassis_move_control->chassis_gimbal_data->chassis_relative_angle -
                            chassis_move_control->chassis_gimbal_data->yaw_set +
                            CHASSIS_SMALL_GYRO_MOVE_PHASE) +
            chassis_calc_small_gyro_v_compensation(chassis_move_control),
        -speed_limit,
        speed_limit);
}

#ifdef __cplusplus
}
#endif
