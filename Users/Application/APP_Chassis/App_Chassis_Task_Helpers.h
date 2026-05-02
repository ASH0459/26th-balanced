#ifndef INFANTRY_ROBOT_APP_CHASSIS_TASK_HELPERS_H
#define INFANTRY_ROBOT_APP_CHASSIS_TASK_HELPERS_H

#include "App_Chassis_Task.h"

#ifdef __cplusplus
extern "C" {
#endif

bool_t                         chassis_init_legs_retracted(const Chassis_Move *chassis_move_control_loop);
fp32                           chassis_calc_init_wheel_theta_scale(const Chassis_Move *chassis_move_control_loop);
void                           chassis_apply_init_wheel_theta_gate(Chassis_Move *chassis_move_control_loop);
fp32                           chassis_limit_init_leg_rotate_torque(fp32 torque, fp32 d_theta);
fp32                           chassis_sanitize_dd_L(fp32 dd_L);
fp32                           chassis_calc_leg_extend_force(leg_control *leg, fp32 target_leg_length);
void                           chassis_update_leg_angle_signals(leg_control *leg, const fp32 *f_theta, uint8_t theta_filter_source, uint8_t d_theta_filter_source);
bool_t                         chassis_is_balancing_state(Chassis_State_e state);
bool_t                         chassis_is_step_mode_state(Chassis_State_e state);
bool_t                         chassis_is_step_up_active(const Chassis_Move *chassis_move_control);
bool_t                         chassis_is_step_up_before_standup(const Chassis_Move *chassis_move_control);
bool_t                         chassis_is_init_like_state(Chassis_State_e state);
bool_t                         chassis_should_reset_small_gyro_translation(const Chassis_Move *chassis_move_control);
bool_t                         chassis_is_small_gyro_active(const Chassis_Move *chassis_move_control);
fp32                           chassis_calc_small_gyro_v_compensation(const Chassis_Move *chassis_move_control);
fp32                           chassis_calc_small_gyro_move_v_set(const Chassis_Move *chassis_move_control, fp32 move_speed_cmd);

#ifdef __cplusplus
}
#endif

#endif
