/**
  ****************************(C) COPYRIGHT 2026 ROBOTZ****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history+
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.1.0
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ROBOTZ****************************
  */
#include "App_Chassis_Task.h"
#include "App_Chassis_Behaviour.h"
#include "arm_math.h"
#include "Alg_PID.h"
#include "Dev_CAN_Receive.h"
#include "App_Detect_Task.h"
#include "INS_Task.h"
#include "wbr.h"
#include "bsp_dwt.h"
#include "kalman_filter.h"
#include "APL_PowerLimitator.hpp"

Chassis_Move chassis_move;

#ifdef __cplusplus
extern "C"
{
#endif

    // 拟合矩阵K，A，B
    fp32 fitted_matrix_K[40][6] = {{
        -1.6803, 0.56887, -2.096, -1.2984, 1.0284, 2.5882},
{-2.9744, 1.903, -2.0786, -2.2104, -0.98175, 3.6878},
{-6.0567, -4.2153, 3.8915, 6.2571, -0.11425, -5.6746},
{-0.56676, -0.65309, 0.63491, 0.8721, -0.026287, -0.81669},
{-3.0723, -30.225, 7.1043, 19.192, 3.7964, -1.0355},
{-0.38372, -4.3757, 0.78489, 1.4987, -0.034365, 0.20218},
{-2.0434, 13.091, -39.955, -12.058, 9.2867, 30.27},
{-0.17317, 1.9047, -6.201, -1.8832, 1.0477, 3.6636},
{-30.464, 62.014, 44.697, -42.96, -64.919, -15.798},
{-3.3448, 6.9302, 5.0038, -4.8504, -7.2622, -1.8195},
{-1.6803, -2.096, 0.56887, 2.5882, 1.0284, -1.2984},
{-2.9744, -2.0786, 1.903, 3.6878, -0.98175, -2.2104},
{6.0567, -3.8915, 4.2153, 5.6746, 0.11425, -6.2571},
{0.56676, -0.63491, 0.65309, 0.81669, 0.026287, -0.8721},
{-2.0434, -39.955, 13.091, 30.27, 9.2867, -12.058},
{-0.17317, -6.201, 1.9047, 3.6636, 1.0477, -1.8832},
{-3.0723, 7.1043, -30.225, -1.0355, 3.7964, 19.192},
{-0.38372, 0.78489, -4.3757, 0.20218, -0.034365, 1.4987},
{-30.464, 44.697, 62.014, -15.798, -64.919, -42.96},
{-3.3448, 5.0038, 6.9302, -1.8195, -7.2622, -4.8504},
{20.105, -44.972, -49.892, 27.386, 49.052, 42.452},
{33.564, -72.339, -86.183, 42.784, 89.101, 71.006},
{-15.272, 43.738, 12.539, -61.197, 4.6168, -21.918},
{-1.4881, 5.2613, 1.3171, -7.705, 2.1476, -2.5338},
{109.7, -183.99, -0.69629, 113.58, 262.51, -56.41},
{21.235, -50.079, 2.314, 39.02, 35.337, -11.363},
{25.668, -205.76, -92.411, 243.95, -196.35, 195.02},
{3.84, -35.641, -10.249, 42.583, -29.948, 29.572},
{-120.03, -1361.1, 476.61, 1615.8, 334.76, -708.64},
{-14.197, -154.66, 55.894, 185.07, 36.636, -82.631},
{20.105, -49.892, -44.972, 42.452, 49.052, 27.386},
{33.564, -86.183, -72.339, 71.006, 89.101, 42.784},
{15.272, -12.539, -43.738, 21.918, -4.6168, 61.197},
{1.4881, -1.3171, -5.2613, 2.5338, -2.1476, 7.705},
{25.668, -92.411, -205.76, 195.02, -196.35, 243.95},
{3.84, -10.249, -35.641, 29.572, -29.948, 42.583},
{109.7, -0.69629, -183.99, -56.41, 262.51, 113.58},
{21.235, 2.314, -50.079, -11.363, 35.337, 39.02},
{-120.03, 476.61, -1361.1, -708.64, 334.76, 1615.8},
{-14.197, 55.894, -154.66, -82.631, 36.636, 185.07}};

    fp32 fitted_matrix_A[15][6];
    fp32 fitted_matrix_B[20][6];

    // 调试用参数
    fp32 LQR_K[4][10] = {
        {-1.8494, -2.9785, -6.1156, -0.56997, -5.961, -0.87399, -5.6496, -0.79925, -17.196, -1.8607},
        {-1.8494, -2.9785, 6.1156, 0.56997, -5.6496, -0.79925, -5.961, -0.87399, -17.196, -1.8607},
        {8.1228, 13.873, -4.4553, -0.27176, 88.423, 15.371, -15.191, -2.3958, -220.02, -25.343},
        {8.1228, 13.873, 4.4553, 0.27176, -15.191, -2.3958, 88.423, 15.371, -220.02, -25.343},
    };

    // 最好的参数1
    //      fp32 LQR_K[4][10] = {
    //          {-0.69427, -2.2387, -3.7475, -0.83554, -5.948, -0.86055, -5.0591, -0.66276, -9.5708, -1.1221},
    //  {-0.69427, -2.2387, 3.7475, 0.83554, -5.0591, -0.66276, -5.948, -0.86055, -9.5708, -1.1221},
    //  {2.121, 7.1838, -15.461, -3.6163, 63.556, 10.488, -14.781, -2.0856, -122.64, -19.559},
    //  {2.121, 7.1838, 15.461, 3.6163, -14.781, -2.0856, 63.556, 10.488, -122.64, -19.559},
    //      };

    fp32 LQR_A[10][10] = {
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, -18.446, 0, -18.446, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, -18.915, 0, 18.915, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 230.26, 0, -5.6636, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, -5.6636, 0, 230.26, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, -28.655, 0, -28.655, 0, 97.234, 0},
    };
    fp32 LQR_B[10][4] = {
        {0, 0, 0, 0},
        {3.7721, 3.7721, -0.82511, -0.82511},
        {0, 0, 0, 0},
        {-25.279, 25.279, -0.8461, 0.8461},
        {0, 0, 0, 0},
        {-18.183, -20.181, 10.3, -0.25335},
        {0, 0, 0, 0},
        {-20.181, -18.183, -0.25335, 10.3},
        {0, 0, 0, 0},
        {-1.2642, -1.2642, -4.0638, -4.0638},
    };

/* 消除DT7摇杆死区 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/* 记录堆栈最高值 */
#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t chassis_high_water;
#endif

    static fp32 Get_FeedForward_Force(fp32 leg_length_m);

    static inline fp32 chassis_select_theta_signal(const leg_control *leg, uint8_t filter_source)
    {
        return (filter_source == CHASSIS_FILTER_LOWPASS) ? leg->theta_l_lowpass : leg->theta_l_filter;
    }

    static inline fp32 chassis_select_d_theta_signal(const leg_control *leg, uint8_t filter_source)
    {
        return (filter_source == CHASSIS_FILTER_LOWPASS) ? leg->d_theta_l_lowpass : leg->d_theta_l_filter;
    }

    static inline fp32 chassis_calc_leg_tbl_attenuation_scale(fp32 abs_theta_l)
    {
#if CHASSIS_LEG_TBL_ANGLE_ATTENUATION_ENABLE
        return 1.0f / (1.0f + expf(CHASSIS_LEG_TBL_ANGLE_ATTENUATION_SHARPNESS *
                                   (abs_theta_l - CHASSIS_LEG_TBL_ANGLE_ATTENUATION_CENTER)));
#else
    (void)abs_theta_l;
    return 1.0f;
#endif
    }

    static inline void chassis_apply_leg_tbl_angle_attenuation(Chassis_Move *chassis_move_control_loop)
    {
#if CHASSIS_LEG_TBL_ANGLE_ATTENUATION_ENABLE
        const fp32 abs_theta_l = fabsf(chassis_select_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_THETA_FILTER_SOURCE));
        const fp32 scale_l = chassis_calc_leg_tbl_attenuation_scale(abs_theta_l);
        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t *= scale_l;

        const fp32 abs_theta_r = fabsf(chassis_select_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_THETA_FILTER_SOURCE));
        const fp32 scale_r = chassis_calc_leg_tbl_attenuation_scale(abs_theta_r);
        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t *= scale_r;
#else
    (void)chassis_move_control_loop;
#endif
    }

    static inline fp32 chassis_calc_init_stand_bias_strength(fp32 abs_theta_l)
    {
        return 1.0f / (1.0f + expf(-CHASSIS_INIT_STAND_DRIVE_BIAS_SHARPNESS *
                                   (abs_theta_l - CHASSIS_INIT_STAND_DRIVE_BIAS_CENTER)));
    }

    static inline void chassis_apply_init_stand_drive_bias(Chassis_Move *chassis_move_control_loop)
    {
#if CHASSIS_INIT_STAND_DRIVE_BIAS_ENABLE
        if (chassis_move_control_loop->state != CHASSIS_INIT ||
            chassis_move_control_loop->init_phase != CHASSIS_INIT_STAND)
        {
            return;
        }

        const fp32 abs_theta_l = fabsf(chassis_select_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_THETA_FILTER_SOURCE));
        const fp32 abs_theta_r = fabsf(chassis_select_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_THETA_FILTER_SOURCE));

        const fp32 bias_l = chassis_calc_init_stand_bias_strength(abs_theta_l);
        const fp32 bias_r = chassis_calc_init_stand_bias_strength(abs_theta_r);

        const fp32 leg_scale_span = CHASSIS_INIT_STAND_LEG_TBL_SCALE_MAX - CHASSIS_INIT_STAND_LEG_TBL_SCALE_MIN;
        const fp32 wheel_scale_span = CHASSIS_INIT_STAND_WHEEL_SCALE_MAX - CHASSIS_INIT_STAND_WHEEL_SCALE_MIN;

        const fp32 leg_scale_l = CHASSIS_INIT_STAND_LEG_TBL_SCALE_MIN + leg_scale_span * bias_l;
        const fp32 leg_scale_r = CHASSIS_INIT_STAND_LEG_TBL_SCALE_MIN + leg_scale_span * bias_r;

        const fp32 wheel_scale_l = CHASSIS_INIT_STAND_WHEEL_SCALE_MAX - wheel_scale_span * bias_l;
        const fp32 wheel_scale_r = CHASSIS_INIT_STAND_WHEEL_SCALE_MAX - wheel_scale_span * bias_r;

        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t *= leg_scale_l;
        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t *= leg_scale_r;

        chassis_move_control_loop->chassis_wheel[0].wheel_T *= wheel_scale_l;
        chassis_move_control_loop->chassis_wheel[1].wheel_T *= wheel_scale_r;
#else
    (void)chassis_move_control_loop;
#endif
    }

    static inline fp32 chassis_sanitize_dd_L(fp32 dd_L)
    {
        if ((dd_L != dd_L) || (dd_L > 1e5f) || (dd_L < -1e5f))
        {
            return 0.0f;
        }
        return dd_L;
    }

    static inline fp32 chassis_calc_kinematic_d_yaw(const Chassis_Move *chassis_move_calc)
    {
        return (WHEEL_RADIUS * (-chassis_move_calc->chassis_wheel[0].vel + chassis_move_calc->chassis_wheel[1].vel) - chassis_move_calc->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_calc->chassis_left_control.wbr_control.theta_l) * chassis_move_calc->chassis_left_control.wbr_control.d_theta_l + chassis_move_calc->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_calc->chassis_right_control.wbr_control.theta_l) * chassis_move_calc->chassis_right_control.wbr_control.d_theta_l) / (2 * DRIVE_WHEEL_DIS);
    }

    static inline fp32 chassis_calc_leg_extend_force(leg_control *leg, fp32 target_leg_length)
    {
        return -Leg_PID_Calc(&leg->leg_pid_control, leg->wbr_control.L, target_leg_length);
    }

    static inline chassis_off_ground_detection_e chassis_update_off_ground_detection_state(chassis_off_ground_detection_e current_state, fp32 support_force)
    {
        if (current_state == CHASSIS_OFF_GROUND)
        {
            return (support_force >= CHASSIS_TOUCH_GROUND_FORCE_THRESHOLD) ? CHASSIS_TOUCH_GROUND : CHASSIS_OFF_GROUND;
        }
        return (support_force <= CHASSIS_OFF_GROUND_FORCE_THRESHOLD) ? CHASSIS_OFF_GROUND : CHASSIS_TOUCH_GROUND;
    }

    static inline fp32 chassis_get_imu_d_yaw(const Chassis_Move *chassis_move_calc)
    {
        return CHASSIS_D_YAW_IMU_SIGN * (*(chassis_move_calc->chassis_INS_gyro + INS_GYRO_Z_ADDRESS_OFFSET));
    }

    static inline fp32 chassis_select_d_yaw_feedback(const Chassis_Move *chassis_move_calc)
    {
#if CHASSIS_D_YAW_SOURCE == CHASSIS_D_YAW_SOURCE_IMU
        return chassis_move_calc->chassis_d_yaw_imu;
#else
    return chassis_move_calc->chassis_d_yaw_kinematic;
#endif
    }

    static void chassis_update_leg_angle_signals(leg_control *leg, const fp32 *f_theta)
    {
        memcpy(leg->chassis_vaestimatekf_theta.F_data, f_theta, 4 * sizeof(fp32));

        leg->theta_l = leg->wbr_control.theta_l;
        leg->d_theta_l = leg->wbr_control.d_theta_l;

        first_order_filter_cali(&leg->theta_l_lowpass_filter, leg->theta_l);
        first_order_filter_cali(&leg->d_theta_l_lowpass_filter, leg->d_theta_l);
        leg->theta_l_lowpass = leg->theta_l_lowpass_filter.out;
        leg->d_theta_l_lowpass = leg->d_theta_l_lowpass_filter.out;

        leg->chassis_vaestimatekf_theta.MeasuredVector[0] = leg->theta_l;
        leg->chassis_vaestimatekf_theta.MeasuredVector[1] = leg->d_theta_l;

        Kalman_Filter_Update(&leg->chassis_vaestimatekf_theta);
        leg->theta_l_filter = leg->chassis_vaestimatekf_theta.FilteredValue[0];
        leg->d_theta_l_filter = leg->chassis_vaestimatekf_theta.FilteredValue[1];
    }

    static bool_t chassis_is_balancing_state(Chassis_State_e state)
    {
        return state == CHASSIS_NORMAL || state == CHASSIS_LEG_1 || state == CHASSIS_LEG_2 || state == CHASSIS_STEP_UP;
    }

    static bool_t chassis_is_yaw_lqr_state(Chassis_State_e state)
    {
        return chassis_is_balancing_state(state) || state == CHASSIS_JUMP;
    }

    static bool_t chassis_is_init_like_state(Chassis_State_e state)
    {
        return state == CHASSIS_INIT || state == CHASSIS_FLIP;
    }

    static bool_t chassis_should_reset_small_gyro_translation(const Chassis_Move *chassis_move_control)
    {
        if (chassis_move_control == NULL || chassis_move_control->chassis_gimbal_data == NULL)
        {
            return 0;
        }

        if (chassis_is_balancing_state(chassis_move_control->state) == 0)
        {
            return 0;
        }

        if (chassis_move_control->chassis_gimbal_data->gyro_enable == 0U)
        {
            return 0;
        }

        return (fabsf(chassis_move_control->chassis_gimbal_data->v_tmp) <= CHASSIS_SMALL_GYRO_ZERO_V_INPUT_EPS);
    }

    static bool_t chassis_is_small_gyro_active(const Chassis_Move *chassis_move_control)
    {
        if (chassis_move_control == NULL || chassis_move_control->chassis_gimbal_data == NULL)
        {
            return 0;
        }

        if (chassis_is_balancing_state(chassis_move_control->state) == 0)
        {
            return 0;
        }

        return (chassis_move_control->chassis_gimbal_data->gyro_enable != 0U);
    }

    static inline fp32 chassis_calc_small_gyro_v_compensation(const Chassis_Move *chassis_move_control)
    {
        if (chassis_move_control == NULL)
        {
            return 0.0f;
        }

        const fp32 relative_angle =
            (chassis_move_control->chassis_gimbal_data != NULL)
                ? chassis_move_control->chassis_gimbal_data->chassis_relative_angle
                : 0.0f;
        const fp32 angle_feedforward =
            CHASSIS_SMALL_GYRO_V_ANGLE_FEEDFORWARD_GAIN *
            arm_sin_f32(relative_angle + CHASSIS_SMALL_GYRO_V_ANGLE_FEEDFORWARD_PHASE);

        fp32 compensation =
            angle_feedforward -
            chassis_move_control->v_filter * CHASSIS_SMALL_GYRO_V_COMPENSATION_GAIN;
        const fp32 limit = fabsf(CHASSIS_SMALL_GYRO_V_COMPENSATION_MAX);

        if (compensation > limit)
        {
            compensation = limit;
        }
        else if (compensation < -limit)
        {
            compensation = -limit;
        }

        return compensation;
    }

    static inline fp32 chassis_calc_small_gyro_move_v_set(const Chassis_Move *chassis_move_control, fp32 move_speed_cmd)
    {
        if (chassis_move_control == NULL || chassis_move_control->chassis_gimbal_data == NULL)
        {
            return move_speed_cmd;
        }

        const fp32 desired_move_direction = chassis_move_control->chassis_gimbal_data->yaw_set;
        const fp32 relative_angle = chassis_move_control->chassis_gimbal_data->chassis_relative_angle;
        const fp32 move_speed =
            move_speed_cmd * CHASSIS_SMALL_GYRO_MOVE_GAIN * CHASSIS_SMALL_GYRO_MOVE_SPEED_SCALE;
        const fp32 compensation = chassis_calc_small_gyro_v_compensation(chassis_move_control);
        fp32 projected_v_set =
            move_speed *
            arm_cos_f32(relative_angle - desired_move_direction + CHASSIS_SMALL_GYRO_MOVE_PHASE);
        fp32 v_set = projected_v_set + compensation;
        const fp32 speed_limit = CHASSIS_KEY_MAX_SPEED * CHASSIS_SMALL_GYRO_MOVE_SPEED_SCALE;

        if (v_set > speed_limit)
        {
            v_set = speed_limit;
        }
        else if (v_set < -speed_limit)
        {
            v_set = -speed_limit;
        }

        return v_set;
    }

    static void chassis_reset_jump_state(Chassis_Move *chassis_move_control_loop)
    {
        chassis_move_control_loop->jump_phase = CHASSIS_JUMP_DONE;
        chassis_move_control_loop->jump_phase_ticks = 0;
    }
    /**
     * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
     * @param[out]     chassis_move_init:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_init(Chassis_Move *chassis_move_init);

    /**
     * @brief          设置底盘控制模式，主要在Chassis_Behaviour_Mode_set函数中改变
     * @param[out]     chassis_move_mode:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_set_mode(Chassis_Move *chassis_move_mode);

    /**
     * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
     * @param[out]     chassis_move_transit:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_mode_change_control_transit(Chassis_Move *chassis_move_transit);

    /**
     * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
     * @param[out]     chassis_move_update:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_feedback_update(Chassis_Move *chassis_move_update);

    /**
     * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
     *                 根据DT7遥控的设定值，计算得到步兵的三个自由度上的设定速度
     * @param[out]     chassis_move_control:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_set_contorl(Chassis_Move *chassis_move_control);

    /**
     * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
     * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_control_loop(Chassis_Move *chassis_move_control_loop);

    /**
     * @brief          底盘初始化起身处理
     * @param[out]     chassis_init_standup:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_init_standup(Chassis_Move *chassis_init_standup);

    static void chassis_zero_output(Chassis_Move *chassis_move_control_loop);
    static void chassis_flip_control(Chassis_Move *chassis_move_control_loop);
    static void chassis_calc_support_force(Chassis_Move *chassis_move_control_loop);
    static void chassis_apply_joint_output(Chassis_Move *chassis_move_control_loop);
    static void chassis_limit_output(Chassis_Move *chassis_move_control_loop);
    static void chassis_save_last_feedback(Chassis_Move *chassis_move_control_loop);
    static void chassis_update_jump_phase(Chassis_Move *chassis_move_control_loop);
    static void chassis_step_up_swing_control(Chassis_Move *chassis_move_control_loop);
    static void chassis_update_step_up_phase(Chassis_Move *chassis_move_control_loop);
    static void chassis_reset_jump_state(Chassis_Move *chassis_move_control_loop);
    static bool_t chassis_is_balancing_state(Chassis_State_e state);
    static bool_t chassis_is_yaw_lqr_state(Chassis_State_e state);
    static bool_t chassis_is_init_like_state(Chassis_State_e state);

    /**
     * @brief          LQR力矩拆解和功率限制控制
     * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_lqr_power_control(Chassis_Move *chassis_move_control_loop);

    /**
     * @brief          离地检查识别
     * @param          chassis_move_prediction   指向底盘控制结构体的指针，包含底盘的状态和参数信息
     * @retval         无
     */
    static void chassis_interference_prevention(Chassis_Move *chassis_move_prediction);

    void Chassis_Task(void *pvParameters)
    {
        /* 空闲以待初始化 */
        vTaskDelay(CHASSIS_TASK_INIT_TIME);

        /* 底盘初始化 */
        chassis_init(&chassis_move);

        /* 定义绝对延时变量 */
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS);
        TickType_t joint_reenable_tick = 0;

        /* 获取当前时间作为基准 */
        xLastWakeTime = xTaskGetTickCount();

        //    /* 判断电机是否掉线：若掉线 则阻塞底盘任务 */
        //    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
        //    {
        //        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
        //    }

        while (1)
        {
            /* 底盘模式设置 */
            chassis_set_mode(&chassis_move);
            /* 底盘模式切换 数据保存 */
            chassis_mode_change_control_transit(&chassis_move);
            /* 底盘数据更新 */
            chassis_feedback_update(&chassis_move);
            /* 底盘控制量设置 */
            chassis_set_contorl(&chassis_move);
            /* 离地检测数据保护 */
            chassis_interference_prevention(&chassis_move);
            /* 底盘控制循环 */
            chassis_control_loop(&chassis_move);

            if (toe_is_error(CHASSIS_JOINT1_TOE) || toe_is_error(CHASSIS_JOINT2_TOE) ||
                toe_is_error(CHASSIS_JOINT3_TOE) || toe_is_error(CHASSIS_JOINT4_TOE))
            {
                // 关节掉线时先清零输出，避免异常扭矩持续下发
                for (uint8_t i = 0; i < 4; i++)
                {
                    chassis_move.chassis_joint[i].joint_T = 0.0f;
                }

                // 非阻塞限频重使能，避免在控制环里引入 HAL_Delay 抖动
                const TickType_t now = xTaskGetTickCount();
                if ((now - joint_reenable_tick) >= pdMS_TO_TICKS(10))
                {
                    for (uint8_t i = 0; i < 4; i++)
                    {
                        chassis_move.chassis_joint[i].chassis_joint_measure->Enable_Joint_Motor();
                    }
                    joint_reenable_tick = now;
                }
                // 轮电机扭矩清零
                // chassis_move.chassis_wheel[0].wheel_T = 0.0f;
                // chassis_move.chassis_wheel[1].wheel_T = 0.0f;
            }

            else if (toe_is_error(VT_TOE))
            {
                // 关节电机扭矩清零
                for (int i = 0; i < 4; i++)
                {
                    chassis_move.chassis_joint[i].joint_T = 0.0f;
                }
                // 轮电机扭矩清零
                chassis_move.chassis_wheel[0].wheel_T = 0.0f;
                chassis_move.chassis_wheel[1].wheel_T = 0.0f;
            }

#if CHASSIS_FORCE_ALL_MOTOR_ZERO_OUTPUT
            chassis_zero_output(&chassis_move);
#endif

            for (int i = 0; i < 4; i++)
            {
                chassis_move.chassis_joint[i].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, chassis_move.chassis_joint[i].joint_T);
                // chassis_move.chassis_joint[i].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, 0.0f);
            }
#if CHASSIS_VERTICAL_SUPPORT_ONLY_MODE
            CAN_cmd_wheel(0.0f, 0.0f);
#else
        CAN_cmd_wheel(-chassis_move.chassis_wheel[0].wheel_T, chassis_move.chassis_wheel[1].wheel_T);
#endif
            // CAN_cmd_wheel(0,0);

            /* 任务阻塞 时间片切出 */
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

#if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
    }

    /**
     * @brief          初始化"chassis_move"变量，包括pid初始化，一阶低通滤波初始化，遥控器指针初始化，3508底盘电机指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
     * @param[out]     chassis_move_init:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_init(Chassis_Move *chassis_move_init)
    {
        if (chassis_move_init == NULL)
        {
            return;
        }
        // 特定腿长下写死的常量
        chassis_move_init->chassis_left_control.l_b = chassis_move_init->chassis_right_control.l_b = 0.087f;
        chassis_move_init->chassis_left_control.i_l = chassis_move_init->chassis_right_control.i_l = 0.01993708;
        chassis_move_init->K = K_APAPT;

        // 卡尔曼滤波矩阵参数
        chassis_move_init->vaEstimateKF_F_VX[0] = 1.0f;
        chassis_move_init->vaEstimateKF_F_VX[1] = 0.002;
        chassis_move_init->vaEstimateKF_F_VX[2] = 0.0f;
        chassis_move_init->vaEstimateKF_F_VX[3] = 1.0f;

        chassis_move_init->vaEstimateKF_P_VX[0] = 100.0f;
        chassis_move_init->vaEstimateKF_P_VX[1] = 0.0f;
        chassis_move_init->vaEstimateKF_P_VX[2] = 0.0f;
        chassis_move_init->vaEstimateKF_P_VX[3] = 100.0f;

        chassis_move_init->vaEstimateKF_Q_VX[0] = 0.001f;
        chassis_move_init->vaEstimateKF_Q_VX[1] = 0.0f;
        chassis_move_init->vaEstimateKF_Q_VX[2] = 0.0f;
        chassis_move_init->vaEstimateKF_Q_VX[3] = 0.001f;

        chassis_move_init->vaEstimateKF_R_VX[0] = 0.01f;
        chassis_move_init->vaEstimateKF_R_VX[1] = 0.0f;
        chassis_move_init->vaEstimateKF_R_VX[2] = 0.0f;
        chassis_move_init->vaEstimateKF_R_VX[3] = 1.0f;

        chassis_move_init->vaEstimateKF_H_VX[0] = 1.0f;
        chassis_move_init->vaEstimateKF_H_VX[1] = 0.0f;
        chassis_move_init->vaEstimateKF_H_VX[2] = 0.0f;
        chassis_move_init->vaEstimateKF_H_VX[3] = 1.0f;

        chassis_move_init->vaEstimateKF_F_theta[0] = 1.0f;
        chassis_move_init->vaEstimateKF_F_theta[1] = 0.002;
        chassis_move_init->vaEstimateKF_F_theta[2] = 0.0f;
        chassis_move_init->vaEstimateKF_F_theta[3] = 1.0f;

        chassis_move_init->vaEstimateKF_P_theta[0] = 100.0f;
        chassis_move_init->vaEstimateKF_P_theta[1] = 0.0f;
        chassis_move_init->vaEstimateKF_P_theta[2] = 0.0f;
        chassis_move_init->vaEstimateKF_P_theta[3] = 100.0f;

        chassis_move_init->vaEstimateKF_Q_theta[0] = 0.001f;
        chassis_move_init->vaEstimateKF_Q_theta[1] = 0.0f;
        chassis_move_init->vaEstimateKF_Q_theta[2] = 0.0f;
        chassis_move_init->vaEstimateKF_Q_theta[3] = 0.001f;

        chassis_move_init->vaEstimateKF_R_theta[0] = 0.01f;
        chassis_move_init->vaEstimateKF_R_theta[1] = 0.0f;
        chassis_move_init->vaEstimateKF_R_theta[2] = 0.0f;
        chassis_move_init->vaEstimateKF_R_theta[3] = 0.5f;

        chassis_move_init->vaEstimateKF_H_theta[0] = 1.0f;
        chassis_move_init->vaEstimateKF_H_theta[1] = 0.0f;
        chassis_move_init->vaEstimateKF_H_theta[2] = 0.0f;
        chassis_move_init->vaEstimateKF_H_theta[3] = 1.0f;

        chassis_move_init->vaEstimateKF_F_VA[0] = 1.0f;
        chassis_move_init->vaEstimateKF_F_VA[1] = 0.002;
        chassis_move_init->vaEstimateKF_F_VA[2] = 0.0f;
        chassis_move_init->vaEstimateKF_F_VA[3] = 1.0f;

        chassis_move_init->vaEstimateKF_P_VA[0] = 100.0f;
        chassis_move_init->vaEstimateKF_P_VA[1] = 0.0f;
        chassis_move_init->vaEstimateKF_P_VA[2] = 0.0f;
        chassis_move_init->vaEstimateKF_P_VA[3] = 100.0f;

        chassis_move_init->vaEstimateKF_Q_VA[0] = 0.001f;
        chassis_move_init->vaEstimateKF_Q_VA[1] = 0.0f;
        chassis_move_init->vaEstimateKF_Q_VA[2] = 0.0f;
        chassis_move_init->vaEstimateKF_Q_VA[3] = 0.001f;

        chassis_move_init->vaEstimateKF_R_VA[0] = 0.01f;
        chassis_move_init->vaEstimateKF_R_VA[1] = 0.0f;
        chassis_move_init->vaEstimateKF_R_VA[2] = 0.0f;
        chassis_move_init->vaEstimateKF_R_VA[3] = 0.5f;

        chassis_move_init->vaEstimateKF_H_VA[0] = 1.0f;
        chassis_move_init->vaEstimateKF_H_VA[1] = 0.0f;
        chassis_move_init->vaEstimateKF_H_VA[2] = 0.0f;
        chassis_move_init->vaEstimateKF_H_VA[3] = 1.0f;

        Kalman_Filter_Init(&chassis_move_init->chassis_vaestimatekf_xv, 2, 0, 2);                          // 状态向量2维 没有控制量 测量向量2维
        Kalman_Filter_Init(&chassis_move_init->chassis_vaestimatekf_va, 2, 0, 2);                          // 状态向量2维 没有控制量 测量向量2维
        Kalman_Filter_Init(&chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta, 2, 0, 2);  // 状态向量2维 没有控制量 测量向量2维
        Kalman_Filter_Init(&chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

        memcpy(chassis_move_init->chassis_vaestimatekf_xv.F_data, chassis_move_init->vaEstimateKF_F_VX, sizeof(chassis_move_init->vaEstimateKF_F_VX));
        memcpy(chassis_move_init->chassis_vaestimatekf_xv.P_data, chassis_move_init->vaEstimateKF_P_VX, sizeof(chassis_move_init->vaEstimateKF_P_VX));
        memcpy(chassis_move_init->chassis_vaestimatekf_xv.Q_data, chassis_move_init->vaEstimateKF_Q_VX, sizeof(chassis_move_init->vaEstimateKF_Q_VX));
        memcpy(chassis_move_init->chassis_vaestimatekf_xv.R_data, chassis_move_init->vaEstimateKF_R_VX, sizeof(chassis_move_init->vaEstimateKF_R_VX));
        memcpy(chassis_move_init->chassis_vaestimatekf_xv.H_data, chassis_move_init->vaEstimateKF_H_VX, sizeof(chassis_move_init->vaEstimateKF_H_VX));

        memcpy(chassis_move_init->chassis_vaestimatekf_va.F_data, chassis_move_init->vaEstimateKF_F_VA, sizeof(chassis_move_init->vaEstimateKF_F_VA));
        memcpy(chassis_move_init->chassis_vaestimatekf_va.P_data, chassis_move_init->vaEstimateKF_P_VA, sizeof(chassis_move_init->vaEstimateKF_P_VA));
        memcpy(chassis_move_init->chassis_vaestimatekf_va.Q_data, chassis_move_init->vaEstimateKF_Q_VA, sizeof(chassis_move_init->vaEstimateKF_Q_VA));
        memcpy(chassis_move_init->chassis_vaestimatekf_va.R_data, chassis_move_init->vaEstimateKF_R_VA, sizeof(chassis_move_init->vaEstimateKF_R_VA));
        memcpy(chassis_move_init->chassis_vaestimatekf_va.H_data, chassis_move_init->vaEstimateKF_H_VA, sizeof(chassis_move_init->vaEstimateKF_H_VA));

        memcpy(chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta.F_data, chassis_move_init->vaEstimateKF_F_theta, sizeof(chassis_move_init->vaEstimateKF_F_theta));
        memcpy(chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta.P_data, chassis_move_init->vaEstimateKF_P_theta, sizeof(chassis_move_init->vaEstimateKF_P_theta));
        memcpy(chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta.Q_data, chassis_move_init->vaEstimateKF_Q_theta, sizeof(chassis_move_init->vaEstimateKF_Q_theta));
        memcpy(chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta.R_data, chassis_move_init->vaEstimateKF_R_theta, sizeof(chassis_move_init->vaEstimateKF_R_theta));
        memcpy(chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta.H_data, chassis_move_init->vaEstimateKF_H_theta, sizeof(chassis_move_init->vaEstimateKF_H_theta));
        memcpy(chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta.F_data, chassis_move_init->vaEstimateKF_F_theta, sizeof(chassis_move_init->vaEstimateKF_F_theta));
        memcpy(chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta.P_data, chassis_move_init->vaEstimateKF_P_theta, sizeof(chassis_move_init->vaEstimateKF_P_theta));
        memcpy(chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta.Q_data, chassis_move_init->vaEstimateKF_Q_theta, sizeof(chassis_move_init->vaEstimateKF_Q_theta));
        memcpy(chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta.R_data, chassis_move_init->vaEstimateKF_R_theta, sizeof(chassis_move_init->vaEstimateKF_R_theta));
        memcpy(chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta.H_data, chassis_move_init->vaEstimateKF_H_theta, sizeof(chassis_move_init->vaEstimateKF_H_theta));

        /* 陀螺仪姿态角数据指针 */
        chassis_move_init->chassis_INS_angle = get_INS_angle_point();
        chassis_move_init->chassis_INS_gyro = get_gyro_data_point();
        chassis_move_init->chassis_motion_accel_n = get_montion_accel_n_data_point();

        /*获取双板通信云台数据指针*/
        chassis_move_init->chassis_gimbal_data = gimbal_data.get_gimbal_point();

        /*获得轮电机数据指针*/
        for (uint8_t i = 0; i < 2; i++)
        {
            chassis_move_init->chassis_wheel[i].chassis_wheel_measure = chassis_wheel[i].get_chassis_motor_measure_point();
        }

        /*获得关节电机数据指针*/
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_init->chassis_joint[i].chassis_joint_measure = chassis_joint[i].get_chassis_motor_measure_point();
            chassis_move_init->chassis_joint[i].chassis_joint_measure->Enable_Joint_Motor();
            vTaskDelay(pdMS_TO_TICKS(JOINT_MOTOR_lIMIT_TIME));
        }

        // 初始化wbr数据
        wbr_init(&chassis_move_init->chassis_left_control.wbr_control);
        wbr_init(&chassis_move_init->chassis_right_control.wbr_control);

        // yaw轴跟随pid初始化
        const static fp32 yaw_angle_pid[3] = {YAW_PID_KP, YAW_PID_KI, YAW_PID_KD};

        // roll和腿长pid初始化
        const static fp32 roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
        const static fp32 leg_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

        PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, yaw_angle_pid, YAW_PID_MAX_OUT, YAW_PID_MAX_OUT); // yaw轴跟随pid

        PID_init(&chassis_move_init->chassis_left_control.roll_control, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);  // 左腿roll轴pid
        PID_init(&chassis_move_init->chassis_right_control.roll_control, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT); // 左腿roll轴pid
        PID_init(&chassis_move_init->chassis_left_control.leg_pid_control, PID_POSITION, leg_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);  // 左腿长pid
        PID_init(&chassis_move_init->chassis_right_control.leg_pid_control, PID_POSITION, leg_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT); // 右腿长pidD
        // 初始化收腿角度PID
        const static fp32 init_leg_angle_pid[3] = {INIT_LEG_ANGLE_PID_KP, INIT_LEG_ANGLE_PID_KI, INIT_LEG_ANGLE_PID_KD};
        PID_init(&chassis_move_init->chassis_left_control.init_leg_angle_pid, PID_POSITION, init_leg_angle_pid, INIT_LEG_ANGLE_PID_MAX_OUT, INIT_LEG_ANGLE_PID_MAX_IOUT);
        PID_init(&chassis_move_init->chassis_right_control.init_leg_angle_pid, PID_POSITION, init_leg_angle_pid, INIT_LEG_ANGLE_PID_MAX_OUT, INIT_LEG_ANGLE_PID_MAX_IOUT);

        /* 滤波参数：DT7发送设定值的低通滤波 */
        const static float chassis_DT7_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
        const static float chassis_DT7_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
        const static fp32 chassis_leg_order_filter[1] = {CHASSIS_ACCEL_LEG_NUM};
        const static fp32 chassis_theta_l_lowpass_filter[1] = {CHASSIS_THETA_L_LOWPASS_NUM};
        const static fp32 chassis_d_theta_l_lowpass_filter[1] = {CHASSIS_D_THETA_L_LOWPASS_NUM};
        const static fp32 chassis_dd_L_lowpass_filter[1] = {CHASSIS_DD_L_LOWPASS_NUM};

        /* 滤波初始化：DT7发送设定值的低通滤波 */
        first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_DT7_x_order_filter);
        first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_DT7_y_order_filter);
        first_order_filter_init(&chassis_move_init->chassis_leg_filter_set, CHASSIS_CONTROL_TIME, chassis_leg_order_filter);
        first_order_filter_init(&chassis_move_init->chassis_left_control.theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_theta_l_lowpass_filter);
        first_order_filter_init(&chassis_move_init->chassis_right_control.theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_theta_l_lowpass_filter);
        first_order_filter_init(&chassis_move_init->chassis_left_control.d_theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_d_theta_l_lowpass_filter);
        first_order_filter_init(&chassis_move_init->chassis_right_control.d_theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_d_theta_l_lowpass_filter);
        first_order_filter_init(&chassis_move_init->chassis_left_control.dd_L_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_dd_L_lowpass_filter);
        first_order_filter_init(&chassis_move_init->chassis_right_control.dd_L_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_dd_L_lowpass_filter);
        chassis_move_init->chassis_left_control.dd_L_lowpass = 0.0f;
        chassis_move_init->chassis_right_control.dd_L_lowpass = 0.0f;
        chassis_move_init->chassis_leg_filter_set.out = CHASSIS_LEG_MAX;

        // 初始化斜坡函数，时间周期为 CHASSIS_CONTROL_TIME(0.001s)
        ramp_init(&chassis_move_init->chassis_left_control.init_angle_ramp, CHASSIS_CONTROL_TIME, 2.0f * PI + 1.0f, -1.0f);
        ramp_init(&chassis_move_init->chassis_right_control.init_angle_ramp, CHASSIS_CONTROL_TIME, 2.0f * PI + 1.0f, -1.0f);

        /* 底盘默认状态 */
        chassis_move_init->state = CHASSIS_STOP;
        chassis_move_init->last_state = CHASSIS_STOP;
        chassis_move_init->pending_state = CHASSIS_NORMAL;
        chassis_move_init->posture = CHASSIS_POSTURE_DOWN;
        chassis_move_init->last_posture = CHASSIS_POSTURE_DOWN;
        chassis_move_init->init_phase = CHASSIS_INIT_FOLD;
        chassis_move_init->last_request_mode = CHASSIS_MODE_RESERVED;
        chassis_move_init->chassis_leg_set = CHASSIS_NORMAL_LEG_TARGET;
        chassis_move_init->posture_stable_ticks = 0;
        chassis_move_init->normal_force_touch_ground_ticks = 0;
        chassis_reset_jump_state(chassis_move_init);

        // 更新数据
        chassis_feedback_update(chassis_move_init);
    }

    /**
     * @brief          设置底盘控制模式，主要在Chassis_Behaviour_Mode_set函数中改变
     * @param[out]     chassis_move_mode:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_set_mode(Chassis_Move *chassis_move_mode)
    {
        if (chassis_move_mode == NULL)
        {
            return;
        }

        Chassis_Behaviour_Mode_Set(chassis_move_mode); // in file "App_Chassis_Behaviour.c"
    }

    /**
     * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
     * @param[out]     chassis_move_transit:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_mode_change_control_transit(Chassis_Move *chassis_move_transit)
    {
        if (chassis_move_transit == NULL)
        {
            return;
        }

        if (chassis_move_transit->last_state == chassis_move_transit->state)
        {
            return;
        }

        chassis_move_transit->normal_force_touch_ground_ticks =
            (chassis_move_transit->state == CHASSIS_NORMAL)
                ? CHASSIS_NORMAL_FORCE_TOUCH_GROUND_TICKS
                : 0U;

        if (chassis_is_balancing_state(chassis_move_transit->state) &&
            (chassis_move_transit->last_state == CHASSIS_STOP ||
             chassis_move_transit->last_state == CHASSIS_FLIP ||
             chassis_move_transit->last_state == CHASSIS_INIT))
        {
            chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel) - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l) / (2 * DRIVE_WHEEL_DIS);
            chassis_move_transit->chassis_yaw = chassis_move_transit->chassis_gimbal_data->chassis_relative_angle;

            chassis_move_transit->chassis_v_set = 0.0f;
            chassis_move_transit->chassis_d_yaw_set = 0.0f;
            chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
            chassis_move_transit->chassis_yaw_err = 0.0f;
            chassis_move_transit->chassis_x_set += CHASSIS_X_BACK;
        }
        else if (chassis_move_transit->state == CHASSIS_JUMP)
        {
            // chassis_move_transit->chassis_v_set = 0.0f;
            // chassis_move_transit->chassis_x_set = chassis_move_transit->x_filter;
            chassis_move_transit->chassis_d_yaw_set = 0.0f;
            chassis_move_transit->chassis_yaw_err = 0.0f;
            chassis_move_transit->chassis_leg_set = CHASSIS_JUMP_TAKEOFF_TARGET;
            chassis_move_transit->chassis_leg_filter_set.out = CHASSIS_JUMP_TAKEOFF_TARGET;
            chassis_move_transit->jump_phase = CHASSIS_JUMP_TAKEOFF;
            chassis_move_transit->jump_phase_ticks = 0;
        }
        else if (chassis_move_transit->state == CHASSIS_INIT || chassis_move_transit->state == CHASSIS_FLIP)
        {
            chassis_move_transit->init_phase = CHASSIS_INIT_FOLD;

            fp32 left_raw = chassis_move_transit->chassis_left_control.theta_l;
            fp32 right_raw = chassis_move_transit->chassis_right_control.theta_l;
            chassis_move_transit->chassis_left_control.init_angle_ramp.out = left_raw < 0.0f ? left_raw + 2.0f * PI : left_raw;
            chassis_move_transit->chassis_right_control.init_angle_ramp.out = right_raw < 0.0f ? right_raw + 2.0f * PI : right_raw;

            chassis_move_transit->chassis_left_control.init_angle_ramp.max_value = 2.0f * PI + 1.0f;
            chassis_move_transit->chassis_left_control.init_angle_ramp.min_value = -1.0f;
            chassis_move_transit->chassis_right_control.init_angle_ramp.max_value = 2.0f * PI + 1.0f;
            chassis_move_transit->chassis_right_control.init_angle_ramp.min_value = -1.0f;

            PID_clear(&chassis_move_transit->chassis_left_control.init_leg_angle_pid);
            PID_clear(&chassis_move_transit->chassis_right_control.init_leg_angle_pid);
            PID_clear(&chassis_move_transit->chassis_left_control.leg_pid_control);
            PID_clear(&chassis_move_transit->chassis_right_control.leg_pid_control);

            chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel) - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l) / (2 * DRIVE_WHEEL_DIS);
            chassis_move_transit->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].total_pos + chassis_move_transit->chassis_wheel[1].total_pos) - chassis_move_transit->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) + chassis_move_transit->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l)) / (2 * DRIVE_WHEEL_DIS);

            chassis_move_transit->chassis_v_set = 0.0f;
            chassis_move_transit->chassis_d_yaw_set = 0.0f;
            chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
            chassis_move_transit->chassis_yaw_err = 0.0f;
            const fp32 leg_length_avg =
                0.5f * (chassis_move_transit->chassis_left_control.wbr_control.L +
                        chassis_move_transit->chassis_right_control.wbr_control.L);
            const fp32 leg_length_init = float_constrain(leg_length_avg, CHASSIS_LEG_MIN, CHASSIS_LEG_MAX);
            chassis_move_transit->chassis_leg_set = leg_length_init;
            chassis_move_transit->chassis_leg_filter_set.out = leg_length_init;
            chassis_move_transit->posture_stable_ticks = 0;
            chassis_reset_jump_state(chassis_move_transit);
        }
        else if (chassis_move_transit->state == CHASSIS_STOP)
        {
            chassis_move_transit->pending_state = CHASSIS_NORMAL;
            chassis_move_transit->init_phase = CHASSIS_INIT_FOLD;
            chassis_move_transit->chassis_leg_set = CHASSIS_LEG_MAX;
            chassis_move_transit->chassis_leg_filter_set.out = CHASSIS_LEG_MAX;
            chassis_move_transit->posture_stable_ticks = 0;
            chassis_reset_jump_state(chassis_move_transit);
        }
    }

    /**
     * @brief          得到底盘控制设置值, 三自由度上的控制设置值是通过chassis_behaviour_control_set函数设置的
     *                 根据三自由度上的控制设置值，计算得到步兵的三个自由度上的速度设置值
     * @param[out]     chassis_move_control:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_set_contorl(Chassis_Move *chassis_move_control)
    {
        if (chassis_move_control == NULL)
        {
            return;
        }

        /**
         * PART1.  本部分代码获取控制设定值，并根据当前行为状态计算机器人速度/角速度/腿长目标
         */

        static fp32 chassis_v_set = 0.0f, chassis_yaw_set = 0.0f, chassis_leg_set = 0.0f, chassis_d_yaw_set = 0.0f;

        /* 获取三个控制设置值 */
        chassis_behaviour_control_set(&chassis_v_set, &chassis_yaw_set, &chassis_d_yaw_set, &chassis_leg_set, chassis_move_control);

        chassis_move_control->chassis_d_yaw_kinematic = chassis_calc_kinematic_d_yaw(chassis_move_control);
        chassis_move_control->chassis_d_yaw_imu = chassis_get_imu_d_yaw(chassis_move_control);
        chassis_move_control->chassis_d_yaw_diff = chassis_move_control->chassis_d_yaw_imu - chassis_move_control->chassis_d_yaw_kinematic;

        if (chassis_is_yaw_lqr_state(chassis_move_control->state))
        {
            chassis_move_control->chassis_d_yaw = chassis_select_d_yaw_feedback(chassis_move_control);

            chassis_move_control->chassis_yaw = chassis_move_control->chassis_gimbal_data->chassis_relative_angle;
            if (chassis_is_small_gyro_active(chassis_move_control) &&
                chassis_should_reset_small_gyro_translation(chassis_move_control))
            {
                // 小陀螺原地时，用 v_set 反向补偿当前 v_filter 扰动；x 保持当前位置。
                // chassis_move_control->chassis_v_set = chassis_calc_small_gyro_v_compensation(chassis_move_control);
                // chassis_move_control->chassis_x_set = chassis_move_control->x_filter;
            }
            else if (chassis_is_small_gyro_active(chassis_move_control))
            {
                // 小陀螺移动时，将标量 v_tmp 按期望平移方向投影为时变 v_set。
                chassis_move_control->chassis_v_set = chassis_calc_small_gyro_move_v_set(chassis_move_control, chassis_v_set);
                chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
                const fp32 MAX_X_ERR = 2.0f;
                if (chassis_move_control->chassis_x_set - chassis_move_control->x_filter > MAX_X_ERR)
                {
                    chassis_move_control->chassis_x_set = chassis_move_control->x_filter + MAX_X_ERR;
                }
                else if (chassis_move_control->chassis_x_set - chassis_move_control->x_filter < -MAX_X_ERR)
                {
                    chassis_move_control->chassis_x_set = chassis_move_control->x_filter - MAX_X_ERR;
                }
            }
            else
            {
                chassis_move_control->chassis_v_set = chassis_v_set;
                chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
                const fp32 MAX_X_ERR = 2.0f;
                if (chassis_move_control->chassis_x_set - chassis_move_control->x_filter > MAX_X_ERR)
                {
                    chassis_move_control->chassis_x_set = chassis_move_control->x_filter + MAX_X_ERR;
                }
                else if (chassis_move_control->chassis_x_set - chassis_move_control->x_filter < -MAX_X_ERR)
                {
                    chassis_move_control->chassis_x_set = chassis_move_control->x_filter - MAX_X_ERR;
                }
            }
            chassis_move_control->chassis_yaw_set = chassis_yaw_set;
            chassis_move_control->chassis_yaw_err = shortest_angle_error(chassis_move_control->chassis_yaw_set, chassis_move_control->chassis_yaw);
            chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
            chassis_move_control->chassis_leg_set = chassis_leg_set;
        }
        else if (chassis_is_init_like_state(chassis_move_control->state))
        {

            chassis_move_control->chassis_d_yaw = chassis_select_d_yaw_feedback(chassis_move_control);
            chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos) - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l)) / (2 * DRIVE_WHEEL_DIS);

            chassis_move_control->chassis_v_set = 0.0f;
            chassis_move_control->chassis_x_set = chassis_move_control->x_filter;
            chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
            chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
            chassis_move_control->chassis_leg_set = chassis_leg_set;
        }
        else if (chassis_move_control->state == CHASSIS_STOP)
        {
            chassis_move_control->chassis_d_yaw = chassis_select_d_yaw_feedback(chassis_move_control);
            chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos) - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l)) / (2 * DRIVE_WHEEL_DIS);

            chassis_move_control->chassis_v_set = chassis_v_set;
            chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
            chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
            chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
            chassis_move_control->chassis_leg_set = chassis_leg_set;
        }
    }

    /**
     * @brief          底盘测量数据更新，主要进行底盘运动学正解算，得到机器人三自由度上的速度
     * @param[out]     chassis_move_update:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_feedback_update(Chassis_Move *chassis_move_update)
    {
        if (chassis_move_update == NULL)
        {
            return;
        }

        // dwt计时器更新
        chassis_move_update->dt = DWT_GetDeltaT(&chassis_move_update->chassis_dwt_count);

        // 轮毂电机数据更新 车轮累计位移 = 编码器累计脉冲数 × 「1 个脉冲对应的转动角度」
        chassis_move_update->chassis_wheel[0].total_pos = -(fp32)chassis_move_update->chassis_wheel[0].chassis_wheel_measure->total_ecd * 17 * PI / 1097594;
        chassis_move_update->chassis_wheel[1].total_pos = (fp32)chassis_move_update->chassis_wheel[1].chassis_wheel_measure->total_ecd * 17 * PI / 1097594;
        chassis_move_update->chassis_wheel[0].vel = -(fp32)chassis_move_update->chassis_wheel[0].chassis_wheel_measure->speed_rpm * 17 * PI / 8040;
        chassis_move_update->chassis_wheel[1].vel = (fp32)chassis_move_update->chassis_wheel[1].chassis_wheel_measure->speed_rpm * 17 * PI / 8040;
        chassis_move_update->chassis_wheel[0].tor = (fp32)chassis_move_update->chassis_wheel[0].chassis_wheel_measure->given_current * 3.03853e-4f;
        chassis_move_update->chassis_wheel[1].tor = -(fp32)chassis_move_update->chassis_wheel[1].chassis_wheel_measure->given_current * 3.03853e-4f;

        // 关节角度更新
        chassis_move_update->chassis_left_control.wbr_control.phi1 = rad_format(chassis_move_update->chassis_joint[0].chassis_joint_measure->pos);
        chassis_move_update->chassis_left_control.wbr_control.phi2 = rad_format(-chassis_move_update->chassis_joint[1].chassis_joint_measure->pos);
        chassis_move_update->chassis_right_control.wbr_control.phi1 = rad_format(-chassis_move_update->chassis_joint[2].chassis_joint_measure->pos);
        chassis_move_update->chassis_right_control.wbr_control.phi2 = rad_format(chassis_move_update->chassis_joint[3].chassis_joint_measure->pos);

        // 根据轮毂电机编码器角度和速度计算自然坐标系下机器人水平方向移动距离和速度
        chassis_move_update->v_real = WHEEL_RADIUS * (chassis_move_update->chassis_wheel[0].vel + chassis_move_update->chassis_wheel[1].vel) / 2;
        chassis_move_update->x_real = WHEEL_RADIUS * (chassis_move_update->chassis_wheel[0].total_pos + chassis_move_update->chassis_wheel[1].total_pos) / 2;

        // 卡尔曼滤波器测量值更新
        chassis_move_update->vaEstimateKF_F_VX[0] = 1.0f;
        chassis_move_update->vaEstimateKF_F_VX[1] = chassis_move_update->dt;
        chassis_move_update->vaEstimateKF_F_VX[2] = 0.0f;
        chassis_move_update->vaEstimateKF_F_VX[3] = 1.0f;
        memcpy(chassis_move_update->chassis_vaestimatekf_xv.F_data, chassis_move_update->vaEstimateKF_F_VX, sizeof(chassis_move_update->vaEstimateKF_F_VX));
        chassis_move_update->chassis_vaestimatekf_xv.MeasuredVector[0] = chassis_move_update->x_real;
        chassis_move_update->chassis_vaestimatekf_xv.MeasuredVector[1] = chassis_move_update->v_real;
        // 卡尔曼滤波器更新函数
        Kalman_Filter_Update(&chassis_move_update->chassis_vaestimatekf_xv);
        // 获得滤波后的速度和位移
        chassis_move_update->x_filter = chassis_move_update->chassis_vaestimatekf_xv.FilteredValue[0];
        chassis_move_update->v_filter = chassis_move_update->chassis_vaestimatekf_xv.FilteredValue[1];

        // 姿态角速度更新（yaw轴根据轮毂电机速度计算，pitch和roll根据陀螺仪直接读取）
        chassis_move_update->chassis_d_roll = *(chassis_move_update->chassis_INS_gyro + INS_GYRO_Y_ADDRESS_OFFSET);
        chassis_move_update->chassis_d_pitch = *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);

        // 机体加速度更新（世界坐标系加速度）
        chassis_move_update->chassis_accel_n_x = -*(chassis_move_update->chassis_motion_accel_n + INS_MONTION_N_ACCEL_Y_ADDRESS_OFFSET);
        chassis_move_update->chassis_accel_n_y = *(chassis_move_update->chassis_motion_accel_n + INS_MONTION_N_ACCEL_X_ADDRESS_OFFSET);
        chassis_move_update->chassis_accel_n_z = *(chassis_move_update->chassis_motion_accel_n + INS_MONTION_N_ACCEL_Z_ADDRESS_OFFSET);

        // 更新底盘姿态角（yaw轴根据电机角度计算，pitch和roll根据陀螺仪直接读取）
        chassis_move_update->chassis_pitch = *(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET);
        chassis_move_update->chassis_roll = -*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

        // 根据机体pitch/roll角度判断底盘是否处于正常水平状态
        chassis_move_update->last_posture = chassis_move_update->posture;
        if (fabs(chassis_move_update->chassis_pitch) <= CHASSIS_PITCH_LEVEL_THRESHOLD &&
            fabs(chassis_move_update->chassis_roll) <= CHASSIS_ROLL_LEVEL_THRESHOLD)
        {
            chassis_move_update->posture = CHASSIS_POSTURE_UP;
        }
        else
        {
            chassis_move_update->posture = CHASSIS_POSTURE_DOWN;
        }

        // 调用WBR计算右腿和左腿的角度以及角速度，用于LQR控制
        wbr_calc(&chassis_move_update->chassis_left_control.wbr_control, chassis_move_update->dt);
        wbr_calc(&chassis_move_update->chassis_right_control.wbr_control, chassis_move_update->dt);

        // // 根据关节角度计算雅可比矩阵 用于求解关节电机扭矩
        wbr_jacobian_calc(&chassis_move_update->chassis_left_control.wbr_control);
        wbr_jacobian_calc(&chassis_move_update->chassis_right_control.wbr_control);

        // test 使用雅可比矩阵计算dd_l
        //  chassis_move_update->chassis_left_control.wbr_control.d_L = chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->vel - chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->vel;
        //  chassis_move_update->chassis_right_control.wbr_control.d_L = -chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->vel + chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->vel;

        // chassis_move_update->chassis_left_control.wbr_control.dd_L = (chassis_move_update->chassis_left_control.wbr_control.d_L - chassis_move_update->chassis_left_control.wbr_control.d_L_p) / chassis_move_update->dt;
        // chassis_move_update->chassis_right_control.wbr_control.dd_L = (chassis_move_update->chassis_right_control.wbr_control.d_L - chassis_move_update->chassis_right_control.wbr_control.d_L_p) / chassis_move_update->dt;
        // test end

        // 计算腿部摆杆角速度和腿长变化速度
        // chassis_move_update->chassis_left_control.wbr_control.d_theta_l = chassis_move_update->chassis_left_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->vel - chassis_move_update->chassis_left_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->vel - *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);
        // chassis_move_update->chassis_right_control.wbr_control.d_theta_l = -chassis_move_update->chassis_right_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->vel + chassis_move_update->chassis_right_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->vel - *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);

        fp32 left_dd_L_raw = chassis_sanitize_dd_L(chassis_move_update->chassis_left_control.wbr_control.dd_L);
        fp32 right_dd_L_raw = chassis_sanitize_dd_L(chassis_move_update->chassis_right_control.wbr_control.dd_L);

        if (chassis_move_update->chassis_left_control.dd_L_lowpass_filter.out != chassis_move_update->chassis_left_control.dd_L_lowpass_filter.out)
        {
            chassis_move_update->chassis_left_control.dd_L_lowpass_filter.out = 0.0f;
        }
        if (chassis_move_update->chassis_right_control.dd_L_lowpass_filter.out != chassis_move_update->chassis_right_control.dd_L_lowpass_filter.out)
        {
            chassis_move_update->chassis_right_control.dd_L_lowpass_filter.out = 0.0f;
        }

        first_order_filter_cali(&chassis_move_update->chassis_left_control.dd_L_lowpass_filter, left_dd_L_raw);
        first_order_filter_cali(&chassis_move_update->chassis_right_control.dd_L_lowpass_filter, right_dd_L_raw);
        chassis_move_update->chassis_left_control.dd_L_lowpass = chassis_sanitize_dd_L(chassis_move_update->chassis_left_control.dd_L_lowpass_filter.out);
        chassis_move_update->chassis_right_control.dd_L_lowpass = chassis_sanitize_dd_L(chassis_move_update->chassis_right_control.dd_L_lowpass_filter.out);

        // 腿部角度卡尔曼滤波参数
        chassis_move_update->vaEstimateKF_F_theta[0] = 1.0f;
        chassis_move_update->vaEstimateKF_F_theta[1] = chassis_move_update->dt;
        chassis_move_update->vaEstimateKF_F_theta[2] = 0.0f;
        chassis_move_update->vaEstimateKF_F_theta[3] = 1.0f;
        chassis_update_leg_angle_signals(&chassis_move_update->chassis_left_control, chassis_move_update->vaEstimateKF_F_theta);
        chassis_update_leg_angle_signals(&chassis_move_update->chassis_right_control, chassis_move_update->vaEstimateKF_F_theta);

        // 实际左腿转矩
        chassis_move_update->chassis_left_control.wbr_control.Tbl_r = (chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[1].chassis_joint_measure->tor + chassis_move_update->chassis_left_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->tor) / (chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_left_control.wbr_control.J[1][1] - chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_left_control.wbr_control.J[1][0]);
        // 实际右腿转矩
        chassis_move_update->chassis_right_control.wbr_control.Tbl_r = -(chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[3].chassis_joint_measure->tor + chassis_move_update->chassis_right_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->tor) / (chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_right_control.wbr_control.J[1][1] - chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_right_control.wbr_control.J[1][0]);

        // 实际左腿支持力
        chassis_move_update->chassis_left_control.wbr_control.Fbl_r = (chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->tor + chassis_move_update->chassis_left_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[0].chassis_joint_measure->tor) / (chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_left_control.wbr_control.J[1][1] - chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_left_control.wbr_control.J[1][0]);

        // 实际右腿支持力
        chassis_move_update->chassis_right_control.wbr_control.Fbl_r = -(chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->tor + chassis_move_update->chassis_right_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[2].chassis_joint_measure->tor) / (chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_right_control.wbr_control.J[1][1] - chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_right_control.wbr_control.J[1][0]);

        /* 以下代码需要机构测量 */
        //  // 机体到腿部质心的距离
        // chassis_move_update->chassis_left_control.l_b = chassis_move_update->chassis_left_control.wbr_control.L * L_BI_SLOPE + L_BI_INTERCEPT;
        // chassis_move_update->chassis_right_control.l_b = chassis_move_update->chassis_right_control.wbr_control.L * L_BI_SLOPE + L_BI_INTERCEPT;
        //
        //  // 腿部转动惯量
        // chassis_move_update->chassis_left_control.i_l = chassis_move_update->chassis_left_control.wbr_control.L * I_L_SLOPE + I_L_INTERCEPT;
        // chassis_move_update->chassis_right_control.i_l = chassis_move_update->chassis_right_control.wbr_control.L * I_L_SLOPE + I_L_INTERCEPT;

        chassis_move_update->chassis_left_control.eta = 1.0f - 2.0f * chassis_move_update->chassis_left_control.l_b / chassis_move_update->chassis_left_control.wbr_control.L;
        chassis_move_update->chassis_right_control.eta = 1.0f - 2.0f * chassis_move_update->chassis_right_control.l_b / chassis_move_update->chassis_right_control.wbr_control.L;

        // 限位支持力
        fp32 left_leg_limit_fd, right_leg_limit_fd;
        if (chassis_move_update->chassis_left_control.wbr_control.L <= CHASSIS_LEG_MIN + 0.02f)
        {
            left_leg_limit_fd = 30.0f;
        }
        else
        {
            left_leg_limit_fd = 0.0f;
        }
        if (chassis_move_update->chassis_right_control.wbr_control.L <= CHASSIS_LEG_MIN + 0.02f)
        {
            right_leg_limit_fd = 30.0f;
        }
        else
        {
            right_leg_limit_fd = 0.0f;
        }
        // 左轮地面支持力
        chassis_move_update->chassis_left_control.Fwn = chassis_move_update->chassis_left_control.wbr_control.Fbl_r * arm_cos_f32(chassis_move_update->chassis_left_control.wbr_control.theta_l) + MASS_OF_LEG * (GRAVITY_ACCELERATION + chassis_move_update->chassis_accel_n_z - (1 - chassis_move_update->chassis_left_control.eta) * chassis_move_update->chassis_left_control.dd_L_lowpass * arm_cos_f32(chassis_move_update->chassis_left_control.wbr_control.theta_l)) + left_leg_limit_fd;

        // 右轮地面支持力
        chassis_move_update->chassis_right_control.Fwn = chassis_move_update->chassis_right_control.wbr_control.Fbl_r * arm_cos_f32(chassis_move_update->chassis_right_control.wbr_control.theta_l) + MASS_OF_LEG * (GRAVITY_ACCELERATION + chassis_move_update->chassis_accel_n_z - (1 - chassis_move_update->chassis_right_control.eta) * chassis_move_update->chassis_right_control.dd_L_lowpass * arm_cos_f32(chassis_move_update->chassis_right_control.wbr_control.theta_l)) + right_leg_limit_fd;

        // 计算矩阵K，A，B
        decompose_fitted_matrix_to_K(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_K, LQR_K);
        //    decompose_fitted_matrix_to_A(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_A, LQR_A);
        //    decompose_fitted_matrix_to_B(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_B, LQR_B);

        // mpc状态预测，预测下一时刻轮子状态并对轮毂电机力矩进行补偿(目前还是定腿长控制)
        // 速度前正后负
        // 腿后伸是加速，对应腿负数加速
        // 机体下是正，促进加速
        // 轮子正向扭矩为加速
        chassis_move_update->chassis_left_control.v_real_n = chassis_move_update->v_real + (LQR_A[1][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l + LQR_A[1][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l + LQR_A[1][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt + (-LQR_B[1][0] * chassis_move_update->chassis_wheel[0].tor - LQR_B[1][1] * chassis_move_update->chassis_wheel[1].tor + LQR_B[1][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r + LQR_B[1][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

        chassis_move_update->chassis_right_control.v_real_n = chassis_move_update->v_real + (LQR_A[1][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l + LQR_A[1][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l + LQR_A[1][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt + (-LQR_B[1][0] * chassis_move_update->chassis_wheel[0].tor - LQR_B[1][1] * chassis_move_update->chassis_wheel[1].tor + LQR_B[1][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r + LQR_B[1][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

        chassis_move_update->chassis_left_control.chassis_d_yaw_n = chassis_move_update->chassis_d_yaw + (LQR_A[3][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l + LQR_A[3][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l + LQR_A[3][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt + (-LQR_B[3][0] * chassis_move_update->chassis_wheel[0].tor - LQR_B[3][1] * chassis_move_update->chassis_wheel[1].tor + LQR_B[3][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r + LQR_B[3][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

        chassis_move_update->chassis_right_control.chassis_d_yaw_n = chassis_move_update->chassis_d_yaw + (LQR_A[3][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l + LQR_A[3][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l + LQR_A[3][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt + (-LQR_B[3][0] * chassis_move_update->chassis_wheel[0].tor - LQR_B[3][1] * chassis_move_update->chassis_wheel[1].tor + LQR_B[3][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r + LQR_B[3][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

        // 驱动轮角速度预测
        chassis_move_update->chassis_left_control.d_theta_w_n = (chassis_move_update->chassis_left_control.v_real_n - DRIVE_WHEEL_DIS * chassis_move_update->chassis_left_control.chassis_d_yaw_n) / WHEEL_RADIUS;
        chassis_move_update->chassis_right_control.d_theta_w_n = (chassis_move_update->chassis_right_control.v_real_n + DRIVE_WHEEL_DIS * chassis_move_update->chassis_right_control.chassis_d_yaw_n) / WHEEL_RADIUS;
        // 驱动轮力矩补偿
        chassis_move_update->chassis_left_control.Tw_adapt = chassis_move_update->K * (chassis_move_update->chassis_left_control.d_theta_w_n - chassis_move_update->chassis_wheel[0].vel);
        chassis_move_update->chassis_right_control.Tw_adapt = chassis_move_update->K * (chassis_move_update->chassis_right_control.d_theta_w_n - chassis_move_update->chassis_wheel[1].vel);
    }

    /**
     * @brief          离地检查识别
     * @param          chassis_move_prediction   指向底盘控制结构体的指针，包含底盘的状态和参数信息
     * @retval         无
     */
    static void chassis_interference_prevention(Chassis_Move *chassis_move_prediction)
    {
        if (chassis_move_prediction->state == CHASSIS_STOP ||
            chassis_move_prediction->state == CHASSIS_FLIP ||
            chassis_move_prediction->state == CHASSIS_INIT ||
            chassis_move_prediction->state == CHASSIS_STEP_UP)
        {
            chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
            chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
            return;
        }

        if (chassis_move_prediction->state == CHASSIS_NORMAL &&
            chassis_move_prediction->normal_force_touch_ground_ticks > 0U)
        {
            chassis_move_prediction->normal_force_touch_ground_ticks--;
            chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
            chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
            return;
        }

        // 采用离地/落地双阈值迟滞，避免支持力在单阈值附近抖动导致状态翻转
        chassis_move_prediction->chassis_left_control.chassis_off_ground_detection =
            chassis_update_off_ground_detection_state(
                chassis_move_prediction->chassis_left_control.chassis_off_ground_detection,
                chassis_move_prediction->chassis_left_control.Fwn);

        chassis_move_prediction->chassis_right_control.chassis_off_ground_detection =
            chassis_update_off_ground_detection_state(
                chassis_move_prediction->chassis_right_control.chassis_off_ground_detection,
                chassis_move_prediction->chassis_right_control.Fwn);

        if (chassis_move_prediction->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND && chassis_move_prediction->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND)
        {
            // 数据不再更新，保持为离地前的那一刻
            chassis_move_prediction->v_real = chassis_move_prediction->v_real_last;
            chassis_move_prediction->x_real = chassis_move_prediction->x_real_last;
            chassis_move_prediction->v_filter = chassis_move_prediction->v_filter_last;
            chassis_move_prediction->chassis_yaw = chassis_move_prediction->chassis_yaw_p;
            chassis_move_prediction->chassis_d_yaw = chassis_move_prediction->chassis_d_yaw_p;
        }
    }

    /**
     * @brief          LQR力矩拆解和功率限制控制，获得功率控制后的所有力矩
     * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_lqr_power_control(Chassis_Move *chassis_move_control_loop)
    {
        if (chassis_move_control_loop == NULL)
        {
            return;
        }

        // ------------------- LQR 力矩拆解与功率分配 -------------------

        // 1. 计算状态误差
        fp32 x_err = -chassis_move_control_loop->x_filter + chassis_move_control_loop->chassis_x_set;
        fp32 v_err = -chassis_move_control_loop->v_filter + chassis_move_control_loop->chassis_v_set;
        fp32 yaw_err = chassis_is_yaw_lqr_state(chassis_move_control_loop->state) ? chassis_move_control_loop->chassis_yaw_err : 0.0f;
        fp32 dyaw_err = chassis_is_yaw_lqr_state(chassis_move_control_loop->state) ? chassis_move_control_loop->chassis_d_yaw_set -
                                                                                         chassis_move_control_loop->chassis_d_yaw
                                                                                   : 0.0f;

        // 2. 提取平动和旋转的控制分量
        fp32 U_speed = LQR_K[0][0] * x_err + LQR_K[0][1] * v_err;

        // 旋转分量 (以左轮为正，右轮系数天然反号，因为 LQR_K[1][2] == -LQR_K[0][2])
        fp32 U_yaw = -LQR_K[0][2] * yaw_err - LQR_K[0][3] * dyaw_err;

        const fp32 left_theta_ctrl = chassis_select_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_THETA_FILTER_SOURCE);
        const fp32 left_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_D_THETA_FILTER_SOURCE);
        const fp32 right_theta_ctrl = chassis_select_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_THETA_FILTER_SOURCE);
        const fp32 right_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_D_THETA_FILTER_SOURCE);

        // 3. 提取维持姿态平衡的扭矩分量
        fp32 U_else_L = LQR_K[0][4] * left_theta_ctrl + LQR_K[0][5] * left_d_theta_ctrl + LQR_K[0][6] * right_theta_ctrl + LQR_K[0][7] * right_d_theta_ctrl - LQR_K[0][8] * (chassis_move_control_loop->chassis_pitch) - LQR_K[0][9] * chassis_move_control_loop->chassis_d_pitch + chassis_move_control_loop->chassis_left_control.Tw_adapt;

        fp32 U_else_R = LQR_K[1][4] * left_theta_ctrl + LQR_K[1][5] * left_d_theta_ctrl + LQR_K[1][6] * right_theta_ctrl + LQR_K[1][7] * right_d_theta_ctrl - LQR_K[1][8] * (chassis_move_control_loop->chassis_pitch) - LQR_K[1][9] * chassis_move_control_loop->chassis_d_pitch + chassis_move_control_loop->chassis_right_control.Tw_adapt;

        fp32 w_L = chassis_move_control_loop->chassis_wheel[0].vel;
        fp32 w_R = chassis_move_control_loop->chassis_wheel[1].vel;

        // 4. 功率限制器计算平动和yaw分量的衰减系数
        WL_PowerManager.Calculate_Decay(U_speed, U_yaw, U_else_L, U_else_R, w_L, w_R);

        fp32 decay_Uspeed = WL_PowerManager.getDecayUspeed();
        fp32 decay_Uyaw = WL_PowerManager.getDecayUyaw();
        // decay_Uspeed = 1.0f;
        // decay_Uyaw = 1.0f;

        // 5. 重新合成最终的安全轮毂力矩
        chassis_move_control_loop->chassis_wheel[0].wheel_T = (U_speed * decay_Uspeed) + (U_yaw * decay_Uyaw) + U_else_L;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = (U_speed * decay_Uspeed) - (U_yaw * decay_Uyaw) + U_else_R;

        // 6. 左右腿Tbl_t同样应用功率限制后的平动/yaw衰减系数
        fp32 U_speed_leg_L = LQR_K[2][0] * x_err + LQR_K[2][1] * v_err;
        fp32 U_yaw_leg_L = LQR_K[2][2] * yaw_err + LQR_K[2][3] * dyaw_err;
        fp32 U_else_leg_L = LQR_K[2][4] * left_theta_ctrl + LQR_K[2][5] * left_d_theta_ctrl + LQR_K[2][6] * right_theta_ctrl + LQR_K[2][7] * right_d_theta_ctrl - LQR_K[2][8] * (chassis_move_control_loop->chassis_pitch) - LQR_K[2][9] * chassis_move_control_loop->chassis_d_pitch;

        fp32 U_speed_leg_R = LQR_K[3][0] * x_err + LQR_K[3][1] * v_err;
        fp32 U_yaw_leg_R = LQR_K[3][2] * yaw_err + LQR_K[3][3] * dyaw_err;
        fp32 U_else_leg_R = LQR_K[3][4] * left_theta_ctrl + LQR_K[3][5] * left_d_theta_ctrl + LQR_K[3][6] * right_theta_ctrl + LQR_K[3][7] * right_d_theta_ctrl - LQR_K[3][8] * (chassis_move_control_loop->chassis_pitch) - LQR_K[3][9] * chassis_move_control_loop->chassis_d_pitch;

        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t =
            (U_speed_leg_L * decay_Uspeed) + (U_yaw_leg_L * decay_Uyaw) + U_else_leg_L;

        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t =
            (U_speed_leg_R * decay_Uspeed) + (U_yaw_leg_R * decay_Uyaw) + U_else_leg_R;

        // INIT 起身站立阶段：摆杆角越大，增强腿水平输出并抑制轮子输出。
        chassis_apply_init_stand_drive_bias(chassis_move_control_loop);

#if CHASSIS_VERTICAL_SUPPORT_ONLY_MODE
        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
#endif
    }

    /**
     * @brief          控制循环，根据控制设定值，PID控制器计算电机电流值，进行输出控制
     *                 将"chassis_set_contorl"中得到的三自由度上的设定速度进行运动学逆解算，得到电机设定转速后进行PID运算
     * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
     * @retval         none
     */
    static void chassis_control_loop(Chassis_Move *chassis_move_control_loop)
    {
        /* 本部分代码包括以下阶段：
         * 1. 基础LQR与功率限制
         * 2. INIT/UP等特殊阶段控制
         * 3. 支持力和关节输出计算
         * 4. 输出限幅和历史反馈保存
         * */

        if (chassis_move_control_loop == NULL)
        {
            return;
        }

        if (chassis_move_control_loop->state == CHASSIS_STOP)
        {
            chassis_zero_output(chassis_move_control_loop);
            chassis_save_last_feedback(chassis_move_control_loop);
            return;
        }

        if (chassis_move_control_loop->state == CHASSIS_FLIP)
        {
            chassis_flip_control(chassis_move_control_loop);
            chassis_limit_output(chassis_move_control_loop);
            chassis_save_last_feedback(chassis_move_control_loop);
            return;
        }

        chassis_update_step_up_phase(chassis_move_control_loop);

        if (chassis_move_control_loop->state == CHASSIS_STEP_UP &&
            chassis_move_control_loop->step_up_phase == STEP_UP_SWING)
        {
            chassis_step_up_swing_control(chassis_move_control_loop);
            chassis_limit_output(chassis_move_control_loop);
            chassis_save_last_feedback(chassis_move_control_loop);
            return;
        }

        /* 此函数已进行：功率控制+所有力矩计算 */
        /* 后方的所有函数和处理均是作为支持力计算和其他附加功能的实现 */
        chassis_lqr_power_control(chassis_move_control_loop);

        //chassis_update_jump_phase(chassis_move_control_loop);
        chassis_init_standup(chassis_move_control_loop);
        chassis_calc_support_force(chassis_move_control_loop);

        chassis_apply_joint_output(chassis_move_control_loop);
        chassis_limit_output(chassis_move_control_loop);
        chassis_save_last_feedback(chassis_move_control_loop);
    }

    static void chassis_zero_output(Chassis_Move *chassis_move_control_loop)
    {
        chassis_move_control_loop->chassis_joint[0].joint_T = 0.0f;
        chassis_move_control_loop->chassis_joint[1].joint_T = 0.0f;
        chassis_move_control_loop->chassis_joint[2].joint_T = 0.0f;
        chassis_move_control_loop->chassis_joint[3].joint_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
    }

    static void chassis_flip_control(Chassis_Move *chassis_move_control_loop)
    {
        if (chassis_move_control_loop->state != CHASSIS_FLIP ||
            chassis_move_control_loop->posture == CHASSIS_POSTURE_UP)
        {
            return;
        }

        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;

        fp32 direction = (chassis_move_control_loop->chassis_pitch > 0.0f) ? -1.0f : 1.0f;
        fp32 rotate_speed = CHASSIS_INIT_LEVEL_ROTATE_SPEED;

        fp32 left_theta_raw = chassis_move_control_loop->chassis_left_control.theta_l;
        fp32 right_theta_raw = chassis_move_control_loop->chassis_right_control.theta_l;
        fp32 left_theta_360 = left_theta_raw < 0.0f ? left_theta_raw + 2.0f * PI : left_theta_raw;
        fp32 right_theta_360 = right_theta_raw < 0.0f ? right_theta_raw + 2.0f * PI : right_theta_raw;
        const fp32 left_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_D_THETA_FILTER_SOURCE);
        const fp32 right_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_D_THETA_FILTER_SOURCE);

        fp32 left_target = chassis_move_control_loop->chassis_left_control.init_angle_ramp.out +
                           direction * CHASSIS_INIT_LEVEL_ANGLE_STEP;
        fp32 right_target = chassis_move_control_loop->chassis_right_control.init_angle_ramp.out +
                            direction * CHASSIS_INIT_LEVEL_ANGLE_STEP;

        if (direction > 0.0f)
        {
            chassis_move_control_loop->chassis_left_control.init_angle_ramp.max_value = left_target;
            chassis_move_control_loop->chassis_right_control.init_angle_ramp.max_value = right_target;
            ramp_calc(&chassis_move_control_loop->chassis_left_control.init_angle_ramp, rotate_speed);
            ramp_calc(&chassis_move_control_loop->chassis_right_control.init_angle_ramp, rotate_speed);
        }
        else
        {
            chassis_move_control_loop->chassis_left_control.init_angle_ramp.min_value = left_target;
            chassis_move_control_loop->chassis_right_control.init_angle_ramp.min_value = right_target;
            ramp_calc(&chassis_move_control_loop->chassis_left_control.init_angle_ramp, -rotate_speed);
            ramp_calc(&chassis_move_control_loop->chassis_right_control.init_angle_ramp, -rotate_speed);
        }

        fp32 left_error = chassis_move_control_loop->chassis_left_control.init_angle_ramp.out - left_theta_360;
        fp32 right_error = chassis_move_control_loop->chassis_right_control.init_angle_ramp.out - right_theta_360;

        fp32 left_torque = PID_Calc(&chassis_move_control_loop->chassis_left_control.init_leg_angle_pid,
                                    0.0f, left_error);
        fp32 right_torque = PID_Calc(&chassis_move_control_loop->chassis_right_control.init_leg_angle_pid,
                                     0.0f, right_error);

        left_torque = float_constrain(left_torque, -CHASSIS_INIT_LEVEL_TORQUE_LIMIT, CHASSIS_INIT_LEVEL_TORQUE_LIMIT);
        right_torque = float_constrain(right_torque, -CHASSIS_INIT_LEVEL_TORQUE_LIMIT, CHASSIS_INIT_LEVEL_TORQUE_LIMIT);

        fp32 leg_progress_error = direction * (left_theta_360 - right_theta_360);
        fp32 lead_torque_ratio = 1.0f - fabs(leg_progress_error) / CHASSIS_INIT_LEVEL_SYNC_ANGLE;
        lead_torque_ratio = float_constrain(lead_torque_ratio, CHASSIS_INIT_LEVEL_SYNC_MIN_RATIO, 1.0f);
        if (leg_progress_error > 0.0f)
        {
            left_torque *= lead_torque_ratio;
        }
        else if (leg_progress_error < 0.0f)
        {
            right_torque *= lead_torque_ratio;
        }

        if (fabs(left_d_theta_ctrl) > CHASSIS_INIT_LEVEL_SPEED_LIMIT &&
            left_torque * left_d_theta_ctrl > 0.0f)
        {
            left_torque = 0.0f;
        }

        if (fabs(right_d_theta_ctrl) > CHASSIS_INIT_LEVEL_SPEED_LIMIT &&
            right_torque * right_d_theta_ctrl > 0.0f)
        {
            right_torque = 0.0f;
        }

        const fp32 leg_extend_target =
            float_constrain(chassis_move_control_loop->chassis_leg_set, CHASSIS_LEG_MIN, CHASSIS_LEG_MAX);
        const fp32 left_leg_extend_force =
            chassis_calc_leg_extend_force(&chassis_move_control_loop->chassis_left_control, leg_extend_target);
        const fp32 right_leg_extend_force =
            chassis_calc_leg_extend_force(&chassis_move_control_loop->chassis_right_control, leg_extend_target);
        const fp32 left_extend_torque_front = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * left_leg_extend_force;
        const fp32 left_extend_torque_back = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * left_leg_extend_force;
        const fp32 right_extend_torque_front = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * right_leg_extend_force;
        const fp32 right_extend_torque_back = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * right_leg_extend_force;

        // 按机体pitch倾斜方向分别控制左右腿，直到feedback_update把posture更新为UP
        chassis_move_control_loop->chassis_joint[0].joint_T = -left_torque + left_extend_torque_front;
        chassis_move_control_loop->chassis_joint[1].joint_T = -left_torque + left_extend_torque_back;
        chassis_move_control_loop->chassis_joint[2].joint_T = right_torque + right_extend_torque_front;
        chassis_move_control_loop->chassis_joint[3].joint_T = right_torque + right_extend_torque_back;
    }

    static void chassis_update_step_up_phase(Chassis_Move *chassis_move_control_loop)
    {
        if (chassis_move_control_loop->state != CHASSIS_STEP_UP) return;

        if (chassis_move_control_loop->step_up_phase == STEP_UP_EXTEND) {
            // 腿长到位后切换到检测阶段
            if (fabsf(chassis_move_control_loop->chassis_left_control.wbr_control.L - CHASSIS_LEG_2_TARGET) < 0.03f)
                chassis_move_control_loop->step_up_phase = STEP_UP_DETECT;
        }
        else if (chassis_move_control_loop->step_up_phase == STEP_UP_DETECT) {
            // 碰撞检测：角度被向后推 + 扭矩异常
            if ((fabsf(chassis_move_control_loop->chassis_left_control.theta_l) > STEP_UP_ANGLE_THRESHOLD
                && chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t > STEP_UP_TORQUE_THRESHOLD)
                ||(fabsf(chassis_move_control_loop->chassis_right_control.theta_l) > STEP_UP_ANGLE_THRESHOLD
                && chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t > STEP_UP_TORQUE_THRESHOLD))
            {
                chassis_move_control_loop->step_up_phase = STEP_UP_SWING;
                // 记录当前角度准备摆腿
                chassis_move_control_loop->chassis_left_control.init_angle_ramp.out = (chassis_move_control_loop->chassis_left_control.theta_l < 0) ? chassis_move_control_loop->chassis_left_control.theta_l + 2*PI : chassis_move_control_loop->chassis_left_control.theta_l;
                chassis_move_control_loop->chassis_right_control.init_angle_ramp.out = (chassis_move_control_loop->chassis_right_control.theta_l < 0) ? chassis_move_control_loop->chassis_right_control.theta_l + 2*PI : chassis_move_control_loop->chassis_right_control.theta_l;
            }
        }
    }

    static void chassis_step_up_swing_control(Chassis_Move *chassis_move_control_loop)
    {
        // 强制关闭轮子动力
        chassis_move_control_loop->chassis_wheel[0].wheel_T = chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;

        fp32 target_360 = STEP_UP_SWING_TARGET_360;
        chassis_move_control_loop->chassis_left_control.init_angle_ramp.min_value = target_360;
        ramp_calc(&chassis_move_control_loop->chassis_left_control.init_angle_ramp, -STEP_UP_SWING_SPEED);
        chassis_move_control_loop->chassis_right_control.init_angle_ramp.min_value = target_360;
        ramp_calc(&chassis_move_control_loop->chassis_right_control.init_angle_ramp, -STEP_UP_SWING_SPEED);

        // 计算关节力矩让腿向后甩 (逻辑参考自 chassis_init_standup)
        fp32 left_theta_360 = (chassis_move_control_loop->chassis_left_control.theta_l < 0) ? chassis_move_control_loop->chassis_left_control.theta_l + 2*PI : chassis_move_control_loop->chassis_left_control.theta_l;
        fp32 left_torque = PID_Calc(&chassis_move_control_loop->chassis_left_control.init_leg_angle_pid, left_theta_360, target_360);
        fp32 right_theta_360 = (chassis_move_control_loop->chassis_right_control.theta_l < 0) ? chassis_move_control_loop->chassis_right_control.theta_l + 2*PI : chassis_move_control_loop->chassis_right_control.theta_l;
        fp32 right_torque = PID_Calc(&chassis_move_control_loop->chassis_right_control.init_leg_angle_pid, right_theta_360, target_360);

        // 将腿部甩到身后
        chassis_move_control_loop->chassis_joint[0].joint_T = -left_torque;
        chassis_move_control_loop->chassis_joint[1].joint_T = -left_torque;
        chassis_move_control_loop->chassis_joint[2].joint_T = right_torque;
        chassis_move_control_loop->chassis_joint[3].joint_T = right_torque;

        // 到位检测：摆腿完成后直接进入原有的起身模式
        if (fabsf(left_theta_360 - target_360) < 0.05f) {
            chassis_move_control_loop->state = CHASSIS_INIT;
            chassis_move_control_loop->init_phase = CHASSIS_INIT_RETRACT; // 复用原有的正常起身流程
        }
    }

    static void chassis_update_jump_phase(Chassis_Move *chassis_move_control_loop)
    {
        if (chassis_move_control_loop->state != CHASSIS_JUMP)
        {
            chassis_reset_jump_state(chassis_move_control_loop);
            return;
        }

        chassis_move_control_loop->jump_phase_ticks++;
        const bool_t left_off_ground = chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND;
        const bool_t right_off_ground = chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND;
        const bool_t both_off_ground = left_off_ground && right_off_ground;
        const bool_t both_touch_ground = !left_off_ground && !right_off_ground;
        const bool_t legs_near_takeoff_target =
            (chassis_move_control_loop->chassis_left_control.wbr_control.L >= CHASSIS_JUMP_TAKEOFF_TARGET - 0.01f) &&
            (chassis_move_control_loop->chassis_right_control.wbr_control.L >= CHASSIS_JUMP_TAKEOFF_TARGET - 0.01f);
        const bool_t takeoff_hold_done = chassis_move_control_loop->jump_phase_ticks >= 60U;

        switch (chassis_move_control_loop->jump_phase)
        {
        case CHASSIS_JUMP_PRELOAD:
            chassis_move_control_loop->jump_phase = CHASSIS_JUMP_TAKEOFF;
            chassis_move_control_loop->jump_phase_ticks = 0;
            break;

        case CHASSIS_JUMP_TAKEOFF:
            if ((takeoff_hold_done && (both_off_ground || legs_near_takeoff_target)) ||
                chassis_move_control_loop->jump_phase_ticks >= CHASSIS_JUMP_LAND_TICKS)
            {
                chassis_move_control_loop->jump_phase = CHASSIS_JUMP_READYLAND;
                chassis_move_control_loop->jump_phase_ticks = 0;
            }
            break;

        case CHASSIS_JUMP_READYLAND:
            if ((both_touch_ground &&
                 chassis_move_control_loop->jump_phase_ticks >= CHASSIS_JUMP_LAND_TICKS &&
                 fabsf(chassis_move_control_loop->chassis_left_control.wbr_control.L - CHASSIS_JUMP_LAND_TARGET) < 0.02f &&
                 fabsf(chassis_move_control_loop->chassis_right_control.wbr_control.L - CHASSIS_JUMP_LAND_TARGET) < 0.02f) ||
                chassis_move_control_loop->jump_phase_ticks >= (CHASSIS_JUMP_LAND_TICKS * 3U))
            {
                chassis_move_control_loop->jump_phase = CHASSIS_JUMP_DONE;
                chassis_move_control_loop->jump_phase_ticks = 0;
            }
            break;

        case CHASSIS_JUMP_DONE:
        default:
            break;
        }
    }

    /**
     * @brief  轮腿落地导纳缓冲控制函数 (Admittance Landing Control)
     * @param  control_loop: 底盘控制主结构体指针
     * @retval None
     * @note   此函数内部包含静态变量，通过判断离地状态自动管理双腿的独立期望长度
     */
    void Chassis_Landing_Admittance_Control(Chassis_Move *control_loop)
    {
        static fp32 independent_leg_set_L = 0.0f;
        static fp32 independent_leg_set_R = 0.0f;
        static uint8_t init_flag = 0;

        // 上电首次运行，初始化独立腿长等于全局腿长
        if (init_flag == 0)
        {
            independent_leg_set_L = CHASSIS_LEG_MIN;
            independent_leg_set_R = CHASSIS_LEG_MIN;
            init_flag = 1;
        }

        // 获取当前双腿的离地状态
        bool is_left_off = (control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND);
        bool is_right_off = (control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND);

        if (is_left_off)
        {
            fp32 diff_L = LEG_NORMAL_SUPPORT_FORCE - control_loop->chassis_left_control.Fwn;
            independent_leg_set_L += diff_L * ADMITTANCE_EXTEND_KP;
        }
        else
        {
            // 左腿落地后恢复到正常设定长度
            if (independent_leg_set_L < control_loop->chassis_leg_set)
            {
                independent_leg_set_L += RAMP_RECOVERY_STEP;
                if (independent_leg_set_L > control_loop->chassis_leg_set)
                    independent_leg_set_L = control_loop->chassis_leg_set;
            }
            else if (independent_leg_set_L > control_loop->chassis_leg_set)
            {
                independent_leg_set_L -= RAMP_RECOVERY_STEP;
                if (independent_leg_set_L < control_loop->chassis_leg_set)
                    independent_leg_set_L = control_loop->chassis_leg_set;
            }
        }

        if (is_right_off)
        {
            fp32 diff_R = LEG_NORMAL_SUPPORT_FORCE - control_loop->chassis_right_control.Fwn;
            independent_leg_set_R += diff_R * ADMITTANCE_EXTEND_KP;
        }
        else
        {
            // 右腿落地后恢复到正常设定长度
            if (independent_leg_set_R < control_loop->chassis_leg_set)
            {
                independent_leg_set_R += RAMP_RECOVERY_STEP;
                if (independent_leg_set_R > control_loop->chassis_leg_set)
                    independent_leg_set_R = control_loop->chassis_leg_set;
            }
            else if (independent_leg_set_R > control_loop->chassis_leg_set)
            {
                independent_leg_set_R -= RAMP_RECOVERY_STEP;
                if (independent_leg_set_R < control_loop->chassis_leg_set)
                    independent_leg_set_R = control_loop->chassis_leg_set;
            }
        }

        // 安全保护：限制积分后的期望腿长不超出物理机械极限
        independent_leg_set_L = float_constrain(independent_leg_set_L, CHASSIS_LEG_MIN, 0.33);
        independent_leg_set_R = float_constrain(independent_leg_set_R, CHASSIS_LEG_MIN, 0.33);

        // 计算底层腿长PID输出 (推力Fbl_t)
        control_loop->chassis_left_control.fd_leg = Leg_PID_Calc(&control_loop->chassis_left_control.leg_pid_control, control_loop->chassis_left_control.wbr_control.L, independent_leg_set_L);
        control_loop->chassis_right_control.fd_leg = Leg_PID_Calc(&control_loop->chassis_right_control.leg_pid_control, control_loop->chassis_right_control.wbr_control.L, independent_leg_set_R);
    }

    static void chassis_calc_support_force(Chassis_Move *chassis_move_control_loop)
    {
        // roll轴PID补偿
        chassis_move_control_loop->chassis_left_control.fd_roll = PID_Calc(&chassis_move_control_loop->chassis_left_control.roll_control, chassis_move_control_loop->chassis_roll, chassis_move_control_loop->chassis_roll_set);
        chassis_move_control_loop->chassis_right_control.fd_roll = PID_Calc(&chassis_move_control_loop->chassis_right_control.roll_control, chassis_move_control_loop->chassis_roll, chassis_move_control_loop->chassis_roll_set);

        // 腿长PID
        chassis_move_control_loop->chassis_left_control.fd_leg = Leg_PID_Calc(&chassis_move_control_loop->chassis_left_control.leg_pid_control, chassis_move_control_loop->chassis_left_control.wbr_control.L, chassis_move_control_loop->chassis_leg_set);
        chassis_move_control_loop->chassis_right_control.fd_leg = Leg_PID_Calc(&chassis_move_control_loop->chassis_right_control.leg_pid_control, chassis_move_control_loop->chassis_right_control.wbr_control.L, chassis_move_control_loop->chassis_leg_set);

        // 重力补偿前馈
        chassis_move_control_loop->chassis_left_control.Fbl_gravity = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_left_control.eta * MASS_OF_LEG) * GRAVITY_ACCELERATION;
        chassis_move_control_loop->chassis_right_control.Fbl_gravity = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_right_control.eta * MASS_OF_LEG) * GRAVITY_ACCELERATION;

        // 弹簧补偿前馈
        chassis_move_control_loop->chassis_left_control.Fbl_spring = Get_FeedForward_Force(chassis_move_control_loop->chassis_left_control.wbr_control.L);
        chassis_move_control_loop->chassis_right_control.Fbl_spring = Get_FeedForward_Force(chassis_move_control_loop->chassis_right_control.wbr_control.L);

        // 侧向惯性力矩补偿前馈
        chassis_move_control_loop->chassis_left_control.Fbl_inertial = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_left_control.eta * MASS_OF_LEG) * chassis_move_control_loop->chassis_left_control.wbr_control.L * chassis_move_control_loop->chassis_d_yaw * chassis_move_control_loop->v_filter / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control_loop->chassis_right_control.Fbl_inertial = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_right_control.eta * MASS_OF_LEG) * chassis_move_control_loop->chassis_right_control.wbr_control.L * chassis_move_control_loop->chassis_d_yaw * chassis_move_control_loop->v_filter / (2 * DRIVE_WHEEL_DIS);

        if (chassis_move_control_loop->state == CHASSIS_INIT &&
            (chassis_move_control_loop->init_phase == CHASSIS_INIT_FOLD ||
             chassis_move_control_loop->init_phase == CHASSIS_INIT_RETRACT))
        {
            chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
            chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
            chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
            chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;

            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_leg + chassis_move_control_loop->chassis_left_control.Fbl_spring + 100;
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_right_control.fd_leg + chassis_move_control_loop->chassis_right_control.Fbl_spring + 100;
        }
        else if (chassis_move_control_loop->state == CHASSIS_JUMP &&
                 chassis_move_control_loop->jump_phase == CHASSIS_JUMP_TAKEOFF)
        {
            // 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_leg - CHASSIS_JUMP_TAKEOFF_FORCE_BONUS;

            // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_right_control.fd_leg - CHASSIS_JUMP_TAKEOFF_FORCE_BONUS;
        }
        else if (chassis_move_control_loop->state == CHASSIS_JUMP &&
                 chassis_move_control_loop->jump_phase == CHASSIS_JUMP_READYLAND)
        {
            // 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_leg + chassis_move_control_loop->chassis_left_control.Fbl_spring;

            // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_right_control.fd_leg + chassis_move_control_loop->chassis_right_control.Fbl_spring;
        }
        else // 正常情况下支持力处理
        {
            Chassis_Landing_Admittance_Control(chassis_move_control_loop);

            if (chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND && chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND)
            {
                chassis_move_control_loop->chassis_left_control.fd_roll = 0.0f;
                chassis_move_control_loop->chassis_right_control.fd_roll = 0.0f;
            }

            // 五连杆左右腿支持力计算
            // -重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID - 左腿roll轴补偿PID + 弹簧补偿
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_roll - chassis_move_control_loop->chassis_left_control.fd_leg - chassis_move_control_loop->chassis_left_control.Fbl_gravity - chassis_move_control_loop->chassis_left_control.Fbl_inertial;

            // -重力补偿 - 侧向惯性力矩补偿 + 右腿腿长PID + 右腿roll轴补偿PID + 弹簧补偿
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = +chassis_move_control_loop->chassis_right_control.fd_roll - chassis_move_control_loop->chassis_right_control.fd_leg - chassis_move_control_loop->chassis_right_control.Fbl_gravity + chassis_move_control_loop->chassis_right_control.Fbl_inertial;
        }
    }

    static void chassis_apply_joint_output(Chassis_Move *chassis_move_control_loop)
    {
        if (chassis_move_control_loop->state == CHASSIS_INIT &&
            chassis_move_control_loop->init_phase == CHASSIS_INIT_STAND)
        {
            // chassis_move_control_loop->chassis_wheel[0].wheel_T *= 0.0
            // chassis_move_control_loop->chassis_wheel[1].wheel_T *= 0.0f;

            // chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = LQR_K[2][4] * chassis_move_control_loop->chassis_left_control.theta_l + LQR_K[2][5] * chassis_move_control_loop->chassis_left_control.d_theta_l + LQR_K[2][6] * chassis_move_control_loop->chassis_right_control.theta_l + LQR_K[2][7] * chassis_move_control_loop->chassis_right_control.d_theta_l;

            // chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = LQR_K[3][4] * chassis_move_control_loop->chassis_left_control.theta_l + LQR_K[3][5] * chassis_move_control_loop->chassis_left_control.d_theta_l + LQR_K[3][6] * chassis_move_control_loop->chassis_right_control.theta_l + LQR_K[3][7] * chassis_move_control_loop->chassis_right_control.d_theta_l;
            // chassis_apply_leg_tbl_angle_attenuation(chassis_move_control_loop);
            //  左前关节电机力矩
            chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t - chassis_move_control_loop->chassis_left_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;
            // 左后关节电机力矩
            chassis_move_control_loop->chassis_joint[1].joint_T = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t + chassis_move_control_loop->chassis_left_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;
            // 右前关节电机力矩
            chassis_move_control_loop->chassis_joint[2].joint_T = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t + chassis_move_control_loop->chassis_right_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;
            // 右后关节电机力矩
            chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t - chassis_move_control_loop->chassis_right_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;
        }
        else if (chassis_move_control_loop->state == CHASSIS_INIT &&
                 chassis_move_control_loop->init_phase == CHASSIS_INIT_RETRACT)
        {
            chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t;
            chassis_move_control_loop->chassis_joint[1].joint_T = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t;
            chassis_move_control_loop->chassis_joint[2].joint_T = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t;
            chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t;
        }
        else if (chassis_move_control_loop->state != CHASSIS_INIT)
        {
            const bool_t left_off_ground =
                (chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND);
            const bool_t right_off_ground =
                (chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND);

            if (left_off_ground)
            {
                chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
#if CHASSIS_VERTICAL_SUPPORT_ONLY_MODE
                chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
#else
            chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t =
                LQR_K[2][4] * chassis_move_control_loop->chassis_left_control.theta_l +
                LQR_K[2][5] * chassis_move_control_loop->chassis_left_control.d_theta_l +
                LQR_K[2][6] * chassis_move_control_loop->chassis_right_control.theta_l +
                LQR_K[2][7] * chassis_move_control_loop->chassis_right_control.d_theta_l;
#endif
            }

            if (right_off_ground)
            {
                chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
#if CHASSIS_VERTICAL_SUPPORT_ONLY_MODE
                chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
#else
            chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t =
                LQR_K[3][4] * chassis_move_control_loop->chassis_left_control.theta_l +
                LQR_K[3][5] * chassis_move_control_loop->chassis_left_control.d_theta_l +
                LQR_K[3][6] * chassis_move_control_loop->chassis_right_control.theta_l +
                LQR_K[3][7] * chassis_move_control_loop->chassis_right_control.d_theta_l;
#endif
            }

            chassis_apply_leg_tbl_angle_attenuation(chassis_move_control_loop);

            // 左前关节电机力矩
            chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t - chassis_move_control_loop->chassis_left_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t; //* soft_lqr_ratio;
            // 左后关节电机力矩
            chassis_move_control_loop->chassis_joint[1].joint_T = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t + chassis_move_control_loop->chassis_left_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t; //* soft_lqr_ratio;
            // 右前关节电机力矩
            chassis_move_control_loop->chassis_joint[2].joint_T = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t + chassis_move_control_loop->chassis_right_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t; //* soft_lqr_ratio;
            // 右后关节电机力矩
            chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t - chassis_move_control_loop->chassis_right_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t; //* soft_lqr_ratio;
        }
    }

    static void chassis_limit_output(Chassis_Move *chassis_move_control_loop)
    {
        // 关节点击力矩限幅，最大值按照关节电机最大扭矩限制
        chassis_move_control_loop->chassis_wheel[0].wheel_T = float_constrain(chassis_move_control_loop->chassis_wheel[0].wheel_T, -6.3f, 6.3f);
        chassis_move_control_loop->chassis_wheel[1].wheel_T = float_constrain(chassis_move_control_loop->chassis_wheel[1].wheel_T, -6.3f, 6.3f);
        chassis_move_control_loop->chassis_joint[0].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[0].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
        chassis_move_control_loop->chassis_joint[1].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[1].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
        chassis_move_control_loop->chassis_joint[2].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[2].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
        chassis_move_control_loop->chassis_joint[3].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[3].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
    }

    static void chassis_save_last_feedback(Chassis_Move *chassis_move_control_loop)
    {
        // 数据更新，用于离地
        chassis_move_control_loop->x_real_last = chassis_move_control_loop->x_real;
        chassis_move_control_loop->v_real_last = chassis_move_control_loop->v_real;
        chassis_move_control_loop->x_filter_last = chassis_move_control_loop->x_filter;
        chassis_move_control_loop->v_filter_last = chassis_move_control_loop->v_filter;
        chassis_move_control_loop->chassis_yaw_p = chassis_move_control_loop->chassis_yaw;
        chassis_move_control_loop->chassis_d_yaw_p = chassis_move_control_loop->chassis_d_yaw;
        chassis_move_control_loop->last_state = chassis_move_control_loop->state;
    }

    // 获取弹簧弹力前馈,输入腿长获得前馈量
    fp32 Get_FeedForward_Force(fp32 leg_length_m)
    {
        // 限制范围防止计算发散
        if (leg_length_m < 0.17f)
            leg_length_m = 0.17f;
        if (leg_length_m > 0.35f)
            leg_length_m = 0.35f;

        float H = leg_length_m;
        // 3次多项式计算
        return (2563.62477 * H * H * H) +
               (-3103.49246 * H * H) +
               (1291.20462 * H) +
               (-63.14450);
    }

    /*底盘正常状态下初始化起身过程*/
    static void chassis_init_standup(Chassis_Move *chassis_init_standup)
    {
        if (chassis_init_standup == NULL)
        {
            return;
        }

        if (chassis_init_standup->state != CHASSIS_INIT)
        {
            return;
        }

        if (chassis_init_standup->last_posture != CHASSIS_POSTURE_UP &&
            chassis_init_standup->posture == CHASSIS_POSTURE_UP)
        {
            fp32 left_raw = chassis_init_standup->chassis_left_control.theta_l;
            fp32 right_raw = chassis_init_standup->chassis_right_control.theta_l;
            chassis_init_standup->chassis_left_control.init_angle_ramp.out = left_raw < 0.0f ? left_raw + 2.0f * PI : left_raw;
            chassis_init_standup->chassis_right_control.init_angle_ramp.out = right_raw < 0.0f ? right_raw + 2.0f * PI : right_raw;
            PID_clear(&chassis_init_standup->chassis_left_control.init_leg_angle_pid);
            PID_clear(&chassis_init_standup->chassis_right_control.init_leg_angle_pid);
        }

        if (chassis_init_standup->posture != CHASSIS_POSTURE_UP)
        {
            chassis_init_standup->posture_stable_ticks = 0;
            return;
        }

        if (chassis_init_standup->init_phase == CHASSIS_INIT_FOLD)
        {
            chassis_init_standup->chassis_left_control.wbr_control.Tbl_t = 0.0f;
            chassis_init_standup->chassis_right_control.wbr_control.Tbl_t = 0.0f;
            chassis_init_standup->chassis_wheel[0].wheel_T = 0.0f;
            chassis_init_standup->chassis_wheel[1].wheel_T = 0.0f;

            fp32 left_theta_raw = chassis_init_standup->chassis_left_control.theta_l;
            fp32 right_theta_raw = chassis_init_standup->chassis_right_control.theta_l;
            fp32 left_theta_360 = left_theta_raw < 0.0f ? left_theta_raw + 2.0f * PI : left_theta_raw;
            fp32 right_theta_360 = right_theta_raw < 0.0f ? right_theta_raw + 2.0f * PI : right_theta_raw;
            fp32 target_360 = CHASSIS_INIT_LEG_ANGLE_TARGET_360;
            fp32 rotate_speed = ROTATE_SPEED;

            if (chassis_init_standup->chassis_left_control.init_angle_ramp.out < target_360)
            {
                chassis_init_standup->chassis_left_control.init_angle_ramp.max_value = target_360;
                ramp_calc(&chassis_init_standup->chassis_left_control.init_angle_ramp, rotate_speed);
            }
            else
            {
                chassis_init_standup->chassis_left_control.init_angle_ramp.min_value = target_360;
                ramp_calc(&chassis_init_standup->chassis_left_control.init_angle_ramp, -rotate_speed);
            }

            // 右腿斜坡：动态设置限幅，并以设定的角速度逼近目标
            if (chassis_init_standup->chassis_right_control.init_angle_ramp.out < target_360)
            {
                chassis_init_standup->chassis_right_control.init_angle_ramp.max_value = target_360;
                ramp_calc(&chassis_init_standup->chassis_right_control.init_angle_ramp, rotate_speed);
            }
            else
            {
                chassis_init_standup->chassis_right_control.init_angle_ramp.min_value = target_360;
                ramp_calc(&chassis_init_standup->chassis_right_control.init_angle_ramp, -rotate_speed);
            }

            fp32 left_error = chassis_init_standup->chassis_left_control.init_angle_ramp.out - left_theta_360;
            fp32 right_error = chassis_init_standup->chassis_right_control.init_angle_ramp.out - right_theta_360;

            bool_t left_ramp_finished = (chassis_init_standup->chassis_left_control.init_angle_ramp.out >= target_360 - 0.05f);
            bool_t right_ramp_finished = (chassis_init_standup->chassis_right_control.init_angle_ramp.out >= target_360 - 0.05f);

            const fp32 left_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_init_standup->chassis_left_control, CHASSIS_LEFT_D_THETA_FILTER_SOURCE);
            const fp32 right_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_init_standup->chassis_right_control, CHASSIS_RIGHT_D_THETA_FILTER_SOURCE);
            bool_t left_stopped = (fabs(left_d_theta_ctrl) < 0.2f);
            bool_t right_stopped = (fabs(right_d_theta_ctrl) < 0.2f);

            bool_t left_in_zone = (left_theta_360 > 4.8f);
            bool_t right_in_zone = (right_theta_360 > 4.8f);

            bool_t left_perfect = (fabs(target_360 - left_theta_360) < CHASSIS_INIT_LEG_ANGLE_THRESHOLD);
            bool_t right_perfect = (fabs(target_360 - right_theta_360) < CHASSIS_INIT_LEG_ANGLE_THRESHOLD);
            bool_t left_stalled = (left_ramp_finished && left_stopped && left_in_zone);
            bool_t right_stalled = (right_ramp_finished && right_stopped && right_in_zone);
            bool_t left_reached = left_perfect || left_stalled;
            bool_t right_reached = right_perfect || right_stalled;

            fp32 left_torque = PID_Calc(&chassis_init_standup->chassis_left_control.init_leg_angle_pid,
                                        0.0f, left_error);
            fp32 right_torque = PID_Calc(&chassis_init_standup->chassis_right_control.init_leg_angle_pid,
                                         0.0f, right_error);

            const fp32 leg_extend_target =
                float_constrain(chassis_init_standup->chassis_leg_set, CHASSIS_LEG_MIN, CHASSIS_LEG_MAX);
            const fp32 left_leg_extend_force =
                chassis_calc_leg_extend_force(&chassis_init_standup->chassis_left_control, leg_extend_target);
            const fp32 right_leg_extend_force =
                chassis_calc_leg_extend_force(&chassis_init_standup->chassis_right_control, leg_extend_target);
            const fp32 left_extend_torque_front = -chassis_init_standup->chassis_left_control.wbr_control.J[0][0] * left_leg_extend_force;
            const fp32 left_extend_torque_back = chassis_init_standup->chassis_left_control.wbr_control.J[1][0] * left_leg_extend_force;
            const fp32 right_extend_torque_front = chassis_init_standup->chassis_right_control.wbr_control.J[0][0] * right_leg_extend_force;
            const fp32 right_extend_torque_back = -chassis_init_standup->chassis_right_control.wbr_control.J[1][0] * right_leg_extend_force;

            chassis_init_standup->chassis_joint[0].joint_T = -left_torque + left_extend_torque_front;
            chassis_init_standup->chassis_joint[1].joint_T = -left_torque + left_extend_torque_back;
            chassis_init_standup->chassis_joint[2].joint_T = right_torque + right_extend_torque_front;
            chassis_init_standup->chassis_joint[3].joint_T = right_torque + right_extend_torque_back;

            if (left_reached && right_reached)
            {
                chassis_init_standup->init_phase = CHASSIS_INIT_RETRACT;
                PID_clear(&chassis_init_standup->chassis_left_control.init_leg_angle_pid);
                PID_clear(&chassis_init_standup->chassis_right_control.init_leg_angle_pid);
            }
        }

        if (chassis_init_standup->init_phase == CHASSIS_INIT_RETRACT)
        {
            if (chassis_init_standup->chassis_left_control.wbr_control.L <= CHASSIS_INIT_RETRACT_LEG_TARGET + 0.005f &&
                chassis_init_standup->chassis_right_control.wbr_control.L <= CHASSIS_INIT_RETRACT_LEG_TARGET + 0.005f)
            {
                chassis_init_standup->init_phase = CHASSIS_INIT_STAND;
                chassis_init_standup->chassis_leg_filter_set.out = CHASSIS_INIT_RETRACT_LEG_TARGET;
                chassis_init_standup->posture_stable_ticks = 0;
            }
        }
        else if (chassis_init_standup->init_phase == CHASSIS_INIT_STAND)
        {
            if (chassis_init_standup->posture == CHASSIS_POSTURE_UP)
            {
                if ((fabs(chassis_init_standup->chassis_left_control.theta_l) < 0.2) ||
                    (fabs(chassis_init_standup->chassis_right_control.theta_l) < 0.2))
                {
                    chassis_init_standup->init_phase = CHASSIS_INIT_DONE;
                }
                // if (chassis_init_standup->posture_stable_ticks < CHASSIS_POSTURE_STABLE_TICKS)
                // {
                //     chassis_init_standup->posture_stable_ticks++;
                // }
                // else
                // {
                //     chassis_init_standup->init_phase = CHASSIS_INIT_DONE;
                // }
            }
            else
            {
                // chassis_init_standup->posture_stable_ticks = 0;
                chassis_init_standup->init_phase = CHASSIS_INIT_DONE;
            }
        }
    }

#ifdef __cplusplus
}
#endif

Chassis_Move *Get_Chassis_Move_Point(void)
{
    return &chassis_move;
}
