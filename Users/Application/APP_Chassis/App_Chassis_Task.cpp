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
    fp32 fitted_matrix_K[40][6] = {{-1.5152, 1.1725, -2.4929, -1.6736, 0.52327, 3.1789},
                                   {-2.2985, 1.9099, -3.1571, -2.6996, 0.21512, 4.076},
                                   {-3.8392, -3.8823, 3.6946, 5.312, -0.29332, -4.748},
                                   {-0.44367, -0.74096, 0.72537, 0.92273, -0.071706, -0.82195},
                                   {-1.9139, -25.382, 3.3251, 13.45, 4.5296, 2.0547},
                                   {-0.17922, -3.4137, 0.18969, 0.51918, 0.072841, 0.62664},
                                   {-1.3792, 9.7159, -34.038, -10.099, 12.448, 20.713},
                                   {-0.050932, 1.2709, -5.0042, -1.5081, 1.6577, 1.9099},
                                   {-18.989, 35.032, 22.479, -22.633, -36.187, -3.1844},
                                   {-2.2919, 4.4638, 2.9656, -2.9707, -4.7073, -0.58453},
                                   {-1.5152, -2.4929, 1.1725, 3.1789, 0.52327, -1.6736},
                                   {-2.2985, -3.1571, 1.9099, 4.076, 0.21512, -2.6996},
                                   {3.8392, -3.6946, 3.8823, 4.748, 0.29332, -5.312},
                                   {0.44367, -0.72537, 0.74096, 0.82195, 0.071706, -0.92273},
                                   {-1.3792, -34.038, 9.7159, 20.713, 12.448, -10.099},
                                   {-0.050932, -5.0042, 1.2709, 1.9099, 1.6577, -1.5081},
                                   {-1.9139, 3.3251, -25.382, 2.0547, 4.5296, 13.45},
                                   {-0.17922, 0.18969, -3.4137, 0.62664, 0.072841, 0.51918},
                                   {-18.989, 22.479, 35.032, -3.1844, -36.187, -22.633},
                                   {-2.2919, 2.9656, 4.4638, -0.58453, -4.7073, -2.9707},
                                   {19.982, -55.688, -39.85, 29.484, 45.368, 41.225},
                                   {30.175, -77.811, -62.972, 39.9, 69.463, 63.67},
                                   {-10.654, 40.234, 7.0439, -54.754, 7.4723, -14.112},
                                   {-1.3169, 6.2027, 0.90269, -9.2219, 4.0576, -2.4699},
                                   {75.891, -72.361, -31.209, -32.591, 269.6, -14.301},
                                   {15.875, -32.898, -2.177, 14.99, 35.637, -4.2622},
                                   {25.537, -165.14, -58.402, 205.14, -247.05, 163.82},
                                   {3.2824, -26.819, -4.1325, 33.848, -40.195, 26.297},
                                   {-34.406, -834, 209.15, 921.73, 263.18, -327.81},
                                   {-6.7723, -114.7, 34.288, 131.86, 31.778, -52.104},
                                   {19.982, -39.85, -55.688, 41.225, 45.368, 29.484},
                                   {30.175, -62.972, -77.811, 63.67, 69.463, 39.9},
                                   {10.654, -7.0439, -40.234, 14.112, -7.4723, 54.754},
                                   {1.3169, -0.90269, -6.2027, 2.4699, -4.0576, 9.2219},
                                   {25.537, -58.402, -165.14, 163.82, -247.05, 205.14},
                                   {3.2824, -4.1325, -26.819, 26.297, -40.195, 33.848},
                                   {75.891, -31.209, -72.361, -14.301, 269.6, -32.591},
                                   {15.875, -2.177, -32.898, -4.2622, 35.637, 14.99},
                                   {-34.406, 209.15, -834, -327.81, 263.18, 921.73},
                                   {-6.7723, 34.288, -114.7, -52.104, 31.778, 131.86}};

    fp32 fitted_matrix_A[15][6];
    fp32 fitted_matrix_B[20][6];

    // 调试用参数
    fp32 LQR_K[4][10] = {
        {-1.6958, -2.5289, -3.8672, -0.44578, -5.554, -0.76506, -5.3346, -0.70678, -10.645, -1.2008},
{-1.6958, -2.5289, 3.8672, 0.44578, -5.3346, -0.70678, -5.554, -0.76506, -10.645, -1.2008},
{5.5762, 9.0773, -3.3588, -0.17982, 69.326, 10.748, -16.177, -2.1284, -138.12, -19.235},
{5.5762, 9.0773, 3.3588, 0.17982, -16.177, -2.1284, 69.326, 10.748, -138.12, -19.235},
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
        {0, 0, 0, 0, -12.689, 0, -12.689, 0, 0, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, -2.5237, 0, 2.5237, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 143.64, 0, 8.6614, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 8.6614, 0, 143.64, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, -19.944, 0, -19.944, 0, 59.245, 0},
    };
    fp32 LQR_B[10][4] = {
        {0, 0, 0, 0},
        {4.0507, 4.0507, -0.80379, -0.80379},
        {0, 0, 0, 0},
        {-4.7198, 4.7198, -0.15986, 0.15986},
        {0, 0, 0, 0},
        {-34.465, -3.4395, 9.0987, 0.54864},
        {0, 0, 0, 0},
        {-3.4395, -34.465, 0.54864, 9.0987},
        {0, 0, 0, 0},
        {-0.42712, -0.42712, -4.0454, -4.0454},
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

    static inline fp32 chassis_calc_max_leg_extend_force(leg_control *leg)
    {
        return -Leg_PID_Calc(&leg->leg_pid_control, leg->wbr_control.L, CHASSIS_LEG_MAX);
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
        return state == CHASSIS_NORMAL || state == CHASSIS_LEG_1 || state == CHASSIS_LEG_2;
    }

    static bool_t chassis_is_yaw_lqr_state(Chassis_State_e state)
    {
        return chassis_is_balancing_state(state) || state == CHASSIS_JUMP;
    }

    static bool_t chassis_is_init_like_state(Chassis_State_e state)
    {
        return state == CHASSIS_INIT || state == CHASSIS_FLIP;
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

        /* 小陀螺旋转速度缩放 */
        chassis_move_init->chassis_spin_change_sen = CHASSIS_SPIN_LOW_SPEED;

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
            chassis_move_transit->chassis_leg_set = CHASSIS_LEG_MAX;
            chassis_move_transit->chassis_leg_filter_set.out = CHASSIS_LEG_MAX;
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

        static fp32 chassis_v_set = 0.0f, chassis_body_yaw_err = 0.0f, chassis_leg_set = 0.0f, chassis_d_yaw_set = 0.0f;

        /* 获取三个控制设置值 */
        chassis_behaviour_control_set(&chassis_v_set, &chassis_body_yaw_err, &chassis_d_yaw_set, &chassis_leg_set, chassis_move_control);

        chassis_move_control->chassis_d_yaw_kinematic = chassis_calc_kinematic_d_yaw(chassis_move_control);
        chassis_move_control->chassis_d_yaw_imu = chassis_get_imu_d_yaw(chassis_move_control);
        chassis_move_control->chassis_d_yaw_diff = chassis_move_control->chassis_d_yaw_imu - chassis_move_control->chassis_d_yaw_kinematic;

        if (chassis_is_yaw_lqr_state(chassis_move_control->state))
        {
            chassis_move_control->chassis_d_yaw = chassis_select_d_yaw_feedback(chassis_move_control);

            chassis_move_control->chassis_yaw = chassis_move_control->chassis_gimbal_data->chassis_relative_angle;
            chassis_move_control->chassis_v_set = chassis_v_set;
            chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
            chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw + chassis_body_yaw_err;
            chassis_move_control->chassis_yaw_err = chassis_body_yaw_err;
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
        // test
        // chassis_move_update->chassis_left_control.wbr_control.theta_l -=0.14f;

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
        chassis_move_update->chassis_left_control.wbr_control.d_theta_l = chassis_move_update->chassis_left_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->vel - chassis_move_update->chassis_left_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->vel - *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);
        chassis_move_update->chassis_right_control.wbr_control.d_theta_l = -chassis_move_update->chassis_right_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->vel + chassis_move_update->chassis_right_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->vel - *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);

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
        // decompose_fitted_matrix_to_K(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_K, LQR_K);
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
        // 用于离地防抖的计数器（当前控制周期1ms，连续2次判定为离地）
        static uint16_t left_off_count = 0;
        static uint16_t right_off_count = 0;

        if (chassis_move_prediction->state == CHASSIS_STOP ||
            chassis_move_prediction->state == CHASSIS_FLIP ||
            chassis_move_prediction->state == CHASSIS_INIT)
        {
            chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
            chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
            left_off_count = right_off_count = 0;
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
        fp32 dyaw_err = chassis_is_yaw_lqr_state(chassis_move_control_loop->state) ? chassis_move_control_loop->chassis_d_yaw - chassis_move_control_loop->chassis_d_yaw_set : 0.0f;

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

        /* 此函数已进行：功率控制+所有力矩计算 */
        /* 后方的所有函数和处理均是作为支持力计算和其他附加功能的实现 */
        chassis_lqr_power_control(chassis_move_control_loop);

        chassis_update_jump_phase(chassis_move_control_loop);
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

        chassis_move_control_loop->chassis_leg_set = CHASSIS_LEG_MAX;
        chassis_move_control_loop->chassis_leg_filter_set.out = CHASSIS_LEG_MAX;

        const fp32 left_leg_extend_force = chassis_calc_max_leg_extend_force(&chassis_move_control_loop->chassis_left_control);
        const fp32 right_leg_extend_force = chassis_calc_max_leg_extend_force(&chassis_move_control_loop->chassis_right_control);
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
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_roll - chassis_move_control_loop->chassis_left_control.fd_leg - chassis_move_control_loop->chassis_left_control.Fbl_gravity -chassis_move_control_loop->chassis_left_control.Fbl_inertial;

            // -重力补偿 - 侧向惯性力矩补偿 + 右腿腿长PID + 右腿roll轴补偿PID + 弹簧补偿
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = +chassis_move_control_loop->chassis_right_control.fd_roll - chassis_move_control_loop->chassis_right_control.fd_leg - chassis_move_control_loop->chassis_right_control.Fbl_gravity +chassis_move_control_loop->chassis_right_control.Fbl_inertial;
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
                LQR_K[2][5] * chassis_move_control_loop->chassis_left_control.d_theta_l+
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
                LQR_K[3][5] * chassis_move_control_loop->chassis_left_control.d_theta_l+
                LQR_K[3][6] * chassis_move_control_loop->chassis_right_control.theta_l +
                LQR_K[3][7] * chassis_move_control_loop->chassis_right_control.d_theta_l;
#endif
            }

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

            chassis_init_standup->chassis_leg_set = CHASSIS_LEG_MAX;
            chassis_init_standup->chassis_leg_filter_set.out = CHASSIS_LEG_MAX;

            const fp32 left_leg_extend_force = chassis_calc_max_leg_extend_force(&chassis_init_standup->chassis_left_control);
            const fp32 right_leg_extend_force = chassis_calc_max_leg_extend_force(&chassis_init_standup->chassis_right_control);
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
                if ((fabs(chassis_init_standup->chassis_left_control.theta_l) < 0.1) ||
                    (fabs(chassis_init_standup->chassis_right_control.theta_l) < 0.1))
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
