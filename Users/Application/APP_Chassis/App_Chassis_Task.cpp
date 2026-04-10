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
extern "C" {
#endif

    // 拟合矩阵K，A，B
    fp32 fitted_matrix_K[40][6] = {{
        -1.2854, -3.6996, 1.1959, 3.4809, 0.3428, -0.22891},
{-1.9774, -5.0911, 1.9644, 4.7791, 0.043449, -0.65803},
{-3.8701, 5.2315, -5.0387, -1.5639, -2.14, 4.178},
{-0.87818, 1.508, -1.5055, -0.25341, -0.76449, 1.1617},
{-1.1035, -35.836, 7.7613, 15.872, 17.479, -7.2598},
{-0.0028341, -5.6861, 0.83568, 1.1584, 2.7225, -0.79063},
{-0.85685, 1.1941, -27.54, 5.9646, -1.5473, 22.839},
{0.059856, -0.021639, -4.2185, 1.2972, -1.2185, 2.9127},
{-19.629, 20.077, 32.962, 7.6761, -46.973, -10.207},
{-2.0777, 2.1885, 3.5042, 0.80113, -5.0817, -1.0632},
{-1.2854, 1.1959, -3.6996, -0.22891, 0.3428, 3.4809},
{-1.9774, 1.9644, -5.0911, -0.65803, 0.043449, 4.7791},
{3.8701, 5.0387, -5.2315, -4.178, 2.14, 1.5639},
{0.87818, 1.5055, -1.508, -1.1617, 0.76449, 0.25341},
{-0.85685, -27.54, 1.1941, 22.839, -1.5473, 5.9646},
{0.059856, -4.2185, -0.021639, 2.9127, -1.2185, 1.2972},
{-1.1035, 7.7613, -35.836, -7.2598, 17.479, 15.872},
{-0.0028341, 0.83568, -5.6861, -0.79063, 2.7225, 1.1584},
{-19.629, 32.962, 20.077, -10.207, -46.973, 7.6761},
{-2.0777, 3.5042, 2.1885, -1.0632, -5.0817, 0.80113},
{34.646, -69.167, -71.247, -17.953, 101.89, 63.486},
{54.408, -97.53, -119.54, -34.75, 166.06, 102.27},
{-4.8367, -50.84, -3.42, 83.19, -102.41, 32.676},
{-1.1774, -14.536, 3.8575, 23.2, -28.915, 2.5149},
{124.94, -50.128, -117.33, -289.13, 432.85, 50.202},
{29.531, -36.224, -13.19, -33.773, 73.523, 0.97842},
{45.347, -199.32, -68.407, 181.45, -316.14, 245.12},
{6.709, -33.362, -10.551, 31.845, -67.852, 57.62},
{31.874, -1392.3, 193, 1337.1, 573.36, -334.71},
{2.9392, -152.74, 22.75, 148.41, 61.167, -38.366},
{34.646, -71.247, -69.167, 63.486, 101.89, -17.953},
{54.408, -119.54, -97.53, 102.27, 166.06, -34.75},
{4.8367, 3.42, 50.84, -32.676, 102.41, -83.19},
{1.1774, -3.8575, 14.536, -2.5149, 28.915, -23.2},
{45.347, -68.407, -199.32, 245.12, -316.14, 181.45},
{6.709, -10.551, -33.362, 57.62, -67.852, 31.845},
{124.94, -117.33, -50.128, 50.202, 432.85, -289.13},
{29.531, -13.19, -36.224, 0.97842, 73.523, -33.773},
{31.874, 193, -1392.3, -334.71, 573.36, 1337.1},
{2.9392, 22.75, -152.74, -38.366, 61.167, 148.41}};

    fp32 fitted_matrix_A[15][6];
    fp32 fitted_matrix_B[20][6];

    //调试用参数
    fp32 LQR_K[4][10] = {
        {-0.69427, -2.2387, -3.3515, -0.79001, -5.9487, -0.86012, -5.0585, -0.66319, -9.5708, -1.1221},
{-0.69427, -2.2387, 3.3515, 0.79001, -5.0585, -0.66319, -5.9487, -0.86012, -9.5708, -1.1221},
{2.121, 7.1838, -13.849, -3.4348, 63.553, 10.49, -14.778, -2.0873, -122.64, -19.559},
{2.121, 7.1838, 13.849, 3.4348, -14.778, -2.0873, 63.553, 10.49, -122.64, -19.559},
    };

//最好的参数
//     fp32 LQR_K[4][10] = {
//         {-0.69427, -2.2387, -3.7475, -0.83554, -5.948, -0.86055, -5.0591, -0.66276, -9.5708, -1.1221},
// {-0.69427, -2.2387, 3.7475, 0.83554, -5.0591, -0.66276, -5.948, -0.86055, -9.5708, -1.1221},
// {2.121, 7.1838, -15.461, -3.6163, 63.556, 10.488, -14.781, -2.0856, -122.64, -19.559},
// {2.121, 7.1838, 15.461, 3.6163, -14.781, -2.0856, 63.556, 10.488, -122.64, -19.559},
//     };


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

    static  fp32 Get_FeedForward_Force(fp32 leg_length_m);

static inline fp32 chassis_select_theta_signal(const leg_control *leg, uint8_t filter_source)
{
    return (filter_source == CHASSIS_FILTER_LOWPASS) ? leg->theta_l_lowpass : leg->theta_l_filter;
}

static inline fp32 chassis_select_d_theta_signal(const leg_control *leg, uint8_t filter_source)
{
    return (filter_source == CHASSIS_FILTER_LOWPASS) ? leg->d_theta_l_lowpass : leg->d_theta_l_filter;
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
static void chassis_up_leg_angle_control(Chassis_Move *chassis_move_control_loop);
static void chassis_init_level_control(Chassis_Move *chassis_move_control_loop);
static void chassis_calc_support_force(Chassis_Move *chassis_move_control_loop);
static fp32 chassis_update_soft_lqr_ratio(Chassis_Move *chassis_move_control_loop);
static void chassis_apply_joint_output(Chassis_Move *chassis_move_control_loop, fp32 soft_lqr_ratio);
static void chassis_limit_output(Chassis_Move *chassis_move_control_loop);
static void chassis_save_last_feedback(Chassis_Move *chassis_move_control_loop);

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
            toe_is_error(CHASSIS_JOINT3_TOE) || toe_is_error(CHASSIS_JOINT4_TOE) )
        {
            // 关节电机扭矩清零
            // for (int i = 0; i < 4; i++) {
            //     chassis_move.chassis_joint[i].joint_T = 0.0f;
            // }
            for (uint8_t i = 0; i < 4; i++)
            {
                // 重新使能
                chassis_move.chassis_joint[i].chassis_joint_measure->Enable_Joint_Motor();
                HAL_Delay(JOINT_MOTOR_lIMIT_TIME);
            }
            // 轮电机扭矩清零
            // chassis_move.chassis_wheel[0].wheel_T = 0.0f;
            // chassis_move.chassis_wheel[1].wheel_T = 0.0f;
        }

        else if (toe_is_error(VT_TOE))
        {
            // 关节电机扭矩清零
            for (int i = 0; i < 4; i++) {
                chassis_move.chassis_joint[i].joint_T = 0.0f;
            }
            // 轮电机扭矩清零
            chassis_move.chassis_wheel[0].wheel_T = 0.0f;
            chassis_move.chassis_wheel[1].wheel_T = 0.0f;
        }

        for(int i = 0; i < 4; i++) {
            chassis_move.chassis_joint[i].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, chassis_move.chassis_joint[i].joint_T);
            //chassis_move.chassis_joint[i].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, 0.0f);
        }
        CAN_cmd_wheel(-chassis_move.chassis_wheel[0].wheel_T, chassis_move.chassis_wheel[1].wheel_T);
        //CAN_cmd_wheel(0,0);

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

     Kalman_Filter_Init(&chassis_move_init->chassis_vaestimatekf_xv, 2, 0, 2);    							// 状态向量2维 没有控制量 测量向量2维
     Kalman_Filter_Init(&chassis_move_init->chassis_vaestimatekf_va, 2, 0, 2);    							// 状态向量2维 没有控制量 测量向量2维
     Kalman_Filter_Init(&chassis_move_init->chassis_left_control.chassis_vaestimatekf_theta, 2, 0, 2);    	// 状态向量2维 没有控制量 测量向量2维
     Kalman_Filter_Init(&chassis_move_init->chassis_right_control.chassis_vaestimatekf_theta, 2, 0, 2);    	// 状态向量2维 没有控制量 测量向量2维

    memcpy(chassis_move_init->chassis_vaestimatekf_xv.F_data, chassis_move_init->vaEstimateKF_F_VX, sizeof(chassis_move_init->vaEstimateKF_F_VX));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.P_data, chassis_move_init->vaEstimateKF_P_VX, sizeof(chassis_move_init->vaEstimateKF_P_VX));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.Q_data, chassis_move_init->vaEstimateKF_Q_VX, sizeof(chassis_move_init->vaEstimateKF_Q_VX));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.R_data, chassis_move_init->vaEstimateKF_R_VX, sizeof(chassis_move_init->vaEstimateKF_R_VX));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.H_data, chassis_move_init->vaEstimateKF_H_VX, sizeof(chassis_move_init->vaEstimateKF_H_VX));

    memcpy(chassis_move_init->chassis_vaestimatekf_xv.F_data, chassis_move_init->vaEstimateKF_F_VA, sizeof(chassis_move_init->vaEstimateKF_F_VA));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.P_data, chassis_move_init->vaEstimateKF_P_VA, sizeof(chassis_move_init->vaEstimateKF_P_VA));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.Q_data, chassis_move_init->vaEstimateKF_Q_VA, sizeof(chassis_move_init->vaEstimateKF_Q_VA));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.R_data, chassis_move_init->vaEstimateKF_R_VA, sizeof(chassis_move_init->vaEstimateKF_R_VA));
    memcpy(chassis_move_init->chassis_vaestimatekf_xv.H_data, chassis_move_init->vaEstimateKF_H_VA, sizeof(chassis_move_init->vaEstimateKF_H_VA));

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
    chassis_move_init->chassis_INS_gyro  = get_gyro_data_point();
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
        HAL_Delay(JOINT_MOTOR_lIMIT_TIME);
    }


    //初始化wbr数据
    wbr_init(&chassis_move_init->chassis_left_control.wbr_control);
    wbr_init(&chassis_move_init->chassis_right_control.wbr_control);

    //yaw轴跟随pid初始化
    const static fp32 yaw_angle_pid[3] = {YAW_PID_KP, YAW_PID_KI, YAW_PID_KD};

    // roll和腿长pid初始化
    const static fp32 roll_pid[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
    const static fp32 leg_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, yaw_angle_pid, YAW_PID_MAX_OUT, YAW_PID_MAX_OUT); //yaw轴跟随pid

    PID_init(&chassis_move_init->chassis_left_control.roll_control, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);   //左腿roll轴pid
    PID_init(&chassis_move_init->chassis_right_control.roll_control, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);   //左腿roll轴pid
    PID_init(&chassis_move_init->chassis_left_control.leg_pid_control, PID_POSITION, leg_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);       // 左腿长pid
    PID_init(&chassis_move_init->chassis_right_control.leg_pid_control, PID_POSITION, leg_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);      // 右腿长pid

    // 初始化收腿角度PID
    const static fp32 init_leg_angle_pid[3] = {INIT_LEG_ANGLE_PID_KP, INIT_LEG_ANGLE_PID_KI, INIT_LEG_ANGLE_PID_KD};
    PID_init(&chassis_move_init->chassis_left_control.init_leg_angle_pid, PID_POSITION, init_leg_angle_pid, INIT_LEG_ANGLE_PID_MAX_OUT, INIT_LEG_ANGLE_PID_MAX_IOUT);
    PID_init(&chassis_move_init->chassis_right_control.init_leg_angle_pid, PID_POSITION, init_leg_angle_pid, INIT_LEG_ANGLE_PID_MAX_OUT, INIT_LEG_ANGLE_PID_MAX_IOUT);

    // 上台阶收腿角度PID
    const static fp32 up_leg_angle_pid[3] = {UP_LEG_ANGLE_PID_KP, UP_LEG_ANGLE_PID_KI, UP_LEG_ANGLE_PID_KD};
    PID_init(&chassis_move_init->chassis_left_control.up_leg_angle_pid, PID_POSITION, up_leg_angle_pid, UP_LEG_ANGLE_PID_MAX_OUT, UP_LEG_ANGLE_PID_MAX_IOUT);
    PID_init(&chassis_move_init->chassis_right_control.up_leg_angle_pid, PID_POSITION, up_leg_angle_pid, UP_LEG_ANGLE_PID_MAX_OUT, UP_LEG_ANGLE_PID_MAX_IOUT);

    /* 滤波参数：DT7发送设定值的低通滤波 */
    const static float chassis_DT7_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_DT7_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 chassis_leg_order_filter[1] = {CHASSIS_ACCEL_LEG_NUM};
    const static fp32 chassis_leg_up_filter[1] = {CHASSIS_ACCEL_UP_NUM};
    const static fp32 chassis_theta_l_lowpass_filter[1] = {CHASSIS_THETA_L_LOWPASS_NUM};
    const static fp32 chassis_d_theta_l_lowpass_filter[1] = {CHASSIS_D_THETA_L_LOWPASS_NUM};

    /* 滤波初始化：DT7发送设定值的低通滤波 */
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_DT7_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_DT7_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_leg_filter_set, CHASSIS_CONTROL_TIME, chassis_leg_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_UP_filter_set, CHASSIS_CONTROL_TIME, chassis_leg_up_filter);
    first_order_filter_init(&chassis_move_init->chassis_left_control.theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_theta_l_lowpass_filter);
    first_order_filter_init(&chassis_move_init->chassis_right_control.theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_theta_l_lowpass_filter);
    first_order_filter_init(&chassis_move_init->chassis_left_control.d_theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_d_theta_l_lowpass_filter);
    first_order_filter_init(&chassis_move_init->chassis_right_control.d_theta_l_lowpass_filter, CHASSIS_CONTROL_TIME, chassis_d_theta_l_lowpass_filter);
    chassis_move_init->chassis_leg_filter_set.out = 0.35;

    // 初始化斜坡函数，时间周期为 CHASSIS_CONTROL_TIME(0.002s)
    ramp_init(&chassis_move_init->chassis_left_control.init_angle_ramp, CHASSIS_CONTROL_TIME, 2.0f * PI + 1.0f, -1.0f);
    ramp_init(&chassis_move_init->chassis_right_control.init_angle_ramp, CHASSIS_CONTROL_TIME, 2.0f * PI + 1.0f, -1.0f);

    ramp_init(&chassis_move_init->chassis_left_control.up_angle_ramp, CHASSIS_CONTROL_TIME, 2.0f * PI + 1.0f, -1.0f);
    ramp_init(&chassis_move_init->chassis_right_control.up_angle_ramp, CHASSIS_CONTROL_TIME, 2.0f * PI + 1.0f, -1.0f);

    /* 小陀螺旋转速度缩放 */
    chassis_move_init->chassis_spin_change_sen = CHASSIS_SPIN_LOW_SPEED;

    /* 底盘默认模式 */
    chassis_move_init->chassis_mode = CHASSIS_ZERO;

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

    Chassis_Behaviour_Mode_Set(chassis_move_mode);//in file "App_Chassis_Behaviour.c"
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

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode) // 模式未切换
    {
        return;
    }
     /* 切入底盘跟随底盘yaw角度模式 或者 进入小陀螺模式*/
    if (((chassis_move_transit->last_chassis_mode != CHASSIS_FOLLOW_CHASSIS_YAW) && (chassis_move_transit->chassis_mode == CHASSIS_FOLLOW_CHASSIS_YAW))
            || ((chassis_move_transit->last_chassis_mode != CHASSIS_GYROSCOPE) && (chassis_move_transit->chassis_mode == CHASSIS_GYROSCOPE)))
    {
        chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel)
                                            - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_transit->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].total_pos + chassis_move_transit->chassis_wheel[1].total_pos)
                                      - chassis_move_transit->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_transit->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_transit->chassis_v_set = 0.0f;
        chassis_move_transit->chassis_d_yaw_set = 0.0f;
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        //chassis_move_transit->chassis_leg_set = 0.201f;

    }

    /* 跟随云台模式 */
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW) && (chassis_move_transit->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW))
    {
        chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel)
                                            - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_transit->chassis_yaw = chassis_move_transit->chassis_gimbal_data->yaw_relative_angle;

        chassis_move_transit->chassis_v_set = 0.0f;
        chassis_move_transit->chassis_d_yaw_set = 0.0f;
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        chassis_move_transit->chassis_x_set += CHASSIS_X_BACK;
        //chassis_move_transit->chassis_leg_set = 0.201f;

        chassis_move_transit->chassis_yaw_target_base = chassis_move_transit->chassis_gimbal_data->final_pos;
    }

    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_INIT) && (chassis_move_transit->chassis_mode == CHASSIS_INIT))
    {
        // 重置初始化收腿到位标志
        chassis_move_transit->init_leg_reach_state = INIT_LEG_UNREACH;

        // 记录进入时的当前角度(0~2π坐标系)，将其设为斜坡函数的起始输出值
        fp32 left_raw = chassis_move_transit->chassis_left_control.theta_l;
        fp32 right_raw = chassis_move_transit->chassis_right_control.theta_l;
        chassis_move_transit->chassis_left_control.init_angle_ramp.out = left_raw < 0.0f ? left_raw + 2.0f * PI : left_raw;
        chassis_move_transit->chassis_right_control.init_angle_ramp.out = right_raw < 0.0f ? right_raw + 2.0f * PI : right_raw;

        // 1. 恢复斜坡函数的默认最大最小值边界，防止第二次自启触发阶跃突变
        chassis_move_transit->chassis_left_control.init_angle_ramp.max_value = 2.0f * PI + 1.0f;
        chassis_move_transit->chassis_left_control.init_angle_ramp.min_value = -1.0f;
        chassis_move_transit->chassis_right_control.init_angle_ramp.max_value = 2.0f * PI + 1.0f;
        chassis_move_transit->chassis_right_control.init_angle_ramp.min_value = -1.0f;

        // 2. 清空自启专用的PID控制器残留数据，防止历史积分干扰输出
        PID_clear(&chassis_move_transit->chassis_left_control.init_leg_angle_pid);
        PID_clear(&chassis_move_transit->chassis_right_control.init_leg_angle_pid);

        chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel)
                                            - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_transit->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].total_pos + chassis_move_transit->chassis_wheel[1].total_pos)
                                      - chassis_move_transit->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_transit->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_transit->chassis_v_set = 0.0f;
        chassis_move_transit->chassis_d_yaw_set = 0.0f;
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        chassis_move_transit->chassis_leg_set = 0.35;
        chassis_move_transit->chassis_leg_filter_set.out = 0.35;
    }

    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_JUMP) && (chassis_move_transit->chassis_mode == CHASSIS_JUMP))
    {
        chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel)
                                            - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_transit->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].total_pos + chassis_move_transit->chassis_wheel[1].total_pos)
                                      - chassis_move_transit->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_transit->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_transit->chassis_v_set = 0.0f;
        chassis_move_transit->chassis_d_yaw_set = 0.0f;
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        chassis_move_transit->chassis_leg_set = 0.35f; //直接把腿长拉满，把支持力全部用来跳跃
        chassis_move_transit->chassis_leg_filter_set.out = 0.35; //准备收腿缓冲
        chassis_move_transit->jump_state = 1;   //进入伸腿阶段
    }

    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_UP) && (chassis_move_transit->chassis_mode == CHASSIS_UP))
    {

        // 记录当前角度，作为摆腿斜坡的起点
        fp32 left_raw = chassis_move_transit->chassis_left_control.theta_l;
        fp32 right_raw = chassis_move_transit->chassis_right_control.theta_l;
        chassis_move_transit->chassis_left_control.up_angle_ramp.out = left_raw < 0.0f ? left_raw + 2.0f * PI : left_raw;
        chassis_move_transit->chassis_right_control.up_angle_ramp.out = right_raw < 0.0f ? right_raw + 2.0f * PI : right_raw;

        chassis_move_transit->chassis_left_control.up_angle_ramp.max_value = 2.0f * PI + 1.0f;
        chassis_move_transit->chassis_left_control.up_angle_ramp.min_value = -1.0f;
        chassis_move_transit->chassis_right_control.up_angle_ramp.max_value = 2.0f * PI + 1.0f;
        chassis_move_transit->chassis_right_control.up_angle_ramp.min_value = -1.0f;

        chassis_move_transit->chassis_UP_filter_set.out = 0.18; //准备伸腿缓冲
        chassis_move_transit->up_state = 1;   //进入伸腿阶段
        chassis_move_transit->up_leg_reached = 0; // 重置上台阶腿角度到位标志

        // 清空 PID
        PID_clear(&chassis_move_transit->chassis_left_control.up_leg_angle_pid);
        PID_clear(&chassis_move_transit->chassis_right_control.up_leg_angle_pid);

        chassis_move_transit->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].vel + chassis_move_transit->chassis_wheel[1].vel)
                                            - chassis_move_transit->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l) * chassis_move_transit->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_transit->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l) * chassis_move_transit->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_transit->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_transit->chassis_wheel[0].total_pos + chassis_move_transit->chassis_wheel[1].total_pos)
                                      - chassis_move_transit->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_transit->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_transit->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_transit->chassis_v_set = 0.0f;
        chassis_move_transit->chassis_d_yaw_set = 0.0f;
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
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
      * PART1.  本部分代码获取三自由度上的控制设定值（DT7），根据行为状态进行不同的解算，从而得到最后的机器人三自由度速度设定值，包括以下行为的解算
      *         1.底盘跟随云台模式  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW
      *         2.底盘跟随yaw模式  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW
      *         3.底盘无角度控制   CHASSIS_VECTOR_NO_FOLLOW_YAW
      *         4.底盘无力模式     CHASSIS_VECTOR_RAW
      *         5.底盘小陀螺模式   CHASSIS_GYROSCOPE
      */

    static fp32 chassis_v_set = 0.0f, chassis_yaw_set = 0.0f, chassis_leg_set = 0.0f, chassis_d_yaw_set = 0.0f;



    /* 获取三个控制设置值 */
    chassis_behaviour_control_set(&chassis_v_set, &chassis_yaw_set, &chassis_d_yaw_set, &chassis_leg_set, chassis_move_control);

    /* 底盘跟随云台模式 */
    if (chassis_move_control->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        // 注意一下正负方向
        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);

        //chassis_move_control->chassis_yaw = chassis_move_control->chassis_gimbal_data->yaw_relative_angle;
        chassis_move_control->chassis_yaw = chassis_yaw_set;
        chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_yaw_set = 0.0f;
        chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
        chassis_move_control->chassis_leg_set = chassis_leg_set;
    }
    /* 底盘跟随底盘yaw轴 */
    else if (chassis_move_control->chassis_mode == CHASSIS_FOLLOW_CHASSIS_YAW)
    {

        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos)
                                      - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
        chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
        //chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw;
        chassis_move_control->chassis_leg_set = chassis_leg_set;
    }
    // 初始化模式
    else if (chassis_move_control->chassis_mode == CHASSIS_INIT)
    {

        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos)
                                      - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        //chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_v_set = 0.0f;
        //chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_x_set = chassis_move_control->x_filter;
        chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
        chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
        chassis_move_control->chassis_leg_set = chassis_leg_set;
    }

    /* 底盘无力模式 */
    else if (chassis_move_control->chassis_mode == CHASSIS_ZERO)
    {
        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos)
                                      - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
        chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
        chassis_move_control->chassis_leg_set = chassis_leg_set;
    }

    /* 底盘小陀螺模式 */
    else if (chassis_move_control->chassis_mode == CHASSIS_GYROSCOPE)
    {
        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos)
                                      - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_control->chassis_v_set = 0.0f;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
        chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
        chassis_move_control->chassis_leg_set = chassis_leg_set;
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_JUMP)
    {
        // 注意一下正负方向
        // chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
        //                                     - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
        //                                     + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
        //                                     / (2 * DRIVE_WHEEL_DIS);
        // chassis_move_control->chassis_yaw = shortest_angle_error(chassis_move_control->chassis_gimbal_data->init_yaw_angle,
        //     chassis_move_control->chassis_gimbal_data->final_pos);
        // chassis_move_control->chassis_v_set = chassis_v_set;
        // chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        // chassis_move_control->chassis_yaw_set = 0.0f;
        // chassis_move_control->chassis_d_yaw_set = PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw, chassis_move_control->chassis_yaw_set); //转速pid计算


        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos)
                                      - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
        chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
    	switch (chassis_move_control->jump_state)
    	{
          	case 0:
    		{
          		break;
        	}
      		case 1: //已经把腿长变成0.35，这一步是判断腿长变化
    		{
          	    // 如果腿长已经大于0.34，则进入下一步收腿
            	if (chassis_move_control->chassis_left_control.wbr_control.L >= 0.34 || chassis_move_control->chassis_left_control.wbr_control.L >= 0.34)
            	{
                	chassis_move_control->jump_state = 2;
            	}
          	    break;
        	}
    	    case 2: //已经到最长，跳起来了，接下来是收回腿
          	{
          	    first_order_filter_cali(&chassis_move_control->chassis_leg_filter_set, 0.18f);
          	    chassis_move_control->chassis_leg_set = chassis_move_control->chassis_leg_filter_set.out;

          	    //成功收腿后
          	    if (Chassis_Behaviour_Mode == CHASSIS_JUMP && chassis_move_control->chassis_leg_set < 0.211)
          	    {
          	        Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW;
          	        chassis_move_control->jump_state = 0;
          	    }
          	    break;
          	}
			default:
        	{
            	break;
        	}
		}
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_UP)
    {
        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].total_pos + chassis_move_control->chassis_wheel[1].total_pos)
                                      - chassis_move_control->chassis_left_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l)
                                      + chassis_move_control->chassis_right_control.wbr_control.L * arm_sin_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l))
                                      / (2 * DRIVE_WHEEL_DIS);

        if (chassis_move_control->up_state >= 2)
        {
            chassis_move_control->chassis_v_set = 0.0f;
            chassis_move_control->chassis_x_set = chassis_move_control->x_filter; // 同步当前位置，消除误差
            chassis_move_control->chassis_d_yaw_set = 0.0f;
            chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw;
        }
        else if (chassis_move_control->up_state <= 1) {
            chassis_move_control->chassis_v_set = chassis_v_set;
            chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
            chassis_move_control->chassis_d_yaw_set = chassis_d_yaw_set;
            chassis_move_control->chassis_yaw_set += chassis_move_control->chassis_d_yaw_set * chassis_move_control->dt;
        }

        static uint16_t hit_stair_count = 0; // 用于撞击滤波
    	switch (chassis_move_control->up_state)
    	{
          	case 0:
    		{
          		break;
        	}
    	    case 1: //已经到最长，判断是否磕到台阶
          	{
          	    first_order_filter_cali(&chassis_move_control->chassis_UP_filter_set, 0.35f);
          	    chassis_move_control->chassis_leg_set = chassis_move_control->chassis_UP_filter_set.out;

          	    // 特征 1：稳态推墙倾角。顶住台阶后，LQR为了发力必定会让车体产生持续的倾斜
          	    // (阈值设为 0.10 rad 约 5.7 度，平地匀速起步一般不会倾斜这么大)
          	    bool_t is_pitch_tilt = (fabs(chassis_move_control->chassis_pitch) > CHASSIS_PITCH_LEVEL_THRESHOLD);//0.1

          	    // 特征 2：机体速度突变 (IMU 捕捉到急刹车的冲击加速度)
          	    // 注意极性：假设向前加速为正，撞击减速则为负。阈值设为 -1.5m/s^2 左右
          	    bool_t is_hard_impact = (chassis_move_control->chassis_accel_n_x < -1.4f);

          	    // 特征 3：持续堵转大扭矩 (有前进指令，且轮毂持续爆发大扭矩推墙)
          	    bool_t is_pushing = (fabs(chassis_move_control->chassis_v_set) > 0.1f &&
                                      (fabs(chassis_move_control->chassis_wheel[0].tor) > 2.0f ||
                                       fabs(chassis_move_control->chassis_wheel[1].tor) > 2.0f));
          	    // ====== 综合判定 ======
          	    // 只要检测到【腿部受击后倾】，且伴随【机体急刹冲击】或【轮子打滑停滞】，即刻判定撞击成功！
          	    if (is_hard_impact && (is_pitch_tilt || is_pushing))
          	    {
          	        hit_stair_count++;
          	        if (hit_stair_count > 10)
          	        {
          	            chassis_move_control->up_state = 2; // 果断切入 Phase 2 (提膝跨越)
          	            chassis_move_control->chassis_UP_filter_set.out = 0.35;
          	            hit_stair_count = 0;

          	            chassis_move_control->chassis_left_control.up_angle_ramp.out = chassis_move_control->chassis_left_control.theta_l;
          	            chassis_move_control->chassis_right_control.up_angle_ramp.out = chassis_move_control->chassis_right_control.theta_l;
          	        }
          	    }
          	    else
          	    {
          	        // 采用衰减计数法：如果没有持续检测到，慢慢减少计数，增加容错率
          	        if(hit_stair_count > 0) hit_stair_count--;
          	    }
          	    break;
          	}
    	    case 2: // 已经碰到台阶了，用PID控制腿角度到目标位置（逆时针方向）
          	{
          	    // 腿角度到位后进入收腿阶段
          	    if (chassis_move_control->up_leg_reached == 1)
          	    {
          	        chassis_move_control->up_state = 3;
          	        chassis_move_control->chassis_UP_filter_set.out = 0.35; //初始化斜坡函数
          	    }
          	    break;
          	}
    	    case 3: // 腿角度已到位，开始收腿
          	{
          	    first_order_filter_cali(&chassis_move_control->chassis_UP_filter_set, 0.18f);
          	    chassis_move_control->chassis_leg_set = chassis_move_control->chassis_UP_filter_set.out;

          	    //腿长小于阈值后，成功收腿，切回follow_chassis模式
          	    if (chassis_move_control->chassis_leg_set < CHASSIS_UP_LEG_LENGTH_THRESHOLD)
          	    {

          	        Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW;
          	        chassis_move_control->up_state = 0;
          	        chassis_move_control->up_leg_reached = 0;
          	    }
          	    break;
          	}
			default:
        	{
            	break;
        	}
		}
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
    chassis_move_update->chassis_left_control.wbr_control.phi1 =  rad_format(chassis_move_update->chassis_joint[0].chassis_joint_measure->pos);
    chassis_move_update->chassis_left_control.wbr_control.phi2 =  rad_format(-chassis_move_update->chassis_joint[1].chassis_joint_measure->pos);
    chassis_move_update->chassis_right_control.wbr_control.phi1 = rad_format(-chassis_move_update->chassis_joint[2].chassis_joint_measure->pos);
    chassis_move_update->chassis_right_control.wbr_control.phi2 = rad_format(chassis_move_update->chassis_joint[3].chassis_joint_measure->pos);

    // 根据轮毂电机编码器角度和速度计算自然坐标系下机器人水平方向移动距离和速度
	chassis_move_update->v_real = WHEEL_RADIUS * (chassis_move_update->chassis_wheel[0].vel + chassis_move_update->chassis_wheel[1].vel) / 2;
    chassis_move_update->x_real = WHEEL_RADIUS * (chassis_move_update->chassis_wheel[0].total_pos + chassis_move_update->chassis_wheel[1].total_pos) / 2;


    //卡尔曼滤波器测量值更新
    chassis_move_update->vaEstimateKF_F_VX[0] = 1.0f;
    chassis_move_update->vaEstimateKF_F_VX[1] = chassis_move_update->dt;
    chassis_move_update->vaEstimateKF_F_VX[2] = 0.0f;
    chassis_move_update->vaEstimateKF_F_VX[3] = 1.0f;
    memcpy(chassis_move_update->chassis_vaestimatekf_xv.F_data, chassis_move_update->vaEstimateKF_F_VX, sizeof(chassis_move_update->vaEstimateKF_F_VX));
    chassis_move_update->chassis_vaestimatekf_xv.MeasuredVector[0] = chassis_move_update->x_real;
    chassis_move_update->chassis_vaestimatekf_xv.MeasuredVector[1] = chassis_move_update->v_real;
    //卡尔曼滤波器更新函数
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

    // 根据机体pitch角度判断底盘是否处于正常水平状态
    chassis_move_update->last_chassis_state = chassis_move_update->chassis_state;
    if (fabs(chassis_move_update->chassis_pitch) <= CHASSIS_PITCH_LEVEL_THRESHOLD)
    {
        chassis_move_update->chassis_state = CHASSIS_NORMAL;
    }
    else
    {
        chassis_move_update->chassis_state = CHASSIS_DOWN;
    }

    // 调用WBR计算右腿和左腿的角度以及角速度，用于LQR控制
     wbr_calc(&chassis_move_update->chassis_left_control.wbr_control);
     wbr_calc(&chassis_move_update->chassis_right_control.wbr_control);
    // // 根据关节角度计算雅可比矩阵 用于求解关节电机扭矩
     wbr_jacobian_calc(&chassis_move_update->chassis_left_control.wbr_control);
     wbr_jacobian_calc(&chassis_move_update->chassis_right_control.wbr_control);
    // 计算腿部摆杆角速度和腿长变化速度
	chassis_move_update->chassis_left_control.wbr_control.d_theta_l = chassis_move_update->chassis_left_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->vel
																	- chassis_move_update->chassis_left_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->vel
        															- *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);
    chassis_move_update->chassis_right_control.wbr_control.d_theta_l = -chassis_move_update->chassis_right_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->vel
																	+ chassis_move_update->chassis_right_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->vel
        															- *(chassis_move_update->chassis_INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);
    chassis_move_update->chassis_left_control.wbr_control.d_L = chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->vel
																- chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->vel;
    chassis_move_update->chassis_right_control.wbr_control.d_L = -chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->vel
																+ chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->vel;

    //腿部角度卡尔曼滤波参数
    chassis_move_update->vaEstimateKF_F_theta[0] = 1.0f;
    chassis_move_update->vaEstimateKF_F_theta[1] = chassis_move_update->dt;
    chassis_move_update->vaEstimateKF_F_theta[2] = 0.0f;
    chassis_move_update->vaEstimateKF_F_theta[3] = 1.0f;
    chassis_update_leg_angle_signals(&chassis_move_update->chassis_left_control, chassis_move_update->vaEstimateKF_F_theta);
    chassis_update_leg_angle_signals(&chassis_move_update->chassis_right_control, chassis_move_update->vaEstimateKF_F_theta);

     // 实际左腿转矩
    chassis_move_update->chassis_left_control.wbr_control.Tbl_r = (chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[1].chassis_joint_measure->tor
                                                                    + chassis_move_update->chassis_left_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[0].chassis_joint_measure->tor)
                                                                    / (chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_left_control.wbr_control.J[1][1]
                                                                    - chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_left_control.wbr_control.J[1][0]);
    // 实际右腿转矩
    chassis_move_update->chassis_right_control.wbr_control.Tbl_r = -(chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_joint[3].chassis_joint_measure->tor
                                                                    + chassis_move_update->chassis_right_control.wbr_control.J[1][0] * chassis_move_update->chassis_joint[2].chassis_joint_measure->tor)
                                                                    / (chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_right_control.wbr_control.J[1][1]
                                                                    - chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_right_control.wbr_control.J[1][0]);

    // 实际左腿支持力
    chassis_move_update->chassis_left_control.wbr_control.Fbl_r = (chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[1].chassis_joint_measure->tor
                                                                    + chassis_move_update->chassis_left_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[0].chassis_joint_measure->tor)
                                                                    / (chassis_move_update->chassis_left_control.wbr_control.J[0][0] * chassis_move_update->chassis_left_control.wbr_control.J[1][1]
                                                                    - chassis_move_update->chassis_left_control.wbr_control.J[0][1] * chassis_move_update->chassis_left_control.wbr_control.J[1][0]);

    // 实际右腿支持力
    chassis_move_update->chassis_right_control.wbr_control.Fbl_r = -(chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_joint[3].chassis_joint_measure->tor
                                                                    + chassis_move_update->chassis_right_control.wbr_control.J[1][1] * chassis_move_update->chassis_joint[2].chassis_joint_measure->tor)
                                                                    / (chassis_move_update->chassis_right_control.wbr_control.J[0][0] * chassis_move_update->chassis_right_control.wbr_control.J[1][1]
                                                                    - chassis_move_update->chassis_right_control.wbr_control.J[0][1] * chassis_move_update->chassis_right_control.wbr_control.J[1][0]);

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

    // 左轮地面支持力
    chassis_move_update->chassis_left_control.Fwn = chassis_move_update->chassis_left_control.wbr_control.Fbl_r * arm_cos_f32(chassis_move_update->chassis_left_control.wbr_control.theta_l)
                                                    + MASS_OF_LEG * (GRAVITY_ACCELERATION + chassis_move_update->chassis_accel_n_z
                                                    - (1 - chassis_move_update->chassis_left_control.eta) * chassis_move_update->chassis_left_control.wbr_control.dd_L
                                                    * arm_cos_f32(chassis_move_update->chassis_left_control.wbr_control.theta_l))
                                                    + Get_FeedForward_Force(chassis_move_update->chassis_left_control.wbr_control.L);

    // 右轮地面支持力
    chassis_move_update->chassis_right_control.Fwn = chassis_move_update->chassis_right_control.wbr_control.Fbl_r * arm_cos_f32(chassis_move_update->chassis_right_control.wbr_control.theta_l)
                                                    + MASS_OF_LEG * (GRAVITY_ACCELERATION + chassis_move_update->chassis_accel_n_z
                                                    - (1 - chassis_move_update->chassis_right_control.eta) * chassis_move_update->chassis_right_control.wbr_control.dd_L
                                                    * arm_cos_f32(chassis_move_update->chassis_right_control.wbr_control.theta_l))
                                                    + Get_FeedForward_Force(chassis_move_update->chassis_right_control.wbr_control.L);

    // 计算矩阵K，A，B
    //decompose_fitted_matrix_to_K(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_K, LQR_K);
//    decompose_fitted_matrix_to_A(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_A, LQR_A);
//    decompose_fitted_matrix_to_B(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_B, LQR_B);

    // mpc状态预测，预测下一时刻轮子状态并对轮毂电机力矩进行补偿(目前还是定腿长控制)
    // 速度前正后负
    // 腿后伸是加速，对应腿负数加速
    // 机体下是正，促进加速
    // 轮子正向扭矩为加速
    chassis_move_update->chassis_left_control.v_real_n = chassis_move_update->v_real
                                                        + (LQR_A[1][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                        + LQR_A[1][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                        + LQR_A[1][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        												+ (-LQR_B[1][0] * chassis_move_update->chassis_wheel[0].tor
                                                        - LQR_B[1][1] * chassis_move_update->chassis_wheel[1].tor
                                                        + LQR_B[1][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                        + LQR_B[1][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    chassis_move_update->chassis_right_control.v_real_n = chassis_move_update->v_real
                                                        + (LQR_A[1][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                        + LQR_A[1][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                        + LQR_A[1][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        												+ (-LQR_B[1][0] * chassis_move_update->chassis_wheel[0].tor
                                                        - LQR_B[1][1] * chassis_move_update->chassis_wheel[1].tor
                                                        + LQR_B[1][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                        + LQR_B[1][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    chassis_move_update->chassis_left_control.chassis_d_yaw_n = chassis_move_update->chassis_d_yaw
                                                            + (LQR_A[3][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                            + LQR_A[3][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                            + LQR_A[3][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        													+ (-LQR_B[3][0] * chassis_move_update->chassis_wheel[0].tor
                                                            - LQR_B[3][1] * chassis_move_update->chassis_wheel[1].tor
                                                            + LQR_B[3][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                            + LQR_B[3][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    chassis_move_update->chassis_right_control.chassis_d_yaw_n = chassis_move_update->chassis_d_yaw
                                                            + (LQR_A[3][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                            + LQR_A[3][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                            + LQR_A[3][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        													+ (-LQR_B[3][0] * chassis_move_update->chassis_wheel[0].tor
                                                            - LQR_B[3][1] * chassis_move_update->chassis_wheel[1].tor
                                                            + LQR_B[3][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                            + LQR_B[3][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

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
    // 用于防抖的计数器（假设控制周期2ms，15次等于30ms）
    static uint16_t left_off_count = 0;
    static uint16_t right_off_count = 0;

    if(chassis_move_prediction->chassis_mode == CHASSIS_INIT ||
       chassis_move_prediction->chassis_mode == CHASSIS_ZERO ||
       chassis_move_prediction->chassis_mode == CHASSIS_UP)
    {
        chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
        chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
        left_off_count = right_off_count = 0;
        return;
    }

    if (chassis_move_prediction->chassis_mode == CHASSIS_JUMP && chassis_move_prediction->jump_state == 2) {
        chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_OFF_GROUND;
        chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_OFF_GROUND;
        return;
    }

    // ========== 左腿防抖检测 ==========
    if (chassis_move_prediction->chassis_left_control.Fwn <= 60.0f) {
        left_off_count++;
        if (left_off_count > 2) {
            chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_OFF_GROUND;
            left_off_count = 2; // 防止溢出
        }
    } else {
        left_off_count = 0; // 一旦受力，立即判定落地
        chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
    }

    if (chassis_move_prediction->chassis_right_control.Fwn <= 60.0f) {
        right_off_count++;
        if (right_off_count > 2) {
            chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_OFF_GROUND;
            right_off_count = 2;
        }
    } else {
        right_off_count = 0;
        chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
    }

    if (chassis_move_prediction->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND
    && chassis_move_prediction->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND)
    {
        //数据不再更新，保持为离地前的那一刻
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
    fp32 x_err    = -chassis_move_control_loop->x_filter + chassis_move_control_loop->chassis_x_set;
    fp32 v_err    = -chassis_move_control_loop->v_filter + chassis_move_control_loop->chassis_v_set;
    fp32 yaw_err  = chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set;
    fp32 dyaw_err = chassis_move_control_loop->chassis_d_yaw - chassis_move_control_loop->chassis_d_yaw_set;

    // 2. 提取平动和旋转的控制分量
    fp32 U_speed = LQR_K[0][0] * x_err + LQR_K[0][1] * v_err;

    // 旋转分量 (以左轮为正，右轮系数天然反号，因为 LQR_K[1][2] == -LQR_K[0][2])
    fp32 U_yaw = -LQR_K[0][2] * yaw_err - LQR_K[0][3] * dyaw_err;

    const fp32 left_theta_ctrl = chassis_select_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_THETA_FILTER_SOURCE);
    const fp32 left_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_move_control_loop->chassis_left_control, CHASSIS_LEFT_D_THETA_FILTER_SOURCE);
    const fp32 right_theta_ctrl = chassis_select_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_THETA_FILTER_SOURCE);
    const fp32 right_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_move_control_loop->chassis_right_control, CHASSIS_RIGHT_D_THETA_FILTER_SOURCE);

    // 3. 提取维持姿态平衡的扭矩分量
    fp32 U_else_L = LQR_K[0][4] * left_theta_ctrl
                  + LQR_K[0][5] * left_d_theta_ctrl
                  + LQR_K[0][6] * right_theta_ctrl
                  + LQR_K[0][7] * right_d_theta_ctrl
                  - LQR_K[0][8] * (chassis_move_control_loop->chassis_pitch)
                  - LQR_K[0][9] * chassis_move_control_loop->chassis_d_pitch
                  + chassis_move_control_loop->chassis_left_control.Tw_adapt;

    fp32 U_else_R = LQR_K[1][4] * left_theta_ctrl
                  + LQR_K[1][5] * left_d_theta_ctrl
                  + LQR_K[1][6] * right_theta_ctrl
                  + LQR_K[1][7] * right_d_theta_ctrl
                  - LQR_K[1][8] * (chassis_move_control_loop->chassis_pitch)
                  - LQR_K[1][9] * chassis_move_control_loop->chassis_d_pitch
                  + chassis_move_control_loop->chassis_right_control.Tw_adapt;

    fp32 w_L = chassis_move_control_loop->chassis_wheel[0].vel;
    fp32 w_R = chassis_move_control_loop->chassis_wheel[1].vel;

    // 4. 功率限制器计算平动和yaw分量的衰减系数
    WL_PowerManager.Calculate_Decay(U_speed, U_yaw, U_else_L, U_else_R, w_L, w_R);

    fp32 decay_Uspeed = WL_PowerManager.getDecayUspeed();
    fp32 decay_Uyaw   = WL_PowerManager.getDecayUyaw();

    // 5. 重新合成最终的安全轮毂力矩
    chassis_move_control_loop->chassis_wheel[0].wheel_T = (U_speed * decay_Uspeed) + (U_yaw * decay_Uyaw) + U_else_L;
    chassis_move_control_loop->chassis_wheel[1].wheel_T = (U_speed * decay_Uspeed) - (U_yaw * decay_Uyaw) + U_else_R;

    // 6. 左右腿Tbl_t同样应用功率限制后的平动/yaw衰减系数
    fp32 U_speed_leg_L = LQR_K[2][0] * x_err + LQR_K[2][1] * v_err;
    fp32 U_yaw_leg_L   = LQR_K[2][2] * yaw_err + LQR_K[2][3] * dyaw_err;
    fp32 U_else_leg_L  = LQR_K[2][4] * left_theta_ctrl
                       + LQR_K[2][5] * left_d_theta_ctrl
                       + LQR_K[2][6] * right_theta_ctrl
                       + LQR_K[2][7] * right_d_theta_ctrl
                       - LQR_K[2][8] * (chassis_move_control_loop->chassis_pitch)
                       - LQR_K[2][9] * chassis_move_control_loop->chassis_d_pitch;

    fp32 U_speed_leg_R = LQR_K[3][0] * x_err + LQR_K[3][1] * v_err;
    fp32 U_yaw_leg_R   = LQR_K[3][2] * yaw_err + LQR_K[3][3] * dyaw_err;
    fp32 U_else_leg_R  = LQR_K[3][4] * left_theta_ctrl
                       + LQR_K[3][5] * left_d_theta_ctrl
                       + LQR_K[3][6] * right_theta_ctrl
                       + LQR_K[3][7] * right_d_theta_ctrl
                       - LQR_K[3][8] * (chassis_move_control_loop->chassis_pitch)
                       - LQR_K[3][9] * chassis_move_control_loop->chassis_d_pitch;

    chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t =
        (U_speed_leg_L * decay_Uspeed) + (U_yaw_leg_L * decay_Uyaw) + U_else_leg_L;

    chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t =
        (U_speed_leg_R * decay_Uspeed) + (U_yaw_leg_R * decay_Uyaw) + U_else_leg_R;
}

/**
  * @brief          控制循环，根据控制设定值，PID控制器计算电机电流值，进行输出控制
  *                 将"chassis_set_contorl"中得到的三自由度上的设定速度进行运动学逆解算，得到电机设定转速后进行PID运算
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(Chassis_Move *chassis_move_control_loop) {
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

    // 初始化模式直接跳出函数
    if (chassis_move_control_loop->chassis_mode == CHASSIS_ZERO)
    {
        chassis_zero_output(chassis_move_control_loop);
        return;
    }

    /* 此函数已进行：功率控制+所有力矩计算 */
    /* 后方的所有函数和处理均是作为支持力计算和其他附加功能的实现 */
    chassis_lqr_power_control(chassis_move_control_loop);

    chassis_up_leg_angle_control(chassis_move_control_loop);
    chassis_init_level_control(chassis_move_control_loop);
    chassis_init_standup(chassis_move_control_loop);
    chassis_calc_support_force(chassis_move_control_loop);

    fp32 soft_lqr_ratio = chassis_update_soft_lqr_ratio(chassis_move_control_loop);
    chassis_apply_joint_output(chassis_move_control_loop, soft_lqr_ratio);
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

static void chassis_up_leg_angle_control(Chassis_Move *chassis_move_control_loop)
{
    if (chassis_move_control_loop->chassis_mode != CHASSIS_UP ||
       (chassis_move_control_loop->up_state != 2 && chassis_move_control_loop->up_state != 3))
    {
        return;
    }

    chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
    chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
    chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
    chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;

    if (chassis_move_control_loop->up_state != 2 || chassis_move_control_loop->up_leg_reached != 0)
    {
        return;
    }

    // 获取原始角度 (范围 -π ~ +π)
    fp32 left_theta_raw = chassis_move_control_loop->chassis_left_control.theta_l;
    fp32 right_theta_raw = chassis_move_control_loop->chassis_right_control.theta_l;

    // 目标角度 (0~2π坐标系)
    fp32 target_base = CHASSIS_UP_LEG_ANGLE_TARGET_360;

    // 设定旋转角速度 (rad/s)
    fp32 rotate_speed = ROTATE_SPEED;

    // 如果目标比当前位置大，就不断减去 2PI，直到目标值严格小于当前位置，强制它向后退
    fp32 left_target = target_base;
    while (left_target > chassis_move_control_loop->chassis_left_control.up_angle_ramp.out) {
        left_target -= 2.0f * PI;
    }

    fp32 right_target = target_base;
    while (right_target > chassis_move_control_loop->chassis_right_control.up_angle_ramp.out) {
        right_target -= 2.0f * PI;
    }

    chassis_move_control_loop->chassis_left_control.up_angle_ramp.min_value = left_target;
    ramp_calc(&chassis_move_control_loop->chassis_left_control.up_angle_ramp, -rotate_speed); // 给负数，只允许递减

    chassis_move_control_loop->chassis_right_control.up_angle_ramp.min_value = right_target;
    ramp_calc(&chassis_move_control_loop->chassis_right_control.up_angle_ramp, -rotate_speed);

    fp32 left_error = chassis_move_control_loop->chassis_left_control.up_angle_ramp.out - left_theta_raw;
    fp32 right_error = chassis_move_control_loop->chassis_right_control.up_angle_ramp.out - right_theta_raw;

    bool_t left_perfect = (fabs(left_target - left_theta_raw) < CHASSIS_UP_LEG_ANGLE_THRESHOLD);
    bool_t right_perfect = (fabs(right_target - right_theta_raw) < CHASSIS_UP_LEG_ANGLE_THRESHOLD);

    // 计算旋转扭矩
    fp32 left_torque = PID_Calc(&chassis_move_control_loop->chassis_left_control.up_leg_angle_pid, 0.0f, left_error);
    fp32 right_torque = PID_Calc(&chassis_move_control_loop->chassis_right_control.up_leg_angle_pid, 0.0f, right_error);

    // 确保扭矩方向只能是顺时针 (负方向)
    if (left_torque > 0.0f) left_torque = 0.0f;
    if (right_torque > 0.0f) right_torque = 0.0f;

    // 0号1号关节电机控制左腿，扭矩方向为顺时针（取负）
    chassis_move_control_loop->chassis_joint[0].joint_T = -left_torque;
    chassis_move_control_loop->chassis_joint[1].joint_T = -left_torque;
    // 2号3号关节电机控制右腿，扭矩方向为顺时针（取负）
    chassis_move_control_loop->chassis_joint[2].joint_T = right_torque;
    chassis_move_control_loop->chassis_joint[3].joint_T = right_torque;

    // 两边腿都到位后，标记到位
    if (left_perfect && right_perfect)
    {
        chassis_move_control_loop->up_leg_reached = 1;
        PID_clear(&chassis_move_control_loop->chassis_left_control.up_leg_angle_pid);
        PID_clear(&chassis_move_control_loop->chassis_right_control.up_leg_angle_pid);
    }
}

static void chassis_init_level_control(Chassis_Move *chassis_move_control_loop)
{
    if (chassis_move_control_loop->chassis_mode != CHASSIS_INIT ||
        chassis_move_control_loop->init_leg_reach_state != INIT_LEG_UNREACH ||
        chassis_move_control_loop->chassis_state == CHASSIS_NORMAL)
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

    // 按机体pitch倾斜方向分别控制左右腿，直到feedback_update把chassis_state更新为CHASSIS_NORMAL
    chassis_move_control_loop->chassis_joint[0].joint_T = -left_torque;
    chassis_move_control_loop->chassis_joint[1].joint_T = -left_torque;
    chassis_move_control_loop->chassis_joint[2].joint_T = right_torque;
    chassis_move_control_loop->chassis_joint[3].joint_T = right_torque;
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
    chassis_move_control_loop->chassis_left_control.Fbl_inertial = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_left_control.eta * MASS_OF_LEG) * chassis_move_control_loop->chassis_left_control.wbr_control.L
                                                                * chassis_move_control_loop->chassis_d_yaw * chassis_move_control_loop->v_filter / (2 * DRIVE_WHEEL_DIS);
    chassis_move_control_loop->chassis_right_control.Fbl_inertial = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_right_control.eta * MASS_OF_LEG) * chassis_move_control_loop->chassis_right_control.wbr_control.L
                                                                * chassis_move_control_loop->chassis_d_yaw * chassis_move_control_loop->v_filter / (2 * DRIVE_WHEEL_DIS);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_INIT &&
        (chassis_move_control_loop->init_leg_reach_state == INIT_LEG_REACH ||
         chassis_move_control_loop->init_leg_reach_state == INIT_LEG_UNREACH)) // 起身阶段支持力的处理
    {
        // 收腿阶段：停止轮子和LQR转矩输出
        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;

        //左腿腿长PID（收腿，弹簧补偿）
        chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_left_control.fd_leg
                                                                            + chassis_move_control_loop->chassis_left_control.Fbl_spring
                                                                            + 80;

        //右腿腿长PID
        chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_right_control.fd_leg
                                                                             + chassis_move_control_loop->chassis_right_control.Fbl_spring
                                                                             + 80;
    }
    else if (chassis_move_control_loop->chassis_mode == CHASSIS_JUMP) //对于跳跃伸腿过程以及收腿过程力的处理11
    {
        if(chassis_move_control_loop->jump_state == 1)  //对于伸腿过程.不需要弹簧补偿，但是需要重力补偿和腿长PD，加上跳跃补偿
        {
            // 重力补偿 + 左腿腿长PID + 跳跃补偿
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_left_control.fd_leg
                                                                                - chassis_move_control_loop->chassis_left_control.Fbl_gravity
                                                                                - 300;

            // 重力补偿 + 右腿腿长PID + 跳跃补偿
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_right_control.fd_leg
                                                                                 - chassis_move_control_loop->chassis_right_control.Fbl_gravity
                                                                                 - 300;
        }
        else if(chassis_move_control_loop->jump_state == 2) //对于收腿过程 ,不需要重力补偿，只需要弹簧力补偿，然后再收腿
        {
            //左腿腿长PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_left_control.fd_leg
                                                                                + chassis_move_control_loop->chassis_left_control.Fbl_spring;

            //右腿腿长PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_right_control.fd_leg
                                                                                 + chassis_move_control_loop->chassis_right_control.Fbl_spring;
        }
    }
    else // 正常情况下支持力处理
    {
        // 五连杆左右腿支持力计算
        if (chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_TOUCH_GROUND || chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_TOUCH_GROUND)
        {
            // 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID + 左腿roll轴补偿PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_roll
                                                                                - chassis_move_control_loop->chassis_left_control.fd_leg
                                                                                - chassis_move_control_loop->chassis_left_control.Fbl_gravity
                                                                                + chassis_move_control_loop->chassis_left_control.Fbl_spring
                                                                                + chassis_move_control_loop->chassis_left_control.Fbl_inertial;

            // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID + 右腿roll轴补偿PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = chassis_move_control_loop->chassis_right_control.fd_roll
                                                                                 - chassis_move_control_loop->chassis_right_control.fd_leg
                                                                                 + chassis_move_control_loop->chassis_right_control.Fbl_spring
                                                                                 - chassis_move_control_loop->chassis_right_control.Fbl_gravity
                                                                                 - chassis_move_control_loop->chassis_right_control.Fbl_inertial;
        }
        else
        {
            // 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_left_control.fd_leg
                                                                                + chassis_move_control_loop->chassis_left_control.Fbl_spring
                                                                                - chassis_move_control_loop->chassis_left_control.Fbl_gravity;

            // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = - chassis_move_control_loop->chassis_right_control.fd_leg
                                                                                 + chassis_move_control_loop->chassis_right_control.Fbl_spring
                                                                                 - chassis_move_control_loop->chassis_right_control.Fbl_gravity;
        }
    }
}

static fp32 chassis_update_soft_lqr_ratio(Chassis_Move *chassis_move_control_loop)
{
    static fp32 soft_lqr_ratio = 0.0f; // 静态变量，默认 1.0（全功率）

    // 1. 检测模式切换边缘：从 INIT 倒地 或 UP 台阶 切换到 正常平衡模式
    if ((chassis_move_control_loop->last_chassis_mode == CHASSIS_INIT ||
         chassis_move_control_loop->last_chassis_mode == CHASSIS_UP) &&
        (chassis_move_control_loop->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW))
    {
        soft_lqr_ratio = 0.0f;
    }

    // 2. 斜坡恢复：如果比例还没到 1.0，就让它慢慢涨上来
    if (soft_lqr_ratio < 1.0f)
    {
        soft_lqr_ratio += 0.002f;

        // 限幅保护
        if (soft_lqr_ratio > 1.0f) {
            soft_lqr_ratio = 1.0f;
        }
    }

    return soft_lqr_ratio;
}

static void chassis_apply_joint_output(Chassis_Move *chassis_move_control_loop, fp32 soft_lqr_ratio)
{
    // 初始化收腿阶段后，进入起身阶段，起身阶段使用收缩LQR输出
    if (chassis_move_control_loop->chassis_mode == CHASSIS_INIT && chassis_move_control_loop->init_leg_reach_state == INIT_LEG_STANDUP)
    {
        // 2. 软启动轮出扭矩 (防止一接管轮子就狂转)
        chassis_move_control_loop->chassis_wheel[0].wheel_T *= soft_lqr_ratio;
        chassis_move_control_loop->chassis_wheel[1].wheel_T *= soft_lqr_ratio;

        // 左前关节电机力矩
        chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t
                                                            - chassis_move_control_loop->chassis_left_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
        // 左后关节电机力矩
        chassis_move_control_loop->chassis_joint[1].joint_T = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t
                                                            + chassis_move_control_loop->chassis_left_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
        // 右前关节电机力矩
        chassis_move_control_loop->chassis_joint[2].joint_T = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t
                                                            + chassis_move_control_loop->chassis_right_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
        // 右后关节电机力矩
        chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t
                                                            - chassis_move_control_loop->chassis_right_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
    }
    else if (chassis_move_control_loop->chassis_mode == CHASSIS_INIT && chassis_move_control_loop->init_leg_reach_state == INIT_LEG_REACH)
    {
        // 收腿阶段：只输出Fbl_t（腿长PID力），不输出LQR转矩
        chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t;
        chassis_move_control_loop->chassis_joint[1].joint_T =  chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t;
        chassis_move_control_loop->chassis_joint[2].joint_T =  chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t;
        chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t;
    }
    else if (chassis_move_control_loop->chassis_mode == CHASSIS_UP && chassis_move_control_loop->up_state == 3) {
        // 【Phase 3: 用力收腿拽上台阶】只输出 Fbl_t（纯收腿径向力）
        // 收腿
        fp32 Fbl_t_l = -chassis_move_control_loop->chassis_left_control.fd_leg + 50.0f;
        fp32 Fbl_t_r = -chassis_move_control_loop->chassis_right_control.fd_leg + 50.0f;

        chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * Fbl_t_l;
        chassis_move_control_loop->chassis_joint[1].joint_T =  chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * Fbl_t_l;
        chassis_move_control_loop->chassis_joint[2].joint_T =  chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * Fbl_t_r;
        chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * Fbl_t_r;
    }
    else if (chassis_move_control_loop->chassis_mode != CHASSIS_INIT)
    {
        if (chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND
            && chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND)
        {
            // 轮子彻底断动力
            chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
            chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t =
                LQR_K[2][4] * chassis_move_control_loop->chassis_left_control.theta_l +
                LQR_K[2][5] * chassis_move_control_loop->chassis_left_control.d_theta_l;

            // 轮子彻底断动力
            chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
            chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t =
                LQR_K[3][6] * chassis_move_control_loop->chassis_right_control.theta_l +
                LQR_K[3][7] * chassis_move_control_loop->chassis_right_control.d_theta_l;
        }

        chassis_move_control_loop->chassis_wheel[0].wheel_T *= soft_lqr_ratio;
        chassis_move_control_loop->chassis_wheel[1].wheel_T *= soft_lqr_ratio;

        // 左前关节电机力矩
        chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t
                                                            - chassis_move_control_loop->chassis_left_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
        // 左后关节电机力矩
        chassis_move_control_loop->chassis_joint[1].joint_T = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t
                                                            + chassis_move_control_loop->chassis_left_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
        // 右前关节电机力矩
        chassis_move_control_loop->chassis_joint[2].joint_T = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t
                                                            + chassis_move_control_loop->chassis_right_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
        // 右后关节电机力矩
        chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t
                                                            - chassis_move_control_loop->chassis_right_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;//* soft_lqr_ratio;
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
}

// 获取弹簧弹力前馈,输入腿长获得前馈量
fp32 Get_FeedForward_Force(fp32 leg_length_m)
{
    // 限制范围防止计算发散
    if (leg_length_m < 0.17f) leg_length_m = 0.17f;
    if (leg_length_m > 0.35f) leg_length_m = 0.35f;

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

    if (chassis_init_standup->chassis_mode != CHASSIS_INIT)
    {
        return;
    }

    if (chassis_init_standup->last_chassis_state != CHASSIS_NORMAL &&
        chassis_init_standup->chassis_state == CHASSIS_NORMAL)
    {
        fp32 left_raw = chassis_init_standup->chassis_left_control.theta_l;
        fp32 right_raw = chassis_init_standup->chassis_right_control.theta_l;
        chassis_init_standup->chassis_left_control.init_angle_ramp.out = left_raw < 0.0f ? left_raw + 2.0f * PI : left_raw;
        chassis_init_standup->chassis_right_control.init_angle_ramp.out = right_raw < 0.0f ? right_raw + 2.0f * PI : right_raw;
        PID_clear(&chassis_init_standup->chassis_left_control.init_leg_angle_pid);
        PID_clear(&chassis_init_standup->chassis_right_control.init_leg_angle_pid);
    }

    /* 初始化收腿阶段：判断左右腿角度是否到达目标角度 */
    if (chassis_init_standup->init_leg_reach_state == INIT_LEG_UNREACH && chassis_init_standup->chassis_state == CHASSIS_NORMAL)
    {
        // 先停止轮子以及LQR输出
        chassis_init_standup->chassis_left_control.wbr_control.Tbl_t = 0.0f;
        chassis_init_standup->chassis_right_control.wbr_control.Tbl_t = 0.0f;
        chassis_init_standup->chassis_wheel[0].wheel_T = 0.0f;
        chassis_init_standup->chassis_wheel[1].wheel_T = 0.0f;

        // 获取原始角度 (范围 -π ~ +π)
        fp32 left_theta_raw = chassis_init_standup->chassis_left_control.theta_l;
        fp32 right_theta_raw = chassis_init_standup->chassis_right_control.theta_l;

        // 换算到 0 ~ 2π 范围
        fp32 left_theta_360 = left_theta_raw < 0.0f ? left_theta_raw + 2.0f * PI : left_theta_raw;
        fp32 right_theta_360 = right_theta_raw < 0.0f ? right_theta_raw + 2.0f * PI : right_theta_raw;

        // 目标角度 (0~2π坐标系)
        fp32 target_360 = CHASSIS_INIT_LEG_ANGLE_TARGET_360;

        // 设定旋转角速度 (rad/s)
        fp32 rotate_speed = ROTATE_SPEED;

        // 左腿斜坡：动态设置限幅，并以设定的角速度逼近目标
        if (chassis_init_standup->chassis_left_control.init_angle_ramp.out < target_360) {
            chassis_init_standup->chassis_left_control.init_angle_ramp.max_value = target_360;
            ramp_calc(&chassis_init_standup->chassis_left_control.init_angle_ramp, rotate_speed);
        } else {
            chassis_init_standup->chassis_left_control.init_angle_ramp.min_value = target_360;
            ramp_calc(&chassis_init_standup->chassis_left_control.init_angle_ramp, -rotate_speed);
        }

        // 右腿斜坡：动态设置限幅，并以设定的角速度逼近目标
        if (chassis_init_standup->chassis_right_control.init_angle_ramp.out < target_360) {
            chassis_init_standup->chassis_right_control.init_angle_ramp.max_value = target_360;
            ramp_calc(&chassis_init_standup->chassis_right_control.init_angle_ramp, rotate_speed);
        } else {
            chassis_init_standup->chassis_right_control.init_angle_ramp.min_value = target_360;
            ramp_calc(&chassis_init_standup->chassis_right_control.init_angle_ramp, -rotate_speed);
        }


        // 计算逆时针方向的误差 (逆时针 = 角度增大方向)
        // 逆时针误差 = (target - current) mod 2π，保证误差 >= 0 为逆时针转
        fp32 left_error = chassis_init_standup->chassis_left_control.init_angle_ramp.out - left_theta_360;
        fp32 right_error = chassis_init_standup->chassis_right_control.init_angle_ramp.out - right_theta_360;

        // 1. 目标斜坡是否已经跑到底 (说明控制器的设定值已经拉满了)
        bool_t left_ramp_finished = (chassis_init_standup->chassis_left_control.init_angle_ramp.out >= target_360 - 0.05f);
        bool_t right_ramp_finished = (chassis_init_standup->chassis_right_control.init_angle_ramp.out >= target_360 - 0.05f);

        // 2. 腿是否已经停转 (角速度接近 0，说明被地面顶住了)
        // 设置 0.2 rad/s 的极小速度阈值
        const fp32 left_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_init_standup->chassis_left_control, CHASSIS_LEFT_D_THETA_FILTER_SOURCE);
        const fp32 right_d_theta_ctrl = chassis_select_d_theta_signal(&chassis_init_standup->chassis_right_control, CHASSIS_RIGHT_D_THETA_FILTER_SOURCE);
        bool_t left_stopped = (fabs(left_d_theta_ctrl) < 0.2f);
        bool_t right_stopped = (fabs(right_d_theta_ctrl) < 0.2f);

        // 3. 角度是否在安全的“触地区间” (防止在半空中或者刚起步时卡住就误判)
        bool_t left_in_zone = (left_theta_360 > 4.8f);
        bool_t right_in_zone = (right_theta_360 > 4.8f);

        // 4. 综合判断：
        // 情况A：完美进入 0.1 rad 的阈值 (地面较软，或者机体姿态恰好完美)
        bool_t left_perfect = (fabs(target_360 - left_theta_360) < CHASSIS_INIT_LEG_ANGLE_THRESHOLD);
        bool_t right_perfect = (fabs(target_360 - right_theta_360) < CHASSIS_INIT_LEG_ANGLE_THRESHOLD);

        // 情况B：目标已经拉满，但在触地区间内腿停住了 (说明结结实实地顶到了地面，不用等那个 0.1 的阈值了！)
        bool_t left_stalled = (left_ramp_finished && left_stopped && left_in_zone);
        bool_t right_stalled = (right_ramp_finished && right_stopped && right_in_zone);

        // 只要满足其一，即可果断判定触地！
        bool_t left_reached = left_perfect || left_stalled;
        bool_t right_reached = right_perfect || right_stalled;
        // ============================================================

        // PID计算关节电机扭矩，使用逆时针误差作为输入
        // 注意：PID输入为 (测量值, 设定值)，这里我们直接用误差控制
        // 将误差作为set=error, measure=0，这样PID输出 = Kp*error + ...，保证扭矩方向为逆时针
        fp32 left_torque = PID_Calc(&chassis_init_standup->chassis_left_control.init_leg_angle_pid,
                                    0.0f, left_error);
        fp32 right_torque = PID_Calc(&chassis_init_standup->chassis_right_control.init_leg_angle_pid,
                                     0.0f, right_error);

        // 确保扭矩方向只能是逆时针 (正方向)，不允许顺时针回转
        // if (left_torque < 0.0f) left_torque = 0.0f;
        // if (right_torque < 0.0f) right_torque = 0.0f;

        // 0号1号关节电机控制左腿，扭矩大小和方向相同
        chassis_init_standup->chassis_joint[0].joint_T = -left_torque;
        chassis_init_standup->chassis_joint[1].joint_T = -left_torque;
        // 2号3号关节电机控制右腿，扭矩大小和方向相同
        chassis_init_standup->chassis_joint[2].joint_T = right_torque;
        chassis_init_standup->chassis_joint[3].joint_T = right_torque;

        // 两边腿都到位后，进入正常CHASSIS_INIT步骤
        if (left_reached && right_reached)
        {
            chassis_init_standup->init_leg_reach_state = INIT_LEG_REACH;
            PID_clear(&chassis_init_standup->chassis_left_control.init_leg_angle_pid);
            PID_clear(&chassis_init_standup->chassis_right_control.init_leg_angle_pid);
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

