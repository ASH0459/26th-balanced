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
#include "App_Chassis_Power_Control.h"
#include "APL_RC_Hub.h"
#include "wbr.h"
#include "bsp_dwt.h"
#include "kalman_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

    // 拟合矩阵K，A，B
    fp32 fitted_matrix_K[40][6];
    fp32 fitted_matrix_A[15][6];
    fp32 fitted_matrix_B[20][6];

    // 调试用的矩阵K，A，B，
    fp32 LQR_K[4][10] = {
        {-1.5266, -2.2013, -3.4055, -0.81822, -4.8754, -0.86253, -4.0439, -0.61554, -8.0769, -1.1539},
{-1.5266, -2.2013, 3.4055, 0.81822, -4.0439, -0.61554, -4.8754, -0.86253, -8.0769, -1.1539},
{5.821, 9.1053, -13.434, -3.3638, 50.918, 11.271, -7.0304, -1.6797, -89.161, -14.984},
{5.821, 9.1053, 13.434, 3.3638, -7.0304, -1.6797, 50.918, 11.271, -89.161, -14.984},
    };

//     fp32 LQR_K[4][10] = {
//         {-1.5266, -2.2013, -3.4055, -0.81822, -4.8754, -0.86253, -4.0439, -0.61554, -8.0769, -1.1539},
// {-1.5266, -2.2013, 3.4055, 0.81822, -4.0439, -0.61554, -4.8754, -0.86253, -8.0769, -1.1539},
// {5.821, 9.1053, -13.434, -3.3638, 50.918, 11.271, -7.0304, -1.6797, -89.161, -14.984},
// {5.821, 9.1053, 13.434, 3.3638, -7.0304, -1.6797, 50.918, 11.271, -89.161, -14.984},
//     };


    fp32 LQR_A[10][10] = {
        {0 ,1 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0},
        {0, 0, 0, 0, -36.028, 0, -36.028, 0, 0, 0},
        {0 ,0 ,0 ,1 ,0 ,0 ,0 ,0 ,0 ,0},
        {0, 0, 0, 0, -20.784, 0, 20.784, 0, 0, 0},
        {0 ,0 ,0 ,0 ,0 ,1 ,0 ,0 ,0 ,0},
        {0, 0, 0, 0, 422.51, 0, -30.624, 0, 0, 0},
        {0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,0 ,0},
        {0, 0, 0, 0, -30.624, 0, 422.51, 0, 0, 0},
        {0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1},
        {0, 0, 0, 0, -37.43, 0, -37.43, 0, 116.18, 0},
    };
    fp32 LQR_B[10][4] = {
        {0 ,0 ,0 ,0},
        {8.8635, 8.8635, -2.0245, -2.0245},
        {0 ,0 ,0 ,0},
        {-38.093, 38.093, -1.1679, 1.1679},
        {0 ,0 ,0 ,0},
        {1.4674, -88.374, 23.742, -1.7209},
        {0 ,0 ,0 ,0},
        {-88.374, 1.4674, -1.7209, 23.742},
        {0 ,0 ,0 ,0},
        {-2.0477, -2.0477, -11.178, -11.178},
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


/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(Chassis_Move *chassis_move_init);


/**
  * @brief          设置底盘控制模式，主要在'Chassis_Behaviour_Mode_set'函数中改变
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
  * @brief          离地检查识别
  * @param          chassis_move_prediction   指向底盘控制结构体的指针，包含底盘的状态和参数信息
  * @retval         无
  */
    static void chassis_interference_prevention(Chassis_Move *chassis_move_prediction);

void Chassis_Task(void *pvParameters)
{
    /* 底盘运动类 */
    static Chassis_Move chassis_move;

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

        for(int i = 0; i < 4; i++) {
            chassis_move.chassis_joint[i].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, chassis_move.chassis_joint[i].joint_T);
            //chassis_move.chassis_joint[i].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, 0.0f);
        }
        //chassis_move.chassis_joint[0].chassis_joint_measure->Joint_MIT_Control(0, 0, 0, 0, 1.0f);
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

    /* 滤波参数：DT7发送设定值的低通滤波 */
    const static float chassis_DT7_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_DT7_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 chassis_leg_order_filter[1] = {CHASSIS_ACCEL_LEG_NUM};

    /* 滤波初始化：DT7发送设定值的低通滤波 */
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_DT7_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_DT7_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_leg_filter_set, CHASSIS_CONTROL_TIME, chassis_leg_order_filter);
    chassis_move_init->chassis_leg_filter_set.out = 0.35;

    /* 小陀螺旋转速度缩放 */
    chassis_move_init->chassis_spin_change_sen = CHASSIS_SPIN_LOW_SPEED;

    /* 底盘默认模式 */
    chassis_move_init->chassis_mode = CHASSIS_ZERO;

    // 更新数据
    chassis_feedback_update(chassis_move_init);
}



/**
  * @brief          设置底盘控制模式，主要在'Chassis_Behaviour_Mode_set'函数中改变
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
        chassis_move_transit->chassis_leg_set = 0.201f;

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
        chassis_move_transit->chassis_leg_set = 0.201f;
    }

    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_INIT) && (chassis_move_transit->chassis_mode == CHASSIS_INIT))
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
        chassis_move_transit->chassis_leg_set = 0.35;
        chassis_move_transit->chassis_leg_filter_set.out = 0.35;
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
        chassis_move_control->chassis_d_yaw = (WHEEL_RADIUS * (-chassis_move_control->chassis_wheel[0].vel + chassis_move_control->chassis_wheel[1].vel)
                                            - chassis_move_control->chassis_left_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_left_control.wbr_control.theta_l) * chassis_move_control->chassis_left_control.wbr_control.d_theta_l
                                            + chassis_move_control->chassis_right_control.wbr_control.L * arm_cos_f32(chassis_move_control->chassis_right_control.wbr_control.theta_l) * chassis_move_control->chassis_right_control.wbr_control.d_theta_l)
                                            / (2 * DRIVE_WHEEL_DIS);
        chassis_move_control->chassis_yaw = chassis_move_control->chassis_gimbal_data->yaw_relative_angle;
        chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
        chassis_move_control->chassis_yaw_set = 0.0f;
        chassis_move_control->chassis_d_yaw_set = PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw, chassis_move_control->chassis_yaw_set); //转速pid计算
        chassis_move_control->chassis_leg_set = float_constrain(chassis_leg_set - (chassis_move_control->chassis_pitch * chassis_move_control->chassis_pitch * 10 / 9), 0.15f, 0.36f);
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

        chassis_move_control->chassis_v_set = chassis_v_set;
        chassis_move_control->chassis_x_set += chassis_move_control->chassis_v_set * chassis_move_control->dt;
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
    	switch (chassis_move_control->jump_state)
    	{
          	case 0:
    		{
          		break;
        	}
      		case 1:     //腿部压缩
    		{
          		chassis_move_control->chassis_v_set = chassis_move_control->chassis_gimbal_data->chassis_vx;
            	chassis_move_control->chassis_d_yaw_set = 0.0;  //跳跃的时候不能有yaw轴方向转动
            	chassis_move_control->chassis_leg_set -= chassis_move_control->dt / 2.0f;
            	if (chassis_move_control->chassis_leg_set <= 0.15f && chassis_move_control->chassis_left_control.wbr_control.L <= 0.15f && chassis_move_control->chassis_right_control.wbr_control.L <= 0.15f)
            	{
                	chassis_move_control->jump_state = 2;
            	}
				return;
        	}
        	case 2:     //释放腿部
    		{
          		chassis_move_control->chassis_v_set = chassis_move_control->chassis_gimbal_data->chassis_vx;
            	chassis_move_control->chassis_d_yaw_set = 0.0f;
            	chassis_move_control->chassis_leg_set = 0.36;
            	if (chassis_move_control->chassis_left_control.wbr_control.L >= 0.34f && chassis_move_control->chassis_right_control.wbr_control.L >= 0.34f)
            	{
                	chassis_move_control->jump_state = 3;
            	}
				return;
        	}
        	case 3:     //腾空
    		{
          		chassis_move_control->chassis_v_set = 0.0f;
            	chassis_move_control->chassis_d_yaw_set = 0.0f;
            	chassis_move_control->chassis_leg_set = 0.2;
            	if (chassis_move_control->chassis_left_control.wbr_control.L <= 0.2f && chassis_move_control->chassis_right_control.wbr_control.L <= 0.2f)
            	{
                	chassis_move_control->jump_state = 4;
            	}
				return;
        	}
            case 4:
    		{
          		chassis_move_control->chassis_v_set = 0.0f;
            	chassis_move_control->chassis_d_yaw_set = 0.0f;
            	chassis_move_control->chassis_leg_set += chassis_move_control->dt;
            	if (chassis_move_control->chassis_leg_set >= 0.25f && chassis_move_control->chassis_left_control.wbr_control.L >= 0.25f && chassis_move_control->chassis_right_control.wbr_control.L >= 0.25f)
            	{
                	chassis_move_control->jump_state = 0;
            	    //跳跃完成，切回云台模式
            	    chassis_move_control->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW;
            	}
				return;

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
    // chassis_move_update->vaEstimateKF_F_VA[0] = 1.0f;
    // chassis_move_update->vaEstimateKF_F_VA[1] = chassis_move_update->dt;
    // chassis_move_update->vaEstimateKF_F_VA[2] = 0.0f;
    // chassis_move_update->vaEstimateKF_F_VA[3] = 1.0f;
    // memcpy(chassis_move_update->chassis_vaestimatekf_xv.F_data, chassis_move_update->vaEstimateKF_F_VA, sizeof(chassis_move_update->vaEstimateKF_F_VA));
    // chassis_move_update->chassis_vaestimatekf_va.MeasuredVector[0] = chassis_move_update->v_obs;
    // chassis_move_update->chassis_vaestimatekf_va.MeasuredVector[1] = chassis_move_update->chassis_accel_n_y;
    // //卡尔曼滤波器更新函数
    // Kalman_Filter_Update(&chassis_move_update->chassis_vaestimatekf_va);
    // // 获得滤波后的机体速度
    // chassis_move_update->v_real = chassis_move_update->chassis_vaestimatekf_va.FilteredValue[0];

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
    chassis_move_update->chassis_accel_n_x = -*(chassis_move_update->chassis_motion_accel_n + INS_MONTION_N_ACCEL_X_ADDRESS_OFFSET);
    chassis_move_update->chassis_accel_n_y = *(chassis_move_update->chassis_motion_accel_n + INS_MONTION_N_ACCEL_Y_ADDRESS_OFFSET);
    chassis_move_update->chassis_accel_n_z = *(chassis_move_update->chassis_motion_accel_n + INS_MONTION_N_ACCEL_Z_ADDRESS_OFFSET);

    // 更新底盘姿态角（yaw轴根据电机角度计算，pitch和roll根据陀螺仪直接读取）
    chassis_move_update->chassis_pitch = *(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET);
    chassis_move_update->chassis_roll = -*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

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
    memcpy(chassis_move_update->chassis_left_control.chassis_vaestimatekf_theta.F_data, chassis_move_update->vaEstimateKF_F_theta, sizeof(chassis_move_update->vaEstimateKF_F_theta));
    memcpy(chassis_move_update->chassis_right_control.chassis_vaestimatekf_theta.F_data, chassis_move_update->vaEstimateKF_F_theta, sizeof(chassis_move_update->vaEstimateKF_F_theta));
    chassis_move_update->chassis_left_control.chassis_vaestimatekf_theta.MeasuredVector[0] = chassis_move_update->chassis_left_control.wbr_control.theta_l;
    chassis_move_update->chassis_right_control.chassis_vaestimatekf_theta.MeasuredVector[0] = chassis_move_update->chassis_right_control.wbr_control.theta_l;
    chassis_move_update->chassis_left_control.chassis_vaestimatekf_theta.MeasuredVector[1] = chassis_move_update->chassis_left_control.wbr_control.d_theta_l;
    chassis_move_update->chassis_right_control.chassis_vaestimatekf_theta.MeasuredVector[1] = chassis_move_update->chassis_right_control.wbr_control.d_theta_l;
    //卡尔曼滤波器更新函数
     Kalman_Filter_Update(&chassis_move_update->chassis_left_control.chassis_vaestimatekf_theta);
     Kalman_Filter_Update(&chassis_move_update->chassis_right_control.chassis_vaestimatekf_theta);
    // 获得滤波后的角度和速度
    chassis_move_update->chassis_left_control.theta_l = chassis_move_update->chassis_left_control.chassis_vaestimatekf_theta.FilteredValue[0];
    chassis_move_update->chassis_right_control.theta_l = chassis_move_update->chassis_right_control.chassis_vaestimatekf_theta.FilteredValue[0];
    chassis_move_update->chassis_left_control.d_theta_l = chassis_move_update->chassis_left_control.chassis_vaestimatekf_theta.FilteredValue[1];
    chassis_move_update->chassis_right_control.d_theta_l = chassis_move_update->chassis_right_control.chassis_vaestimatekf_theta.FilteredValue[1];

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
                                                    * arm_cos_f32(chassis_move_update->chassis_left_control.wbr_control.theta_l));

    // 右轮地面支持力
    chassis_move_update->chassis_right_control.Fwn = chassis_move_update->chassis_right_control.wbr_control.Fbl_r * arm_cos_f32(chassis_move_update->chassis_right_control.wbr_control.theta_l)
                                                    + MASS_OF_LEG * (GRAVITY_ACCELERATION + chassis_move_update->chassis_accel_n_z
                                                    - (1 - chassis_move_update->chassis_right_control.eta) * chassis_move_update->chassis_right_control.wbr_control.dd_L
                                                    * arm_cos_f32(chassis_move_update->chassis_right_control.wbr_control.theta_l));

    // 计算矩阵K，A，B
//    decompose_fitted_matrix_to_K(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_K, LQR_K);
//    decompose_fitted_matrix_to_A(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_A, LQR_A);
//    decompose_fitted_matrix_to_B(&chassis_move_update->chassis_left_control.wbr_control, &chassis_move_update->chassis_right_control.wbr_control, fitted_matrix_B, LQR_B);

    // mpc状态预测，预测下一时刻轮子状态并对轮毂电机力矩进行补偿(目前还是定腿长控制)
    chassis_move_update->chassis_left_control.v_real_n = (chassis_move_update->v_real
                                                        + LQR_A[1][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                        + LQR_A[1][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                        + LQR_A[1][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        												+ (LQR_B[1][0] * chassis_move_update->chassis_wheel[0].tor
                                                        + LQR_B[1][1] * chassis_move_update->chassis_wheel[1].tor
                                                        + LQR_B[1][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                        + LQR_B[1][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    chassis_move_update->chassis_right_control.v_real_n = (chassis_move_update->v_real
                                                        + LQR_A[1][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                        + LQR_A[1][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                        + LQR_A[1][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        												+ (LQR_B[1][0] * chassis_move_update->chassis_wheel[0].tor
                                                        + LQR_B[1][1] * chassis_move_update->chassis_wheel[1].tor
                                                        + LQR_B[1][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                        + LQR_B[1][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    chassis_move_update->chassis_left_control.chassis_d_yaw_n = (chassis_move_update->chassis_d_yaw
                                                            + LQR_A[3][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                            + LQR_A[3][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                            + LQR_A[3][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        													+ (LQR_B[3][0] * chassis_move_update->chassis_wheel[0].tor
                                                            + LQR_B[3][1] * chassis_move_update->chassis_wheel[1].tor
                                                            + LQR_B[3][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                            + LQR_B[3][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    chassis_move_update->chassis_right_control.chassis_d_yaw_n = (chassis_move_update->chassis_d_yaw
                                                            + LQR_A[3][4] * chassis_move_update->chassis_left_control.wbr_control.theta_l
                                                            + LQR_A[3][6] * chassis_move_update->chassis_right_control.wbr_control.theta_l
                                                            + LQR_A[3][8] * chassis_move_update->chassis_pitch) * chassis_move_update->dt
        													+ (LQR_B[3][0] * chassis_move_update->chassis_wheel[0].tor
                                                            + LQR_B[3][1] * chassis_move_update->chassis_wheel[1].tor
                                                            + LQR_B[3][2] * chassis_move_update->chassis_left_control.wbr_control.Tbl_r
                                                            + LQR_B[3][3] * chassis_move_update->chassis_right_control.wbr_control.Tbl_r) * chassis_move_update->dt;

    // 驱动轮角速度预测
    chassis_move_update->chassis_left_control.d_theta_w_n = (chassis_move_update->chassis_left_control.v_real_n - DRIVE_WHEEL_DIS * chassis_move_update->chassis_left_control.chassis_d_yaw_n) / WHEEL_RADIUS;
    chassis_move_update->chassis_right_control.d_theta_w_n = (chassis_move_update->chassis_right_control.v_real_n - DRIVE_WHEEL_DIS * chassis_move_update->chassis_right_control.chassis_d_yaw_n) / WHEEL_RADIUS;
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
    if(chassis_move_prediction->chassis_mode != CHASSIS_INIT && chassis_move_prediction->chassis_mode != CHASSIS_ZERO)
    {
        // 如果腿部支持力小于10，则判断离地
        if (chassis_move_prediction->chassis_left_control.Fwn <= 20.0f)
        {
            chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_OFF_GROUND;
        }
        else
        {
            chassis_move_prediction->chassis_left_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
        }

        if (chassis_move_prediction->chassis_right_control.Fwn <= 20.0f)
        {
            chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_OFF_GROUND;
        }
        else
        {
            chassis_move_prediction->chassis_right_control.chassis_off_ground_detection = CHASSIS_TOUCH_GROUND;
        }

        if (chassis_move_prediction->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND
            && chassis_move_prediction->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND)
        {
            //数据不再更新，保持为离地前的那一刻
            // chassis_move_prediction->v_real = chassis_move_prediction->v_real_last;
            // chassis_move_prediction->x_real = chassis_move_prediction->x_real_last;
            // chassis_move_prediction->v_filter = chassis_move_prediction->v_filter_last;
            // chassis_move_prediction->chassis_yaw = chassis_move_prediction->chassis_yaw_p;
            // chassis_move_prediction->chassis_d_yaw = chassis_move_prediction->chassis_d_yaw_p;
        }
    }

}


/**
  * @brief          控制循环，根据控制设定值，PID控制器计算电机电流值，进行输出控制
  *                 将"chassis_set_contorl"中得到的三自由度上的设定速度进行运动学逆解算，得到电机设定转速后进行PID运算
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(Chassis_Move *chassis_move_control_loop)
{
    /*本部分代码包括以下部分：
     * 1.底盘运动学逆解算得到底盘电机速度设定值
     * 2.对电机速度设定值进行线性缩放并计算速度环PID
     * 3.功率控制（内含缓冲能量环PID计算）
     * 4.输出give_current给电机
     * 5.待续
     * */

    if (chassis_move_control_loop == NULL)
    {
        return;
    }

    // 初始化模式直接跳出函数
    if (chassis_move_control_loop->chassis_mode == CHASSIS_ZERO)
    {
        chassis_move_control_loop->chassis_joint[0].joint_T = 0.0f;
        chassis_move_control_loop->chassis_joint[1].joint_T = 0.0f;
        chassis_move_control_loop->chassis_joint[2].joint_T = 0.0f;
        chassis_move_control_loop->chassis_joint[3].joint_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
        return;
    }

    // 离地检测
    if (chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND
        && chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_OFF_GROUND)
    {
        // // 1. 备份需要保留的腿部PD参数
        // // 对应矩阵第3行(Row 2)：左腿关节力矩 -> 对应 左腿角度(Col 4) 和 左腿角速度(Col 5)
        // fp32 k_left_p = LQR_K[2][4];
        // fp32 k_left_d = LQR_K[2][5];
        //
        // // 对应矩阵第4行(Row 3)：右腿关节力矩 -> 对应 右腿角度(Col 6) 和 右腿角速度(Col 7)
        // fp32 k_right_p = LQR_K[3][6];
        // fp32 k_right_d = LQR_K[3][7];
        //
        // // 2. 批量将整个矩阵置为 0
        // memset(LQR_K, 0, sizeof(LQR_K));
        //
        // // 3. 还原保留的参数 (实现空中锁定腿部姿态)
        // LQR_K[2][4] = k_left_p;
        // LQR_K[2][5] = k_left_d;
        //
        // LQR_K[3][6] = k_right_p;
        // LQR_K[3][7] = k_right_d;
    }

    // 左轮毂力矩，直接输出
    chassis_move_control_loop->chassis_wheel[0].wheel_T =
                                                          LQR_K[0][0] * (-chassis_move_control_loop->x_filter + chassis_move_control_loop->chassis_x_set)
                                                         + LQR_K[0][1] * (-chassis_move_control_loop->v_filter + chassis_move_control_loop->chassis_v_set)
                                                         - LQR_K[0][2] * (chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set)
                                                         - LQR_K[0][3] * (chassis_move_control_loop->chassis_d_yaw - chassis_move_control_loop->chassis_d_yaw_set)
                                                         + LQR_K[0][4] * chassis_move_control_loop->chassis_left_control.theta_l
                                                         + LQR_K[0][5] * chassis_move_control_loop->chassis_left_control.d_theta_l
                                                         + LQR_K[0][6] * chassis_move_control_loop->chassis_right_control.theta_l
                                                         + LQR_K[0][7] * chassis_move_control_loop->chassis_right_control.d_theta_l
                                                        - LQR_K[0][8] * (chassis_move_control_loop->chassis_pitch)
                                                        - LQR_K[0][9] * chassis_move_control_loop->chassis_d_pitch;
                                                        //- chassis_move_control_loop->chassis_left_control.Tw_adapt;

    // 右轮毂力矩，直接输出
    chassis_move_control_loop->chassis_wheel[1].wheel_T =
                                                          LQR_K[1][0] * (-chassis_move_control_loop->x_filter + chassis_move_control_loop->chassis_x_set)
                                                         + LQR_K[1][1] * (-chassis_move_control_loop->v_filter + chassis_move_control_loop->chassis_v_set)
                                                         - LQR_K[1][2] * (chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set)
                                                         - LQR_K[1][3] * (chassis_move_control_loop->chassis_d_yaw - chassis_move_control_loop->chassis_d_yaw_set)
                                                         + LQR_K[1][4] * chassis_move_control_loop->chassis_left_control.theta_l
                                                         + LQR_K[1][5] * chassis_move_control_loop->chassis_left_control.d_theta_l
                                                         + LQR_K[1][6] * chassis_move_control_loop->chassis_right_control.theta_l
                                                         + LQR_K[1][7] * chassis_move_control_loop->chassis_right_control.d_theta_l
                                                        - LQR_K[1][8] * (chassis_move_control_loop->chassis_pitch)
                                                        - LQR_K[1][9] * chassis_move_control_loop->chassis_d_pitch;
                                                        //- chassis_move_control_loop->chassis_right_control.Tw_adapt;

    // 左腿转矩
    chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t =
                                                                          -LQR_K[2][0] * (chassis_move_control_loop->x_filter - chassis_move_control_loop->chassis_x_set)
                                                                        - LQR_K[2][1] * (chassis_move_control_loop->v_filter - chassis_move_control_loop->chassis_v_set)
                                                                         + LQR_K[2][2] * (chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set)
                                                                         + LQR_K[2][3] * (chassis_move_control_loop->chassis_d_yaw - chassis_move_control_loop->chassis_d_yaw_set)
                                                                         + LQR_K[2][4] * chassis_move_control_loop->chassis_left_control.theta_l
                                                                         + LQR_K[2][5] * chassis_move_control_loop->chassis_left_control.d_theta_l
                                                                         + LQR_K[2][6] * chassis_move_control_loop->chassis_right_control.theta_l
                                                                         + LQR_K[2][7] * chassis_move_control_loop->chassis_right_control.d_theta_l
                                                                         - LQR_K[2][8] * (chassis_move_control_loop->chassis_pitch)
                                                                         - LQR_K[2][9] * chassis_move_control_loop->chassis_d_pitch;

    // 右腿转矩
    chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t =
                                                                          -LQR_K[3][0] * (chassis_move_control_loop->x_filter - chassis_move_control_loop->chassis_x_set)
                                                                        - LQR_K[3][1] * (chassis_move_control_loop->v_filter - chassis_move_control_loop->chassis_v_set)
                                                                         + LQR_K[3][2] * (chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set)
                                                                         + LQR_K[3][3] * (chassis_move_control_loop->chassis_d_yaw - chassis_move_control_loop->chassis_d_yaw_set)
                                                                         + LQR_K[3][4] * chassis_move_control_loop->chassis_left_control.theta_l
                                                                         + LQR_K[3][5] * chassis_move_control_loop->chassis_left_control.d_theta_l
                                                                         + LQR_K[3][6] * chassis_move_control_loop->chassis_right_control.theta_l
                                                                         + LQR_K[3][7] * chassis_move_control_loop->chassis_right_control.d_theta_l
                                                                         - LQR_K[3][8] * (chassis_move_control_loop->chassis_pitch)
                                                                         - LQR_K[3][9] * chassis_move_control_loop->chassis_d_pitch;

    if (chassis_move_control_loop->chassis_mode == CHASSIS_INIT) {
        chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t = 0.0f;
        chassis_move_control_loop->chassis_wheel[0].wheel_T = 0.0f;
        chassis_move_control_loop->chassis_wheel[1].wheel_T = 0.0f;
    }

	// roll轴PID补偿
	chassis_move_control_loop->chassis_left_control.fd_roll = PID_Calc(&chassis_move_control_loop->chassis_left_control.roll_control, chassis_move_control_loop->chassis_roll, chassis_move_control_loop->chassis_roll_set);
	chassis_move_control_loop->chassis_right_control.fd_roll = PID_Calc(&chassis_move_control_loop->chassis_right_control.roll_control, chassis_move_control_loop->chassis_roll, chassis_move_control_loop->chassis_roll_set);

	// 腿长PID
	chassis_move_control_loop->chassis_left_control.fd_leg = PID_Calc(&chassis_move_control_loop->chassis_left_control.leg_pid_control, chassis_move_control_loop->chassis_left_control.wbr_control.L, chassis_move_control_loop->chassis_leg_set);
	chassis_move_control_loop->chassis_right_control.fd_leg = PID_Calc(&chassis_move_control_loop->chassis_right_control.leg_pid_control, chassis_move_control_loop->chassis_right_control.wbr_control.L, chassis_move_control_loop->chassis_leg_set);

    // 重力补偿前馈
    chassis_move_control_loop->chassis_left_control.Fbl_gravity = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_left_control.eta * MASS_OF_LEG) * GRAVITY_ACCELERATION;
    chassis_move_control_loop->chassis_right_control.Fbl_gravity = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_right_control.eta * MASS_OF_LEG) * GRAVITY_ACCELERATION;

    // 侧向惯性力矩补偿前馈
    chassis_move_control_loop->chassis_left_control.Fbl_inertial = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_left_control.eta * MASS_OF_LEG) * chassis_move_control_loop->chassis_left_control.wbr_control.L
                                                                * chassis_move_control_loop->chassis_d_yaw * chassis_move_control_loop->v_filter / (2 * DRIVE_WHEEL_DIS);
    chassis_move_control_loop->chassis_right_control.Fbl_inertial = (MASS_OF_BODY / 2 + chassis_move_control_loop->chassis_right_control.eta * MASS_OF_LEG) * chassis_move_control_loop->chassis_right_control.wbr_control.L
                                                                * chassis_move_control_loop->chassis_d_yaw * chassis_move_control_loop->v_filter / (2 * DRIVE_WHEEL_DIS);

    // if(chassis_move_control_loop->jump_state == 2)
    // {
    //     //跳跃补偿前馈
    //     chassis_move_control_loop->chassis_right_control.Tbl_jump = 25;
    //
    //     chassis_move_control_loop->chassis_left_control.Tbl_jump  = 25;
    //
    // }
    // else
    // {
    //     chassis_move_control_loop->chassis_left_control.Tbl_jump  = 0.0f;
    //
    //     chassis_move_control_loop->chassis_right_control.Tbl_jump = 0.0f;
    // }


    if(chassis_move_control_loop->chassis_mode == CHASSIS_INIT)
    {
        //左腿腿长PID
        chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_leg;

        //右腿腿长PID
        chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_right_control.fd_leg;
    }
    else
    {
        // 五连杆左右腿支持力计算
        if (chassis_move_control_loop->chassis_left_control.chassis_off_ground_detection == CHASSIS_TOUCH_GROUND || chassis_move_control_loop->chassis_right_control.chassis_off_ground_detection == CHASSIS_TOUCH_GROUND)
        {
            // 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID + 左腿roll轴补偿PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = //-chassis_move_control_loop->chassis_left_control.fd_roll
                                                                                - chassis_move_control_loop->chassis_left_control.fd_leg
                                                                                - chassis_move_control_loop->chassis_left_control.Fbl_gravity
                                                                                + chassis_move_control_loop->chassis_left_control.Fbl_inertial;

            // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID + 右腿roll轴补偿PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = //chassis_move_control_loop->chassis_right_control.fd_roll
                                                                                 - chassis_move_control_loop->chassis_right_control.fd_leg
                                                                                - chassis_move_control_loop->chassis_right_control.Fbl_gravity
                                                                                - chassis_move_control_loop->chassis_right_control.Fbl_inertial;
        }
        else
        {
            // 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID
            chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_leg
                                                                                - chassis_move_control_loop->chassis_left_control.Fbl_gravity
                                                                                + chassis_move_control_loop->chassis_left_control.Fbl_inertial;

            // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID
            chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_right_control.fd_leg
                                                                                - chassis_move_control_loop->chassis_right_control.Fbl_gravity
                                                                                - chassis_move_control_loop->chassis_right_control.Fbl_inertial;
        }
    }

 //    if (chassis_move_control_loop->chassis_mode == CHASSIS_JUMP)
	// {
	// 	// 重力补偿 + 侧向惯性力矩补偿 + 左腿腿长PID
 //        chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_left_control.fd_leg
 //                                                                            - chassis_move_control_loop->chassis_left_control.Fbl_gravity
 //                                                                            + chassis_move_control_loop->chassis_left_control.Fbl_inertial;
 //
 //        // 重力补偿 + 侧向惯性力矩补偿 + 右腿腿长PID
 //        chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t = -chassis_move_control_loop->chassis_right_control.fd_leg
 //                                                                            - chassis_move_control_loop->chassis_right_control.Fbl_gravity
 //                                                                            - chassis_move_control_loop->chassis_right_control.Fbl_inertial;
	// }

    // 左前关节电机力矩
    chassis_move_control_loop->chassis_joint[0].joint_T = -chassis_move_control_loop->chassis_left_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t
                                                        - chassis_move_control_loop->chassis_left_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;
                                                        //+ chassis_move_control_loop->chassis_left_control.Tbl_jump;
    // 左后关节电机力矩
    chassis_move_control_loop->chassis_joint[1].joint_T = chassis_move_control_loop->chassis_left_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_left_control.wbr_control.Fbl_t
                                                        + chassis_move_control_loop->chassis_left_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_left_control.wbr_control.Tbl_t;
                                                        //+ chassis_move_control_loop->chassis_left_control.Tbl_jump;
    // 右前关节电机力矩
    chassis_move_control_loop->chassis_joint[2].joint_T = chassis_move_control_loop->chassis_right_control.wbr_control.J[0][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t
                                                        + chassis_move_control_loop->chassis_right_control.wbr_control.J[0][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;
                                                        //+ chassis_move_control_loop->chassis_right_control.Tbl_jump;
    // 右后关节电机力矩
    chassis_move_control_loop->chassis_joint[3].joint_T = -chassis_move_control_loop->chassis_right_control.wbr_control.J[1][0] * chassis_move_control_loop->chassis_right_control.wbr_control.Fbl_t
                                                        - chassis_move_control_loop->chassis_right_control.wbr_control.J[1][1] * chassis_move_control_loop->chassis_right_control.wbr_control.Tbl_t;
                                                        //+ chassis_move_control_loop->chassis_right_control.Tbl_jump;

	// 关节点击力矩限幅，最大值按照关节电机最大扭矩限制
    chassis_move_control_loop->chassis_wheel[0].wheel_T = float_constrain(chassis_move_control_loop->chassis_wheel[0].wheel_T, -6.3f, 6.3f);
    chassis_move_control_loop->chassis_wheel[1].wheel_T = float_constrain(chassis_move_control_loop->chassis_wheel[1].wheel_T, -6.3f, 6.3f);
	chassis_move_control_loop->chassis_joint[0].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[0].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
	chassis_move_control_loop->chassis_joint[1].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[1].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
	chassis_move_control_loop->chassis_joint[2].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[2].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);
	chassis_move_control_loop->chassis_joint[3].joint_T = float_constrain(chassis_move_control_loop->chassis_joint[3].joint_T, JOINT_MIN_TORQUE, JOINT_MAX_TORQUE);

    // 数据更新，用于离地
    chassis_move_control_loop->x_real_last = chassis_move_control_loop->x_real;
    chassis_move_control_loop->v_real_last = chassis_move_control_loop->v_real;
    chassis_move_control_loop->x_filter_last = chassis_move_control_loop->x_filter;
    chassis_move_control_loop->v_filter_last = chassis_move_control_loop->v_filter;
	chassis_move_control_loop->chassis_yaw_p = chassis_move_control_loop->chassis_yaw;
	chassis_move_control_loop->chassis_d_yaw_p = chassis_move_control_loop->chassis_d_yaw;
}

#ifdef __cplusplus
}
#endif

