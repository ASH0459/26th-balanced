/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef INFANTRY_ROBOT_APP_CHASSIS_TASK_H
#define INFANTRY_ROBOT_APP_CHASSIS_TASK_H

#include "main.h"

#include "Dev_CAN_Receive.h"
#include "Dev_Remote_Control.h"

#include "Alg_PID.h"
#include "Alg_UserLib.h"

#include "APL_RC_Hub.h"
#include "wbr.h"
#include "kalman_filter.h"
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

// 收缩系数
#define INIT_LQR_VAL                     1.0f

// 初始化腿旋转速度
#define ROTATE_SPEED                    0.7f

// 重力加速度
#define GRAVITY_ACCELERATION			9.81f

// 机体到腿部质心的距离
#define L_B                             0.08676f

// 腿转动惯量
#define I_L                             0.05892206f

// 驱动轮补偿系数
#define K_APAPT							0.25f

// 任务开始前空闲一段时间
#define CHASSIS_TASK_INIT_TIME			1000

// 最长腿长和最短腿长 待改
#define LEG_MAX_LENGTH					0.3f
#define LEG_MIN_LENGTH					0.1f

// 底盘任务控制间隔
#define CHASSIS_CONTROL_TIME_MS			1
#define JOINT_MOTOR_lIMIT_TIME			1

// 最大最小速度设定
#define NORMAL_MAX_CHASSIS_SPEED_V		0.5f
#define NORMAL_MIN_CHASSIS_SPEED_V		-0.5f

// 小陀螺最大最小速度设定
#define ROTATION_MAX_CHASSIS_SPEED_V	0.5f
#define ROTATION_MIN_CHASSIS_SPEED_V	-0.5f

// 关节最大最小力矩
#define  JOINT_MAX_TORQUE				40.0f
#define  JOINT_MIN_TORQUE				-40.0f

// 腿的最大最小长度
#define CHASSIS_LEG_MAX					0.35f
#define CHASSIS_LEG_MIN					0.201f

// 初始化收腿目标角度 (rad)，原始坐标系下为-1.26rad，换算到0~2π下为2π-1.26
#define CHASSIS_INIT_LEG_ANGLE_TARGET_RAW   (-1.26f)
#define CHASSIS_INIT_LEG_ANGLE_TARGET_360   5.1f
// 初始化收腿角度到位阈值 (rad)
#define CHASSIS_INIT_LEG_ANGLE_THRESHOLD (0.1f)
// 初始化收腿PID参数
#define INIT_LEG_ANGLE_PID_KP           5.0f
#define INIT_LEG_ANGLE_PID_KI           0.0f
#define INIT_LEG_ANGLE_PID_KD           10.0f
#define INIT_LEG_ANGLE_PID_MAX_OUT      20.0f
#define INIT_LEG_ANGLE_PID_MAX_IOUT     0.0f

// 上台阶收腿目标角度 (0~2π坐标系, rad)
#define CHASSIS_UP_LEG_ANGLE_TARGET_360   5.2f
// 上台阶收腿角度到位阈值 (rad)
#define CHASSIS_UP_LEG_ANGLE_THRESHOLD    (0.1f)
// 上台阶收腿完成腿长阈值
#define CHASSIS_UP_LEG_LENGTH_THRESHOLD   0.18f
// 上台阶收腿PID参数
#define UP_LEG_ANGLE_PID_KP              5.0f
#define UP_LEG_ANGLE_PID_KI              0.0f
#define UP_LEG_ANGLE_PID_KD              10.0f
#define UP_LEG_ANGLE_PID_MAX_OUT         20.0f
#define UP_LEG_ANGLE_PID_MAX_IOUT        0.0f

//yaw轴跟随PID
#define YAW_PID_KP						6.0f
#define YAW_PID_KI						0.0f
#define YAW_PID_KD						3.0f

#define YAW_PID_MAX_OUT				    6.0f
#define YAW_PID_MAX_IOUT				0.0f

// 左右腿roll轴PID
#define ROLL_PID_KP						1000.0f
#define ROLL_PID_KI						0.0f
#define ROLL_PID_KD						300.0f
#define ROLL_PID_MAX_OUT				400.0f
#define ROLL_PID_MAX_IOUT				0.0f

#define  CHASSIS_X_BACK                  0.45f
// 左右腿长度PID
#define LEG_PID_KP						2000.0f
#define LEG_PID_KI						0.0f
#define LEG_PID_KD						100000.0f
#define LEG_PID_MAX_OUT					200.0f  //300
#define LEG_PID_MAX_IOUT				0.0f

/* 轮子相关数据 */
// 轮子质量 KG
#define MASS_OF_WHEEL					0.4863f
// 轮子半径 m
#define WHEEL_RADIUS					0.058f
// 驱动轮 轮距的一半 m
#define DRIVE_WHEEL_DIS				0.24155f

/* 腿部相关数据 */
// 腿部质量
#define MASS_OF_LEG						2.109f
// 机体到腿部质心距离拟合直线斜率和截距
#define L_BI_SLOPE						0.85532f
#define L_BI_INTERCEPT					-0.027935f
// 腿部转动惯量拟合直线斜率和截距
#define I_L_SLOPE						0.034616f
#define I_L_INTERCEPT					0.014371f

/* 机体相关数据 */
// 机体质量
#define MASS_OF_BODY					20.0f

// 驱动轮补偿系数
#define K_APAPT							0.25f

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1

//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0 //右侧按键开关

//遥控器前进摇杆（max 660）转化成车体前后速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0025f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.0009f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

/*  */
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_LEG_NUM 0.3333333333f
#define CHASSIS_ACCEL_UP_NUM 0.5f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//底盘与云台之间的距离
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//底盘小陀螺按键
#define GYROSCOPE_KEY       KEY_PRESSED_OFFSET_SHIFT
#define GYROSCOPE_OFF_KEY   KEY_PRESSED_OFFSET_B

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY   KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY    KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY    KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY   KEY_PRESSED_OFFSET_D

// 键盘控制最大速度 (m/s)
#define CHASSIS_KEY_MAX_SPEED   2.0f
#define CHASSIS_KEY_MAX_SPEED_UP   1.0f

#define CHASSIS_KEY_ACCEL 1.8f

// 键盘速度斜坡步长 (m/s per loop, 500Hz下 0→1.5m/s 约0.75秒)
#define CHASSIS_KEY_RAMP_STEP   0.004f

// M3508电机原始转速数据转化成Rpm的比例  3591/187
//#define M3508_MOTOR_RPM_TO_VECTOR           0.052074631021999f
#define M3508_MOTOR_RPM_TO_VECTOR           0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN     M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 40.0f

/* 无超电 X方向最大速度 */
#define NORMAL_MAX_CHASSIS_SPEED_X 4.5f

/* 无超电 Y方向最大速度 */
#define NORMAL_MAX_CHASSIS_SPEED_Y 4.5f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.0f

//底盘小陀螺减缓速度
#define CHASSIS_SPIN_LOW_SPEED 1.5f

//底盘小陀螺基本速度
#define CHASSIS_SPIN_MAIN_SPEED 10.0f

//底盘小陀螺减去x,y方向的速度比例系数
#define CHASSIS_SPIN_LOW_SEN 0.6

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f

//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f


//底盘摇杆x方向PID
#define CHASSIS_DT7_X_PID_KP 0.5f
#define CHASSIS_DT7_X_PID_KI 0.0f
#define CHASSIS_DT7_X_PID_KD 0.0f
#define CHASSIS_DT7_X_PID_MAX_OUT 10.0f
#define CHASSIS_DT7_X_PID_MAX_IOUT 2.0f

//底盘遥感y方向PID
#define CHASSIS_DT7_Y_PID_KP 0.5f
#define CHASSIS_DT7_Y_PID_KI 0.0f
#define CHASSIS_DT7_Y_PID_KD 0.0f
#define CHASSIS_DT7_Y_PID_MAX_OUT 10.0f
#define CHASSIS_DT7_Y_PID_MAX_IOUT 2.0f

/* 底盘状态 */
typedef enum
{
    CHASSIS_FOLLOW_GIMBAL_YAW,   // 底盘跟随云台
    CHASSIS_FOLLOW_CHASSIS_YAW,  // 底盘有底盘角度控制闭环
    CHASSIS_ZERO,                // 底盘无力模式
    CHASSIS_GYROSCOPE,           // 底盘小陀螺模式
    CHASSIS_INIT,                // 初始化，翻倒自救
    CHASSIS_JUMP,                 // 底盘跳跃模式
    CHASSIS_UP                   // 上台阶模式
}Chassis_Mode_e;

typedef enum
{
    CHASSIS_OFF_GROUND,													// 离地
    CHASSIS_TOUCH_GROUND												// 触底
} chassis_off_ground_detection_e;

#ifdef __cplusplus


/* 底盘关节电机数据 */
class Chassis_Joint_Motor {
    public:
    const Joint_Motor_Measure* chassis_joint_measure;   //关节数据
    fp32 joint_T;   //关节扭矩
};

/*底盘轮电机数据*/
class Chassis_Wheel_Motor {
    public:
    const Wheel_Motor_Measure* chassis_wheel_measure;
    fp32 vel;
    fp32 total_pos;
    fp32 tor;
    fp32 wheel_T;
};

/* 腿部控制类 */
class leg_control {
public:
    KalmanFilter_t chassis_vaestimatekf_theta;							// 卡尔曼滤波观测器
    chassis_off_ground_detection_e chassis_off_ground_detection;		// 离地检测枚举

    ramp_function_source_t init_angle_ramp;                             // 用于倒地自启的斜坡函数
    ramp_function_source_t up_angle_ramp;
    PidTypeDef_t roll_control;											// roll轴PID
    fp32 fd_roll;														// roll轴PID输出值
    PidTypeDef_t leg_pid_control;									    // 腿长PID
    fp32 fd_leg;														// 腿长PID输出值
    PidTypeDef_t init_leg_angle_pid;									// 初始化收腿角度PID
    PidTypeDef_t up_leg_angle_pid;										// 上台阶收腿角度PID
    fp32 Fbl_gravity;													// 重力补偿前馈
    fp32 Fbl_inertial;													// 侧向惯性力矩补偿前馈
    fp32 Tbl_jump;                                                      // 跳跃补偿前馈
    fp32 Fbl_spring;
    wbr_leg_t wbr_control;												// wbr连杆解算
    fp32 eta;															// 腿部质心位置系数
    fp32 l_b = L_B;														// 机体到腿部质心的距离
    fp32 i_l = I_L;														// 腿部转动惯量
    fp32 Fwn;															// 驱动轮支持力
    fp32 v_real_n;														// 速度预测
    fp32 chassis_d_yaw_n;												// yaw轴角速度预测
    fp32 d_theta_w_n;													// 驱动轮角速度预测
    fp32 Tw_adapt;														// 驱动轮力矩补偿
    fp32 theta_l;                                                       // 腿角度
    fp32 theta_l_set;                                                   // 腿角度设置
    fp32 d_theta_l;                                                     // 腿角速度
    fp32 d_theta_l_set;                                                 // 腿角速度设置
};

/* 底盘控制类 */
class Chassis_Move
{
    public:

    uint8_t jump_state = 0;
    uint8_t up_state = 0;
    uint8_t up_leg_reached = 0;  // 上台阶腿角度是否到位标志 0:未到位 1:已到位
    uint8_t init_leg_reached = 0;  // 初始化收腿是否到位标志 0:未到位 1:已到位
    bool is_up_mode = false;

    const dr16_control_t *chassis_RC;             //底盘使用的遥控器指针
    const remote_control_t *chassis_RC_0X304;     //底盘使用的图传链路数据指针
    const vt_rc_control_t *vt_rc_control;         //图传键位

    const fp32 *chassis_INS_gyro;				  //机体角速度指针
    const fp32 *chassis_INS_angle;                //获取陀螺仪解算出的欧拉角指针
    const fp32 *chassis_motion_accel_n;			  //绝对坐标系加速度指针

    const Gimbal_Data *chassis_gimbal_data;      //获取云台数据
    uint32_t chassis_relative_angle_set;

    float chassis_yaw_target_base;
    Chassis_Mode_e chassis_mode;                  //底盘控制状态机
    Chassis_Mode_e last_chassis_mode;             //底盘上次控制状态机

    Chassis_Joint_Motor chassis_joint[4]; //底盘关节电机数据
    Chassis_Wheel_Motor chassis_wheel[2]; //底盘轮电机数据

    leg_control chassis_left_control;			  // 左腿控制类
    leg_control chassis_right_control;			  // 右腿控制类

    fp32 v_real;														// 实际机体速度
    fp32 v_obs;                                                         // 机体观测速度
    fp32 v_filter;														// 滤波后的机体速度
    fp32 v_real_last;													// 上一时刻实际速度、
    fp32 v_filter_last;													// 上一时刻滤波速度
    fp32 x_real;														// 期望位移
    fp32 x_filter;														// 滤波后的位移
    fp32 x_real_last;													// 上一时刻期望位移
    fp32 x_filter_last;													// 上一时刻期望滤波位移

    fp32 chassis_yaw;													// 底盘yaw轴角度
    fp32 chassis_pitch;													// 底盘pitch轴角度
    fp32 chassis_roll;													// 底盘roll轴角度

    fp32 chassis_d_yaw;													// 底盘yaw轴角速度
    fp32 chassis_d_pitch;												// 底盘pitch轴角速度
    fp32 chassis_d_roll;												// 底盘roll轴角速度

    fp32 chassis_accel_n_x;												// 绝对坐标下x方向加速度
    fp32 chassis_accel_n_y;												// 绝对坐标下y方向加速度
    fp32 chassis_accel_n_z;												// 绝对坐标下z方向加速度

    fp32 chassis_yaw_p;													// 上一时刻yaw轴角度
    fp32 chassis_d_yaw_p;												// 上一时刻yaw轴角速度

    KalmanFilter_t chassis_vaestimatekf_xv;		  // 速度卡尔曼滤波
    KalmanFilter_t chassis_vaestimatekf_va;		  // 速度卡尔曼滤波

    PidTypeDef_t chassis_angle_pid;               //底盘跟随角度pid
    PidTypeDef_t buffer_pid;                      //缓冲能量pid

    uint32_t chassis_dwt_count;											// 任务时间间隔计数
    fp32 dt;															// 任务间隔时间

    /* 低通滤波器：DT7发送的设定值 */
    first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓机器人速度设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓机器人速度设定值

    fp32 chassis_v_set;													// 目标速度
    fp32 chassis_x_set;													// 目标位移
    fp32 chassis_d_yaw_set;												// 目标角速度
    fp32 chassis_yaw_set;
    fp32 chassis_leg_set;												// 腿部目标长度
    first_order_filter_type_t chassis_leg_filter_set;                                        // 低通滤波后的腿部长度
    first_order_filter_type_t chassis_UP_filter_set;
    fp32 chassis_roll_set;												// 底盘roll轴设定值

    fp32 K = K_APAPT;														// 驱动轮补偿系数

    //机体运动数据卡尔曼滤波计算，二阶卡尔曼对机体V和X滤波
    fp32 vaEstimateKF_F_VX[4] = {1.0f, 0.002f, 0.0f, 1.0f};				    // 配置状态转移矩阵 F（离散时间模型）
    fp32 vaEstimateKF_P_VX[4] = {100.0f, 0.0f, 0.0f, 100.0f};				// 初始协方差 P
    fp32 vaEstimateKF_Q_VX[4] = {0.001f, 0.0f, 0.0f, 0.001f};				// 配置过程噪声协方差 Q
    fp32 vaEstimateKF_R_VX[4] = {0.01f, 0.0f, 0.0f, 1.0f};					// 配置测量噪声协方差 R
    fp32 vaEstimateKF_H_VX[4] = {1.0f, 0.0f, 0.0f, 1.0f};					// 配置测量矩阵 H（直接测量两个状态）

    //腿角度和角速度卡尔曼滤波计算，二阶卡尔曼对腿角度和角速度
    fp32 vaEstimateKF_F_theta[4] = {1.0f, 0.002f, 0.0f, 1.0f};				// 配置状态转移矩阵 F（离散时间模型）
    fp32 vaEstimateKF_P_theta[4] = {100.0f, 0.0f, 0.0f, 100.0f};			// 初始协方差 P
    fp32 vaEstimateKF_Q_theta[4] = {0.001f, 0.0f, 0.0f, 0.001f};			// 配置过程噪声协方差 Q
    fp32 vaEstimateKF_R_theta[4] = {0.01f, 0.0f, 0.0f, 0.5f};				// 配置测量噪声协方差 R
    fp32 vaEstimateKF_H_theta[4] = {1.0f, 0.0f, 0.0f, 1.0f};				// 配置测量矩阵 H（直接测量两个状态）

    //对轮速求出的机体速度和机体加速度滤波，用来防打滑检测
    fp32 vaEstimateKF_F_VA[4] = {1.0f, 0.002f, 0.0f, 1.0f};				// 配置状态转移矩阵 F（离散时间模型）
    fp32 vaEstimateKF_P_VA[4] = {100.0f, 0.0f, 0.0f, 100.0f};			// 初始协方差 P
    fp32 vaEstimateKF_Q_VA[4] = {0.001f, 0.0f, 0.0f, 0.001f};			// 配置过程噪声协方差 Q
    fp32 vaEstimateKF_R_VA[4] = {0.01f, 0.0f, 0.0f, 0.5f};				// 配置测量噪声协方差 R
    fp32 vaEstimateKF_H_VA[4] = {1.0f, 0.0f, 0.0f, 1.0f};				// 配置测量矩阵 H（直接测量两个状态）

    fp32 chassis_spin_change_sen; //底盘旋转速度
};

extern Chassis_Move chassis_move;

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void Chassis_RC_To_Control_Vector(float *vx_set, float *vy_set, Chassis_Move *chassis_move_rc_to_vector);

#endif

#ifdef __cplusplus
extern "C" {
#endif
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void Chassis_Task(void *pvParameters);
#ifdef __cplusplus
}
#endif


#endif
