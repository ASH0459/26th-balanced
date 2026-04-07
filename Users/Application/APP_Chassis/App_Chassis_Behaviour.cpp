/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       chassis_behaviour.c/h
* @brief      according to remote control, change the chassis behaviour.
*             根据遥控器的值，决定底盘行为。
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*  V1.1.0     Nov-11-2019     RM              1. add some annotation
*
@verbatim
==============================================================================
  如果要添加一个新的行为模式
  1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
  erum
  {
      ...
      ...
      CHASSIS_XXX_XXX, // 新添加的
  }chassis_behaviour_e,

  2. 实现一个新的函数 chassis_xxx_xxx_control(float *vx, float *vy, float *wz, Chassis_Move * chassis )
      "vx,vy,wz" 参数是底盘运动控制输入量
      第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
      第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
      第三个参数: 'wz' 可能是角度控制或者旋转速度控制
      在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
  3.  在"Chassis_Behaviour_Mode_set"这个函数中，添加新的逻辑判断，给Chassis_Behaviour_Mode赋值成CHASSIS_XXX_XXX
      在函数最后，添加"else if(Chassis_Behaviour_Mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
      4种:
      CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
      你可以命名成"xxx_angle_set"而不是'wz'
      CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
      你可以命名成"xxx_angle_set"
      CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
      CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
  4.  在"chassis_behaviour_control_set" 函数的最后，添加
      else if(Chassis_Behaviour_Mode == CHASSIS_XXX_XXX)
      {
          chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
      }
==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2019 DJI****************************
*/
#include "App_Chassis_Behaviour.h"
#include "cmsis_os.h"
#include "App_Chassis_Task.h"
#include "arm_math.h"
#include "Alg_UserLib.h"
#include <stdio.h>

// #include "AHRS.h"
#include "App_Detect_Task.h"
#include "arm_math.h"

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_zero_force_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_infantry_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, float *yaw_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_engineer_follow_chassis_yaw_control(float *vx_set, float *vy_set, float *angle_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */
static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_init_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          小陀螺模式 以初始值方向为正轴，在此行为下，云台地盘分开控制 保持云台的正轴方向不变。
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      angle_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_gyroscope(float *vx_set, float *vy_set, float *wz_set, Chassis_Move *chassis_move_rc_to_vector);

/**
 * @brief          小陀螺模式变速部分生成
 * @author         Light
 * @param[in]      random_seed   随机数种子
 * @param[in]      range_min     生成随机数的最小值
 * @param[in]      range_max     生成随机数的最小值
 * @param[in]      range_const   正弦函数的基础值
 * @param[in]      sin_frequency 正弦函数的频率
 * @param[in]      range_const   正弦函数的基础值
 * @note           变速部分的幅值大小由一个（常数+随机数）* sin函数得到
 * @retval         返回空
 */
static float get_changeable_gyrospeed(float amplitude, float sin_frequency);

static void chassis_up_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector);

Chassis_Behaviour_e Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_ZERO_FORCE;

/**
 * @brief          通过逻辑判断，赋值"Chassis_Behaviour_Mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
void Chassis_Behaviour_Mode_Set(Chassis_Move *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    static int8_t last_s = 1;
    static int8_t last_mode_sw = 1;
    static int8_t last_jump = 0;

    static uint8_t last_ctrl = 0;
    // static bool is_up_mode = false; // 用于记录当前的“上台阶”翻转状态

    // 提取 CTRL 键当前状态
    uint8_t curr_ctrl = ((chassis_move_mode->chassis_gimbal_data->key & KEY_PRESSED_OFFSET_CTRL) || (chassis_move_mode->chassis_gimbal_data->jump == 1)) ? 1 : 0;

    // 边缘检测：检测到按下的一瞬间，翻转状态
    if (curr_ctrl && !last_ctrl)
    {
        chassis_move_mode->is_up_mode = !chassis_move_mode->is_up_mode; // 状态翻转：上台阶 <-> 正常跟随
    }
    last_ctrl = curr_ctrl;

    /* 遥控器设置模式部分:针对右侧按键开关 */

    /* 若开关为中档 */
    if (chassis_move_mode->chassis_gimbal_data->chassis_mode == 2)
    {
        /* 上一个状态是无力模式,开关往下，但是这次开关往中间，则切换到起身模式 */
        if (last_s <= 1 && chassis_move_mode->chassis_gimbal_data->chassis_mode == 2)
        {
            Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_INIT;
            chassis_move_mode->is_up_mode = false;

            chassis_move_mode->init_leg_reached = 0;
        }

        // 进入一次起身模式后需要完全起身后才能自动切入行动模式
        if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT && chassis_move_mode->init_leg_reached != 2)
        {
            Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_INIT;
            chassis_move_mode->is_up_mode = false; // 断电时顺便重置状态，防止重新上电猛然上台阶
        }
        else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT && chassis_move_mode->init_leg_reached == 2)
        // && fabs(chassis_move_mode->chassis_left_control.theta_l) < 0.1
        // && fabs(chassis_move_mode->chassis_right_control.theta_l) < 0.1
        // && fabs(chassis_move_mode->chassis_pitch) < 0.1)
        {
            // Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_FOLLOW_CHASSIS_YAW;
            Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW;
        }
        else if (chassis_move_mode->chassis_gimbal_data->chassis_mode == 2 && Chassis_Behaviour_Mode != CHASSIS_BEHAVIOUR_INIT)
        {
            if (chassis_move_mode->is_up_mode)
            {
                Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_UP; // 进入上台阶
            }
            else
            {
                Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW; // 正常跟随
            }
        }
    }
    else if (chassis_move_mode->chassis_gimbal_data->chassis_mode == 0 || chassis_move_mode->chassis_gimbal_data->chassis_mode == 1)
    {
        Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_ZERO_FORCE; // 底盘断电
    }
    /* 若开关为上档,且上一次为中间:上台阶*/
    /* 遥控器模式执行部分:根据行为模式选择一个底盘控制模式 */

    if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO; // 底盘无力模式
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW; // 底盘会跟随云台
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_CHASSIS_YAW; // 底盘有底盘角度控制闭环
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT)
    {
        chassis_move_mode->chassis_mode = CHASSIS_INIT; // 倒地自救
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_GYROSCOPE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_GYROSCOPE; // 小陀螺
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_JUMP)
    {
        chassis_move_mode->chassis_mode = CHASSIS_JUMP; // 跳跃
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_UP)
    {
        chassis_move_mode->chassis_mode = CHASSIS_UP; // 磕上台阶
    }
    last_s = chassis_move_mode->chassis_gimbal_data->chassis_mode;
    last_mode_sw = chassis_move_mode->chassis_gimbal_data->chassis_mode;
    last_jump = chassis_move_mode->chassis_gimbal_data->jump;
    last_ctrl = curr_ctrl;
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param          w_set,  旋转速度设置
 * @param          le  g_set, 腿长设置
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *yaw_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || yaw_set == NULL || d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    // 底盘无力模式
    if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_ZERO_FORCE) // 底盘无力
    {
        chassis_zero_force_control(vx_set, yaw_set, leg_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_FOLLOW_GIMBAL_YAW) // 底盘跟随云台或者跳跃
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, d_yaw_set, leg_set, yaw_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_FOLLOW_CHASSIS_YAW || Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_JUMP) // 跟随底盘yaw，没装云台时候测试
    {
        chassis_no_follow_yaw_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT) // 起立模式(后续修改)
    {
        chassis_init_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_GYROSCOPE) // 小陀螺
    {
        chassis_gyroscope(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_UP)
    {
        chassis_up_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
    }
}

/**
 * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
 * @author         RM
 * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
 * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
 * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_zero_force_control(fp32 *v_set, fp32 *add_w_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (v_set == NULL || add_w_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *v_set = 0.0f;
    *add_w_set = 0.0f;
    *leg_set = 0.35f;
}

/**
 * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_no_move_control(fp32 *v_set, fp32 *add_w_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (v_set == NULL || add_w_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *v_set = 0.0f;
    *add_w_set = 0.0f;
    *leg_set = 0.18f;
}

/**
 * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘与云台控制到的相对角度
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, fp32 *yaw_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    /**
     * PART1. 速度设定值计算
     *   1. 从图传通道取原始值并死区滤波
     *   2. 映射成目标速度
     *   3. 键盘W/S覆盖目标速度为最大/最小
     */
    static int16_t vx_channel_ref = 0;
    static fp32 vx_channel_target = 0;

    uint16_t key_v = chassis_move_rc_to_vector->chassis_gimbal_data->key;

    // 遥控器拨杆原始值死区滤波
    int16_t rc_vx = int16_deadline(chassis_move_rc_to_vector->chassis_gimbal_data->chassis_vx,
                                   -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE);

    // 提取键盘 WASD 向量 (-1.0 到 1.0)
    fp32 key_x = ((key_v & KEY_PRESSED_OFFSET_W) ? 1.0f : 0.0f) - ((key_v & KEY_PRESSED_OFFSET_S) ? 1.0f : 0.0f);
    fp32 key_y = ((key_v & KEY_PRESSED_OFFSET_D) ? 1.0f : 0.0f) - ((key_v & KEY_PRESSED_OFFSET_A) ? 1.0f : 0.0f);

    // 提取 Shift 键状态
    // uint8_t key_shift = (key_v & KEY_PRESSED_OFFSET_SHIFT || chassis_move_rc_to_vector->chassis_gimbal_data->jump == 1) ? 1 : 0;
    uint8_t key_shift = (key_v & KEY_PRESSED_OFFSET_SHIFT) ? 1 : 0;

    fp32 target_angle_offset = 0.0f; // 底盘相对于云台的目标夹角
    fp32 speed_cmd = 0.0f;           // 目标线速度

    // 解算目标夹角和运动方向
    if (key_x != 0.0f || key_y != 0.0f)
    {
        // 1. 如果有键盘按键输入，优先执行键盘的全向逻辑
        fp32 max_speed = CHASSIS_KEY_MAX_SPEED;
        if (key_x >= 0.0f)
        {
            // 包含前进 (W, A, D, WA, WD)
            target_angle_offset = atan2f(key_y, key_x);
            speed_cmd = max_speed;
        }
        else
        {
            // 包含后退 (S, SA, SD) -> 利用倒车实现，车头不掉头
            target_angle_offset = atan2f(-key_y, -key_x);
            speed_cmd = -max_speed;
        }
    }
    else
    {
        // 2. 如果没有键盘输入，自动切回遥控器摇杆逻辑
        target_angle_offset = 0.0f; // 遥控器控制时，默认车头正对云台
        speed_cmd = rc_vx * CHASSIS_VX_RC_SEN;
    }

    // fp32 target_yaw_angle = chassis_move_rc_to_vector->chassis_gimbal_data->init_yaw_angle + target_angle_offset;
    fp32 target_yaw_angle = chassis_move_rc_to_vector->chassis_yaw_target_base + target_angle_offset;
    // 计算当前角度与目标角度的最短误差
    fp32 angle_error = shortest_angle_error(target_yaw_angle, chassis_move_rc_to_vector->chassis_gimbal_data->final_pos);
    fp32 target_vx = 0.0f;
    fp32 target_d_yaw = 0.0f;
    if (key_shift)
    {
        // 1. 调用极度顺滑的变速函数 (确保 get_changeable_gyrospeed 定义在文件上方)
        // 振幅 1.5f，频率步长 0.01f (可根据实车测试手感微调)
        float changeable_gyrospeed = get_changeable_gyrospeed(2.5f, 0.01f);

        target_d_yaw = CHASSIS_SPIN_MAIN_SPEED; // + changeable_gyrospeed;

        target_vx = 0.0f;
        *yaw_set = 0.0f;
    }
    else
    {
        // 松开 Shift：回到正常的“先转后走”底盘跟随模式
        target_d_yaw = PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, -angle_error);
        *yaw_set = angle_error;

        // 速度衰减计算：当车头没转过去时，限制前进速度
        fp32 turn_ratio = cosf(angle_error);
        if (speed_cmd * turn_ratio < 0.0f && speed_cmd > 0.0f)
        {
            turn_ratio = 0.0f;
        }
        target_vx = speed_cmd * turn_ratio;
    }

    // 斜坡函数平滑速度 (替换原有的一阶低通与突变)
    static ramp_function_source_t vx_ramp;
    static uint8_t ramp_init_flag = 0;

    // 初始化斜坡结构体
    if (ramp_init_flag == 0)
    {
        ramp_init(&vx_ramp, 0.002f, CHASSIS_KEY_MAX_SPEED, -CHASSIS_KEY_MAX_SPEED);
        ramp_init_flag = 1;
    }

    // 核心急停与平滑控制逻辑
    if (key_x == 0.0f && key_y == 0.0f && rc_vx == 0 && !key_shift)
    {
        // 1. 无任何键盘和摇杆输入：瞬间清零，直接急停，不需要斜坡缓冲
        vx_ramp.out = 0.0f;
    }
    else
    {
        static fp32 accel_step;
        if (key_x != 0.0f || key_y != 0.0f)
        {
            // 2. 有输入：使用斜坡函数平滑加速或切换方向
            accel_step = CHASSIS_KEY_ACCEL; // 正常加速/减速的斜坡步长
        }
        else
        {
            accel_step = CHASSIS_KEY_ACCEL;
        }

        if (vx_ramp.out < target_vx)
        {
            ramp_calc(&vx_ramp, accel_step);
            if (vx_ramp.out > target_vx)
                vx_ramp.out = target_vx; // 防止超调
        }
        else if (vx_ramp.out > target_vx)
        {
            ramp_calc(&vx_ramp, -accel_step);
            if (vx_ramp.out < target_vx)
                vx_ramp.out = target_vx; // 防止超调
        }
    }
    // 赋值给底盘最终输出
    *vx_set = vx_ramp.out;
    *d_yaw_set = target_d_yaw;

    /**
     * PART4. 腿长 & Yaw 设定 (使用 UserLib 斜坡函数平滑)
     */
    static fp32 leg_target = 0.24f; // 用于记录全局的目标腿长
    static ramp_function_source_t leg_ramp;
    static uint8_t leg_init_flag = 0;

    // 1. 初始化斜坡结构体 (仅运行一次，继承当前实际腿长作为起点)
    if (leg_init_flag == 0)
    {
        // 参数依次为：ramp结构体、伸缩极限范围
        ramp_init(&leg_ramp, 0.002f, 0.35f, 0.18f);
        leg_ramp.out = *leg_set;
        leg_target = *leg_set;
        leg_init_flag = 1;
    }

    // 2. 提取键盘 Q 和 E 的按键状态
    if (key_v & KEY_PRESSED_OFFSET_Q)
    {
        leg_target = 0.3f; // 按下 Q 键，目标高度设为 0.3
    }
    else if (key_v & KEY_PRESSED_OFFSET_E)
    {
        leg_target = 0.18f; // 按下 E 键，目标高度设为最短 0.201
    }
    else
    {
        // 如果没有按键输入，依然保留遥控器拨轮/通道控制腿长的功能
        leg_target += chassis_move_rc_to_vector->chassis_gimbal_data->chassis_leg_set * 0.000001f;
    }

    // 3. 目标值绝对限幅 (保护机械结构)
    if (leg_target > 0.35f)
    {
        leg_target = 0.35f;
    }
    else if (leg_target < 0.18f)
    {
        leg_target = 0.18f;
    }

    // 4. 使用斜坡函数平滑逼近目标值
    // leg_step 决定了腿伸缩的速度。
    // 假设控制周期是 2ms(500Hz)，0.0002 代表每秒伸缩 0.1 米。0.201到0.28只需约0.8秒，非常平缓。
    // 如果觉得动作太慢，可以把 0.0002f 改大（如 0.0005f）；如果觉得太快，可以改小（如 0.0001f）。
    fp32 leg_step = 0.09f;

    if (leg_ramp.out < leg_target)
    {
        ramp_calc(&leg_ramp, leg_step);
        if (leg_ramp.out > leg_target)
            leg_ramp.out = leg_target; // 防止超调抖动
    }
    else if (leg_ramp.out > leg_target)
    {
        ramp_calc(&leg_ramp, -leg_step);
        if (leg_ramp.out < leg_target)
            leg_ramp.out = leg_target; // 防止超调抖动
    }

    // 5. 最终赋值给底层闭环
    *leg_set = leg_ramp.out;
    *yaw_set = 0.0f; // 保持原有 Yaw 的置零逻辑
}

/**
 * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_engineer_follow_chassis_yaw_control(float *vx_set, float *vy_set, float *angle_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    Chassis_RC_To_Control_Vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_gimbal_data->yaw_relative_angle);
}

/**
 * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
 * @author         RM
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{

    if (d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    /**
     * PART1. 速度设定值计算 (与 follow_gimbal_yaw 相同逻辑)
     */
    static int16_t vx_channel_ref = 0;
    static fp32 vx_channel_target = 0;

    // 死区滤波
    vx_channel_ref = int16_deadline(chassis_move_rc_to_vector->chassis_gimbal_data->chassis_vx,
                                    -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE);

    // 映射计算
    vx_channel_target = vx_channel_ref * CHASSIS_VX_RC_SEN;

    // 键盘W/S覆盖
    if (chassis_move_rc_to_vector->chassis_gimbal_data->key & CHASSIS_FRONT_KEY)
    {
        vx_channel_target = CHASSIS_KEY_MAX_SPEED;
    }
    else if (chassis_move_rc_to_vector->chassis_gimbal_data->key & CHASSIS_BACK_KEY)
    {
        vx_channel_target = -CHASSIS_KEY_MAX_SPEED;
    }

    /**
     * PART2. 一阶低通滤波平滑 + 急停处理
     */
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_channel_target);

    if (vx_channel_target < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_channel_target > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;

    /**
     * PART3. yaw 和腿长设定
     */
    static fp32 leg_set_add;
    leg_set_add = chassis_move_rc_to_vector->chassis_gimbal_data->yaw_relative_angle * 0.000001;
    //*d_yaw_set = chassis_move_rc_to_vector->chassis_gimbal_data->d_yaw_set;
    *d_yaw_set = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_leg_set * 0.008; // 目前暂时把leg作为yaw控制
    *leg_set += leg_set_add;

    if (*leg_set >= 0.35)
    {
        *leg_set = 0.35;
    }
    else if (*leg_set <= 0.18)
    {
        *leg_set = 0.18;
    }
}

static void chassis_up_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{

    if (d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    uint16_t key_v = chassis_move_rc_to_vector->chassis_gimbal_data->key;

    // 遥控器拨杆原始值死区滤波
    int16_t rc_vx = int16_deadline(chassis_move_rc_to_vector->chassis_gimbal_data->chassis_vx,
                                   -CHASSIS_RC_DEADLINE, CHASSIS_RC_DEADLINE);

    // 提取键盘 WASD 向量 (-1.0 到 1.0)
    fp32 key_x = ((key_v & KEY_PRESSED_OFFSET_W) ? 1.0f : 0.0f) - ((key_v & KEY_PRESSED_OFFSET_S) ? 1.0f : 0.0f);
    fp32 key_y = ((key_v & KEY_PRESSED_OFFSET_D) ? 1.0f : 0.0f) - ((key_v & KEY_PRESSED_OFFSET_A) ? 1.0f : 0.0f);

    // 提取 Shift 键状态
    // uint8_t key_shift = (key_v & KEY_PRESSED_OFFSET_SHIFT || chassis_move_rc_to_vector->chassis_gimbal_data->jump == 1) ? 1 : 0;
    uint8_t key_shift = (key_v & KEY_PRESSED_OFFSET_SHIFT) ? 1 : 0;

    fp32 target_angle_offset = 0.0f; // 底盘相对于云台的目标夹角
    fp32 speed_cmd = 0.0f;           // 目标线速度

    // 解算目标夹角和运动方向
    if (key_x != 0.0f || key_y != 0.0f)
    {
        // 1. 如果有键盘按键输入，优先执行键盘的全向逻辑
        fp32 max_speed = CHASSIS_KEY_MAX_SPEED_UP;
        if (key_x >= 0.0f)
        {
            // 包含前进 (W, A, D, WA, WD)
            target_angle_offset = atan2f(key_y, key_x);
            speed_cmd = max_speed;
        }
        else
        {
            // 包含后退 (S, SA, SD) -> 利用倒车实现，车头不掉头
            target_angle_offset = atan2f(-key_y, -key_x);
            speed_cmd = -max_speed;
        }
    }
    else
    {
        // 2. 如果没有键盘输入，自动切回遥控器摇杆逻辑
        target_angle_offset = 0.0f; // 遥控器控制时，默认车头正对云台
        speed_cmd = rc_vx * CHASSIS_VX_RC_SEN;
    }

    // fp32 target_yaw_angle = chassis_move_rc_to_vector->chassis_gimbal_data->init_yaw_angle + target_angle_offset;
    fp32 target_yaw_angle = chassis_move_rc_to_vector->chassis_yaw_target_base + target_angle_offset;
    // 计算当前角度与目标角度的最短误差
    fp32 angle_error = shortest_angle_error(target_yaw_angle, chassis_move_rc_to_vector->chassis_gimbal_data->final_pos);
    fp32 target_vx = speed_cmd;
    fp32 target_d_yaw = 0.0f;

    // 斜坡函数平滑速度 (替换原有的一阶低通与突变)
    static ramp_function_source_t vx_ramp;
    static uint8_t ramp_init_flag = 0;

    // 初始化斜坡结构体
    if (ramp_init_flag == 0)
    {
        ramp_init(&vx_ramp, 0.002f, CHASSIS_KEY_MAX_SPEED, -CHASSIS_KEY_MAX_SPEED);
        ramp_init_flag = 1;
    }

    // 核心急停与平滑控制逻辑
    if (key_x == 0.0f && key_y == 0.0f && rc_vx == 0 && !key_shift)
    {
        // 1. 无任何键盘和摇杆输入：瞬间清零，直接急停，不需要斜坡缓冲
        vx_ramp.out = 0.0f;
    }
    else
    {
        static fp32 accel_step;
        if (key_x != 0.0f || key_y != 0.0f)
        {
            // 2. 有输入：使用斜坡函数平滑加速或切换方向
            accel_step = 0.5; // 正常加速/减速的斜坡步长
        }
        else
        {
            accel_step = 0.5f;
        }

        if (vx_ramp.out < target_vx)
        {
            ramp_calc(&vx_ramp, accel_step);
            if (vx_ramp.out > target_vx)
                vx_ramp.out = target_vx; // 防止超调
        }
        else if (vx_ramp.out > target_vx)
        {
            ramp_calc(&vx_ramp, -accel_step);
            if (vx_ramp.out < target_vx)
                vx_ramp.out = target_vx; // 防止超调
        }
    }
    // 赋值给底盘最终输出
    *vx_set = vx_ramp.out;
    *d_yaw_set = target_d_yaw;
    //*vx_set = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_vx * CHASSIS_VX_RC_SEN;
    //*d_yaw_set = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_leg_set * 0.01;//目前暂时把leg作为yaw控制

    if (*leg_set >= 0.35)
    {
        *leg_set = 0.35;
    }
    else if (*leg_set <= 0.18)
    {
        *leg_set = 0.18;
    }
}

/**
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_init_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = 0.0f;
    *d_yaw_set = 0.0f;

    // Phase 1: 触地后，用力收腿
    if (chassis_move_rc_to_vector->init_leg_reached == 1)
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, 0.18f);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;

        // 当腿收缩到 0.21m 以下时，准备切入 LQR 平衡模式
        if (chassis_move_rc_to_vector->chassis_left_control.wbr_control.L <= 0.185f && chassis_move_rc_to_vector->chassis_right_control.wbr_control.L <= 0.185f)
        {
            chassis_move_rc_to_vector->init_leg_reached = 2;
        }
    }
    // Phase 2: LQR 已经接管，开始平滑起立
    else if (chassis_move_rc_to_vector->init_leg_reached == 2)
    {
        // 【新增：恢复站立高度】把目标腿长滤滑到正常高度（例如 0.25f 或更高）
        // 这样机器人在切入平衡后，会从蹲姿慢慢站起来，极大地增加抗扰动能力
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, 0.18f);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
}

/**
 * @brief          小陀螺模式
 * @author         LWH
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
 * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         返回空
 */

static void chassis_gyroscope(fp32 *v_set, fp32 *d_yaw_set, fp32 *leg_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (v_set == NULL || d_yaw_set == NULL || leg_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    /* 变速小陀螺由以下几部分角速度叠加而成
     * 1.fixed_gyrospeed：固定角速度
     * 2.changeable_gyrospeed：变化的角速度
     * 3.xy_deleted_gyrospeed：平动削减分量
     * */

    const float fixed_gyrospeed = CHASSIS_SPIN_MAIN_SPEED;

    *v_set = 0;
    *leg_set = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_leg_set;
    *d_yaw_set = fixed_gyrospeed; //+ changeable_gyrospeed;
}

/**
/**
  * @brief          正弦变速小陀螺速度生成
  * @param[in]      amplitude     正弦波的振幅（决定了忽快忽慢的幅度大小）
  * @param[in]      sin_frequency 正弦波的频率（决定了多长时间完成一次快慢循环）
  * @retval         当前周期的速度增量
  */
static float get_changeable_gyrospeed(float amplitude, float sin_frequency)
{
    static float gyroscope_phase = 0.0f; // 正弦相位

    // 计算当前的正弦速度分量
    float Speed_Changeable = amplitude * arm_sin_f32(gyroscope_phase);

    // 相位自增
    gyroscope_phase += sin_frequency;

    // 【核心保护】：相位防溢出。防止机器人开机时间过长导致 float 精度丢失引发死机
    if (gyroscope_phase > 2.0f * PI)
    {
        gyroscope_phase -= 2.0f * PI;
    }

    return Speed_Changeable;
}