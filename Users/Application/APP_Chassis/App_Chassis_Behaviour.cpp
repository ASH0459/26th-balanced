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
 * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
 * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
 * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
 * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
 * @param[in]      chassis_move_rc_to_vector底盘数据
 * @retval         none
 */

static void chassis_init_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move *chassis_move_rc_to_vector);

Chassis_Behaviour_e Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_ZERO_FORCE;

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

static fp32 clamp_leg_length(fp32 leg_set)
{
    if (leg_set > 0.35f)
    {
        return 0.35f;
    }
    if (leg_set < 0.17f)
    {
        return 0.17f;
    }
    return leg_set;
}

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

    static chassis_mode_e last_requested_mode = CHASSIS_MODE_RESERVED;
    const chassis_mode_e requested_mode = chassis_move_mode->chassis_gimbal_data->chassis_behaviour_mode;

    if (requested_mode == CHASSIS_MODE_NORMAL)
    {
        if ((last_requested_mode == CHASSIS_MODE_NO_FORCE || last_requested_mode == CHASSIS_MODE_RESERVED) &&
            requested_mode == CHASSIS_MODE_NORMAL)
        {
            Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_INIT;
            chassis_move_mode->init_leg_reach_state = INIT_LEG_UNREACH;
        }

        if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT &&
            chassis_move_mode->init_leg_reach_state != INIT_LEG_STANDUP)
        {
            Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_INIT;
        }
        else
        {
            Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW;
        }
    }
    else
    {
        Chassis_Behaviour_Mode = CHASSIS_BEHAVIOUR_ZERO_FORCE;
    }

    if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO;
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT)
    {
        chassis_move_mode->chassis_mode = CHASSIS_INIT;
    }
    last_requested_mode = requested_mode;
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
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, d_yaw_set, leg_set, yaw_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_BEHAVIOUR_INIT) // 起立模式(后续修改)
    {
        chassis_init_control(vx_set, d_yaw_set, leg_set, chassis_move_rc_to_vector);
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

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *d_yaw_set, fp32 *leg_set, fp32 *yaw_set, Chassis_Move *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || d_yaw_set == NULL || leg_set == NULL || yaw_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    const fp32 target_vx = clamp_abs_fp32(chassis_move_rc_to_vector->chassis_gimbal_data->v_tmp, CHASSIS_KEY_MAX_SPEED);
    const fp32 target_yaw_angle = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_yaw_set;
    const fp32 current_yaw_angle = chassis_move_rc_to_vector->chassis_gimbal_data->chassis_relative_angle;
    const fp32 angle_error = shortest_angle_error(target_yaw_angle, current_yaw_angle);
    const fp32 target_d_yaw = PID_Calc(&chassis_move_rc_to_vector->chassis_angle_pid, 0.0f, -angle_error);

    // 斜坡函数平滑速度 (替换原有的一阶低通与突变)
    static ramp_function_source_t vx_ramp;
    static uint8_t ramp_init_flag = 0;

    // 初始化斜坡结构体
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
    *d_yaw_set = target_d_yaw;
    *leg_set = clamp_leg_length(*leg_set > 0.0f ? *leg_set : 0.17f);
    *yaw_set = angle_error;
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
    if (chassis_move_rc_to_vector->init_leg_reach_state == INIT_LEG_REACH)
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, 0.18f);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;

        // 当腿收缩到 0.21m 以下时，准备切入 LQR 平衡模式
        if (chassis_move_rc_to_vector->chassis_left_control.wbr_control.L <= 0.185f && chassis_move_rc_to_vector->chassis_right_control.wbr_control.L <= 0.185f)
        {
            chassis_move_rc_to_vector->init_leg_reach_state = INIT_LEG_STANDUP;
        }
    }
    // Phase 2: LQR 已经接管，开始平滑起立
    else if (chassis_move_rc_to_vector->init_leg_reach_state == INIT_LEG_STANDUP)
    {
        // 【新增：恢复站立高度】把目标腿长滤滑到正常高度（例如 0.25f 或更高）
        // 这样机器人在切入平衡后，会从蹲姿慢慢站起来，极大地增加抗扰动能力
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_leg_filter_set, 0.18f);
        *leg_set = chassis_move_rc_to_vector->chassis_leg_filter_set.out;
    }
}

