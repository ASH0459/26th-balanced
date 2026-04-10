/**
  ****************************(C) COPYRIGHT 2026 ROBOT-Z****************************
  * @file       INS_task.c/h
  * @brief      2026赛季舵轮步兵欧拉角计算任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dev-20-2024     冉文治           1. 添加卡尔曼滤波对零漂进行处理
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 ROBOT-Z****************************
  */

//#include "task.h"
#include "INS_task.h"
#include "QuaternionEKF.h"
#include "Alg_PID.h"
#include "BMI088driver.h"
#include "bsp_imu_pwm.h"
#include "bsp_dwt.h"
#include "kalman_filter.h"
#include "App_Chassis_Task.h"

// PWM给定
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)

const fp32 xb[3] = {1, 0, 0};
const fp32 yb[3] = {0, 1, 0};
const fp32 zb[3] = {0, 0, 1};

/**
  * @brief          将机体坐标系中的向量转换到地球坐标系中
  * @param          vecBF   输入参数：机体坐标系中的向量（3D向量）
  * @param          vecEF   输出参数：转换后的地球坐标系中的向量（3D向量）
  * @param          q      输入参数：四元数，用于描述机体坐标系到地球坐标系的旋转
  * @retval         none
  */
void BodyFrameToEarthFrame(const fp32 *vecBF, fp32 *vecEF, fp32 *q);

/**
  * @brief          将地球坐标系中的向量转换到机体坐标系中
  * @param          vecEF   输入参数：地球坐标系中的向量（3D向量）
  * @param          vecBF   输出参数：转换后的机体坐标系中的向量（3D向量）
  * @param          q      输入参数：四元数，用于描述机体坐标系到地球坐标系的旋转
  * @retval         none
  */
void EarthFrameToBodyFrame(const fp32 *vecEF, fp32 *vecBF, fp32 *q);

/**
  * @brief          控制bmi088的温度
  * @author         冉文治
  * @param          temp bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

KalmanFilter_t KalmanFilter;

static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PidTypeDef_t imu_temp_pid;

uint32_t INS_DWT_Count = 0;
static fp32 dt = 0;
static uint8_t first_temperate;

// 角速度
fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
// 加速度
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
// 四元数
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// 欧拉角，单位rad
fp32 INS_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// 机体坐标加速度
fp32 Motion_accel_b[3] = {0.0f, 0.0f, 0.0f};
// 绝对系加速度
fp32 Motion_accel_n[3] = {0.0f, 0.0f, 0.0f};
// 加速度在绝对系的向量表示
fp32 Xn[3] = {0.0f, 0.0f, 0.0f};
fp32 Yn[3] = {0.0f, 0.0f, 0.0f};
fp32 Zn[3] = {0.0f, 0.0f, 0.0f};

/**
  * @brief          imu任务, 初始化 bmi088, 计算欧拉角
  * @author         冉文治
  * @param          pvParameters: NULL
  * @retval         none
  */
void INS_task(void *pvParameters)
{
	// 四元数EKF初始化
	IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.3);
	// PID初始化
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	const fp32 gravity[3] = {0, 0, 9.81f};

    while(1)
    {
    	// 设置时间间隔
		dt = DWT_GetDeltaT(&INS_DWT_Count);
    	// 计算欧拉角
		IMU_QuaternionEKF_Update(INS_quat, INS_gyro, INS_accel, INS_angle, dt);
        // 机体系基向量转换到导航坐标系，选取惯性系为导航系
        BodyFrameToEarthFrame(xb, Xn, INS_quat);
        BodyFrameToEarthFrame(yb, Yn, INS_quat);
        BodyFrameToEarthFrame(zb, Zn, INS_quat);
        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        fp32 gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS_quat);
        // 同样过一个低通滤波
        for (uint8_t i = 0; i < 3; i++)
        {
            Motion_accel_b[i] = (INS_accel[i] - gravity_b[i]) * dt / (ACCELLPF + dt) + Motion_accel_b[i] * ACCELLPF / (ACCELLPF + dt);
        }
        // 转换回导航系n
        BodyFrameToEarthFrame(Motion_accel_b, Motion_accel_n, INS_quat);
    	// 控制温度
		imu_temp_control(BMI088.temperature);
    	// 任务延时
		vTaskDelay(IMU_CONTROL_TIME_MS);
    }
}

/**
  * @brief          将机体坐标系中的向量转换到地球坐标系中
  * @param          vecBF   输入参数：机体坐标系中的向量（3D向量）
  * @param          vecEF   输出参数：转换后的地球坐标系中的向量（3D向量）
  * @param          q      输入参数：四元数，用于描述机体坐标系到地球坐标系的旋转
  * @retval         none
  */
void BodyFrameToEarthFrame(const fp32 *vecBF, fp32 *vecEF, fp32 *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
  * @brief          将地球坐标系中的向量转换到机体坐标系中
  * @param          vecEF   输入参数：地球坐标系中的向量（3D向量）
  * @param          vecBF   输出参数：转换后的机体坐标系中的向量（3D向量）
  * @param          q      输入参数：四元数，用于描述机体坐标系到地球坐标系的旋转
  * @retval         none
  */
void EarthFrameToBodyFrame(const fp32 *vecEF, fp32 *vecBF, fp32 *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
  * @brief          控制bmi088的温度
  * @author         冉文治
  * @param          temp bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_Calc(&imu_temp_pid, temp, 40.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        if (temp > 40.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.Iout = BMI088_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(BMI088_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          获取四元数
  * @author         冉文治
  * @param          none
  * @retval         INS_quat的指针
  */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴, 3:total_yaw 单位 rad/s
  * @author         冉文治
  * @param          none
  * @retval         INS_gyro的指针
  */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         INS_accel的指针
  */
const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         INS_accel的指针
  */
const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}

/**
  * @brief          获取机体坐标加速度,0:x轴, 1:y轴, 2:z轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         Motion_accel_b的指针
  */
const fp32 *get_montion_accel_b_data_point(void)
{
    return Motion_accel_b;
}

/**
  * @brief          获取绝对系加速度,0:x轴, 1:y轴, 2:z轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         Motion_accel_n的指针
  */
const fp32 *get_montion_accel_n_data_point(void)
{
    return Motion_accel_n;
}

/**
  * @brief          获取加速度在绝对系的向量
  * @author         冉文治
  * @param          none
  * @retval         Xn的指针
  */
const fp32 *get_xn_data_point(void)
{
    return Xn;
}
const fp32 *get_yn_data_point(void)
{
    return Yn;
}
const fp32 *get_zn_data_point(void)
{
    return Zn;
}
