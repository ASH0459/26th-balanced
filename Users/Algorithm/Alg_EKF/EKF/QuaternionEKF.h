#ifndef QUAT_EKF_H
#define QUAT_EKF_H

#include "main.h"
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    fp32 GyroBias[3]; // 陀螺仪零偏估计值

    fp32 OrientationCosine[3];

    fp32 accLPFcoef;
    fp32 gyro_norm;
    fp32 accl_norm;
    fp32 AdaptiveGainScale;

    fp32 Q1; // 四元数更新过程噪声
    fp32 Q2; // 陀螺仪零偏过程噪声
    fp32 R;  // 加速度计量测噪声

    fp32 dt; // 姿态更新周期
    mat ChiSquare;
    fp32 ChiSquare_Data[1];      // 卡方检验检测函数
    fp32 ChiSquareTestThreshold; // 卡方检验阈值
    fp32 lambda;                 // 渐消因子

    int16_t YawRoundCount;
    fp32 last_yaw;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern fp32 chiSquare;
extern fp32 ChiSquareTestThreshold;
extern void IMU_QuaternionEKF_Init(fp32 process_noise1, fp32 process_noise2, fp32 measure_noise, fp32 lambda, fp32 lpf);
extern void IMU_QuaternionEKF_Update(fp32 *INS_quat, fp32 *INS_gyro, fp32 *INS_accel, fp32 *INS_angle, fp32 dt);

#endif
