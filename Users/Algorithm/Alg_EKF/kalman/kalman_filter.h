#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "main.h"
#include "arm_math.h"
#include "stdint.h"
#include "stdlib.h"
#include "CMSIS_OS.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct kf_t
{
    fp32 *FilteredValue;
    fp32 *MeasuredVector;
    fp32 *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment;
    uint8_t MeasurementValidNum;

    uint8_t *MeasurementMap;
    fp32 *MeasurementDegree;
    fp32 *MatR_DiagonalElements;
    fp32 *StateMinVariance;
    uint8_t *temp;

    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    mat xhat;
    mat xhatminus;
    mat u;
    mat z;
    mat P;
    mat Pminus;
    mat F, FT;
    mat B;
    mat H, HT;
    mat Q;
    mat R;
    mat K;
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus;

    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    fp32 *xhat_data, *xhatminus_data;
    fp32 *u_data;
    fp32 *z_data;
    fp32 *P_data, *Pminus_data;
    fp32 *F_data, *FT_data;
    fp32 *B_data;
    fp32 *H_data, *HT_data;
    fp32 *Q_data;
    fp32 *R_data;
    fp32 *K_data;
    fp32 *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;

/**
  * @brief          初始化卡尔曼滤波器（Kalman Filter）的各项参数和矩阵
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @param          xhatSize     状态向量的维度 (xhat的大小)
  * @param          uSize        控制向量的维度 (u的大小)
  * @param          zSize        测量向量的维度 (z的大小)
  * @retval         无返回值。初始化后的卡尔曼滤波器将保存在 `kf` 结构体中。
  *
  * @note           此函数用于初始化卡尔曼滤波器的所有参数，包括状态向量、测量向量、协方差矩阵等。初始化过程中，
  *                 会为卡尔曼滤波器分配内存，并设置相关的参数值（如状态维度、控制维度、测量维度等）。此外，还会
  *                 对所有的矩阵和向量进行零初始化。
  *
  * @note           `sizeof_float` 和 `sizeof_double` 变量分别用于保存 `fp32` 和 `fp64` 类型的大小。
  */
extern void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);

/**
  * @brief          处理卡尔曼滤波器的测量数据，更新测量向量
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的测量向量（z）和控制向量（u）。
  *
  * @note           如果 `UseAutoAdjustment` 为 1，调用 `H_K_R_Adjustment` 进行自动调整。否则，直接从 `MeasuredVector`
  *                 拷贝数据到 `z_data`。同时，将 `ControlVector` 拷贝到 `u_data`。
  */
extern void Kalman_Filter_Measure(KalmanFilter_t *kf);

/**
  * @brief          更新卡尔曼滤波器的预测状态向量 `xhatminus`
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的预测状态 `xhatminus`。
  *
  * @note           根据状态转移矩阵 `F` 和控制矩阵 `B`，更新预测状态向量 `xhatminus`。如果存在控制向量，则需要
  *                 额外乘上控制向量 `u`。
  */
extern void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);

/**
  * @brief          更新卡尔曼滤波器的预测协方差矩阵 `Pminus`
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的预测协方差矩阵 `Pminus`。
  *
  * @note           根据状态转移矩阵 `F` 和当前协方差矩阵 `P`，计算预测协方差矩阵 `Pminus`。并且加上过程噪声
  *                 协方差矩阵 `Q`，得到最终的预测协方差矩阵。
  */
extern void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);

/**
  * @brief          计算卡尔曼增益矩阵 `K`
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的增益矩阵 `K`。
  *
  * @note           根据卡尔曼滤波的标准更新公式，计算卡尔曼增益矩阵 `K`。首先通过状态转移矩阵 `H`、预测协方差
  *                 矩阵 `Pminus`、测量噪声协方差矩阵 `R`，以及中间结果矩阵，最终计算得到增益矩阵 `K`。计算过程
  *                 涉及矩阵乘法、转置、加法和求逆等操作。
  */
extern void Kalman_Filter_SetK(KalmanFilter_t *kf);

/**
  * @brief          更新卡尔曼滤波器的状态估计 `xhat(k)`
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的状态估计 `xhat(k)`。
  *
  * @note           该函数使用卡尔曼增益 `K`、测量矩阵 `H` 和测量值 `z(k)` 来更新状态估计 `xhat(k)`。
  *                 首先计算残差项 `z(k) - H·xhat'(k)`，然后通过增益矩阵 `K` 对状态估计进行修正。
  */
extern void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);

/**
  * @brief          更新卡尔曼滤波器的协方差矩阵 `P(k)`
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的协方差矩阵 `P(k)`。
  *
  * @note           该函数根据卡尔曼增益矩阵 `K(k)` 和测量矩阵 `H` 更新协方差矩阵 `P(k)`。
  *                 通过计算 `K(k)·H·P'(k)`，然后从预测协方差矩阵 `P'(k)` 中减去这个量，更新协方差矩阵。
  */
extern void Kalman_Filter_P_Update(KalmanFilter_t *kf);

/**
  * @brief          更新卡尔曼滤波器的状态估计、协方差矩阵以及相关输出。
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         返回过滤后的状态估计值 `FilteredValue`，类型为 `fp32*`。
  *
  * @note           该函数综合执行卡尔曼滤波器的主要更新步骤，包括：
  *                 1. 处理测量数据。
  *                 2. 根据预测值更新状态 `xhat(k)` 和协方差矩阵 `P(k)`。
  *                 3. 根据外部用户定义的回调函数，允许用户在特定步骤插入自定义操作。
  *                 4. 对状态协方差矩阵进行限制，确保不小于最小方差。
  *                 5. 返回更新后的状态估计 `FilteredValue`。
  */
extern fp32 *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif
