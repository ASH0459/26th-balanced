
#ifndef KALMAN_FILTER_H  // 【第1步】防止重定义
#define KALMAN_FILTER_H

#include "main.h"
#include "arm_math.h"
#include "stdint.h"
#include "stdlib.h"
#ifdef __cplusplus
extern "C" {
#endif

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
    /* ================= 外部接口指针 ================= */
    fp32 *FilteredValue;        // 输出：滤波后的值（指向 xhat 的数据区域或用户指定的变量）
    fp32 *MeasuredVector;       // 输入：测量向量（传感器原始数据）
    fp32 *ControlVector;        // 输入：控制向量（如电机的电流、扭矩等控制量）

    /* ================= 维度定义 ================= */
    uint8_t xhatSize;           // 状态变量的维度 (n)，例如位置+速度为2维
    uint8_t uSize;              // 控制变量的维度 (l)
    uint8_t zSize;              // 观测（测量）变量的维度 (m)

    /* ================= 功能配置开关 ================= */
    uint8_t UseAutoAdjustment;    // 启用自适应调整（可能用于自动调整 R 或 Q 矩阵）
    uint8_t MeasurementValidNum;  // 当前周期有效的测量数据个数（用于传感器部分丢失的情况）

    /* ================= 自适应/高级参数 ================= */
    uint8_t *MeasurementMap;      // 测量映射表（用于标记哪些测量量对应哪些状态，或处理传感器冗余）
    fp32 *MeasurementDegree;      // 测量置信度或相关系数
    fp32 *MatR_DiagonalElements;  // 测量噪声协方差矩阵 R 的对角线元素指针（用于动态调整 R）
    fp32 *StateMinVariance;       // 状态最小方差限制（防止 P 矩阵过度收敛导致滤波发散）
    uint8_t *temp;                // 临时缓存区（通常用于矩阵运算库内部，如求逆时的排列）

    /* ================= 优化跳过标志 ================= */
    /* 标准卡尔曼滤波有5个核心公式，这些标志位用于跳过某些步骤
       例如：没有测量更新时，跳过公式 3,4,5 */
    uint8_t SkipEq1; // 跳过状态预测 ( x_k|k-1 = F * x_k-1|k-1 + B * u )
    uint8_t SkipEq2; // 跳过协方差预测 ( P_k|k-1 = F * P_k-1|k-1 * F^T + Q )
    uint8_t SkipEq3; // 跳过卡尔曼增益计算 ( K = P * H^T * inv(H * P * H^T + R) )
    uint8_t SkipEq4; // 跳过状态更新 ( x_k|k = x_k|k-1 + K * (z - H * x) )
    uint8_t SkipEq5; // 跳过协方差更新 ( P_k|k = (I - K * H) * P_k|k-1 )

    /* ================= 矩阵实体 (Matrix 结构体) ================= */
    mat xhat;        // 后验状态估计向量 (x_k, 维度 n*1) —— 最终结果
    mat xhatminus;   // 先验状态估计向量 (x_k-, 维度 n*1) —— 预测值
    mat u;           // 控制向量 (u, 维度 l*1)
    mat z;           // 测量向量 (z, 维度 m*1)
    mat P;           // 后验误差协方差矩阵 (P_k, 维度 n*n) —— 估计精度
    mat Pminus;      // 先验误差协方差矩阵 (P_k-, 维度 n*n)
    mat F, FT;       // 状态转移矩阵 (F) 及其转置 (F^T) —— 描述系统如何随时间演变
    mat B;           // 控制输入矩阵 (B) —— 描述控制量如何影响状态
    mat H, HT;       // 观测矩阵 (H) 及其转置 (H^T) —— 描述状态如何映射到测量值
    mat Q;           // 过程噪声协方差矩阵 (Q) —— 系统模型的不确定性
    mat R;           // 测量噪声协方差矩阵 (R) —— 传感器的噪声水平
    mat K;           // 卡尔曼增益矩阵 (K) —— 相信测量值还是相信预测值的权重系数
    mat S;           // 创新(Innovation)协方差矩阵 (S = H*P-*HT + R)
    mat temp_matrix; // 矩阵运算过程中的临时矩阵1
    mat temp_matrix1;// 矩阵运算过程中的临时矩阵2
    mat temp_vector; // 矩阵运算过程中的临时向量1
    mat temp_vector1;// 矩阵运算过程中的临时向量2

    int8_t MatStatus; // 矩阵运算状态（例如矩阵求逆是否成功，0表示成功）

    /* ================= 回调函数指针 (用于扩展卡尔曼滤波 EKF) ================= */
    /* 这里的函数指针通常用于在滤波的不同阶段插入用户代码，
       主要用于非线性系统的线性化（更新雅可比矩阵 F 和 H） */
    void (*User_Func0_f)(struct kf_t *kf); // 初始化后/步骤1之前的回调
    void (*User_Func1_f)(struct kf_t *kf); // 预测步骤 (Predict) 之前的回调 —— 更新 F 矩阵
    void (*User_Func2_f)(struct kf_t *kf); // 预测步骤之后的回调
    void (*User_Func3_f)(struct kf_t *kf); // 测量更新 (Update) 之前的回调 —— 更新 H 矩阵
    void (*User_Func4_f)(struct kf_t *kf); // 测量更新之后的回调
    void (*User_Func5_f)(struct kf_t *kf); // 一般保留
    void (*User_Func6_f)(struct kf_t *kf); // 一般保留

    /* ================= 矩阵数据存储区 (Data Buffers) ================= */
    /* `mat` 结构体通常只包含行、列和指向数据的指针。
       以下指针指向实际分配的内存区域，存放浮点数数据。 */
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

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_H