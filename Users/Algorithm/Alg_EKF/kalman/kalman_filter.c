/**
  ****************************(C) COPYRIGHT 2025 ROBOT_Z****************************
  * @file       kalman_filter.c/h
  * @brief      卡尔曼滤波器运算库
  * @note       包括卡尔曼滤波器的初始化，数据更新，运算等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0                     王洪玺           1.完成
  *  V2.0.0     Dec-24-2024     冉文治           1.改成H7
  *                                             2.适用于大疆格式代码
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 ROBOT_Z****************************
  */

#include "kalman_filter.h"
#include "arm_math.h"

uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
  * @brief          初始化卡尔曼滤波器（Kalman Filter）的各项参数和矩阵
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
void Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    // 获取 fp32 和 fp64 类型的大小
    sizeof_float = sizeof(fp32);
    sizeof_double = sizeof(fp64);

    // 设置滤波器的状态、控制和测量向量的维度
    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    // 初始化与测量相关的参数
    kf->MeasurementValidNum = 0;
    kf->MeasurementMap = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->MeasurementMap, 0, sizeof(uint8_t) * zSize);
    kf->MeasurementDegree = (fp32 *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasurementDegree, 0, sizeof_float * zSize);
    kf->MatR_DiagonalElements = (fp32 *)user_malloc(sizeof_float * zSize);
    memset(kf->MatR_DiagonalElements, 0, sizeof_float * zSize);
    kf->StateMinVariance = (fp32 *)user_malloc(sizeof_float * xhatSize);
    memset(kf->StateMinVariance, 0, sizeof_float * xhatSize);
    kf->temp = (uint8_t *)user_malloc(sizeof(uint8_t) * zSize);
    memset(kf->temp, 0, sizeof(uint8_t) * zSize);

    // 初始化滤波器数据（xhat, z, u, P 等）
    kf->FilteredValue = (fp32 *)user_malloc(sizeof_float * xhatSize);
    memset(kf->FilteredValue, 0, sizeof_float * xhatSize);
    kf->MeasuredVector = (fp32 *)user_malloc(sizeof_float * zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * zSize);
    kf->ControlVector = (fp32 *)user_malloc(sizeof_float * uSize);
    memset(kf->ControlVector, 0, sizeof_float * uSize);

    // 初始化状态向量 xhat (x(k|k))
    kf->xhat_data = (fp32 *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhat_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhat, kf->xhatSize, 1, (fp32 *)kf->xhat_data);

    // 初始化预测状态向量 xhatminus (x(k|k-1))
    kf->xhatminus_data = (fp32 *)user_malloc(sizeof_float * xhatSize);
    memset(kf->xhatminus_data, 0, sizeof_float * xhatSize);
    Matrix_Init(&kf->xhatminus, kf->xhatSize, 1, (fp32 *)kf->xhatminus_data);

    // 初始化控制向量 u (如果有的话)
    if (uSize != 0)
    {
        kf->u_data = (fp32 *)user_malloc(sizeof_float * uSize);
        memset(kf->u_data, 0, sizeof_float * uSize);
        Matrix_Init(&kf->u, kf->uSize, 1, (fp32 *)kf->u_data);
    }

    // 初始化测量向量 z
    kf->z_data = (fp32 *)user_malloc(sizeof_float * zSize);
    memset(kf->z_data, 0, sizeof_float * zSize);
    Matrix_Init(&kf->z, kf->zSize, 1, (fp32 *)kf->z_data);

    // 初始化协方差矩阵 P(k|k)
    kf->P_data = (fp32 *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->P_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->P, kf->xhatSize, kf->xhatSize, (fp32 *)kf->P_data);

    // 初始化预测协方差矩阵 P(k|k-1)
    kf->Pminus_data = (fp32 *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Pminus_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Pminus, kf->xhatSize, kf->xhatSize, (fp32 *)kf->Pminus_data);

    // 初始化状态转移矩阵 F 和其转置矩阵 FT
    kf->F_data = (fp32 *)user_malloc(sizeof_float * xhatSize * xhatSize);
    kf->FT_data = (fp32 *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->F_data, 0, sizeof_float * xhatSize * xhatSize);
    memset(kf->FT_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->F, kf->xhatSize, kf->xhatSize, (fp32 *)kf->F_data);
    Matrix_Init(&kf->FT, kf->xhatSize, kf->xhatSize, (fp32 *)kf->FT_data);

    // 初始化控制矩阵 B（如果有的话）
    if (uSize != 0)
    {
        // control matrix B
        kf->B_data = (fp32 *)user_malloc(sizeof_float * xhatSize * uSize);
        memset(kf->B_data, 0, sizeof_float * xhatSize * uSize);
        Matrix_Init(&kf->B, kf->xhatSize, kf->uSize, (fp32 *)kf->B_data);
    }

    // 初始化测量矩阵 H 和其转置矩阵 HT
    kf->H_data = (fp32 *)user_malloc(sizeof_float * zSize * xhatSize);
    kf->HT_data = (fp32 *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->H_data, 0, sizeof_float * zSize * xhatSize);
    memset(kf->HT_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->H, kf->zSize, kf->xhatSize, (fp32 *)kf->H_data);
    Matrix_Init(&kf->HT, kf->xhatSize, kf->zSize, (fp32 *)kf->HT_data);

    // 初始化过程噪声协方差矩阵 Q
    kf->Q_data = (fp32 *)user_malloc(sizeof_float * xhatSize * xhatSize);
    memset(kf->Q_data, 0, sizeof_float * xhatSize * xhatSize);
    Matrix_Init(&kf->Q, kf->xhatSize, kf->xhatSize, (fp32 *)kf->Q_data);

    // 初始化测量噪声协方差矩阵 R
    kf->R_data = (fp32 *)user_malloc(sizeof_float * zSize * zSize);
    memset(kf->R_data, 0, sizeof_float * zSize * zSize);
    Matrix_Init(&kf->R, kf->zSize, kf->zSize, (fp32 *)kf->R_data);

    // 初始化卡尔曼增益 K
    kf->K_data = (fp32 *)user_malloc(sizeof_float * xhatSize * zSize);
    memset(kf->K_data, 0, sizeof_float * xhatSize * zSize);
    Matrix_Init(&kf->K, kf->xhatSize, kf->zSize, (fp32 *)kf->K_data);

    // 初始化其他临时矩阵
    kf->S_data = (fp32 *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data = (fp32 *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_matrix_data1 = (fp32 *)user_malloc(sizeof_float * kf->xhatSize * kf->xhatSize);
    kf->temp_vector_data = (fp32 *)user_malloc(sizeof_float * kf->xhatSize);
    kf->temp_vector_data1 = (fp32 *)user_malloc(sizeof_float * kf->xhatSize);
    Matrix_Init(&kf->S, kf->xhatSize, kf->xhatSize, (fp32 *)kf->S_data);
    Matrix_Init(&kf->temp_matrix, kf->xhatSize, kf->xhatSize, (fp32 *)kf->temp_matrix_data);
    Matrix_Init(&kf->temp_matrix1, kf->xhatSize, kf->xhatSize, (fp32 *)kf->temp_matrix_data1);
    Matrix_Init(&kf->temp_vector, kf->xhatSize, 1, (fp32 *)kf->temp_vector_data);
    Matrix_Init(&kf->temp_vector1, kf->xhatSize, 1, (fp32 *)kf->temp_vector_data1);

    // 标记是否跳过方程的计算
    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;
}

/**
  * @brief          处理卡尔曼滤波器的测量数据，更新测量向量
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的测量向量（z）和控制向量（u）。
  *
  * @note           如果 `UseAutoAdjustment` 为 1，调用 `H_K_R_Adjustment` 进行自动调整。否则，直接从 `MeasuredVector`
  *                 拷贝数据到 `z_data`。同时，将 `ControlVector` 拷贝到 `u_data`。
  */
void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 判断是否使用自动调整
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);  // 调用自定义调整函数
    else
    {
        // 否则，直接复制测量向量到 z_data，并清空原始测量向量
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    // 将控制向量复制到 u_data
    memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

/**
  * @brief          更新卡尔曼滤波器的预测状态向量 `xhatminus`
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的预测状态 `xhatminus`。
  *
  * @note           根据状态转移矩阵 `F` 和控制矩阵 `B`，更新预测状态向量 `xhatminus`。如果存在控制向量，则需要
  *                 额外乘上控制向量 `u`。
  */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    // 如果不跳过方程1
    if (!kf->SkipEq1)
    {
        // 如果有控制向量
        if (kf->uSize > 0)
        {
            // 计算 F * xhat (状态转移乘以当前状态)
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);

            // 计算 B * u (控制矩阵乘以控制向量)
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);

            // 将两者相加得到预测状态 xhatminus
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            // 如果没有控制向量，仅计算 F * xhat
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

/**
  * @brief          更新卡尔曼滤波器的预测协方差矩阵 `Pminus`
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的预测协方差矩阵 `Pminus`。
  *
  * @note           根据状态转移矩阵 `F` 和当前协方差矩阵 `P`，计算预测协方差矩阵 `Pminus`。并且加上过程噪声
  *                 协方差矩阵 `Q`，得到最终的预测协方差矩阵。
  */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    // 如果不跳过方程2
    if (!kf->SkipEq2)
    {
        // 计算 F * P * F^T (状态转移乘以当前协方差矩阵再乘以状态转移矩阵的转置)
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);

        // 临时存储 F * P * F^T
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F * P * F^T

        // 加上过程噪声协方差矩阵 Q
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}

/**
  * @brief          计算卡尔曼增益矩阵 `K`
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的增益矩阵 `K`。
  *
  * @note           根据卡尔曼滤波的标准更新公式，计算卡尔曼增益矩阵 `K`。首先通过状态转移矩阵 `H`、预测协方差
  *                 矩阵 `Pminus`、测量噪声协方差矩阵 `R`，以及中间结果矩阵，最终计算得到增益矩阵 `K`。计算过程
  *                 涉及矩阵乘法、转置、加法和求逆等操作。
  */
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    // 如果不跳过方程3
    if (!kf->SkipEq3)
    {
        // 计算 H^T (H 的转置)
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z

        // 计算 H·P'(k) (测量矩阵与预测协方差矩阵的乘积)
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)

        // 计算 H·P'(k)·H^T (最终形成协方差矩阵的部分)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT

        // 计算 S = H·P'(k)·H^T + R (增益矩阵的分母部分)
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H·P'(k)·HT + R

        // 求解 S 的逆矩阵
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1); // temp_matrix1 = inv(H·P'(k)·HT + R)

        // 计算 P'(k)·H^T (预测协方差矩阵与测量矩阵的乘积)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT

        // 最终计算卡尔曼增益矩阵 K = P'(k)·H^T·inv(H·P'(k)·HT + R)
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}

/**
  * @brief          更新卡尔曼滤波器的状态估计 `xhat(k)`
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的状态估计 `xhat(k)`。
  *
  * @note           该函数使用卡尔曼增益 `K`、测量矩阵 `H` 和测量值 `z(k)` 来更新状态估计 `xhat(k)`。
  *                 首先计算残差项 `z(k) - H·xhat'(k)`，然后通过增益矩阵 `K` 对状态估计进行修正。
  */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    // 如果不跳过方程4
    if (!kf->SkipEq4)
    {
        // 计算 H·xhat'(k)
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)

        // 计算残差 z(k) - H·xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)

        // 计算 K(k)·(z(k) - H·xhat'(k))
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

        // 更新 xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}

/**
  * @brief          更新卡尔曼滤波器的协方差矩阵 `P(k)`
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值。更新卡尔曼滤波器的协方差矩阵 `P(k)`。
  *
  * @note           该函数根据卡尔曼增益矩阵 `K(k)` 和测量矩阵 `H` 更新协方差矩阵 `P(k)`。
  *                 通过计算 `K(k)·H·P'(k)`，然后从预测协方差矩阵 `P'(k)` 中减去这个量，更新协方差矩阵。
  */
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    // 如果不跳过方程5
    if (!kf->SkipEq5)
    {
        // 计算 K(k)·H
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H

        // 计算 K(k)·H·P'(k)
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)

        // 更新协方差矩阵 P(k) = P'(k) - K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}

/**
  * @brief          更新卡尔曼滤波器的状态估计、协方差矩阵以及相关输出。
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
fp32 *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 处理测量数据
    Kalman_Filter_Measure(kf);

    // 执行用户定义的回调函数0（如果有的话）
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 更新 xhatminus 状态估计
    Kalman_Filter_xhatMinusUpdate(kf);

    // 执行用户定义的回调函数1（如果有的话）
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 更新 Pminus 协方差矩阵
    Kalman_Filter_PminusUpdate(kf);

    // 执行用户定义的回调函数2（如果有的话）
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    // 如果测量有效或未启用自动调整，则继续更新卡尔曼增益、状态估计和协方差矩阵
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 计算卡尔曼增益
        Kalman_Filter_SetK(kf);

        // 执行用户定义的回调函数3（如果有的话）
        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 更新状态估计 xhat
        Kalman_Filter_xhatUpdate(kf);

        // 执行用户定义的回调函数4（如果有的话）
        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 更新协方差矩阵 P
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 如果没有有效的测量，直接使用预测值进行更新
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 执行用户定义的回调函数5（如果有的话）
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 限制协方差矩阵的对角线元素，确保其不小于最小方差
    for (uint8_t i = 0; i < kf->xhatSize; i++)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    // 更新滤波后的状态值
    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    // 执行用户定义的回调函数6（如果有的话）
    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    // 返回滤波后的状态估计值
    return kf->FilteredValue;
}

/**
  * @brief          调整卡尔曼滤波器中的测量矩阵 H、卡尔曼增益矩阵 K 和测量噪声矩阵 R。
  * @author         冉文治
  * @param          kf           指向卡尔曼滤波器结构体的指针
  * @retval         无返回值
  *
  * @note           该函数用于根据有效的测量数据调整卡尔曼滤波器的测量矩阵（H）、卡尔曼增益矩阵（K）和测量噪声矩阵（R）。具体过程如下：
  *                 1. 通过测量向量 `MeasuredVector` 更新测量数据 `z_data`，并清空 `MeasuredVector`。
  *                 2. 遍历测量数据，将有效的非零测量值存储到 `z_data` 中，同时更新相关的 `H` 和 `R` 矩阵。
  *                 3. 更新测量矩阵 `H`，其行数为有效测量数目 `MeasurementValidNum`，列数为状态估计数目 `xhatSize`。
  *                 4. 更新卡尔曼增益矩阵 `K`，测量噪声矩阵 `R` 的尺寸均为有效测量数目 `MeasurementValidNum`。
  *                 5. 更新测量向量 `z`，其行数为有效测量数目 `MeasurementValidNum`。
  */
static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    // 将测量向量 MeasuredVector 的数据复制到 z_data 中
    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 清空 R 和 H 矩阵的数据
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);

    // 遍历测量数据，找到有效的非零测量值
    for (uint8_t i = 0; i < kf->zSize; i++)
    {
        if (kf->z_data[i] != 0)
        {
            // 存储有效的测量值和测量的索引
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;

            // 根据测量映射更新 H 矩阵
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }

    // 根据有效的测量数目更新 R 矩阵的对角线元素
    for (uint8_t i = 0; i < kf->MeasurementValidNum; i++)
    {
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 更新 H、R、K 矩阵的维度
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;

    // 更新测量向量的维度
    kf->z.numRows = kf->MeasurementValidNum;
}
