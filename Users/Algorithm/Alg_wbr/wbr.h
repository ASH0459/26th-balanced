#ifndef WBR_H
#define WBR_H

#include "main.h"

#define L1U		0.2055f
#define L2U		0.2055f
#define L1D		0.258f
#define L2D		0.258f

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	const fp32 *INS_angle;

	// 设定腿长
	fp32 L1u;
	fp32 L1d;
	fp32 L2u;
	fp32 L2d;

    // 五连杆角度phi1和phi2，通过电机编码器读取
    fp32 phi1;
    fp32 phi2;

    // 计算得到的五连杆长度相关数据
    fp32 L;
	fp32 d_L, d_L_p;
	fp32 dd_L;

	// 计算得到的五连杆角度相关数据
	fp32 theta;

	// 腿部角度
	fp32 theta_l, theta_l_p;
	fp32 d_theta_l;

    // 雅可比矩阵各元素
    fp32 PHI_1l, PHI_1theta, PHI_2l, PHI_2theta;
    // 雅可比矩阵
    fp32 J[2][2];

	// 目标支持力和转矩
	fp32 Fbl_t, Tbl_t;
	// 实际支持力和转矩
	fp32 Fbl_r, Tbl_r;

    // 衍生变量
    fp32 X1, Y1;
    fp32 X2, Y2;
    fp32 Xe, Ye;
    fp32 sig;
    fp32 detA;

    fp32 J_inv[2][2];

	// 腿旋转圈数
	int8_t count;
	fp32 total_theta_l;

} wbr_leg_t;

/**
  * @brief  		初始化WBR结构体，设置各项参数并获取惯性测量单元（INS）数据
  * @param			wbr  指向WBR结构体的指针
  * @retval     	无
  */
extern void wbr_init(wbr_leg_t *wbr);

/**
  * @brief  		基于时间步长dt，计算WNR的关键参数
  * @param			wbr  指向WBR结构体的指针
  * @param			dt   时间步长，单位：秒
  * @retval     	无
  */
extern void wbr_calc(wbr_leg_t *wbr);

/**
  * @brief  		根据已知的物理参数计算WBR的力矩设置
  * @param			wbr  指向WBR结构体的指针
  * @retval     	无
  */
extern void wbr_jacobian_calc(wbr_leg_t *wbr);

/**
  * @brief  		将线性化的拟合矩阵根据不同腿长计算出反馈矩阵K
  * @param			fitted_matrix	拟合矩阵
  * @param			K_matrices		反馈矩阵K
  * @retval     	无
  */
extern void decompose_fitted_matrix_to_K(wbr_leg_t *left_leg, wbr_leg_t *right_leg, fp32 fitted_matrix[40][6], fp32 K_matrices[4][10]);

/**
  * @brief  		将线性化的拟合矩阵根据不同腿长计算出反馈矩阵A
  * @param			fitted_matrix	拟合矩阵
  * @param			A_matrices		反馈矩阵A
  * @retval     	无
  */
extern void decompose_fitted_matrix_to_A(wbr_leg_t *left_leg, wbr_leg_t *right_leg, fp32 fitted_matrix[15][6], fp32 A_matrices[10][10]);

/**
  * @brief  		将线性化的拟合矩阵根据不同腿长计算出反馈矩阵B
  * @param			fitted_matrix	拟合矩阵
  * @param			B_matrices		反馈矩阵B
  * @retval     	无
  */
extern void decompose_fitted_matrix_to_B(wbr_leg_t *left_leg, wbr_leg_t *right_leg, fp32 fitted_matrix[20][6], fp32 B_matrices[10][4]);

#ifdef __cplusplus
}
#endif

#endif
