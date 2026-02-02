#include "wbr.h"
#include "INS_task.h"
#include "arm_math.h"
#include "math.h"

/**
  * @brief  		初始化WBR结构体，设置各项参数并获取惯性测量单元（INS）数据
  * @param			wbr  指向WBR结构体的指针
  * @retval     	无
  */
void wbr_init(wbr_leg_t *wbr)
{
	// 设置VMC各个长度参数
	wbr->L1u = L1U;
	wbr->L1d = L1D;
	wbr->L2u = L2U;
	wbr->L2d = L2D;

	wbr->INS_angle = get_INS_angle_point();		// 获取角度数据指针
}

/**
  * @brief  		基于时间步长dt，计算WBR的关键参数
  * @param			wbr  指向WBR结构体的指针
  * @param			dt   时间步长，单位：秒
  * @retval     	无
  */
void wbr_calc(wbr_leg_t *wbr)
{
	wbr->X1 = - wbr->L1u * arm_cos_f32(wbr->phi1);
	wbr->X2 = - wbr->L2u * arm_cos_f32(wbr->phi2);
	wbr->Y1 = wbr->L1u * arm_sin_f32(wbr->phi1);
	wbr->Y2 = wbr->L2u * arm_sin_f32(wbr->phi2);

	// 计算sigma
	arm_sqrt_f32(-(-wbr->L1d * wbr->L1d - 2 * wbr->L1d * wbr->L2d - wbr->L2d * wbr->L2d + wbr->X1 * wbr->X1
					+ 2 * wbr->X1 * wbr->X2 + wbr->X2 * wbr->X2 + wbr->Y1 * wbr->Y1 - 2 * wbr->Y1 * wbr->Y2 + wbr->Y2 * wbr->Y2)
					* (-wbr->L1d * wbr->L1d + 2 * wbr->L1d * wbr->L2d - wbr->L2d * wbr->L2d + wbr->X1 * wbr->X1
					+ 2 * wbr->X1 * wbr->X2 + wbr->X2 * wbr->X2 + wbr->Y1 * wbr->Y1 - 2 * wbr->Y1 * wbr->Y2 + wbr->Y2 * wbr->Y2)
					, &wbr->sig);

	// 计算 Xe 和 Ye
	wbr->Xe = (wbr->Y1 * wbr->sig - wbr->Y2 * wbr->sig + wbr->L1d * wbr->L1d * wbr->X1 + wbr->L1d * wbr->L1d * wbr->X2
				- wbr->L2d * wbr->L2d * wbr->X1 - wbr->L2d * wbr->L2d * wbr->X2 + wbr->X1 * wbr->X2 * wbr->X2
				- wbr->X1 * wbr->X1 * wbr->X2 - wbr->X1 * wbr->Y1 * wbr->Y1 - wbr->X1 * wbr->Y2 * wbr->Y2
				+ wbr->X2 * wbr->Y1 * wbr->Y1 + wbr->X2 * wbr->Y2 * wbr->Y2 - wbr->X1 * wbr->X1 * wbr->X1
				+ wbr->X2 * wbr->X2 * wbr->X2 + 2 * wbr->X1 * wbr->Y1 * wbr->Y2 - 2 * wbr->X2 * wbr->Y1 * wbr->Y2)
				/ (2 * (wbr->X1 * wbr->X1 + 2 * wbr->X1 * wbr->X2 + wbr->X2 * wbr->X2 + wbr->Y1 * wbr->Y1
				- 2 * wbr->Y1 * wbr->Y2 + wbr->Y2 * wbr->Y2));  // 计算 Xe

	wbr->Ye = (wbr->X1 * wbr->sig + wbr->X2 * wbr->sig - wbr->L1d * wbr->L1d * wbr->Y1 + wbr->L1d * wbr->L1d * wbr->Y2
				+ wbr->L2d * wbr->L2d * wbr->Y1 - wbr->L2d * wbr->L2d * wbr->Y2 + wbr->X1 * wbr->X1 * wbr->Y1
				+ wbr->X1 * wbr->X1 * wbr->Y2 + wbr->X2 * wbr->X2 * wbr->Y1 + wbr->X2 * wbr->X2 * wbr->Y2
				- wbr->Y1 * wbr->Y2 * wbr->Y2 - wbr->Y1 * wbr->Y1 * wbr->Y2 + wbr->Y1 * wbr->Y1 * wbr->Y1
				+ wbr->Y2 * wbr->Y2 * wbr->Y2 + 2 * wbr->X1 * wbr->X2 * wbr->Y1 + 2 * wbr->X1 * wbr->X2 * wbr->Y2)
				/ (2 * (wbr->X1 * wbr->X1 + 2 * wbr->X1 * wbr->X2 + wbr->X2 * wbr->X2 + wbr->Y1 * wbr->Y1
				- 2 * wbr->Y1 * wbr->Y2 + wbr->Y2 * wbr->Y2));  // 计算 Ye

	// 五连杆长度L相关数据
	arm_sqrt_f32(wbr->Xe * wbr->Xe + wbr->Ye * wbr->Ye, &wbr->L);  //计算虚拟杆长
	wbr->dd_L = wbr->d_L - wbr->d_L_p;	//计算虚拟杆加速度用于离地检测

	// 五连杆角度theta相关数据
	wbr->theta = atan2f(wbr->Xe, wbr->Ye);

	// 腿部倾斜角相关数据
	wbr->theta_l = wbr->theta - *(wbr->INS_angle + INS_PITCH_ADDRESS_OFFSET);


	if (wbr->theta_l - wbr->theta_l_p > PI)
	{
		wbr->count--;
	}
	else if (wbr->theta_l - wbr->theta_l_p < -PI)
	{
		wbr->count++;
	}
	wbr->total_theta_l = 2 * PI * wbr->count + wbr->theta_l;

	// 更新数据
	wbr->d_L_p = wbr->d_L;
	wbr->theta_l_p = wbr->theta_l;
}

/**
  * @brief  		雅可比矩阵求关节电机力矩
  * @param			wbr  指向WBR结构体的指针
  * @retval     	无
  */
void wbr_jacobian_calc(wbr_leg_t *wbr)
{
	// 计算雅可比矩阵的元素
	wbr->PHI_1l = ((wbr->Xe + wbr->X1) * arm_sin_f32(wbr->theta) + (wbr->Ye - wbr->Y1) * arm_cos_f32(wbr->theta))
				/ (wbr->L1u * (-(wbr->Xe + wbr->X1) * arm_sin_f32(wbr->phi1) + (wbr->Ye - wbr->Y1) * arm_cos_f32(wbr->phi1)));

	wbr->PHI_1theta = wbr->L * ((wbr->Xe + wbr->X1) * arm_cos_f32(wbr->theta) - (wbr->Ye - wbr->Y1) * arm_sin_f32(wbr->theta))
					/ (wbr->L1u * (-(wbr->Xe + wbr->X1) * arm_sin_f32(wbr->phi1) + (wbr->Ye - wbr->Y1) * arm_cos_f32(wbr->phi1)));

	wbr->PHI_2l = ((wbr->Xe - wbr->X2) * arm_sin_f32(wbr->theta) + (wbr->Ye - wbr->Y2) * arm_cos_f32(wbr->theta))
				/ (wbr->L2u * ((wbr->Xe - wbr->X2) * arm_sin_f32(wbr->phi2) + (wbr->Ye - wbr->Y2) * arm_cos_f32(wbr->phi2)));

	wbr->PHI_2theta = wbr->L * ((wbr->Xe - wbr->X2) * arm_cos_f32(wbr->theta) - (wbr->Ye - wbr->Y2) * arm_sin_f32(wbr->theta))
				/ (wbr->L2u * ((wbr->Xe - wbr->X2) * arm_sin_f32(wbr->phi2) + (wbr->Ye - wbr->Y2) * arm_cos_f32(wbr->phi2)));

	wbr->detA = wbr->PHI_1l * wbr->PHI_2theta - wbr->PHI_1theta * wbr->PHI_2l;

	//对矩阵求逆
	wbr->J_inv[0][0] = wbr->PHI_2theta / wbr->detA;
	wbr->J_inv[0][1] = -wbr->PHI_1theta / wbr->detA;
	wbr->J_inv[1][0] = -wbr->PHI_2l / wbr->detA;
	wbr->J_inv[1][1] = wbr->PHI_1l / wbr->detA;

	wbr->J[0][0] = wbr->J_inv[0][0];
	wbr->J[0][1] = wbr->J_inv[1][0];
	wbr->J[1][0] = wbr->J_inv[0][1];
	wbr->J[1][1] = wbr->J_inv[1][1];
}

/**
  * @brief  		将线性化的拟合矩阵根据不同腿长计算出反馈矩阵K
  * @param			fitted_matrix	拟合矩阵
  * @param			K_matrices		反馈矩阵K
  * @retval     	无
  */
void decompose_fitted_matrix_to_K(wbr_leg_t *left_leg, wbr_leg_t *right_leg, fp32 fitted_matrix[40][6], fp32 K_matrices[4][10])
{
	// 预计算 L 和 L^2，减少重复运算
	fp32 L_left = left_leg->L;
	fp32 L_right = right_leg->L;
	fp32 L_left2 = L_left * L_left;
	fp32 L_right2 = L_right * L_right;
	fp32 L_left_right = L_left * L_right;

	for (uint8_t k = 0; k < 4; k++)
	{
		for (uint8_t i = 0; i < 10; i++)
		{
			uint8_t fit_row = i + k * 10;
			fp32 *row = fitted_matrix[fit_row];  // 直接指向当前行，提高可读性

			K_matrices[k][i] = row[0]
							 + row[1] * L_left
							 + row[2] * L_right
							 + row[3] * L_left2
							 + row[4] * L_left_right
							 + row[5] * L_right2;
		}
	}
}

/**
  * @brief  		将线性化的拟合矩阵根据不同腿长计算出反馈矩阵A
  * @param			fitted_matrix	拟合矩阵
  * @param			A_matrices		反馈矩阵A
  * @retval     	无
  */
void decompose_fitted_matrix_to_A(wbr_leg_t *left_leg, wbr_leg_t *right_leg, fp32 fitted_matrix[15][6], fp32 A_matrices[10][10])
{
	// 预计算 L 和 L^2，减少重复运算
	fp32 L_left = left_leg->L;
	fp32 L_right = right_leg->L;
	fp32 L_left2 = L_left * L_left;
	fp32 L_right2 = L_right * L_right;
	fp32 L_left_right = L_left * L_right;

	for (int16_t i = 0; i < 15; ++i)
	{
		int16_t row = (i / 3) * 2 + 1;
		int16_t col = (i % 3) * 2 + 4;

		// 直接使用指针减少访问开销
		fp32 *row_data = fitted_matrix[i];

		A_matrices[row][col] = row_data[0]
							 + row_data[1] * L_left
							 + row_data[2] * L_right
							 + row_data[3] * L_left2
							 + row_data[4] * L_left_right
							 + row_data[5] * L_right2;
	}
	A_matrices[0][1] = 1;
	A_matrices[2][3] = 1;
	A_matrices[4][5] = 1;
	A_matrices[6][7] = 1;
	A_matrices[8][9] = 1;
}

/**
  * @brief  		将线性化的拟合矩阵根据不同腿长计算出反馈矩阵B
  * @param			fitted_matrix	拟合矩阵
  * @param			B_matrices		反馈矩阵B
  * @retval     	无
  */
void decompose_fitted_matrix_to_B(wbr_leg_t *left_leg, wbr_leg_t *right_leg, fp32 fitted_matrix[20][6], fp32 B_matrices[10][4])
{
	// 预计算 L 和 L^2，减少重复运算
	fp32 L_left = left_leg->L;
	fp32 L_right = right_leg->L;
	fp32 L_left2 = L_left * L_left;
	fp32 L_right2 = L_right * L_right;
	fp32 L_left_right = L_left * L_right;

	for (int16_t i = 0; i < 20; ++i)
	{
		int16_t row = (i / 4) * 2 + 1;
		int16_t col = i % 4;

		// 直接使用指针减少访问开销
		fp32 *row_data = fitted_matrix[i];

		B_matrices[row][col] = row_data[0]
							 + row_data[1] * L_left
							 + row_data[2] * L_right
							 + row_data[3] * L_left2
							 + row_data[4] * L_left_right
							 + row_data[5] * L_right2;
	}
}
