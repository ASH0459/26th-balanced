/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Alg_PID.h"
#include "Alg_UserLib.h"

/*---------------------------------------------------------------
 * Function:        这其实是个宏
 * Description:     对需要的值进行限幅操作，这里是I的限幅和PID控制器最后输出的限幅
 * Input:           input				想要进行限幅的值  
										max					限幅值  
 * Output:          NULL
---------------------------------------------------------------*/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/*---------------------------------------------------------------
 * Function: 				PID_init(...)
 * Description:     PID初始化函数
 * Input:           pid					相应PID控制器的结构体指针
                    mode				0：Position位置式PID  1：delta增量式PID
                    max_out				对PID控制器最后输出的限幅值
                    max_iout			对PID控制器运算中Iout的限幅值
 * Output:          NULL
---------------------------------------------------------------*/
void PID_init(PidTypeDef_t *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->last_fdb = pid->last_Dout = 0.0f;
}

/*---------------------------------------------------------------
 * Function:        float PID_Calc(...)
 * Description:     PID控制器运算函数
 * Input:           pid						相应PID控制器的结构体指针
										ref						当前回传的实际值
										set						设定值/期望值
 * Output:          PID控制器的输出值
---------------------------------------------------------------*/
float PID_Calc(PidTypeDef_t *pid, float ref, float set)
{

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		
		
    if (pid->mode == PID_POSITION)			//采用位移式PID
    {
        // pid->Pout = pid->Kp * pid->error[0];
        // pid->Iout += pid->Ki * pid->error[0];
        // pid->Dbuf[2] = pid->Dbuf[1];
        // pid->Dbuf[1] = pid->Dbuf[0];
        // pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        // pid->Dout = pid->Kd * pid->Dbuf[0];
        // LimitMax(pid->Iout, pid->max_iout);
        // pid->out = pid->Pout + pid->Iout + pid->Dout;
        // LimitMax(pid->out, pid->max_out);


    	// 1. 计算比例项 (P)
    	pid->Pout = pid->Kp * pid->error[0];

    	// 2. 计算积分项 (I) - [新增积分分离逻辑]
    	// 假设你要在误差小于 5.0 的时候才开启积分 (这个 5.0 需要根据你的物理单位，比如毫米、厘米来修改)
    	// 最好是将这个阈值加到你的 PidTypeDef_t 结构体里，比如 pid->I_Band
    	float I_Band = 0.01f;

    	if (fabsf(pid->error[0]) < I_Band)
    	{
    		// 当误差在阈值范围内时，正常累加积分
    		pid->Iout += pid->Ki * pid->error[0];
    		LimitMax(pid->Iout, pid->max_iout); // 原有的积分限幅保留
    	}
    	else
    	{
    		// [关键] 当误差过大时，不仅不累加积分，还要把积分项清零
    		// 防止在快速冲向目标的过程中累积错误的力矩
    		pid->Iout = 0.0f;
    	}

    	// 3. 计算微分项 (D)
    	pid->Dbuf[2] = pid->Dbuf[1];
    	pid->Dbuf[1] = pid->Dbuf[0];
    	pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    	pid->Dout = pid->Kd * pid->Dbuf[0];

    	// 4. 计算总输出
    	pid->out = pid->Pout + pid->Iout + pid->Dout;
    	LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_Incremental)		//采用增量式PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
		
    return pid->out;
}

/*---------------------------------------------------------------
 * Function:        float Double_PID_Calc(...)
 * Description:     PID控制器运算函数
 * Input:           pid						相应PID控制器的结构体指针
										ref						当前回传的实际值
										set						设定值/期望值
 * Output:          PID控制器的输出值
---------------------------------------------------------------*/
float Double_PID_Calc(PidTypeDef_t *pid_out, PidTypeDef_t *pid_in, float ref_out, float ref_in, float set_out, float set_in)
{
	PID_Calc(pid_out, ref_out, set_out);
	PID_Calc(pid_in, ref_in, pid_out->out);
}

/*---------------------------------------------------------------
 * Function:        float IMU_PID_Calc(...)
 * Description:     PID控制器运算函数(针对角度环)
 * Input:           pid						相应PID控制器的结构体指针
										ref						当前回传的实际值
										set						设定值/期望值
										error_delta   两次误差之差：此处是角速度 
 * Output:          PID控制器的输出值
---------------------------------------------------------------*/
float IMU_PID_Calc(PidTypeDef_t* pid, float ref, float set, float error_delta)
{
	float err;
	
	if (pid == NULL)
	{
			return 0.0f;
	}
	
	pid->fdb = ref;
	pid->set = set;

	err = set - ref;
	pid->error[0] = rad_format(err); //利用循环限幅函数将误差限制在[-PI，PI]
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dout = pid->Kd * error_delta;
	abs_limit(&pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	abs_limit(&pid->out, pid->max_out);
	return pid->out;
}


float Leg_PID_Calc(PidTypeDef_t *pid, float ref, float set)
{
	if (pid == NULL) return 0.0f;

	// 更新误差历史
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->set = set;
	pid->fdb = ref;
	pid->error[0] = set - ref;

	if (pid->mode == PID_POSITION)
	{
		// 1. 计算比例项 (P)
		pid->Pout = pid->Kp * pid->error[0];

		// 2. 计算积分项 (I) - [带死区的四段式积分管理]
		float I_Band = 0.01;
		float I_Deadband = 0.001f; // 【新增】积分死区：5毫米 (0.005m)

		if (fabsf(pid->error[0]) < I_Deadband)
		{
			// 【区域1：积分死区】
			// 误差已经小于 1 毫米了！此时可以说是“完美到达目标”。
			// 我们什么都不做，保持当前的 Iout 不变！
			// 这将彻底消除最后那“一点上升”的过冲现象和低频呼吸震荡。
		}
		else if (fabsf(pid->error[0]) < I_Band)
		{
			// 【区域2：正常积分区】
			// 误差在 1毫米 到 1.5厘米 之间，大胆快速地积分！
			// 注意：你现在可以把外面的 Ki 调得非常大（比如 100~300），让它在 0.5 秒内就冲到位
			pid->Iout += pid->Ki * pid->error[0];
			LimitMax(pid->Iout, pid->max_iout); // 积分绝对限幅
		}
		else if (fabsf(pid->error[0]) < (I_Band * 2.0f))
		{
			// 【区域3：缓冲冻结区】
			// 误差在 1.5厘米 到 3厘米 之间，边缘试探，保持 Iout 不变，防抽搐
		}
		else
		{
			// 【区域4：彻底清零区】
			// 误差极大，清空积分
			pid->Iout = 0.0f;
		}

		// 3. 计算微分项 (D)
		float current_speed = -(pid->fdb - pid->last_fdb);
		float raw_Dout = pid->Kd * current_speed;

		// 【修复全局抖动】：把滤波系数大幅度减小，增强低通滤波效果！
		// 如果改了 0.05 还是抖，可以继续改到 0.02。
		pid->D_alpha = 0.3f;
		pid->Dout = (pid->D_alpha * raw_Dout) + ((1.0f - pid->D_alpha) * pid->last_Dout);

		// 保存本次计算数据
		pid->last_fdb = pid->fdb;
		pid->last_Dout = pid->Dout;

		// 4. 计算总输出
		pid->out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->out, pid->max_out);
	}

	return pid->out;
}
/*---------------------------------------------------------------
 * Function:        PID_clear(PidTypeDef_t *pid)
 * Description:     PID复位（清零）函数
 * Input:           pid	相应PID控制器的结构体指针
 * Output:          NULL
---------------------------------------------------------------*/
void PID_clear(PidTypeDef_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
    pid->last_fdb = pid->last_Dout = 0.0f;
}

//修改pid数值
void PID_Change(PidTypeDef_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
