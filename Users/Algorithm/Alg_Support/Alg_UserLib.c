/* Includes ------------------------------------------------------------------*/
#include "Alg_UserLib.h"

/**
  * @brief          RSR(reciprocal square root) function 平方根倒数
  * @param[in]      val
  * @retval         return rsr(val)
  */
float rsr(float val)
{
    float halfnum = 0.5f * val;
    float y = val;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
            first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(float *num, float Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
float sign(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    else if (value < 0.0f)
    {
        return -1.0f;
    }
    else
    {
        return 0;
    }
}

//浮点死区
float float_deadline(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//uint16_t型限幅函数
void fn_Uint16Limit(uint16_t *value, uint16_t min, uint16_t max)
{
  if (*value < min)
  {
    *value = min;
  }
  else if (*value > max)
  {
    *value = max;
  }
}
//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI


/**
    * @brief 计算最短角度误差 (目标 - 当前)，结果限制在 [-180, 180) 内
    * @param target 目标角度
    * @param current 当前角度
    * @return 最短路径误差
    */
float shortest_angle_error(float target, float current) {
    float error = target - current;

    // 核心逻辑：将误差归一化到 [-180, 180]
    while (error > PI) {
        error -= 2.0f*PI;
    }
    while (error < -PI) {
        error += 2.0f*PI;
    }

    return error;
}

//角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

float angle_format_360(float angle)
{
    angle = fmod(angle, 360.0f);
    if (angle < 0)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
  * @brief          生成随机数
  * @author         罗威昊、Light
  * @param[in]      随机数种子
  * @param[in]      生成随机数的最小值
  * @param[in]      生成随机数的最大值
  * @retval         返回生成的随机数
  */
uint16_t customRand(unsigned int *seed)
{
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    return *seed;
}

float Range_Number(float min, float max, const unsigned int *seed)
{
    float r = (float)customRand(seed) / 0x7fffffff; // 生成0到1之间的随机数
    return min + r * (max - min);
}

/**
  * @brief          计算浮点数数组之和
  * @param[in]      array   需要计算的数组
  *                 length  需要计算的数组长度
  *
  * @retval         浮点数数组之和
  */
float float_sum(float *array, uint8_t length)
{
    float sum = 0;

    for (int i = 0; i < length; i++)
    {
        sum += array[i];
    }

    return sum;
}