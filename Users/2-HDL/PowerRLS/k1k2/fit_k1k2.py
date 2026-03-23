import pandas as pd
import numpy as np
import glob
import os
import sys

# ==========================================================
#                      用户配置区
# ==========================================================

# [关键] 已知的 K3 (静态功耗)
K3_VALUE = 0.705

# [关键] 已知的 K0 (扭矩常数)
# 1. 如果你的 CSV_COL_TORQUE 列已经是真正的扭矩 (Nm)，请填 1.0
# 2. 如果你的 CSV_COL_TORQUE 列其实是电流 (A)，请填你的电机 K0 (例如 0.18 或 0.3)
K0_VALUE = 1.0

# [关键] CSV 列名配置
# 注意：Ozone 导出的 CSV 列名可能包含空格，请确保完全一致
CSV_COL_INPUT_T  = "(Motor_PowerLimitator[0]).getTorqueFeedback"
CSV_COL_INPUT_W  = "(Motor_PowerLimitator[0]).getRadFeedback"
CSV_COL_POWER    = "((SuperCap_Data).Data).chassisPower"

# [关键] 电机数量
# RLS 会将单电机的 t 和 w 扩展到整车 (x4) 来拟合总功率
# 如果你的数据是单个电机的数据，这里填 1
# 如果你想用这个单电机代表全车 4 个电机，这里填 4
MOTOR_COUNT = 1

# RLS 初始参数
INITIAL_GUESS_K1 = 0.01   
INITIAL_GUESS_K2 = 1.0    
FORGETTING_FACTOR = 1.0   

# ==========================================================

class RLSModelVector:
    """
    向量版 RLS 滤波器 (修正了维度错误)
    """
    def __init__(self, n_features, initial_theta, initial_P_val=1000.0):
        self.n = n_features
        # 确保 theta 是 (n, 1) 的列向量
        self.theta = np.array(initial_theta, dtype=float).reshape(n_features, 1)
        self.P = np.eye(n_features) * initial_P_val
        self.lambda_factor = FORGETTING_FACTOR

    def update(self, x, y):
        # 确保 x 是 (n, 1) 的列向量
        x = np.array(x).reshape(self.n, 1)
        
        # 1. Prediction & Error
        # [修正点] 使用 .item() 将 (1,1) 的矩阵转为标量数值
        prediction_matrix = np.dot(x.T, self.theta)
        prediction = prediction_matrix.item() 
        
        error = y - prediction

        # 2. Gain Calculation
        Px = np.dot(self.P, x)
        denominator_matrix = self.lambda_factor + np.dot(x.T, Px)
        denominator = denominator_matrix.item()
        
        if denominator == 0:
            return 0.0
            
        K = Px / denominator

        # 3. Update Parameters
        # K 是 (2,1)，error 是标量，可以直接乘
        self.theta = self.theta + K * error

        # 4. Update Covariance
        self.P = (self.P - np.dot(K, x.T).dot(self.P)) / self.lambda_factor
        
        return float(error)

def main():
    csv_files = glob.glob("*.csv")
    if not csv_files:
        print("错误: 当前目录下未找到 .csv 文件。")
        return

    # 初始化 RLS: 2个特征 (k1, k2)
    rls = RLSModelVector(2, [INITIAL_GUESS_K1, INITIAL_GUESS_K2])
    
    total_points = 0
    last_error = 0.0
    processed_files = []

    print(f"正在扫描并处理数据... (共 {len(csv_files)} 个文件)")
    print(f"配置: K0={K0_VALUE}, K3={K3_VALUE}, N={MOTOR_COUNT}")

    for file_path in csv_files:
        try:
            # [优化] 尝试智能推断分隔符，Ozone有时候是逗号有时候是分号
            df = pd.read_csv(file_path, sep=None, engine='python')
            
            # 去除列名可能存在的首尾空格
            df.columns = df.columns.str.strip()
            
            required_cols = [CSV_COL_INPUT_T, CSV_COL_INPUT_W, CSV_COL_POWER]
            
            # 检查列是否存在
            missing_cols = [col for col in required_cols if col not in df.columns]
            if missing_cols:
                print(f"跳过 {file_path}: 缺少列 {missing_cols}")
                # 打印一下实际列名供调试
                # print(f"  文件包含列名: {list(df.columns)}")
                continue
            
            processed_files.append(file_path)

            for _, row in df.iterrows():
                try:
                    # 1. 读取原始数据
                    raw_input_t = float(row[CSV_COL_INPUT_T])
                    w_val = float(row[CSV_COL_INPUT_W])
                    p_real = float(row[CSV_COL_POWER])

                    # 2. 计算真实扭矩 (Nm)
                    # t = input * k0
                    t_val = raw_input_t * K0_VALUE

                    # 3. 计算单电机机械功率 (P_mech = t * w)
                    # 机械功率有正负，不做绝对值处理
                    p_mech_single = t_val * w_val
                    
                    # 4. 扩展到整车机械功率
                    p_mech_total = p_mech_single * MOTOR_COUNT
                    
                    # 5. 构建 RLS 目标值 Y (纯损耗)
                    # Y = P_real - P_mech - P_static
                    y_target = p_real - p_mech_total - K3_VALUE

                    # 6. 构建 RLS 输入特征 X
                    # 损耗模型: Loss = N * (k1 * |w| + k2 * t^2)
                    x_features = [
                        MOTOR_COUNT * abs(w_val),    # x1: 对应 k1
                        MOTOR_COUNT * (t_val ** 2)   # x2: 对应 k2
                    ]

                    # 7. 简单的去噪
                    if p_real < 0.1: 
                        continue

                    # 8. RLS 更新
                    last_error = rls.update(x_features, y_target)
                    total_points += 1
                    
                except ValueError:
                    continue

        except Exception as e:
            print(f"处理 {file_path} 失败: {e}")
            import traceback
            traceback.print_exc()

    # ================= 最终输出 =================
    os.system('cls' if os.name == 'nt' else 'clear')
    
    k1_result = rls.theta[0][0]
    k2_result = rls.theta[1][0]

    k1_display = max(0.0, k1_result)
    k2_display = max(0.0, k2_result)

    print("##################################################")
    print("           RoboMaster 参数双拟合 (Fixed)          ")
    print("##################################################")
    print(f"  [-] 有效样本            : {total_points}")
    print(f"  [-] K0 (扭矩/电流常数)  : {K0_VALUE}")
    print(f"  [-] K3 (静态功耗)       : {K3_VALUE}")
    print("--------------------------------------------------")
    print(f"  float k1 = {k1_display:.10f}f;   // 摩擦系数")
    print(f"  float k2 = {k2_display:.10f}f;   // 热损耗系数")
    print("--------------------------------------------------")
    print(f"  [DEBUG] k1 (raw)        : {k1_result}")
    print(f"  [DEBUG] k2 (raw)        : {k2_result}")
    print("")

if __name__ == "__main__":
    main()
