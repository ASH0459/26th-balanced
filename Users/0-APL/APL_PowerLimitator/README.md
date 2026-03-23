# 底盘功率控制模块 (PowerLimitator) 使用说明与原理解析

本文档详细介绍了 `APL_PowerLimitator` 和 `HDL_PowerLimitator` 模块的工作原理、核心逻辑以及如何将其接入到现有的机器人代码中，以实现对底盘电机功率的精准控制，防止因超功率而导致机器人扣血。

---

## 1. 模块概述

在 RoboMaster 比赛中，裁判系统对机器人的底盘功率有严格的限制。如果底盘电机的瞬时功率超过了裁判系统规定的上限，机器人将会受到扣血惩罚。
本模块的核心作用是：**在底盘电机执行 PID 输出电流之前，通过建立电机功率模型，预测当前指令下的总功率。如果预测功率超限，则根据各电机的当前状态（转速、扭矩等）动态计算并削减（衰减）电机的输出电流，从而将总功率严格限制在安全范围内。**

同时，本模块还引入了**能量环 PID 控制器**，结合超级电容的剩余能量，动态调整底盘的最大功率上限，实现更平滑、更高效的功率调度。

---

## 2. 核心逻辑与工作原理

### 2.1 功率模型 (Power Model)
电机的功率并非简单的电压乘以电流，而是与电机的转速、扭矩以及内部损耗密切相关。本模块使用了一个基于物理特性的多项式功率模型：
$$ P = k_0 \cdot \tau \cdot \omega + k_1 \cdot \omega^2 + k_2 \cdot \tau^2 + k_3 $$
其中：
- $\tau$ (Torque)：电机扭矩（由电流转换而来）。
- $\omega$ (Omega)：电机角速度（由转速 RPM 转换而来）。
- $k_0, k_1, k_2, k_3$：通过系统辨识（如 RLS 算法）或实验测得的电机功率模型系数。

在 `PowerLimitator_Status_Update` 函数中，系统会遍历所有底盘电机，分别计算：
- **预估功率 (Estimated Power)**：基于电机**当前实际反馈**的转速和扭矩计算出的当前消耗功率。
- **命令功率 (Command Power)**：基于电机**当前实际反馈**的转速和**PID控制器期望输出**的扭矩计算出的即将消耗的功率。

### 2.2 功率限制与电流衰减 (Decay Current)
当计算出的**命令总功率 (`commandPower`)** 超过了**最大允许功率 (`maxPower`)** 时，系统必须对电机的输出电流进行限制。
在 `PowerLimitator_getDecayCurrent` 函数中，系统执行以下逻辑：
1. **计算功率权重**：根据每个电机的转速，计算其在总功率削减中应承担的比例。转速越高的电机，削减其电流对降低功率的效果越明显。
2. **求解衰减电流**：将功率限制问题转化为求解一元二次方程。通过公式计算出每个电机需要减少的电流值（`decayCurrent`）。
3. **输出最终电流**：最终实际发送给电机的电流为：
   $$ \text{finalcurrent} = \text{pids\_out} - \text{decayCurrent} $$

### 2.3 能量环 PID 控制 (Energy PID)
为了充分利用超级电容的能量，模块中包含了一个能量环 PID 控制器 (`PowerLimitator_Energy_PID`)。
- **输入**：实际给定的最大功率的开平方。
- **反馈**：超级电容剩余电量（`capEnergy`）的开平方。
- **输出**：基于裁判系统给定的底盘最高功率的**功率调整量**。
*注：对能量和功率开平方是为了贴合能量与功率的非线性特性，使 PID 控制器在调节时更加线性、稳定。*

---

## 3. 代码结构

- **`HDL_PowerLimitator` (硬件抽象层)**：
  - 包含功率控制的核心算法、低通滤波器、PID 控制器以及电机状态数据的存储结构。
  - 负责纯数学计算和状态更新，不涉及具体的硬件通信。
- **`APL_PowerLimitator` (应用层)**：
  - 包含 FreeRTOS 任务 `PowerLimitator_Task`。
  - 负责周期性地调用 HDL 层的算法函数，并与实际的硬件数据（如电机反馈、裁判系统数据、超级电容数据）进行交互。
- **`PowerRLS` (参数拟合工具)**：
  - 包含用于离线计算电机功率模型参数 ($k_1, k_2$) 的 Python 脚本。

---

## 4. 电机参数 ($k_0 \sim k_3$) 测量与标定指南

为了让功率模型准确预测功耗，必须通过实验测量出 $k_0, k_1, k_2, k_3$ 这四个参数。

### 4.1 核心注意事项
- **分类测量**：参数标定必须**针对同一类电机**进行（例如 3508 和 6020 必须分开测）。
- **单一变量**：因为超级电容只能反馈整个底盘的总功率，无法区分每个电机的功耗。所以在测量某一类电机（如 3508）时，**必须将其他类型的电机（如 6020）断电**。测完一类后，再断电当前电机，给另一类电机上电进行测量。
- **功率读取**：测量过程中的实际消耗功率，可以通过读取超级电容反馈的底盘功率数据获得。

### 4.2 测量步骤

**第一步：确定 $k_0$ 和 $k_3$**
- **$k_0$ (扭矩常数)**：直接查阅电机的数据手册获取。注意区分是转子本身的扭矩常数，还是经过减速箱后输出轴的扭矩常数，代码中需要统一单位。
- **$k_3$ (静态功耗)**：
  1. 将机器人架空，确保轮子悬空无负载。
  2. 给目标电机发送 `0` 转速指令（即让电机保持静止但处于使能状态）。
  3. 观察并记录此时超级电容反馈的底盘功率。
  4. 记录一段时间的数据并取平均值，这个平均功率就是 $k_3$。

**第二步：测量 $k_1$ (空载动态功耗系数)**
$k_1$ 反映了电机转速对功耗的影响。
1. 保持机器人架空（空载状态）。
2. 给电机设定不同的目标转速（例如从低速到高速设置多个梯度）。
3. 在每个转速稳定后，记录当前的**转速**以及对应的**底盘功率**。
4. 尽量多收集不同转速下的数据点，数据越丰富，拟合越准确。

**第三步：测量 $k_2$ (堵转功耗系数)**
$k_2$ 反映了电机输出扭矩对功耗的影响。
1. 将电机转子或轮子**物理锁死**（堵转状态），使其转速始终为 0。
2. 给电机发送不同的扭矩指令（或电流指令）。
3. 在每个扭矩指令下，记录当前的**输出扭矩**以及对应的**底盘功率**。
4. 同样，收集多个不同扭矩下的数据点。

### 4.3 使用 `fit_k1k2.py` 脚本计算参数

在完成上述第二步和第三步的数据采集后，你需要使用 `PowerRLS/k1k2/fit_k1k2.py` 脚本来计算 $k_1$ 和 $k_2$。

**1. 导出数据**
使用 Ozone 等调试工具，将你采集到的数据导出为 CSV 文件。确保导出的 CSV 文件中包含以下三列数据：
- 电机实际扭矩反馈 (或电流反馈)
- 电机实际角速度反馈
- 超级电容反馈的底盘总功率

**2. 配置 Python 脚本**
打开 `PowerRLS/k1k2/fit_k1k2.py` 文件，找到开头的 **用户配置区**，根据你的实际情况修改以下变量：

```python
# ==========================================================
#                      用户配置区
# ==========================================================

# [关键] 已知的 K3 (静态功耗)
# 填入你在“第一步”中测量得到的静态功耗平均值
K3_VALUE = 0.705

# [关键] 已知的 K0 (扭矩常数)
# 1. 如果你的 CSV_COL_TORQUE 列已经是真正的扭矩 (Nm)，请填 1.0
# 2. 如果你的 CSV_COL_TORQUE 列其实是电流 (A)，请填你的电机 K0 (例如 0.18 或 0.3)
K0_VALUE = 1.0

# [关键] CSV 列名配置
# 注意：这里必须与你从 Ozone 导出的 CSV 文件中的表头名称完全一致！(包括括号和空格)
CSV_COL_INPUT_T  = "(Motor_PowerLimitator[0]).getTorqueFeedback" # 扭矩/电流列名
CSV_COL_INPUT_W  = "(Motor_PowerLimitator[0]).getRadFeedback"    # 角速度列名
CSV_COL_POWER    = "((SuperCap_Data).Data).chassisPower"         # 总功率列名

# [关键] 电机数量
# 如果你导出的数据是单个电机运行的数据，这里填 1
# 如果你是 4 个电机同时运行测出的总功率，这里填 4
MOTOR_COUNT = 1
```

**3. 运行脚本**
1. 将你导出的所有 `.csv` 文件放到与 `fit_k1k2.py` 相同的目录下（即 `PowerRLS/k1k2/` 目录）。
2. 确保你的电脑安装了 Python 以及 `pandas` 和 `numpy` 库 (`pip install pandas numpy`)。
3. 在终端中运行该脚本：
   ```bash
   cd PowerRLS/k1k2
   python fit_k1k2.py
   ```
4. 脚本会自动读取目录下所有的 CSV 文件，使用 RLS（递归最小二乘法）算法进行拟合，并在终端输出计算得到的 `k1` 和 `k2` 的值。

**4. 人工校准**
通常脚本计算出的 $k_1$ 比较准确，但预估出的 $k_2$ 理论上会偏低。在实际下地测试时，需要根据机器人的实际表现，手动微调 $k_2$ 的数值，直到功率限制效果达到最佳。

---

## 5. 如何使用该代码（接入指南）

要使该模块正常工作，你需要完善 `APL_PowerLimitator.cpp` 中的任务逻辑，并将其与你的底盘控制代码对接。以下是具体的步骤：

### 步骤 1：初始化电机参数
在系统初始化阶段（或 `PowerLimitator_Init` 中），使用你刚刚测量和计算出的参数，为底盘电机初始化功率模型。

```cpp
// 示例：初始化 0 号电机的参数
PowerLimitator_Motor_Status_Init(0,                 // 电机编号 (0~3)
                                 0.005f,            // k0 (查手册)
                                 0.055f,            // k1 (脚本计算得出)
                                 7.14f,             // k2 (脚本计算得出，可能需微调)
                                 0.57f,             // k3 (实际测量得出)
                                 0.005f             // torqueConst (力矩常数)
                                 );
```

### 步骤 2：完善 APL 层任务逻辑 (`PowerLimitator_Task`)
在 `APL_PowerLimitator.cpp` 的 `while(1)` 循环中，按顺序执行以下操作：

#### 2.1 获取外部数据并更新最大功率
从裁判系统获取底盘功率上限，从超级电容获取剩余能量，并结合能量环 PID 计算出当前允许的最大功率。
```cpp
// 伪代码示例
float referee_power_limit = 裁判系统获取的功率上限();
float cap_energy = SuperCap_Data.Data.capEnergy; // 从超电模块获取

// 更新超电容量并计算能量环 PID
PowerLimitator_Energy_Capacity_Set(cap_energy);
PowerLimitator_Energy_PID_Update();

// 计算最终允许的最大功率 (裁判系统限制 + 能量环 PID 调整量)
PowerLimitator_Chassis.maxPower = referee_power_limit + PowerLimitator_Energy_PID.out;
```

#### 2.2 更新电机反馈数据
将底盘电机最新的传感器数据（转速、扭矩）以及底盘速度环 PID 计算出的期望电流传入功率模型。
```cpp
for (int i = 0; i < PowerLimitator_Motor_Number; i++) {
    float current_rpm = 获取电机实际转速(i);
    float current_torque = 获取电机实际扭矩(i); // 通常由实际电流转换而来
    float pid_target_current = 获取底盘PID计算出的期望电流(i);
    
    PowerLimitator_Motor_Status_Update(i,
                                       rpm2rad(current_rpm), // 转换为角速度
                                       current_torque,
                                       pid_target_current);
}
```

#### 2.3 执行核心算法
依次调用 HDL 层的更新函数，计算衰减电流。
```cpp
/* 更新电机数据 (包含低通滤波等) */
PowerLimitator_Motor_Update();

/* 更新最大功率输出及预估功率 */
PowerLimitator_Status_Update(PowerLimitator_Chassis.maxPower);

/* 计算衰减电流并得出最终输出电流 */
PowerLimitator_getDecayCurrent();
```

#### 2.4 将最终电流发送给电机
经过上述计算后，`Motor_PowerLimitator[i].finalcurrent` 中存储的就是经过功率限制后的安全电流值。你需要将这个值发送给电机驱动器（如通过 CAN 总线）。
```cpp
int16_t motor_currents[4];
for (int i = 0; i < PowerLimitator_Motor_Number; i++) {
    // 将浮点电流转换为电机驱动器需要的整型格式
    motor_currents[i] = (int16_t)Motor_PowerLimitator[i].finalcurrent;
}
// 调用 CAN 发送函数将 motor_currents 发送给底盘电机
CAN_Send_Chassis_Current(motor_currents);
```

### 步骤 3：参数整定与优化 (RLS)
代码中预留了 RLS（递归最小二乘法）更新电机参数的接口。在实际运行中，电机的 $k_0 \sim k_3$ 参数可能会随温度、磨损等因素发生变化。你可以引入 RLS 算法，在机器人运行过程中实时辨识并更新这些参数，以提高功率预测的准确性。

---

## 5. 完整执行流程与核心变量/函数总结

为了更清晰地理解整个功率控制模块的运作，以下是从系统初始化到完成一次电流衰减计算的完整流程，以及其中涉及的核心变量和函数：

### 阶段一：系统初始化 (Initialization)
在机器人启动时，需要初始化功率模型参数和能量环 PID。
- **核心函数**：
  - `PowerLimitator_Init()`：初始化低通滤波器和能量环 PID 控制器。
  - `PowerLimitator_Motor_Status_Init()`：为每个电机初始化 $k_0, k_1, k_2, k_3$ 和 `torqueConst`。
- **核心变量**：
  - `Motor_PowerLimitator[i].k0 ~ k3`：存储电机的功率模型系数。
  - `PowerLimitator_Energy_PID`：能量环 PID 控制器实例。

### 阶段二：数据采集与更新 (Data Acquisition & Update)
在 `PowerLimitator_Task` 任务的每次循环中，首先需要收集外部数据并更新到模型中。
- **核心函数**：
  - `PowerLimitator_Energy_Capacity_Set(capacity)`：将超级电容的剩余能量传入系统。
  - `PowerLimitator_Energy_PID_Update()`：执行能量环 PID 计算，输出功率调整量。
  - `PowerLimitator_Motor_Status_Update()`：将电机的实际转速、实际扭矩和 PID 期望电流更新到模型中。
- **核心变量**：
  - `PowerLimitator_Chassis.maxPower`：当前允许的最大底盘功率（裁判系统限制 + 能量环 PID 调整量）。
  - `Motor_PowerLimitator[i].getRadFeedback`：电机实际角速度。
  - `Motor_PowerLimitator[i].getTorqueFeedback`：电机实际扭矩。
  - `Motor_PowerLimitator[i].pids_out`：底盘速度环 PID 计算出的期望电流。

### 阶段三：功率预测与状态更新 (Power Prediction & Status Update)
基于最新的电机数据，计算当前的预估功率和即将产生的命令功率。
- **核心函数**：
  - `PowerLimitator_Motor_Update()`：对电机扭矩进行低通滤波处理，平滑数据。
  - `PowerLimitator_Status_Update(updatedMaxPower)`：遍历所有电机，利用功率模型公式计算总预估功率和总命令功率。
- **核心变量**：
  - `PowerLimitator_Chassis.estimatedPower`：基于当前实际状态计算出的总消耗功率。
  - `PowerLimitator_Chassis.commandPower`：基于期望电流计算出的即将消耗的总功率。

### 阶段四：电流衰减计算 (Decay Current Calculation)
判断命令功率是否超限，若超限则计算衰减电流。
- **核心函数**：
  - `PowerLimitator_getDecayCurrent()`：核心算法函数。判断 `commandPower > maxPower`，若成立，则根据各电机的转速权重，求解一元二次方程，计算出每个电机需要衰减的电流。
- **核心变量**：
  - `Motor_PowerLimitator[i].decayCurrent`：计算出的需要削减的电流值。
  - `Motor_PowerLimitator[i].finalcurrent`：最终安全输出电流（`pids_out - decayCurrent`）。

### 阶段五：输出执行 (Execution)
将计算出的安全电流发送给电机驱动器。
- **操作**：读取 `Motor_PowerLimitator[i].finalcurrent`，将其转换为电机驱动器所需的格式（如 `int16_t`），并通过 CAN 总线发送。

**一句话总结整个闭环**：
初始化参数 $\rightarrow$ 采集实际状态与期望电流 $\rightarrow$ 预测总功率 $\rightarrow$ 若超限则计算衰减量 $\rightarrow$ 输出安全电流 $\rightarrow$ 循环往复。
