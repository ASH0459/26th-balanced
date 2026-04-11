# Chassis State Machine

## 1. 总览

当前底盘内部状态机统一为 7 个主状态：

- `CHASSIS_STOP`
- `CHASSIS_FLIP`
- `CHASSIS_INIT`
- `CHASSIS_NORMAL`
- `CHASSIS_LEG_1`
- `CHASSIS_LEG_2`
- `CHASSIS_JUMP`

这套状态机的目标是把原先混在一起的：

- 上层 `mode_byte` 请求
- 底盘业务状态
- 机体翻倒/站稳姿态
- 初始化子阶段
- 跳跃子阶段

拆成职责清晰的几层：

- 上层请求：`NO_FORCE / NORMAL / STEP_1 / STEP_2 / JUMP`
- 内部主状态：`STOP / FLIP / INIT / NORMAL / LEG_1 / LEG_2 / JUMP`
- 姿态状态：`CHASSIS_POSTURE_UP / CHASSIS_POSTURE_DOWN`
- INIT 子阶段：`FOLD / RETRACT / STAND / DONE`
- JUMP 子阶段：`PRELOAD / TAKEOFF / AIRBORNE / LAND / DONE`


## 2. 代码位置

核心实现分布如下：

- 状态、姿态、阶段枚举与常量：
  [App_Chassis_Task.h](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Task.h)
- 主状态切换、上层请求解析、目标速度/腿长生成：
  [App_Chassis_Behaviour.cpp](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Behaviour.cpp)
- 状态切换后的参数重置、姿态更新、INIT/FLIP/JUMP 控制逻辑：
  [App_Chassis_Task.cpp](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Task.cpp)


## 3. 上层输入如何映射到内部状态

上层通过 CAN 下发 `chassis_behaviour_mode`，定义在：

- [Dev_Can_Receive.h](/C:/Code/RM/RM/26th_balanced_code/Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.h)

当前映射关系如下：

- `CHASSIS_MODE_NO_FORCE` -> 卸力请求
- `CHASSIS_MODE_NORMAL` -> 正常运动请求
- `CHASSIS_MODE_STEP_1` -> 一级腿长事件
- `CHASSIS_MODE_STEP_2` -> 二级腿长事件
- `CHASSIS_MODE_JUMP` -> 跳跃事件

注意：

- `STEP_1`、`STEP_2`、`JUMP` 都按上升沿处理
- 上升沿的判断基于 `last_request_mode`
- 只有 `NORMAL` 下才允许响应 `STEP_1`、`STEP_2`、`JUMP`


## 4. 主状态定义

### 4.1 `CHASSIS_STOP`

语义：

- 完全卸力
- 关节和轮子输出清零
- 是所有带动力状态的统一入口

控制特征：

- 行为层输出速度和角速度为 0
- 任务层直接 `chassis_zero_output()`

进入条件：

- 任意状态收到 `NO_FORCE`

退出条件：

- 仅 `NORMAL` 请求可以触发离开
- 离开时不会直接进 `NORMAL`，而是先进入前置链路


### 4.2 `CHASSIS_FLIP`

语义：

- 机体翻倒后的翻身自救

控制特征：

- 只运行翻身关节控制
- 轮子输出为 0
- 不允许正常移动

进入条件：

- `STOP` 下收到 `NORMAL`
- 且当前姿态是 `CHASSIS_POSTURE_DOWN`

退出条件：

- 姿态恢复为 `CHASSIS_POSTURE_UP`
- 下一步切入 `CHASSIS_INIT`


### 4.3 `CHASSIS_INIT`

语义：

- 翻正后或站立前的初始化起立过程

控制特征：

- 不接受正常平移命令
- 负责收腿、LQR 接管、恢复正常腿长

进入条件：

- `STOP` 下收到 `NORMAL` 且姿态已经直立
- 或 `FLIP` 完成后

退出条件：

- `init_phase == CHASSIS_INIT_DONE`
- 然后跳到 `pending_state`

当前 `pending_state` 默认只会落到：

- `CHASSIS_NORMAL`


### 4.4 `CHASSIS_NORMAL`

语义：

- 正常平衡移动状态

控制特征：

- 使用现有平衡/LQR/yaw 主链路
- 目标腿长为 `CHASSIS_NORMAL_LEG_TARGET`

进入条件：

- `INIT` 完成
- 或 `LEG_1/LEG_2/JUMP` 回退

可转移到：

- `LEG_1`
- `LEG_2`
- `JUMP`
- `STOP`


### 4.5 `CHASSIS_LEG_1`

语义：

- 一级腿长状态

控制特征：

- 与 `NORMAL` 的平衡和运动结构相同
- 只把目标腿长改成 `CHASSIS_LEG_1_TARGET`

进入条件：

- `NORMAL` 下收到 `STEP_1` 上升沿
- 或 `LEG_2` 下收到 `STEP_1` 上升沿

退出条件：

- 自身状态下再次收到 `STEP_1` 上升沿，回 `NORMAL`
- 收到 `STEP_2` 上升沿，切到 `LEG_2`
- 收到 `NO_FORCE`，回 `STOP`


### 4.6 `CHASSIS_LEG_2`

语义：

- 二级腿长状态

控制特征：

- 与 `NORMAL` 的平衡和运动结构相同
- 只把目标腿长改成 `CHASSIS_LEG_2_TARGET`

进入条件：

- `NORMAL` 下收到 `STEP_2` 上升沿
- 或 `LEG_1` 下收到 `STEP_2` 上升沿

退出条件：

- 自身状态下再次收到 `STEP_2` 上升沿，回 `NORMAL`
- 收到 `STEP_1` 上升沿，切到 `LEG_1`
- 收到 `NO_FORCE`，回 `STOP`


### 4.7 `CHASSIS_JUMP`

语义：

- 单次跳跃动作

控制特征：

- 进入后由 `jump_phase` 接管过程
- 不是上升沿退出，而是阶段完成后自动退出

进入条件：

- `NORMAL` 下收到 `JUMP` 上升沿

退出条件：

- `jump_phase == CHASSIS_JUMP_DONE`
- 自动回 `NORMAL`


## 5. 主状态流转

### 5.1 从 `STOP` 到正常工作

这是最核心的一条链路：

```text
STOP
  ├─ 姿态翻倒 -> FLIP -> INIT -> NORMAL
  └─ 姿态正常 -> INIT -> NORMAL
```

这里的原则是：

- 不允许 `STOP -> NORMAL` 直接硬切
- 必须先完成翻身或起立过程


### 5.2 腿长切换

腿长状态只允许从 `NORMAL` 派生：

```text
NORMAL --STEP_1上升沿--> LEG_1
LEG_1  --STEP_1上升沿--> NORMAL

NORMAL --STEP_2上升沿--> LEG_2
LEG_2  --STEP_2上升沿--> NORMAL

LEG_1  --STEP_2上升沿--> LEG_2
LEG_2  --STEP_1上升沿--> LEG_1
```

重要约束：

- `LEG_1` 和 `LEG_2` 不会改变运动模式
- 只改腿长目标值


### 5.3 跳跃切换

```text
NORMAL --JUMP上升沿--> JUMP -> NORMAL
```

重要约束：

- `LEG_1`、`LEG_2`、`STOP`、`FLIP`、`INIT` 下收到 `JUMP` 都忽略
- `JUMP` 结束后自动回 `NORMAL`


## 6. 姿态判定逻辑

当前姿态判定在：

- [App_Chassis_Task.cpp](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Task.cpp)

逻辑如下：

```cpp
fabs(chassis_pitch) <= CHASSIS_PITCH_LEVEL_THRESHOLD &&
fabs(chassis_roll) <= CHASSIS_ROLL_LEVEL_THRESHOLD
```

满足时：

- `posture = CHASSIS_POSTURE_UP`

否则：

- `posture = CHASSIS_POSTURE_DOWN`

也就是说现在翻倒判定不再只看 pitch，而是：

- `pitch` 超阈值算翻倒
- `roll` 超阈值也算翻倒


## 7. INIT 子阶段流程

当前 `CHASSIS_INIT` 内部分为 4 段：

- `CHASSIS_INIT_FOLD`
- `CHASSIS_INIT_RETRACT`
- `CHASSIS_INIT_STAND`
- `CHASSIS_INIT_DONE`

流程如下：

```text
INIT_FOLD
  -> 关节转到收腿触地区域
  -> 到位后进入 INIT_RETRACT

INIT_RETRACT
  -> 通过单独的一阶低通参数柔和收腿到最短目标
  -> 左右腿长度都到位后进入 INIT_STAND

INIT_STAND
  -> LQR 接管
  -> 保持 NORMAL 最短腿长
  -> 姿态稳定一段时间后进入 INIT_DONE

INIT_DONE
  -> 主状态机会跳到 pending_state
```

当前相关常量：

- `CHASSIS_INIT_LEG_ANGLE_TARGET_360`
- `CHASSIS_INIT_RETRACT_LEG_TARGET`
- `CHASSIS_INIT_RETRACT_LEG_NUM`
- `CHASSIS_NORMAL_LEG_TARGET`
- `CHASSIS_POSTURE_STABLE_TICKS`


## 8. JUMP 子阶段流程

当前 `CHASSIS_JUMP` 内部保留 5 个枚举阶段，其中 `PRELOAD` 只作为兼容兜底阶段，正常进入跳跃时会直接从 `TAKEOFF` 开始：

- `CHASSIS_JUMP_PRELOAD`
- `CHASSIS_JUMP_TAKEOFF`
- `CHASSIS_JUMP_AIRBORNE`
- `CHASSIS_JUMP_LAND`
- `CHASSIS_JUMP_DONE`

流程如下：

```text
JUMP_PRELOAD
  -> 兼容兜底，下一拍直接进入 TAKEOFF
  -> 正常流程不会主动进入这个阶段

JUMP_TAKEOFF
  -> 目标腿长切到最大
  -> 支持力附加 takeoff bonus
  -> 双腿离地或双腿伸到最长阈值后进入 AIRBORNE

JUMP_AIRBORNE
  -> 目标腿长切到 0.20m
  -> 空中保留 0.03m 压缩缓冲量
  -> 双腿确认触地后进入 LAND

JUMP_LAND
  -> 继续维持 0.20m 缓冲腿长
  -> 触地稳定一段时间且腿长接近 0.20m 后进入 DONE

JUMP_DONE
  -> 主状态自动退回 NORMAL
  -> NORMAL 重新接管后恢复 0.17m 正常最短腿长
```

离地/落地判据复用了原有：

- `chassis_off_ground_detection`

没有新增传感器链路。

当前跳跃关键取舍：

- `PRELOAD -> TAKEOFF` 是兼容兜底，下一拍直接切换
- `TAKEOFF -> AIRBORNE` 优先看是否双腿离地，同时保留“伸到最大腿长阈值”的失败兜底
- `AIRBORNE -> LAND` 看是否双腿重新触地
- `LAND -> DONE` 看是否触地稳定且腿长接近 0.20m


## 9. 每个状态的控制输出特征

### `STOP`

- 关节电流清零
- 轮子电流清零

### `FLIP`

- 只输出翻身关节力矩
- 轮子不参与

### `INIT`

- 早期阶段停止正常轮子/LQR移动
- 中后期恢复支撑力和 LQR 接管

### `NORMAL`

- 正常速度控制
- 正常 yaw 跟随
- 正常腿长目标

### `LEG_1`

- 和 `NORMAL` 相同
- 仅腿长目标改为 `CHASSIS_LEG_1_TARGET`
- 腿长目标通过 `CHASSIS_LEG_STEP_RAMP_SPEED` 斜坡限速

### `LEG_2`

- 和 `NORMAL` 相同
- 仅腿长目标改为 `CHASSIS_LEG_2_TARGET`
- 腿长目标通过 `CHASSIS_LEG_STEP_RAMP_SPEED` 斜坡限速

### `JUMP`

- 平移与 yaw 目标清零
- 腿长由 `jump_phase` 控制
- 起跳阶段额外增加支持力偏置
- `TAKEOFF` 直接给最大腿长，不加斜坡
- `AIRBORNE/LAND` 通过 `CHASSIS_JUMP_AIRBORNE_LEG_RAMP_SPEED` 快速收向 0.20m 缓冲腿长


## 10. 我如何修改现有状态机

下面按最常见改法拆开说明。

### 10.1 只想改腿长数值

只改头文件常量即可：

- [App_Chassis_Task.h](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Task.h)

重点看这些值：

- `CHASSIS_NORMAL_LEG_TARGET`
- `CHASSIS_LEG_1_TARGET`
- `CHASSIS_LEG_2_TARGET`
- `CHASSIS_LEG_STEP_RAMP_SPEED`
- `CHASSIS_JUMP_TAKEOFF_TARGET`
- `CHASSIS_JUMP_AIRBORNE_TARGET`
- `CHASSIS_JUMP_LAND_TARGET`
- `CHASSIS_JUMP_AIRBORNE_LEG_RAMP_SPEED`

适用场景：

- 一级腿长太低/太高
- 二级腿长不够长
- 跳跃预压缩不够


### 10.2 只想改翻倒判定

改这里：

- `CHASSIS_PITCH_LEVEL_THRESHOLD`
- `CHASSIS_ROLL_LEVEL_THRESHOLD`

如果你要把翻倒判定改成别的形式，比如：

- `pitch` 和 `roll` 权重不同
- 合成角度判定
- 加角速度参与

就改 `chassis_feedback_update()` 里 `posture` 的赋值逻辑。


### 10.3 只想改 `STEP_1 / STEP_2` 的切换规则

改这里：

- [App_Chassis_Behaviour.cpp](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Behaviour.cpp)

重点函数：

- `Chassis_Behaviour_Mode_Set()`

现在的规则是：

- `NORMAL -> LEG_1`
- `LEG_1 -> NORMAL`
- `NORMAL -> LEG_2`
- `LEG_2 -> NORMAL`
- `LEG_1 <-> LEG_2`

如果你想改成：

- `STEP_1` 只能进不能退
- `STEP_2` 优先级更高
- `LEG_1/LEG_2` 也允许触发跳跃

都在这个函数里改。


### 10.4 只想改某个状态下的目标量

改这里：

- `chassis_behaviour_control_set()`
- `chassis_follow_control()`
- `chassis_init_control()`
- `chassis_jump_control()`

这里决定的是：

- `vx_set`
- `yaw_set`
- `d_yaw_set`
- `leg_set`

也就是“状态对应的目标命令”。

如果只是要让某状态：

- 不允许移动
- 限速
- 固定腿长
- 固定 yaw

优先改行为层，不要先改 LQR 主链路。


### 10.5 想新增一个新的主状态

建议按下面步骤做。

#### 第一步：在头文件里加状态枚举

改：

- [App_Chassis_Task.h](/C:/Code/RM/RM/26th_balanced_code/Users/Application/APP_Chassis/App_Chassis_Task.h)

在 `Chassis_State_e` 里加入新状态，例如：

```cpp
CHASSIS_CROUCH,
```

如果它需要自己的子阶段，也同时新增阶段枚举和结构体字段。


#### 第二步：定义它从什么事件进入

改：

- `Chassis_Behaviour_Mode_Set()`

明确写清楚：

- 允许从哪些状态进入
- 进入条件是不是上升沿
- 是否需要前置链路
- 退出回哪个状态

不要把规则分散写到多个函数里。


#### 第三步：定义该状态输出什么目标量

改：

- `chassis_behaviour_control_set()`

至少要明确：

- 目标腿长
- 是否允许速度输入
- 是否允许 yaw 输入


#### 第四步：定义该状态在任务层如何控制

改：

- `chassis_control_loop()`
- 需要的话新增专属函数，比如：
  `chassis_crouch_control()`

如果该状态只是“改目标腿长”，不要新增单独控制分支，直接复用 `NORMAL` 结构更稳。


#### 第五步：定义状态切换时要不要重置参数

改：

- `chassis_mode_change_control_transit()`

这里负责处理“刚切进状态时”的一次性动作，例如：

- 清 PID
- 复位 ramp
- 设置腿长滤波初值
- 复位阶段计数器


### 10.6 想新增一个新的跳跃阶段

改动顺序：

1. 在 `Chassis_Jump_Phase_e` 增加新阶段
2. 在 `chassis_update_jump_phase()` 增加状态推进逻辑
3. 在 `chassis_jump_control()` 定义该阶段的腿长目标
4. 如果需要特殊支持力或关节控制，在：
   `chassis_calc_support_force()` 或 `chassis_apply_joint_output()` 增加分支

建议原则：

- “阶段推进”只放在 `chassis_update_jump_phase()`
- “目标腿长/目标命令”只放在 `chassis_jump_control()`
- “电机输出补偿”只放在任务层

这样不会乱。


## 11. 我如何安全修改而不把状态机改乱

建议遵守下面几条。

### 原则 1

主状态跳转只在 `Chassis_Behaviour_Mode_Set()` 做。

不要在别的函数里偷偷改 `state`，否则很难追边沿。

### 原则 2

姿态判定只在 `chassis_feedback_update()` 做。

不要在别的地方临时写 `posture = ...`。

### 原则 3

阶段推进只在对应阶段函数里做：

- INIT 阶段只在 `chassis_init_standup()` 推进
- JUMP 阶段只在 `chassis_update_jump_phase()` 推进

### 原则 4

如果一个状态只是“正常状态的变体”，优先复用 `NORMAL` 主链路。

例如：

- 一级腿长
- 二级腿长
- 小范围站姿变化

这类状态最好只改目标量，不要复制一套 LQR 分支。

### 原则 5

切换时需要清理的一次性变量，统一放进 `chassis_mode_change_control_transit()`。

不要把清理逻辑分散写到多个控制函数里。


## 12. 当前已知的实现取舍

这版实现里有几个明确取舍，后面改时要注意：

- `STOP` 下目前只响应 `NORMAL` 请求，不响应 `STEP_1/STEP_2/JUMP`
- `pending_state` 当前主要用于 `STOP -> INIT -> NORMAL` 这条链
- `LEG_1/LEG_2` 目前只改腿长目标，不锁死速度输入
- `JUMP` 当前是最小可运行版：
  通过腿长目标切换 + 起跳支持力偏置 + 现有离地判据闭环完成
- `JUMP` 是否“更像弹射”还是“更像跃起”，主要靠这几个参数调：
  `CHASSIS_JUMP_TAKEOFF_TARGET`
  `CHASSIS_JUMP_AIRBORNE_TARGET`
  `CHASSIS_JUMP_TAKEOFF_FORCE_BONUS`
  `CHASSIS_JUMP_LAND_TICKS`


## 13. 推荐修改流程

以后你要改状态机，建议固定按这个顺序：

1. 先改枚举和常量
2. 再改 `Chassis_Behaviour_Mode_Set()` 的转移规则
3. 再改 `chassis_behaviour_control_set()` 的目标量输出
4. 最后改任务层专属控制逻辑
5. 每次改完至少重新编译一次

如果你愿意，下一步我可以继续帮你把这份文档补成：

- Mermaid 状态图版本
- “加新状态模板”
- “调参建议表”
