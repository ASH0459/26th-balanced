# APP_Chassis

本目录维护底盘控制、双板协议消费、状态机与上台阶流程。

## 1. 模块概览

核心文件：

- `Users/Application/APP_Chassis/App_Chassis_Task.h`
- `Users/Application/APP_Chassis/App_Chassis_Task.cpp`
- `Users/Application/APP_Chassis/App_Chassis_Behaviour.h`
- `Users/Application/APP_Chassis/App_Chassis_Behaviour.cpp`
- `Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.h`

主循环 `Chassis_Task()` 的关键流程：

1. `chassis_set_mode`
2. `chassis_mode_change_control_transit`
3. `chassis_feedback_update`
4. `chassis_set_contorl`
5. `chassis_interference_prevention`
6. `chassis_control_loop`

## 2. `0x302` 云台到底盘协议

协议来源：`Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.h`

CAN ID：`0x302`

字节布局：

- `byte0..1`: `v_set` (`int16`, 大端)
- `byte2`: `auto_aim_state` (`uint8`)
- `byte3`: `chassis_feature_flags` (`uint8` bitmask)
- `byte4`: `mode` (`uint8`)
- `byte5`: `fric_state` (`uint8`)
- `byte6..7`: `turn_set` (`int16`, 大端, `rad * 1000`)

### `auto_aim_state`

- `0`: `CHASSIS_AUTO_AIM_STATE_NO_TARGET`
- `1`: `CHASSIS_AUTO_AIM_STATE_MANUAL_FIRE_TARGET`
- `2`: `CHASSIS_AUTO_AIM_STATE_AUTO_FIRE_TARGET`

### `chassis_feature_flags`

- `bit0 (0x01)`: 小陀螺使能
- `bit1 (0x02)`: 上台阶检测使能
- `bit2 (0x04)`: UI reset 请求
- `bit3 (0x08)`: 台阶次数 (0=1次, 1=2次)
- `bit4~bit7`: 预留，忽略

### `mode`

- `0`: `CHASSIS_MODE_NO_FORCE`
- `1`: `CHASSIS_MODE_RESERVED`
- `2`: `CHASSIS_MODE_NORMAL`
- `3`: `CHASSIS_MODE_JUMP` 保留枚举位
- `4`: `CHASSIS_MODE_STEP_1`
- `5`: `CHASSIS_MODE_STEP_2`

### `fric_state`

- `0`: `FRIC_OFF`
- `1`: `FRIC_ON`
- `2`: `FRIC_ERROR`

### 合法性与安全降级

接收层当前校验：

- `mode` 仅接受 `0..5`
- `auto_aim_state` 仅接受 `0/1/2`
- `fric_state` 仅接受 `0/1/2`
- `feature_flags` 只提取已定义 bit，不做整体非法判断

协议无效时：

- `protocol_valid = 0`
- `v_tmp = 0`
- `yaw_set = chassis_relative_angle`
- `gyro_enable = 0`
- `step_enable = 0`
- `auto_aim_state = NO_TARGET`
- `chassis_behaviour_mode = CHASSIS_MODE_NO_FORCE`

## 3. 行为层语义

行为入口：`chassis_behaviour_control_set()`

- `feature_flags.bit0` 解码为 `gyro_enable`
- `STEP_1 / STEP_2` 仍然只是腿长模式
- 协议 `mode` 只定义到 `CHASSIS_MODE_STEP_2`
- `STEP_UP` 不再依赖 `JUMP` 按键边沿进入

### 腿长模式

- `NORMAL --STEP_1--> LEG_1 --STEP_1--> NORMAL`
- `NORMAL --STEP_2--> LEG_2 --STEP_2--> NORMAL`
- `LEG_1 --STEP_2--> LEG_2`
- `LEG_2 --STEP_1--> LEG_1`

## 4. 上台阶流程

载体状态：`CHASSIS_LEG_1` 或 `CHASSIS_LEG_2`

说明：上台阶不再单独占用 `CHASSIS_STEP_UP` 主状态，而是挂在 `LEG_1 / LEG_2` 下作为 `step_up_phase` 子流程执行。

相位：

- `STEP_UP_EXTEND`
- `STEP_UP_DETECT`
- `STEP_UP_SWING`
- `STEP_UP_HOLD`
- `STEP_UP_RETRACT`
- `STEP_UP_STAND`
- `STEP_UP_DONE`

当前入口规则：

- `step_enable = 0` 时，`LEG_1 / LEG_2` 只是普通伸腿
- `step_enable = 1` 时，只允许在 `LEG_1 / LEG_2` 下检测台阶碰撞阈值
- 从 `LEG_1` 进入时，`step_up_leg_target = CHASSIS_LEG_1_TARGET`
- 从 `LEG_2` 进入时，`step_up_leg_target = CHASSIS_LEG_2_TARGET`
- `step_enable` 只影响“要不要检测撞台阶”，不应直接改变当前腿长和操作体感
- 只有在 `LEG_1 / LEG_2` 下检测到碰撞阈值后，才启动 `step_up_phase`
- `STEP_UP_DONE` 后会回到 `NORMAL`

`STEP_UP_EXTEND` 和 `STEP_UP_DETECT` 都使用 `step_up_leg_target`，不再写死 `CHASSIS_LEG_2_TARGET`。

## 5. UI 同步

UI 相关文件：`Users/Application/App_UI/App_UI_Task.cpp`

- UI reset 读取 `feature_flags.bit2`
- 中间自瞄框直接读取 `auto_aim_state`
- 右侧摩擦轮状态框只显示 `FRIC_OFF / FRIC_ON / FRIC_ERROR`

## 6. 验证

逻辑校验脚本：`tools/chassis_protocol_logic_check.ps1`

执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\chassis_protocol_logic_check.ps1
```

脚本当前检查：

- 控制周期宏一致性
- 小陀螺 ramp 平滑性
- 新 `0x302` 协议静态解析路径
- `STEP_UP` 不再依赖 `jump_edge`
- UI reset 改为读取 `feature_flags.bit2`
- `VT_TOE` 安全停机分支
