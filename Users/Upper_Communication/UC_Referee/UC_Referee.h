#ifndef INFANTRY_ROBOT_UC_REFEREE_H
#define INFANTRY_ROBOT_UC_REFEREE_H

#include "main.h"
#include "UC_Protocol.h"

/*
 * 裁判系统协议数据结构定义。
 *
 * 设计目标:
 * 1. 每个 ext_xxx_t 都与一个标准 cmd_id 对应
 * 2. 结构体布局与协议文档字节布局严格一致
 * 3. 注释尽量写到"不翻 PDF 也能知道怎么用"
 */

/* 机器人 ID 与裁判系统附录编号保持一致。 */
typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
} robot_id_t;

/* 比赛阶段枚举, 对应 0x0001 中 game_progress 的数值定义。 */
typedef enum
{
    PROGRESS_UNSTART = 0,      /* 未开始比赛 */
    PROGRESS_PREPARE = 1,      /* 准备阶段 */
    PROGRESS_SELFCHECK = 2,    /* 十五秒裁判系统自检阶段 */
    PROGRESS_5sCOUNTDOWN = 3,  /* 五秒倒计时阶段 */
    PROGRESS_BATTLE = 4,       /* 比赛中 */
    PROGRESS_CALCULATING = 5,  /* 比赛结算中 */
} game_progress_t;

#if defined(__GNUC__) || defined(__clang__)
#define UC_REF_PACKED_PREFIX
#define UC_REF_PACKED_SUFFIX __attribute__((packed))
#else
#define UC_REF_PACKED_PREFIX __packed
#define UC_REF_PACKED_SUFFIX
#endif

/* 0x0001 比赛状态数据, 固定 1Hz 下发给全体机器人。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t game_type : 4;      /* bit 0-3: 比赛类型, 1超级对抗赛 2高校单项赛 3ICRA AI挑战赛 4高校联盟赛3V3 5高校联盟赛步兵对抗 */
    uint8_t game_progress : 4;  /* bit 4-7: 当前比赛阶段, 0未开始 1准备阶段 2十五秒裁判系统自检 3五秒倒计时 4比赛中 5比赛结算中 */
    uint16_t stage_remain_time; /* 当前阶段剩余时间, 单位 s */
    uint64_t SyncTimeStamp;     /* UNIX 时间戳, 连接到裁判系统 NTP 服务器后生效 */
} ext_game_status_t;

/* 0x0002 比赛结果数据, 比赛结束时触发发送。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t winner; /* 比赛结果, 0平局 1红方胜利 2蓝方胜利 */
} ext_game_result_t;

/* 0x0003 机器人血量数据。
 * 2026 协议按"己方视角"给出本方机器人、前哨站和基地血量。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint16_t ally_1_robot_HP; /* 己方 1 号英雄机器人血量 */
    uint16_t ally_2_robot_HP; /* 己方 2 号工程机器人血量 */
    uint16_t ally_3_robot_HP; /* 己方 3 号步兵机器人血量 */
    uint16_t ally_4_robot_HP; /* 己方 4 号步兵机器人血量 */
    uint16_t reserved;        /* 保留位 */
    uint16_t ally_7_robot_HP; /* 己方 7 号哨兵机器人血量 */
    uint16_t ally_outpost_HP; /* 己方前哨站血量 */
    uint16_t ally_base_HP;    /* 己方基地血量 */
} ext_game_robot_HP_t;

/* 0x0101 场地事件数据。
 * 这是一个 32 bit 压缩状态字, 业务读取时通常要按位拆解。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    /* bit 0: 己方补给区占领状态 */
    /* bit 1: 保留 */
    /* bit 2: 己方 RMUL 补给区占领状态 */
    /* bit 3-4: 己方小能量机关状态, 0未激活 1已激活 2正在激活 */
    /* bit 5-6: 己方大能量机关状态, 0未激活 1已激活 2正在激活 */
    /* bit 7-8: 己方中央高地占领状态, 1己方占领 2对方占领 */
    /* bit 9-10: 己方梯形高地占领状态 */
    /* bit 11-19: 对方飞镖最近一次命中己方前哨站或基地的时间, 0-420 */
    /* bit 20-22: 对方飞镖最近一次命中的目标类型 */
    /* bit 23-24: 中心增益点占领状态, 仅 RMUL 适用 */
    /* bit 25-26: 己方堡垒增益点占领状态 */
    /* bit 27-28: 己方前哨站增益点占领状态 */
    /* bit 29: 己方基地增益点占领状态 */
    /* bit 30-31: 保留 */
    uint32_t event_type; /* 场地事件位图 */
} ext_event_data_t;

/* 0x0104 裁判警告数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t level;        /* 己方最后一次受到判罚的等级, 1双方黄牌 2黄牌 3红牌 4判负 */
    uint8_t foul_robot_id;/* 己方最后一次受到判罚的违规机器人 ID, 判负和双方黄牌时为 0 */
    uint8_t count;        /* 对应判罚等级下, 该违规机器人累计违规次数 */
} ext_referee_warning_t;

/* 0x0105 飞镖发射相关数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t dart_remaining_time; /* 己方飞镖发射剩余时间, 单位 s */
    /* bit 0-2: 最近一次己方飞镖击中的目标 */
    /* bit 3-5: 对方最近被击中的目标累计被击中次数, 至多为 4 */
    /* bit 6-8: 当前飞镖选定的击打目标, 0未选定或前哨站 1基地固定 2基地随机固定 3基地随机移动 4基地末端移动 */
    /* bit 9-15: 保留 */
    uint16_t dart_info; /* 飞镖命中/选靶信息 */
} ext_dart_remaining_time_t;

/* 0x0201 机器人性能体系数据。
 * 包含机器人等级、当前血量、热量参数和底盘功率上限。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t robot_id;                      /* 本机器人 ID */
    uint8_t robot_level;                   /* 本机器人等级 */
    uint16_t remain_HP;                    /* 本机器人当前血量 */
    uint16_t max_HP;                       /* 本机器人血量上限 */
    uint16_t shooter_barrel_cooling_value; /* 机器人射击热量每秒冷却值 */
    uint16_t shooter_barrel_heat_limit;    /* 机器人射击热量上限 */
    uint16_t chassis_power_limit;          /* 机器人底盘功率上限 */
    uint8_t mains_power_gimbal_output : 1; /* bit 0: gimbal 口输出状态, 0无输出 1输出 24V */
    uint8_t mains_power_chassis_output : 1;/* bit 1: chassis 口输出状态, 0无输出 1输出 24V */
    uint8_t mains_power_shooter_output : 1;/* bit 2: shooter 口输出状态, 0无输出 1输出 24V */
} ext_game_robot_status_t;

/* 0x0202 实时缓冲能量和射击热量数据。
 * 注意 2026 协议中不再包含实时底盘功率、电压和电流。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint16_t reserved_1;            /* 保留位 */
    uint16_t reserved_2;            /* 保留位 */
    float reserved_3;               /* 保留位 */
    uint16_t buffer_energy;         /* 缓冲能量, 单位 J */
    uint16_t shooter_17mm_barrel_heat; /* 17mm 发射机构射击热量 */
    uint16_t shooter_42mm_barrel_heat; /* 42mm 发射机构射击热量 */
} ext_power_heat_data_t;

/* 0x0203 本机器人位置数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    float x;     /* 本机器人位置 x 坐标, 单位 m */
    float y;     /* 本机器人位置 y 坐标, 单位 m */
    float angle; /* 本机器人测速模块朝向, 单位 deg, 正北为 0 度 */
} ext_game_robot_pos_t;

/* 0x0204 机器人增益和能量反馈数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t recovery_buff;     /* 机器人回血增益百分比, 10 表示每秒恢复血量上限的 10% */
    uint16_t cooling_buff;     /* 机器人射击热量冷却增益直接值, x 表示热量冷却增加 x/s */
    uint8_t defence_buff;      /* 机器人防御增益百分比 */
    uint8_t vulnerability_buff;/* 机器人负防御增益百分比, 30 表示 -30% 防御增益 */
    uint16_t attack_buff;      /* 机器人攻击增益百分比 */
    /* bit 0-6: 剩余能量比例标志位, 分别表示 >=125%、>=100%、>=50%、>=30%、>=15%、>=5%、>=1% */
    uint8_t remaining_energy;  /* 剩余能量值反馈位图 */
} ext_buff_t;

/* 0x0206 伤害状态数据。
 * 这是本地裁判模块的即时判定, 最终是否有效仍以服务器结算为准。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t armor_type : 4; /* bit 0-3: 受击装甲模块或测速模块 ID, 其他原因扣血时为 0 */
    uint8_t hurt_type : 4;  /* bit 4-7: 扣血原因, 0弹丸攻击 1装甲模块/超电管理模块离线 5装甲模块撞击 */
} ext_robot_hurt_t;

/* 0x0207 实时射击数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t bullet_type; /* 弹丸类型, 1 表示 17mm, 2 表示 42mm */
    uint8_t shooter_id;  /* 发射机构 ID, 1 表示 17mm 发射机构, 3 表示 42mm 发射机构 */
    uint8_t bullet_freq; /* 弹丸射频, 单位 Hz */
    float bullet_speed;  /* 弹丸初速度, 单位 m/s */
} ext_shoot_data_t;

/* 0x0208 允许发弹量和金币数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint16_t projectile_allowance_17mm;     /* 机器人自身拥有的 17mm 弹丸允许发弹量 */
    uint16_t projectile_allowance_42mm;     /* 42mm 弹丸允许发弹量 */
    uint16_t remaining_gold_coin;           /* 剩余金币数量 */
    uint16_t projectile_allowance_fortress; /* 堡垒增益点提供的储备 17mm 弹丸允许发弹量 */
} ext_bullet_remaining_t;

/* 0x0209 RFID 状态数据。
 * 位图较长, 因此协议拆成 4 byte + 1 byte 两段存储。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    /* bit 0-31: RFID 状态位图, 依次覆盖基地/高地/飞坡/公路/堡垒/前哨站/补给区/装配点/中心增益点/隧道等增益点 */
    uint32_t rfid_status; /* 第 1 段 RFID 状态位图 */
    /* bit 0-5: 对方隧道相关增益点 RFID 检测状态 */
    uint8_t rfid_status_2; /* 第 2 段 RFID 状态位图 */
} ext_rfid_status_t;

/* 0x020A 飞镖选手端指令数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t dart_launch_opening_status; /* 当前飞镖发射站状态, 0已开启 1关闭 2正在开启或关闭中 */
    uint8_t reserved;                   /* 保留位 */
    uint16_t target_change_time;        /* 切换击打目标时的比赛剩余时间, 单位 s */
    uint16_t latest_launch_cmd_time;    /* 最近一次确认发射指令时的比赛剩余时间, 单位 s */
} ext_dart_client_cmd_t;

/* 0x020B 地面机器人位置数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    float hero_x;       /* 己方英雄机器人 x 坐标, 单位 m */
    float hero_y;       /* 己方英雄机器人 y 坐标, 单位 m */
    float engineer_x;   /* 己方工程机器人 x 坐标, 单位 m */
    float engineer_y;   /* 己方工程机器人 y 坐标, 单位 m */
    float standard_3_x; /* 己方 3 号步兵机器人 x 坐标, 单位 m */
    float standard_3_y; /* 己方 3 号步兵机器人 y 坐标, 单位 m */
    float standard_4_x; /* 己方 4 号步兵机器人 x 坐标, 单位 m */
    float standard_4_y; /* 己方 4 号步兵机器人 y 坐标, 单位 m */
    float reserved_1;   /* 保留位 */
    float reserved_2;   /* 保留位 */
} ext_ground_robot_position_t;

/* 0x020C 雷达标记进度数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    /* bit 0-5: 对方 1/2/3/4/6/7 号机器人易伤或特殊标识情况 */
    /* bit 6-11: 己方 1/2/3/4/6/7 号机器人特殊标识情况 */
    /* bit 12-15: 保留 */
    uint16_t mark_progress; /* 雷达标记/特殊标识状态位图 */
} ext_radar_mark_data_t;

/* 0x020D 哨兵自主决策同步数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    /* bit 0-10: 哨兵成功兑换的允许发弹量 */
    /* bit 11-14: 哨兵成功远程兑换允许发弹量的次数 */
    /* bit 15-18: 哨兵成功远程兑换血量的次数 */
    /* bit 19: 是否可以确认免费复活 */
    /* bit 20: 是否可以兑换立即复活 */
    /* bit 21-30: 当前兑换立即复活所需金币数 */
    /* bit 31: 保留 */
    uint32_t sentry_info; /* 哨兵兑换/复活相关信息 */
    /* bit 0: 哨兵当前是否处于脱战状态 */
    /* bit 1-11: 队伍 17mm 允许发弹量的剩余可兑换数 */
    /* bit 12-13: 哨兵当前姿态, 1进攻 2防御 3移动 */
    /* bit 14: 己方能量机关是否能够进入正在激活状态 */
    /* bit 15: 保留 */
    uint16_t sentry_info_2; /* 哨兵姿态与队伍可兑换信息 */
} ext_sentry_info_t;

/* 0x020E 雷达自主决策同步数据。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    /* bit 0-1: 雷达是否拥有触发双倍易伤的机会, 至多为 2 */
    /* bit 2: 对方是否正在被触发双倍易伤 */
    /* bit 3-4: 己方加密等级, 开局为 1, 最高为 3 */
    /* bit 5: 当前是否可以修改密钥 */
    /* bit 6-7: 保留 */
    uint8_t radar_info; /* 雷达自主决策信息 */
} ext_radar_info_t;

/* 0x0301 机器人交互数据统一子头, 固定 6 byte。
 * 真正的数据段格式是:
 *   data_cmd_id(2) + sender_id(2) + receiver_id(2) + payload(x)
 *
 * 常见子内容 ID:
 * - 0x0200~0x02FF: 机器人间通信
 * - 0x0100: 删除图层
 * - 0x0101/0x0102/0x0103/0x0104: 绘图
 * - 0x0110: 字符绘制
 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint16_t data_cmd_id; /* 机器人交互数据的子内容 ID */
    uint16_t sender_ID;   /* 发送者 ID, 需与自身机器人 ID 匹配 */
    uint16_t receiver_ID; /* 接收者 ID, 需为规则允许的多机通信接收者 */
} ext_student_interactive_header_data_t;

/* 下列自定义数据结构属于工程私有封装, 不是裁判系统标准数据段。 */
typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef UC_REF_PACKED_PREFIX struct UC_REF_PACKED_SUFFIX
{
    uint8_t data[32];
} ext_download_stream_data_t;

/* 被其他模块直接读取最多的两组全局镜像。 */
extern ext_game_robot_status_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;

#ifdef __cplusplus
extern "C"
{
#endif

    /* 初始化所有裁判数据缓存为 0。 */
    extern void init_referee_struct_data(void);

    /* 解析单个已经通过 CRC 校验的完整协议包。 */
    extern void referee_data_solve(uint8_t *frame);

    /* 兼容历史接口:
     * power 在 2026 常规链路中已无对应字段, 当前固定返回 0;
     * buffer 返回真实缓冲能量。 */
    extern void get_chassis_power_and_buffer(float *power, uint16_t *buffer);

    /* 获取机器人 ID 与对应客户端 ID。 */
    extern void get_robot_id(uint16_t *robot_id, uint16_t *client_id);

    /* 仅获取机器人 ID 的轻量接口。 */
    extern void robot_id(uint16_t *robot_id);

    /* 获取 17mm 热量上限与当前热量。 */
    extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

    /* 获取 42mm 热量上限与当前热量。 */
    extern void get_shoot_heat2_limit_and_heat2(uint16_t *heat2_limit, uint16_t *heat2);

    /* 获取射击热量每秒冷却值。 */
    extern void get_shoot_cooling_value1(uint16_t *cooling_value);

    /* 获取最近一次伤害事件中的装甲 ID 和扣血原因。 */
    extern void get_robot_hurt(uint8_t *armor_id, uint8_t *hurt_type);

    /* 获取底盘功率上限。 */
    extern void get_chassis_power_limit(uint16_t *power_limit);

    /* 获取机器人等级。 */
    extern void get_robot_level(uint8_t *level);

    /* 获取 17mm 允许发弹量。
     * 虽然接口名叫 ammo, 但语义是 allowance 而不是 old remaining bullet。 */
    extern void get_ammo(uint16_t *bullet_remaining);

    /* 获取己方哨兵血量。 */
    extern void get_sentry_HP(uint16_t *sentry_HP);

#ifdef __cplusplus
}
#endif

#undef UC_REF_PACKED_PREFIX
#undef UC_REF_PACKED_SUFFIX

#endif
