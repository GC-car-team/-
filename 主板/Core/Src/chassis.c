/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   底盘模块（麦克纳姆轮 + 4 台 PD42S1 闭环步进）实现
  *
  *  代码结构（按顺序读就能懂）：
  *      1. 宏与小工具（角度归一化、限幅）
  *      2. 内部状态
  *      3. 运动学：Mix（整车→轮子）/ Unmix（轮子→整车）
  *      4. 速度下发（限幅、死区、逐台发）
  *      5. 反馈与里程计（读编码器位置、算车速/航向）
  *      6. 航向保持（可选）
  *      7. 上电自检（走方形）
  *      8. 对外接口（Init/Task/SetVelocity/Stop…）
  *
  *  ⚠️ 两个踩过的坑，这里都处理了：
  *     ① 同一次循环里别连发两条串口指令（底层 2ms 间隔保护会把第二条丢掉）
  *        → 本文件里所有"发指令"都集中在各自节拍里，一次只发一条
  *     ② 停车不要用 0xFC 刹车（会锁刹车态 state=8 且严重发烫）
  *        → 底盘"停"= 速度给 CHASSIS_MIN_RPM（0.1rpm，等效不动但保持使能）
  *          只有急停才用 0xFC，之后必须 Chassis_Resume() 清状态
  ******************************************************************************
  */

#include "chassis.h"
#include "pd42s1.h"
#include "imu.h"
#include <math.h>

#if CHASSIS_ENABLE

/* ==========================================================================
 * 1. 宏与小工具
 * ========================================================================== */
#define CHASSIS_PI          3.14159265f
#define CHASSIS_MM_PER_REV  (CHASSIS_PI * CHASSIS_WHEEL_DIA_MM)     /* 轮子转一圈走多远(mm) */

/* 每台电机给底盘用的配置：电机表索引 + 方向符号（+1/-1） */
typedef struct
{
    uint8_t motor;          /* 对应 config.h 电机表的第几行（0 = 1 号口） */
    float   sign;           /* 麦轮方向符号（±1，装反了改 config.h） */
} chassis_wheel_t;

static const chassis_wheel_t s_wheel[CHASSIS_WHEEL_NUM] = {
    { CHASSIS_MOTOR_FL, CHASSIS_SIGN_FL },      /* 左前 */
    { CHASSIS_MOTOR_FR, CHASSIS_SIGN_FR },      /* 右前 */
    { CHASSIS_MOTOR_RL, CHASSIS_SIGN_RL },      /* 左后 */
    { CHASSIS_MOTOR_RR, CHASSIS_SIGN_RR }       /* 右后 */
};

/* 角度归一到 ±180° */
static float wrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* 限幅 */
static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* ==========================================================================
 * 1.5 通用 PID（航向稳定 / 旋转稳定 / 平移稳定 三个环共用）
 *
 *   特点（都是踩过坑才加的）：
 *     · 误差死区 deadband：|err| 小于它 → 当 0（否则轮子/舵机会一直小幅抖）
 *     · 积分限幅 i_max：抗积分饱和（长时间到不了目标时不会憋爆）
 *     · 输出限幅 out_max：★必加，误差大时防止猛甩
 *     · 微分低通：vdf（0~1，越大越平滑）——编码器/IMU 的 D 项噪声很大
 *   ⚠️ 角度类误差要**调用方先 wrap180()**，这个函数只做纯数值 PID
 * ========================================================================== */
typedef struct
{
    float kp, ki, kd;
    float i_max;        /* 积分限幅（<=0 表示不限） */
    float out_max;      /* 输出限幅（<=0 表示不限） */
    float deadband;     /* 误差死区 */
    float vdf;          /* 微分低通系数 0~1（0.7 左右比较合适） */
    float i;            /* 积分累加 */
    float prev_err;
    float d_state;      /* 微分低通状态 */
    float err;          /* 最近一次（去掉死区后的）误差，给 VOFA 看 */
    float out;          /* 最近一次输出 */
} chassis_pid_t;

static void pid_reset(chassis_pid_t *p)
{
    p->i = 0.0f;
    p->prev_err = 0.0f;
    p->d_state = 0.0f;
    p->err = 0.0f;
    p->out = 0.0f;
}

static float pid_step(chassis_pid_t *p, float err, float dt)
{
    float d_raw, out;

    if (dt <= 0.0f) return p->out;

    /* 死区：小误差当 0（防抖；代价是稳态可能有 ±deadband 的静差） */
    if ((err < p->deadband) && (err > -p->deadband)) err = 0.0f;
    p->err = err;

    /* 积分 + 限幅 */
    p->i += err * dt;
    if (p->i_max > 0.0f) p->i = clampf(p->i, -p->i_max, p->i_max);

    /* 微分 + 低通 */
    d_raw = (err - p->prev_err) / dt;
    p->d_state = p->vdf * p->d_state + (1.0f - p->vdf) * d_raw;
    p->prev_err = err;

    out = p->kp * err + p->ki * p->i + p->kd * p->d_state;
    if (p->out_max > 0.0f) out = clampf(out, -p->out_max, p->out_max);
    p->out = out;
    return out;
}

/* ==========================================================================
 * 2. 内部状态
 * ========================================================================== */
static float    s_vx_cmd, s_vy_cmd, s_wz_cmd;      /* 上层给的目标速度（mm/s, mm/s, 度/s） */
static float    s_rpm_cmd[CHASSIS_WHEEL_NUM];      /* 上一次真正发出去的每轮 rpm */
static float    s_rpm_meas[CHASSIS_WHEEL_NUM];     /* 驱动器回传的每轮 rpm */

static float    s_yaw_deg;                         /* 航向角（度，逆时针为正，上电清零） */
static float    s_yaw_scale = CHASSIS_YAW_SCALE;   /* ★IMU 标度修正系数（K 命令可在线改） */

/* ★零漂估计（非阻塞）：静止时学习漂移率，之后连续补偿 */
static float    s_drift_dps;                       /* 估计出的零漂率（度/秒） */
static float    s_drift_off;                       /* 已累计的补偿量（度） */
static float    s_drift_y_last;                    /* 上一拍原始 yaw */
static uint8_t  s_drift_have;                      /* 有上一拍数据了 */
static float    s_yaw_odom_deg;                    /* 纯轮式积分的航向（给互补滤波用） */
static float    s_dist_mm;                         /* 累计路程 */
static float    s_fwd_mm;                          /* 机体系前向位移 ∫vx（mm，带符号） */
static float    s_side_mm;                         /* 机体系横向位移 ∫vy（mm，与 vy 同号：正 = 左） */

static uint32_t s_last_task_tick;                  /* Chassis_Task 上一次执行的时刻 */
static uint32_t s_fb_tick;                         /* 反馈轮询节拍 */
static uint8_t  s_fb_sel;                          /* 轮到读第几台电机 */
static uint32_t s_refresh_tick;                    /* 定时重发速度用的节拍 */
static uint8_t  s_emergency;                       /* 1 = 已急停（刹车态），需要 Resume 才能再动 */

/* ---- 三个闭环（config.h 5.7 区）---- */
static chassis_pid_t s_pid_hdg;                    /* ① 航向稳定：度误差 → wz 修正(度/s) */
static chassis_pid_t s_pid_turn;                   /* ② 旋转稳定：度误差 → wz(度/s) */
static chassis_pid_t s_pid_side;                   /* ③ 平移稳定：mm 误差 → vy(mm/s) */

static float    s_trim_rpm[CHASSIS_WHEEL_NUM] = { CHASSIS_TRIM_RPM_FL, CHASSIS_TRIM_RPM_FR, CHASSIS_TRIM_RPM_RL, CHASSIS_TRIM_RPM_RR };  /* ★每轮速度系数 */
static float    s_crab_trim = CHASSIS_CRAB_TRIM;   /* ★侧滑补偿系数（vy = vx × 它） */
static float    s_straight_trim = CHASSIS_STRAIGHT_TRIM_DEG;  /* ★走直微调角（度） */
static float    s_hdg_target_deg;                  /* 想保持的航向 */
static float    s_turn_target_deg;                 /* 旋转目标角（IMU 坐标系） */
static float    s_turn_rel_deg;                    /* 相对目标角（IMU 断帧时用指令积分开环） */
static float    s_turn_spun_cmd_deg;               /* 已按指令 wz 积出的相对转角 */
static uint8_t  s_turn_openloop;                   /* 1 = 本段转向改走开环（粘性，中途不切回） */
static uint8_t  s_turn_active;                     /* 1 = 正在转向 */
static uint8_t  s_turn_done;                       /* 1 = 到位并稳住 */
static uint32_t s_turn_settle_tick;                /* 进入到位窗口后开始计时 */

static float    s_strafe_vx;                       /* 平移时的前进速度 */
static float    s_strafe_side_target;              /* 平移目标横向位移（mm） */
static uint8_t  s_strafe_active;
static uint8_t  s_strafe_done;

/* ---- 到位判据也做成"运行时可改"（调参脚本要在线上试不同阈值）---- */
static float    s_turn_arrive_deg;                 /* 旋转到位阈值（度） */
static float    s_turn_settle_ms;                  /* 进阈值后要稳住多久（ms） */
static float    s_side_arrive_mm;                  /* 平移到位阈值（mm） */

static void chassis_pid_init_all(void)
{
    /* ① 航向稳定 */
    s_pid_hdg.kp = CHASSIS_HEADING_KP;
    s_pid_hdg.ki = CHASSIS_HEADING_KI;
    s_pid_hdg.kd = CHASSIS_HEADING_KD;
    s_pid_hdg.i_max = CHASSIS_HDG_I_MAX;
    s_pid_hdg.out_max = CHASSIS_HDG_WZ_MAX;
    s_pid_hdg.deadband = CHASSIS_HDG_DEADBAND_DEG;
    s_pid_hdg.vdf = 0.7f;
    pid_reset(&s_pid_hdg);

    /* ② 旋转稳定 */
    s_pid_turn.kp = CHASSIS_TURN_KP;
    s_pid_turn.ki = CHASSIS_TURN_KI;
    s_pid_turn.kd = CHASSIS_TURN_KD;
    s_pid_turn.i_max = CHASSIS_TURN_I_MAX;
    s_pid_turn.out_max = CHASSIS_TURN_WZ_MAX;
    s_pid_turn.deadband = CHASSIS_TURN_DEADBAND_DEG;
    s_pid_turn.vdf = 0.7f;
    pid_reset(&s_pid_turn);

    /* ③ 平移稳定 */
    s_pid_side.kp = CHASSIS_SIDE_KP;
    s_pid_side.ki = CHASSIS_SIDE_KI;
    s_pid_side.kd = CHASSIS_SIDE_KD;
    s_pid_side.i_max = CHASSIS_SIDE_I_MAX;
    s_pid_side.out_max = CHASSIS_SIDE_VY_MAX;
    s_pid_side.deadband = CHASSIS_SIDE_DEADBAND_MM;
    s_pid_side.vdf = 0.7f;
    pid_reset(&s_pid_side);

    s_hdg_target_deg = CHASSIS_HEADING_TARGET_DEG;
    s_turn_active = 0u;
    s_turn_done = 0u;
    s_strafe_active = 0u;
    s_strafe_done = 0u;
    s_strafe_vx = 0.0f;
    s_strafe_side_target = 0.0f;

    /* 到位判据（可被调参脚本在线上改） */
    s_turn_arrive_deg = CHASSIS_TURN_ARRIVE_DEG;
    s_turn_settle_ms  = (float)CHASSIS_TURN_SETTLE_MS;
    s_side_arrive_mm  = CHASSIS_SIDE_ARRIVE_MM;
}

/* ==========================================================================
 * 3. 运动学
 *
 *   麦轮布局（俯视，X 型辊子）：
 *        FL ── FR            前进 vx：4 轮同向正转
 *        │      │            左移 vy：左前/左后 反转，右前/右后 正转
 *        RL ── RR            左转 wz：左侧两轮反转，右侧两轮正转
 *
 *   k = (轴距 + 轮距) / 2   （mm），旋转时轮子的线速度 = wz_rad * k
 * ========================================================================== */
static float chassis_k_mm(void)
{
    return (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f;
}

void Chassis_Mix(float vx_mmps, float vy_mmps, float wz_dps, float *rpm_out)
{
    float k      = chassis_k_mm();
    float wz_rad = wz_dps * CHASSIS_PI / 180.0f;
    float v_fl, v_fr, v_rl, v_rr;
    float vmax, scale;

#if CHASSIS_ROLLER_X
    float s = 1.0f;                     /* X 型辊子 */
#else
    float s = -1.0f;                    /* O 型辊子：平移那两项符号反过来 */
#endif

    /* 4 个轮子的线速度（mm/s） */
    v_fl = vx_mmps - s * vy_mmps - k * wz_rad;
    v_fr = vx_mmps + s * vy_mmps + k * wz_rad;
    v_rl = vx_mmps + s * vy_mmps - k * wz_rad;
    v_rr = vx_mmps - s * vy_mmps + k * wz_rad;

    /* 线速度 → 电机转速 rpm：rpm = v / (πD) * 60 * 减速比 */
    rpm_out[CHASSIS_WHEEL_FL] = v_fl / CHASSIS_MM_PER_REV * 60.0f * CHASSIS_GEAR_RATIO;
    rpm_out[CHASSIS_WHEEL_FR] = v_fr / CHASSIS_MM_PER_REV * 60.0f * CHASSIS_GEAR_RATIO;
    rpm_out[CHASSIS_WHEEL_RL] = v_rl / CHASSIS_MM_PER_REV * 60.0f * CHASSIS_GEAR_RATIO;
    rpm_out[CHASSIS_WHEEL_RR] = v_rr / CHASSIS_MM_PER_REV * 60.0f * CHASSIS_GEAR_RATIO;

    /* 限幅：任何一个超了最大转速就整体按比例缩（保持运动方向不变） */
    vmax = 0.0f;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        float a = fabsf(rpm_out[i]);
        if (a > vmax) vmax = a;
    }
    if (vmax > CHASSIS_MAX_RPM && vmax > 0.0f)
    {
        scale = CHASSIS_MAX_RPM / vmax;
        for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++) rpm_out[i] *= scale;
    }

    /* 套上每个轮子的方向符号（麦轮朝向/装反了在 config.h 里改） */

    /* ★每轮速度系数（走直补偿）：在方向符号之后乘，只影响大小不影响方向 */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        rpm_out[i] *= s_wheel[i].sign;
        rpm_out[i] *= s_trim_rpm[i];
    }
}

void Chassis_Unmix(const float *rpm_in, float *vx_mmps, float *vy_mmps, float *wz_dps)
{
    float k = chassis_k_mm();
    float v[CHASSIS_WHEEL_NUM];
    float vx, vy, wz_rad;

    /* 先把 rpm 还原成线速度（去掉方向符号） */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        v[i] = rpm_in[i] * s_wheel[i].sign * CHASSIS_MM_PER_REV / 60.0f / CHASSIS_GEAR_RATIO;
    }

#if CHASSIS_ROLLER_X
    float s = 1.0f;
#else
    float s = -1.0f;
#endif

    vx = ( v[CHASSIS_WHEEL_FL] + v[CHASSIS_WHEEL_FR] + v[CHASSIS_WHEEL_RL] + v[CHASSIS_WHEEL_RR]) * 0.25f;
    vy = ((-v[CHASSIS_WHEEL_FL] + v[CHASSIS_WHEEL_FR] + v[CHASSIS_WHEEL_RL] - v[CHASSIS_WHEEL_RR]) * 0.25f) * s;
    wz_rad = ((-v[CHASSIS_WHEEL_FL] + v[CHASSIS_WHEEL_FR] - v[CHASSIS_WHEEL_RL] + v[CHASSIS_WHEEL_RR])
              / (4.0f * k));

    if (vx_mmps) *vx_mmps = vx;
    if (vy_mmps) *vy_mmps = vy;
    if (wz_dps)  *wz_dps  = wz_rad * 180.0f / CHASSIS_PI;
}

/* ==========================================================================
 * 4. 指令队列与下发
 *
 *   ⚠️ 铁律：**一次循环最多只发一条串口指令**（那条总线是 7 个口共用的）。
 *   所以这里不直接发，而是给每个轮子挂一个"待发动作"，由 chassis_do_one_action()
 *   每轮只执行一条 —— 既不会违反驱动器"两条指令间隔 ≥2ms"的要求，也不会堵主循环。
 * ========================================================================== */
typedef enum
{
    WACT_NONE = 0,      /* 没事干 */
    WACT_SPEED,         /* 下发速度 */
    WACT_BRAKE,         /* 0xFC 刹车（急停用） */
    WACT_CLEAR,         /* 0xFB 清状态（解除刹车） */
    WACT_ENABLE,        /* 0xFA 使能/失能 */
    WACT_READ           /* 读转速（反馈用） */
} wheel_act_t;

static wheel_act_t s_act[CHASSIS_WHEEL_NUM];
static float       s_act_rpm[CHASSIS_WHEEL_NUM];    /* WACT_SPEED 时的目标 rpm */
static uint8_t     s_act_en[CHASSIS_WHEEL_NUM];     /* WACT_ENABLE 时的 0/1 */
static uint8_t     s_act_sel;                        /* 轮询起点（保证 4 台轮流） */

/* 给某个轮子挂一个动作（已经被挂过就覆盖，不叠加） */
static void chassis_queue(uint8_t wheel, wheel_act_t act)
{
    if (wheel >= CHASSIS_WHEEL_NUM) return;

    /* ★不能无条件覆盖！"读转速"每 20ms 就会轮到每个轮子一次，
       如果它把还没发出去的 SPEED/BRAKE 覆盖成 READ，
       那条运动指令就被吞掉了（现象：发命令车不动 / 动一下就停 / 四轮不同步）。
       所以：读操作只在该轮子"没别的事"时才挂。 */
    if ((act == WACT_READ) && (s_act[wheel] != WACT_NONE)) return;

    s_act[wheel] = act;
}

/* 执行一条待发动作，返回 1 = 发了；0 = 队列空 */
static uint8_t chassis_do_one_action(void)
{
    for (uint8_t k = 0; k < CHASSIS_WHEEL_NUM; k++)
    {
        uint8_t i = (uint8_t)((s_act_sel + k) % CHASSIS_WHEEL_NUM);

        if (s_act[i] == WACT_NONE) continue;

        switch (s_act[i])
        {
            case WACT_SPEED:
            {
                /* 转成驱动器要的"方向 + 速度"：速度取绝对值，负数走 dir=1
                   ★目标为 0 时【必须真的发 0】：驱动器发 0.1rpm 是真在转
                     （75mm 轮：0.1rpm ≈ 0.4mm/s，一分钟 2.4cm → 就是"停下还微微动"）。
                     之前"开机强制零速"发 0.0 确实能让车停住，说明 0 有效。 */
                uint8_t dir = (s_act_rpm[i] < 0.0f) ? 1u : 0u;
                float   spd = fabsf(s_act_rpm[i]);

                if (spd < CHASSIS_MIN_RPM) spd = 0.0f;     /* 0.1rpm 以下一律按"停"= 0 */
                PD42S1_MotorSpeed(s_wheel[i].motor, dir, spd);   /* 内部自动套 dir_inv/加速度 */
                s_rpm_cmd[i] = s_act_rpm[i];
            }
                break;

            case WACT_BRAKE:
                PD42S1_MotorStop(s_wheel[i].motor);              /* 0xFC */
                s_rpm_cmd[i] = 0.0f;
                break;

            case WACT_CLEAR:
                PD42S1_ClearState(PD42S1_GetMotorCfg(s_wheel[i].motor)->addr);   /* 0xFB */
                break;

            case WACT_ENABLE:
                PD42S1_MotorEnable(s_wheel[i].motor, s_act_en[i]);              /* 0xFA */
                break;

            case WACT_READ:
                PD42S1_ReadSpeed(s_wheel[i].motor);              /* 0x29 */
                break;

            default:
                break;
        }

        s_act[i]   = WACT_NONE;
        s_act_sel  = (uint8_t)((i + 1u) % CHASSIS_WHEEL_NUM);
        return 1u;
    }
    return 0u;
}

/* 把 4 个轮子的目标转速交给队列（只挂"变化超过死区"的，省总线） */
static void chassis_send_all(const float *rpm)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        float d = rpm[i] - s_rpm_cmd[i];
        if (d < 0.0f) d = -d;

        if (d >= CHASSIS_CMD_DEADBAND_RPM)
        {
            s_act_rpm[i] = rpm[i];
            chassis_queue(i, WACT_SPEED);
        }
    }
}

/* 4 个轮子都停下来（给最小速度，保持使能，不刹车） */
static void chassis_queue_stop(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_act_rpm[i] = 0.0f;
        chassis_queue(i, WACT_SPEED);
    }
}

/* ==========================================================================
 * 5. 反馈与里程计
 * ========================================================================== */
static void chassis_read_feedback(void)
{
#if CHASSIS_FORCE_ECHO
    /* ★周期性地把 1~4 号电机设成"要回响"（0x6F）
       如果某台被设成"不回响"，它会执行命令但一个字节都不回 →
       转速/位置反馈全读不到（现象就是 rpm 恒 0、N0/4）。
       每 3 秒刷一次，便宜且自愈。 */
    {
        static uint32_t s_echo_tick;
        uint32_t now2 = HAL_GetTick();

        if ((uint32_t)(now2 - s_echo_tick) >= 3000u)
        {
            uint8_t k;

            s_echo_tick = now2;
            for (k = 0u; k < (uint8_t)CHASSIS_WHEEL_NUM; k++)
            {
                PD42S1_SetEcho((uint8_t)(k + 1u), 1u);
            }
        }
    }
#endif

    if ((uint32_t)(HAL_GetTick() - s_fb_tick) < (uint32_t)CHASSIS_FB_PERIOD_MS) return;
    s_fb_tick = HAL_GetTick();

    /* 挂一条"读转速"到队列，由 chassis_do_one_action() 发（4 台轮流） */
    chassis_queue(s_fb_sel, WACT_READ);
    s_fb_sel = (uint8_t)((s_fb_sel + 1u) % CHASSIS_WHEEL_NUM);
}

static void chassis_update_odometry(void)
{
#if CHASSIS_ODOM_ENABLE
    static uint32_t last_tick;
    static uint8_t  primed;
    uint32_t now = HAL_GetTick();
    float    dt;
    float    rpm[CHASSIS_WHEEL_NUM];
    float    vx, vy, wz;
    uint8_t  i;

    if (now == last_tick) return;
    dt = (float)(now - last_tick) * 0.001f;
    last_tick = now;
    if (dt <= 0.0f || dt > 0.2f) return;            /* 第一次或超时不算 */

    for (i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        const pd42s1_t *st = PD42S1_GetMotorState(s_wheel[i].motor);

        if (st == 0) { primed = 0u; return; }

        /* ⚠️ 这里**不能**用 pos_valid 当前置条件：底盘路径只读转速（0x29）、从不读位置，
           所以 pos_valid 永远是 0 → 里程计会被卡死（s_dist_mm 恒为 0）。
           里程计算本来就用转速；以后要做"位置差分里程计"，先让底盘路径去读位置（0x29 之外再发位置读帧）。 */
        rpm[i] = (float)st->rpm;
    }

    if (primed == 0u) { primed = 1u; return; }       /* 第一轮先只记录，不积分 */

    /* 正运动学 → 整车速度估计 */
    Chassis_Unmix(rpm, &vx, &vy, &wz);

    /* 累计路程（用车体速度的模长近似） */
    s_dist_mm += sqrtf(vx * vx + vy * vy) * dt;

    /* 机体系位移（带符号）：位置闭环要用（平移稳定 / 走到某个点）
       ⚠️ 这是轮式积分，打滑会累积误差；能用 IMU 的地方用 IMU 修正航向 */
    s_fwd_mm  += vx * dt;
    s_side_mm += vy * dt;

    /* 轮式航向积分（给互补滤波做参考） */
    s_yaw_odom_deg = wrap180(s_yaw_odom_deg + wz * dt);
#else
    (void)0;
#endif
}

/* 航向融合：陀螺积分为主，用轮式航向慢慢拉回（抑制漂移） */
static void chassis_update_yaw(void)
{
    static uint32_t last_tick;
    uint32_t now = HAL_GetTick();
    float    dt;

    if (now == last_tick) return;
    dt = (float)(now - last_tick) * 0.001f;
    last_tick = now;
    if (dt <= 0.0f || dt > 0.2f) return;

#if CHASSIS_ODOM_ENABLE
    /* 先用轮式航向兜底（没有 IMU 时就是它） */
    if (IMU_IsOnline() == 0u)
    {
        s_yaw_deg = s_yaw_odom_deg;
        return;
    }
#endif

#if IMU_USE_FUSED_YAW
    /* IMU 自带姿态融合：直接用模块的偏航角
       ★但模块的陀螺零偏会让 yaw 一直漂（实测约 10°/分 = 0.165°/s）。
       这里做【非阻塞】的漂移估计 + 连续补偿：
         · 车"确定没在动"（无转动/平移/速度指令/急停）且 yaw 变化率很小
           → 认为这点变化就是零漂 → 用低通慢慢学成 s_drift_dps
         · 无论动还是静，每周期都从 yaw 里扣掉 (漂移率 × dt)
       好处：静止零漂≈0、运动中也不跑弯，而且不像"冻结"那样藏住转向回弹。 */
    {
        float y_raw = wrap180(IMU_GetYawDeg() * s_yaw_scale);
        float dt2   = (dt > 0.001f) ? dt : 0.02f;
        float dy    = wrap180(y_raw - s_drift_y_last);
        float rate  = dy / dt2;
        uint8_t still = ((s_turn_active == 0u) && (s_strafe_active == 0u) &&
                         (s_emergency == 0u) &&
                         (s_vx_cmd == 0.0f) && (s_vy_cmd == 0.0f) && (s_wz_cmd == 0.0f)) ? 1u : 0u;

#if CHASSIS_DRIFT_TRIM
        /* 只在"静止且变化很慢"时才学习，避免把真实转动学成漂移 */
        if ((s_drift_have != 0u) && (still != 0u) &&
            (fabsf(rate) < (float)CHASSIS_DRIFT_LEARN_MAX_DPS))
        {
            float g = (float)CHASSIS_DRIFT_LEARN_GAIN;

            s_drift_dps += (rate - s_drift_dps) * g;
            if (s_drift_dps >  (float)CHASSIS_DRIFT_MAX_DPS) s_drift_dps =  (float)CHASSIS_DRIFT_MAX_DPS;
            if (s_drift_dps < -(float)CHASSIS_DRIFT_MAX_DPS) s_drift_dps = -(float)CHASSIS_DRIFT_MAX_DPS;
        }

        /* 连续补偿：把这段漂移从航向里扣掉 */
        s_drift_off -= s_drift_dps * dt2;
#endif
        s_drift_y_last = y_raw;
        s_drift_have   = 1u;

#if CHASSIS_STILL_YAW_HOLD
        /* （旧方案，默认关闭）静止时干脆冻结 yaw —— 简单但会藏住转向回弹，导致连转累积 */
        if ((still != 0u) && (fabsf(rate) < (float)CHASSIS_STILL_DPS))
        {
            /* 冻结：保持上一次的航向 */
        }
        else
        {
            s_yaw_deg = wrap180(y_raw + s_drift_off);
        }
#else
        s_yaw_deg = wrap180(y_raw + s_drift_off);
#endif
    }
    return;
#else
    /* 旧方案：陀螺积分 + 和轮式航向互补滤波 */
    s_yaw_deg = wrap180(s_yaw_deg + IMU_GetGyroZDps() * dt);
#if CHASSIS_ODOM_ENABLE
    s_yaw_deg = wrap180(s_yaw_deg + IMU_YAW_ODOM_K * wrap180(s_yaw_odom_deg - s_yaw_deg));
#endif
#endif
}

/* ==========================================================================
 * 6. 三个闭环 PID（航向稳定 / 旋转稳定 / 平移稳定）
 *
 *   三个环都是"误差 → 速度"，最终都落在 Chassis_Task 里：
 *     ① 航向稳定：wz 由航向环补（上层没要求自转时）
 *     ② 旋转稳定：wz 完全由旋转环给（转到目标角）
 *     ③ 平移稳定：vy 由平移环给（横移到位），同时航向环管着 wz（不偏头）
 *   优先级：旋转 > 航向（同一个 wz 只能有一个主人）
 * ========================================================================== */

/* ★IMU 数据是否"还新鲜"：电机一转 IMU 常常断帧，此时 yaw 是旧值。
   用 32 位帧计数（uint8 版会在 255 回绕时误判）。 */
static uint8_t imu_fresh(void)
{
    static uint32_t last_frames = 0u;
    static uint32_t last_tick   = 0u;
    static uint8_t  inited      = 0u;
    uint32_t now   = HAL_GetTick();
    uint32_t frames = IMU_GetFrames32();

    if (inited == 0u) { inited = 1u; last_frames = frames; last_tick = now; return 1u; }

    if (frames != last_frames) { last_frames = frames; last_tick = now; return 1u; }

    return (uint8_t)(((uint32_t)(now - last_tick) <= (uint32_t)IMU_STALE_MS) ? 1u : 0u);
}

/* ① 航向稳定：返回 wz 修正量（度/s） */
static float chassis_heading_correct(float dt)
{
    float err = wrap180(s_hdg_target_deg - s_yaw_deg);      /* 想保持的航向 − 当前航向 */

    /* ★大误差保护：航向环只负责"走直时的小偏差"。误差超过门限说明
       （a）上电后 yaw 没归零、（b）有人手动搬了车、（c）目标设错。
       这时还按 PID 输出会直接打满（实测 120°/s）把车原地拧飞 120° ——
       危险，比赛时会直接撞墙。所以大误差时航向环不参与，交给旋转环或人工 */
    if ((err > (float)CHASSIS_HDG_MAX_ERR_DEG) || (err < -(float)CHASSIS_HDG_MAX_ERR_DEG))
    {
        return 0.0f;
    }

    if (imu_fresh() == 0u) { pid_reset(&s_pid_hdg); return 0.0f; }     /* IMU 断帧：yaw 是旧值，不纠 */

    return pid_step(&s_pid_hdg, err, dt);
}

static float chassis_turn_correct(float dt)
{
    float err;
    float out;
    static float prev_out;
    float max_step;

    /* IMU 新鲜：闭环跟融合 yaw。
       IMU 断帧：轮速反馈稀疏，轮式航向几乎积不动 → 误差钉在 ~90° 会一直打满转好几圈。
       改用「指令 wz 积分」开环，并粘住，避免 fresh/stale 来回切。 */
    if ((imu_fresh() == 0u) && (s_turn_openloop == 0u))
    {
        float start = wrap180(s_turn_target_deg - s_turn_rel_deg);
        s_turn_spun_cmd_deg = wrap180(s_yaw_deg - start); /* 中途切入：带上 IMU 已转过的量 */
        s_turn_openloop = 1u;
    }

    if (s_turn_openloop == 0u)
    {
        err = wrap180(s_turn_target_deg - s_yaw_deg);
        out = pid_step(&s_pid_turn, err, dt);
    }
    else
    {
        pid_reset(&s_pid_turn);
        err = s_turn_rel_deg - s_turn_spun_cmd_deg;   /* 不用 wrap：开环相对角线性积 */
        out = err * 4.0f;
        if (out >  (float)CHASSIS_TURN_WZ_MAX) out =  (float)CHASSIS_TURN_WZ_MAX;
        if (out < -(float)CHASSIS_TURN_WZ_MAX) out = -(float)CHASSIS_TURN_WZ_MAX;
        /* 接近目标时不再抬最小角速度，否则 2° 误差也会被抬到 25°/s 冲过 */
        if ((err < s_turn_arrive_deg) && (err > -s_turn_arrive_deg)) out = 0.0f;
    }

    max_step = CHASSIS_TURN_SLEW_DPS2 * dt;
    if (max_step > 0.0f) out = clampf(out, prev_out - max_step, prev_out + max_step);
    prev_out = out;

    if (s_turn_openloop != 0u) s_turn_spun_cmd_deg += out * dt;

    if ((err < s_turn_arrive_deg) && (err > -s_turn_arrive_deg))
    {
        if (s_turn_settle_tick == 0u) s_turn_settle_tick = HAL_GetTick();
        else if ((float)(uint32_t)(HAL_GetTick() - s_turn_settle_tick) >= s_turn_settle_ms)
        {
            s_turn_done = 1u;
        }
    }
    else
    {
        s_turn_settle_tick = 0u;
    }

    return out;
}

/* ==========================================================================
 * 7. 上电自检：走方形，用来验证运动学与接线
 *
 *   序列：前进 → 停 → 左平移 → 停 → 原地左转 → 停 → 后退 → 停（然后循环）
 *   看什么：
 *     · 有轮子不转 / 转反 → 改 config.h 的 CHASSIS_SIGN_xx
 *     · "左平移"变成"前进"或方向乱 → 改 CHASSIS_ROLLER_X
 *     · 车原地打转/画圈 → 四个轮子的符号组合错了
 * ========================================================================== */
#if CHASSIS_SELFTEST
static uint8_t  s_test_step;
static uint32_t s_test_tick;

void Chassis_TestTask(void)
{
    if ((uint32_t)(HAL_GetTick() - s_test_tick) < (uint32_t)CHASSIS_TEST_STEP_MS) return;
    s_test_tick = HAL_GetTick();

    switch (s_test_step)
    {
        case 0u: Chassis_SetVelocity( CHASSIS_TEST_SPEED, 0.0f, 0.0f); break;   /* 前进 */
        case 1u: Chassis_Stop();                                      break;
        case 2u: Chassis_SetVelocity(0.0f,  CHASSIS_TEST_SPEED, 0.0f); break;   /* 左平移 */
        case 3u: Chassis_Stop();                                      break;
        case 4u: Chassis_SetVelocity(0.0f, 0.0f,  CHASSIS_TEST_WZ);    break;   /* 原地左转 */
        case 5u: Chassis_Stop();                                      break;
        case 6u: Chassis_SetVelocity(-CHASSIS_TEST_SPEED, 0.0f, 0.0f); break;   /* 后退 */
        default: Chassis_Stop();                                      break;
    }

    s_test_step = (uint8_t)((s_test_step + 1u) % 8u);
}
#endif /* CHASSIS_SELFTEST */

/* ==========================================================================
 * 8. 对外接口
 * ========================================================================== */
void Chassis_Init(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_rpm_cmd[i]  = 0.0f;
        s_rpm_meas[i] = 0.0f;
        s_act[i]      = WACT_NONE;
    }

    s_vx_cmd = s_vy_cmd = s_wz_cmd = 0.0f;
    s_yaw_deg = s_yaw_odom_deg = 0.0f;
    s_dist_mm = 0.0f;
    s_fwd_mm = s_side_mm = 0.0f;
    s_emergency = 0u;
    s_act_sel = 0u;

    chassis_pid_init_all();             /* 三个闭环（航向/旋转/平移）按 config.h 5.7 区初始化 */

    s_last_task_tick = HAL_GetTick();
    s_fb_tick        = HAL_GetTick();
    s_refresh_tick   = HAL_GetTick();
#if CHASSIS_SELFTEST
    s_test_tick      = HAL_GetTick();
#endif

    /* 上电让 4 个轮子进入"使能且不动"：排队给最小速度（会自动一条一条发出去） */
    chassis_queue_stop();

    /* ★开机就做一次"航向归零 + 漂移测量"（会静止采样 IMU_TRIM_MS，默认 600ms）。
       ⚠️ 原来只有屏幕切模式才归零 → 上电直接看 ch0 的话，漂移根本没被测过/抵消过，
          表现就是"ch0 一直在漂"。这一步之后，ch0 的漂移率才真正被扣掉。 */
    Chassis_ResetYaw();
}

void Chassis_SetVelocity(float vx_mmps, float vy_mmps, float wz_dps)
{
    s_vx_cmd = vx_mmps;
    s_vy_cmd = vy_mmps;
    s_wz_cmd = wz_dps;
    s_emergency = 0u;
}

void Chassis_Stop(void)
{
    s_vx_cmd = s_vy_cmd = s_wz_cmd = 0.0f;
    chassis_queue_stop();
}

void Chassis_EmergencyStop(void)
{
    s_vx_cmd = s_vy_cmd = s_wz_cmd = 0.0f;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        chassis_queue(i, WACT_BRAKE);           /* 0xFC 刹车（一条一条发出去） */
    }
    s_emergency = 1u;
}

void Chassis_Resume(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        chassis_queue(i, WACT_CLEAR);           /* 0xFB 清状态，解除刹车 */
    }
    s_emergency = 0u;
}

void Chassis_Enable(uint8_t en)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        s_act_en[i] = (uint8_t)(en ? 1u : 0u);
        chassis_queue(i, WACT_ENABLE);
    }
}

void Chassis_Task(void)
{
    float rpm[CHASSIS_WHEEL_NUM];
    uint32_t now = HAL_GetTick();

    /* --- 0. ★开机 2 秒内【强制】把 0 速度推给 4 个轮子 ---
       为什么必须：PD42S1 在"通信速度模式"下会一直保持最后一次收到的速度。
       MCU 一复位，内部目标也是 0，而 chassis_send_all 有"变化 < 0.5rpm 就不发"的死区，
       于是【一帧都不发】→ 驱动器继续按复位前那个残留速度转（现象：静止时车头缓转）。
       这里开机阶段每 50ms 直接发一次 0，绕开死区和队列。 */
    {
        static uint32_t s_boot_tick;
        static uint32_t s_force_tick;
        static uint8_t  s_force_left = 40u;      /* 40 次 × 50ms ≈ 2 秒 */

        if (s_boot_tick == 0u) s_boot_tick = now ? now : 1u;

        if ((s_force_left > 0u) && ((uint32_t)(now - s_force_tick) >= 50u))
        {
            uint8_t k;

            s_force_tick = now;
            s_force_left--;
            for (k = 0u; k < (uint8_t)CHASSIS_WHEEL_NUM; k++)
            {
                PD42S1_MotorSpeed(s_wheel[k].motor, 0u, 0.0f);   /* 直接发，速度 0 */
                s_rpm_cmd[k] = 0.0f;
            }
        }
    }

    /* --- 1. 每 20ms 算一次目标轮速，并把变化挂进队列 --- */
    if ((uint32_t)(now - s_last_task_tick) >= 20u)
    {
        float vx, vy, wz;
        /* ★三个 PID 用【真实经过的时间】，不能用写死的 0.02：
           主循环被别的事拖慢时（比如蓝牙那路阻塞发送、步进总线应答慢），
           实际周期可能是 30~50ms。写死 dt 会让积分项偏小、微分项偏大，
           而且偏多少取决于当时的循环速度 → 调出来的参数换台车/换个负载就不对了。 */
        float dt = (float)(uint32_t)(now - s_last_task_tick) * 0.001f;

        if ((dt < 0.005f) || (dt > 0.2f)) dt = 0.02f;    /* 防呆：异常值退回标称周期 */

        s_last_task_tick = now;

#if CHASSIS_SELFTEST && !CHASSIS_DIST_TEST_ON && !CHASSIS_TURN_TEST_ON && !CHASSIS_STRAFE_TEST_ON && !CHASSIS_MISSION1_ACTIVE
        Chassis_TestTask();                     /* 自检改目标速度；⑧/⑨区测试打开时自检自动让位 */
#endif
        vx = s_vx_cmd;
        vy = s_vy_cmd;
        wz = s_wz_cmd;

        /* ★侧滑补偿：麦轮装反/不平行会产生恒定横向漂移（车头不弯、整体横移）
           → 前进时叠加 vy = vx × CHASSIS_CRAB_TRIM 抵消它 */
        if ((s_crab_trim != 0.0f) && (s_vy_cmd == 0.0f) && (s_strafe_active == 0u))
        {
            vy += s_vx_cmd * s_crab_trim;
        }
        /* ③ 平移稳定：横向速度交给平移环（横移到位），wz 交给航向环（不偏头） */
        if ((s_strafe_active != 0u) && (s_emergency == 0u))
        {
            float side_err = s_strafe_side_target - s_side_mm;

            vx = s_strafe_vx;
            vy = pid_step(&s_pid_side, side_err, dt);
            if ((side_err < s_side_arrive_mm) && (side_err > -s_side_arrive_mm))
            {
                s_strafe_done = 1u;
            }
            wz += chassis_heading_correct(dt);      /* 横移时也保持航向 */
        }
        /* ② 旋转稳定：wz 由旋转环独占 */
        else if ((s_turn_active != 0u) && (s_emergency == 0u))
        {
            wz = chassis_turn_correct(dt);
        }
        /* ① 航向稳定：上层没要求自转时，用 PID 修方向
           ⚠️ 只在【真的要动】(vx/vy 非 0) 时纠偏：停车后不再自己掰方向，
              否则 Chassis_Stop() 之后车会自己慢慢转回 CHASSIS_HEADING_TARGET_DEG，很吓人 */
#if CHASSIS_HEADING_HOLD
        else if ((s_emergency == 0u) && (wz == 0.0f) &&
                 ((vx != 0.0f) || (vy != 0.0f)))
        {
            wz += chassis_heading_correct(dt);
        }
#endif

        if (s_emergency == 0u)
        {
            Chassis_Mix(vx, vy, wz, rpm);
            chassis_send_all(rpm);
        }

        /* 可选：定时重发一次速度（治"停下后轮子微动"；CHASSIS_REFRESH_MS=250）
           ★必须用【当前目标】重发，不能用 s_rpm_cmd（那是"上一次已经发出去的"旧值）：
             否则刚算出的"目标 0"还没轮到发送，就被刷新用旧速度顶掉 → 车该停不停、一直转。 */
#if CHASSIS_REFRESH_MS > 0
        if ((uint32_t)(now - s_refresh_tick) >= (uint32_t)CHASSIS_REFRESH_MS)
        {
            s_refresh_tick = now;
            for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
            {
                s_act_rpm[i] = rpm[i];          /* ← 当前这一拍算出来的目标 */
                chassis_queue(i, WACT_SPEED);
            }
        }
#else
        (void)s_refresh_tick;
#endif
    }

    /* --- 2. 串口：这一轮最多发一条指令（动作优先） --- */
    (void)chassis_do_one_action();

    /* --- 3. 反馈节拍（挂"读转速"进队列，下一轮才会真正发出去） --- */
    chassis_read_feedback();

    /* --- 4. 纯计算：更新测量值、里程计、航向 --- */
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        const pd42s1_t *st = PD42S1_GetMotorState(s_wheel[i].motor);
        if (st != 0) s_rpm_meas[i] = (float)st->rpm;
    }

    chassis_update_odometry();
    chassis_update_yaw();
}

float Chassis_GetWheelRpm(uint8_t wheel)
{
    if (wheel >= CHASSIS_WHEEL_NUM) return 0.0f;
    return s_rpm_meas[wheel];
}

float Chassis_GetYawDeg(void)
{
    return s_yaw_deg;
}

/* ★yaw 标度修正：IMU 报的角度可能和真实角度有百分比差（实测 90° 只报 87~88° 之类）。
   修正后转向环会按"真实角度"闭环，转角自然准。
   用法：手动转 4 个 90°，看 IMU 累计报了多少（比如 350 而不是 360），
         标度 = 360 / 350 = 1.0286 → 发命令 K1.0286 */
void Chassis_SetYawScale(float k)
{
    if (k < 0.80f) k = 0.80f;
    if (k > 1.25f) k = 1.25f;
    s_yaw_scale = k;
}

float Chassis_GetYawScale(void)
{
    return s_yaw_scale;
}

/* ★诊断：当前估计出的零漂率（度/分），正数 = yaw 在往正方向漂 */
float Chassis_GetDriftDpm(void)
{
    return s_drift_dps * 60.0f;
}

void Chassis_GetVelocityEstimate(float *vx_mmps, float *vy_mmps, float *wz_dps)
{
    Chassis_Unmix(s_rpm_meas, vx_mmps, vy_mmps, wz_dps);
}

float Chassis_GetDistanceMm(void)
{
    return s_dist_mm;
}

/* ★车是不是"正在动"（给云台跟随/行进姿态用）：
   有转动环/平移环在跑，或者速度指令非 0 → 认为在动 */
uint8_t Chassis_IsMoving(void)
{
    if (s_turn_active != 0u)   return 1u;
    if (s_strafe_active != 0u) return 1u;
    if (s_vx_cmd != 0.0f)      return 1u;
    if (s_vy_cmd != 0.0f)      return 1u;
    if (s_wz_cmd != 0.0f)      return 1u;
    return 0u;
}

uint8_t Chassis_IsOnline(void)
{
    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        const pd42s1_t *st = PD42S1_GetMotorState(s_wheel[i].motor);
        if (st == 0 || st->online == 0u) return 0u;
    }
    return 1u;
}

/* ★诊断（屏幕显示用）：四个轮子驱动器
     online_cnt    = 有应答过的台数（0~4）→ 0 = 串口总线 RX 完全没通
     rpm_valid_cnt = 转速字段有效的台数    → 0 = 应答有、但"读转速(0x29)"没解析上 */
void Chassis_GetFeedbackHealth(uint8_t *online_cnt, uint8_t *rpm_valid_cnt)
{
    uint8_t on = 0u, rv = 0u;

    for (uint8_t i = 0; i < CHASSIS_WHEEL_NUM; i++)
    {
        const pd42s1_t *st = PD42S1_GetMotorState(s_wheel[i].motor);

        if (st == 0) continue;
        if (st->online != 0u)    on++;
        if (st->rpm_valid != 0u) rv++;
    }
    if (online_cnt != 0)    *online_cnt = on;
    if (rpm_valid_cnt != 0) *rpm_valid_cnt = rv;
}

void Chassis_ResetYaw(void)
{
    s_yaw_deg = 0.0f;
    s_yaw_odom_deg = 0.0f;
    s_drift_off = 0.0f;                 /* ★补偿量归零（漂移率的估计值保留，继续用） */
    s_drift_y_last = wrap180(IMU_GetYawDeg() * s_yaw_scale);
    s_drift_have = 1u;
    s_fwd_mm = 0.0f;                    /* 位置闭环的参考点也一起归零 */
    s_side_mm = 0.0f;
    s_hdg_target_deg = CHASSIS_HEADING_TARGET_DEG;
    pid_reset(&s_pid_hdg);
    pid_reset(&s_pid_turn);
    pid_reset(&s_pid_side);
    s_turn_active = 0u;
    s_turn_done = 0u;
    s_strafe_active = 0u;
    s_strafe_done = 0u;
    /* ★归零的同时测一次 yaw 漂移率并抵消（IMU_TRIM_MS 毫秒静止采样）：
       六轴融合的 yaw 会漂，模块自带校准不生效时（我们的模块就是）能漂到几十度/分，
       调航向环 / 跑直线全靠这一步把它压掉。IMU_TRIM_MS=0 时等价于普通归零 */
    (void)IMU_TrimYaw((uint16_t)IMU_TRIM_MS);
}

/* ==========================================================================
 * 9. 三个闭环 PID 的对外接口（config.h 5.7 区）
 * ========================================================================== */

/* ---- ① 航向稳定 ---- */
void  Chassis_SetWheelTrim(uint8_t idx, float k) { if (idx < CHASSIS_WHEEL_NUM) { if (k < 0.9f) k = 0.9f; if (k > 1.1f) k = 1.1f; s_trim_rpm[idx] = k; } }
float Chassis_GetWheelTrim(uint8_t idx) { return (idx < CHASSIS_WHEEL_NUM) ? s_trim_rpm[idx] : 1.0f; }

void  Chassis_SetCrabTrim(float k)  { s_crab_trim = k; }
float Chassis_GetCrabTrim(void)      { return s_crab_trim; }

void  Chassis_SetStraightTrimDeg(float deg) { s_straight_trim = deg; }
float Chassis_GetStraightTrimDeg(void)      { return s_straight_trim; }

void Chassis_SetHeadingTargetDeg(float deg)
{
    s_hdg_target_deg = wrap180(deg);
}

float Chassis_GetHeadingTargetDeg(void) { return s_hdg_target_deg; }
float Chassis_GetHeadingErrDeg(void)    { return wrap180(s_hdg_target_deg - s_yaw_deg); }

/* ---- ② 旋转稳定 ---- */
void Chassis_TurnStart(float deg)
{
    /* ★多打 CHASSIS_TURN_BIAS_DEG：实测停下会回弹约 1.9°，这样回弹后正好停在目标上 */
    s_turn_target_deg  = wrap180(deg + (float)CHASSIS_TURN_BIAS_DEG);
    s_turn_rel_deg     = wrap180(s_turn_target_deg - s_yaw_deg);
    s_turn_spun_cmd_deg = 0.0f;
    s_turn_openloop    = (imu_fresh() == 0u) ? 1u : 0u;
    s_turn_active      = 1u;
    s_turn_done        = 0u;
    s_turn_settle_tick = 0u;
    pid_reset(&s_pid_turn);
}

void Chassis_TurnStop(void)
{
    s_turn_active = 0u;
    s_turn_done   = 0u;
    pid_reset(&s_pid_turn);
}

uint8_t Chassis_TurnIsActive(void) { return s_turn_active; }
uint8_t Chassis_TurnIsDone(void)   { return s_turn_done; }
float   Chassis_GetTurnErrDeg(void){ return wrap180(s_turn_target_deg - s_yaw_deg); }

/* ---- ③ 平移稳定 ---- */
void Chassis_StrafeStart(float vx_mmps, float side_mm)
{
    s_strafe_vx          = vx_mmps;
    s_strafe_side_target = s_side_mm + side_mm;     /* 相对【当前】位置再走 side_mm（右为正） */
    s_strafe_active      = 1u;
    s_strafe_done        = 0u;
    pid_reset(&s_pid_side);
    pid_reset(&s_pid_hdg);
    s_hdg_target_deg     = s_yaw_deg;               /* 横移时保持当前航向 */
}

void Chassis_StrafeStop(void)
{
    s_strafe_active = 0u;
    s_strafe_done   = 0u;
    pid_reset(&s_pid_side);
}

uint8_t Chassis_StrafeIsActive(void) { return s_strafe_active; }
uint8_t Chassis_StrafeIsDone(void)   { return s_strafe_done; }
float   Chassis_GetSideErrMm(void)   { return s_strafe_side_target - s_side_mm; }

float Chassis_GetForwardMm(void) { return s_fwd_mm; }
float Chassis_GetSideMm(void)    { return s_side_mm; }

/* 调参用：一次把三个环的中间量取出来（VOFA_SRC_CHASSIS 就是按这个顺序发的）
   out[0] yaw(度)          out[1] 航向误差(度)     out[2] 航向环输出(度/s)  out[3] 实测 wz(度/s)
   out[4] 前向位移(mm)     out[5] 横向位移(mm)     out[6] 旋转误差(度)      out[7] 旋转环输出(度/s)
   out[8] 平移误差(mm)     out[9] 平移环输出(mm/s) */
void Chassis_GetPidDebug(float *out, uint8_t n)
{
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    float v[10];

    if ((out == 0) || (n == 0u)) return;
    if (n > 10u) n = 10u;

    Chassis_Unmix(s_rpm_meas, &vx, &vy, &wz);

    v[0] = s_yaw_deg;
    v[1] = wrap180(s_hdg_target_deg - s_yaw_deg);
    v[2] = s_pid_hdg.out;
    v[3] = wz;
    v[4] = s_fwd_mm;
    v[5] = s_side_mm;
    v[6] = wrap180(s_turn_target_deg - s_yaw_deg);
    v[7] = s_pid_turn.out;
    v[8] = s_strafe_side_target - s_side_mm;
    v[9] = s_pid_side.out;

    for (uint8_t i = 0u; i < n; i++) out[i] = v[i];
}

/* ==========================================================================
 * 9.5 调参脚本用的"在线上改参数"接口（只改 RAM，不写 flash；重启就回到 config.h）
 *
 *   loop : 0 = 航向稳定  1 = 旋转稳定  2 = 平移稳定
 *   which: 0 = Kp  1 = Ki  2 = Kd
 *   param: 0 = 死区(deadband)  1 = 输出限幅(out_max)  2 = 积分限幅(i_max)
 *          3 = 到位阈值(arrive：旋转=度 / 平移=mm)  4 = 稳定时间(settle：只对旋转有意义，ms)
 *   返回 1 = 成功；0 = 索引不对
 * ========================================================================== */
static chassis_pid_t *chassis_pid_of(uint8_t loop)
{
    if (loop == 0u) return &s_pid_hdg;
    if (loop == 1u) return &s_pid_turn;
    if (loop == 2u) return &s_pid_side;
    return 0;
}

void Chassis_PidSetGain(uint8_t loop, uint8_t which, float v)
{
    chassis_pid_t *p = chassis_pid_of(loop);

    if (p == 0) return;
    if      (which == 0u) p->kp = v;
    else if (which == 1u) { p->ki = v; pid_reset(p); }   /* 改 I 就把积分清零，免得上一次的积分残留 */
    else if (which == 2u) p->kd = v;
}

float Chassis_PidGetGain(uint8_t loop, uint8_t which)
{
    chassis_pid_t *p = chassis_pid_of(loop);

    if (p == 0) return 0.0f;
    if (which == 0u) return p->kp;
    if (which == 1u) return p->ki;
    if (which == 2u) return p->kd;
    return 0.0f;
}

void Chassis_PidSetParam(uint8_t loop, uint8_t param, float v)
{
    chassis_pid_t *p = chassis_pid_of(loop);

    if (param == 3u)                     /* 到位阈值：旋转/平移各一个 */
    {
        if (loop == 1u) s_turn_arrive_deg = v;
        if (loop == 2u) s_side_arrive_mm  = v;
        return;
    }
    if (param == 4u)                     /* 稳定时间（只对旋转环） */
    {
        if (loop == 1u) s_turn_settle_ms = v;
        return;
    }

    if (p == 0) return;
    if      (param == 0u) p->deadband = v;
    else if (param == 1u) p->out_max  = v;
    else if (param == 2u) p->i_max    = v;
}

float Chassis_PidGetParam(uint8_t loop, uint8_t param)
{
    chassis_pid_t *p = chassis_pid_of(loop);

    if (param == 3u)
    {
        if (loop == 1u) return s_turn_arrive_deg;
        if (loop == 2u) return s_side_arrive_mm;
        return 0.0f;
    }
    if (param == 4u) return (loop == 1u) ? s_turn_settle_ms : 0.0f;

    if (p == 0) return 0.0f;
    if (param == 0u) return p->deadband;
    if (param == 1u) return p->out_max;
    if (param == 2u) return p->i_max;
    return 0.0f;
}

/* 三个环最近一次的误差 / 输出（调参脚本判"稳态残差、饱和度"用） */
float Chassis_PidGetErr(uint8_t loop)
{
    chassis_pid_t *p = chassis_pid_of(loop);

    return (p != 0) ? p->err : 0.0f;
}

float Chassis_PidGetOut(uint8_t loop)
{
    chassis_pid_t *p = chassis_pid_of(loop);

    return (p != 0) ? p->out : 0.0f;
}

#else   /* ---------------- 未启用：全部空实现（main.c 不用改） ---------------- */

void Chassis_Init(void) {}
void Chassis_Task(void) {}
void Chassis_SetVelocity(float vx, float vy, float wz) { (void)vx; (void)vy; (void)wz; }
void Chassis_Stop(void) {}
void Chassis_EmergencyStop(void) {}
void Chassis_Resume(void) {}
void Chassis_Enable(uint8_t en) { (void)en; }
void Chassis_TestTask(void) {}
float Chassis_GetWheelRpm(uint8_t w) { (void)w; return 0.0f; }
float Chassis_GetYawDeg(void) { return 0.0f; }
void Chassis_GetVelocityEstimate(float *vx, float *vy, float *wz) { *vx = 0.0f; *vy = 0.0f; *wz = 0.0f; }
float Chassis_GetDistanceMm(void) { return 0.0f; }
uint8_t Chassis_IsMoving(void) { return 0u; }
uint8_t Chassis_IsOnline(void) { return 0u; }
void Chassis_ResetYaw(void) {}
void Chassis_Mix(float vx, float vy, float wz, float *rpm) { (void)vx; (void)vy; (void)wz; for (uint8_t i=0;i<4;i++) rpm[i]=0.0f; }
void Chassis_Unmix(const float *rpm, float *vx, float *vy, float *wz) { (void)rpm; *vx=0.0f; *vy=0.0f; *wz=0.0f; }

/* 三个闭环 PID 的空实现（底盘关掉时） */
float Chassis_GetForwardMm(void) { return 0.0f; }
float Chassis_GetSideMm(void) { return 0.0f; }
void  Chassis_SetHeadingTargetDeg(float d) { (void)d; }
float Chassis_GetHeadingTargetDeg(void) { return 0.0f; }
float Chassis_GetHeadingErrDeg(void) { return 0.0f; }
void    Chassis_TurnStart(float d) { (void)d; }
void    Chassis_TurnStop(void) {}
uint8_t Chassis_TurnIsActive(void) { return 0u; }
uint8_t Chassis_TurnIsDone(void) { return 0u; }
float   Chassis_GetTurnErrDeg(void) { return 0.0f; }
void    Chassis_StrafeStart(float v, float s) { (void)v; (void)s; }
void    Chassis_StrafeStop(void) {}
uint8_t Chassis_StrafeIsActive(void) { return 0u; }
uint8_t Chassis_StrafeIsDone(void) { return 0u; }
float   Chassis_GetSideErrMm(void) { return 0.0f; }
void    Chassis_GetPidDebug(float *out, uint8_t n) { uint8_t i; for (i = 0u; (out != 0) && (i < n) && (i < 10u); i++) out[i] = 0.0f; }

/* 在线上改参数（调参脚本用）：底盘关掉时全是空实现 */
void  Chassis_PidSetGain(uint8_t loop, uint8_t which, float v) { (void)loop; (void)which; (void)v; }
float Chassis_PidGetGain(uint8_t loop, uint8_t which) { (void)loop; (void)which; return 0.0f; }
void  Chassis_PidSetParam(uint8_t loop, uint8_t param, float v) { (void)loop; (void)param; (void)v; }
float Chassis_PidGetParam(uint8_t loop, uint8_t param) { (void)loop; (void)param; return 0.0f; }
float Chassis_PidGetErr(uint8_t loop) { (void)loop; return 0.0f; }
float Chassis_PidGetOut(uint8_t loop) { (void)loop; return 0.0f; }

#endif /* CHASSIS_ENABLE */
