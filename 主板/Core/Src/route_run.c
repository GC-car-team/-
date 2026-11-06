/**
  * @file    route_run.c
  * @brief   比赛路线执行（来自视觉屏幕修正版，独立成模块，不动 main.c 的其它部分）
  *
  *  流程：屏幕"出发" → HMI_LoadPlannedPath() 按障碍算路 → HMI_CopyPath() 拿字母序列
  *        → 本模块按字母逐步执行：
  *           F 前进 ROUTE_SEGMENT_MM     L/R 转 90°    U 转 180°
  *           C 扫码  M 原料  P 放下  G 抓回  B 暂存
  *        工位动作 = 云台转到盘位 + 夹爪开合 + 等视觉到位（ROUTE_VISION_MS 超时兜底）
  */
#include "main.h"
#include "config.h"
#include "route_run.h"

#if CHASSIS_ENABLE && HMI_ENABLE

#include "hmi.h"
#include "chassis.h"
#include "gm6020.h"
#include "servo.h"
#include "arm.h"      /* 升降/齿条：ARM_Jog + 导程/脉冲换算 */
#include "vision.h"
#include "scan.h"
#include "imu.h"
#include <math.h>

/* ---- 路线状态 ---- */
static uint8_t  s_route[HMI_PATH_MAX];
static uint8_t  s_route_n       = 0u;   /* 字母总数 */
static uint8_t  s_route_i       = 0u;   /* 当前字母下标 */
static uint8_t  s_route_phase   = 2u;   /* 0 正在做，1 步间停顿，2 已结束 */
static uint8_t  s_route_busy    = 0u;   /* 这一步的指令已经发出 */
static uint8_t  s_gimbal_aim    = 0u;   /* 1 = 允许视觉带动云台 */
static uint8_t  s_route_imu     = 0u;   /* 这一步开始时 IMU 是否在线 */
static uint32_t s_route_tick    = 0u;
static float    s_route_deg0    = 0.0f; /* 这一步开始时的航向 */
static uint8_t  s_route_active  = 0u;   /* 1 = 正在跑路线 */

static void route_match_reset(void);
#if CHASSIS_ENABLE && HMI_ENABLE
static float route_wrap(float deg)
{
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* 这一步做完：停车，再留 ROUTE_GAP_MS 接到下一个字母 */
static void route_advance(uint32_t now)
{
    Chassis_Stop();
    s_route_busy = 0u;
    if (ROUTE_GAP_MS == 0u)
    {
        s_route_i++;
        s_route_phase = 0u;
    }
    else
    {
        s_route_phase = 1u;
        s_route_tick  = now;
    }
}

/* 本场统计。每次按启动清零。 */
static uint8_t s_wk_phase;
static uint8_t s_wk_slot;
static uint8_t s_wk_kind;   /* 0 扫码，1 抓取，2 放置 */
static uint8_t s_seen_m;
static uint8_t s_seen_p;
static uint8_t s_seen_g;
static uint8_t s_seen_b;
static uint8_t s_grab;
static uint8_t s_place;
static char    s_task[16];   /* 扫码得到的 15 字符任务码，无则空 */

/* 任务码第 slot 个颜色(1-6)/环号(1-3)。batch0=第一批，batch1=第二批。无效返回 0 */
static uint8_t route_task_color(uint8_t batch, uint8_t slot)
{
    uint8_t base = (batch == 0u) ? 0u : 8u;
    char c;
    if ((s_task[0] == '\0') || (slot > 2u)) return 0u;
    c = s_task[base + slot];
    if ((c < '1') || (c > '6')) return 0u;
    return (uint8_t)(c - '0');
}

static uint8_t route_task_ring(uint8_t batch, uint8_t slot)
{
    uint8_t base = (batch == 0u) ? 4u : 12u;
    char c;
    if ((s_task[0] == '\0') || (slot > 2u)) return 0u;
    c = s_task[base + slot];
    if ((c < '1') || (c > '3')) return 0u;
    return (uint8_t)(c - '0');
}

/* 按工位发视觉模式 + R1/单色过滤（设计 §2.0） */
static void route_vision_arm(uint8_t act)
{
    uint8_t batch = 0u;
    uint8_t color;
    uint8_t ring;
    uint8_t cmd;

    if (act == (uint8_t)'M') batch = (s_seen_m > 0u) ? 1u : 0u;
    else if (act == (uint8_t)'P') batch = (s_seen_p > 0u) ? 1u : 0u;
    else if (act == (uint8_t)'G') batch = (s_seen_g > 0u) ? 1u : 0u;
    else if (act == (uint8_t)'B') batch = (s_seen_b > 0u) ? 1u : 0u;

    Vision_BeginStep();

    if (act == (uint8_t)'M')
    {
        /* 原料：单色模式 + color 过滤 */
        color = route_task_color(batch, s_wk_slot);
        cmd = Vision_CmdForColor(color);
        Vision_SetExpect(VISION_KIND_MAT, color);
        Vision_SendCmd(cmd);
    }
    else
    {
        /* 粗加工/暂存放或抓回：圆环模式 + digit 过滤（R1） */
        ring = route_task_ring(batch, s_wk_slot);
        Vision_SetExpect(VISION_KIND_RING, ring);
        Vision_SendCmd(VISION_CMD_RING);
    }
    s_gimbal_aim = 1u;
    s_route_tick = HAL_GetTick();
}

/* 在 0~360 的读数上比较：目标更大就递增，更小就递减，不穿过 0°。 */
static void route_gimbal_goto(float deg)
{
    float cur = GM6020_GetCurrentDeg();
    int8_t dir = (deg >= cur) ? (int8_t)1 : (int8_t)(-1);

    s_gimbal_aim = 0u;
    GM6020_SetTargetPosDegDir(deg, dir);
}

static float route_disc_deg(uint8_t slot)
{
    if (slot == 0u) return ROT_DISC1_DEG;
    if (slot == 1u) return ROT_DISC2_DEG;
    return ROT_DISC3_DEG;
}

static uint8_t route_gimbal_done(uint32_t now)
{
    if (GM6020_IsOnline() == 0u) return 1u;
    if (GM6020_IsArrived() != 0u) return 1u;
    return (uint8_t)((uint32_t)(now - s_route_tick) >= (uint32_t)ROT_MOVE_MS);
}

static uint8_t route_grip_done(uint32_t now)
{
    return (uint8_t)((uint32_t)(now - s_route_tick) >= (uint32_t)GRIP_SETTLE_MS);
}

static void route_match_reset(void)
{
    s_wk_phase = 0u;
    s_wk_slot  = 0u;
    s_wk_kind  = 0u;
    s_gimbal_aim = 0u;
    s_seen_m = 0u;
    s_seen_p = 0u;
    s_seen_g = 0u;
    s_seen_b = 0u;
    s_grab = 0u;
    s_place = 0u;
    s_task[0] = '\0';
    route_gimbal_goto(ROT_TRAVEL_DEG);
#if VISION_ENABLE
    Vision_ClearTask();
    Vision_ClearExpect();
    Vision_SendCmd(VISION_CMD_IDLE);
#endif
}

static void route_score(void)
{
    char b[8];

    b[0] = 'G';
    b[1] = (char)('0' + (s_grab % 10u));
    b[2] = 'P';
    b[3] = (char)('0' + (s_place % 10u));
    b[4] = '\0';
    HMI_ShowBox("seq.txt", b);
}

#if VISION_ENABLE
static void route_send_vision(uint8_t act)
{
    route_vision_arm(act);
}

/* 这一件做完。还有下一件就转到下一盘/摆正，三件都完了回 1。 */
static uint8_t route_piece_done(uint8_t act, uint32_t now)
{
    s_gimbal_aim = 0u;
    if (s_wk_slot < 2u)
    {
        s_wk_slot++;
        s_wk_phase = 0u;
        s_route_tick = now;
        route_gimbal_goto((s_wk_kind == 1u) ? ROT_HOME_DEG : route_disc_deg(s_wk_slot));
        return 0u;
    }
    Vision_SendCmd(VISION_CMD_IDLE);
    if (act == (uint8_t)'M') s_seen_m++;
    else if (act == (uint8_t)'P') s_seen_p++;
    else if (act == (uint8_t)'G') s_seen_g++;
    else s_seen_b++;
    return 1u;
}
#endif

/* 返回 1 = 这个工位做完，可以去下一段路。扫码失败会直接结束全场。
   抓取（M/G）：摆正 → 视觉 → 合爪 → 转到本件物料盘 → 张开。
   放置（P/B）：转到本件物料盘 → 合爪 → 摆正 → 视觉 → 张开。 */
/* ==================== ★工位动作表执行器（接入升降/齿条/6020/夹爪）==================== */
typedef struct { char kind; float param; } st_step_t;

/* 表：取料类 / 放置类 / 扫码 */
static const st_step_t s_st_pick[]  = ST_PICK_STEPS;
static const st_step_t s_st_place[] = ST_PLACE_STEPS;
static const st_step_t s_st_scan[]  = ST_SCAN_STEPS;

static uint8_t  s_st_idx;       /* 当前第几步 */
static uint8_t  s_st_busy;      /* 这一步动作已下发 */
static uint32_t s_st_tick;      /* 这一步开始时 */

/* 估算"走 mm 需要多久"：按当前 jog 速度与导程算，再加余量 */
static uint32_t st_move_ms(uint8_t axis, float mm)
{
    float rpm  = ARM_GetJogRpm(axis);
    float lead = (axis == 0u) ? (float)ARM_SCREW_LEAD_MM : (float)ARM_RACK_LEAD_MM;
    float mmps;

    if (rpm < 1.0f) rpm = 1.0f;
    if (lead < 0.01f) lead = 1.0f;
    mmps = rpm * lead / 60.0f;              /* mm/s */
    if (mmps < 0.05f) mmps = 0.05f;
    return (uint32_t)((mm < 0.0f ? -mm : mm) / mmps * 1000.0f) + ST_STEP_SETTLE_MS;
}

/* 开跑：选表、复位 */
static void st_begin(uint8_t act)
{
    s_st_idx  = 0u;
    s_st_busy = 0u;
    s_st_tick = HAL_GetTick();

    if (act == (uint8_t)'C')      s_wk_kind = 0u;
    else if (act == (uint8_t)'M' || act == (uint8_t)'G') s_wk_kind = 1u;
    else                          s_wk_kind = 2u;

    s_gimbal_aim = 0u;
    Chassis_Stop();
}

static const st_step_t *st_table(uint8_t *n)
{
    if (s_wk_kind == 0u) { *n = (uint8_t)(sizeof(s_st_scan)/sizeof(s_st_scan[0]));   return s_st_scan; }
    if (s_wk_kind == 1u) { *n = (uint8_t)(sizeof(s_st_pick)/sizeof(s_st_pick[0]));   return s_st_pick; }
    *n = (uint8_t)(sizeof(s_st_place)/sizeof(s_st_place[0]));
    return s_st_place;
}

/* 每拍推进；返回 1 = 本工位全部做完 */
static uint8_t route_station_table(uint8_t act, uint32_t now)
{
    uint8_t n = 0u;
    const st_step_t *tb;
    const st_step_t *s;

    if (s_st_busy == 0u && s_st_idx == 0u && s_route_busy == 0u)
    {
        s_route_busy = 1u;
        st_begin(act);
    }

    tb = st_table(&n);
    if (s_st_idx >= n) return 1u;           /* 全部做完 */

    s = &tb[s_st_idx];

    /* ---- 第一步：下发 ---- */
    if (s_st_busy == 0u)
    {
        s_st_busy = 1u;
        s_st_tick = now;

        switch (s->kind)
        {
        case 'S':   /* 螺杆 */
            if (s->param != 0.0f) (void)ARM_Jog(0u, (int32_t)(s->param * (float)ARM_SCREW_PULSE_PER_MM));
            break;
        case 'R':   /* 旧齿条：已拆除，忽略 */
            break;
        case 'T':   /* 6020 转位（按 0~360 比方向，不绕线） */
            route_gimbal_goto(s->param);
            break;
        case 'G':   /* 夹爪 */
            SERVO_Gripper((s->param != 0.0f) ? 1u : 0u);
            break;
        case 'V':   /* 等视觉：按当前工位 act 切模式（M 单色 / 环 02） */
            route_vision_arm(act);
            break;
        default:    /* 'W' 等时间 */
            break;
        }
    }

    /* ---- 第二步：判断这一步做完没有 ---- */
    {
        uint8_t done = 0u;

        switch (s->kind)
        {
        case 'S': done = (uint8_t)((uint32_t)(now - s_st_tick) >= st_move_ms(0u, s->param)); break;
        case 'R': done = 1u; break; /* 伸缩已删 */
        case 'T': done = route_gimbal_done(now); break;
        case 'G': done = route_grip_done(now);   break;
        case 'V':
            {
                uint8_t arrived = Vision_IsArrived();
                uint8_t timed   = (uint8_t)(Vision_IsTimeout() ||
                                   ((uint32_t)(now - s_st_tick) >= (uint32_t)ROUTE_VISION_MS));
                if ((arrived != 0u) || (timed != 0u)) { s_gimbal_aim = 0u; done = 1u; }
            }
            break;
        default:
            done = (uint8_t)((uint32_t)(now - s_st_tick) >= (uint32_t)s->param);
            break;
        }

        if (done != 0u)
        {
            s_st_idx++;
            s_st_busy = 0u;
            if (s_st_idx >= n) return 1u;
        }
    }
    return 0u;
}

static uint8_t route_station(uint8_t act, uint32_t now)
{
#if !VISION_ENABLE
    if (s_route_busy == 0u)
    {
        s_route_busy = 1u;
        s_route_tick = now;
        Chassis_Stop();
    }
    return (uint8_t)((uint32_t)(now - s_route_tick) >= ROUTE_ACT_MS);
#else
    if (s_route_busy == 0u)
    {
        Chassis_Stop(); /* 定高：只停车对准，螺杆和齿条不下发位移 */
        s_route_busy = 1u;
        s_route_tick = now;
        s_wk_slot = 0u;
        s_wk_phase = 0u;
        s_gimbal_aim = 0u;
        if (act == (uint8_t)'C') s_wk_kind = 0u;
        else if ((act == (uint8_t)'M') || (act == (uint8_t)'G')) s_wk_kind = 1u;
        else s_wk_kind = 2u;
        if (s_wk_kind == 1u) route_gimbal_goto(ROT_HOME_DEG);
        else if (s_wk_kind == 2u) route_gimbal_goto(route_disc_deg(0u));
    }

    if (s_wk_kind == 0u)
    {
        char code[16];

        /* 任务码来自扫码相机 UART8（Scan_*），不再走视觉 UART7 */
        if (Scan_CopyCode(code, (uint8_t)sizeof(code)) != 0u)
        {
            uint8_t ti;
            for (ti = 0u; (ti < 15u) && (code[ti] != '\0'); ti++) s_task[ti] = code[ti];
            s_task[(ti < 15u) ? ti : 15u] = '\0';
            HMI_ShowPage(1u);
            HMI_ShowBox("plan.txt", code);
            HMI_ReportLine("SCAN");
            return 1u;
        }
        if ((uint32_t)(now - s_route_tick) >= (uint32_t)ROUTE_SCAN_MS)
        {
            Chassis_Stop();
            HMI_ReportLine("NOSCAN");
            s_route_phase = 2u;
        }
        return 0u;
    }

    /* 0：等到摆正（抓取）或物料盘（放置） */
    if (s_wk_phase == 0u)
    {
        if (route_gimbal_done(now) == 0u) return 0u;
        if (s_wk_kind == 1u)
        {
            route_send_vision(act);
            s_wk_phase = 1u;
        }
        else
        {
            SERVO_Gripper(0u);
            s_route_tick = now;
            s_wk_phase = 1u;
        }
        return 0u;
    }

    /* 1：抓取等视觉再合爪；放置等夹爪合严再去摆正 */
    if (s_wk_phase == 1u)
    {
        if (s_wk_kind == 1u)
        {
            uint8_t arrived = Vision_IsArrived();
            uint8_t timed = (uint8_t)(Vision_IsTimeout() ||
                             ((uint32_t)(now - s_route_tick) >= (uint32_t)ROUTE_VISION_MS));

            if ((arrived == 0u) && (timed == 0u)) return 0u;
            if (arrived != 0u)
            {
                if (s_grab < 9u) s_grab++;
                route_score();
            }
            s_gimbal_aim = 0u;
            SERVO_Gripper(0u);
            s_route_tick = now;
            s_wk_phase = 2u;
        }
        else
        {
            if (route_grip_done(now) == 0u) return 0u;
            route_gimbal_goto(ROT_HOME_DEG);
            s_route_tick = now;
            s_wk_phase = 2u;
        }
        return 0u;
    }

    /* 2：抓取等合爪再转向物料盘；放置等到摆正再开视觉 */
    if (s_wk_phase == 2u)
    {
        if (s_wk_kind == 1u)
        {
            if (route_grip_done(now) == 0u) return 0u;
            route_gimbal_goto(route_disc_deg(s_wk_slot));
            s_route_tick = now;
            s_wk_phase = 3u;
        }
        else
        {
            if (route_gimbal_done(now) == 0u) return 0u;
            route_send_vision(act);
            s_wk_phase = 3u;
        }
        return 0u;
    }

    /* 3：抓取等到物料盘再张开；放置等视觉再张开 */
    if (s_wk_phase == 3u)
    {
        if (s_wk_kind == 1u)
        {
            if (route_gimbal_done(now) == 0u) return 0u;
            SERVO_Gripper(1u);
            s_route_tick = now;
            s_wk_phase = 4u;
        }
        else
        {
            uint8_t arrived = Vision_IsArrived();
            uint8_t timed = (uint8_t)(Vision_IsTimeout() ||
                             ((uint32_t)(now - s_route_tick) >= (uint32_t)ROUTE_VISION_MS));

            if ((arrived == 0u) && (timed == 0u)) return 0u;
            if (arrived != 0u)
            {
                if (s_place < 9u) s_place++;
                route_score();
            }
            s_gimbal_aim = 0u;
            SERVO_Gripper(1u);
            s_route_tick = now;
            s_wk_phase = 4u;
        }
        return 0u;
    }

    if (route_grip_done(now) == 0u) return 0u;
    return route_piece_done(act, now);
#endif
}

/* 确认帧的字母：F 前进一段，L/R 转 90°，U 转 180°。行进时云台停在 ROT_TRAVEL_DEG。
   C 等任务码。M/G 各从场地抓 3 个放进物料盘，P/B 各从物料盘取出放到场地。 */
static void RouteRun_Task_Inner(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  act;
    uint8_t  done;
    float    dir;
    float    ang;
    float    err;
    uint32_t dur;

    if (s_route_phase == 2u) return;

    if (s_route_i >= s_route_n)
    {
        Chassis_Stop();
        s_route_phase = 2u;
#if HMI_REPORT_STATUS
        HMI_ReportLine("DONE");
#endif
        return;
    }

    if (s_route_phase == 1u)
    {
        if ((uint32_t)(now - s_route_tick) >= (uint32_t)ROUTE_GAP_MS)
        {
            s_route_i++;
            s_route_phase = 0u;
            s_route_busy  = 0u;
        }
        return;
    }

    act  = s_route[s_route_i];
    done = 0u;

    if ((act == (uint8_t)'C') || (act == (uint8_t)'M') || (act == (uint8_t)'P') ||
        (act == (uint8_t)'G') || (act == (uint8_t)'B'))
    {
#if STATION_USE_TABLE
        if (route_station_table(act, now) != 0u) done = 1u;
#else
        if (route_station(act, now) != 0u) done = 1u;
#endif
    }
    else if (act == (uint8_t)'F')
    {
        if (s_route_busy == 0u)
        {
            s_route_busy = 1u;
            s_route_tick = now;
            route_gimbal_goto(ROT_TRAVEL_DEG);
            Chassis_SetHeadingTargetDeg(Chassis_GetYawDeg() + Chassis_GetStraightTrimDeg());   /* ★直行前锁定当前朝向 */
            Chassis_SetVelocity(ROUTE_SPEED_MMPS, 0.0f, 0.0f);
        }
        if ((uint32_t)(now - s_route_tick) >= ROUTE_FWD_MS) done = 1u;
    }
    else if ((act == (uint8_t)'L') || (act == (uint8_t)'R') || (act == (uint8_t)'U'))
    {
        dir = (act == (uint8_t)'R') ? -1.0f : 1.0f;
        ang = (act == (uint8_t)'U') ? 180.0f : 90.0f;
        dur = (act == (uint8_t)'U') ? ROUTE_TURN180_MS : ROUTE_TURN90_MS;
        if (s_route_busy == 0u)
        {
            s_route_busy = 1u;
            s_route_tick = now;
            s_route_deg0 = Chassis_GetYawDeg();
            s_route_imu  = IMU_IsOnline();
            route_gimbal_goto(ROT_TRAVEL_DEG);
            Chassis_SetVelocity(0.0f, 0.0f, dir * ROUTE_TURN_WZ);
        }
        if (s_route_imu != 0u)
        {
            err = route_wrap(Chassis_GetYawDeg() - (s_route_deg0 + dir * ang));
            if ((err <= ROUTE_TURN_TOL_DEG) && (err >= -ROUTE_TURN_TOL_DEG)) done = 1u;
        }
        if ((uint32_t)(now - s_route_tick) >= dur) done = 1u;
    }
    else
    {
        Chassis_Stop();
        s_route_phase = 2u;
#if HMI_REPORT_STATUS
        HMI_ReportLine("BAD");
#endif
        return;
    }

    if (done != 0u) route_advance(now);
}
#endif /* CHASSIS_ENABLE && HMI_ENABLE */
/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/* 出发：按当前障碍重算路线并开始跑。返回 1 = 有路可走 */
uint8_t RouteRun_Start(void)
{
    uint8_t n;

    Chassis_Stop();
    Chassis_ResetYaw();                 /* 当前车头记为 0°，路线里的转弯都相对它 */
#if SERVO_ENABLE
    SERVO_Gripper(1u);                  /* 发车前夹爪张开 */
#endif
    (void)HMI_LoadPlannedPath();        /* 按屏幕上报的障碍算路 */
    n = HMI_CopyPath(s_route, (uint8_t)sizeof(s_route));

    if (n == 0u)
    {
        s_route_n     = 0u;
        s_route_phase = 2u;
        s_route_active = 0u;
        return 0u;
    }

    s_route_n      = n;
    s_route_i      = 0u;
    s_route_phase  = 0u;
    s_route_busy   = 0u;
    s_route_active = 1u;
    route_match_reset();
    return 1u;
}

/* 取消：停车、清空路线 */
void RouteRun_Stop(void)
{
    Chassis_Stop();
    s_route_n      = 0u;
    s_route_i      = 0u;
    s_route_phase  = 2u;
    s_route_busy   = 0u;
    s_route_active = 0u;
}

uint8_t RouteRun_IsActive(void)
{
    return s_route_active;
}

/* ★调试/可视化：读出路线内容与当前进度
   n = 总步数；idx = 当前第几步（0 起）；phase = 0 正在做 / 1 步间停顿 / 2 已结束 */
void RouteRun_Info(uint8_t *n, uint8_t *idx, uint8_t *phase, const uint8_t **letters)
{
    if (n != 0)       *n       = s_route_n;
    if (idx != 0)     *idx     = s_route_i;
    if (phase != 0)   *phase   = s_route_phase;
    if (letters != 0) *letters = s_route;
}

uint8_t RouteRun_Remaining(void)
{
    if (s_route_i >= s_route_n) return 0u;
    return (uint8_t)(s_route_n - s_route_i);
}

/* 主循环里每拍调用 */
void RouteRun_Task(void)
{
    if (s_route_active == 0u) return;
    RouteRun_Task_Inner();
}

#else   /* 没开底盘或屏幕：空实现，main.c 不用改 */

uint8_t RouteRun_Start(void)      { return 0u; }
void    RouteRun_Stop(void)       { }
uint8_t RouteRun_IsActive(void)   { return 0u; }
/* ★调试/可视化：读出路线内容与当前进度
   n = 总步数；idx = 当前第几步（0 起）；phase = 0 正在做 / 1 步间停顿 / 2 已结束 */
void RouteRun_Info(uint8_t *n, uint8_t *idx, uint8_t *phase, const uint8_t **letters)
{
    if (n != 0)       *n       = s_route_n;
    if (idx != 0)     *idx     = s_route_i;
    if (phase != 0)   *phase   = s_route_phase;
    if (letters != 0) *letters = s_route;
}

uint8_t RouteRun_Remaining(void)  { return 0u; }
void    RouteRun_Task(void)       { }

#endif