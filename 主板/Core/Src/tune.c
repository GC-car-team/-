/**
  ******************************************************************************
  * @file    tune.c
  * @brief   调参命令通道实现（命令解析 + 小动作状态机 + 文本遥测）
  *
  *  设计要点：
  *    · 一切"小动作"都带**超时兜底**：跑不到目标也会自己停（场地安全第一）
  *    · 动作完成判据用 **IMU yaw**（直线/转角都是真值）；平移用轮式里程计（会打滑，见注释）
  *    · 遥测走文本 CSV：PC 侧解析零歧义（JustFloat 也能解，但文本还能肉眼看）
  *    · 打开遥测时会 **关掉 VOFA JustFloat**（同一个串口，两种协议不能混着发）
  ******************************************************************************
  */
#include "tune.h"

#if TUNE_ENABLE

#include "usart.h"
#include "vofa.h"
#include "chassis.h"
#include "imu.h"
#include "servo.h"   /* 总线舵机夹爪 U/O/C */
#if ARM_ENABLE
#include "arm.h"
#include "gm6020.h"
#include "pd42s1.h"     /* L/N/A/V/E 命令：臂两轴的位置/零点/绝对移动/速度/使能 */        /* J 命令：升降臂点动测试（ARM_Jog / ARM_GetCanExtCount）*/
#endif
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ====================================================================
  *  1. 状态
  * ==================================================================== */
static UART_HandleTypeDef *s_uart;      /* TUNE_UART（有线串口2 或 蓝牙串口4） */
static uint8_t  s_rx_byte;
static char     s_line[TUNE_LINE_MAX];
static uint8_t  s_line_n;

typedef enum { TUNE_IDLE = 0, TUNE_GO, TUNE_TURN, TUNE_STRAFE } tune_act_t;

static tune_act_t s_act;
static uint32_t s_act_tick;             /* 本动作开始时刻 */
static float    s_go_start_mm;          /* G 动作：起点（前向位移） */
static float    s_go_target_mm;         /* G 动作：要走多少（带符号） */
static float    s_strafe_start_mm;      /* S 动作：起点（横向位移） */
static float    s_strafe_target_mm;     /* S 动作：要走多少（带符号，左正） */
static float    s_turn_start_deg;       /* T 动作：起点朝向 */
static float    s_turn_target_deg;      /* T 动作：相对转角（带符号） */

static uint8_t  s_telem;                /* 1 = 正在发文本遥测 */
static uint32_t s_telem_tick;
static uint16_t s_telem_hz = TUNE_TELEM_HZ;
static float    s_arm_rpm[2] = { 120.0f, 120.0f };   /* 臂两轴的运动速度（V 命令可改） */   /* 运行时可用 R<Hz> 改（蓝牙 9600 建议 5） */
static float    s_remote_speed = REMOTE_SPEED_MMPS;   /* ★遥控：当前速度 */
static uint32_t s_remote_tick;                        /* ★遥控：最后按键时刻 */
static uint8_t  s_remote_active;                      /* ★遥控：1 = 正在驱动 */
static uint32_t s_cmd_cnt;
static uint32_t s_act_cnt;              /* 跑过多少次动作 */
static uint32_t s_last_act_ms;          /* 上一次动作耗时（PC 判"快慢"用） */

/* ★动作结束后 1.5 秒再测一次"真正停稳后的角度"
   为什么要：判定到位那一刻车还在动/还没回弹（机械背隙），那时打印的误差是假的；
   停稳后再用 IMU 原始 yaw 量一次，才是车真正停在哪。 */
static uint8_t  s_settle_wait;          /* 1 = 正在等车停稳 */
static uint32_t s_settle_tick;
static float    s_settle_start;         /* 动作起点角/位移 */
static float    s_settle_target;        /* 目标增量 */
static uint8_t  s_settle_kind;          /* 'T' / 'G' / 'S' */
static uint8_t  s_vofa_was_on;

/* ★CAN 状态（放在遥测行末尾，判断臂的 CAN 驱动器回不回）
   EXT > 0 = 收到扩展帧（驱动器回了！）；TEC 一直涨 = 没人 ACK */
#if ARM_ENABLE
static uint16_t tune_can_ext_cnt(void) { return ARM_GetCanExtCount(); }
static uint16_t tune_can_tec(void)
{
    uint16_t tec = 0u, rec = 0u;
    uint8_t  lec = 0u;

    ARM_GetCanErrors(&tec, &rec, &lec);
    return tec;
}
static uint16_t tune_can_tx_ok(void) { return ARM_GetCanTxOk(); }
#else
static uint16_t tune_can_ext_cnt(void) { return 0u; }
static uint16_t tune_can_tec(void)     { return 0u; }
static uint16_t tune_can_tx_ok(void)   { return 0u; }
#endif

/* ====================================================================
  *  2. 发送
  * ==================================================================== */
static void tune_send(const char *s)
{
    if ((s_uart == 0) || (s == 0)) return;
    (void)HAL_UART_Transmit(s_uart, (uint8_t *)s, (uint16_t)strlen(s), 30u);
}

static void tune_printf(const char *fmt, ...)
{
    char buf[TUNE_TXT_MAX];
    va_list ap;
    int n;

    if (s_uart == 0) return;
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    /* ★超时必须按长度算：9600 波特下 1 个字节约 1.05ms，
       以前固定 30ms → 超过 28 字符就被截断（遥测行只发出前 6 个字段）。
       这里给 len/8 + 60 ms 的余量（9600 时约 1.1ms/字符）。 */
    {
        uint32_t to = (uint32_t)(n) / 8u + 60u;
        if (to > 400u) to = 400u;
        (void)HAL_UART_Transmit(s_uart, (uint8_t *)buf, (uint16_t)n, to);
    }
}

/* ====================================================================
  *  3. 动作：停 / 走 / 转 / 平移
  * ==================================================================== */
static void tune_stop_all(void)
{
    Chassis_TurnStop();
    Chassis_StrafeStop();
    Chassis_Stop();
    s_act = TUNE_IDLE;
}

static void tune_start_go(float mm)
{
    tune_stop_all();
    s_go_start_mm  = Chassis_GetForwardMm();
    s_go_target_mm = mm;
    s_act_tick     = HAL_GetTick();
    s_act          = TUNE_GO;
    Chassis_SetHeadingTargetDeg(Chassis_GetYawDeg() + Chassis_GetStraightTrimDeg());   /* ★直行前锁定当前朝向 */
    Chassis_SetVelocity((mm >= 0.0f) ? TUNE_CRUISE_SPEED : -TUNE_CRUISE_SPEED, 0.0f, 0.0f);
}

static void tune_start_strafe(float mm)
{
    tune_stop_all();
    s_strafe_start_mm  = Chassis_GetSideMm();
    s_strafe_target_mm = mm;
    s_act_tick         = HAL_GetTick();
    s_act              = TUNE_STRAFE;
    /* 平移环自己去修横向位移（左正右负，和 Chassis_StrafeStart 的约定一致） */
    Chassis_StrafeStart(0.0f, mm);
}

static void tune_start_turn(float deg)
{
    tune_stop_all();
    s_turn_start_deg  = Chassis_GetYawDeg();
    s_turn_target_deg = deg;
    s_act_tick        = HAL_GetTick();
    s_act             = TUNE_TURN;
    /* ⚠️ Chassis_TurnStart 是【绝对】目标角 → 这里加当前朝向，做成"相对转 deg 度" */
    Chassis_TurnStart(s_turn_start_deg + deg);
}

/* 一个动作是不是"到了" */
static uint8_t tune_act_done(void)
{
    switch (s_act)
    {
    case TUNE_GO:
        return (float)fabsf(Chassis_GetForwardMm() - s_go_start_mm) >= fabsf(s_go_target_mm) - TUNE_GO_TOL_MM;
    case TUNE_STRAFE:
        return (float)fabsf(Chassis_GetSideMm() - s_strafe_start_mm) >= fabsf(s_strafe_target_mm) - TUNE_STRAFE_TOL_MM;
    case TUNE_TURN:
        return (Chassis_TurnIsDone() != 0u) ||
               ((float)fabsf(Chassis_GetYawDeg() - (s_turn_start_deg + s_turn_target_deg)) <= TUNE_TURN_TOL_DEG);
    default:
        return 1u;
    }
}

/* ★动作结束打印一行"结果行"：每个动作只产生一行，丢包也不影响判断
   转向： R,T,<D/T>,<起点角>,<相对目标>,<最终yaw>,<误差>,<耗时ms>
   走直： R,G,<D/T>,<起点mm>,<目标mm>,<最终mm>,<误差mm>,<耗时ms>
   平移： R,S,<D/T>,<起点mm>,<目标mm>,<最终mm>,<误差mm>,<耗时ms> */
static float tune_wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void tune_report_result(uint8_t act, char why)
{
    switch (act)
    {
    case TUNE_TURN:
        {
            float fin = Chassis_GetYawDeg();
            float err = tune_wrap180(fin - (s_turn_start_deg + s_turn_target_deg));

            tune_printf("R,T,%c,%.1f,%.1f,%.1f,%.1f,%lu\r\n", why,
                        (double)s_turn_start_deg, (double)s_turn_target_deg,
                        (double)fin, (double)err, (unsigned long)s_last_act_ms);
        }
        break;

    case TUNE_GO:
        {
            float fin = Chassis_GetForwardMm();

            tune_printf("R,G,%c,%.0f,%.0f,%.0f,%.0f,%lu\r\n", why,
                        (double)s_go_start_mm, (double)s_go_target_mm,
                        (double)fin, (double)(fin - (s_go_start_mm + s_go_target_mm)),
                        (unsigned long)s_last_act_ms);
        }
        break;

    case TUNE_STRAFE:
        {
            float fin = Chassis_GetSideMm();

            tune_printf("R,S,%c,%.0f,%.0f,%.0f,%.0f,%lu\r\n", why,
                        (double)s_strafe_start_mm, (double)s_strafe_target_mm,
                        (double)fin, (double)(fin - (s_strafe_start_mm + s_strafe_target_mm)),
                        (unsigned long)s_last_act_ms);
        }
        break;

    default:
        break;
    }
}

/* ====================================================================
  *  4. 参数设置 / 打印
  * ==================================================================== */
static uint8_t loop_of(char c)
{
    if (c == 'h') return 0u;
    if (c == 't') return 1u;
    if (c == 's') return 2u;
    return 0xFFu;
}

static void tune_print_params(void)
{
    static const char *name[3] = { "h", "t", "s" };

    for (uint8_t i = 0u; i < 3u; i++)
    {
        tune_printf("P,%s,p,%.4f\r\n",   name[i], Chassis_PidGetGain(i, 0u));
        tune_printf("P,%s,i,%.4f\r\n",   name[i], Chassis_PidGetGain(i, 1u));
        tune_printf("P,%s,d,%.4f\r\n",   name[i], Chassis_PidGetGain(i, 2u));
        tune_printf("P,%s,db,%.4f\r\n",  name[i], Chassis_PidGetParam(i, 0u));
        tune_printf("P,%s,om,%.4f\r\n",  name[i], Chassis_PidGetParam(i, 1u));
        tune_printf("P,%s,im,%.4f\r\n",  name[i], Chassis_PidGetParam(i, 2u));
    }
    tune_printf("P,t,arr,%.4f\r\n", Chassis_PidGetParam(1u, 3u));
    tune_printf("P,t,set,%.4f\r\n", Chassis_PidGetParam(1u, 4u));
    tune_printf("P,s,arr,%.4f\r\n", Chassis_PidGetParam(2u, 3u));
    /* ★诊断：估计出的零漂率（度/分）+ yaw 标度 */
    tune_printf("P,drift_dpm,%.2f\r\n", (double)Chassis_GetDriftDpm());

    /* ★IMU 健康度：在线 / 收帧数 / 坏帧数 —— 判断 yaw 跳变是不是丢帧造成的 */
    tune_printf("IMU,on=%u,frames=%lu,bad=%lu,algo=%u\r\n",
                (unsigned)IMU_IsOnline(),
                (unsigned long)IMU_GetFrames32(),
                (unsigned long)IMU_GetBadFrames(),
                (unsigned)IMU_GetScanResult());
    tune_printf("P,yaw_scale,%.4f\r\n", (double)Chassis_GetYawScale());
    /* 总线舵机当前位置 P */
    tune_printf("S,pos,%u\r\n", (unsigned)SERVO_GetPos());
    tune_printf("S,busy,%u\r\n", (unsigned)SERVO_IsBusy());
    tune_send("P,END,0,0\r\n");
}

/* "P t p 3.0" → 改参数。返回 1 = 成功 */
static uint8_t tune_set_param(char *rest)
{
    char *a = strtok(rest, " \t");      /* 环 */
    char *b = strtok(0, " \t");         /* 项 */
    char *c = strtok(0, " \t");         /* 值 */
    uint8_t loop;
    float v;

    if ((a == 0) || (b == 0) || (c == 0)) return 0u;
    loop = loop_of(a[0]);
    if (loop == 0xFFu) return 0u;
    v = (float)atof(c);

    if      (strcmp(b, "p") == 0)   Chassis_PidSetGain(loop, 0u, v);
    else if (strcmp(b, "i") == 0)   Chassis_PidSetGain(loop, 1u, v);
    else if (strcmp(b, "d") == 0)   Chassis_PidSetGain(loop, 2u, v);
    else if (strcmp(b, "db") == 0)  Chassis_PidSetParam(loop, 0u, v);
    else if (strcmp(b, "om") == 0)  Chassis_PidSetParam(loop, 1u, v);
    else if (strcmp(b, "im") == 0)  Chassis_PidSetParam(loop, 2u, v);
    else if (strcmp(b, "arr") == 0) Chassis_PidSetParam(loop, 3u, v);
    else if (strcmp(b, "set") == 0) Chassis_PidSetParam(loop, 4u, v);
    else return 0u;

    tune_printf("OK,P,%c,%s,%.4f\r\n", a[0], b, v);
    return 1u;
}

/* ====================================================================
  *  5. 命令解析（一行）
  * ==================================================================== */
static void tune_handle_line(char *ln)
{
    char c;

    /* 去掉行尾的 \r 和空白 */
    while ((*ln == ' ') || (*ln == '\t')) ln++;
    if (*ln == 0) return;

    c = ln[0];
    s_cmd_cnt++;

    switch (c)
    {
    case 'Z':                                   /* 归零 + 测漂移 */
        tune_stop_all();
        Chassis_ResetYaw();
        tune_send("OK,Z\r\n");
        break;

    case 'R':                                   /* 软复位 */
        tune_stop_all();
        Chassis_ResetYaw();
        tune_send("OK,R\r\n");
        break;

    case 'G':
        tune_start_go((float)atof(ln + 1));
        tune_printf("OK,G,%.1f\r\n", (double)atof(ln + 1));
        break;

    case 'S':
        tune_start_strafe((float)atof(ln + 1));
        tune_printf("OK,S,%.1f\r\n", (double)atof(ln + 1));
        break;

    case 'T':
        tune_start_turn((float)atof(ln + 1));
        tune_printf("OK,T,%.1f\r\n", (double)atof(ln + 1));
        break;

    case 'X':
        tune_stop_all();
        tune_send("OK,X\r\n");
        break;

    case 'P':
        if (tune_set_param(ln + 1) == 0u) tune_send("ERR,P\r\n");
        break;

    case 'Q':
        tune_print_params();
        break;

    case 'F':                                   /* F<Hz>：运行时改遥测频率（蓝牙 9600 建议 5） */        {
            int hz = atoi(ln + 1);

            if (hz < 1)  hz = 1;
            if (hz > 50) hz = 50;
            s_telem_hz = (uint16_t)hz;
            tune_printf("OK,F,%d\r\n", hz);
        }
        break;

    case 'W':                                   /* W<系数>：设 yaw 标度修正（如 W1.0286） */
        {
            float k = (float)atof(ln + 1);

            if (k > 0.0f) { Chassis_SetYawScale(k); }
            tune_printf("OK,W,%.4f\r\n", (double)Chassis_GetYawScale());
        }
        break;

    case 'M':
        s_telem = (ln[1] == '1') ? 1u : 0u;
        if (s_telem != 0u)        {
            s_vofa_was_on = VOFA_IsEnable();
            VOFA_SetEnable(0u);                 /* 同一个串口：先关 JustFloat，再发文本 */
            s_telem_tick = HAL_GetTick();
            tune_send("OK,M1\r\n");
        }
        else
        {
            VOFA_SetEnable(s_vofa_was_on);
            tune_send("OK,M0\r\n");
        }
        break;

    /* ---------------- 夹爪总线舵机（config.h ⑰）---------------- */
    case 'D':                                   /* D<0~180>：映射到 P_MIN~P_MAX（调试用） */
        {
            float deg = (float)atof(ln + 1);
            if (deg < 0.0f)   deg = 0.0f;
            if (deg > 180.0f) deg = 180.0f;
            SERVO_SetDeg(deg);
            tune_printf("OK,D,%.1f,P%u\r\n", (double)deg, (unsigned)SERVO_GetPos());
        }
        break;

    case 'U':                                   /* U<P>：直接写总线位置 500~2500 */
        SERVO_SetPos((uint16_t)atoi(ln + 1), (uint16_t)BUS_SERVO_MOVE_MS);
        tune_printf("OK,U,P%u\r\n", (unsigned)SERVO_GetPos());
        break;

    case 'O':                                   /* O：张开 */
        SERVO_Gripper(1u);
        tune_printf("OK,O,P%u\r\n", (unsigned)SERVO_GetPos());
        break;

    case 'C':                                   /* C：夹紧 */
        SERVO_Gripper(0u);
        tune_printf("OK,C,P%u\r\n", (unsigned)SERVO_GetPos());
        break;

    case 'K':                                   /* K：旧 PWM 扫描已废除 */
        tune_printf("ERR,K,removed\r\n");
        break;

#if ARM_ENABLE
    /* J<轴><脉冲>  点动测试：J1 20000 = 轴①（螺杆）正转 20000 脉冲；J1 -20000 反向。
       这是"验证驱动器收不收帧"最直接的办法（相对位移，不依赖当前零点）*/
    case 'J':
    {
        uint8_t axis = (ln[1] == '2') ? 1u : 0u;
        long    pls  = (long)atof(ln + 2);

        if (ARM_Jog(axis, (int32_t)pls) != 0u)
        {
            tune_printf("OK,J,%u,%ld,%u\r\n", (unsigned)axis, pls,
                        (unsigned)ARM_GetCanExtCount());
        }
        else
        {
            tune_send("ERR,J\r\n");
        }
        break;
    }

    /* ★===== 升降臂调试 / 定标命令（2026-10-21 新增）=====
       轴1 = 螺杆（地址5）  轴2 = 齿条（地址6），都走和底盘同一个串口总线。
       L            读两轴当前位置（脉冲 + 是否有效 + 转速）
       N1 / N2      把当前位置设为零点（0xF8）→ 定标用
       A1 <脉冲>    绝对位置移动（0xF2，带符号）
       V1 <rpm>     设该轴的运动速度（影响 A 命令）
       E1 0 / E1 1  该轴失能 / 使能（0xFA）
       Y<角度>      GM6020（旋转）目标角度 0~360
       H            6020 状态（在线/当前角/到位）
       ================================================== */
    case 'L':
    {
        const pd42s1_t *s1 = PD42S1_GetMotorState((uint8_t)ARM_AXIS1_IDX);
        const pd42s1_t *s2 = PD42S1_GetMotorState((uint8_t)ARM_AXIS2_IDX);

        /* 格式：POS,<轴>,<位置>,<位置有效>,<转速>,<在线>,<模式>,<应答条数>
           模式（手册 0x62）：0 = 通信位置 / 1 = 通信速度 / 2 = 脉冲 ... */
        tune_printf("POS,1,%ld,%u,%d,%u,%u,%u\r\n",
                    (long)((s1 != 0) ? s1->pos : 0),
                    (unsigned)((s1 != 0) ? s1->pos_valid : 0u),
                    (int)((s1 != 0) ? s1->rpm : 0),
                    (unsigned)((s1 != 0) ? s1->online : 0u),
                    (unsigned)((s1 != 0) ? s1->mode : 0u),
                    (unsigned)((s1 != 0) ? s1->rx_ok : 0u));
        tune_printf("POS,2,%ld,%u,%d,%u,%u,%u\r\n",
                    (long)((s2 != 0) ? s2->pos : 0),
                    (unsigned)((s2 != 0) ? s2->pos_valid : 0u),
                    (int)((s2 != 0) ? s2->rpm : 0),
                    (unsigned)((s2 != 0) ? s2->online : 0u),
                    (unsigned)((s2 != 0) ? s2->mode : 0u),
                    (unsigned)((s2 != 0) ? s2->rx_ok : 0u));
        /* ★虚拟位置（累计下发脉冲）与毫米换算：驱动器不回读时看这个 */
        tune_printf("VPOS,1,%ld,%.2fmm\r\n", (long)ARM_GetVirtPos(0),
                    (double)((float)ARM_GetVirtPos(0) / (float)ARM_SCREW_PULSE_PER_MM));
        tune_printf("VPOS,2,%ld,%.2fmm\r\n", (long)ARM_GetVirtPos(1),
                    (double)((float)ARM_GetVirtPos(1) / (float)ARM_RACK_PULSE_PER_MM));
        tune_send("POS,END,0,0,0,0,0,0\r\n");
        break;
    }

    case 'N':
    {
        uint8_t axis = (ln[1] == '2') ? 1u : 0u;
        uint8_t addr = (axis == 0u) ? (uint8_t)ARM_NODE_1 : (uint8_t)ARM_NODE_2;

        PD42S1_AngleZero(addr);                       /* 0xF8：当前位置清零 */
        tune_printf("OK,N,%u,%u\r\n", (unsigned)(axis + 1u), (unsigned)addr);
        break;
    }

    case 'I':                                   /* I<轴> <mm>：按【毫米】相对移动（螺杆/齿条定标用）
                                                   例：I1 10  = 螺杆上升 10mm
                                                       I2 -5  = 齿条缩回 5mm */
        {
            uint8_t  axis = (ln[1] == '2') ? 1u : 0u;
            float    mm   = (float)atof(ln + 2);
            float    k    = (axis == 0u) ? (float)ARM_SCREW_PULSE_PER_MM
                                         : (float)ARM_RACK_PULSE_PER_MM;
            long     pls  = (long)(mm * k);

            if (pls == 0) { tune_send("ERR,I,0\r\n"); break; }
            if (pls >  500000) pls =  500000;
            if (pls < -500000) pls = -500000;

            if (ARM_Jog(axis, (int32_t)pls) != 0u)
            {
                tune_printf("OK,I,%u,%.2f,%ld\r\n", (unsigned)(axis + 1u), (double)mm, pls);
            }
            else
            {
                tune_send("ERR,I\r\n");
            }
        }
        break;

    case 'A':
    {
        uint8_t axis = (ln[1] == '2') ? 1u : 0u;
        long    pls  = (long)atof(ln + 2);
        uint8_t addr = (axis == 0u) ? (uint8_t)ARM_NODE_1 : (uint8_t)ARM_NODE_2;
        uint16_t rpm = (uint16_t)((s_arm_rpm[axis] < 1.0f) ? 1.0f : s_arm_rpm[axis]);
        uint8_t  dir = (pls < 0) ? 1u : 0u;
        uint32_t mag = (uint32_t)((pls < 0) ? -pls : pls);

        PD42S1_PosModeAbs(addr, dir, ARM_CAN_ACC, rpm, mag);   /* 0xF2 绝对位置 */
        tune_printf("OK,A,%u,%ld,%u\r\n", (unsigned)(axis + 1u), pls, (unsigned)rpm);
        break;
    }

    case 'V':
    {
        uint8_t axis = (ln[1] == '2') ? 1u : 0u;
        float   rpm  = (float)atof(ln + 2);

        if (rpm < 1.0f)   rpm = 1.0f;
        if (rpm > 300.0f) rpm = 300.0f;
        s_arm_rpm[axis] = rpm;
        ARM_SetJogRpm(axis, rpm);      /* ★同时改 ARM_Jog 实际用的速度 */
        tune_printf("OK,V,%u,%.0f\r\n", (unsigned)(axis + 1u), (double)rpm);
        break;
    }

    case 'E':
    {
        uint8_t axis = (ln[1] == '2') ? 1u : 0u;
        uint8_t idx  = (axis == 0u) ? (uint8_t)ARM_AXIS1_IDX : (uint8_t)ARM_AXIS2_IDX;
        uint8_t en   = (ln[2] == '0') ? 0u : 1u;

        PD42S1_MotorEnable(idx, en);                   /* 0xFA 使能/失能 */
        tune_printf("OK,E,%u,%u\r\n", (unsigned)(axis + 1u), (unsigned)en);
        break;
    }

    case 'Y':                                          /* Y<角度> [方向]：转向目标角
                                                          方向省略/0 = 最短路径
                                                          1  = 角度递增（回初始位的绕线保护方向）
                                                         -1 = 角度递减 */
        {
            float  deg = (float)atof(ln + 1);
            char  *p   = strchr(ln + 1, ' ');
            int8_t dir = 0;

            if (p != 0) dir = (int8_t)atoi(p + 1);

            GM6020_SetTargetPosDegDir(deg, dir);
            tune_printf("OK,Y,%.1f,%d\r\n", (double)GM6020_GetTargetDeg(), (int)GM6020_GetSlewDir());
        }
        break;

    case 'H':                                          /* 6020 状态 + CAN 全链路诊断 */
        tune_printf("G6020,%u,%.1f,%.1f,%d,id=%lu,rx=%u,tx=%u,fail=%u,loop=%u,start=%u\r\n",
                    (unsigned)GM6020_IsOnline(),
                    (double)GM6020_GetCurrentDeg(),
                    (double)GM6020_GetTargetDeg(),
                    (int)GM6020_GetCurrentOutput(),
                    (unsigned long)GM6020_GetLastRxId(),
                    (unsigned)GM6020_GetRxCount(),
                    (unsigned)GM6020_GetTxOkCount(),
                    (unsigned)GM6020_GetTxFailCount(),
                    (unsigned)GM6020_GetLoopCount(),
                    (unsigned)GM6020_GetCanStartOk());
        break;
#endif

    /* ★小写命令（大写字母已占满，小写留给"只读查询"，上位机可视化用） */
    case 'r':                                   /* r：路线内容 + 进度 */
        {
            uint8_t n = 0u, idx = 0u, ph = 2u;
            const uint8_t *L = 0;
            uint8_t k;

            RouteRun_Info(&n, &idx, &ph, &L);
            {
                char buf[40];
                uint8_t m = 0u;

                for (k = 0u; (k < n) && (L != 0) && (m < (uint8_t)(sizeof(buf) - 1u)); k++)
                {
                    buf[m++] = (char)L[k];
                }
                buf[m] = 0;
                tune_printf("RT,%u,%u,%u,%s\r\n", (unsigned)n, (unsigned)idx, (unsigned)ph, buf);
            }
        }
        break;

    case 'o':                                   /* o：出发点 + 障碍掩码 */
        {
            uint8_t  up = 0u;
            uint16_t on = 0u, oe = 0u;

            HMI_GetObs(&up, &on, &oe);
            tune_printf("OB,%u,%u,%u\r\n", (unsigned)up, (unsigned)on, (unsigned)oe);
        }
        break;

    case 't':                                   /* t<轮号 1~4> <系数>：每轮速度系数（走直补偿）
                                                   t1 0.98 = 左前乘 0.98 ； t = 打印四个当前值 */
        if ((ln[1] >= '1') && (ln[1] <= '4'))
        {
            uint8_t idx = (uint8_t)(ln[1] - '1');
            float   k   = (float)atof(ln + 2);

            if (k > 0.5f) Chassis_SetWheelTrim(idx, k);
            tune_printf("OK,t,%u,%.3f\r\n", (unsigned)(idx + 1u), (double)Chassis_GetWheelTrim(idx));
        }
        else
        {
            tune_printf("TRIM,%.3f,%.3f,%.3f,%.3f\r\n",
                        (double)Chassis_GetWheelTrim(0u), (double)Chassis_GetWheelTrim(1u),
                        (double)Chassis_GetWheelTrim(2u), (double)Chassis_GetWheelTrim(3u));
        }
        break;
#if REMOTE_ENABLE
    /* ---------------- ★手机蓝牙遥控（单键，松手自动停）---------------- */
    case 'w':   /* 前进 */
    case 's':   /* 后退 */
    case 'a':   /* 左转 */
    case 'd':   /* 右转 */
    case 'q':   /* 左前 */
    case 'e':   /* 右前 */
        {
            float v = s_remote_speed;
            float w = (float)REMOTE_TURN_DPS;
            float vx = 0.0f, wz = 0.0f;

            if (c == 'w')      { vx =  v; }
            else if (c == 's') { vx = -v; }
            else if (c == 'a') { wz =  w; }
            else if (c == 'd') { wz = -w; }
            else if (c == 'q') { vx =  v; wz =  w; }
            else               { vx =  v; wz = -w; }

            Chassis_SetHeadingTargetDeg(Chassis_GetYawDeg());   /* 遥控时也锁定航向 */
            Chassis_SetVelocity(vx, 0.0f, wz);
            s_remote_tick  = HAL_GetTick();
            s_remote_active = 1u;
            tune_printf("RM,%c,%.0f\r\n", c, (double)v);
        }
        break;

    case '+':   /* 加速 */
        s_remote_speed += 25.0f;
        if (s_remote_speed > (float)REMOTE_SPEED_MAX) s_remote_speed = (float)REMOTE_SPEED_MAX;
        tune_printf("RM,+,%.0f\r\n", (double)s_remote_speed);
        break;

    case '-':   /* 减速 */
        s_remote_speed -= 25.0f;
        if (s_remote_speed < (float)REMOTE_SPEED_MIN) s_remote_speed = (float)REMOTE_SPEED_MIN;
        tune_printf("RM,-,%.0f\r\n", (double)s_remote_speed);
        break;

    case 'j':   /* 夹爪张开 */
        SERVO_Gripper(1u);
        tune_send("RM,j\r\n");
        break;

    case 'k':   /* 夹爪合紧 */
        SERVO_Gripper(0u);
        tune_send("RM,k\r\n");
        break;

    case 'u':   /* 螺杆上升 5mm */
        (void)ARM_Jog(0u, (int32_t)(5.0f * (float)ARM_SCREW_PULSE_PER_MM));
        tune_send("RM,u\r\n");
        break;

    case 'n':   /* 螺杆下降 5mm */
        (void)ARM_Jog(0u, -(int32_t)(5.0f * (float)ARM_SCREW_PULSE_PER_MM));
        tune_send("RM,n\r\n");
        break;

    case 'l':   /* 齿条伸出 10mm */
        (void)ARM_Jog(1u, (int32_t)(10.0f * (float)ARM_RACK_PULSE_PER_MM));
        tune_send("RM,l\r\n");
        break;

    case 'm':   /* 齿条缩回 10mm */
        (void)ARM_Jog(1u, -(int32_t)(10.0f * (float)ARM_RACK_PULSE_PER_MM));
        tune_send("RM,m\r\n");
        break;

    case '1':   /* 6020 到盘1 */
    case '2':   /* 盘2 */
    case '3':   /* 盘3 */
    case '0':   /* 回初始位 */
        {
            float deg = (c == '1') ? (float)ROT_DISC1_DEG
                      : (c == '2') ? (float)ROT_DISC2_DEG
                      : (c == '3') ? (float)ROT_DISC3_DEG
                      : (float)ROT_HOME_DEG;

            GM6020_SetTargetPosDegDir(deg, 1);      /* 递增方向，防绕线 */
            tune_printf("RM,%c,%.1f\r\n", c, (double)deg);
        }
        break;
#endif /* REMOTE_ENABLE */
    case 'c':                                   /* c<值>：侧滑补偿系数（前进时叠加 vy = vx × 值）
                                                   c0.06 / c-0.06 ；c 单独发 = 打印当前值 */
        if (ln[1] != 0)
        {
            Chassis_SetCrabTrim((float)atof(ln + 1));
        }
        tune_printf("OK,c,%.4f\r\n", (double)Chassis_GetCrabTrim());
        break;
    case 'b':                                   /* b<度>：走直微调角（正=朝左走，负=朝右走） */
        {
            float d = (float)atof(ln + 1);

            Chassis_SetStraightTrimDeg(d);
            tune_printf("OK,b,%.2f\r\n", (double)Chassis_GetStraightTrimDeg());
        }
        break;

    case 'v':                                   /* v：视觉状态（B3 帧 / 偏差 / 任务码） */
        {
            int16_t dx = 0, dy = 0;
            char    code[20];
            uint8_t has = Vision_GetOffset(&dx, &dy);

            code[0] = 0;
            (void)Vision_TakeTask(code, (uint8_t)sizeof(code));
            tune_printf("VIS,on=%u,bytes=%lu,frames=%lu,arr=%u,kind=%u,dx=%d,dy=%d\r\n",
                        (unsigned)Vision_IsOnline(),
                        (unsigned long)Vision_RxByteCount(),
                        (unsigned long)Vision_FrameCount(),
                        (unsigned)Vision_IsArrived(),
                        (unsigned)Vision_GetKind(),
                        (int)dx, (int)dy);
            if (has == 0u) tune_send("VIS,NOFRAME\r\n");
            if (code[0] != 0) tune_printf("CODE,%s\r\n", code);
        }
        break;

    default:
        tune_send("ERR,?\r\n");
        break;
    }
}

/* ★字节入口（不碰串口，只解析）
   ① 调参通道自己当串口主人时：TUNE_RxIsr 拿中断字节喂进来；
   ② 串口被升降臂滑块解析器占着时（ARM_ENABLE=1 且同用串口2）：
      由 arm.c 的 ARM_VofaRxIsr 把每个字节转发进来 → 一条线同时支持滑块和调参命令 */
void TUNE_FeedByte(char b)
{
    if ((b == '\n') || (b == '\r'))
    {
        if (s_line_n > 0u)
        {
            s_line[s_line_n] = '\0';
            tune_handle_line(s_line);
            s_line_n = 0u;
        }
    }
    else if (s_line_n < (uint8_t)(TUNE_LINE_MAX - 1u))
    {
        s_line[s_line_n++] = b;
    }
    else
    {
        s_line_n = 0u;                          /* 太长：丢掉，防止解析一坨垃圾 */
    }
}

/* 收到一字节：攒到 \n 或 \r 就处理（自己当串口主人时用） */
void TUNE_RxIsr(UART_HandleTypeDef *huart)
{
    TUNE_FeedByte((char)s_rx_byte);
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}

/* ====================================================================
  *  6. 周期任务
  * ==================================================================== */
static void tune_telem_send(void)
{
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;

    if ((uint32_t)(HAL_GetTick() - s_telem_tick) < (uint32_t)(1000u / ((s_telem_hz != 0u) ? s_telem_hz : 1u))) return;
    s_telem_tick = HAL_GetTick();

    Chassis_GetVelocityEstimate(&vx, &vy, &wz);     /* 里程计估计的整车速度 */

    /* 一行一份 CSV：PC 侧 split(',') 就能用，肉眼也能读
       ★蓝牙 9600 下字符预算很紧（最多约 96 字符/行 @10Hz）：
         只保留调底盘最必需的字段，末位是 4 个轮子的【实测转速】——
         谁在转、是不是驱动自己在转，一眼就看出来。
       格式： A,ms,state,yaw,err,out,wz,fwd,side,turn_err,side_err,rpm1,rpm2,rpm3,rpm4,EXT,TEC
       末尾两个 CAN 字段用于判断"臂的 CAN 驱动器到底回不回"：
         EXT = 收到的扩展帧计数（>0 = 驱动器回了！）
         TEC = CAN 发送错误计数（一直涨 = 没人 ACK） */
    tune_printf("A,%lu,%u,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f,%.1f,%.1f,"
                "%.1f,%.1f,%.1f,%.1f,%u,%u,%u\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned)((s_act == TUNE_IDLE) ? 0u : 1u),   /* state：1 = 正在动作 */
                (double)Chassis_GetYawDeg(),                  /* yaw（IMU 真值，已扣漂移） */
                (double)Chassis_GetHeadingErrDeg(),           /* 航向误差 */
                (double)Chassis_PidGetOut(0u),                /* 航向环输出 */
                (double)wz,                                   /* 实测角速度 */
                (double)Chassis_GetForwardMm(),               /* 前向位移 */
                (double)Chassis_GetSideMm(),                  /* 横向位移 */
                (double)Chassis_GetTurnErrDeg(),              /* 旋转误差 */
                (double)Chassis_GetSideErrMm(),               /* 平移误差 */
                (double)Chassis_GetWheelRpm(0u),              /* 轮1 实测转速 */
                (double)Chassis_GetWheelRpm(1u),              /* 轮2 */
                (double)Chassis_GetWheelRpm(2u),              /* 轮3 */
                (double)Chassis_GetWheelRpm(3u),              /* 轮4 */
                (unsigned)tune_can_ext_cnt(),                 /* CAN 扩展帧计数 */
                (unsigned)tune_can_tec(),                     /* CAN 发送错误计数 */
                (unsigned)tune_can_tx_ok());                  /* CAN 成功入邮箱的帧数 */
}

void TUNE_Task(void)
{
#if REMOTE_ENABLE
    /* ★遥控超时保护：超过 REMOTE_TIMEOUT_MS 没按键 → 自动停车 */
    if ((s_remote_active != 0u) &&
        ((uint32_t)(HAL_GetTick() - s_remote_tick) >= (uint32_t)REMOTE_TIMEOUT_MS))
    {
        s_remote_active = 0u;
        Chassis_Stop();
        tune_send("RM,T\r\n");
    }
#endif
    uint32_t now = HAL_GetTick();

    /* --- 1. 动作推进 --- */
    if (s_act != TUNE_IDLE)
    {
        if (tune_act_done() != 0u)
        {
            uint8_t act = s_act;                /* ★先存下来：tune_stop_all() 会把它改成 IDLE */

            tune_stop_all();
            s_act_cnt++;
            s_last_act_ms = (uint32_t)(now - s_act_tick);
            tune_printf("OK,DONE,%lu\r\n", (unsigned long)s_last_act_ms);
            tune_report_result(act, 'D');

            /* 安排 1.5 秒后的"停稳复测" */
            if (act == TUNE_TURN)
            {
                s_settle_kind  = 'T';
                s_settle_start = s_turn_start_deg;
                s_settle_target= s_turn_target_deg;
                s_settle_tick  = now;
                s_settle_wait  = 1u;
            }
        }
        else if ((uint32_t)(now - s_act_tick) >= (uint32_t)TUNE_ACT_TIMEOUT_MS)
        {
            uint8_t act = s_act;

            tune_stop_all();                    /* 超时兜底：绝不一直开下去 */
            s_act_cnt++;
            s_last_act_ms = (uint32_t)(now - s_act_tick);
            tune_printf("OK,TIMEOUT,%lu\r\n", (unsigned long)s_last_act_ms);
            tune_report_result(act, 'T');

            if (act == TUNE_TURN)
            {
                s_settle_kind  = 'T';
                s_settle_start = s_turn_start_deg;
                s_settle_target= s_turn_target_deg;
                s_settle_tick  = now;
                s_settle_wait  = 1u;
            }
        }
    }

    /* --- 1.5 动作结束后：等车"真正停稳"（yaw 不再变化）再测一次 ---
       为什么不能固定延时：每次转向耗时 1.5~5 秒不等，固定 1.5 秒常常车还在转，
       测出来的误差就是假的（实测出现 -9° 和 +11° 这种乱值）。
       这里改成：连续 400ms 内 yaw 变化 < 0.3°/s 才算停稳 → 再量。最多等 6 秒。 */
    if (s_settle_wait != 0u)
    {
        static float    s_prev_yaw;
        static uint32_t s_prev_tick;
        static uint32_t s_stable_tick;
        uint32_t dt_ms = (uint32_t)(now - s_prev_tick);

        if (dt_ms >= 100u)
        {
            float yaw = tune_wrap180(Chassis_GetYawDeg());   /* ★和起点同一坐标系（底盘 yaw），不能混用 IMU 原始值 */
            float rate = fabsf(yaw - s_prev_yaw) / ((float)dt_ms * 0.001f);

            s_prev_yaw  = yaw;
            s_prev_tick = now;

            if (rate < 0.3f)
            {
                if (s_stable_tick == 0u) s_stable_tick = now;
            }
            else
            {
                s_stable_tick = 0u;
            }

            if (((s_stable_tick != 0u) && ((uint32_t)(now - s_stable_tick) >= 400u)) ||
                ((uint32_t)(now - s_settle_tick) >= 6000u))
            {
                float fin  = yaw;
                float want = tune_wrap180(s_settle_start + s_settle_target);
                float err  = tune_wrap180(fin - want);
                float real = tune_wrap180(fin - s_settle_start);

                s_settle_wait = 0u;
                s_stable_tick = 0u;
                tune_printf("R2,T,%.1f,%.1f,%.1f,%.1f\r\n",
                            (double)s_settle_start, (double)s_settle_target,
                            (double)real, (double)err);
            }
        }
    }

    /* --- 2. 遥测 --- */
    if (s_telem != 0u) tune_telem_send();
}

void TUNE_Init(void)
{
    s_uart    = TUNE_UART;
    s_line_n  = 0u;
    s_act     = TUNE_IDLE;
    s_telem   = 0u;
    s_cmd_cnt = 0u;
    s_act_cnt = 0u;
    s_last_act_ms = 0u;
    s_vofa_was_on = 1u;

    if (s_uart == 0) return;

#if TUNE_TELEM_ON_BOOT
    /* ★开机就默认开文本遥测（并关掉 JustFloat 二进制）
       为什么：板子一旦重启（电机大电流导致欠压复位），遥测会回到"关闭"状态，
       上位机就一片空白、看起来像"没数据"。开机默认开着就永远不会再懵。
       调试期用它；比赛跑流程时把本宏改 0（免得占串口带宽）。 */
    s_telem = 1u;
    s_telem_tick = HAL_GetTick();
    VOFA_SetEnable(0u);
#endif

#if ARM_ENABLE && ARM_IFACE_UART == 0 && (TUNE_ON_BT == 0)
    /* 串口2 的接收归升降臂滑块解析器（ARM_VofaRxIsr），它会转发字节过来 → 这里不要再挂一次，
       否则两边抢同一个接收缓冲（现象：滑块和命令都时好时坏）*/
#else
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
#endif
}

uint8_t  TUNE_IsBusy(void)      { return (s_act == TUNE_IDLE) ? 0u : 1u; }
float    TUNE_GetYawDeg(void)   { return Chassis_GetYawDeg(); }
uint32_t TUNE_GetCmdCount(void) { return s_cmd_cnt; }

#else   /* ---------------- 没开调参通道：空实现（main.c 不用改） ---------------- */

void     TUNE_Init(void)                    {}
void     TUNE_RxIsr(UART_HandleTypeDef *h)  { (void)h; }
void     TUNE_Task(void)                    {}
uint8_t  TUNE_IsBusy(void)                  { return 0u; }
float    TUNE_GetYawDeg(void)               { return 0.0f; }
uint32_t TUNE_GetCmdCount(void)             { return 0u; }

#endif /* TUNE_ENABLE */

