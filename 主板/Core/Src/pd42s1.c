/**
  ******************************************************************************
  * @file    pd42s1.c
  * @brief   正点原子 PD42S1 闭环步进驱动器（串口 TTL）驱动封装
  *
  *  帧格式（上行 / 下行一样）：
  *    [0] 0xC5  [1] 从机地址  [2] 功能码  [3..] 指令数据  [len-2] 校验和  [len-1] 0x5C
  *    校验和 = (0xC5 + 地址 + 功能码 + 数据区全部字节) & 0xFF
  *
  *  下行应答：[3] 是错误码，0x01 = 操作成功，其它见手册（0xE1 帧太短 / 0xE2 帧头错 /
  *            0xE3 帧尾错 / 0xE4 校验错 / 0xE5 不支持的功能码 / 0xE6 数据不合法）
  *
  *  应答只有 5~40 字节，而且是“一帧一口气发完”，所以这里用「串口空闲」来切帧
  *  （不能用 0x5C 当结束符：位置数据里就可能出现 0x5C）。
  ******************************************************************************
  */

#include "pd42s1.h"
#include "vision.h"    /* MaixCam 视觉帧（UART7）分派 */
#include "scan.h"      /* 扫码相机（UART8）分派 */
#include "hmi.h"
#include "imu.h"        /* IMU（YbImu）走 UART4 时，接收回调转发给 IMU_RxIsr() */
#include "arm.h"        /* VOFA 拖条：串口2 收到的字节转发给 ARM_VofaRxIsr() */
#include "bt.h"         /* 蓝牙（UART4 = PA0/PA1）：收到的字节转发给 BT_RxIsr() */
#include "tune.h"       /* 调参命令通道（有线串口2 / 蓝牙串口4）：转发给 TUNE_RxIsr() */
#include <string.h>

/* 未使能步进、或工程里还没加串口（CubeMX 没勾 USART）时，全部退化成空函数：
   main.c 不用改，也不会编译报错 */
#if STEPPER_ENABLE && defined(HAL_UART_MODULE_ENABLED)

#include "usart.h"      /* CubeMX 生成：里面有 huart1 / huart2 / huart3 … */

/* ==================== 内部状态 ==================== */

static UART_HandleTypeDef *s_uart = STEPPER_UART;   /* 默认 config.h 里的 STEPPER_UART */

/* 多电机配置表（来自 config.h 的 STEPPER_MOTOR_1 .. 7），表长 = 板上步进口数量 */
static const stepper_cfg_t s_motors[] = {
    STEPPER_MOTOR_1, STEPPER_MOTOR_2, STEPPER_MOTOR_3, STEPPER_MOTOR_4,
    STEPPER_MOTOR_5, STEPPER_MOTOR_6, STEPPER_MOTOR_7
};
#define PD42S1_MOTOR_NUM    ((uint8_t)(sizeof(s_motors) / sizeof(s_motors[0])))

/* 每台电机一份反馈（按从机地址区分） */
static pd42s1_t s_dev[PD42S1_MOTOR_NUM];
static uint8_t  s_active;               /* 最近一次通信/操作的电机（表索引） */

static uint8_t  s_rx_byte;              /* 单字节接收中断用 */
static uint8_t  s_rxbuf[64];            /* 一帧应答的暂存 */
static uint16_t s_rx_len;
static uint8_t  s_rx_active;            /* 1 = 正在收一帧 */
static uint32_t s_rx_last_tick;         /* 最后一个字节到达的时刻（用于空闲切帧） */
static uint32_t s_last_tx_tick;         /* 上一次发指令的时刻（2ms 指令间隔保护） */

/* 地址 → 表索引；找不到返回 0xFF */
static uint8_t pd42s1_index(uint8_t addr)
{
    uint8_t i;

    for (i = 0u; i < PD42S1_MOTOR_NUM; i++)
    {
        if (s_motors[i].addr == addr) return i;
    }
    return 0xFFu;
}

/* 取某台电机的反馈结构（idx 越界就返回当前活跃那台） */
static pd42s1_t *pd42s1_dev(uint8_t idx)
{
    if (idx >= PD42S1_MOTOR_NUM) idx = s_active;
    if (idx >= PD42S1_MOTOR_NUM) idx = 0u;
    return &s_dev[idx];
}

/* ==================== 小工具 ==================== */

static int16_t be_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int32_t be_i32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8)  | ((uint32_t)p[3]));
}

/* 4 字节大端 → float（驱动器的 float 是大端字节序） */
static float be_f32(const uint8_t *p)
{
    union { float f; uint8_t b[4]; } u;
    u.b[0] = p[3];
    u.b[1] = p[2];
    u.b[2] = p[1];
    u.b[3] = p[0];
    return u.f;
}

/* ==================== 组帧 / 发送 ==================== */

uint8_t PD42S1_BuildFrame(uint8_t addr, uint8_t fct, const uint8_t *data, uint8_t dlen, uint8_t *out)
{
    uint8_t i;
    uint8_t sum = 0;
    uint8_t n   = (uint8_t)(3u + dlen);   /* 帧头 + 地址 + 功能码 + 数据 */

    out[0] = PD42S1_FRAME_HEAD;
    out[1] = addr;
    out[2] = fct;

    for (i = 0u; i < dlen; i++)
    {
        out[3u + i] = data[i];
    }

    for (i = 0u; i < n; i++)
    {
        sum = (uint8_t)(sum + out[i]);
    }

    out[n]        = sum;
    out[n + 1u]   = PD42S1_FRAME_TAIL;

    return (uint8_t)(n + 2u);
}

uint8_t PD42S1_SendFrame(const uint8_t *frame, uint8_t len)
{
    if (s_uart == 0 || frame == 0 || len < 5u) return 0u;

    /* ⚠️ 手册要求：两条完整指令之间至少间隔 2ms（给从机时间解析上一条）。
       这里选择"等一会儿再发"，**不丢指令** —— 之前是直接丢，结果上层连发时
       第二条被静默丢掉（电机收不到"停/反转"，排查了很久）。
       最多只等 ~2ms，而且只在连续调用时才等；只在主循环里调用，别放中断！ */
    while ((uint32_t)(HAL_GetTick() - s_last_tx_tick) < (uint32_t)PD42S1_CMD_GAP_MS)
    {
        /* 空转等待，最多 2ms */
    }

    if (HAL_UART_Transmit(s_uart, (uint8_t *)frame, len, 20u) != HAL_OK)
    {
        pd42s1_dev(s_active)->tx_fail++;        /* UART 忙/出错（一直涨就是硬件问题） */
        return 0u;
    }

    s_last_tx_tick = HAL_GetTick();
    pd42s1_dev(s_active)->tx_cnt++;

    return 1u;
}

/* 组帧 + 发送一条指令（data 可为 0） */
static uint8_t pd42s1_cmd(uint8_t fct, uint8_t addr, const uint8_t *data, uint8_t dlen)
{
    uint8_t frame[16];      /* 最长帧：3 + 9 数据 + 校验 + 帧尾 = 14 */
    uint8_t len = PD42S1_BuildFrame(addr, fct, data, dlen, frame);

    return PD42S1_SendFrame(frame, len);
}

/* ==================== 接收：字节 → 空闲切帧 → 解析 ==================== */

void PD42S1_RxByte(uint8_t b)
{
    if (s_rx_len < (uint16_t)sizeof(s_rxbuf))
    {
        s_rxbuf[s_rx_len++] = b;
    }
    else
    {
        /* 溢出：丢掉这一帧，重新开始（下一帧的帧头会重新对齐） */
        s_rx_len = 0u;
        pd42s1_dev(s_active)->rx_err++;
    }

    s_rx_last_tick = HAL_GetTick();
    s_rx_active    = 1u;
}

static void pd42s1_parse(const uint8_t *f, uint16_t len)
{
    uint8_t  i;
    uint8_t  sum = 0;
    uint8_t  dlen;
    uint8_t  idx;
    pd42s1_t *d_dev;
    const uint8_t *d;

    /* 最小帧：头 + 地址 + 功能码 + 错误码 + 校验 + 尾 = 6 字节 */
    if (len < 6u || f[0] != PD42S1_FRAME_HEAD || f[len - 1u] != PD42S1_FRAME_TAIL)
    {
        d_dev = pd42s1_dev(s_active);
        d_dev->rx_err++;
        return;
    }

    /* 按应答里的从机地址找到是哪台电机 */
    idx = pd42s1_index(f[1]);
    if (idx != 0xFFu) s_active = idx;
    d_dev = pd42s1_dev(idx);

    /* 校验：帧头 + 地址 + 功能码 + 数据区（不含校验和与帧尾） */
    for (i = 0u; i + 2u < len; i++)
    {
        sum = (uint8_t)(sum + f[i]);
    }
    if (sum != f[len - 2u])
    {
        d_dev->rx_err++;
        return;
    }

    d_dev->rx_ok++;
    d_dev->online     = 1u;
    d_dev->last_rx_tick = HAL_GetTick();
    d_dev->last_fct   = f[2];
    d_dev->last_ack   = f[3];

    if (f[3] != PD42S1_ACK_OK) return;      /* 指令没执行成功，后面数据无意义 */

    dlen = (uint8_t)(len - 6u);
    d    = &f[4];

    switch (f[2])
    {
        case PD42S1_FCT_READ_SPEED:                     /* 0x29 转速 */
            if (dlen >= 2u) { d_dev->rpm = be_i16(d); d_dev->rpm_valid = 1u; }
            break;

        case PD42S1_FCT_READ_POS:                       /* 0x2A 位置 */
            if (dlen >= 4u) { d_dev->pos = be_i32(d); d_dev->pos_valid = 1u; }
            break;

        case PD42S1_FCT_READ_POS_ERR:                   /* 0x2B 位置误差 */
            if (dlen >= 4u) { d_dev->pos_err = be_i32(d); }
            break;

        case PD42S1_FCT_READ_STATE:                     /* 0x2C 运行状态 */
            if (dlen >= 1u) { d_dev->state = d[0]; d_dev->state_valid = 1u; }
            break;

        case PD42S1_FCT_READ_ENABLE:                    /* 0x2F 使能状态 */
            if (dlen >= 1u) { d_dev->enable = d[0]; }
            break;

        case PD42S1_FCT_READ_ARRIVED:                   /* 0x30 到位状态 */
            if (dlen >= 1u) { d_dev->arrived = d[0]; }
            break;

        case PD42S1_FCT_READ_VOLT:                      /* 0x24 总线电压 */
            if (dlen >= 4u) { d_dev->volt = be_f32(d); }
            break;

        case PD42S1_FCT_SET_MODE:                       /* 0x62 设置工作模式的应答：回读新模式 */
            if (dlen >= 2u) { d_dev->mode = d[1]; d_dev->mode_valid = 1u; }
            break;

        case PD42S1_FCT_READ_DRIVE:                     /* 0x32 驱动参数：模式/回响/波特率/细分… */
            if (dlen >= 12u)
            {
                d_dev->mode      = d[0];
                d_dev->mode_valid = 1u;
                d_dev->echo      = d[1];                 /* 1 = 不回响 */
                d_dev->uart_baud = ((uint32_t)d[2] << 24) | ((uint32_t)d[3] << 16) |
                                  ((uint32_t)d[4] << 8)  |  (uint32_t)d[5];
                d_dev->dir_level = d[8];
                d_dev->en_level  = d[9];
                d_dev->step_div  = (uint16_t)(((uint16_t)d[10] << 8) | (uint16_t)d[11]);
            }
            break;

        case PD42S1_FCT_READ_SYS:                       /* 0x31 系统参数（一次读回一大堆） */
            if (dlen >= 40u)
            {
                d_dev->volt      = be_f32(&d[0]);
                d_dev->rpm       = be_i16(&d[18]);
                d_dev->pos       = be_i32(&d[24]);
                d_dev->pos_err   = be_i32(&d[28]);
                d_dev->enable    = d[36];    /* 0 使能，1 失能 */
                d_dev->arrived   = d[37];    /* 1 到位 */
                d_dev->state     = (uint8_t)(d[38] ? PD42S1_ST_STALL : PD42S1_ST_IDLE);
                d_dev->rpm_valid = 1u;
                d_dev->pos_valid = 1u;
                d_dev->state_valid = 1u;
            }
            break;

        default:                                        /* 其它指令的应答只更新错误码 */
            break;
    }
}

void PD42S1_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  i;

    /* 总线空闲 PD42S1_RX_IDLE_MS 毫秒 → 认为这一帧收完了 */
    if (s_rx_active != 0u &&
        (uint32_t)(now - s_rx_last_tick) >= (uint32_t)PD42S1_RX_IDLE_MS)
    {
        s_rx_active = 0u;
        pd42s1_parse(s_rxbuf, s_rx_len);
        s_rx_len = 0u;
    }

    /* 掉线判断（每台电机各判各的） */
    for (i = 0u; i < PD42S1_MOTOR_NUM; i++)
    {
        if (s_dev[i].online != 0u &&
            (uint32_t)(now - s_dev[i].last_rx_tick) > (uint32_t)PD42S1_OFFLINE_MS)
        {
            s_dev[i].online = 0u;
        }
    }
}

/* ==================== 初始化 ==================== */

void PD42S1_InitUart(UART_HandleTypeDef *huart)
{
    if (huart != 0)
    {
        s_uart = huart;
    }

    memset(s_dev, 0, sizeof(s_dev));
    s_active       = 0u;
    s_rx_len       = 0u;
    s_rx_active    = 0u;
    s_rx_last_tick = HAL_GetTick();
    s_last_tx_tick = HAL_GetTick() - (uint32_t)PD42S1_CMD_GAP_MS;   /* 允许马上发第一条 */

    /* 开单字节接收中断（CubeMX 里记得勾上 USART3 全局中断） */
    if (s_uart != 0)
    {
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
    }
}

/* ★重新挂上串口接收（串口出错后 HAL 会中止接收，必须重新挂，否则该串口永久收不到数据）
   现象：一开始能读转速/位置，跑一会儿全部变 0、驱动器"实为在线却 N0/4" */
void PD42S1_RearmRx(void)
{
    if (s_uart == 0) return;
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

void PD42S1_Init(void)
{
    PD42S1_InitUart(STEPPER_UART);
}

/* ---------------- 多电机访问 ---------------- */

uint8_t PD42S1_GetMotorNum(void)
{
    return PD42S1_MOTOR_NUM;
}

const stepper_cfg_t *PD42S1_GetMotorCfg(uint8_t idx)
{
    if (idx >= PD42S1_MOTOR_NUM) return 0;
    return &s_motors[idx];
}

const pd42s1_t *PD42S1_GetMotorState(uint8_t idx)
{
    if (idx >= PD42S1_MOTOR_NUM) return 0;
    return &s_dev[idx];
}

uint8_t PD42S1_GetActiveMotor(void)
{
    return s_active;
}

const pd42s1_t *PD42S1_Get(void)
{
    return &s_dev[s_active];
}

uint8_t PD42S1_IsOnline(void)
{
    return s_dev[s_active].online;
}

/* 接收中断回调：只负责把字节喂给驱动，不在这里做解析（解析放主循环，安全） */
#if PD42S1_PROVIDE_UART_CALLBACK
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == STEPPER_UART_INSTANCE)
    {
        PD42S1_RxByte(s_rx_byte);
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
    }
#if HMI_ENABLE
    /* 触摸屏（UART5）：交给菜单解析，并重新挂上接收 */
    else if (huart->Instance == HMI_UART->Instance)
    {
        HMI_RxIsr(huart);
    }
#endif
#if IMU_ENABLE && IMU_IF_UART
    /* IMU（UART4 = PC10/PC11）：交给 YbImu 串口解析，并在里面重新挂上接收 */
    else if (huart->Instance == IMU_UART->Instance)
    {
        IMU_RxIsr(huart);
    }
#endif
#if ARM_ENABLE
    /* VOFA 串口（默认串口2）：收上位机控件（拖条/按钮）发下来的文本 */
    else if (huart->Instance == VOFA_UART->Instance)
    {
        ARM_VofaRxIsr(huart);
    }
#endif
#if BT_ENABLE && !(TUNE_ENABLE && TUNE_ON_BT)
    /* 蓝牙（UART4 = PA0/PA1）：收手机/上位机的文本命令（'0'~'4' 直接当模式命令）
       ⚠️ 把调参通道放到蓝牙上时（TUNE_ON_BT=1），这一路字节归调参通道 → 这里让位 */
    else if (huart->Instance == BT_UART->Instance)
    {
        BT_RxIsr(huart);
    }
#endif
#if TUNE_ENABLE
    /* 调参命令通道：有线走串口2、蓝牙走串口4（由 TUNE_ON_BT 决定） */
    else if (huart->Instance == TUNE_UART->Instance)
    {
        TUNE_RxIsr(huart);
    }
#endif
#if VISION_ENABLE
    /* MaixCam（UART7 = PE8/PE7）：B3 视觉帧 */
    else if (huart->Instance == VISION_UART_INSTANCE)
    {
        Vision_RxIsr(huart);
    }
#endif
#if SCAN_ENABLE
    /* 扫码相机（UART8 = PE1/PE0）：$任务码# */
    else if (huart->Instance == SCAN_UART_INSTANCE)
    {
        Scan_RxIsr(huart);
    }
#endif
}
#endif

/* ==================== 运动指令：开环 ==================== */

void PD42S1_OpenLoopSpeed(uint8_t addr, uint8_t dir, uint8_t acc, float rpm)
{
    uint8_t d[6];
    union { float f; uint8_t b[4]; } u;

    u.f = rpm;
    d[0] = dir;
    d[1] = acc;
    d[2] = u.b[3];      /* 速度是大端 float */
    d[3] = u.b[2];
    d[4] = u.b[1];
    d[5] = u.b[0];

    (void)pd42s1_cmd(PD42S1_FCT_OL_SPEED, addr, d, 6u);
}

/* 位置类指令共用：dir + acc + 速度(2) + 脉冲(4) */
static void pd42s1_pos_cmd(uint8_t fct, uint8_t addr, uint8_t dir, uint8_t acc,
                           uint16_t rpm, uint32_t pulses)
{
    uint8_t d[9];
    uint8_t dlen = 8u;

    d[0] = dir;
    d[1] = acc;
    d[2] = (uint8_t)((rpm >> 8) & 0xFFu);
    d[3] = (uint8_t)((rpm >> 0) & 0xFFu);
    d[4] = (uint8_t)((pulses >> 24) & 0xFFu);
    d[5] = (uint8_t)((pulses >> 16) & 0xFFu);
    d[6] = (uint8_t)((pulses >> 8)  & 0xFFu);
    d[7] = (uint8_t)((pulses >> 0)  & 0xFFu);

#if PD42S1_POS_EXTRA_ZERO
    /* 正点原子例程在位置模式这里多发了一个 0x00 再跟校验和
       （手册示例是 13 字节，例程是 14 字节）。校验和不含这一字节也成立，
       默认跟手册（13 字节）；万一你的驱动器不响应，把 config.h 里这个开关改成 1 试 */
    d[8] = 0x00u;
    dlen = 9u;
#endif

    (void)pd42s1_cmd(fct, addr, d, dlen);
}

void PD42S1_OpenLoopPosAbs(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses)
{
    pd42s1_pos_cmd(PD42S1_FCT_OL_POS_ABS, addr, dir, acc, rpm, pulses);
}

void PD42S1_OpenLoopPosRel(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses)
{
    pd42s1_pos_cmd(PD42S1_FCT_OL_POS_REL, addr, dir, acc, rpm, pulses);
}

/* ==================== 运动指令：闭环 ==================== */

void PD42S1_TorqueMode(uint8_t addr, uint8_t dir, uint16_t current_ma)
{
    uint8_t d[3];

    d[0] = dir;
    d[1] = (uint8_t)((current_ma >> 8) & 0xFFu);
    d[2] = (uint8_t)((current_ma >> 0) & 0xFFu);

    (void)pd42s1_cmd(PD42S1_FCT_TORQUE, addr, d, 3u);
}

void PD42S1_SpeedMode(uint8_t addr, uint8_t dir, uint8_t acc, float rpm)
{
    uint8_t d[6];
    union { float f; uint8_t b[4]; } u;

    u.f = rpm;
    d[0] = dir;
    d[1] = acc;
    d[2] = u.b[3];
    d[3] = u.b[2];
    d[4] = u.b[1];
    d[5] = u.b[0];

    (void)pd42s1_cmd(PD42S1_FCT_SPEED, addr, d, 6u);
}

void PD42S1_PosModeAbs(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses)
{
    pd42s1_pos_cmd(PD42S1_FCT_POS_ABS, addr, dir, acc, rpm, pulses);
}

void PD42S1_PosModeRel(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses)
{
    pd42s1_pos_cmd(PD42S1_FCT_POS_REL, addr, dir, acc, rpm, pulses);
}

/* ==================== 控制指令 ==================== */

void PD42S1_SetAddress(uint8_t addr, uint8_t new_addr)
{
    uint8_t d[1];

    d[0] = new_addr;                    /* 1~255（0 是广播地址，不能当从机地址） */
    (void)pd42s1_cmd(PD42S1_FCT_SET_ADDR, addr, d, 1u);
}

void PD42S1_SetMode(uint8_t addr, uint8_t mode)
{
    uint8_t d[1];

    d[0] = mode;                        /* 0x00~0x07，开环模式不能用这条设 */
    (void)pd42s1_cmd(PD42S1_FCT_SET_MODE, addr, d, 1u);
}

void PD42S1_ReadDriveParams(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_DRIVE, addr, 0, 0u);
}

void PD42S1_Enable(uint8_t addr, uint8_t en)
{
    uint8_t d[1];

    d[0] = (uint8_t)(en ? 1u : 0u);     /* 0 使能，1 失能 */
    (void)pd42s1_cmd(PD42S1_FCT_ENABLE, addr, d, 1u);
}

void PD42S1_StopNow(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_STOP_NOW, addr, 0, 0u);
}

void PD42S1_ClearState(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_CLEAR_STATE, addr, 0, 0u);
}

void PD42S1_UnlockStall(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_UNLOCK_STALL, addr, 0, 0u);
}

void PD42S1_AngleZero(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_ANGLE_ZERO, addr, 0, 0u);
}

/* ==================== 多电机封装（自动套用表里的地址/方向/加速度） ==================== */

void PD42S1_MotorSpeed(uint8_t idx, uint8_t dir, float rpm)
{
    const stepper_cfg_t *m = PD42S1_GetMotorCfg(idx);

    if (m == 0) return;
    if (m->dir_inv != 0u) dir = (uint8_t)(dir ? 0u : 1u);   /* 机械装反 → 软件反过来 */

    /* 闭环速度 0xF1（官方例程验证过的）；想用开环把下面换成 PD42S1_OpenLoopSpeed */
    PD42S1_SpeedMode(m->addr, dir, m->acc, rpm);
}

void PD42S1_MotorPosAbs(uint8_t idx, uint32_t pulses)
{
    const stepper_cfg_t *m = PD42S1_GetMotorCfg(idx);

    if (m == 0) return;
    PD42S1_PosModeAbs(m->addr, 0u, m->acc, (uint16_t)m->speed_rpm, pulses);
}

void PD42S1_MotorPosRel(uint8_t idx, uint32_t pulses)
{
    const stepper_cfg_t *m = PD42S1_GetMotorCfg(idx);

    if (m == 0) return;
    PD42S1_PosModeRel(m->addr, 0u, m->acc, (uint16_t)m->speed_rpm, pulses);
}

void PD42S1_MotorStop(uint8_t idx)
{
    const stepper_cfg_t *m = PD42S1_GetMotorCfg(idx);

    if (m == 0) return;
    PD42S1_StopNow(m->addr);
}

void PD42S1_MotorEnable(uint8_t idx, uint8_t en)
{
    const stepper_cfg_t *m = PD42S1_GetMotorCfg(idx);

    if (m == 0) return;
    PD42S1_Enable(m->addr, en);
}

/* ==================== 读取指令 ==================== */

/* ★设置指令回响（0x6F）：enable=1 → 正常应答；enable=0 → 不回答
   驱动器若被设成"不回响"，它照样执行命令但一个字节都不回，
   结果就是底盘转速/位置反馈永远读不到（N0/4、rpm 恒 0 的隐形原因之一）。 */
void PD42S1_SetEcho(uint8_t addr, uint8_t enable)
{
    uint8_t d = (enable != 0u) ? 0u : 1u;      /* 0 = 正常回响，1 = 不回响 */

    (void)pd42s1_cmd(PD42S1_FCT_SET_ECHO, addr, &d, 1u);
}

void PD42S1_ReadSpeed(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_SPEED, addr, 0, 0u);
}

void PD42S1_ReadPos(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_POS, addr, 0, 0u);
}

void PD42S1_ReadPosErr(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_POS_ERR, addr, 0, 0u);
}

void PD42S1_ReadState(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_STATE, addr, 0, 0u);
}

void PD42S1_ReadEnable(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_ENABLE, addr, 0, 0u);
}

void PD42S1_ReadArrived(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_ARRIVED, addr, 0, 0u);
}

void PD42S1_ReadVolt(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_VOLT, addr, 0, 0u);
}

void PD42S1_ReadSysParam(uint8_t addr)
{
    (void)pd42s1_cmd(PD42S1_FCT_READ_SYS, addr, 0, 0u);
}

#else   /* ---------------- 未使能 / 没串口：空实现 ---------------- */

static pd42s1_t s_dev_dummy;

void PD42S1_Init(void)                     {}
void PD42S1_Update(void)                   {}
void PD42S1_RxByte(uint8_t b)              { (void)b; }
const pd42s1_t *PD42S1_Get(void)           { return &s_dev_dummy; }
uint8_t PD42S1_IsOnline(void)              { return 0u; }

/* 多电机接口（未使能时也留着，main.c 不用改） */
static const stepper_cfg_t s_cfg_dummy[] = {
    STEPPER_MOTOR_1, STEPPER_MOTOR_2, STEPPER_MOTOR_3, STEPPER_MOTOR_4,
    STEPPER_MOTOR_5, STEPPER_MOTOR_6, STEPPER_MOTOR_7
};
uint8_t PD42S1_GetMotorNum(void)                       { return 1u; }
const stepper_cfg_t *PD42S1_GetMotorCfg(uint8_t idx)   { (void)idx; return &s_cfg_dummy[0]; }
const pd42s1_t *PD42S1_GetMotorState(uint8_t idx)      { (void)idx; return &s_dev_dummy; }
uint8_t PD42S1_GetActiveMotor(void)                    { return 0u; }
void PD42S1_MotorSpeed(uint8_t i, uint8_t d, float r)  { (void)i; (void)d; (void)r; }
void PD42S1_MotorPosAbs(uint8_t i, uint32_t p)         { (void)i; (void)p; }
void PD42S1_MotorPosRel(uint8_t i, uint32_t p)         { (void)i; (void)p; }
void PD42S1_MotorStop(uint8_t i)                       { (void)i; }
void PD42S1_MotorEnable(uint8_t i, uint8_t e)          { (void)i; (void)e; }

void PD42S1_SetAddress(uint8_t a, uint8_t n) { (void)a; (void)n; }
void PD42S1_OpenLoopSpeed(uint8_t a, uint8_t d, uint8_t c, float r) { (void)a; (void)d; (void)c; (void)r; }
void PD42S1_OpenLoopPosAbs(uint8_t a, uint8_t d, uint8_t c, uint16_t r, uint32_t p) { (void)a; (void)d; (void)c; (void)r; (void)p; }
void PD42S1_OpenLoopPosRel(uint8_t a, uint8_t d, uint8_t c, uint16_t r, uint32_t p) { (void)a; (void)d; (void)c; (void)r; (void)p; }
void PD42S1_TorqueMode(uint8_t a, uint8_t d, uint16_t cu) { (void)a; (void)d; (void)cu; }
void PD42S1_SpeedMode(uint8_t a, uint8_t d, uint8_t c, float r) { (void)a; (void)d; (void)c; (void)r; }
void PD42S1_PosModeAbs(uint8_t a, uint8_t d, uint8_t c, uint16_t r, uint32_t p) { (void)a; (void)d; (void)c; (void)r; (void)p; }
void PD42S1_PosModeRel(uint8_t a, uint8_t d, uint8_t c, uint16_t r, uint32_t p) { (void)a; (void)d; (void)c; (void)r; (void)p; }
void PD42S1_Enable(uint8_t a, uint8_t e)   { (void)a; (void)e; }
void PD42S1_SetMode(uint8_t a, uint8_t m)  { (void)a; (void)m; }
void PD42S1_ReadDriveParams(uint8_t a)     { (void)a; }
void PD42S1_StopNow(uint8_t a)             { (void)a; }
void PD42S1_ClearState(uint8_t a)          { (void)a; }
void PD42S1_UnlockStall(uint8_t a)         { (void)a; }
void PD42S1_AngleZero(uint8_t a)           { (void)a; }
void PD42S1_ReadSpeed(uint8_t a)           { (void)a; }
void PD42S1_ReadPos(uint8_t a)             { (void)a; }
void PD42S1_ReadPosErr(uint8_t a)          { (void)a; }
void PD42S1_ReadState(uint8_t a)           { (void)a; }
void PD42S1_ReadEnable(uint8_t a)          { (void)a; }
void PD42S1_ReadArrived(uint8_t a)         { (void)a; }
void PD42S1_ReadVolt(uint8_t a)            { (void)a; }
void PD42S1_ReadSysParam(uint8_t a)        { (void)a; }

uint8_t PD42S1_BuildFrame(uint8_t addr, uint8_t fct, const uint8_t *data, uint8_t dlen, uint8_t *out)
{
    (void)addr; (void)fct; (void)data; (void)dlen; (void)out;
    return 0u;
}

uint8_t PD42S1_SendFrame(const uint8_t *frame, uint8_t len)
{
    (void)frame; (void)len;
    return 0u;
}

#if defined(HAL_UART_MODULE_ENABLED)
void PD42S1_InitUart(UART_HandleTypeDef *huart) { (void)huart; }
#endif

#endif
