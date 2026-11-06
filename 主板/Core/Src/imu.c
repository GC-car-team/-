/**
  ******************************************************************************
  * @file    imu.c
  * @brief   YbImu 驱动（I2C，地址 0x23）—— 只读我们需要的：欧拉角 yaw + 陀螺 Z
  *
  *  协议（来自官方 YbImuLib）：往"功能码"寄存器读 N 字节即可
  *      0x01 VERSION  3B   版本号 major/minor/patch      ← 用来判断在线
  *      0x0A RAW_GYRO 6B   3×int16 **小端**，±2000dps 满量程
  *      0x26 EULER    12B  3×float32 **小端**，单位【弧度】(roll, pitch, yaw)
  *      0x70 CALIB_IMU 写 1 → 轮询该寄存器变为非 0 表示校准完成
  *      0x61 ALGO_TYPE 写 6 或 9
  *
  *  ⚠️ 注意：**底盘坐标约定是"逆时针(左转)为正"**，而模块的 yaw 符号取决于模块
  *     在车上的安装方向 → 用 IMU_GYRO_Z_SIGN 统一翻（装反了就填 -1.0f）。
  ******************************************************************************
  */
#include "imu.h"
#include "usart.h"    /* huart4（UART4 = PC10/PC11）—— 串口接法要用 */
#include "arm.h"      /* 串口出错恢复时要用 ARM_RearmRx()（蓝牙/VOFA 主口） */
#include "pd42s1.h"
#include "vision.h"   /* 串口出错恢复时也要通知视觉解析器 */
#include "scan.h"     /* 扫码 UART8 出错重挂 */
#if !IMU_IF_UART
#include "i2c.h"      /* hi2c1 在 CubeMX 生成的 i2c.h 里（I2C1 必须已在 CubeMX 里打开） */
#endif

#if IMU_ENABLE && IMU_IF_UART

/* =============================================================================
 *  一、串口版（UART4 = PC10/PC11，115200 8N1）—— YbImu 官方串口协议
 *
 *  帧格式：  7E 23 LEN FUNC <DATA...> SUM8
 *      · LEN  = 整帧字节数（含最后的 SUM8）
 *      · SUM8 = 前面所有字节累加 & 0xFF
 *
 *  我们下发（每帧 7 字节，LEN=0x07）：
 *      7E 23 07 80 26 00 CS     请求欧拉角（0x80 = 请求数据，后跟功能码）
 *      7E 23 07 80 01 00 CS     请求版本号（上电握手用 = "你在不在"）
 *      7E 23 07 60 <Hz> 5F CS   设置上报频率（10~100Hz，只在主动上报模式用）
 *      7E 23 07 61 <6或9> 5F CS 融合算法 6/9（存模块 flash）
 *      7E 23 07 70 01 5F CS     校准 IMU（模块回 0x81 状态帧，状态非 0 = 校准完成）
 *
 *  模块上报：
 *      7E 23 11 26 <12B> CS     欧拉角 roll,pitch,yaw 各 float32 小端，单位【弧度】
 *      7E 23 17 04 <18B> CS     原始数据 ax,ay,az,gx,gy,gz,mx,my,mz 各 int16 小端
 *                               （★Z 轴角速度 = gz = 第 10~11 字节，不是第 4~5 字节）
 *      7E 23 07 81 <功能码> <状态> CS   命令回执
 *
 *  默认策略：**我们轮询**（IMU_PERIOD_MS 发一次请求）——不写模块 flash、不依赖模块里
 *  存的上报设置，上电就能用；想改成模块主动上报：config.h 的 IMU_USE_ACTIVE_REPORT 改 1。
 * ========================================================================== */

#define YB_H1          0x7Eu
#define YB_H2          0x23u
#define YB_F_VERSION   0x01u
#define YB_F_RAW       0x04u
#define YB_F_EULER     0x26u
#define YB_F_RATE      0x60u
#define YB_F_ALGO      0x61u
#define YB_F_CALIB     0x70u
#define YB_F_REQ       0x80u
#define YB_F_STATE     0x81u

#define YB_RX_MAX      40u          /* 单帧数据区上限（官方库也是 40） */
#define YB_RAD2DEG     57.2957795f
#define YB_GYRO_LSB    2000.0f      /* 满量程 ±2000 dps */

static UART_HandleTypeDef *s_uart;
static uint8_t  s_rx_byte;          /* 接收中断的落脚点（必须常驻内存） */
static uint8_t  s_rx_flag;          /* 解析状态机 0..5 */
static uint8_t  s_rx_len;           /* 本帧总长度 */
static uint8_t  s_rx_func;          /* 本帧功能码 */
static uint8_t  s_rx_cnt;           /* 本帧已收数据字节数 */
static uint8_t  s_rx_sum;           /* 本帧累加校验 */
static uint8_t  s_rx_data[YB_RX_MAX];

static uint32_t s_frames;   /* 32-bit: uint8 wraps at 255 and fakes a stall */           /* 校验通过的帧数 */
static uint8_t  s_bad_frames;       /* 校验失败的帧数 */
static uint8_t  s_last_func;        /* 最近收到的功能码（诊断/对应 VOFA ch4） */
static uint8_t  s_req_pending;      /* 已发请求、还没收到回帧（应答式：上一帧没回来就不追问） */
static uint32_t s_req_tick;         /* 上次发请求的时刻（超时当丢帧，重发） */
static uint8_t  s_calib_state;      /* 校准回执状态（非 0 = 完成） */
static uint8_t  s_online;
static uint8_t  s_status;           /* 0 = 正常；1 = 这周期没收到帧；2 = 连续失败（离线） */
static uint16_t s_err_cnt;
static uint32_t s_last_tick;
static uint32_t s_last_frame_tick;  /* 最近收到有效帧的时刻（在线判断用） */
static uint8_t  s_version;
static float    s_yaw_deg;          /* 模块原始 yaw（已转度） */
static float    s_yaw_zero;         /* 软件零点（ResetYaw 记的） */
static float    s_roll_deg, s_pitch_deg;
static float    s_gyro_z_dps;
static float    s_trim_dps;         /* 航向归零时测出来的漂移率（度/s），之后按时间抵消 */
static uint32_t s_trim_tick;        /* 测漂移率那一刻（= 零点锚点） */

static float yb_wrap180(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* 小端解析（模块发的是小端） */
static int16_t yb_le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static float yb_le_f32(const uint8_t *p)
{
    union { uint32_t u; float f; } v;
    v.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return v.f;
}

/* 组帧并发送：LEN = 整帧长度 = 4（头2+长度+功能）+ nparam + 1（校验） */
static void yb_send(uint8_t func, uint8_t p0, uint8_t p1, uint8_t nparam)
{
    uint8_t f[8];
    uint8_t i;
    uint8_t sum = 0u;

    if (s_uart == 0) return;

    f[0] = YB_H1;
    f[1] = YB_H2;
    f[2] = (uint8_t)(5u + nparam);
    f[3] = func;
    if (nparam >= 1u) f[4] = p0;
    if (nparam >= 2u) f[5] = p1;

    for (i = 0u; i < (uint8_t)(4u + nparam); i++) sum = (uint8_t)(sum + f[i]);
    f[4u + nparam] = sum;

    (void)HAL_UART_Transmit(s_uart, f, (uint16_t)(5u + nparam), (uint32_t)IMU_TX_TIMEOUT_MS);
}

static void yb_request(uint8_t func) { yb_send(YB_F_REQ, func, 0x00u, 2u); }
static void yb_set_rate(uint8_t hz)  { yb_send(YB_F_RATE, hz, 0x5Fu, 2u); }
static void yb_set_algo(uint8_t a)   { yb_send(YB_F_ALGO, a, 0x5Fu, 2u); }
static void yb_set_calib(void)       { yb_send(YB_F_CALIB, 0x01u, 0x5Fu, 2u); }

/* 一帧校验通过后：按功能码把数据搬进静态变量 */
static void yb_handle_frame(uint8_t func, const uint8_t *d, uint8_t len)
{
    s_frames++;
    s_last_func       = func;
    s_last_frame_tick = HAL_GetTick();
    /* 回帧到了 → 下一个请求可以发了（0x26 姿态、0x04 原始数据都算回帧；
       0x81 是我们主动校准的回执，不算"请求的应答"） */
    if ((func == YB_F_EULER) || (func == YB_F_RAW)) s_req_pending = 0u;

    if (func == YB_F_EULER && len >= 12u)
    {
        s_roll_deg  = yb_le_f32(&d[0]) * YB_RAD2DEG;
        s_pitch_deg = yb_le_f32(&d[4]) * YB_RAD2DEG;
        s_yaw_deg   = yb_le_f32(&d[8]) * YB_RAD2DEG;
    }
    else if (func == YB_F_RAW && len >= 12u)
    {
        s_gyro_z_dps = (float)yb_le_i16(&d[10]) * (YB_GYRO_LSB / 32767.0f);
    }
    else if (func == YB_F_VERSION && len >= 3u)
    {
        s_version = d[0];
    }
    else if (func == YB_F_STATE && len >= 2u)
    {
        /* 回执帧：官方库的排布是 data = [被回执的功能码, 状态]（厂商 YbImuSerialLib：
           _rx_func = ext_data[0]; _rx_state = ext_data[1]，校准成功 = 1）。
           这里两种排布都认，免得模块固件把顺序反过来时"校准成功却判成失败" */
        if (d[0] == YB_F_CALIB)      s_calib_state = d[1];
        else if (d[1] == YB_F_CALIB) s_calib_state = d[0];
        else                          s_calib_state = d[1];
    }
}

/* 接收中断：逐字节喂状态机（状态机 = 官方 YbImuSerialLib 的移植） */
void IMU_RxIsr(UART_HandleTypeDef *huart)
{
    uint8_t b = s_rx_byte;
    (void)huart;

    switch (s_rx_flag)
    {
    case 0u:                                  /* 等 0x7E */
        if (b == YB_H1) s_rx_flag = 1u;
        break;

    case 1u:                                  /* 等 0x23 */
        if (b == YB_H2)      s_rx_flag = 2u;
        else if (b == YB_H1) s_rx_flag = 1u;  /* 连续 7E：仍在等第二字节 */
        else                 s_rx_flag = 0u;
        break;

    case 2u:                                  /* 长度：整帧字节数 */
        s_rx_len = b;
        if ((b >= 6u) && (b <= (uint8_t)(YB_RX_MAX + 5u)))
        {
            s_rx_sum  = (uint8_t)(YB_H1 + YB_H2 + b);
            s_rx_flag = 3u;
        }
        else
        {
            s_rx_flag = 0u;
        }
        break;

    case 3u:                                  /* 功能码 */
        s_rx_func = b;
        s_rx_sum  = (uint8_t)(s_rx_sum + b);
        s_rx_cnt  = 0u;
        s_rx_flag = 4u;
        break;

    case 4u:                                  /* 数据区 */
        if (s_rx_cnt < YB_RX_MAX) s_rx_data[s_rx_cnt] = b;
        s_rx_cnt = (uint8_t)(s_rx_cnt + 1u);
        s_rx_sum = (uint8_t)(s_rx_sum + b);
        if (s_rx_cnt >= (uint8_t)(s_rx_len - 5u)) s_rx_flag = 5u;   /* 只剩校验字节 */
        break;

    default:                                  /* 校验字节 */
        if (b == s_rx_sum)
        {
            uint8_t n = (s_rx_cnt <= YB_RX_MAX) ? s_rx_cnt : YB_RX_MAX;
            yb_handle_frame(s_rx_func, s_rx_data, n);
        }
        else if (s_bad_frames < 0xFFu)
        {
            s_bad_frames++;                   /* 校验错：不计入在线判断（可能只是半个帧） */
        }
        s_rx_flag = 0u;
        break;
    }

    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);   /* 立刻挂上下一字节 */
}

/* 串口"卡住"时的自愈：冲掉接收、重置状态机、重新挂中断
   （比如线拔插瞬间的帧错误/溢出错误会让 HAL 停在 BUSY） */
static void yb_recover(void)
{
    if (s_uart == 0) return;
    (void)HAL_UART_AbortReceive(s_uart);
    s_rx_flag = 0u;
    s_err_cnt = 0u;
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

/* 串口出错回调（帧错误/溢出 ORE 等）：★必须给【所有】串口重新挂接收
   为什么：HAL 在 UART 出错时会 UART_EndRxTransfer() 把接收彻底停掉，
   如果这里不重新挂，该串口就【永久收不到数据】直到复位。
   现象：一开始正常，跑一会儿蓝牙命令没反应 / 底盘转速和位置全变 0（N0/4）。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    volatile uint32_t tmp;

    if (huart == 0) return;

    /* 清 ORE/FE/NE/PE：F4 的规矩是先读 SR 再读 DR */
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;

    /* ① IMU（USART6） */
    if ((s_uart != 0) && (huart->Instance == s_uart->Instance))
    {
        s_rx_flag = 0u;
        (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
        return;
    }

#if ARM_ENABLE
    /* ② 蓝牙/VOFA 主口（UART4）：接收归 ARM 的拖条解析器 */
    if (huart->Instance == VOFA_UART->Instance)
    {
        ARM_RearmRx();
        return;
    }
#endif

    /* ③ 视觉（UART7）：解析器要复位状态并重挂接收 */
    Vision_OnUartError(huart);
#if SCAN_ENABLE
    Scan_OnUartError(huart);
#endif

    /* ④ 步进总线（USART3）：接收归 PD42S1 */
#if STEPPER_ENABLE
    if (huart->Instance == STEPPER_UART_INSTANCE)
    {
        PD42S1_RearmRx();
        return;
    }
#endif
}

void IMU_Init(void)
{
    uint16_t waited = 0u;

    s_uart            = IMU_UART;        /* config.h：默认 &huart4 */
    s_online          = 0u;
    s_status          = 2u;              /* 握手成功前先当离线 */
    s_err_cnt         = 0u;
    s_version         = 0u;
    s_last_func       = 0u;
    s_calib_state     = 0u;
    s_frames          = 0u;
    s_bad_frames      = 0u;
    s_req_pending     = 0u;
    s_req_tick        = 0u;
    s_rx_flag         = 0u;
    s_rx_cnt          = 0u;
    s_rx_len          = 0u;
    s_yaw_deg         = 0.0f;
    s_yaw_zero        = 0.0f;
    s_roll_deg        = 0.0f;
    s_pitch_deg       = 0.0f;
    s_gyro_z_dps      = 0.0f;
    s_trim_dps        = 0.0f;            /* 漂移率要等第一次 TrimYaw/ResetYaw 才测出来 */
    s_trim_tick       = HAL_GetTick();
    s_last_tick       = HAL_GetTick();
    s_last_frame_tick = s_last_tick;

    if (s_uart == 0) return;

    /* 先挂接收中断、再发请求：顺序反了会丢掉第一帧应答 */
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);

    /* 上电握手：问一次版本号，等模块回帧（"有帧" = 模块在，等价于 I²C 版的读版本号） */
    yb_request(YB_F_VERSION);
    while (waited < (uint16_t)IMU_UART_PROBE_MS)
    {
        HAL_Delay(10u);
        waited = (uint16_t)(waited + 10u);
        if (s_frames != 0u) break;
    }

    if (s_frames != 0u)
    {
        s_online = 1u;
        s_status = 0u;
    }
    else
    {
        s_status = 2u;                   /* 一帧都没收到 → 接线/波特率/模块没上电 */
    }

    /* 再要一次姿态（主动上报模式下这帧也会被忽略，无害） */
    yb_request(YB_F_EULER);
    s_last_tick = HAL_GetTick();

#if IMU_USE_ACTIVE_REPORT
    (void)IMU_SetReportRate((uint8_t)IMU_REPORT_RATE_HZ);   /* 让模块自己按频率上报 */
#endif

    /* ⚠️ 这里【不要】默认写融合算法类型（0x61）：那是存模块 flash 的一次性设置，
       每次上电写会让模块融合算法重启（表现：刚上电一阵姿态是 0 / 乱跳）。
       要设就单独调一次 IMU_SetAlgoType(IMU_ALGO_TYPE)，或者把下面开关改 1。 */
#if IMU_SET_ALGO_ON_BOOT
    yb_set_algo((uint8_t)IMU_ALGO_TYPE);
    HAL_Delay(20u);
#endif

    /* 一次性校准（陀螺+加计）：车必须静止、水平；结果存模块 flash，做完就把开关关掉 */
#if IMU_CALIB_ON_BOOT
    (void)IMU_Calibrate();
#endif
}

void IMU_Update(void)
{
#if IMU_UPDATE_ENABLE
    uint32_t now = HAL_GetTick();

    /* 限频：默认 10ms（100Hz）一轮，别在主循环里猛发串口 */
    if ((uint32_t)(now - s_last_tick) < (uint32_t)IMU_PERIOD_MS) return;
    s_last_tick = now;

    /* 在线判断：卡"最近收到帧的时刻"这个时间窗，而不是"这一周期有没有帧"
       → 偶尔丢一两帧不会闪成"失败"，也不会让按 IMU_IsOnline() 选判据的测试中途改判据
       ⚠️ 收不到帧时**保留上一次的角度值**（不清零），否则波形上看到"掉到 0"
          分不清是"通信断了"还是"模块真输出 0" → 用 IMU_GetStatus() 区分 */
    if ((uint32_t)(now - s_last_frame_tick) <= (uint32_t)IMU_UART_TIMEOUT_MS)
    {
        s_online  = 1u;
        s_status  = 0u;
        s_err_cnt = 0u;
    }
    else
    {
        if (s_err_cnt < 0xFFFFu) s_err_cnt++;
        s_status = (s_err_cnt >= (uint16_t)IMU_ERR_LIMIT) ? 2u : 1u;
        if (s_err_cnt >= (uint16_t)IMU_ERR_LIMIT)
        {
            s_online = 0u;
            yb_recover();                /* 连续失败到阈值 → 重挂串口接收 */
        }
    }

    /* 该不该发下一次请求：
       · 轮询模式（默认）：每周期发一次，但上一帧回来之前不追问
         → 模块慢的时候请求不会堆积、延迟不会越滚越大
       · 主动上报模式：平时不用问，只有断流（超过 IMU_UART_STREAM_MS 没帧）才补一次
       两种模式共用同一套"超时就当丢帧、补发一次"的逻辑，避免死等 */
    {
        uint8_t need;
#if IMU_USE_ACTIVE_REPORT
        need = ((uint32_t)(now - s_last_frame_tick) > (uint32_t)IMU_UART_STREAM_MS) ? 1u : 0u;
#else
        need = 1u;
#endif
        if ((need != 0u) &&
            ((s_req_pending == 0u) ||
             ((uint32_t)(now - s_req_tick) > (uint32_t)IMU_REQ_TIMEOUT_MS)))
        {
#if IMU_GYRO_POLL_DIV > 0
            /* 每 N 次轮询里夹一次"原始数据帧"（0x04）—— 只为了拿到陀螺 Z：
               静止时它读数的绝对值就是零偏，×60 就是"每分钟漂多少度"，
               比盯着 yaw 看 1 分钟直观得多（调完可以改 0 省点带宽） */
            static uint8_t s_poll_cnt;

            s_poll_cnt++;
            if ((uint8_t)(s_poll_cnt % (uint8_t)IMU_GYRO_POLL_DIV) == 0u)
            {
                yb_request(YB_F_RAW);
            }
            else
            {
                yb_request(YB_F_EULER);
            }
#else
            yb_request(YB_F_EULER);
#endif
            s_req_pending = 1u;
            s_req_tick    = now;
        }
    }
#endif /* IMU_UPDATE_ENABLE */
}

uint8_t IMU_GetStatus(void)     { return s_status; }
uint8_t IMU_GetAddr(void)       { return IMU_ADDR_7BIT; }
uint8_t IMU_GetScanResult(void) { return s_last_func; }   /* 最近收到的功能码：0x26 = 姿态帧正常 */
uint8_t IMU_GetCalibState(void) { return s_calib_state; } /* 校准回执状态：非 0 = 上电那次校准拿到了回执 */
float   IMU_GetYawTrimDpm(void) { return s_trim_dps * 60.0f; }  /* 刚测出来的漂移率（度/分） */
uint8_t IMU_GetFrames(void)     { return (uint8_t)s_frames; }
uint32_t IMU_GetFrames32(void)  { return s_frames; }
uint8_t IMU_GetBadFrames(void)  { return s_bad_frames; }

uint8_t IMU_IsOnline(void)      { return s_online; }
uint8_t IMU_GetVersion(void)    { return s_version; }

float IMU_GetGyroZDps(void)     { return s_gyro_z_dps * IMU_GYRO_Z_SIGN; }
float IMU_GetRollDeg(void)      { return s_roll_deg; }
float IMU_GetPitchDeg(void)     { return s_pitch_deg; }

float IMU_GetYawDeg(void)
{
    /* 模块 yaw → 减零点 → 【再扣掉按测得的漂移率累积的漂移】→ 翻符号 → 归一到 ±180
       ★为什么要扣：六轴融合的 yaw 没有绝对参考，陀螺零偏会一路积成角度漂移。
         模块自带的校准如果没生效（我们这台就是：回执收不到），漂移能到几十度/分。
         这里的做法是"每次航向归零时先静止采样，测出漂移率，之后按时间线性抵消" ——
         不依赖模块固件，纯软件，重启/换场地都能重测 */
    {
        float t = (float)(HAL_GetTick() - s_trim_tick) * 0.001f;

        if (t < 0.0f) t = 0.0f;
        return yb_wrap180((s_yaw_deg - s_yaw_zero) - s_trim_dps * t) * IMU_GYRO_Z_SIGN;
    }
}

/* 航向归零 = 把"现在"当 0°；同时把【锚点】挪到此刻（之前测到的漂移率继续用） */
void IMU_ResetYaw(void)
{
    s_yaw_zero  = s_yaw_deg;
    s_trim_tick = HAL_GetTick();
}

/* ★带漂移测量的航向归零：静止采样 ms 毫秒 → 算出漂移率(度/s) → 之后按时间抵消它
   车在动的时候别调（测出来的会是"车的真实转动"，被 IMU_TRIM_MAX_DPS 挡掉、按 0 处理） */
uint8_t IMU_TrimYaw(uint16_t ms)
{
    float    y0, y1, d, dt, rate;
    uint32_t t0, t1;

    if (s_uart == 0) { IMU_ResetYaw(); return 0u; }
    if (ms == 0u)    { IMU_ResetYaw(); return 0u; }

    y0 = s_yaw_deg;
    t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < (uint32_t)ms)
    {
        HAL_Delay(10u);                 /* 阻塞采样：调用点必须是"车已停稳"的时刻 */
    }
    y1 = s_yaw_deg;
    t1 = HAL_GetTick();

    dt = (float)(t1 - t0) * 0.001f;
    d  = yb_wrap180(y1 - y0);
    rate = (dt > 0.01f) ? (d / dt) : 0.0f;

    if ((rate > IMU_TRIM_MAX_DPS) || (rate < -IMU_TRIM_MAX_DPS))
    {
        rate = 0.0f;                    /* 超出合理漂移 → 车在动/被碰，这次不采信 */
    }

    s_trim_dps  = rate;
    s_yaw_zero  = y1;                   /* 零点定在采样结束的那一刻（和 s_trim_tick 对齐） */
    s_trim_tick = t1;

    return 1u;
}

uint8_t IMU_Calibrate(void)
{
    uint16_t waited = 0u;

    if (s_uart == 0) return 0u;

    s_calib_state = 0u;
    yb_set_calib();

    /* 等 0x81 回执（注意：校准期间姿态帧照旧在发，所以不能拿"有没有新帧"当判据） */
    while (waited < (uint16_t)IMU_CALIB_WAIT_MS)
    {
        HAL_Delay(50u);
        waited = (uint16_t)(waited + 50u);
        if (s_calib_state != 0u) return 1u;
    }
    return 0u;
}

uint8_t IMU_SetAlgoType(uint8_t algo6_or_9)
{
    if (algo6_or_9 != 6u && algo6_or_9 != 9u) return 0u;
    if (s_uart == 0) return 0u;
    yb_set_algo(algo6_or_9);
    return 1u;                       /* 该命令没有回执，发出即算成功 */
}

/* 设置模块主动上报频率（10~100Hz，存模块里，只需设一次）
   → 只有在 IMU_USE_ACTIVE_REPORT = 1 时才需要；我们默认是轮询，不用调 */
uint8_t IMU_SetReportRate(uint8_t hz)
{
    if (s_uart == 0) return 0u;
    if (hz < 10u)  hz = 10u;
    if (hz > 100u) hz = 100u;
    yb_set_rate(hz);
    return 1u;
}

#elif IMU_ENABLE && defined(HAL_I2C_MODULE_ENABLED)

/* ---- YbImu 寄存器（功能码） ---- */
#define YB_REG_VERSION   0x01u
#define YB_REG_GYRO      0x0Au
#define YB_REG_EULER     0x26u
#define YB_REG_ALGO      0x61u
#define YB_REG_CALIB_IMU 0x70u

#define YB_ADDR          ((uint16_t)(s_addr << 1))            /* HAL 要 8 位地址（s_addr 是 7 位） */

#define YB_RAD2DEG       57.2957795f
#define YB_GYRO_LSB      2000.0f                      /* 满量程 ±2000 dps */
#define YB_I2C_TIMEOUT   50u

static I2C_HandleTypeDef *s_i2c;
static uint8_t  s_online;
static uint8_t  s_status;         /* 0 = 正常；1 = 这次读失败；2 = 一直失败（离线） */
static uint16_t s_err_cnt;        /* 连续失败次数（用于自动恢复） */
static uint32_t s_last_tick;      /* 限频用 */
static uint8_t  s_addr;           /* 实际使用的 7 位地址（默认 IMU_ADDR_7BIT，扫描后可改） */
static uint8_t  s_scan_found;     /* 扫描到的地址（0 = 总线上什么都没找到） */
static uint8_t  s_version;
static float    s_yaw_deg;        /* 模块原始 yaw（已转度） */
static float    s_yaw_zero;       /* 软件零点（ResetYaw 记的） */
static float    s_roll_deg, s_pitch_deg;
static float    s_gyro_z_dps;

/* ---------------- 总线扫描：找出模块到底在哪个地址 ---------------- */
static uint8_t yb_scan(void)
{
    uint8_t a;

    if (s_i2c == 0) return 0u;

    for (a = 0x08u; a <= 0x77u; a++)
    {
        if (HAL_I2C_IsDeviceReady(s_i2c, (uint16_t)(a << 1), 3u, 20u) == HAL_OK)
        {
            return a;                    /* 找到第一个应答的地址 */
        }
    }
    return 0u;
}

/* ---------------- 底层读写 ---------------- */
static uint8_t yb_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (s_i2c == 0) return 0u;
    return (HAL_I2C_Mem_Read(s_i2c, YB_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, YB_I2C_TIMEOUT) == HAL_OK) ? 1u : 0u;
}

static uint8_t yb_write(uint8_t reg, uint8_t val)
{
    if (s_i2c == 0) return 0u;
    return (HAL_I2C_Mem_Write(s_i2c, YB_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1u, YB_I2C_TIMEOUT) == HAL_OK) ? 1u : 0u;
}

/* 小端解析（模块发的是小端） */
static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static float le_f32(const uint8_t *p)
{
    union { uint32_t u; float f; } v;
    v.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return v.f;
}

static float wrap180(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* ---------------- 对外接口 ---------------- */
void IMU_Init(void)
{
    uint8_t ver[3];
    uint8_t i;

    s_i2c        = IMU_I2C;          /* config.h 里配（默认 &hi2c1 = PB6/PB7） */
    s_online     = 0u;
    s_version    = 0u;
    s_yaw_deg    = 0.0f;
    s_yaw_zero   = 0.0f;
    s_roll_deg   = 0.0f;
    s_pitch_deg  = 0.0f;
    s_gyro_z_dps = 0.0f;

    for (i = 0u; i < 3u; i++) ver[i] = 0u;

    /* 读版本号 = 最轻量的"你在不在"检查（读不到就是没接/地址不对/I2C 没开） */
    s_status     = 0u;
    s_err_cnt    = 0u;
    s_last_tick  = HAL_GetTick();
    s_addr       = IMU_ADDR_7BIT;
    s_scan_found = 0u;

    /* ① 先降到 100kHz：弱上拉 / 杜邦线 / 有电机干扰时，400k 很容易全失败 */
    if (s_i2c != 0)
    {
        s_i2c->Init.ClockSpeed = IMU_I2C_SPEED_HZ;
        (void)HAL_I2C_Init(s_i2c);
    }

    /* ② 读版本号（最轻量的"你在不在"） */
    if (yb_read(YB_REG_VERSION, ver, 3u) == 0u)
    {
        /* ✗ 没应答 → 扫描总线，看模块到底在哪个地址（或者根本不在总线上） */
        uint8_t found = yb_scan();

        s_scan_found = found;
        if (found != 0u && found != s_addr)
        {
            s_addr = found;                 /* 采用扫到的地址再试一次 */
        }
    }

    if (yb_read(YB_REG_VERSION, ver, 3u) != 0u)
    {
        s_version = ver[0];
        s_online  = 1u;
        s_status  = 0u;
        s_scan_found = s_addr;

        /* ⚠️ 这里【不要】写融合算法类型（0x61）：那是"存模块 flash 的一次性设置"，
           每次上电写会让模块的融合算法重启（表现：刚上电一段时间姿态是 0 / 乱跳），
           还白白磨损 flash。要设就单独调一次 IMU_SetAlgoType(IMU_ALGO_TYPE)。
           如需上电自动设，把 config.h 的 IMU_SET_ALGO_ON_BOOT 改 1。 */
#if IMU_SET_ALGO_ON_BOOT
        (void)yb_write(YB_REG_ALGO, IMU_ALGO_TYPE);
        HAL_Delay(20u);
#endif
    }
    else
    {
        s_status = 2u;
    }
}

/* I²C 出错自愈：HAL 的 I²C 出错后会卡在 BUSY，必须 DeInit + 重新 Init 才能继续用 */
static void yb_recover(void)
{
    if (s_i2c == 0) return;
    (void)HAL_I2C_DeInit(s_i2c);
    MX_I2C1_Init();
    s_err_cnt = 0u;
}

void IMU_Update(void)
{
#if IMU_UPDATE_ENABLE
    uint8_t buf[12];
    uint32_t now = HAL_GetTick();

    /* 限频：默认 10ms（100Hz）读一次，别每轮主循环都占 I²C */
    if ((uint32_t)(now - s_last_tick) < (uint32_t)IMU_PERIOD_MS) return;
    s_last_tick = now;

    if (yb_read(YB_REG_EULER, buf, 12u) != 0u)
    {
        s_roll_deg  = le_f32(&buf[0]) * YB_RAD2DEG;
        s_pitch_deg = le_f32(&buf[4]) * YB_RAD2DEG;
        s_yaw_deg   = le_f32(&buf[8]) * YB_RAD2DEG;
        s_online    = 1u;
        s_status    = 0u;
        s_err_cnt   = 0u;
    }
    else
    {
        /* ⚠️ 读失败时**保留上一次的值**（不要清零），否则波形上会看到"掉到 0"
           到底是"通信断了"还是"模块输出 0"，用 IMU_GetStatus() 区分 */
        if (s_err_cnt < 0xFFFFu) s_err_cnt++;
        s_status = (s_err_cnt >= IMU_ERR_LIMIT) ? 2u : 1u;
        s_online = 0u;
        if (s_err_cnt == (uint16_t)IMU_ERR_LIMIT) yb_recover();   /* 连续失败到阈值 → 重初始化 I²C */
        return;
    }

    if (yb_read(YB_REG_GYRO, buf, 6u) != 0u)
    {
        s_gyro_z_dps = (float)le_i16(&buf[4]) * (YB_GYRO_LSB / 32767.0f);
    }
#endif
}

uint8_t IMU_GetStatus(void) { return s_status; }
uint8_t IMU_GetAddr(void)   { return s_addr; }          /* 实际用的 7 位地址 */
uint8_t IMU_GetScanResult(void) { return s_scan_found; } /* 扫描结果：0 = 总线上没找到任何器件 */
uint8_t IMU_GetCalibState(void) { return 0u; }           /* I²C 版没有校准回执 */
uint8_t IMU_GetFrames(void)    { return s_online ? 1u : 0u; }                        /* I²C 版没有帧计数 */
uint8_t IMU_GetBadFrames(void) { return (s_err_cnt > 255u) ? 255u : (uint8_t)s_err_cnt; }
void    IMU_RxIsr(UART_HandleTypeDef *huart) { (void)huart; }                        /* I²C 版不用串口中断 */
uint8_t IMU_SetReportRate(uint8_t hz) { (void)hz; return 0u; }                       /* 只有串口版有上报频率 */

uint8_t IMU_IsOnline(void)      { return s_online; }
uint8_t IMU_GetVersion(void)    { return s_version; }

float IMU_GetGyroZDps(void)     { return s_gyro_z_dps * IMU_GYRO_Z_SIGN; }
float IMU_GetRollDeg(void)      { return s_roll_deg; }
float IMU_GetPitchDeg(void)     { return s_pitch_deg; }

float IMU_GetYawDeg(void)
{
    /* 模块 yaw → 减零点 → 按安装方向翻符号 → 归一到 ±180 */
    return wrap180((s_yaw_deg - s_yaw_zero) * IMU_GYRO_Z_SIGN);
}

void IMU_ResetYaw(void)
{
    s_yaw_zero = s_yaw_deg;          /* 软件零点：把"现在"当 0° */
}

/* I²C 版：不用漂移抵消（模块 I²C 路子很少用），给个空实现保证 main/chassis 不用改 */
uint8_t IMU_TrimYaw(uint16_t ms) { (void)ms; IMU_ResetYaw(); return 0u; }
float   IMU_GetYawTrimDpm(void)  { return 0.0f; }

uint8_t IMU_Calibrate(void)
{
    uint8_t st = 0u;
    uint16_t waited = 0u;

    if (yb_write(YB_REG_CALIB_IMU, 1u) == 0u) return 0u;

    /* 轮询直到非 0（官方例程是轮询该寄存器） */
    while (waited < IMU_CALIB_WAIT_MS)
    {
        HAL_Delay(100u);
        waited = (uint16_t)(waited + 100u);
        st = 0u;
        if (yb_read(YB_REG_CALIB_IMU, &st, 1u) != 0u && st != 0u) return 1u;
    }
    return 0u;
}

uint8_t IMU_SetAlgoType(uint8_t algo6_or_9)
{
    if (algo6_or_9 != 6u && algo6_or_9 != 9u) return 0u;
    return yb_write(YB_REG_ALGO, algo6_or_9);
}

#else   /* ---------------- 未启用 / 没开 I2C：全部零值桩 ---------------- */

void    IMU_Init(void)                 {}
void    IMU_Update(void)               {}
uint8_t IMU_IsOnline(void)             { return 0u; }
float   IMU_GetGyroZDps(void)          { return 0.0f; }
float   IMU_GetYawDeg(void)            { return 0.0f; }
float   IMU_GetRollDeg(void)           { return 0.0f; }
float   IMU_GetPitchDeg(void)          { return 0.0f; }
void    IMU_ResetYaw(void)             {}
uint8_t IMU_TrimYaw(uint16_t ms)       { (void)ms; return 0u; }
float   IMU_GetYawTrimDpm(void)        { return 0.0f; }
uint8_t IMU_GetVersion(void)           { return 0u; }
uint8_t IMU_Calibrate(void)            { return 0u; }
uint8_t IMU_SetAlgoType(uint8_t a)     { (void)a; return 0u; }
uint8_t IMU_GetStatus(void)            { return 2u; }   /* 2 = 离线（没启用 IMU） */
uint8_t IMU_GetAddr(void)              { return 0u; }
uint8_t IMU_GetScanResult(void)        { return 0u; }
uint8_t IMU_GetCalibState(void)        { return 0u; }
uint8_t IMU_GetFrames(void)            { return 0u; }
uint8_t IMU_GetBadFrames(void)         { return 0u; }
void    IMU_RxIsr(UART_HandleTypeDef *h) { (void)h; }
uint8_t IMU_SetReportRate(uint8_t hz)  { (void)hz; return 0u; }

#endif /* IMU_ENABLE && HAL_I2C_MODULE_ENABLED */
