/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ 上位机波形/调试输出（通用封装）
  *
  *  JustFloat 协议：N 个 float32（小端）连续排布，最后 4 字节固定 00 00 80 7F。
  *  VOFA+ 里数据格式选 JustFloat，通道数自动识别。
  *
  *  本文件不依赖云台/步进等任何业务模块，谁都可以直接调。
  ******************************************************************************
  */

#include "vofa.h"
#include <string.h>

/* 未使能 VOFA、或工程里还没加串口（CubeMX 没勾 USART，HAL_UART_MODULE_ENABLED 没开）时，
   下面全部退化成空函数：main.c 不用改，也不会编译报错 */
#if VOFA_OUTPUT_ENABLE && defined(HAL_UART_MODULE_ENABLED)

#include <stdarg.h>
#include <stdio.h>
#include "usart.h"      /* CubeMX 加了串口之后才有（里面声明 huart1 / huart2 …） */

/* 帧尾：JustFloat 固定 00 00 80 7F（0x7F800000 是 float 的 +Inf，不会被当成数据） */
#define VOFA_TAIL_0     0x00
#define VOFA_TAIL_1     0x00
#define VOFA_TAIL_2     0x80
#define VOFA_TAIL_3     0x7F

static UART_HandleTypeDef *s_uart      = VOFA_UART;   /* 默认用 config.h 里的串口 */
static uint8_t             s_enable    = 1;
static uint16_t            s_rate_hz   = VOFA_TX_HZ;
static uint32_t            s_last_tick = 0;
static void              (*s_source)(void) = 0;

/* ---- 第二路输出（蓝牙 HC-05）：同一帧、自己的频率 ---- */
static UART_HandleTypeDef *s_uart2     = 0;
static uint16_t            s_rate2_hz  = 0;
static uint32_t            s_last2_tick = 0;
static uint8_t             s_en1       = 1;           /* 主口（有线）是否发 */
static uint8_t             s_tx2_now   = 0;           /* 本次调用要不要也发第二路（VOFA_Task 里定） */

static uint8_t s_txbuf[VOFA_MAX_CH * 4u + 4u];
static uint8_t s_txbuf1it[VOFA_MAX_CH * 4u + 4u];  /* 主口【中断发送】专用缓冲：发着的时候不能被下一帧改写 */
static uint8_t s_txbuf2[VOFA_MAX_CH * 4u + 4u];   /* 第二路（蓝牙）走中断发送，要自己的缓冲 */
static char    s_txtbuf[160];
static float   s_last_tx_ms;                      /* 上一帧阻塞发送耗时(ms)：诊断"主循环被谁拖慢" */

/* 诊断用：上一帧发送耗时(ms) / 当前主口波特率（÷1000，115200 → 115.2） */
float   VOFA_GetTxMs(void)      { return s_last_tx_ms; }
float   VOFA_GetUartBaudK(void) { return (s_uart != 0) ? ((float)s_uart->Init.BaudRate / 1000.0f) : 0.0f; }

/* ---------------- 初始化 / 配置 ---------------- */

void VOFA_Init(void)
{
    s_uart      = VOFA_UART;
    s_enable    = 1;
    s_en1       = 1;
    s_rate_hz   = VOFA_TX_HZ;
    s_last_tick = HAL_GetTick();
    s_source    = 0;
    memset(s_txbuf, 0, sizeof(s_txbuf));
}

void VOFA_SetUart(UART_HandleTypeDef *huart)
{
    if (huart != 0)
    {
        s_uart = huart;
    }
}

void VOFA_SetUart2(UART_HandleTypeDef *huart, uint16_t hz)
{
    s_uart2     = huart;                 /* 传 0 = 关掉第二路 */
    s_rate2_hz  = hz;
    s_last2_tick = HAL_GetTick();
}

void VOFA_SetWiredEnable(uint8_t en)
{
    s_en1 = en ? 1u : 0u;
}

uint8_t VOFA_IsReady(void)
{
    return (s_uart != 0) ? 1u : 0u;
}

void VOFA_SetEnable(uint8_t en)
{
    s_enable = en ? 1u : 0u;
}

uint8_t VOFA_IsEnable(void)
{
    return s_enable;
}

void VOFA_SetRate(uint16_t hz)
{
    s_rate_hz = hz;
}

/* ---------------- 发送 ---------------- */

uint8_t VOFA_SendArray(const float *ch, uint8_t n)
{
    uint16_t len;
    uint8_t  i;
    HAL_StatusTypeDef st;

    /* 主口没开/没串口时，只要第二路（蓝牙）这一轮要发，就继续组装数据 */
    if ((s_enable == 0) || (ch == 0)) return 0u;
    if (((s_uart == 0) || (s_en1 == 0u)) && ((s_tx2_now == 0u) || (s_uart2 == 0))) return 0u;
    if (n == 0u || n > VOFA_MAX_CH)              return 0u;

    /* N 个 float32 小端 */
    for (i = 0u; i < n; i++)
    {
        memcpy(&s_txbuf[i * 4u], &ch[i], 4u);
    }
    /* 帧尾 00 00 80 7F */
    s_txbuf[n * 4u + 0u] = VOFA_TAIL_0;
    s_txbuf[n * 4u + 1u] = VOFA_TAIL_1;
    s_txbuf[n * 4u + 2u] = VOFA_TAIL_2;
    s_txbuf[n * 4u + 3u] = VOFA_TAIL_3;

    len = (uint16_t)((uint16_t)n * 4u + 4u);

    /* 阻塞发送：32 字节 @115200 约 2.8ms，只能放主循环。
       嫌占时间可以改成 HAL_UART_Transmit_DMA（要在 CubeMX 里给串口加 DMA） */
    /* ★主口发送：走【中断发送，非阻塞】
       —— 以前用阻塞发送，115200 下 44 字节要 3.8ms、9600 下要 46ms，
          直接把控制环（20ms）拖慢，PID 的 dt 都跟着乱。
       这一帧还没发完就跳过本帧（丢一帧没关系，别拖住控制）。
       ⚠️ VOFA_IT_TX=0 时退回老的阻塞发送（需要"绝不丢帧"时用） */
#if VOFA_IT_TX
    st = HAL_OK;
    if ((s_uart != 0) && (s_en1 != 0u))
    {
        uint32_t tx0 = HAL_GetTick();

        if (s_uart->gState == HAL_UART_STATE_READY)
        {
            memcpy(s_txbuf1it, s_txbuf, len);
            (void)HAL_UART_Transmit_IT(s_uart, s_txbuf1it, len);
        }
        s_last_tx_ms = (float)(uint32_t)(HAL_GetTick() - tx0);
    }
#else
    st = HAL_OK;
    if ((s_uart != 0) && (s_en1 != 0u))
    {
        uint32_t tx0 = HAL_GetTick();

        st = HAL_UART_Transmit(s_uart, s_txbuf, len, 50u);
        s_last_tx_ms = (float)(uint32_t)(HAL_GetTick() - tx0);   /* 诊断：这一帧发了多久 */
    }
#endif

    /* ★第二路（蓝牙）：同一帧再发一份 —— 走【中断发送】，不阻塞主循环！
       为什么必须这样：蓝牙模块常见是 9600 波特，36 字节 = 37.5ms 的阻塞发送，
       每帧都等它发完会把主循环从 100Hz 拖到 20Hz（控制环、PID 全跟着变慢）。
       上一帧还没发完（gState 不是 READY）就**直接跳过这一帧** —— 丢帧没关系，别拖住控制。 */
    if ((s_tx2_now != 0u) && (s_uart2 != 0))
    {
        memcpy(s_txbuf2, s_txbuf, len);
        if (s_uart2->gState == HAL_UART_STATE_READY)
        {
            (void)HAL_UART_Transmit_IT(s_uart2, s_txbuf2, len);
        }
    }

    return (st == HAL_OK) ? 1u : 0u;
}

uint8_t VOFA_Send(uint8_t n, ...)
{
    float   ch[VOFA_MAX_CH];
    va_list ap;
    uint8_t i;

    if (n == 0u || n > VOFA_MAX_CH) return 0u;

    va_start(ap, n);
    for (i = 0u; i < n; i++)
    {
        ch[i] = (float)va_arg(ap, double);   /* float 在可变参数里会被提升成 double */
    }
    va_end(ap);

    return VOFA_SendArray(ch, n);
}

uint8_t VOFA_Send1(float v)
{
    return VOFA_SendArray(&v, 1u);
}

uint8_t VOFA_Send2(float v1, float v2)
{
    float ch[2];
    ch[0] = v1; ch[1] = v2;
    return VOFA_SendArray(ch, 2u);
}

uint8_t VOFA_Send3(float v1, float v2, float v3)
{
    float ch[3];
    ch[0] = v1; ch[1] = v2; ch[2] = v3;
    return VOFA_SendArray(ch, 3u);
}

void VOFA_Printf(const char *fmt, ...)
{
    va_list ap;
    int len;

    if (s_enable == 0 || s_uart == 0 || fmt == 0) return;

    va_start(ap, fmt);
    len = vsnprintf(s_txtbuf, sizeof(s_txtbuf), fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if (len > (int)sizeof(s_txtbuf)) len = (int)sizeof(s_txtbuf);

    HAL_UART_Transmit(s_uart, (uint8_t *)s_txtbuf, (uint16_t)len, 50u);
}

/* ---------------- 周期任务 ---------------- */

void VOFA_SetSource(void (*fn)(void))
{
    s_source = fn;
}

void VOFA_Task(void)
{
    uint32_t now;
    uint8_t  due1, due2;
    uint32_t ms1, ms2;

    if (s_enable == 0 || s_source == 0) return;

    now = HAL_GetTick();
    ms1 = (s_rate_hz  != 0u) ? (uint32_t)(1000u / s_rate_hz)  : 0u;
    ms2 = (s_rate2_hz != 0u) ? (uint32_t)(1000u / s_rate2_hz) : 0u;

    /* 主口（有线）该不该发 */
    due1 = ((s_uart != 0) && (s_en1 != 0u) &&
            ((ms1 == 0u) || ((uint32_t)(now - s_last_tick) >= ms1))) ? 1u : 0u;

    /* 第二路（蓝牙）该不该发 —— 频率可以比主口低很多 */
    due2 = ((s_uart2 != 0) &&
            ((ms2 == 0u) || ((uint32_t)(now - s_last2_tick) >= ms2))) ? 1u : 0u;

    if ((due1 == 0u) && (due2 == 0u)) return;

    if (due1 != 0u) s_last_tick  = now;
    if (due2 != 0u) s_last2_tick = now;

    /* 数据源只跑一次，两个口共用这一帧 */
    s_tx2_now = due2;
    s_source();
    s_tx2_now = 0u;
}

#else   /* ---------------- 未使能：空实现（main.c 不用改） ---------------- */

void    VOFA_Init(void)            {}
void    VOFA_SetEnable(uint8_t en) { (void)en; }
uint8_t VOFA_IsEnable(void)        { return 0u; }
void    VOFA_SetRate(uint16_t hz)  { (void)hz; }
void    VOFA_SetSource(void (*fn)(void)) { (void)fn; }
void    VOFA_Task(void)            {}
uint8_t VOFA_IsReady(void)         { return 0u; }
uint8_t VOFA_Send(uint8_t n, ...)  { (void)n; return 0u; }
uint8_t VOFA_SendArray(const float *ch, uint8_t n) { (void)ch; (void)n; return 0u; }
uint8_t VOFA_Send1(float v)        { (void)v; return 0u; }
uint8_t VOFA_Send2(float v1, float v2) { (void)v1; (void)v2; return 0u; }
uint8_t VOFA_Send3(float v1, float v2, float v3) { (void)v1; (void)v2; (void)v3; return 0u; }
void    VOFA_Printf(const char *fmt, ...) { (void)fmt; }
#if defined(HAL_UART_MODULE_ENABLED)
void    VOFA_SetUart(UART_HandleTypeDef *huart) { (void)huart; }
void    VOFA_SetUart2(UART_HandleTypeDef *h, uint16_t hz) { (void)h; (void)hz; }
#endif
void    VOFA_SetWiredEnable(uint8_t en) { (void)en; }
float   VOFA_GetTxMs(void)      { return 0.0f; }
float   VOFA_GetUartBaudK(void) { return 0.0f; }

#endif
