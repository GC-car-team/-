/**
  ******************************************************************************
  * @file    vision.c
  * @brief   B3 B3 帧解析 + FF cmd FE 下发（成品/视觉锁定契约）
  ******************************************************************************
  */
#include "vision.h"
#include "usart.h"
#include "gm6020.h"

#if VISION_ENABLE && defined(HAL_UART_MODULE_ENABLED)

#ifndef VISION_CAM_CX
#define VISION_CAM_CX  160
#endif
#ifndef VISION_CAM_CY
#define VISION_CAM_CY  120
#endif
#ifndef VISION_ALIGN_DEAD_PX
#define VISION_ALIGN_DEAD_PX  8
#endif

static UART_HandleTypeDef *s_uart;
static uint8_t  s_rx_byte;
static uint8_t  s_state;
static uint8_t  s_type;          /* 0xB1 / 0xB2 */
static uint8_t  s_pay[7];
static uint8_t  s_pay_i;

static volatile int16_t  s_x;
static volatile int16_t  s_y;
static volatile uint8_t  s_id;
static volatile uint8_t  s_kind;
static volatile uint8_t  s_has;
static volatile uint32_t s_tick;
static volatile uint32_t s_frames;
static volatile uint32_t s_rx_bytes;

static uint8_t  s_exp_kind;      /* 0=不滤 kind；1=MAT；2=RING */
static uint8_t  s_exp_id;        /* 0=不滤 id */

static char     s_qr[16];
static uint8_t  s_qr_n;
static volatile uint8_t s_qr_ready;

static uint8_t vision_qr_ok(void)
{
    uint8_t i;

    if (s_qr_n != 15u) return 0u;
    if ((s_qr[3] != '+') || (s_qr[7] != '+') || (s_qr[11] != '+')) return 0u;
    for (i = 0u; i < 15u; i++)
    {
        char c, lo, hi;
        if ((i == 3u) || (i == 7u) || (i == 11u)) continue;
        c = s_qr[i];
        if ((i < 3u) || ((i >= 8u) && (i <= 10u))) { lo = '1'; hi = '6'; }
        else { lo = '1'; hi = '3'; }
        if ((c < lo) || (c > hi)) return 0u;
    }
    return 1u;
}

static void vision_rearm(void)
{
    HAL_StatusTypeDef st;
    if (s_uart == 0) return;
    st = HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
    if (st == HAL_OK) return;
    (void)HAL_UART_AbortReceive(s_uart);
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

static uint8_t vision_accept(uint8_t kind, uint8_t id)
{
    if ((s_exp_kind != 0u) && (kind != s_exp_kind)) return 0u;
    if ((s_exp_id != 0u) && (id != s_exp_id)) return 0u;
    return 1u;
}

static void vision_commit(uint8_t kind, int16_t x, int16_t y, uint8_t id)
{
    if (vision_accept(kind, id) == 0u) return;
    s_x = x;
    s_y = y;
    s_id = id;
    s_kind = kind;
    s_has = 1u;
    s_tick = HAL_GetTick();
    s_frames++;
}

static void vision_feed(uint8_t b)
{
    s_rx_bytes++;

    switch (s_state)
    {
    case 0u:
        if (b == 0xB3u) s_state = 1u;
        else if (b == (uint8_t)'$') { s_qr_n = 0u; s_state = 20u; }
        break;

    case 20u:
        if (b == (uint8_t)'#')
        {
            if (vision_qr_ok() != 0u) s_qr_ready = 1u;
            s_state = 0u;
        }
        else if ((s_qr_n < 15u) && (b >= 0x20u) && (b < 0x7Fu))
            s_qr[s_qr_n++] = (char)b;
        else
            s_state = 0u;
        break;

    case 1u:
        if (b == 0xB3u) s_state = 2u;
        else if (b != 0xB3u) s_state = 0u;
        break;

    case 2u:
        if ((b == 0xB1u) || (b == 0xB2u))
        {
            s_type = b;
            s_pay_i = 0u;
            s_state = 3u;
        }
        else if (b == 0xB3u) s_state = 1u;
        else s_state = 0u;
        break;

    case 3u:
        s_pay[s_pay_i++] = b;
        if (s_pay_i >= 7u)
        {
            if (s_pay[6] == 0x5Bu)
            {
                int16_t x = (int16_t)(((uint16_t)s_pay[0] << 8) | s_pay[1]);
                int16_t y = (int16_t)(((uint16_t)s_pay[2] << 8) | s_pay[3]);
                if (s_type == 0xB1u)
                    vision_commit(VISION_KIND_MAT, x, y, s_pay[5]); /* color */
                else
                    vision_commit(VISION_KIND_RING, x, y, s_pay[4]); /* digit */
            }
            s_state = 0u;
        }
        break;

    default:
        s_state = 0u;
        break;
    }
}

void Vision_Init(void)
{
    s_uart = VISION_UART;
    s_state = 0u;
    s_has = 0u;
    s_kind = VISION_KIND_NONE;
    s_frames = 0u;
    s_rx_bytes = 0u;
    s_exp_kind = 0u;
    s_exp_id = 0u;
    s_qr_n = 0u;
    s_qr_ready = 0u;
    if ((s_uart == 0) || (s_uart->Instance != VISION_UART_INSTANCE)) return;
    vision_rearm();
}

void Vision_RxIsr(UART_HandleTypeDef *huart)
{
    if ((s_uart == 0) || (huart->Instance != s_uart->Instance)) return;
    vision_feed(s_rx_byte);
    vision_rearm();
}

void Vision_OnUartError(UART_HandleTypeDef *huart)
{
    if ((s_uart == 0) || (huart->Instance != s_uart->Instance)) return;
    s_state = 0u;
    vision_rearm();
}

void Vision_SetExpect(uint8_t kind, uint8_t expect)
{
    s_exp_kind = kind;
    s_exp_id = expect;
    s_has = 0u;
}

void Vision_ClearExpect(void)
{
    s_exp_kind = 0u;
    s_exp_id = 0u;
}

uint8_t Vision_GetTarget(int16_t *x, int16_t *y, uint8_t *id)
{
    if (!s_has) return 0u;
    if (x) *x = s_x;
    if (y) *y = s_y;
    if (id) *id = s_id;
    return 1u;
}

uint8_t Vision_GetKind(void)
{
    return s_has ? s_kind : VISION_KIND_NONE;
}

uint8_t Vision_IsArrived(void)
{
    int16_t ex, ey;
    if (!s_has) return 0u;
    ex = (int16_t)(s_x - (int16_t)VISION_CAM_CX);
    ey = (int16_t)(s_y - (int16_t)VISION_CAM_CY);
    if (ex < 0) ex = (int16_t)(-ex);
    if (ey < 0) ey = (int16_t)(-ey);
    return (uint8_t)((ex <= (int16_t)VISION_ALIGN_DEAD_PX) &&
                     (ey <= (int16_t)VISION_ALIGN_DEAD_PX));
}

uint8_t Vision_IsTimeout(void) { return 0u; }

uint8_t Vision_IsOnline(void)
{
    if (!s_has) return 0u;
    return (uint8_t)((HAL_GetTick() - s_tick) <= (uint32_t)VISION_OFFLINE_MS);
}

uint32_t Vision_FrameCount(void) { return s_frames; }
uint32_t Vision_RxByteCount(void) { return s_rx_bytes; }

void Vision_SendCmd(uint8_t cmd)
{
    uint8_t frame[3];
    uint8_t i;
    uint32_t guard;

    if (s_uart == 0) return;
    if ((cmd == 0xFFu) || (cmd == 0xFEu)) return;

    frame[0] = 0xFFu;
    frame[1] = cmd;
    frame[2] = 0xFEu;
    for (i = 0u; i < 3u; i++)
    {
        guard = 100000u;
        while (((s_uart->Instance->SR & USART_SR_TXE) == 0u) && (guard != 0u)) guard--;
        if (guard == 0u) return;
        s_uart->Instance->DR = frame[i];
    }
}

uint8_t Vision_CmdForColor(uint8_t color)
{
    /* 与成品/视觉 CMD_COLOR_MAP + HSV 业务色号一致 */
    switch (color)
    {
    case 1u: return 0x03u; /* 红 */
    case 2u: return 0x07u; /* 黄 */
    case 3u: return 0x05u; /* 蓝 */
    case 4u: return 0x04u; /* 绿 */
    case 5u: return 0x06u; /* 黑 */
    case 6u: return 0x08u; /* 浅蓝 */
    default: return VISION_CMD_MAT_ALL;
    }
}

void Vision_BeginStep(void)
{
    s_has = 0u;
}

uint8_t Vision_TakeTask(char *dst, uint8_t max)
{
    uint8_t i, n;
    if (s_qr_ready == 0u) return 0u;
    n = s_qr_n;
    if (dst != 0)
    {
        for (i = 0u; (i < n) && ((uint8_t)(i + 1u) < max); i++) dst[i] = s_qr[i];
        if (max > 0u) dst[(i < max) ? i : (uint8_t)(max - 1u)] = '\0';
    }
    s_qr_ready = 0u;
    return 1u;
}

void Vision_ClearTask(void)
{
    s_qr_ready = 0u;
    s_qr_n = 0u;
}

uint8_t Vision_GetOffset(int16_t *dx, int16_t *dy)
{
    if (!s_has) return 0u;
    if (dx) *dx = (int16_t)(s_x - (int16_t)VISION_CAM_CX);
    if (dy) *dy = (int16_t)(s_y - (int16_t)VISION_CAM_CY);
    return 1u;
}

#if VISION_GIMBAL_FOLLOW
static uint8_t  s_follow_en;
static uint32_t s_follow_tick;
static uint32_t s_follow_last_frames;

void Vision_FollowSetEnable(uint8_t en)
{
    s_follow_en = (en != 0u) ? 1u : 0u;
    s_follow_last_frames = s_frames;
}

uint8_t Vision_FollowIsEnable(void)
{
    return s_follow_en;
}

void Vision_FollowTask(void)
{
    int16_t dx = 0, dy = 0;
    float step, tgt, cur;
    uint32_t now;

    if (s_follow_en == 0u) return;
    if (GM6020_IsOnline() == 0u) return;

    now = HAL_GetTick();
    if ((uint32_t)(now - s_follow_tick) < (uint32_t)VISION_GIMBAL_PERIOD_MS) return;
    s_follow_tick = now;

    /* 无新帧不推，避免停在旧误差上一直拧 */
    if (s_frames == s_follow_last_frames) return;
    s_follow_last_frames = s_frames;

    if (Vision_GetOffset(&dx, &dy) == 0u) return;
    (void)dy; /* 无同步带：垂直靠固定高度，只跟水平 */

    if (Vision_IsArrived() != 0u) return;
    if (dx < 0) { if ((int16_t)(-dx) <= (int16_t)VISION_ALIGN_DEAD_PX) return; }
    else if (dx <= (int16_t)VISION_ALIGN_DEAD_PX) return;

    step = (float)dx * (float)VISION_GIMBAL_KP_DEG_PER_PX * (float)VISION_GIMBAL_DIR;
    if (step > (float)VISION_GIMBAL_MAX_STEP_DEG) step = (float)VISION_GIMBAL_MAX_STEP_DEG;
    if (step < -(float)VISION_GIMBAL_MAX_STEP_DEG) step = -(float)VISION_GIMBAL_MAX_STEP_DEG;
    if ((step > 0.0f) && (step < (float)VISION_GIMBAL_MIN_STEP_DEG))
        step = (float)VISION_GIMBAL_MIN_STEP_DEG;
    if ((step < 0.0f) && (step > -(float)VISION_GIMBAL_MIN_STEP_DEG))
        step = -(float)VISION_GIMBAL_MIN_STEP_DEG;

    cur = GM6020_GetCurrentDeg();
    tgt = cur + step;
    while (tgt < 0.0f) tgt += 360.0f;
    while (tgt >= 360.0f) tgt -= 360.0f;

#if VISION_GIMBAL_CLAMP_SAFE
    if (tgt < (float)ROT_SAFE_MIN_DEG) tgt = (float)ROT_SAFE_MIN_DEG;
    if (tgt > (float)ROT_SAFE_MAX_DEG) tgt = (float)ROT_SAFE_MAX_DEG;
#endif

    /* dir=0 最短路径；单步很小，绕线风险低 */
    GM6020_SetTargetPosDegDir(tgt, 0);
}
#else
void Vision_FollowSetEnable(uint8_t en) { (void)en; }
uint8_t Vision_FollowIsEnable(void) { return 0u; }
void Vision_FollowTask(void) {}
#endif

#else

void     Vision_Init(void) {}
void     Vision_RxIsr(UART_HandleTypeDef *huart) { (void)huart; }
void     Vision_OnUartError(UART_HandleTypeDef *huart) { (void)huart; }
void     Vision_SetExpect(uint8_t k, uint8_t e) { (void)k; (void)e; }
void     Vision_ClearExpect(void) {}
uint8_t  Vision_GetTarget(int16_t *x, int16_t *y, uint8_t *id) { (void)x;(void)y;(void)id; return 0u; }
uint8_t  Vision_GetKind(void) { return 0u; }
uint8_t  Vision_IsArrived(void) { return 0u; }
uint8_t  Vision_IsTimeout(void) { return 0u; }
uint8_t  Vision_IsOnline(void) { return 0u; }
uint32_t Vision_FrameCount(void) { return 0u; }
uint32_t Vision_RxByteCount(void) { return 0u; }
void     Vision_SendCmd(uint8_t cmd) { (void)cmd; }
uint8_t  Vision_CmdForColor(uint8_t c) { (void)c; return 0x01u; }
void     Vision_BeginStep(void) {}
uint8_t  Vision_TakeTask(char *dst, uint8_t max) { (void)dst; (void)max; return 0u; }
void     Vision_ClearTask(void) {}
uint8_t  Vision_GetOffset(int16_t *dx, int16_t *dy) { (void)dx; (void)dy; return 0u; }
void     Vision_FollowSetEnable(uint8_t en) { (void)en; }
uint8_t  Vision_FollowIsEnable(void) { return 0u; }
void     Vision_FollowTask(void) {}

#endif
