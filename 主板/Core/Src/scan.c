/**
  ******************************************************************************
  * @file    scan.c
  * @brief   扫码：$任务码# @ UART8；校验通过 → 持有 + $ACK# + TUNE 推 CODE
  ******************************************************************************
  */
#include "scan.h"
#include "usart.h"
#include "tune.h"
#include <string.h>

#if SCAN_ENABLE && defined(HAL_UART_MODULE_ENABLED)

static UART_HandleTypeDef *s_uart;
static uint8_t  s_rx_byte;
static uint8_t  s_state;
static char     s_buf[16];
static uint8_t  s_n;

static char     s_code[16];
static volatile uint8_t s_has;
static volatile uint8_t s_need_ack;
static volatile uint8_t s_need_push;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_accepts;

static uint8_t scan_ok(void)
{
    uint8_t i;
    if (s_n != 15u) return 0u;
    if ((s_buf[3] != '+') || (s_buf[7] != '+') || (s_buf[11] != '+')) return 0u;
    for (i = 0u; i < 15u; i++)
    {
        char c, lo, hi;
        if ((i == 3u) || (i == 7u) || (i == 11u)) continue;
        c = s_buf[i];
        if ((i < 3u) || ((i >= 8u) && (i <= 10u))) { lo = '1'; hi = '6'; }
        else { lo = '1'; hi = '3'; }
        if ((c < lo) || (c > hi)) return 0u;
    }
    return 1u;
}

static void scan_rearm(void)
{
    HAL_StatusTypeDef st;
    if (s_uart == 0) return;
    st = HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
    if (st == HAL_OK) return;
    (void)HAL_UART_AbortReceive(s_uart);
    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

static void scan_feed(uint8_t b)
{
    s_rx_bytes++;
    switch (s_state)
    {
    case 0u:
        if (b == (uint8_t)'$') { s_n = 0u; s_state = 1u; }
        break;
    case 1u:
        if (b == (uint8_t)'#')
        {
            if (scan_ok() != 0u)
            {
                uint8_t i;
                for (i = 0u; i < 15u; i++) s_code[i] = s_buf[i];
                s_code[15] = '\0';
                s_has = 1u;
                s_accepts++;
                s_need_ack = 1u;
                s_need_push = 1u;
            }
            s_state = 0u;
        }
        else if ((s_n < 15u) && (b >= 0x20u) && (b < 0x7Fu))
            s_buf[s_n++] = (char)b;
        else
            s_state = 0u;
        break;
    default:
        s_state = 0u;
        break;
    }
}

void Scan_Init(void)
{
    s_uart = SCAN_UART;
    s_state = 0u;
    s_n = 0u;
    s_has = 0u;
    s_need_ack = 0u;
    s_need_push = 0u;
    s_rx_bytes = 0u;
    s_accepts = 0u;
    s_code[0] = '\0';
    if ((s_uart == 0) || (s_uart->Instance != SCAN_UART_INSTANCE)) return;
    scan_rearm();
}

void Scan_RxIsr(UART_HandleTypeDef *huart)
{
    if ((s_uart == 0) || (huart->Instance != s_uart->Instance)) return;
    scan_feed(s_rx_byte);
    scan_rearm();
}

void Scan_OnUartError(UART_HandleTypeDef *huart)
{
    if ((s_uart == 0) || (huart->Instance != s_uart->Instance)) return;
    s_state = 0u;
    scan_rearm();
}

void Scan_Task(void)
{
    if (s_need_ack != 0u)
    {
        static const char ack[] = "$ACK#";
        s_need_ack = 0u;
        if (s_uart != 0)
            (void)HAL_UART_Transmit(s_uart, (uint8_t *)ack, 5u, 20u);
    }
    if (s_need_push != 0u)
    {
        char line[28];
        uint8_t i;
        s_need_push = 0u;
        if (s_has == 0u) return;
        line[0] = 'C'; line[1] = 'O'; line[2] = 'D'; line[3] = 'E'; line[4] = ',';
        for (i = 0u; i < 15u; i++) line[5u + i] = s_code[i];
        line[20] = '\r'; line[21] = '\n'; line[22] = '\0';
        TUNE_SendLine(line);
    }
}

uint8_t Scan_HasCode(void)
{
    return s_has;
}

uint8_t Scan_CopyCode(char *dst, uint8_t max)
{
    uint8_t i;
    if ((s_has == 0u) || (dst == 0) || (max == 0u)) return 0u;
    for (i = 0u; (i < 15u) && ((uint8_t)(i + 1u) < max); i++) dst[i] = s_code[i];
    if (max > 0u) dst[(i < max) ? i : (uint8_t)(max - 1u)] = '\0';
    return 1u;
}

void Scan_Clear(void)
{
    s_has = 0u;
    s_code[0] = '\0';
    s_need_ack = 0u;
    s_need_push = 0u;
}

uint32_t Scan_RxByteCount(void) { return s_rx_bytes; }
uint32_t Scan_AcceptCount(void) { return s_accepts; }

#endif /* SCAN_ENABLE */
