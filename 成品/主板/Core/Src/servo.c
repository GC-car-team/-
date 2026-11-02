/**
  ******************************************************************************
  * @file    servo.c
  * @brief   ZP 总线舵机夹爪：#000PxxxxTtttt! 非阻塞等待
  ******************************************************************************
  */
#include "servo.h"

#if SERVO_ENABLE

#include "usart.h"
#include <stdio.h>

static UART_HandleTypeDef *s_uart;
static uint16_t s_pos;
static uint8_t  s_busy;
static uint32_t s_busy_until;

static void bus_tx(const uint8_t *data, uint16_t n)
{
    uint16_t i;
    uint32_t guard;

    if ((s_uart == 0) || (data == 0) || (n == 0u)) return;
    for (i = 0u; i < n; i++)
    {
        guard = 100000u;
        while (((s_uart->Instance->SR & USART_SR_TXE) == 0u) && (guard != 0u)) guard--;
        if (guard == 0u) return;
        s_uart->Instance->DR = data[i];
    }
}

static void bus_goto(uint16_t pos, uint16_t time_ms)
{
    char cmd[20];
    int n;
    uint16_t t = time_ms;
    uint16_t id = (uint16_t)BUS_SERVO_ID;

    if (pos < (uint16_t)BUS_SERVO_P_MIN) pos = (uint16_t)BUS_SERVO_P_MIN;
    if (pos > (uint16_t)BUS_SERVO_P_MAX) pos = (uint16_t)BUS_SERVO_P_MAX;
    if (t < 100u) t = 100u;
    if (t > 9999u) t = 9999u;

    n = sprintf(cmd, "#%03uP%04uT%04u!", (unsigned)id, (unsigned)pos, (unsigned)t);
    if (n > 0) bus_tx((const uint8_t *)cmd, (uint16_t)n);

    s_pos = pos;
    s_busy = 1u;
    s_busy_until = HAL_GetTick() + (uint32_t)t + (uint32_t)BUS_SERVO_SETTLE_EXTRA_MS;
}

void SERVO_Init(void)
{
    s_uart = BUS_SERVO_UART;
    s_pos = (uint16_t)BUS_SERVO_OPEN_P;
    s_busy = 0u;
    s_busy_until = 0u;
    if (s_uart != 0)
        bus_goto((uint16_t)BUS_SERVO_OPEN_P, (uint16_t)BUS_SERVO_MOVE_MS);
}

void SERVO_Task(void)
{
    if ((s_busy != 0u) && ((int32_t)(HAL_GetTick() - s_busy_until) >= 0))
        s_busy = 0u;
}

void SERVO_Gripper(uint8_t open)
{
#if BUS_SERVO_INVERT
    open = (uint8_t)(open ? 0u : 1u);
#endif
    bus_goto(open ? (uint16_t)BUS_SERVO_OPEN_P : (uint16_t)BUS_SERVO_CLOSE_P,
             (uint16_t)BUS_SERVO_MOVE_MS);
}

void SERVO_SetPos(uint16_t pos, uint16_t time_ms)
{
    bus_goto(pos, time_ms);
}

uint16_t SERVO_GetPos(void) { return s_pos; }
uint8_t  SERVO_IsBusy(void) { return s_busy; }

void SERVO_SetDeg(float deg)
{
    float p;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    p = (float)BUS_SERVO_P_MIN +
        ((float)BUS_SERVO_P_MAX - (float)BUS_SERVO_P_MIN) * deg / 180.0f;
    SERVO_SetPos((uint16_t)(p + 0.5f), (uint16_t)BUS_SERVO_MOVE_MS);
}

#else

void     SERVO_Init(void) {}
void     SERVO_Task(void) {}
void     SERVO_Gripper(uint8_t open) { (void)open; }
void     SERVO_SetPos(uint16_t p, uint16_t t) { (void)p; (void)t; }
uint16_t SERVO_GetPos(void) { return 0u; }
uint8_t  SERVO_IsBusy(void) { return 0u; }
void     SERVO_SetDeg(float deg) { (void)deg; }

#endif
