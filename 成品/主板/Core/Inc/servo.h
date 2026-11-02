/**
  ******************************************************************************
  * @file    servo.h
  * @brief   夹爪 = ZP 总线舵机（#IDPxxxxTtttt!），非 PWM / 非 MG370
  *
  *  UART：config.h BUS_SERVO_UART（暂 USART2，脚位 TBD）
  ******************************************************************************
  */
#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "config.h"

void SERVO_Init(void);
void SERVO_Task(void);

/* 开合：1=张开 0=夹紧 */
void SERVO_Gripper(uint8_t open);

/* 直接位置（ZP 的 P，一般 500~2500） */
void SERVO_SetPos(uint16_t pos, uint16_t time_ms);
uint16_t SERVO_GetPos(void);
uint8_t  SERVO_IsBusy(void);

/* 0~180° 线性映射到 P_MIN~P_MAX（仅调试） */
void SERVO_SetDeg(float deg);

#endif /* __SERVO_H__ */
