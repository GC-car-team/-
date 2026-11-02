#ifndef __SERIAL2_H
#define __SERIAL2_H

#include <stdio.h>

void Serial2_Init(void);
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);
void Serial2_SendNumber(uint32_t Number, uint8_t Length);
void Serial2_Printf(char *format, ...);

uint8_t Serial2_GetRxFlag(void);
uint8_t Serial2_GetRxData(void);

#endif
