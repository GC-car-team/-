#ifndef __TCRT5000_H
#define __TCRT5000_H

void TCRT_Init(void);
uint8_t TCRT_Get(void);			//返回1表示检测到物体，0表示未检测到
uint16_t TCRT_GetValue(void);	//返回ADC原始值，0~4095，数值越大反射越强

#endif
