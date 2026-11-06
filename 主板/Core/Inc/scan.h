/**
  ******************************************************************************
  * @file    scan.h
  * @brief   扫码相机（UART8 = PE1 TX / PE0 RX）：收 $任务码#，回 $ACK#，持有码
  ******************************************************************************
  */
#ifndef __SCAN_H__
#define __SCAN_H__

#include "main.h"
#include "config.h"

#if SCAN_ENABLE
void     Scan_Init(void);
void     Scan_RxIsr(UART_HandleTypeDef *huart);
void     Scan_OnUartError(UART_HandleTypeDef *huart);
void     Scan_Task(void);                 /* 主循环：发 ACK + 经 TUNE 主动推 CODE */

uint8_t  Scan_HasCode(void);              /* 1 = 已持有有效任务码 */
uint8_t  Scan_CopyCode(char *dst, uint8_t max); /* 拷贝持有码，不消费；无码返回 0 */
void     Scan_Clear(void);
uint32_t Scan_RxByteCount(void);
uint32_t Scan_AcceptCount(void);
#else
static inline void     Scan_Init(void) {}
static inline void     Scan_RxIsr(UART_HandleTypeDef *h) { (void)h; }
static inline void     Scan_OnUartError(UART_HandleTypeDef *h) { (void)h; }
static inline void     Scan_Task(void) {}
static inline uint8_t  Scan_HasCode(void) { return 0u; }
static inline uint8_t  Scan_CopyCode(char *d, uint8_t m) { (void)d; (void)m; return 0u; }
static inline void     Scan_Clear(void) {}
static inline uint32_t Scan_RxByteCount(void) { return 0u; }
static inline uint32_t Scan_AcceptCount(void) { return 0u; }
#endif

#endif /* __SCAN_H__ */
