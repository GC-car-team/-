/**
  ******************************************************************************
  * @file    bt.h
  * @brief   HC-05 蓝牙串口（无线 VOFA / 手机发命令）—— 移植自电脑鼠工程 bsp/hc05.c
  *
  *  为什么是"两个模式"：本质就是【同一个 MCU 的串口，插不同模块】。
  *     模式1 有线：串口2（PA2/PA3）接 USB-TTL → 电脑 VOFA+ 看波形（照旧）
  *     模式2 蓝牙：本模块占串口4（PA0/PA1）接 HC-05 → 电脑/手机蓝牙看波形
  *   两个口可以同时用（config.h 里 VOFA_BT_EN 和有线那路各自独立开关），
  *   也可以把有线关掉只留蓝牙 —— 见 config.h ⑮ 区。
  *
  *  接线（板子 → HC-05）：
  *     PA0  (UART4_TX) → 模块 RXD      （交叉：板子发 → 模块收）
  *     PA1  (UART4_RX) ← 模块 TXD      （想用手机发命令才必须接）
  *     5V              → 模块 VCC      （实惠版板载 LDO；别接 12/24V）
  *     GND             → 模块 GND      （必须共地）
  *     EN/KEY 悬空（只有进 AT 模式才接 3V3）；STATE 不接
  *
  *  ⚠️ UART4 原来是老 IMU 走的 PC10/PC11，现在按"板上串口4 排针 = PA0/PA1"重映射到 PA0/PA1。
  *  ⚠️ 波特率必须和模块一致：HC-05 出厂 9600，推荐先用 USB-TTL 发 AT+BAUD=8 改成 115200
  *     （改不了就把 config.h 的 BT_BAUD 设成模块实际波特率，本模块启动时会按它重配 UART4）
  *  ⚠️ 蓝牙 SPP 的实际吞吐比 USB-TTL 差不少：蓝牙档的 VOFA 发送频率要单独降
  *     （config.h 的 VOFA_BT_HZ，建议 20~30Hz）
  ******************************************************************************
  */
#ifndef __BT_H__
#define __BT_H__

#include "main.h"
#include "config.h"

/* 初始化：按 BT_BAUD 重配 UART4 波特率 + 启动中断接收（BT_ENABLE=0 时全是空函数） */
void BT_Init(void);

/* 当前蓝牙实际在用的波特率（BT_AT_UPGRADE 成功 → BT_BAUD_HI；否则 BT_BAUD；0 = 未初始化） */
uint32_t BT_GetLinkBaud(void);

#if BT_LINK_SCAN
/* 链路找波特率：每 1.5s 轮一档，每 120ms 发一行 "PING <波特率>"（查完把 BT_LINK_SCAN 改 0） */
void BT_LinkScanTask(void);
#endif

/* 串口接收中断里调（由 pd42s1.c 的 HAL_UART_RxCpltCallback 分发进来） */
void BT_RxIsr(UART_HandleTypeDef *huart);

/* 主循环里调（BT_ENABLE=0 时不用调，空实现） */
void BT_Task(void);

/* 发送（阻塞） */
void BT_Send(const uint8_t *buf, uint16_t len);
void BT_SendText(const char *str);
void BT_Printf(const char *fmt, ...);

/* 接收侧状态：有没有数据 / 读一行 / 统计（判"手机到底发过来没有"） */
uint8_t  BT_DataAvailable(void);
uint8_t  BT_ReadByte(void);
uint16_t BT_ReadLine(char *buf, uint16_t size);
void     BT_FlushRx(void);

uint8_t  BT_IsReady(void);       /* 1 = 串口已挂上接收 */
uint32_t BT_GetRxBytes(void);    /* 累计收到多少字节（0 = 没连上/对面没发） */
uint32_t BT_GetRxDrop(void);     /* 环形缓冲满被丢掉的字节数 */
uint32_t BT_GetLines(void);      /* 收到过几行（\n 结尾算一行） */
uint32_t BT_GetCmds(void);       /* 其中被认成模式命令的有几个 */

#endif /* __BT_H__ */
