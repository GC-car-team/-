/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ 上位机波形/调试输出（通用封装，谁都能调）
  *
  *  两种协议（同一个串口，别混着发）：
  *    ① JustFloat（默认，看波形用）：N 个 float32 小端 + 固定帧尾 00 00 80 7F
  *    ② FireWater（看文字用）：普通 ASCII 文本，串口助手也能直接看
  *
  *  典型用法（main.c）：
  *      VOFA_Init();                       // 绑定 config.h 里的 VOFA_UART（默认串口2 = PA2）
  *      VOFA_SetSource(&我的数据源函数);     // 注册一个“往 VOFA 里塞数据”的函数
  *      while (1) { VOFA_Task(); }          // 内部按 VOFA_TX_HZ 限频自动发送
  *
  *  数据源函数里这样发（7 个通道示例）：
  *      static void 我的数据源函数(void)
  *      {
  *          VOFA_Send(7, 目标角度, 实际角度, 误差, 输出, 转速, 步进转速, 步进位置);
  *      }
  *  ⚠️ VOFA_Send 是可变参数，所有通道必须传 float（整数写成 1.0f 或 (float)x），
  *     否则参数类型对不上会发出乱码。类型不确定就用 VOFA_SendArray()。
  *
  *  也可以完全不用数据源，自己按需要用：
  *      VOFA_Send(3, a, b, c);             // 立刻发一帧（不经过限频）
  *      float ch[3] = {a, b, c};
  *      VOFA_SendArray(ch, 3);             // 数组版，推荐（类型安全）
  *      VOFA_Printf("rpm=%.1f pos=%ld\n", rpm, pos);   // 文本调试（FireWater）
  *
  *  VOFA+ 设置：数据格式选 JustFloat，波特率与 CubeMX 一致（默认 115200），
  *              采样率填 config.h 里的 VOFA_TX_HZ。
  *
  *  ⚠️ 发送是阻塞的（一帧 32 字节约 2.8ms @115200）。只能在主循环调用，
  *     不要在 1kHz 控制中断里调，否则会拖垮控制环。
  ******************************************************************************
  */

#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "config.h"

/* 一帧最多几个通道（占 RAM：通道数 × 4 字节）
   当前用到 17 个：云台 5 + 步进 6 + 底盘 3 + IMU 3（见《使用说明》七.2 通道表） */
#define VOFA_MAX_CH     20

/* ---------------- 初始化 / 开关 ---------------- */

/* 绑定 config.h 里的 VOFA_UART 并复位状态（未使能或无串口时是空函数） */
void VOFA_Init(void);

/* 想临时换成别的串口：VOFA_SetUart(&huart3);（VOFA_Init 之后调）
   （工程里还没加任何串口时，UART_HandleTypeDef 不存在，这个函数直接不声明） */
#if defined(HAL_UART_MODULE_ENABLED)
void VOFA_SetUart(UART_HandleTypeDef *huart);
#endif

/* ★第二路输出（蓝牙 HC-05，见 config.h ⑮ 区）：
   同一帧数据会【同时】发给主口和这一路，但这一路有自己的发送频率（蓝牙吞吐差，要降速）。
   hz 传 0 = 这一路不限制（不建议）；huart 传 0 或调用 VOFA_SetUart2(0, 0) = 关掉这一路。
   典型：VOFA_SetUart2(BT_UART, VOFA_BT_HZ);  */
#if defined(HAL_UART_MODULE_ENABLED)
void VOFA_SetUart2(UART_HandleTypeDef *huart, uint16_t hz);
#endif

/* 主口（有线）单独开关：0 = 有线那路不发（只走蓝牙时用） */
void VOFA_SetWiredEnable(uint8_t en);

/* ---------------- 诊断（挂到 VOFA 通道上查"主循环被谁拖慢"） ---------------- */
/* 上一帧阻塞发送花了多少毫秒：115200 发 40 字节约 3.5ms；如果是几十毫秒
   → 说明主口波特率被改小了（比如 9600），主循环就被它拖住了 */
float VOFA_GetTxMs(void);

/* 当前主口的波特率 ÷1000（115200 → 115.2）：用来确认"波特率到底是多少" */
float VOFA_GetUartBaudK(void);

/* 1 = 串口已就绪 */
uint8_t VOFA_IsReady(void);

/* 数据流开关（调试时暂停用，不影响电机控制） */
void    VOFA_SetEnable(uint8_t en);
uint8_t VOFA_IsEnable(void);

/* 发送频率（Hz），0 表示不限频；默认取 config.h 的 VOFA_TX_HZ */
void VOFA_SetRate(uint16_t hz);

/* ---------------- 发送 ---------------- */

/* JustFloat：n 个通道，可变参数（全部传 float/double！） */
uint8_t VOFA_Send(uint8_t n, ...);

/* JustFloat：数组版，类型安全，推荐 */
uint8_t VOFA_SendArray(const float *ch, uint8_t n);

/* FireWater 文本输出（自带 "\r\n" 由调用者给，这里只加不加） */
void VOFA_Printf(const char *fmt, ...);

/* Chrome 单浮点通道快速看：VOFA_Send1(v) 等价于 VOFA_Send(1, v) */
uint8_t VOFA_Send1(float v);
uint8_t VOFA_Send2(float v1, float v2);
uint8_t VOFA_Send3(float v1, float v2, float v3);

/* ---------------- 周期任务 ---------------- */

/* 注册“数据源”函数：VOFA_Task() 到点会自动调它，里面用 VOFA_Send 填数据 */
void VOFA_SetSource(void (*fn)(void));

/* 主循环里每轮调用：按 VOFA_TX_HZ 限频执行数据源函数（未注册就什么都不做） */
void VOFA_Task(void);

#endif /* __VOFA_H__ */
