/**
  ******************************************************************************
  * @file    hmi.h
  * @brief   触摸屏菜单（陶晶驰 TJC / USART HMI 系）—— 只做"收按钮 → 请求模式"
  *
  *  接线：屏幕 TX → 板子 PD2 = UART5_RX；屏幕 RX → 板子 PC12 = UART5_TX；GND 共地
  *        （UART5 要在 CubeMX 里打开：115200 8N1 + 开中断）
  *
  *  屏幕端每个按钮的"触摸弹起事件"发 1 个字节（见 hmi.c 顶部的命令表）：
 *        确认 = 帧 AA 5A + 动作字母 + FE
 *        出发 = 0x10    取消 = 0x00
 *        平移 = 0x02    转弯 = 0x03    里程 = 0x04
 *  动作字母：F 前进  L 左转  R 右转  U 掉头  C 扫码  M 原料  P 放下  G 抓回  B 暂存
 *  也接受 ASCII 字符 '2'~'4' / '0'/'s'，所以用串口助手也能切调试模式、取消路线。
  ******************************************************************************
  */
#ifndef __HMI_H__
#define __HMI_H__

#include "main.h"
#include "config.h"

/* 初始化：绑定串口并启动接收（HMI_ENABLE=0 时是空函数） */
void HMI_Init(UART_HandleTypeDef *huart);

/* 串口接收中断里调：取走这一字节并重新挂上接收（HAL 回调里用这个） */
void HMI_RxIsr(UART_HandleTypeDef *huart);

/* ★给别的串口用：把一字节直接喂进命令表（不回挂接收、也不动那个串口）
   蓝牙（bt.c）收到手机的 '0'~'4' 就是走这个入口，和触摸屏完全同一套命令 */
void HMI_FeedByte(uint8_t b);

/* 主循环里调：取走一次"请求的模式"（0 = 没有请求，取走后清零） */
uint8_t HMI_TakeRequest(void);

/* 取走一次出发（启动键 0x10）。1 = 这次要开始比赛流程 */
uint8_t HMI_TakeStart(void);

/* 按当前出发点和障碍重算比赛路线，写入内部缓冲。返回字母数，0 = 没有可走的路。 */
uint8_t HMI_LoadPlannedPath(void);

/* 取走一次取消（取消键 0x00）。路线已在收到该字节时清掉 */
uint8_t HMI_TakeStop(void);

/* 已确认的路线。字母含义见文件头。取消后长度为 0 */
#define HMI_PATH_MAX 96u
uint8_t HMI_PathLength(void);
uint8_t HMI_PathAt(uint8_t index);

/* 把已确认的路线抄出来。帧正在被改写时返回 0，调用方这次不要发车 */
uint8_t HMI_CopyPath(uint8_t *dst, uint8_t max);

/* 往屏幕回一行状态（显示在文本组件 t0 上；HMI_ENABLE=0 或没接屏幕时什么都不做） */
void HMI_ReportMode(uint8_t mode);

/* 往 t0 写一行原文（ASCII）。云台诊断用。 */
void HMI_ReportLine(const char *text);

/* 往指定文本框写 ASCII，例如 HMI_ShowBox("plan.txt", "156+123+516+231") */
void HMI_ShowBox(const char *box, const char *text);

/* ★调试：读出发点与障碍掩码 */
void HMI_GetObs(uint8_t *start_up, uint16_t *obs_n, uint16_t *obs_e);

/* 切到页面。page 1 用来全屏显示任务码。该页初始化里不能再发字节。 */
void HMI_ShowPage(uint8_t page);

/* 主循环里每次都调：接收若已停，重新挂上。 */
void HMI_Poll(void);

/* 主循环里调。屏幕改了出发点或障碍时，算路、涂色，并保存同一条字母。 */
void HMI_PlanTask(void);


/* ★调试用（HMI_DEBUG_LINE 打开时才用）：往任意文本框写一行 ASCII */
void HMI_SetText(const char *s);
void HMI_SetTextObj(const char *obj, const char *s);
#endif /* __HMI_H__ */
