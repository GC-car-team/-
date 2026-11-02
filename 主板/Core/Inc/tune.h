/**
  ******************************************************************************
  * @file    tune.h
  * @brief   调参命令通道（PC 自动整定 / 手工命令行调参）
  *
  *  为什么要有它：三个 PID 的参数原来全是编译期宏，每改一个数就要重编译+重烧
  *  （约 1 分钟）。有了这条通道，PC 可以在线上改参数、触发小动作、收遥测，
  *  一轮试验只要几秒 —— 自动整定（继电反馈 / Optuna）才有意义。
  *
  *  走哪个串口由 config.h ⑯ 区的 TUNE_ON_BT 决定：
  *      TUNE_ON_BT = 0 → 串口2 = PA2(TX)/PA3(RX)（有线 USB-TTL，推荐：延迟稳定）
  *      TUNE_ON_BT = 1 → 串口4 = PA0(TX)/PA1(RX)（HC-05 蓝牙，无线；模块建议先 AT+BAUD=8）
  *  有线模式下**必须**把 USB-TTL 的 TX 接到板子 PA3（现在只接了 PA2 单向）。
  *
  *  命令（都是 ASCII 一行，以 \n 或 \r 结尾）：
  *      Z            航向归零（同时测一次漂移率并抵消）
  *      G<mm>        前进/后退 mm（相对当前位置，小动作：建议 |mm| ≤ 500）
  *      S<mm>        左(+)/右(-) 平移 mm
  *      T<deg>       相对当前朝向转 deg 度（正 = 逆时针/左转）
  *      X            停（含 TurnStop/StrafeStop）
  *      P <环> <项> <值>   改参数（三项用空格分开）：
  *                    环：h = 航向稳定  t = 旋转稳定  s = 平移稳定
  *                    项：p = Kp   i = Ki   d = Kd
  *                        db  = 死区(deg/mm)   om = 输出限幅   im = 积分限幅
  *                        arr = 到位阈值(deg/mm)   set = 稳定时间(ms，只旋转环)
  *                    例： "P t p 3.0"   "P t d 0.2"   "P t arr 1.5"
  *                        "P h om 120"  "P s db 2.0"  "P h i 0.02"
  *      Q            打印当前全部参数（一行一份，PC 解析）
  *      M<0|1>       遥测开关：1 = 发文本遥测（同时关掉 VOFA JustFloat），0 = 恢复
  *      R            软复位：航向归零 + 清 PID 状态（不动底盘使能）
  *
  *  遥测（M1 之后，一行一份，TUNE_TELEM_HZ 频率）：
  *      A,<ms>,<state>,<yaw>,<hdg_err>,<hdg_out>,<wz>,<fwd>,<side>,
  *        <turn_err>,<turn_out>,<side_err>,<side_out>,<loop_ms>
  *      state: 0 = 空闲（可以做下一条动作）/ 1 = 正在动作
  ******************************************************************************
  */
#ifndef __TUNE_H__
#define __TUNE_H__

#include "main.h"
#include "config.h"

/* 初始化：绑定串口、启动接收、初始化动作状态机（TUNE_ENABLE=0 时是空函数） */
void TUNE_Init(void);

/* 串口接收中断里调（由 pd42s1.c 的 HAL_UART_RxCpltCallback 分发进来） */
void TUNE_RxIsr(UART_HandleTypeDef *huart);

/* ★只喂字节、不碰串口：串口被别人占着时（例如升降臂滑块解析器同用串口2），
   由 arm.c 的 ARM_VofaRxIsr 把字节转发进来 → 一条线同时支持滑块和调参命令 */
void TUNE_FeedByte(char b);

/* 主循环里调：解析命令 + 推进动作状态机 + 到点发遥测 */
void TUNE_Task(void);

/* 状态查询（给 VOFA/诊断用） */
uint8_t TUNE_IsBusy(void);       /* 1 = 正在跑一条动作 */
float   TUNE_GetYawDeg(void);    /* 当前航向（= 底盘航向） */
uint32_t TUNE_GetCmdCount(void); /* 收到过多少条命令（0 = PC 那边一根线都没接对） */

#endif /* __TUNE_H__ */
