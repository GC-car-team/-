/**
  ******************************************************************************
  * @file    gm6020.h
  * @brief   GM6020（云台偏航）CAN 驱动 + 位置环/速度环控制
  *          控制周期 1kHz（由 TIM6 中断调用 GM6020_Loop）
  *
  *  硬件连接：GM6020 --CAN_H/CAN_L--> 主控 CAN1（PA11=RX / PA12=TX）
  *            供电 24V（6S 电池）；CAN 总线两端各 120Ω 终端电阻
  ******************************************************************************
  */

#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"
#include "can.h"
#include "config.h"

/* ============================ CAN 报文 ID ============================ */
#define GM6020_FEEDBACK_ID_BASE     0x205   /* 反馈 ID = 0x205 + 电机ID(1~7) */
#define GM6020_CTRL_ID_LOW          0x1FF   /* 控制电机 ID 1~4 */
#define GM6020_CTRL_ID_HIGH         0x2FF   /* 控制电机 ID 5~7 */
#define GM6020_CURRENT_FULLSCALE    30000   /* 协议电流满量程 */

/* ============================ 电机反馈 ============================ */
typedef struct
{
    uint16_t ecd;           /* 机械角度，0~8191（0~360°） */
    int16_t  speed_rpm;     /* 转速 rpm */
    int16_t  current;       /* 实际转矩电流 */
    uint8_t  temp;          /* 温度 ℃ */
    uint16_t last_ecd;      /* 上一次角度（算多圈用） */
    int32_t  total_ecd;     /* 累计角度（多圈，仅用于显示/记录） */
    uint32_t last_tick;     /* 最近一次收到反馈的时间戳 */
    uint8_t  online;        /* 1 = 反馈正常 */
    uint16_t lost_cnt;      /* 累计丢反馈次数 */
} gm6020_measure_t;

/* ============================ PID ============================ */
typedef struct
{
    float kp, ki, kd;
    float error;            /* 本次误差 */
    float last_error;       /* 上次误差 */
    float integral;         /* 积分累加（已限幅） */
    float d_state;          /* 微分低通状态 */
    float ff;               /* 本次前馈输出 */
    float output;           /* 本次输出 */
} gm6020_pid_t;

/* ============================ 运行模式 ============================ */
#define GM6020_MODE_POSITION    0
#define GM6020_MODE_SPEED       1

/* ============================ 接口 ============================ */
/* 初始化（CAN 接口用）：CAN 滤波器 + 启动 + 打开 RX 中断。放在 MX_CAN1_Init() 之后调用 */
void GM6020_Init(void);

/* 初始化（PWM 舵机模式用）：绑定 PWM 定时器 + 启动 PWM 输出 + 先摆到 0°
   例：GM6020_PwmInit(&htim1, TIM_CHANNEL_1);
   定时器必须先配成 50Hz、1µs 分辨率（计数频率 1MHz，ARR = 20000-1） */
void GM6020_PwmInit(TIM_HandleTypeDef *htim, uint32_t channel);

/* 1kHz 控制环：在 TIM6 中断（HAL_TIM_PeriodElapsedCallback）里调用 */
void GM6020_Loop(void);

/* 模式：默认由 config.h 决定，运行中也可以切 */
void    GM6020_SetMode(uint8_t mode);
uint8_t GM6020_GetMode(void);

/* 设定目标 */
void GM6020_SetTargetPos(uint16_t ecd);     /* 目标角度：0~8191 */
void GM6020_SetTargetPosDeg(float deg);

/* ★带方向的转到某角度（绕线保护）：dir 0=最短路径 +1=角度递增 -1=角度递减 */
void  GM6020_SetTargetPosDegDir(float deg, int8_t dir);
int8_t GM6020_GetSlewDir(void);     /* 目标角度：0~360° */
void GM6020_SetTargetSpeed(int16_t rpm);    /* 目标转速 */

/* 立即停机（输出归零、积分清零） */
void GM6020_Stop(void);

/* 状态查询 */
uint8_t GM6020_IsOnline(void);              /* 1 = 反馈正常 */
uint8_t GM6020_IsArrived(void);             /* 1 = 位置环已到位 */
float   GM6020_GetCurrentDeg(void);         /* 当前角度（°，0~360） */
float   GM6020_GetTargetDeg(void);          /* 目标角度（°，0~360） */
int16_t GM6020_GetCurrentOutput(void);      /* 当前输出电流指令 */
const gm6020_measure_t *GM6020_GetMeasure(void);

/* ★诊断：最后收到的 6020 反馈 ID 和总帧数（0 = 一帧都没收到） */
uint32_t GM6020_GetLastRxId(void);
uint16_t GM6020_GetRxCount(void);
uint16_t GM6020_GetTxOkCount(void);
uint16_t GM6020_GetTxFailCount(void);
uint16_t GM6020_GetLoopCount(void);
uint8_t  GM6020_GetCanStartOk(void);

/* 发送四路控制电流（0x1FF/0x2FF）。一般不用直接调，Loop 里已处理 */
void GM6020_SendCurrent(int16_t i1, int16_t i2, int16_t i3, int16_t i4);

#endif /* __GM6020_H__ */
