/**
  ******************************************************************************
  * @file    vision.h
  * @brief   MaixCam ↔ 底板（对齐 成品/视觉 锁定契约，非手册 4.0）
  *
  *  相机口：UART7 = PE8(TX)/PE7(RX)，115200 8N1。
  *
  *  视觉→板：
  *    物料 B3 B3 B1 xH xL yH yL r color 5B
  *    圆环 B3 B3 B2 xH xL yH yL digit 00 5B
  *  板→视觉：FF cmd FE
  *    01 物料全色  02 圆环  03~08 单色（见 Vision_CmdForColor）
  *  扫码：$156+123+516+231#
  ******************************************************************************
  */
#ifndef __VISION_H__
#define __VISION_H__

#include "main.h"
#include "config.h"

#define VISION_CMD_MAT_ALL   0x01u
#define VISION_CMD_RING      0x02u
/* 兼容旧名：待机用全色预览，勿再发 0x0F */
#define VISION_CMD_IDLE      VISION_CMD_MAT_ALL

#define VISION_KIND_NONE     0u
#define VISION_KIND_MAT      1u   /* B1 */
#define VISION_KIND_RING     2u   /* B2 */

void     Vision_Init(void);
void     Vision_RxIsr(UART_HandleTypeDef *huart);
void     Vision_OnUartError(UART_HandleTypeDef *huart);

/* 期望过滤：kind=MAT/RING，expect=色号1-6或环号1-3；expect=0 表示该 kind 任意 id */
void     Vision_SetExpect(uint8_t kind, uint8_t expect);
void     Vision_ClearExpect(void);

/* 最近一帧（已通过 expect 过滤）。尚无合法帧返回 0 */
uint8_t  Vision_GetTarget(int16_t *x, int16_t *y, uint8_t *id);
uint8_t  Vision_GetKind(void);           /* VISION_KIND_* */
uint8_t  Vision_IsArrived(void);         /* dx∈[DX_MIN,DX_MAX] 且 dy∈[DY_MIN,DY_MAX] */
uint8_t  Vision_IsTimeout(void);         /* 本步保留：当前契约无超时事件帧，恒 0 */
uint8_t  Vision_IsOnline(void);
uint32_t Vision_FrameCount(void);
uint32_t Vision_RxByteCount(void);

void     Vision_SendCmd(uint8_t cmd);
/* 业务色号 1-6 → 0x03~0x08；非法返回 MAT_ALL */
uint8_t  Vision_CmdForColor(uint8_t color);

void     Vision_BeginStep(void);         /* 清到位，准备等新结果 */
uint8_t  Vision_TakeTask(char *dst, uint8_t max);
void     Vision_ClearTask(void);

/* 兼容旧调用：dx=x-cx, dy=y-cy（未过滤的最近目标；无帧返回 0） */
uint8_t  Vision_GetOffset(int16_t *dx, int16_t *dy);

/* ---- 底盘微调跟瞄（config VISION_CHASSIS_FOLLOW；死区=到位窗）---- */
void     Vision_FollowSetEnable(uint8_t en);   /* 1=底盘按 dx/dy 微调；0=停 */
uint8_t  Vision_FollowIsEnable(void);
void     Vision_FollowTask(void);              /* 主循环：P+Vmin，进到位窗停车 */

#endif /* __VISION_H__ */
