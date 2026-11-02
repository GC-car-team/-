/**
  ******************************************************************************
  * @file    arm.h
  * @brief   升降臂（两个直线轴：螺杆 + 齿条）
  *
  *  当前阶段 = 【定标】：只读两个编码器 + 收 VOFA 拖条的值，默认**不给力**
  *    · 齿条（轴②）：失能后能手推 → VOFA ch1 跟着变 → 记下"哪个数对应哪个位置"
  *    · 螺杆（轴①）：自锁手推不动 → 把 ARM_AXIS1_ALLOW_TORQUE 改 1，用拖条让它动
  *
  *  VOFA 通道（VOFA_SOURCE = VOFA_SRC_ARM）：
  *      ch0 = 轴①（螺杆）编码器位置      ch1 = 轴②（齿条）编码器位置
  *      单位是【脉冲】（51200 = 一圈）
  *      ARM_VOFA_SHOW_SLIDER=1 时多发 ch2 = 拖条收到的值（调拖条用）
  *      ARM_PROBE_ENABLE=1 时改发 CAN 嗅探结果（10 路）
  *
  *  拖条怎么配（VOFA+ 的"参数/控件"面板）：让它发**一段文本**即可，格式随便，
  *  只要里面有数字 —— 固件会抓这一行里的第一个数：
  *      `1234`   `p=1234`   `pos:1234.5`   都能用（结尾 CR/LF 随便，没有也行）
  ******************************************************************************
  */
#ifndef __ARM_H__
#define __ARM_H__

#include "main.h"
#include "can.h"
#include "config.h"

#if ARM_ENABLE

/* ---------------- 初始化 / 周期任务 ---------------- */
/* 初始化：走串口时会把"不该出力的轴"失能（定标时手才推得动齿条） */
void ARM_Init(void);

/* 主循环里每轮调：读编码器位置 + 处理拖条目标（很轻） */
void ARM_Task(void);

/* VOFA 拖条发下来的字节（由 pd42s1.c 的串口接收回调按 USART2 转发进来） */
void ARM_RxByte(uint8_t b);

/* VOFA 串口的接收中断：读字节 → 喂解析 → 重新挂接收（pd42s1.c 里按 VOFA_UART 转发） */
void ARM_VofaRxIsr(UART_HandleTypeDef *huart);

/* 重新挂上 VOFA 主口的接收：蓝牙当主口时 BT_Init() 会重配波特率，main 要在其后调一次 */
void ARM_RearmRx(void);

#if ARM_CAN_BAUD_SCAN
/* CAN 速率扫描结果：hits 位掩码（1=1M 收到过帧 / 2=500K / 4=250K）、当前档位（0=1M 1=500K 2=250K） */
uint8_t ARM_GetCanScanHits(void);
uint8_t ARM_GetCanScanIdx(void);
uint8_t ARM_GetCanListenOnly(void);   /* 1 = 当前这一段只收不发（纯监听） */
#endif

/* 诊断：最后收到的扩展帧（驱动器回过什么 ID / DLC / 数据） */
uint32_t ARM_GetLastExtId(void);
uint8_t  ARM_GetLastExtDlc(void);
uint8_t  ARM_GetLastExtByte(uint8_t i);

/* CAN 接收总入口：由 gm6020.c 的接收回调转发（嗅探记录 / CANopen SDO 应答解析都在这分派） */
void ARM_CanRx(const CAN_RxHeaderTypeDef *rx, const uint8_t *data);

/* ---------------- 位置 / 滑块 查询 ---------------- */
float   ARM_GetPosPulses(uint8_t axis);   /* 编码器位置（脉冲，51200 = 一圈） */
float   ARM_GetPosDeg(uint8_t axis);      /* 换成角度（0~360） */
uint8_t ARM_GetPosValid(uint8_t axis);    /* 1 = 已经收到过位置反馈 */

/* 滑块（两个轴各一个）：轴①（螺杆）= 0 发 `A=值`；轴②（齿条）= 1 发 `B=值`；不带字母两个轴都收 */
float    ARM_GetSlider(uint8_t axis);       /* 该轴滑块最近一次发来的值 */
uint16_t ARM_GetSliderCount(uint8_t axis);  /* 该轴收到过多少次（拖动时会涨） */
int32_t  ARM_GetTargetPulses(uint8_t axis); /* 该轴滑块值换算成的目标脉冲 */

/* 该轴实际在用的节点号（走 CAN = 自动学到的 CANopen 节点；走串口 = 从机地址）
   返回 0 = 还没认到（总线上没人回位置）→ 加到 Keil 的 Watch 窗口里看 */
uint8_t  ARM_GetNode(uint8_t axis);

/* 地址扫描结果（ARM_ADDR_SCAN=1 时才有）：bit0 = 地址1 有应答 … bit6 = 地址7 有应答
   ★ch0/ch1 一直是 0 时看这个：返回 0 = 这条总线上没有任何驱动器应答 */
uint16_t ARM_ScanMask(void);

/* ★诊断：CAN 上收到的【扩展帧】累计数
   驱动器应答是扩展帧（0x100x）、GM6020 是标准帧（不计入）→
   一直在涨 = 驱动器在说话；一直 0 = 驱动器根本没答（没上电 / 不在总线上 / 波特率 / 地址）*/
uint16_t ARM_GetCanExtCount(void);

/* 诊断：CAN 收到的【所有帧】总数（含标准帧，如 GM6020 的 0x205）
   涨 = 总线上至少有一个节点在说话；恒 0 = 总线完全没动静 */
uint16_t ARM_GetCanRxTotal(void);

/* 诊断：成功放进发送邮箱的帧数 / 失败数（TX_OK 恒 0 = CAN 根本没在发帧） */
uint16_t ARM_GetCanTxOk(void);

/* ★虚拟位置：累计已下发的脉冲（驱动器不回读时当位置用） */
int32_t ARM_GetVirtPos(uint8_t axis);

/* ★每轴运行速度（rpm，1~300）：I/J/A 命令都用它 */
void  ARM_SetJogRpm(uint8_t axis, float rpm);
float ARM_GetJogRpm(uint8_t axis);
void    ARM_SetVirtPos(uint8_t axis, int32_t v);
uint16_t ARM_GetCanTxFail(void);

/* 诊断：扩展帧滤波器配置是否成功（0 = 失败 → 扩展帧 0x100x 被硬件丢掉） */
uint8_t  ARM_GetCanFilterOk(void);

/* ★诊断：CAN 错误计数器（不需要开嗅探）
   TEC 涨 = 我们发的帧没人 ACK（总线上没人认/没接上）
   REC 涨 = 总线上有人在发但波特率对不上（我们把它当错误帧丢了）→ 多半是对端 500K、我们 1M
   两者都 0 = 总线上除了 GM6020 没有别的节点在发 */
void ARM_GetCanErrors(uint16_t *tec, uint16_t *rec, uint8_t *lec);

/* ★点动测试：让指定轴相对走 N 个脉冲（CAN 走 0xF3；串口那条也发一遍）
   axis: 0 = 轴①螺杆 / 1 = 轴②齿条；pulses 带符号（±500000 内会被限幅）
   用途：验证"驱动器到底收不收我们的帧"——不用知道当前位置、不用绝对零点 */
uint8_t ARM_Jog(uint8_t axis, int32_t pulses);

/* ---------------- CAN 嗅探（ARM_PROBE_ENABLE=1 时才有） ---------------- */
#if ARM_PROBE_ENABLE
void ARM_ProbeInit(void);
void ARM_ProbeRx(const CAN_RxHeaderTypeDef *rx, const uint8_t *data);
void ARM_ProbeTask(void);
void ARM_ProbeClear(void);
uint16_t ARM_ProbeFrameCount(void);
uint8_t  ARM_ProbeDistinct(void);
uint32_t ARM_ProbeId(uint8_t i);
uint8_t  ARM_ProbeIsExt(uint8_t i);
uint8_t  ARM_ProbeDlc(uint8_t i);
uint8_t  ARM_ProbeData(uint8_t i, uint8_t b);
uint16_t ARM_ProbeCount(uint8_t i);
uint8_t  ARM_ProbeOverflow(void);
uint16_t ARM_ProbeSent(void);

/* CAN 错误计数器（嗅探时一起看）：速率不匹配会让 REC/TEC 涨起来
   tec = 发送错误计数, rec = 接收错误计数, lec = 最近错误码(0=没错) */
void ARM_ProbeErrorCounters(uint16_t *tec, uint16_t *rec, uint8_t *lec);
#endif

#endif /* ARM_ENABLE */

#endif /* __ARM_H__ */
