/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘模块（麦克纳姆轮 + 4 台 PD42S1 闭环步进）
  *
  *  这个文件负责：
  *    ① 运动学：整车速度 (vx, vy, wz) → 4 个轮子的转速   【Chassis_Mix()】
  *    ② 下发：限幅 + 变化死区 → 逐台发给步进驱动器
  *    ③ 反馈：读 4 台电机编码器 → 里程计 + 航向估计
  *    ④ 航向保持（可选，配合陀螺仪）
  *    ⑤ 上电自检：走方形动作序列，用来验证运动学/接线对不对
  *
  *  ⚠️ 单位约定（业务代码里只用这三样，不用管 rpm）：
  *        vx, vy : mm/s     （vx>0 前进，vy>0 向左平移）
  *        wz     : 度/s     （>0 逆时针 / 左转）
  *        yaw    : 度       （上电清零，逆时针为正）
  *
  *  典型用法（main.c 里就这么写的）：
  *        Chassis_Init();                       // 上电一次
  *        while (1)
  *        {
  *            Chassis_Task();                   // 主循环里每轮调（内部按节拍干活，非阻塞）
  *            ...
  *        }
  *
  *        Chassis_SetVelocity(300, 0, 0);       // 前进 300mm/s
  *        Chassis_SetVelocity(0, 300, 0);       // 向左平移 300mm/s
  *        Chassis_SetVelocity(0, 0, 90);        // 原地左转 90度/s
  *        Chassis_SetVelocity(300, 200, -45);   // 斜着走 + 右转（可以叠加）
  *        Chassis_Stop();                       // 停车（保持使能，可随时再加速）
  *        Chassis_EmergencyStop();              // 急停（0xFC 刹车，恢复要 Chassis_Resume()）
  ******************************************************************************
  */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h"
#include "config.h"

/* 四个轮子的编号（数组下标用这个，别用 0/1/2/3 裸数字，免得看混） */
#define CHASSIS_WHEEL_FL    0       /* 左前 */
#define CHASSIS_WHEEL_FR    1       /* 右前 */
#define CHASSIS_WHEEL_RL    2       /* 左后 */
#define CHASSIS_WHEEL_RR    3       /* 右后 */
#define CHASSIS_WHEEL_NUM   4

/* ============================ 初始化 / 周期任务 ============================ */

/* 上电初始化：绑定 4 个轮子对应的电机、复位里程计与航向（放在 PD42S1_Init() 之后） */
void Chassis_Init(void);

/* 主循环里每轮调用：内部按节拍下发速度、读反馈、更新里程计（非阻塞） */
void Chassis_Task(void);

/* ============================ 对外控制接口 ============================ */

/* 设置整车速度：vx/vy 单位 mm/s，wz 单位 度/s（>0 左转）。不调它就是停车 */
void Chassis_SetVelocity(float vx_mmps, float vy_mmps, float wz_dps);

/* 停车：4 个轮子速度给到最小合法值（保持使能，不刹车、不发热） */
void Chassis_Stop(void);

/* 急停：发 0xFC 刹车（驱动器会锁在刹车态，且不清理会严重发烫）
   用法：急停之后想再走，必须先调一次 Chassis_Resume() */
void Chassis_EmergencyStop(void);

/* 解除刹车：发 0xFB 清状态（配合 Chassis_EmergencyStop 用） */
void Chassis_Resume(void);

/* 使能/失能 4 台电机：0 使能，1 失能 */
void Chassis_Enable(uint8_t en);

/* ============================ 状态查询 ============================ */

/* 某个轮子的实际转速 rpm（来自驱动器回传，正负表示方向） */
float Chassis_GetWheelRpm(uint8_t wheel);

/* 航向角（度，上电清零；逆时针为正） */
float Chassis_GetYawDeg(void);

/* ★yaw 标度修正系数（0.80~1.25）：IMU 角度与真实角度的比例
   手动转 4 个 90° 看累计报多少（如 350）→ 标度 = 360/350 = 1.0286
   也可用串口命令 K1.0286 在线设置 */
void  Chassis_SetYawScale(float k);
float Chassis_GetDriftDpm(void);   /* 诊断：估计出的零漂率（度/分） */
float Chassis_GetYawScale(void);

/* 里程计估计的整车速度（mm/s）与自转角速度（度/s） */
void  Chassis_GetVelocityEstimate(float *vx_mmps, float *vy_mmps, float *wz_dps);

/* 里程计累计路程（mm）*/
float Chassis_GetDistanceMm(void);

/* 机体系位移（mm，带符号）：forward = 前向积分 ∫vx，side = 横向积分 ∫vy（右为正）
   用途：位置闭环（平移稳定、走到某个点）。注意这是**轮式积分**，打滑会累积误差 */
float Chassis_GetForwardMm(void);
float Chassis_GetSideMm(void);

/* 1 = 4 台电机都在线 */
uint8_t Chassis_IsOnline(void);

/* ★1 = 车正在动（转动/平移/速度指令非 0）*/
uint8_t Chassis_IsMoving(void);

/* ★诊断（屏幕显示用）：online_cnt = 有应答的台数(0~4)；rpm_valid_cnt = 转速字段有效的台数
   online=0 → 串口总线 RX 没通；online>0 但 rpm_valid=0 → 读转速指令/解析有问题 */
void Chassis_GetFeedbackHealth(uint8_t *online_cnt, uint8_t *rpm_valid_cnt);

/* 航向清零（比如手动摆正车之后调一次） */
void Chassis_ResetYaw(void);

/* ============================ 三个闭环 PID（config.h 5.7 区） ============================
 *  ① 航向稳定：CHASSIS_HEADING_HOLD=1 时自动生效（走直/平移时不让车偏头）
 *  ② 旋转稳定：Chassis_TurnStart(角度) → 转到并保持；Chassis_TurnIsDone() 判到位
 *  ③ 平移稳定：Chassis_StrafeStart(vx, 横向位移) → 一边前进一边修到指定位移
 *  三个环的中间量（误差/输出）都能从 Chassis_GetPidDebug() 取出来看 VOFA
 * ============================================================================ */

/* ① 航向稳定 */
void  Chassis_SetHeadingTargetDeg(float deg);   /* 想保持的航向（0 = ResetYaw 的方向） */
float Chassis_GetHeadingTargetDeg(void);
float Chassis_GetHeadingErrDeg(void);

/* ★走直微调角（度）：正 = 稍微朝左走。直行开始时 目标 = 当前朝向 + 这个值 */
void  Chassis_SetStraightTrimDeg(float deg);

/* ★每轮速度系数（0.90~1.10）：走直补偿，不用轮速反馈。idx: 0=左前 1=右前 2=左后 3=右后 */
void  Chassis_SetWheelTrim(uint8_t idx, float k);

/* ★侧滑补偿系数：前进时 vy = vx × k（抵消麦轮横向漂移） */
void  Chassis_SetCrabTrim(float k);
float Chassis_GetCrabTrim(void);
float Chassis_GetWheelTrim(uint8_t idx);
float Chassis_GetStraightTrimDeg(void);           /* 当前航向误差（度） */

/* ② 旋转稳定：转到 deg（相对 ResetYaw 的绝对角度，可给 -180~180 或 0~360）
   用法：Chassis_TurnStart(90.0f); while(!Chassis_TurnIsDone()){...} 或放进状态机里轮询 */
void    Chassis_TurnStart(float deg);
void    Chassis_TurnStop(void);
uint8_t Chassis_TurnIsActive(void);
uint8_t Chassis_TurnIsDone(void);               /* 1 = 到位并稳住 */
float   Chassis_GetTurnErrDeg(void);

/* ③ 平移稳定：以 vx(mm/s) 前进的同时，把横向位移修到 side_mm（右为正）
   用法：Chassis_StrafeStart(300.0f, 400.0f); 一直轮询 Chassis_StrafeIsDone() */
void    Chassis_StrafeStart(float vx_mmps, float side_mm);
void    Chassis_StrafeStop(void);
uint8_t Chassis_StrafeIsActive(void);
uint8_t Chassis_StrafeIsDone(void);
float   Chassis_GetSideErrMm(void);             /* 横向位移误差（mm） */

/* 调参用：把三个环的中间量一次取出来（顺序见实现里的注释），n ≤ 10 */
void    Chassis_GetPidDebug(float *out, uint8_t n);

/* ============ 调参脚本用：在线上改三个环的参数（只改 RAM，不写 flash） ============
 *   loop : 0 = 航向稳定  1 = 旋转稳定  2 = 平移稳定
 *   which: 0 = Kp  1 = Ki  2 = Kd
 *   param: 0 = 死区  1 = 输出限幅  2 = 积分限幅  3 = 到位阈值  4 = 稳定时间(旋转环)
 *   ⚠️ 改 Ki 会顺手把该环的积分清零（避免上一次试验的积分残留搅乱这一次）
 * ============================================================================ */
void  Chassis_PidSetGain(uint8_t loop, uint8_t which, float v);
float Chassis_PidGetGain(uint8_t loop, uint8_t which);
void  Chassis_PidSetParam(uint8_t loop, uint8_t param, float v);
float Chassis_PidGetParam(uint8_t loop, uint8_t param);
float Chassis_PidGetErr(uint8_t loop);      /* 该环最近一次误差 */
float Chassis_PidGetOut(uint8_t loop);      /* 该环最近一次输出 */

/* ============================ 纯计算（方便队友理解/单独测试） ============================ */

/* 逆运动学：整车速度 → 4 个轮子的目标转速(rpm)
   rpm_out 需要 4 个 float 的数组，顺序是 FL/FR/RL/RR */
void Chassis_Mix(float vx_mmps, float vy_mmps, float wz_dps, float *rpm_out);

/* 正运动学：4 个轮子实际转速(rpm) → 整车速度估计 */
void Chassis_Unmix(const float *rpm_in, float *vx_mmps, float *vy_mmps, float *wz_dps);

/* 上电自检任务（CHASSIS_SELFTEST=1 时由 Chassis_Task 自动调用；不需要就关掉） */
void Chassis_TestTask(void);

#endif /* __CHASSIS_H__ */
