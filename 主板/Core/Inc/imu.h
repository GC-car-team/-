/**
  ******************************************************************************
  * @file    imu.h
  * @brief   IMU 抽象层 —— 对接【YbImu】模块（默认走串口 UART4 = PC10/PC11；
  *          也保留 I²C 0x23 那条路：config.h 里 IMU_IF_UART 改 0 即可切）
  *
  *  这个文件负责：
  *    ① 航向角（度）—— 直接用模块内部融合好的欧拉角 yaw，不是我们积分陀螺
  *    ② 陀螺 Z 角速度（度/s）—— 给 VOFA 看 / 备用
  *    ③ 在线判断、航向清零（软件零点）
  *
  *  注意：**不是 MPU6050**！YbImu 两种接法：
  *     · 串口（默认）：UART4 115200，帧 = 7E 23 LEN FUNC <DATA> SUM8
  *     · I²C：地址 0x23，寄存器是"功能码读多字节"
  *  功能码两种接法一样：
  *       0x01 版本(3B)  0x04 原始数据(18B,int16 小端)  0x26 欧拉角(12B,3×float32 弧度)
  *       0x70 校准IMU(写1)  0x61 融合算法(写6或9)
  *  没接模块 / IMU_ENABLE=0 时，本文件全是零值桩，底盘照样跑。
  ******************************************************************************
  */
#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"
#include "config.h"

/* 初始化（默认 UART4；IMU_IF_UART=0 时是 I²C1）。没接模块就是空实现 */
void IMU_Init(void);

/* 串口接收中断回调：由 pd42s1.c 的 HAL_UART_RxCpltCallback 按串口号转发进来
   （IMU_IF_UART=0 时是空函数；只做解析出帧 + 重新挂接收，别的都不干） */
void IMU_RxIsr(UART_HandleTypeDef *huart);

/* 主循环里每轮调：读欧拉角 + 陀螺 Z */
void IMU_Update(void);

/* 1 = 这次读到了数据（在线） */
uint8_t IMU_IsOnline(void);

/* 陀螺 Z 角速度（度/s，已按 IMU_GYRO_Z_SIGN 修正方向） */
float IMU_GetGyroZDps(void);

/* 航向角（度，已减去 ResetYaw 的零点；逆时针为正，范围 ±180） */
float IMU_GetYawDeg(void);

/* 横滚 / 俯仰（度）—— 顺便读回来的，给调平/诊断用 */
float IMU_GetRollDeg(void);
float IMU_GetPitchDeg(void);

/* 航向清零：把当前航向设为 0°（软件零点，不动模块内部数据） */
void IMU_ResetYaw(void);

/* 模块版本号（主版本，读 0x01 的第一个字节；0 = 没读到） */
uint8_t IMU_GetVersion(void);

/* 状态：0 = 正常；1 = 这次读失败（值保留上一次）；2 = 连续失败到阈值（已尝试重初始化 I²C）
   用它区分"通信断了"和"模块输出 0" —— 波形掉到 0 时看这一路就知道 */
uint8_t IMU_GetStatus(void);

/* 通信诊断（排查用）：
   串口接法：GetAddr 固定返回 IMU_ADDR_7BIT；GetScanResult 返回【最近收到的功能码】
             （0 = 一帧都没收到；0x26 = 38 = 姿态帧正常；0x01 = 3 = 只收到版本帧）
   I²C 接法：GetAddr = 实际用的 7 位地址；GetScanResult = 上电扫描结果（0 = 总线上没东西） */
uint8_t IMU_GetAddr(void);
uint8_t IMU_GetScanResult(void);

/* 校准回执状态：非 0 = 这次上电的 IMU_Calibrate() 拿到了模块回执（校准生效，结果存模块 flash）
   ⚠️ 上电自动校准（IMU_CALIB_ON_BOOT=1）时 VOFA 是看不到 129 那帧的 ——
      校准发生在主循环之前，等 VOFA 开始发数据时功能码早就变回 0x26 了，所以用这个看 */
uint8_t IMU_GetCalibState(void);

/* ★带【漂移抵消】的航向归零（六轴 yaw 会漂，模块自带校准又可能不生效时的兜底）：
   静止采样 ms 毫秒 → 测出漂移率 → 之后 IMU_GetYawDeg() 按时间线性扣掉它。
   调用点必须是"车已停稳"（开机、每次按测试模式、脚本每条动作前）。
   测出来的速率超过 IMU_TRIM_MAX_DPS 就当"车在动"，按 0 处理（不会越修越坏）。
   ms = 0 等价于普通 IMU_ResetYaw()。返回 1 = 测到并应用了。 */
uint8_t IMU_TrimYaw(uint16_t ms);

/* 上一次测出来的漂移率（度/分）：正负号同 yaw。×1 就是"每分钟漂多少度"
   用它判断"漂移到底修掉没有"：修之前比如 60，修之后 ch0 应该基本不动 */
float   IMU_GetYawTrimDpm(void);

/* 串口接法专用计数：收到多少【校验通过】的帧 / 多少【校验失败】的帧
   （失败数一直涨 = 波特率不对或线太长；通过数不涨 = 模块没在发） */
uint8_t  IMU_GetFrames(void);
uint32_t IMU_GetFrames32(void);
uint8_t IMU_GetBadFrames(void);

/* 校准陀螺仪+加速度计：**车静止、水平放置时**调用；阻塞最多 IMU_CALIB_WAIT_MS
   结果存在模块自己的 flash 里，正常只需做一次（换车/受撞击后再做）
   返回 1 = 校准完成，0 = 失败/超时 */
uint8_t IMU_Calibrate(void);

/* 融合算法：6 = 六轴（yaw 相对上电，车上推荐）/ 9 = 九轴（用磁力计，怕电机磁场）
   结果存模块 flash，只需设一次 */
uint8_t IMU_SetAlgoType(uint8_t algo6_or_9);

/* 设置模块【主动上报】频率（10~100Hz，存模块里）
   ⚠️ 只有把 config.h 的 IMU_USE_ACTIVE_REPORT 改成 1（不用轮询）时才需要调；
      默认轮询模式下不用管。返回 1 = 命令已发出 */
uint8_t IMU_SetReportRate(uint8_t hz);

#endif /* __IMU_H__ */
