/**
  ******************************************************************************
  * @file    pd42s1.h
  * @brief   正点原子 PD42S1 闭环步进驱动器（串口 TTL / RS485）驱动封装
  *
  *  协议：帧头 0xC5 + 从机地址 + 功能码 + 指令数据 + 校验和 + 帧尾 0x5C
  *        校验和 = (帧头 + 地址 + 功能码 + 数据区所有字节) & 0xFF
  *        串口默认 115200 8N1，出厂从机地址 1，位置单位 51200 = 一圈
  *  资料：PD42S1步进电机闭环驱动器用户手册_V1.2.pdf
  *
  *  接线（无极 S2 开发板，闭环步进接口 8P：5V/GND/EN/PWM/DIR/T/R）：
  *        T/R 这一对就是串口，板上复用 USART3 = PB10(TX) / PB11(RX)
  *        电机 V+/GND 接 12~32V（推荐 24V，电流 ≥3A）
  *        如果发指令没反应：先把 T/R 两根线对调试一次（TX/RX 交叉）
  *        串口模式下 EN/PWM/DIR 不用接（EN 只对脉冲模式有效）
  *
  *  用法（main.c）：
  *        PD42S1_Init();                       // 绑定串口 + 开接收中断
  *        while (1) {
  *            PD42S1_Update();                 // 主循环解析应答（很轻，随便调）
  *            ...
  *        }
  *        PD42S1_OpenLoopSpeed(1, 0, 100, 120.0f);   // 开环正转 120RPM
  *
  *  ⚠️ 两条指令之间至少间隔 2ms（手册要求），模块内部已经做了保护；
  *     发送是阻塞的（11 字节约 1ms @115200），别放到 1kHz 控制中断里。
  ******************************************************************************
  */

#ifndef __PD42S1_H__
#define __PD42S1_H__

#include "main.h"
#include "config.h"

/* ---------------- 协议常量（改协议才需要动） ---------------- */
#define PD42S1_FRAME_HEAD       0xC5u
#define PD42S1_FRAME_TAIL       0x5Cu
#define PD42S1_ACK_OK           0x01u   /* 应答里的“操作成功”错误码 */

/* 设置 / 读取参数 */
#define PD42S1_FCT_SET_ADDR     0x60u   /* 设置从机地址（多机共总线时必用） */
#define PD42S1_FCT_SET_MODE     0x62u   /* 设置工作模式（只支持 0~7，开环模式不在其中） */
#define PD42S1_FCT_READ_DRIVE   0x32u   /* 读取驱动参数（模式/回响/波特率/DIR/EN/细分） */

/* 运动相关功能码 */
#define PD42S1_FCT_OL_SPEED     0xE0u   /* 开环速度模式 */
#define PD42S1_FCT_OL_POS_ABS   0xE1u   /* 开环绝对位置模式 */
#define PD42S1_FCT_OL_POS_REL   0xE2u   /* 开环相对位置模式 */
#define PD42S1_FCT_OL_PULSE     0xE3u   /* 开环脉冲模式 */
#define PD42S1_FCT_TORQUE       0xF0u   /* 力矩模式（闭环） */
#define PD42S1_FCT_SPEED        0xF1u   /* 速度模式（闭环） */
#define PD42S1_FCT_POS_ABS      0xF2u   /* 绝对位置模式（闭环） */
#define PD42S1_FCT_POS_REL      0xF3u   /* 相对位置模式（闭环） */
#define PD42S1_FCT_ANGLE_ZERO   0xF8u   /* 当前位置清零 */
#define PD42S1_FCT_UNLOCK_STALL 0xF9u   /* 解除堵转 */
#define PD42S1_FCT_ENABLE       0xFAu   /* 电机使能 / 失能 */
#define PD42S1_FCT_CLEAR_STATE  0xFBu   /* 清除状态（堵转/刹车/失能） */
#define PD42S1_FCT_STOP_NOW     0xFCu   /* 立即停止（刹车） */
#define PD42S1_FCT_SET_ECHO     0x6Fu   /* 设置指令回响：Byte1 0=正常回响 1=不回响 */

/* 读取相关功能码 */
#define PD42S1_FCT_READ_VOLT    0x24u   /* 总线电压 */
#define PD42S1_FCT_READ_SPEED   0x29u   /* 实时转速（int16 rpm） */
#define PD42S1_FCT_READ_POS     0x2Au   /* 实时位置（int32，51200=一圈） */
#define PD42S1_FCT_READ_POS_ERR 0x2Bu   /* 位置误差 */
#define PD42S1_FCT_READ_STATE   0x2Cu   /* 电机运行状态 */
#define PD42S1_FCT_READ_ENABLE  0x2Fu   /* 使能状态 */
#define PD42S1_FCT_READ_ARRIVED 0x30u   /* 到位状态 */
#define PD42S1_FCT_READ_SYS     0x31u   /* 系统参数（电压/电流/转速/位置/状态一把抓） */

/* 位置单位：驱动器内部 51200 个脉冲 = 电机一圈 */
#define PD42S1_PULSE_PER_REV    51200L

/* ---------------- 工作模式（0x32 应答 Byte1 / 0x62 可设置的范围） ----------------
   ⚠️ 0x62 只能设置 0~7；8~10（开环模式）只能在驱动器屏幕上选，
      屏幕切到"开环速度"之后 0xE0 才会被执行 */
typedef enum
{
    PD42S1_MODE_POS_COMM    = 0,    /* 通信位置（出厂默认） */
    PD42S1_MODE_SPEED_COMM  = 1,    /* 通信速度 */
    PD42S1_MODE_TORQUE_COMM = 2,    /* 通信力矩 */
    PD42S1_MODE_PULSE       = 3,    /* 脉冲模式 */
    PD42S1_MODE_PW_POS      = 4,    /* 脉宽位置 */
    PD42S1_MODE_PW_SPEED    = 5,    /* 脉宽速度 */
    PD42S1_MODE_PW_TORQUE   = 6,    /* 脉宽力矩 */
    PD42S1_MODE_HOMING      = 7,    /* 回零模式 */
    PD42S1_MODE_OL_SPEED    = 8,    /* 开环速度（只能屏幕选） */
    PD42S1_MODE_OL_POS      = 9,    /* 开环位置（只能屏幕选） */
    PD42S1_MODE_OL_PULSE    = 10    /* 开环脉冲（只能屏幕选） */
} pd42s1_mode_t;

/* ---------------- 电机运行状态（0x2C 应答） ---------------- */
typedef enum
{
    PD42S1_ST_IDLE      = 0,    /* 空闲 */
    PD42S1_ST_DONE      = 1,    /* 已完成 */
    PD42S1_ST_RUNNING   = 2,    /* 正在运行 */
    PD42S1_ST_OVERLOAD  = 3,    /* 过载 */
    PD42S1_ST_STALL     = 4,    /* 堵转 */
    PD42S1_ST_UNDERVOLT = 5     /* 欠压 */
} pd42s1_state_t;

/* ---------------- 反馈数据（主循环 PD42S1_Update() 里刷新） ---------------- */
typedef struct
{
    uint8_t  online;        /* 1 = 最近收到过合法应答（PD42S1_OFFLINE_MS 内有回应） */
    uint8_t  last_ack;      /* 最近一次应答的错误码（PD42S1_ACK_OK = 成功） */
    uint8_t  last_fct;      /* 最近一次应答对应的功能码 */

    uint8_t  enable;        /* 0x2F：0 使能，1 失能 */
    uint8_t  arrived;       /* 0x30：1 到位 */
    uint8_t  state;         /* 0x2C：见 pd42s1_state_t */

    /* ---- 驱动器参数（用 PD42S1_ReadDriveParams() 读一次，用来排查"模式不对/波特率不对" ---- */
    uint8_t  mode;          /* 0x32 Byte1：当前工作模式，见 pd42s1_mode_t（很关键！） */
    uint8_t  mode_valid;    /* 1 = mode 已读到 */
    uint8_t  echo;          /* 0x32 Byte2：1 = 指令不回响，0 = 正常回响 */
    uint32_t uart_baud;     /* 0x32：驱动器当前串口波特率 */
    uint16_t step_div;      /* 0x32：细分 */
    uint8_t  dir_level;     /* 0x32：DIR 有效电平 */
    uint8_t  en_level;      /* 0x32：EN 有效电平 */

    int16_t  rpm;           /* 0x29 实时转速 */
    int32_t  pos;           /* 0x2A 实时位置（51200 = 一圈） */
    int32_t  pos_err;       /* 0x2B 位置误差 */
    float    volt;          /* 0x24 总线电压 */

    uint8_t  rpm_valid;
    uint8_t  pos_valid;
    uint8_t  state_valid;

    uint32_t last_rx_tick;  /* 最近一次收到合法应答的时间戳（内部用） */
    uint32_t rx_ok;         /* 收到并解析成功的帧数 */
    uint32_t rx_err;        /* 校验/格式错误的帧数 */
    uint32_t tx_cnt;        /* 已发出的指令数 */
    uint32_t tx_fail;       /* 发送失败/被 2ms 间隔挡掉的次数（一直涨 = UART 有问题） */
} pd42s1_t;

typedef struct
{
    uint8_t  use;           /* 1 = 参与自检 */
    uint8_t  addr;          /* 从机地址（必须唯一） */
    uint8_t  dir_inv;       /* 1 = 方向取反 */
    float    speed_rpm;     /* 默认转速 */
    uint8_t  acc;           /* 加速度 */
    uint32_t pulses;        /* 位置模式脉冲（51200 = 一圈） */
} stepper_cfg_t;

/* ---------------- 初始化 / 周期任务 ---------------- */

/* 绑定 config.h 里的 STEPPER_UART（默认 &huart3 = 板子闭环步进口的 PB10/PB11） */
void PD42S1_Init(void);

/* 想用别的串口：PD42S1_Init() 之后调这个（例如 PD42S1_InitUart(&huart2)）
   （工程里还没加任何串口时，UART_HandleTypeDef 不存在，这个函数直接不声明） */
#if defined(HAL_UART_MODULE_ENABLED)
void PD42S1_InitUart(UART_HandleTypeDef *huart);

/* ★串口出错后重新挂接收（HAL 在中止接收后不会自己恢复，
   不重新挂该串口就永久收不到数据 → 表现为"驱动器在线但转速/位置全是 0"） */
void PD42S1_RearmRx(void);
#endif

/* 主循环里每轮调用：空闲分帧 + 解析应答 + 在线判断（非阻塞） */
void PD42S1_Update(void);

/* 读反馈（只读指针，别改）。多电机时返回"最近一次通信的那台"；单电机就是它自己 */
const pd42s1_t *PD42S1_Get(void);
uint8_t PD42S1_IsOnline(void);

/* ---------------- 多电机（config.h 里的 STEPPER_MOTOR_TABLE） ----------------
   idx：0 ~ PD42S1_GetMotorNum()-1，0 就是表里第一行（1 号口）
   MotorXxx 系列会自动套用该电机的【从机地址、方向取反、加速度、转速】 */
uint8_t PD42S1_GetMotorNum(void);                          /* 表里几台电机 */
const stepper_cfg_t *PD42S1_GetMotorCfg(uint8_t idx);      /* 该电机配置（0 = 越界） */
const pd42s1_t *PD42S1_GetMotorState(uint8_t idx);         /* 该电机反馈 */
uint8_t PD42S1_GetActiveMotor(void);                       /* 最近通信的是哪台（idx） */

void PD42S1_MotorSpeed(uint8_t idx, uint8_t dir, float rpm);     /* 速度模式：dir 0正1反 */
void PD42S1_MotorPosAbs(uint8_t idx, uint32_t pulses);           /* 绝对位置 */
void PD42S1_MotorPosRel(uint8_t idx, uint32_t pulses);           /* 相对位置 */
void PD42S1_MotorStop(uint8_t idx);                              /* 立即停 */
void PD42S1_MotorEnable(uint8_t idx, uint8_t en);                /* 0 使能 / 1 失能 */

/* 接收一个字节（一般是 HAL_UART_RxCpltCallback 里调，也可以在别处手动喂） */
void PD42S1_RxByte(uint8_t b);

/* ---------------- 运动指令：开环（不需要编码器，最安全的自检） ---------------- */

/* 开环速度模式 0xE0：速度范围 0.1~400 RPM，acc 0~200（0 = 直接启动） */
void PD42S1_OpenLoopSpeed(uint8_t addr, uint8_t dir, uint8_t acc, float rpm);

/* 开环绝对位置模式 0xE1：rpm 0~400，pulses 为绝对位置（51200 = 一圈） */
void PD42S1_OpenLoopPosAbs(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses);

/* 开环相对位置模式 0xE2：走相对当前点 pulses 个脉冲 */
void PD42S1_OpenLoopPosRel(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses);

/* ---------------- 运动指令：闭环（需要驱动器已校准编码器） ---------------- */

void PD42S1_TorqueMode(uint8_t addr, uint8_t dir, uint16_t current_ma);   /* 0xF0：0~3000mA */
void PD42S1_SpeedMode(uint8_t addr, uint8_t dir, uint8_t acc, float rpm); /* 0xF1：0.1~3000RPM */
void PD42S1_PosModeAbs(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses); /* 0xF2 */
void PD42S1_PosModeRel(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t rpm, uint32_t pulses); /* 0xF3 */

/* ---------------- 控制指令 ---------------- */

/* 设置从机地址 0x60：new_addr 1~255
   ⚠️ 总线上不能有两台一样；改完这台就只认新地址了（注意把 config.h 表里的地址也改掉） */
void PD42S1_SetAddress(uint8_t addr, uint8_t new_addr);

/* 设置工作模式 0x62：mode 取 pd42s1_mode_t 的 0~7
   ⚠️ 开环模式（8/9/10）不能用这条指令设，只能屏幕选 */
void PD42S1_SetMode(uint8_t addr, uint8_t mode);

/* 读驱动参数 0x32：结果进 pd42s1_t 的 mode / echo / uart_baud / step_div … 
   排查"到底是模式不对还是波特率不对"时先读这一条 */
void PD42S1_ReadDriveParams(uint8_t addr);

void PD42S1_Enable(uint8_t addr, uint8_t en);   /* 0 = 使能，1 = 失能 */
void PD42S1_StopNow(uint8_t addr);              /* 立即刹车停 */
void PD42S1_ClearState(uint8_t addr);           /* 清堵转/刹车/失能状态 */
void PD42S1_UnlockStall(uint8_t addr);          /* 解除堵转保护 */
void PD42S1_AngleZero(uint8_t addr);            /* 把当前位置清零（绝对位置模式下很有用） */

/* ---------------- 读取指令（应答由 PD42S1_Update() 解析进 pd42s1_t） ---------------- */
void PD42S1_ReadSpeed(uint8_t addr);
void PD42S1_ReadPos(uint8_t addr);
void PD42S1_ReadPosErr(uint8_t addr);
void PD42S1_ReadState(uint8_t addr);
void PD42S1_ReadEnable(uint8_t addr);
void PD42S1_ReadArrived(uint8_t addr);
void PD42S1_ReadVolt(uint8_t addr);
void PD42S1_ReadSysParam(uint8_t addr);

/* ★设置"指令回响"（0x6F）：1 = 让它正常应答；0 = 让它不回答
   为什么需要：驱动器被设成"不回响"时，会干活但一个字节都不回 →
   底盘 4 个轮子的转速反馈永远读不到（表现为 N0/4、rpm 恒 0）。
   底盘初始化后周期性地把 1~4 号都设成"要回响"，可自愈这个设置。 */
void PD42S1_SetEcho(uint8_t addr, uint8_t enable);

/* ---------------- 底层（想自己拼帧时用） ---------------- */

/* 组帧：返回帧长度（5 + dlen），data 可为 0 */
uint8_t PD42S1_BuildFrame(uint8_t addr, uint8_t fct, const uint8_t *data, uint8_t dlen, uint8_t *out);

/* 直接发一帧（带 2ms 指令间隔保护）；返回 1 成功，0 被间隔保护挡掉/发送失败 */
uint8_t PD42S1_SendFrame(const uint8_t *frame, uint8_t len);

#endif /* __PD42S1_H__ */
