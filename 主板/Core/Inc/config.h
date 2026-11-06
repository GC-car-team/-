/**
  ******************************************************************************
  * @file    config.h
  * @brief   GC_Car 全部编译期开关与参数（改这里就够了，不用动 .c）
  *
  *  目录
  *    ① 板子 / 工程
  *    ② 时间基准（1kHz 控制环）
  *    ③ 云台 GM6020      3.1 模式  3.2 接口  3.3 PWM  3.4 位置环  3.5 速度环  3.6 保护
  *    ④ 步进 PD42S1      4.1 串口  4.2 ★电机表（每台电机的设置）  4.3 上电自检
  *    ⑤ 底盘（麦轮）     5.1 机械参数  5.2 轮子↔电机  5.3 限幅下发
  *                       5.4 自检  5.5 里程计/航向  5.6 IMU
  *    ⑥ VOFA+ 上位机波形
  *    ⑦ 快速检查清单      ⑧ 里程测试（前进 N 米 → 停 → 后退 N 米）
  *
  *  风格：一个功能块里选中一个 #define，其余用 //#define 注释掉
  *        （选错组合会直接 #error 报错，不会出现"模式没定义"的玄学问题）
  ******************************************************************************
  */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/* ==========================================================================
 * ① 板子 / 工程
 * ========================================================================== */
#define BOARD_MAIN_BOARD                    /* 工创小车主控：无极 S2 / YDI-D31，STM32F429VET6 */

/* 引脚速查（硬件固定，不用配，写在这里方便对照）
 *   CAN1      PA11(RX) PA12(TX)                  —— GM6020 总线
 *   云台 PWM  PE9  = TIM1_CH1                     —— GM6020 舵机模式
 *   串口2     PA2(TX) PA3(RX)  = USART2          —— VOFA+ 上位机看波形
 *   步进口T/R PB10(TX) PB11(RX)= USART3          —— 7 路步进口共用这条总线
 *   视觉      PE8(TX) PE7(RX)  = UART7           —— MaixCam 物料/圆环 115200
 *   扫码      PE1(TX) PE0(RX)  = UART8           —— 扫码相机 115200
 *   舵机      PC8 PC9 PE5 PE6 PB14 PB15          —— 6 路硬件 PWM
 */

/* ==========================================================================
 * ③ 云台 GM6020（偏航 / 底座回转，目前只有这一台）
 * ========================================================================== */
/* ---------------- 3.1 控制模式：二选一 ---------------- */
#define GIMBAL_BASE_POSITION_MODE           /* 位控：给角度 → 转到该角度（默认） */
//#define GIMBAL_BASE_SPEED_MODE            /* 速控：给转速 → 恒转速转（⚠️ 只能用 CAN） */

#if defined(GIMBAL_BASE_POSITION_MODE) && defined(GIMBAL_BASE_SPEED_MODE)
#error "config.h: 云台底座模式只能选一个（位控 / 速控）"
#endif
#if !defined(GIMBAL_BASE_POSITION_MODE) && !defined(GIMBAL_BASE_SPEED_MODE)
#error "config.h: 云台底座必须选一个模式（位控 或 速控）"
#endif

/* ---------------- 3.2 控制接口：二选一 ---------------- */
/* CAN：要 CAN 收发器 + 120Ω 终端电阻；有角度反馈，位置环/速度环都能用（推荐正式比赛用）
   ★真 GM6020 只支持 CAN（没有 PWM/模拟输入），所以用 GM6020 就必须选这个 */
#define GIMBAL_IFACE_CAN
/* PWM：舵机模式（给普通 PWM 舵机用的，不是 GM6020），一根线就行、不需要收发器；
   只能位控、没有角度反馈（调车阶段方便） */
//#define GIMBAL_IFACE_PWM

#if defined(GIMBAL_IFACE_CAN) && defined(GIMBAL_IFACE_PWM)
#error "config.h: 控制接口只能选一个（CAN / PWM）"
#endif
#if !defined(GIMBAL_IFACE_CAN) && !defined(GIMBAL_IFACE_PWM)
#error "config.h: 必须选一个控制接口（CAN 或 PWM）"
#endif
#if defined(GIMBAL_IFACE_PWM) && defined(GIMBAL_BASE_SPEED_MODE)
#error "config.h: PWM 接口只能配位控（GIMBAL_BASE_POSITION_MODE）；速度环必须用 CAN"
#endif

/* ---------------- 3.3 PWM 舵机模式参数 ---------------- */
/* 官方 demo 实测：50Hz、脉宽 1080~1920µs 对应 0~360°，即 2.333µs/度
   1080µs=0°  1290µs=90°  1500µs=180°  1710µs=270°  1920µs=360° */
#define GIMBAL_PWM_MIN_US           1080.0f /* 0°   对应脉宽(µs) */
#define GIMBAL_PWM_MAX_US           1920.0f /* 360° 对应脉宽(µs) */
#define GIMBAL_PWM_SETTLE_MS        600     /* PWM 无反馈：发完等这么久就当到位了 */

/* ---------------- 3.4 位置环 ---------------- */
#define GIMBAL_ECD_RANGE            8192    /* 编码器一圈计数（0~8191 = 0~360°） */
#define GIMBAL_ECD_HALF             4096    /* 过零点判断阈值（半圈） */

/* PID：默认值来自已经调好的那套（test_freertos），换负载后按《使用说明》重调 */
#define GIMBAL_POS_KP               23.0f
#define GIMBAL_POS_KI               0.05f
#define GIMBAL_POS_KD               23.0f
#define GIMBAL_POS_ERR_MAX          2048.0f /* 单次参与计算的角度误差上限（1/4 圈） */
#define GIMBAL_POS_INTEGRAL_MAX     3000.0f /* 积分限幅（抗积分饱和） */
#define GIMBAL_POS_SLEW_STEP        3.0f     /* 目标角度缓变：每 1ms 最多变多少 ecd（0 = 不缓变）
                                                ★3.0 ecd/ms ≈ 79°/s（走 120° 约 1.5s）—— 带负载/机械臂用这个
                                                15.0 ≈ 659°/s 太快，只有空载快速回中时才用 */
#define GIMBAL_POS_ARRIVE_ECD       30.0f   /* 到位判定阈值（约 1.3°） */

/* 前馈（可选，先给 0 更安全） */
#define GIMBAL_POS_KFF_POS          0.0f
#define GIMBAL_POS_KFF_VEL          0.0f

/* ---------------- 3.5 速度环 ---------------- */
#define GIMBAL_SPD_KP               20.0f
#define GIMBAL_SPD_KI               0.5f
#define GIMBAL_SPD_KD               0.0f
#define GIMBAL_SPD_ERR_MAX          500.0f  /* 速度误差限幅（rpm） */
#define GIMBAL_SPD_INTEGRAL_MAX     5000.0f

/* ---------------- 3.6 保护 / 限幅（防抽风） ---------------- */
#define GIMBAL_CURRENT_MAX          3000.0f  /* ★输出限幅（0x1FF 满量程按 ±30000 算 → 3000 = 1/10 ≈ 2.4V）
                                                ★第一次带负载调试就用它；确认方向/限位/不抖之后再往大调
                                                原来的 10000（≈8V）对机械臂关节太猛
                                                ⚠️ 0x1FF 的官方量程（±25000 还是 ±30000）带负载前按手册核对 */
#define GIMBAL_OUT_SLEW_STEP        60.0f    /* ★输出（电压指令）斜率限制：每 1ms 最多变化多少
                                                ★最后一道闸：即使 PID 一拍算出满输出，也要 3000/60=50ms
                                                  才升上去（软启动/软刹车）。0 = 关掉（不建议） */
#define GIMBAL_D_LPF                0.7f     /* 微分项低通系数（0~1，越大越平滑） */
#define GIMBAL_FEEDBACK_TIMEOUT_MS  30       /* 这么久没收到反馈 → 输出强制归零 */
#define GIMBAL_STARTUP_DELAY_MS     300      /* 上电后先不发电流，等反馈稳定 */

/* ---------------- 3.7 三位置循环测试（看 VOFA 反馈用） ---------------- */
/* 上电后自动：位置① → 位置② → 位置③ → 位置① …（每个点到位后停 GIMBAL_TEST_DWELL_MS 再走）
   角度是【机械角 0~360°】（编码器 0~8191 对应的那圈），不是多圈累计 */
#define GIMBAL_TEST_ENABLE          0        /* 0 = 不跑三位置循环（云台不动，只保持上电那一下的目标角）
                                                ★现在调升降臂 CAN：关掉它，云台不会转起来缠线，
                                                  但仍然正常发/收 CAN 帧 → 它的"在线"状态可以当
                                                  "总线是否健康"的指示灯（VOFA ch8）*/
#define GIMBAL_TEST_POS_1_DEG       0.0f     /* 位置①：0° */
#define GIMBAL_TEST_POS_2_DEG       120.0f   /* 位置②：120°（按你的机械限位改） */
#define GIMBAL_TEST_POS_3_DEG       240.0f   /* 位置③：240°（按你的机械限位改） */
#define GIMBAL_TEST_DWELL_MS        800      /* 到位后再停这么久才去下一个点（0 = 到位立刻走） */
#define GIMBAL_TEST_MOVE_TIMEOUT_MS 5000     /* 一个点超过这么久还没到位就强制去下一个点
                                                （防卡死；到不了就看 VOFA 里的角度误差）
                                                ⚠️ 必须 > 最慢一次动作的时间：79°/s 走最长 240° 约 3s，
                                                  所以留 5000；把 GIMBAL_POS_SLEW_STEP 调更小就要加大它 */

/* ---------------- 3.8 保护：只留最必要的（★不搞一堆） ---------------- */
/* 实际生效的（参数在 3.4 / 3.6 区，代码在 gm6020.c 的 GM6020_Loop 里）：
     ① 输出限幅            GIMBAL_CURRENT_MAX = 3000（满量程 1/10 ≈ 2.4V）
     ② 输出斜率限制        GIMBAL_OUT_SLEW_STEP = 60/ms → 升到满输出要 50ms（软启动/软刹车）
     ③ 目标角度缓变        GIMBAL_POS_SLEW_STEP = 3 ecd/ms ≈ 79°/s（走 120° 约 1.5s）
     ④ 误差/积分限幅 + 微分低通
     ⑤ CAN 反馈超时 30ms → 输出归零；CAN 一恢复自动继续（不用手动清）
     ⑥ 上电静默 300ms（等反馈稳定）
   ⚠️ 特意【没加】的：过温/超速/堵转判据、软限位、故障锁存
      —— GM6020 模块自带过温过流保护；机械行程限位等臂装好、量出真实限位再加更准。
         需要时说一声，补回来很快。 */

/* ==========================================================================
 * ② 时间基准（放在云台后面是因为它只服务控制环）
 * ========================================================================== */
/* 1 = 用 TIM6 中断跑 1kHz 控制环（CubeMX 已配好：PSC=89 / ARR=999 / NVIC 已勾）
   0 = 主循环软件节拍兜底（抖动大，只适合临时测试） */
#define GIMBAL_USE_TIM6_TICK            1

/* 1 = 由 gm6020.c 提供 CAN1_RX0_IRQHandler（CubeMX 里没勾 CAN1 RX0 中断时用这个）
   0 = 你在 CubeMX 勾了 "CAN1 RX0 interrupt"（it.c 会生成同名函数）→ 必须改 0，
       否则重复定义、链接报错 */
#define GIMBAL_PROVIDE_CAN_IRQ_HANDLER  1

/* ==========================================================================
 * ④ 步进电机（正点原子 PD42S1 闭环步进，串口 TTL 控制）
 *
 *   板上 7 路「闭环步进接口」(8P) 的接线：
 *      1 脚 R  → PB11 (USART3_RX)   2 脚 T  → PB10 (USART3_TX)
 *      3 DIR / 4 PWM / 5 EN →【没接 MCU】，脉冲模式要自己飞线
 *      6 5V   7 GND   8 BATT（电池电压 12~32V，接驱动器 V+）
 *   ⇒ 7 路共用一条串口总线，区分电机只能靠【从机地址】，每台地址必须唯一！
 * ========================================================================== */

/* ---------------- 4.1 通信参数 ---------------- */
#define STEPPER_ENABLE              1           /* 0 = 整个步进模块关掉（其它文件不用改） */

#define STEPPER_UART                (&huart3)   /* PB10=TX(PB11=RX)=板上步进口 T/R
                                                   发指令没反应 → 先把 T/R 两根线对调试试 */
#define STEPPER_UART_INSTANCE       USART3

#define PD42S1_CMD_GAP_MS           2           /* 两条指令最小间隔 ms（手册要求 ≥2ms） */
#define PD42S1_RX_IDLE_MS           3           /* 串口空闲这么久 → 认为一帧收完 */
#define PD42S1_OFFLINE_MS           500         /* 多久收不到应答算掉线 */
#define PD42S1_PROVIDE_UART_CALLBACK 1          /* 1 = 由 pd42s1.c 提供串口接收中断回调 */
#define PD42S1_POS_EXTRA_ZERO       0           /* 位置模式帧是否多发一个 0x00（按手册=0） */

/* ---------------- 4.2 ★多电机表：每台电机在这里设置 ---------------- */
/*
 *  板上 7 个步进口 = 7 行，一行一台电机（`启用` 写 0 就跳过，没接的先写 0）
 *
 *  每行格式： { 启用, 从机地址, 方向取反, 默认转速rpm, 加速度, 位置脉冲 }
 *      启用     : 1 = 自检/测试用这台，0 = 跳过
 *      从机地址 : 同一总线上必须唯一！出厂都是 1，必须逐个改：
 *                 ① 驱动器屏幕菜单里改；② 代码改：PD42S1_SetAddress(旧地址, 新地址);
 *      方向取反 : 1 = 这台装反了，软件里反过来（不用改线）★但注意：它只翻【下发命令】，
 *                 不翻驱动器回传的转速/位置 → 会算错里程计/航向。整侧镜像请改用 5.2 的 CHASSIS_SIGN_xx
 *      默认转速 : 速度模式用（开环 0.1~400 RPM，闭环 0.1~3000 RPM）
 *      加速度   : 0~200（0 = 直接启动，冲击大）
 *      位置脉冲 : 位置模式用，51200 = 一圈
 */
#define STEPPER_MOTOR_1   { 1, 1, 0, 120.0f, 100, 51200u }   /* 地址 1（默认接 1 号口） */
#define STEPPER_MOTOR_2   { 1, 2, 0, 120.0f, 100, 51200u }   /* 地址 2（默认接 2 号口，未接 → 启用写 0） */
#define STEPPER_MOTOR_3   { 1, 3, 0, 120.0f, 100, 51200u }   /* 地址 3 */
#define STEPPER_MOTOR_4   { 1, 4, 0, 120.0f, 100, 51200u }   /* 地址 4 */
#define STEPPER_MOTOR_5   { 1, 5, 0, 120.0f, 100, 51200u }   /* 地址 5 ★升降（同步带，单轴）*/
#define STEPPER_MOTOR_6   { 0, 6, 0, 120.0f, 100, 51200u }   /* 地址 6 旧齿条伸缩：已拆除，禁用 */
#define STEPPER_MOTOR_7   { 0, 7, 0, 120.0f, 100, 51200u }   /* 地址 7 */

/* 多机接线/调试小抄
 *   ① 一次只接一台，先把它的地址改成目标地址（屏幕菜单 或 PD42S1_SetAddress(1, 2)）；
 *   ② 全部接上后，自检会按表格顺序【一台一台】跑：每台 正转→停→反转→停；
 *   ③ VOFA：ch5/ch6 = 当前那台的转速/位置，ch7 = 当前电机号，ch8 = 该驱动器模式；
 *   ④ 想看某台的单独数据：PD42S1_GetMotorState(0..6)->rpm / ->pos / ->state / ->volt。 */

/* ---------------- 4.3 上电自检（先确认每台都能转） ---------------- */
#define STEPPER_SELFTEST            1           /* 0 = 只留驱动，不做自检 */

/* 测试模式：⚠️ 手册里「设置工作模式」0x62 只能设 0~7，开环模式（8/9/10）只能在
   驱动器屏幕上选 —— 所以"用哪种命令"要跟驱动器当前模式对上，否则它可能不执行 */
#define STEPPER_TEST_MODE           6           /* 6 自动适配 ★推荐：先读驱动器当前模式
                                                      ・模式是 8/9/10（开环）→ 发 0xE0 开环速度
                                                      ・其它模式 → 发 0x62 设通信速度 + 0xF1 闭环
                                                      ・读不到模式 → 两种轮流试
                                                   7 地址扫描（不发运动指令，只探测 1~7 地址
                                                     哪些有应答 → 看 VOFA ch7/ch9）
                                                   4 强制闭环速度 0xF1（并设模式=通信速度）
                                                   5 强制闭环绝对位置 0xF2
                                                   1 开环速度 0xE0（需屏幕切到"开环速度"）
                                                   2 开环相对位置 0xE2（同上）
                                                   3 开环绝对位置 0xE1（同上） */

#define STEPPER_TEST_SET_MODE       0           /* 1 = 强制用 0x62 设模式（仅模式 4/5 用）
                                                ⚠️ 注意：设成通信速度=闭环模式，若编码器没
                                                   校准会转不起来；模式 6 自动判断更保险 */
#define STEPPER_TEST_PERIOD_MS      2000        /* 每个动作持续多久（ms） */
#define STEPPER_TEST_PHASES         4           /* 速度模式相位：4 = 正转/停/反转/停
                                                   2 = 只正转/反转（多台时测得快） */

/* ==========================================================================
 * ⑤ 底盘（麦克纳姆轮，4 台 PD42S1 步进）
 *
 *   对外单位：mm/s（速度）、度/s（自转）、度（航向）—— 业务代码不用管 rpm，
 *   换算只在 chassis.c 的运动学函数里做一次。
 *   坐标约定：vx>0 前进、vy>0 向左平移、wz>0 逆时针（左转）
 * ========================================================================== */
#define CHASSIS_ENABLE              1           /* 1 = 启用底盘模块（会自动关掉步进自检，
                                                   免得自检和底盘抢电机） */
#define CHASSIS_FORCE_ECHO          1           /* ★1 = 每 3 秒把 1~4 号电机设成"要回响"(0x6F)
                                                   驱动器若被设成"不回响"会执行命令但不回字节 →
                                                   转速/位置反馈全丢（rpm 恒 0、N0/4）。
                                                   这个宏让它自愈；查清后可以关掉省总线流量 */

/* ---------------- 5.1 机械参数（★按你的车量/按图纸填） ---------------- */
#define CHASSIS_WHEEL_DIA_MM        75.0f       /* 麦轮外径（含辊子），已确认 = 75mm */
#define CHASSIS_WHEELBASE_MM        200.0f      /* 轴距：前轮中心 ↔ 后轮中心 */
#define CHASSIS_TRACK_MM            200.0f      /* 轮距：左轮中心 ↔ 右轮中心 */
#define CHASSIS_GEAR_RATIO          1.0f        /* 减速比：步进直驱 = 1 */

/* ---------------- 5.2 四个轮子 ↔ ④区 步进电机表 的对应 ----------------
   索引 = STEPPER_MOTOR_x 的行号（0 = 第 1 行 = 从机地址 1），按实际接线填
   ★★ 决定"哪台电机当左前"的是【从机地址】，**不是插在板上哪个 8P 口**！
       7 路步进口在板内已经并到同一条 USART3 总线，代码只按地址点名。
       推荐：给驱动器改地址 1=左前 / 2=右前 / 3=左后 / 4=右后 → 插哪个口都行，
       下面这 4 行就用默认值、一个字都不用改。
       只有"地址 ↔ 轮位"的实际对应不是这个顺序时，才改这 4 行。 */
#define CHASSIS_MOTOR_FL            0           /* 左前 */
#define CHASSIS_MOTOR_FR            1           /* 右前 */
#define CHASSIS_MOTOR_RL            2           /* 左后 */
#define CHASSIS_MOTOR_RR            3           /* 右后 */

/* 每个轮子的方向符号：±1.0f
   ★ 当前值（2026-09-30 调试中）：FL/RR = +1.0f，FR/RL = -1.0f。换电机/换线后按下面办法重测。
   以后换电机/换线了，照这个办法重测：架空给"前进"指令，逐轮看【轮子顶部】往哪边转 ——
     顶部朝【车头】= 这个轮子在往前驱动 → 填 +1.0f；顶部朝【车尾】= 装反了 → 填 -1.0f。
     四个轮子各自独立填，别靠"车往哪边偏"去推（左右镜像、前后镜像都会打转，推不出是哪两个）。
   ⚠️ 修镜像必须用这里的 CHASSIS_SIGN，**别用 ④区电机表的 dir_inv**：
      dir_inv 只翻下发命令、不翻反馈，用它车能开对但里程计/航向会算错。 */
#define CHASSIS_SIGN_FL             1.0f
#define CHASSIS_SIGN_FR             -1.0f
#define CHASSIS_SIGN_RL             1.0f
#define CHASSIS_SIGN_RR             -1.0f

/* 麦轮辊子布局：1 = X 型（常见），0 = O 型
   自检时"左右平移"如果变成"前后"→ 把这个改一下 */
#define CHASSIS_ROLLER_X            1

/* ---------------- 5.3 限幅与下发 ---------------- */
#define CHASSIS_MAX_RPM             300.0f      /* 单轮最大转速（超了按比例缩，保持方向）
                                                   ⚠️ 先给小点（如 120）确认动作对了再加 */
#define CHASSIS_MAX_ACC             100         /* 加速度 0~200（交给驱动器做加减速） */
#define CHASSIS_MIN_RPM             0.1f        /* 驱动器最小合法速度；底盘"停"就用它
                                                   ⚠️ 别用 0xFC 刹车来停车：会锁在刹车态(state=8)
                                                      而且电机严重发烫（手册 4.4.13） */
#define CHASSIS_CMD_DEADBAND_RPM    0.5f        /* 目标变化小于这个值就不重发（★恢复原值） */
#define CHASSIS_REFRESH_MS          250         /* ★>0 = 每这么久重发一次速度（含 0）
                                                   为什么需要：停车时"速度 0"那帧只发一次，
                                                   驱动器内部若有一点偏差就会慢慢蠕动
                                                   （现象：停下了轮子还微微动）。定时重发能摁住它。
                                                   0 = 只变化时发（省总线，但会蠕动） */

/* ---------------- 5.4 上电自检（走方形，验证运动学/接线） ---------------- */
#define CHASSIS_SELFTEST            0           /* 1 = 上电自动跑一遍动作序列（★静态测 IMU 时必须是 0，否则一上电车就动） */
#define CHASSIS_TEST_STEP_MS        1500        /* 每个动作持续多久 */
#define CHASSIS_TEST_SPEED          250.0f      /* 自检线速度 mm/s */
#define CHASSIS_TEST_WZ             60.0f       /* 自检自转角速度 度/s */

/* ---------------- 5.5 里程计 / 航向 ---------------- */
#define CHASSIS_FB_PERIOD_MS        20          /* 每这么久读一台电机反馈（4 台轮流 → 80ms 一轮） */
#define CHASSIS_ODOM_ENABLE         1           /* 用 4 个轮子的编码器位置算里程计/航向 */
/* 航向保持、以及三个闭环（航向稳定/旋转稳定/平移稳定）的参数都在下面 5.7 区 */

/* ---------------- 5.6 陀螺仪（IMU = YbImu 模块，默认走【串口 UART4】） ----------------
   接法（串口，当前方案）：模块 TX→PC11(UART4_RX)  模块 RX→PC10(UART4_TX)  GND→GND  VCC→3V3/5V
   启用：① CubeMX 打开 UART4（PC10/PC11，Asynchronous，115200，勾上 UART4 全局中断）
         ② 本区 IMU_ENABLE = 1、⑬ 区 IMU_IF_UART = 1 → 编译烧录
   （老的 I²C 接法 SCL→PB6、SDA→PB7 保留着：把 ⑬ 区 IMU_IF_UART 改 0 就切回去） */
#define IMU_ENABLE                  1           /* 1 = 使用外接 IMU */
#define IMU_ADDR_7BIT               0x23        /* ★YbImu 固定 0x23（不是 MPU6050 的 0x68！） */
#define IMU_GYRO_Z_SIGN             1.0f        /* 陀螺 Z 符号：左转读数为正填 1.0f，否则 -1.0f */
#define IMU_CALIB_MS                1000        /* （旧 MPU6050 用；YbImu 已改用下面的 IMU_CALIB_WAIT_MS） */
#define IMU_YAW_ODOM_K              0.02f       /* 互补滤波：用轮式航向慢慢拉回陀螺漂移 */

/* ==========================================================================
 * 5.7 三个闭环 PID（航向稳定 / 旋转稳定 / 平移稳定）
 *
 *   ① 航向稳定：走直、平移时**不让车偏头** —— 误差(度) → wz 修正(度/s)
 *   ② 旋转稳定：转到并保持某个角度     —— 误差(度) → wz(度/s)
 *   ③ 平移稳定：横移时不跑偏、到指定横向位移 —— 误差(mm) → vy(mm/s)
 *
 *   PID 实现要点（都在 chassis.c 的 pid_step 里）：
 *     · 误差死区：|err| 小于死区就当 0（防抖，代价是稳态有微小静差）
 *     · 积分限幅（抗积分饱和）：长时间到不了目标时不会把积分憋爆
 *     · 输出限幅：再大的误差也只给到 out_max
 *     · 微分低通：编码器角度的 D 项噪声大，直接微分会抖
 *   ⚠️ 调这个之前先确认 IMU 是通的（ch3=0、ch4=38）：航向环靠 yaw 反馈，
 *      没 IMU 时只能用轮式航向兜底（会漂，先用它把方向调对也行）
 * ========================================================================== */

/* ---------------- ① 航向稳定（CHASSIS_HEADING_HOLD=1 时生效） ---------------- */
#define CHASSIS_HEADING_HOLD         1

/* ★IMU 断帧门限（ms）：收帧计数超过这么久没增长，就认为 yaw 是旧值 → 航向/旋转环输出 0
   原因：电机一转 IMU 就断帧（实测转向 1.6 秒内 0 帧），拿旧 yaw 闭环会让车越偏越大 */
#define IMU_STALE_MS                 400u        /* ★400ms：帧率低时不至于一直判断帧（150 太严，航向环常年沉默） */
#define CHASSIS_HEADING_TARGET_DEG  0.0f        /* 想保持的航向（0 = 上电方向 / 上次 ResetYaw 的方向） */
/* ★走直微调（度）：正 = 稍微朝左走，负 = 朝右走。
   实测走 2 米右偏 10cm ≈ 偏角 2.9° → 填 +2.9 抵消。运行时命令 b<度> 在线改，不用重烧 */

/* ★每轮速度系数（走直补偿）：直接乘在轮速上，不依赖轮速反馈
   调法：走 2 米量偏移 → 右偏就把右侧(<1.00)、左侧(>1.00) 微调
   范围 0.90~1.10；运行命令 t<轮号> <系数> 在线改，如 t2 0.98
   轮号：1=左前 2=右前 3=左后 4=右后 */
#define CHASSIS_TRIM_RPM_FL  1.000f
#define CHASSIS_TRIM_RPM_FR  1.000f
#define CHASSIS_TRIM_RPM_RL  1.000f
#define CHASSIS_TRIM_RPM_RR  1.000f

/* ★侧滑补偿系数：前进时叠加 vy = vx × 这个值，抵消麦轮横向漂移
   实测 走 2 米横移 12cm（车头没弯）→ 比例 0.06，先试 ±0.03 定方向再加大
   正负表示往左/往右补偿；运行时命令 c<值> 在线改 */
#define CHASSIS_CRAB_TRIM   0.0f
#define CHASSIS_STRAIGHT_TRIM_DEG   0.0f
#define CHASSIS_HEADING_KP          10.0f       /* P：1 度误差 → 10 度/s 修正（★恢复为之前调好的值） */
#define CHASSIS_HEADING_KI          0.0f       /* I：消除"总差一点点"的静差（先 0，最后再加） */
#define CHASSIS_HEADING_KD          0.0f        /* D：抑制过头/摆动（加之前先看波形抖不抖） */
#define CHASSIS_HDG_DEADBAND_DEG    0.5f        /* 误差死区：小于它就当不偏（★恢复原值） */
#define CHASSIS_HDG_I_MAX           200.0f      /* 积分限幅 */
#define CHASSIS_HDG_WZ_MAX          120.0f      /* 修正量限幅（度/s）★恢复原值 */
#define CHASSIS_HDG_MAX_ERR_DEG     25.0f       /* ★大误差保护：航向误差超过它就【不纠偏】
                                                   （航向环只管走直时的小偏差；上电没归零/yaw 漂飞/被搬动时
                                                     防止它把车原地猛拧，实测会拧 120° 转一整圈） */

/* ---------------- ② 旋转稳定（Chassis_TurnToDeg 用） ---------------- */
#define CHASSIS_TURN_KP             7.0f        /* P（度误差 → 度/s）：★实测最优（2026-10-20 调参） */
#define CHASSIS_TURN_KI             0.10f      /* I：很小的一点，补最后的静差
                                                  ⚠️ 欠压时不要加 I：会积分饱和，一恢复供电就冲过头（实测冲过 83°） */
#define CHASSIS_TURN_KD             0.7f       /* D：接近目标时刹车，抑制超调 */
#define CHASSIS_TURN_DEADBAND_DEG   0.6f        /* 误差死区 */
#define CHASSIS_TURN_I_MAX          300.0f
#define CHASSIS_TURN_WZ_MAX         140.0f      /* 最大角速度（度/s）：实测 140 时 90° 约 0.9~1.1 秒 */
#define CHASSIS_TURN_SLEW_DPS2      400.0f      /* wz 变化率限制（度/s²）：防"一顿一顿"地甩 */
#define CHASSIS_TURN_ARRIVE_DEG     1.5f        /* 到位判定阈值（度） */
/* ★转向固定偏置补偿：实测每次停下都比目标少约 1.9°（与转角无关，90°和180°都少同样多，
   说明是机械背隙/回弹而非 IMU 标度问题）→ 目标角多打这么多，回弹后正好停准。
   实测数据：90° 平均 -1.90°、180° -1.10°、收紧阈值到 0.8° 后仍 -1.90° */
#define CHASSIS_TURN_BIAS_DEG       0.0f        /* 2026-10-21 归零重新标定：现在有零漂补偿、关闭了静止冻结、修好了指令队列，旧的 1.9 不再可靠。测出真实误差后再填 */

/* ★yaw 标度修正系数：IMU 报的角度与真实角度的比例
   标定方法：手动转 4 个 90°（共 360°），看 IMU 累计报了多少（例如 350°）
            标度 = 360 / 350 = 1.0286
   之后可用命令 K1.0286 在线改，不用重新烧录
   1.0 = 不修正（默认） */
#define CHASSIS_YAW_SCALE           1.0f

/* ---- ★非阻塞零漂补偿（推荐 1）----
   原理：模块的陀螺零偏会让 yaw 一直漂（实测约 10°/分）。这里在静止时用低通学出
         "漂移率"，然后无论动还是静都按时间连续扣掉 → 静止几乎不漂、长直线也不跑弯。
   和旧方案的区别：旧方案是"冻结 yaw"（把变化藏起来），会藏住转向回弹导致连转累积；
   这个方案是"真的把漂移扣掉"，转向的回弹照样如实反映。 */
#define CHASSIS_DRIFT_TRIM          1
#define CHASSIS_DRIFT_LEARN_GAIN    0.04f      /* 学习速度：每次只改 4%，约 3~5 秒收敛 */
#define CHASSIS_DRIFT_LEARN_MAX_DPS 0.8f       /* 变化率小于它才认为是"静止漂移"（否则是车真在转） */
#define CHASSIS_DRIFT_MAX_DPS       1.5f       /* 漂移率限幅（度/秒）：防止把大误差学进去 */
#define CHASSIS_TURN_SETTLE_MS      150         /* 进入阈值后要稳住这么久才算到位（防冲过头误判） */
#define CHASSIS_TURN_USE_PID        1           /* ★1 = 转弯测试用旋转环（yaw 到了才算转够，
                                                   要用 IMU；到 ARRIVE_DEG 内稳住 SETTLE_MS 算到位）
                                                   0 = 老的固定 wz（开环，转多少看时间） */

/* ---------------- ③ 平移稳定（Chassis_StrafeStart 用） ---------------- */
#define CHASSIS_SIDE_KP             0.0f        /* P（mm 误差 → mm/s） */
#define CHASSIS_SIDE_KI             0.0f
#define CHASSIS_SIDE_KD             0.0f
#define CHASSIS_SIDE_DEADBAND_MM    2.0f        /* 误差死区 */
#define CHASSIS_SIDE_I_MAX          300.0f
#define CHASSIS_SIDE_VY_MAX         300.0f      /* 横向速度限幅（mm/s） */
#define CHASSIS_SIDE_ARRIVE_MM      5.0f        /* 到位判定（mm） */
#define CHASSIS_STRAFE_USE_PID      0           /* 平移测试（⑩区）是否用这个环：1 = 目标位移交给平移环（到位 MM 见 CHASSIS_SIDE_ARRIVE_MM） */

/* ==========================================================================
 * ⑥ VOFA+ 上位机波形（JustFloat 协议）
 * ========================================================================== */
#define VOFA_OUTPUT_ENABLE          1           /* 0 = 关掉（vofa.c 变空函数，main 不用改）
                                                  ★现在 = 1：调底盘要看波形 */
#define VOFA_TX_HZ                  20          /* 发送频率 Hz，与 VOFA+ 里的采样率填一致
                                                  ★蓝牙 9600：10 路 44 字节/帧 → 20Hz = 880B/s，
                                                    正好在 9600 上限内（丢几帧没关系，非阻塞发送）
                                                    走有线 USB-TTL 时改回 100 */
/* ---- 主口发送方式 ----
   1 = 中断发送（非阻塞，默认）：这一帧还没发完就跳过，绝不拖住控制环
   0 = 阻塞发送：会等这一帧发完才返回（9600 下每帧 46ms，别用） */
#define VOFA_IT_TX                  1

/* ---- VOFA 发哪套数据（三选一，改这里 main.c 自动切）----
   ★当前 = VOFA_SRC_IMU（正在联调 IMU）；回去调升降臂就改回 VOFA_SRC_ARM，
     调云台改 VOFA_SRC_GIMBAL */
#define VOFA_SRC_IMU                0           /* IMU 姿态 5 路：ch0 yaw / ch1 roll / ch2 pitch / ch3 状态 / ch4 最近功能码 */
#define VOFA_SRC_GIMBAL             1           /* 云台 GM6020 10 路 */
#define VOFA_SRC_ARM                2           /* 升降臂 2 路（两个编码器）/ 开嗅探时 12 路 */
#define VOFA_SRC_CHASSIS            3           /* ★调 PID 用：10 路（yaw/误差/输出/位移…），通道表见下 */
#define VOFA_SOURCE                 VOFA_SRC_IMU  /* ★现在只看陀螺仪/姿态：4 路（yaw/roll/pitch/wz） */
/* VOFA_SRC_CHASSIS 的通道表（调三个环时看这个）：
     ch0 yaw(度)        ch1 航向误差(度)   ch2 航向环输出(度/s)  ch3 实测 wz(度/s)
     ch4 前向位移(mm)   ch5 横向位移(mm)   ch6 旋转误差(度)      ch7 旋转环输出(度/s)
     ch8 平移误差(mm)   ch9 平移环输出(mm/s) */
/* 通道表见《使用说明》七.2 / 《底盘调车手册》⑦ */

/* 发给哪个串口：
     (&huart2) = 「串口2」= USART2 = PA2(TX)/PA3(RX)  ← 有线 USB-TTL
     (&huart4) = 「串口4」= UART4  = PA0(TX)/PA1(RX)  ← ★蓝牙 HC-05（现在用这个）
   ★主口是蓝牙时：VOFA_BT_EN 必须填 0（同一个口不用发两份）、VOFA_TX_HZ 填 20、
     BT_ENABLE 填 1（BT_Init 会把 UART4 重配成 BT_BAUD=9600）。
   换口就改这里，或在 main 里调 VOFA_SetUart() */
#define VOFA_UART                   (&huart4)

/* ==================== 心跳灯（板载用户 LED = PD0，低电平点亮） ====================
   1 = 打开：主循环里每 500ms 翻转一次
   用途：一眼看出"MCU 主循环到底在不在跑"——
        灯在闪 = 程序活着（那 VOFA 没数据就是串口/上位机那边的问题）
        灯不闪 = 程序卡死/没跑起来（那就跟 VOFA 无关，是固件的问题） */
#define MAIN_LED_HEARTBEAT          1
#define MAIN_LED_HEARTBEAT_MS       500

/* ==========================================================================
 * ⑦ 快速检查清单（出问题先看这几条）
 * ==========================================================================
 *  云台不动：      GIMBAL_IFACE_* 选对了吗？CAN 有 120Ω 吗？PWM 模式别发 CAN 控制帧
 *                  （CAN 优先于 PWM）→ 看 GM6020_IsOnline()
 *  步进不动：      ① T/R 有没有接反（最常见）② 驱动器供电 12~32V
 *                  ③ 从机地址和表里一致吗 ④ 模式对不对（看 ch8 / PD42S1_Get()->mode）
 *  波形是 0 / 没波形：串口2 的 PA2 接 USB-TTL 的 RX、GND 共地、VOFA+ 选 JustFloat
 *  编译报 htim1：  CubeMX 里没加 TIM1（PWM 模式必须加）
 *  编译报重复定义：CubeMX 勾了 CAN1 RX0 中断 → 把 GIMBAL_PROVIDE_CAN_IRQ_HANDLER 改 0
 */

/* ==========================================================================
 * ⑧ 里程测试（调车用：前进 N 米 → 停 → 后退同样距离 → 停）
 *
 *   判距离用【时间 × 速度】，**不用里程计** —— 简单、稳，不受方向符号影响：
 *        每段要跑多久 = 距离 ÷ 速度（下面的 CHASSIS_DIST_TEST_MS 自动算好）
 *        例：2000mm ÷ 150mm/s ≈ 13.3 秒
 *   步进驱动器速度模式是闭环的，转速准，实际距离误差只有几个百分点，
 *   用来验证"车会不会走、方向对不对"够用。
 *   （以后要跑"精确距离 / 直线 / 定点"再上里程计闭环，那时先按实测标定轮径。）
 *
 *   打开后会发生什么：
 *     · 上电自动开跑，走完自己停，**不用接上位机**
 *     · CHASSIS_SELFTEST（走方形那个自检）自动失效，两者不抢速度指令
 *     · VOFA 只发 3 个"里程计"通道（里程计 vx / 累计路程 / 本段目标路程），
 *       云台、步进、IMU 的通道全部停用（见 main.c 的 App_VofaSource）
 *
 *   ⚠️⚠️ 上电就会真的把车开出去！先架空，或留够 2 米以上直线空间。
 *   怎么核对走得对不对 —— 看 VOFA 的"累计路程"（ch1）：
 *       前进段从 0 涨到 2000 左右，后退段再从 2000 涨到 4000 左右
 *       （它是"累计路程"，来回都累加，所以倒退那段也是涨的）。
 *       前进时 vx（ch0）应该是【正】的；一直是负的 → 轮子方向符号反了，
 *       改 5.2 区的 CHASSIS_SIGN_xx，不是改这里。
 * ========================================================================== */

#define CHASSIS_DIST_TEST           0       /* ★1 = 打开里程测试；0 = 关闭（恢复正常） */
#define CHASSIS_DIST_TEST_MM        2000.0f  /* ★走多少毫米：2000 = 2 米（前进这一段） */
#define CHASSIS_DIST_TEST_SPEED     300.0f   /* 测试速度 mm/s（原 150；第一次调车先别给大） */
#define CHASSIS_DIST_TEST_PAUSE_MS  800      /* 走完一段停多久再反向 */
#define CHASSIS_DIST_TEST_LOOP      0        /* 1 = 来回循环跑不停；0 = 去、回各一次就停 */

/* 每段跑多久（自动算 = 距离 ÷ 速度），一般不用改 */
#define CHASSIS_DIST_TEST_MS        ((uint32_t)((CHASSIS_DIST_TEST_MM / CHASSIS_DIST_TEST_SPEED) * 1000.0f))

/* 实际生效条件（main.c 里用它判断）：测试打开 且 底盘模块启用 */
#define CHASSIS_DIST_TEST_ACTIVE    (CHASSIS_DIST_TEST && CHASSIS_ENABLE)

/* ==========================================================================
 * ⑨ 转弯测试（调车用：左转 A1 → 右转 A1 → 左转 A2 → 右转 A2）
 *
 *   序列（每段之间停 CHASSIS_TURN_TEST_PAUSE_MS）：
 *       ① 左转 90°  ② 停  ③ 右转 90°  ④ 停     ← ③ 之后应该转回原朝向
 *       ⑤ 左转 180° ⑥ 停  ⑦ 右转 180° ⑧ 停     ← ⑦ 之后又回到原朝向
 *   成对设计的用意：如果最后车头指向和起步一致，说明"左右对称、转角大致对"。
 *
 *   方向判据：**wz > 0 = 逆时针 = 左转**（wz 单位 度/s，和整车坐标约定一致）
 *   精度说明：和 ⑧ 一样是"时间 × 角速度"，**没有 IMU 就没法测真实转角**。
 *     原地转时四个麦轮横向刮地、打滑明显，实际转角一般比设定值【偏小】（轮子转了、车没转够）。
 *     要看真实转角 → 接 IMU（IMU_ENABLE 1 + 航向保持），或事后用卷尺量车身落点。
 *
 *   ⚠️⚠️ 上电就会真的原地转！留出车长的空间，别让线缆绕住。
 *   ⚠️ 和 ⑧ 里程测试互斥：**两个都打开时以转弯测试为准**（⑨ 优先）。
 *   ⚠️ CHASSIS_SIGN_xx 只能是 ±1.0f：它直接乘在目标转速上，写 2.0f 会让转角和线速度一起翻倍！
 * ========================================================================== */

#define CHASSIS_TURN_TEST           0        /* 0 = 关闭：开着会让上电默认进转弯模式，一复位就自己转 */        /* ★1 = 打开转弯测试（屏幕菜单 ⑨）：自动跑
                                                   左90 → 右90 → 左180 → 右180，用旋转环到位
                                                   这样不依赖电脑就能调转向 */
#define CHASSIS_TURN_TEST_WZ        60.0f    /* 转弯角速度 度/s（60 → 90°≈1.5s、180°≈3s） */
#define CHASSIS_TURN_TEST_A1        90.0f    /* 第一组转角：左转 A1，再右转 A1 */
#define CHASSIS_TURN_TEST_A2        180.0f   /* 第二组转角：左转 A2，再右转 A2 */
#define CHASSIS_TURN_TEST_PAUSE_MS  800      /* 每段之间停多久 */
#define CHASSIS_TURN_TEST_LOOP      0        /* 1 = 四段跑完接着从头再来；0 = 跑一遍就停 */
#define CHASSIS_TURN_TEST_TIMEOUT_MS 8000    /* PID 版单段超时兜底（转不到就放弃，别死等） */

/* 实际生效条件（main.c 里用它判断）：转弯测试打开 且 底盘模块启用 */
#define CHASSIS_TURN_TEST_ACTIVE    (CHASSIS_TURN_TEST && CHASSIS_ENABLE)


/* ==========================================================================
 * ⑩ 平移测试（调车用：左平移 N mm → 停 → 右平移 N mm）
 *
 *   序列：① 左平移 CHASSIS_STRAFE_TEST_MM → ② 停 → ③ 右平移同样距离 → ④ 停
 *   方向判据：**vy > 0 = 向左平移**（和整车坐标约定一致）
 *
 *   为什么平移比转弯好用：**没有 IMU 也能一眼判对错** ——
 *       车【横着走】= 对；车【前后走】= 辊子方向/符号错了（改 CHASSIS_ROLLER_X 或 CHASSIS_SIGN_xx）
 *       车【原地转/斜着走】= 四个轮子符号不成套
 *
 *   判距离用"时间 × 速度"（每段 = 距离 ÷ 速度），和 ⑧ 同一套思路，不需要反馈。
 *   ⚠️ 上电就会真的横着走，两侧各留出 CHASSIS_STRAFE_TEST_MM 的空间。
 *   ⚠️ 与 ⑧⑨ 互斥：三个测试的优先级是 **⑩ 平移 > ⑨ 转弯 > ⑧ 里程**。
 * ========================================================================== */

#define CHASSIS_STRAFE_TEST           0        /* ★1 = 打开平移测试（会盖掉 ⑧⑨） */
#define CHASSIS_STRAFE_TEST_MM        400.0f   /* ★平移多少毫米（横着走，占地方，先给 300~500） */
#define CHASSIS_STRAFE_TEST_SPEED     150.0f   /* 平移速度 mm/s */
#define CHASSIS_STRAFE_TEST_PAUSE_MS  800      /* 每段之间停多久 */
#define CHASSIS_STRAFE_TEST_LOOP      0        /* 1 = 左右来回不停；0 = 各一次就停 */

/* 每段跑多久（自动算 = 距离 ÷ 速度） */
#define CHASSIS_STRAFE_TEST_MS        ((uint32_t)((CHASSIS_STRAFE_TEST_MM / CHASSIS_STRAFE_TEST_SPEED) * 1000.0f))
#define CHASSIS_STRAFE_TEST_TIMEOUT_MS 8000  /* PID 版单段超时兜底 */

/* 实际生效条件：平移测试打开 且 底盘模块启用 */
#define CHASSIS_STRAFE_TEST_ACTIVE    (CHASSIS_STRAFE_TEST && CHASSIS_ENABLE)

/* ==========================================================================
 * ⑪ 测试例程 1（调车用：1×1 米方形 → 回起点 → 左平移 1 米 → 平移回来）
 *
 *   路线（共 11 步，走完约 6 米）：          每步的完成条件
 *     ① 前进 SIDE_MM → ② 左转90°            走距离：里程计累计路程增量 ≥ 距离 - 容差
 *     ③ 前进 SIDE_MM → ④ 左转90°            转角度：时间 × 角速度（没 IMU 时）
 *     ⑤ 前进 SIDE_MM → ⑥ 左转90°                    航向 ≥ 角度 - 容差（接了 IMU 时自动切这支）
 *     ⑦ 前进 SIDE_MM → ⑧ 左转90°  ← 回到起点、朝向转回来
 *     ⑨ 左平移 STRAFE_MM（vy>0 = 向左）→ ⑩ 右平移 STRAFE_MM（平移回来）→ ⑪ 停
 *
 *   ⚠️ 占地方：方形 SIDE_MM 见方，再往左横 STRAFE_MM → **两侧各留出 SIDE+STRAFE 的空间**。
 *   ⚠️ 纯开环（没 IMU）：4 个 90° 都是"时间×角速度"估的，每转可能差十几度 →
 *      走完方形往往不闭合（位置/朝向都会偏），后面的平移也跟着偏。
 *      想闭合：接 MPU6050 → IMU_ENABLE 改 1，本文件里 #if IMU_ENABLE 那支会自动生效。
 *   ⚠️ 与 ⑧⑨⑩ 互斥：优先级 **⑪ 例程1 > ⑩ 平移 > ⑨ 转弯 > ⑧ 里程**。
 * ========================================================================== */

#define CHASSIS_MISSION1_TEST         0        /* ★1 = 打开测试例程1（会盖掉 ⑧⑨⑩）
                                                  ★静态测 IMU / 校准时先写 0：这样上电默认模式是"停"，
                                                    车不会自己跑（例程1 还是能从屏幕点"1"跑） */
#define CHASSIS_MISSION1_SIDE_MM      1000.0f  /* ★方形边长 mm（1×1 米就填 1000） */
#define CHASSIS_MISSION1_STRAFE_MM    1000.0f  /* ★走完方形后左平移多少 mm，再平移回来 */
#define CHASSIS_MISSION1_SPEED        300.0f   /* 直行 / 平移速度 mm/s */
#define CHASSIS_MISSION1_TURN_WZ      60.0f    /* 转弯角速度 度/s（90° 约 1.5s） */
#define CHASSIS_MISSION1_TOL_MM       30.0f    /* 距离容差（里程计有误差，别设 0） */
#define CHASSIS_MISSION1_TOL_DEG      5.0f     /* 角度容差（只有接了 IMU 才用得到） */
#define CHASSIS_MISSION1_GAP_MS       500      /* 每步之间停顿多久（0 = 不停，直接接下一步） */
#define CHASSIS_MISSION1_TIMEOUT_MS   8000     /* 单步超时保护（标称单步 1.5~3.3s，8s 足够；别调太大） */
#define CHASSIS_MISSION1_LOOP         0        /* 1 = 跑完整条路线接着从头再来 */

/* 实际生效条件：例程打开 且 底盘模块启用 */
#define CHASSIS_MISSION1_ACTIVE       (CHASSIS_MISSION1_TEST && CHASSIS_ENABLE)

/* ---- 方形怎么画（⑪ 的核心：没有 IMU 时怎么才能闭合）----
   1 = **用"平移"画方形**（前进1米 → 左移1米 → 后退1米 → 右移1米）。⭐推荐，没有 IMU 也能闭合：
       ① 全程不转弯 → 航向不会累积误差（转弯版每转一次都留下十几度误差）
       ② 四段轨迹两两对称 → 单段走短/走长会被下一段抵消（平移打滑少走 10%，左右各少走一点，正好回起点）
       代价：画出来的是"用平移走出来的方形"，车身朝向全程不变。
   0 = 用"转弯"画方形（前进1米 → 左转90° ×4）。没 IMU 时 4 个 90° 全靠"时间×角速度"估，
       原地转打滑每次差十几度 → 4 次累计 20~40°，**走完第 4 边回不到起点**（接 IMU 才能闭合）。*/
#define CHASSIS_MISSION1_NO_TURN      1

/* ---- 每段"走够了没有"用什么判 ----
   0 = **用时间**（距离 ÷ 速度）：和 ⑧⑩ 一样，不依赖任何反馈，**默认推荐** ★
       代价：实际距离取决于速度标定/轮径（闭环步进通常差几个百分点），1 米大概差几厘米
   1 = 用里程计（累计路程增量）：标定准了更精确，但**前提是里程计真的在工作** ——
       上电后先看 VOFA 的 ch4（里程计累计路程）是否随车移动而增长，不涨就别用这个模式
       （本车曾因里程计被卡死导致"一直跑到超时"，所以默认先不用它） */
#define CHASSIS_MISSION1_JUDGE_BY_ODOM   0

/* ---- 各自动测试的最终使能（优先级：⑪例程1 > ⑩平移 > ⑨转弯 > ⑧里程；高优先级打开时低优先级不编译）---- */
#define CHASSIS_STRAFE_TEST_ON        (CHASSIS_STRAFE_TEST_ACTIVE && !CHASSIS_MISSION1_ACTIVE)
#define CHASSIS_TURN_TEST_ON          (CHASSIS_TURN_TEST_ACTIVE   && !CHASSIS_STRAFE_TEST_ACTIVE && !CHASSIS_MISSION1_ACTIVE)
#define CHASSIS_DIST_TEST_ON          (CHASSIS_DIST_TEST_ACTIVE   && !CHASSIS_TURN_TEST_ACTIVE && !CHASSIS_STRAFE_TEST_ACTIVE && !CHASSIS_MISSION1_ACTIVE)


/* ==========================================================================
 * ⑫ 触摸屏菜单（陶晶驰 TJC，走 UART5 = PC12(TX) / PD2(RX)）
 *
 *   用法：屏幕上 5 个按钮，每个按钮的"触摸弹起事件"里写一句 printh XX：
 *       例程1 = printh 01    平移 = printh 02    转弯 = printh 03
 *       里程  = printh 04    停止 = printh 00
 *   固件收到后【立刻切换当前测试模式并复位该测试】，不需要重新烧程序。
 *   （也能用串口助手 / 手机串口 App 发 ASCII '1'~'4' / '0' 来切，方便脱屏调试）
 *
 *   ⚠️ 打开前必须先在 CubeMX 里加 UART5：
 *        PC12 = UART5_TX、PD2 = UART5_RX、115200 8N1、**开 UART5 全局中断**
 *      生成代码后再把下面 HMI_ENABLE 改成 1（否则 huart5 不存在，编译不过）。
 * ========================================================================== */

#define HMI_ENABLE          1        /* ★1 = 启用触摸屏菜单（先按上面在 CubeMX 里开 UART5） */
#define HMI_UART            (&huart5)
#define HMI_REPORT_STATUS   1        /* 1 = 切模式时往屏幕回一行状态（显示在文本组件 t0 上） */
#define HMI_DEBUG_LINE      0        /* ★0 = 不往屏幕 t0/t1 写调试行（t0 归屏幕工程用，别再抢） */        /* ★1 = 每 500ms 把 yaw/航向误差/四轮转速写到屏幕 t0 上
                                         （蓝牙在电机一转就掉线，屏幕才是可靠的调试出口）
                                         注意：会占用 t0 组件，不再显示 MODE 文字 */

/* ---- 运行期"当前测试模式"代号（屏幕按钮发过来的就是这个）---- */
#define RUNMODE_NONE        0u   /* 停：所有自动测试都不跑（业务代码自己控车） */
#define RUNMODE_M1          1u   /* ⑪ 测试例程1（1×1方框 + 平移） */
#define RUNMODE_STRAFE      2u   /* ⑩ 平移测试 */
#define RUNMODE_TURN        3u   /* ⑨ 转弯测试 */
#define RUNMODE_DIST        4u   /* ⑧ 里程测试 */


/* ==========================================================================
 * ⑬ IMU（YbImu 模块，走【串口 USART6 = PC6(TX)/PC7(RX)】）
 *
 *   接线（串口，当前方案）：
 *       模块 TX  → PC11 (UART4_RX)      模块 RX  → PC10 (UART4_TX)
 *       GND      → GND（必须共地）        VCC      → 3V3/5V
 *   启用：① CubeMX 里打开 **UART4**（Asynchronous，PC10/PC11，115200，NVIC 勾 UART4 global interrupt）
 *         ② 把上面 5.6 区的 IMU_ENABLE 改成 1、本区 IMU_IF_UART 保持 1 → 编译烧录
 *
 *   接线（老 I²C 方案，代码还留着）：SCL→PB6、SDA→PB7，把 IMU_IF_UART 改 0 即可切回
 *
 *   这个模块是【自带融合】的：直接输出欧拉角（yaw），不用我们自己积分陀螺
 *   → 走直线 / 转固定角度全靠它（配合 CHASSIS_HEADING_HOLD）。
 *
 *   一次性设置（都存模块 flash，只需做一次）：
 *     · 校准陀螺+加计：车【静止、水平】放好，调用一次 IMU_Calibrate()
 *     · 算法类型：默认已设 6 = 六轴（yaw 相对上电；车上推荐，避免电机磁场干扰）
 * ========================================================================== */

/* ---- 接口选择：串口 还是 I²C ---- */
#define IMU_IF_UART                 1          /* ★1 = 走串口 UART4（当前方案，推荐）；0 = 走 I²C */
#if IMU_IF_UART
#define IMU_UART                    (&huart6)  /* ★串口句柄：**USART6 = PC6(TX) / PC7(RX)**
                                                  接法：模块 TX→PC7、模块 RX→PC6、GND↔GND、VCC→3V3
                                                  （老方案 UART4 = PC10/PC11 的代码还在，
                                                    要切回去就把这里改成 &huart4） */
#define IMU_UART_PROBE_MS           300        /* 上电握手：等模块回帧最多这么久（收不到 → 状态=2 离线） */
#define IMU_TX_TIMEOUT_MS           20         /* 发一帧最多等这么久（115200 下发 7 字节 ≈ 0.6ms） */
#define IMU_USE_ACTIVE_REPORT       1        /* ★1 = 模块主动上报：主循环被底盘串口阻塞时也不断帧（原来 0=轮询，一忙就断） */
#define IMU_UART_STREAM_MS          200      /* 主动上报模式下超这么久没帧才补一次请求 */
#define IMU_UART_TIMEOUT_MS         50         /* 超过这么久没收到【有效帧】才算异常
                                                  （抗抖动：偶尔丢一两帧不算断线，ch3 不会 0/1 闪） */
#define IMU_REQ_TIMEOUT_MS          40         /* 应答式轮询：发了请求这么久还没回帧 → 当丢了，重发 */
#define IMU_REPORT_RATE_HZ          100      /* 主动上报频率（10~100Hz）：越高越跟手，100 够用 */
#endif

#define IMU_I2C                     (&hi2c1)   /* I2C 句柄（IMU_IF_UART = 0 时才用） */
#define IMU_UPDATE_ENABLE           1          /* 1 = 主循环里读 IMU（关掉就不读，省 I2C 时间） */
#define IMU_ALGO_TYPE               6          /* 6 = 六轴（推荐）/ 9 = 九轴（用磁力计） */
#define IMU_CALIB_WAIT_MS           7000       /* IMU_Calibrate() 最多等这么久（官方例程 7 秒） */
#define IMU_USE_FUSED_YAW           1          /* 1 = 底盘航向直接用模块融合值（推荐）
                                                  0 = 用我们自己积分陀螺 + 轮式互补（旧方案） */
/* ---- ★静止零漂冻结（推荐 1）----
   车"确定没在动"（没有转动/平移/速度指令/急停）时，yaw 本不该变化：
   变化率小于 CHASSIS_STILL_DPS 就判为传感器零漂，直接把 yaw 冻结住不跟。
   效果：静止零漂 = 0，转 90° 的基准不会被漂移带歪；
   有人在推车（变化快）时仍会正常跟随，不会误冻结。 */
#define CHASSIS_STILL_YAW_HOLD      0
#define CHASSIS_STILL_DPS           0.6f       /* 判定门限（度/秒）：小于它才算"静止漂移" */

/* ---- ★yaw 漂移抵消（六轴融合的 yaw 一定会漂；模块自带校准可能不生效）----
   原理：每次"航向归零"先静止采样 IMU_TRIM_MS 毫秒，测出当前漂移率，之后按时间线性扣掉。
   （实测本车模块约 60°/min = 陀螺零偏 1°/s；修完 ch0 静止时应基本不动）
   ⚠️ 采样期间车必须停稳 —— 所以调用点是"开机 / 屏幕切模式 / 脚本每条动作前"，
      不能每 10ms 都测（那会把车的真实转动当成漂移） */
#define IMU_TRIM_MS                 0          /* ★0 = 归零不阻塞采样（只归零）
                                                 原来是 600：在主循环里用 HAL_Delay 阻塞 600ms，
                                                 一旦串口中断风暴把 SysTick 饿死就永久卡死
                                                 （现象：发 Z 后呼吸灯不闪、板子假死）。
                                                 先改 0 保证稳定；漂移改用别的方式处理 */
#define IMU_TRIM_MAX_DPS            5.0f       /* 测出的速率超过它 = 车在动 → 本次不采信（按 0） */


/* ---- IMU 读取/稳定性（⑬ 区续）---- */
#define IMU_I2C_SPEED_HZ            100000     /* I²C 速率：100kHz 更容错（弱上拉/长线/有电机干扰时先用它）
                                                  确认稳定后可以回到 400000 */
#define IMU_PERIOD_MS               10         /* 多久读一次 IMU（ms）：10 = 100Hz，够姿态用；
                                                  调小更灵敏但更占 I²C，调大波形更平滑 */
#define IMU_SET_ALGO_ON_BOOT        1          /* ★1 = 每次上电写一次融合算法类型（IMU_ALGO_TYPE=6 六轴）
                                                  为什么要写：这个设置是存模块 flash 的，但**没人写过**时
                                                  模块保持出厂九轴（用磁力计）→ 被步进电机磁场带跑
                                                  （实测：装车上 -74 度/分，拿远 -4.9 度/分）。
                                                  写成 6 之后磁干扰就彻底不影响了。
                                                  ★确认 6 轴生效并稳定后，可以改回 0（模块已存住）。 */
#define IMU_ERR_LIMIT               5          /* 连续读失败几次就重初始化（串口=重挂接收，I²C=重初始化 I²C） */
#define IMU_GYRO_POLL_DIV           10         /* 每 N 次轮询里夹一次"原始数据帧"（0x04）→ 拿到陀螺 Z
                                                   静止时 |陀螺Z| ≈ 零偏，×60 = 每分钟漂多少度（ch5 就发它）
                                                   0 = 不夹（只轮询姿态帧，省带宽）；10 = 约 10Hz 更新陀螺 */
#define IMU_CALIB_ON_BOOT           0          /* ★1 = 每次上电自动校准一次（车必须静止、水平放置！会阻塞最多 7 秒）
                                                  校准结果存模块 flash，正常只需做一次：
                                                  第一次装好车时改 1 → 烧录 → 上电等 7 秒 → 再改回 0
                                                  想确认校准成功：VOFA ch4（最近功能码）会变成 129 = 0x81 */
#define IMU_VOFA_STATUS_CH          0          /* 0 = 只发姿态 4 路（yaw/roll/pitch/wz）
                                                  1 = 发 17 路诊断（蓝牙 9600 下太重，调底盘别开） */

/* ==========================================================================
 * ⑭ 升降臂（两个直线轴，闭环步进）
 *
 *   轴① = 螺杆（丝杆）：**自锁**，断电不动、手也推不动 → 定标要让电机带着走（VOFA 拖条）
 *   轴② = 齿条（齿轮齿条）：**不自锁**，失能/断电时手推得动 → 直接手推定标
 *   直线换算：螺杆 位移 = 导程(mm/圈) × 圈数 ；齿条 位移 = π×分度圆直径 × 圈数
 *
 *   位置：闭环步进是【单圈绝对】—— 通电就知道这一圈内的角度，但**多圈行程断电就丢**
 *         ⇒ 每次上电都要重新定标/回零（当前阶段就是手动定标）
 *
 *   ★定标阶段（现在）：只看编码器 + 收拖条值，**不给力**
 *      · 齿条：失能 → 手推 → 看 VOFA ch1
 *      · 螺杆：自锁推不动 → 拖条让它动（把 ARM_AXIS1_ALLOW_TORQUE 改 1）
 *
 *   两条通信路，先选一条（改 ARM_IFACE_UART）：
 *      0 = 串口：插板上 5/6 号闭环步进口（和底盘同一款 PD42S1 + 现成驱动，立刻能读编码器）
 *      1 = CAN ：和 GM6020 共用 CAN1（1 Mbps）→ 按 CANopen 读 0x6064 实际位置
 *                 （若驱动器不是 CANopen，先把 ARM_PROBE_ENABLE 开 1 跑嗅探，把结果告诉我）
 * ========================================================================== */
#define ARM_ENABLE                  1        /* 1 = 编译升降臂模块 */
#define ARM_IFACE_UART              1        /* ★★ 2026-10-21 改成 1：臂的两台步进电机（螺杆=地址5、
                                                 齿条=地址6）改走【和底盘一样的串口总线】（同一个 7 口 hub）。
                                                 CAN 只留给 GM6020 旋转电机。 */
#define ARM_AXIS_NUM                2        /* 两个步进轴（旋转用的 GM6020 走 CAN，不算在这里） */

/* ---- 轴 ↔ 电机 对应 ---- */
#define ARM_AXIS1_IDX               4        /* 轴①（螺杆）= 步进表第 5 台 = STEPPER_MOTOR_5 */
#define ARM_AXIS2_IDX               5        /* 轴②（齿条）= 步进表第 6 台 = STEPPER_MOTOR_6 */
#define ARM_NODE_1                  5        /* 轴①（螺杆）的**从机地址**：5 */
#define ARM_NODE_2                  6        /* 轴②（齿条）的**从机地址**：6 */
/* 走 CAN 时用的扩展帧 ID（手册 6.2：ID 必须符合 0x100x 格式；0x1001/0x100A/0x100F 都合法） */
#define ARM_CAN_TX_ID               0x1000u
/* ★两条路【同时发】（推荐 1）：
   手册里 CAN 版和 TTL 版是【同一对引脚】的不同焊装版本（TX/485_A/CAN_H 是三合一的脚），
   光看驱动器菜单分不出是哪个版本 → 干脆串口（板上 5/6 口）和 CAN 各发一遍，谁回应就用谁。
   CAN 的发送格式还会自动【交替】：一次带串口外壳 C5…5C、一次不带
   （手册里"读位置"应答是 10 字节 > CAN 单帧 8 字节，去掉外壳正好 8 字节 → 两种都可能）。
   收到的一侧带不带外壳都自动识别。
   0 = 只发一条（按 ARM_IFACE_UART 决定走哪条） */
#define ARM_IFACE_CAN_TOO           0        /* ★0 = 只走串口（臂的驱动器已经换成串口版了；
                                                 同时发 CAN 只会白占总线）。CAN 只给 GM6020 用 */
/* ★VOFA 显示哪套（6 路，通道数不变就不用在 VOFA+ 里重建控件）
     0 = 定标用：轴①位置/有效、轴②位置/有效、滑块①、滑块②
     1 = CAN 诊断用：TEC/hits/档位/扩展帧计数/回帧ID/首字节（查 CAN 时改 1）
   ⚠️ 改完要重新编译烧录 */
#define ARM_VOFA_CAN_DIAG           0
/* ★地址扫描（诊断用，默认 1）：轮流问地址 1~7 的"读位置"，把有应答的记进位掩码。
   ch0/ch1 一直是 0 时最有用 —— 它是判断"总线上到底哪个地址活着"的唯一硬证据。
   看结果：Keil 的 Watch 窗口加 `ARM_ScanMask()`：
       bit0 = 地址1 有应答, bit1 = 地址2 … bit6 = 地址7
       mask = 0  → 这条总线上没有任何驱动器应答（接线/供电/驱动器的通讯模式不对）
       mask 里有哪几位 → 那几位就是实际在用的地址（对应 STEPPER_MOTOR_x 表） */
#define ARM_ADDR_SCAN               1

/* ---- ★给不给力（定标阶段两个都先 0；只有"螺杆要动"时把轴①改 1）---- */
#define ARM_AXIS1_ALLOW_TORQUE      1        /* 轴①（螺杆）：1 = 允许按拖条的目标动 */
#define ARM_AXIS2_ALLOW_TORQUE      0        /* 轴②（齿条）：定标阶段保持 0（手推） */

/* ---- VOFA 拖条（控件）输入 ---- */
#define ARM_SLIDER_IS_DEG           0        /* 拖条值当什么：0 = 脉冲（51200 = 一圈）；1 = 角度 0~360 */
#define ARM_SLIDER_CMD_MS           50       /* 最快多久下发一次目标（拖条会不停发，限速） */

/* ---- 读取 / 显示 ---- */
#define ARM_POLL_MS                 50       /* 读一次编码器位置的间隔（两条路都是发"读位置 0x2A"指令） */
#define ARM_VOFA_SHOW_SLIDER        1        /* 0 = VOFA 只发 2 路（两个编码器）★你要的
                                                1 = 多发一路"拖条收到的值"（共 3 路，调拖条时临时开） */

/* ---- CAN 嗅探（走 CAN 又不知道协议时用；正常读位置时保持 0） ---- */
#define ARM_PROBE_ENABLE            0        /* 1 = CAN 嗅探（走 CAN 又不知道协议时才开）
                                                ★走串口读位置时必须 0 —— 开了 VOFA 会变成 12 路嗅探数据，
                                                  就看不到两个编码器了 */
#define ARM_PROBE_SEND_CANOPEN      1        /* 嗅探时主动发 CANopen 探针（NMT+SDO 读 0x1000） */

/* ---- CAN 引脚 GPIO 自检（判断 PA11/PA12 到底是不是好的）----
   1 = 把 PA11、PA12 强制配成普通推挽输出，每 250ms 反转（两根脚反相）
       → 看 VOFA ch12/ch13（软件示波器采的"低电平占比"）：
          两路都 ≈0.5 → 两根引脚都活着（问题在模块/线）
          ch12≈0      → PA12 引脚死了
          ch13≈0      → PA11 引脚死了
          两根都 0     → GPIOA 口被打坏 / 代码没跑到
   ⚠️ 这个模式下 CAN 完全不能用（引脚被占用）。测完必须改回 0！*/
#define CAN_BAUD_500K               0
#define CAN_PIN_GPIO_TEST           0

/* ---- CAN1 用哪组引脚（F429 的 CAN1 有三组复用脚）----
     0 = PA11(RX) / PA12(TX)   ← 原来的接法
     1 = PB8(RX)  / PB9(TX)    ← 备用接法（怀疑 PA11/PA12 被打坏时用这个）
   （第三组 PD0/PD1 不能用：PD0 是心跳灯）
   ⚠️ 改了这里之后**接线要跟着换**：收发器 RXD ← RX 脚（PB8）、TXD → TX 脚（PB9）*/
#define CAN_ON_BACKUP_PINS          0

/* ---- CAN 回环自检（只在排查 CAN 硬件时临时打开）----
   1 = CAN 进入内部回环：自己发的帧自己收、自己 ACK，总线上其它节点收不到 → 电机不会动
   用途：判断"发不出去"是 MCU/配置的问题，还是收发器/线/对端的问题
        · 回环下 VOFA ch5（扩展帧计数）在涨、ch6（TEC）保持 0 → MCU 侧没问题
        · 回环下也不涨 → MCU 侧配置问题（我这边查）
   查完必须改回 0！*/
#define CAN_LOOPBACK_TEST           0

/* ---- CAN 位置指令（0xF2）用的加速度 / 速度：定标时慢一点，别猛冲 ---- */
#define ARM_CAN_ACC                 60       /* 加速度档位 0~255（越小越软） */
#define ARM_CAN_RPM                 120      /* 位置模式巡航转速 RPM（螺杆定标建议 60~150） */

/* ---- CAN 速率自动扫描（查"驱动器到底挂在哪档速率"用）----
   ★为什么要它：CAN 收不到 ACK 时，最常见原因就是两边速率不一样（手册 CAN 版出厂 500K，
     屏幕上改的 1M 常常要【断电重上电】才生效）。与其一版一版烧，不如让固件自己轮：
       每 ARM_CAN_SCAN_MS 换一档：1M → 500K → 250K → 125K → 100K → 回到 1M
     VOFA 看两路：
       · hits 掩码（bit0=1M bit1=500K bit2=250K bit3=125K bit4=100K，收到过帧就置位）
       · 当前档位索引（0..4）
     查到结果后把它改回 0，并按结果把速率定死。
   ⚠️ 扫描期间 GM6020（1M）的帧可能被当乱码 → 一定要先把 GM6020 的 CAN 线拔掉/断电。 */
#define ARM_CAN_BAUD_SCAN           0        /* 0 = 不扫速率，固定用 MX_CAN1 的 1Mbps
                                                （CAN 版驱动器的速率已通过 USB 读出来 = 1000 Kbps） */
#define ARM_CAN_SCAN_MS             2500u    /* 每档停留时间 ms（≥2000，给驱动器留应答时间） */

/* ---- CAN 地址全扫（诊断"驱动器到底回不回复"，查完改回 0）----
   1 = 读位置指令的【从机地址】在 0~10 之间连续轮（驱动器菜单⑦ 的 ID 范围就是 0~10），
       配合 4 种载荷格式（带壳/不带壳 × DLC 实际/补 0）一起轮。
   用途：如果驱动器收得到、只是地址或格式对不上，轮一圈（11×4=44 组合）就会有一帧回复。
   判读：VOFA 上看"扩展帧计数"和"最后一个扩展帧 ID/数据"那几路。 */
#define ARM_DIAG_SWEEP              1

/* ==========================================================================
 * ⑮ 蓝牙串口（HC-05，走【UART4 = PA0(TX) / PA1(RX)】，也就是板上「串口4」那对脚）—— 无线 VOFA
 *
 *   两个模式（本质：同一个 MCU 的不同串口，接不同模块）
 *     模式1 有线：串口2（PA2/PA3）接 USB-TTL → 电脑 VOFA+（照旧，一直有效）
 *     模式2 蓝牙：串口4（PA0/PA1）接 HC-05 → 电脑/手机蓝牙看波形
 *   两路可以【同时】发（VOFA_WIRED_EN / VOFA_BT_EN 各自开关），默认只开有线。
 *
 *   ⚠️ UART4 原来是老 IMU 走的 PC10/PC11；IMU 搬到 USART6（PC6/PC7）后这路空出来了。
 *      现在按"板上串口4 的排针是 PA0/PA1"把 UART4 重映射到 PA0/PA1（AF8）。
 *      如果以后要改回 PC10/PC11：`GC_Car.ioc` 里 UART4 的两脚改回 PC10/PC11 +
 *      `usart.c` 的 UART4 MspInit 改回 GPIOC/10/11，这一区的宏不用动。
 *
 *   用蓝牙的步骤：
 *     ① 接线：PA0→模块RXD、PA1←模块TXD、5V→VCC、GND→GND（详见 bt.h 顶部）
 *     ② 先用 USB-TTL 把模块波特率改成 115200：AT+BAUD=8（出厂 9600；改不了就填 BT_BAUD）
 *     ③ 电脑蓝牙配对 HC-05 → 会多出一个"标准串行"COM 口 → VOFA+ 选那个口
 *     ④ 下面 VOFA_BT_EN 改 1，重烧
 *   ⚠️ 蓝牙 SPP 实际吞吐比 USB-TTL 差很多：蓝牙那路的发送频率要单独降到 VOFA_BT_HZ
 *      （20~30Hz；100Hz 会丢帧）。通道数越少越好（IMU 5 路 24 字节最稳）。
 *   ⚠️ 波特率对不上 = 蓝牙那头全是乱码/没数据，不是没连上。
 * ========================================================================== */

#define BT_ENABLE                   1        /* 1 = 编译蓝牙模块并把串口挂上接收；0 = 整个模块变空函数
                                                ★VOFA_UART=(&huart4) 时必须为 1：BT_Init 才会把
                                                  UART4 重配成 BT_BAUD（9600），否则波特率不匹配 = 乱码 */
#define BT_UART                     (&huart4)  /* ★蓝牙挂哪个串口：UART4 = PA0/PA1 */

#define BT_BAUD                     9600u    /* 模块当前的通信波特率（= 没升级成功时的回落值）：
                                                HC-05 "实惠版"出厂就是 9600；发过 AT+BAUD=8 就是 115200u */
#define BT_BAUD_HI                  115200u  /* AT 升级成功后用的波特率（BT_AT_UPGRADE=1 时） */
#define BT_AT_UPGRADE               0        /* ★0 = 不做 AT 自动升速（那条路有坑：模块已升到 115200 后，
                                                若这次没进 AT 模式就会回落 9600，链路直接死）
                                                现在按 9600 跑；主口已改非阻塞发送，20Hz 足够调 PID。
                                                要升 115200：单独烧一个只发 AT 的版本升一次，
                                                再把 BT_BAUD 改成 115200u 定死 */

/* ---- 链路找波特率（查"模块到底在哪个波特率"，查完必须改回 0）----
   1 = 每 1.5s 轮一档（9600/115200/38400/57600/19200/4800），每 120ms 发一行 "PING <波特率>"
       配合 VOFA_OUTPUT_ENABLE=0 用（不要二进制帧混进来）
   上位机抓 12 秒，能读出人类可读的 "PING xxxx" 的那一档 = 模块的波特率 */
#define BT_LINK_SCAN                0

/* ---- CAN 诊断走【纯文本行】（不依赖 VOFA 通道表，串口助手/网页直接读）----
   1 = 每 300ms 往蓝牙打印一行：
        DIAG BT=<串口波特率> CAN=<档位> LISTEN=<0/1> TEC=.. REC=.. LEC=.. EXT=.. ID=.. D0=..
       同时每 4 秒自动轮换 UART4 波特率（9600/115200/38400/57600）——
       哪一档能读出完整的一行，就说明蓝牙模块就是那一档波特率（这一步顺便把链路波特率定死）
   查完把本宏改回 0 */
#define ARM_TEXT_DIAG               0        /* 0 = 不打文本诊断（现在要波形，文本会和二进制帧混在一起） */
#define BT_BAUD_CYCLE               0        /* 配合 ARM_TEXT_DIAG：自动轮 UART4 波特率 */
#define ARM_CAN_LISTEN_MS           1200u    /* 每个速率档位里"只收不发"的时长 ms */
#define BT_SET_BAUD                 1        /* 1 = 启动时把 UART4 重配成 BT_BAUD（模块不是 115200 时用）
                                                0 = 用 CubeMX 生成的 115200（模块已改成 115200 就填 0） */

/* ---- VOFA 往哪一路发（两个可以同时为 1）----
   ★VOFA_UART 已经是 (&huart4) 时（蓝牙当主口）：VOFA_BT_EN 填 0，
     否则同一个口会发两份、波形发虚 */
#define VOFA_WIRED_EN               1        /* 主口是否发送（1 = 发；主口既是蓝牙也填 1） */
#define VOFA_BT_EN                  0        /* ★第二路（蓝牙镜像）：只有"主口=串口2"时才填 1 */
#define VOFA_BT_HZ                  20       /* 蓝牙那路的发送频率（Hz）：20~30 稳，别照抄 100 */

/* ---- 链路自检（查"到底通没通"用；正常跑车时 BT_ALIVE_MS 保持 0）----
   ★心跳是【文本】，混在 JustFloat 波形里会让波形乱 —— 只用于确认链路，查完改回 0 */
#define BT_ALIVE_MS                 0        /* >0 = 每 N ms 往蓝牙发一行文本心跳（0 = 关） */
#define BT_AUTOBAUD                 0        /* 1 = 心跳期间自动轮换 9600/19200/38400/57600/115200
                                                （不知道模块波特率时开它：串口助手固定在 9600，
                                                  哪一档能读出文字，那行里的 baud= 就是模块的波特率） */
#define BT_AUTOBAUD_MS              3000     /* 每档停多久 */

/* ---- 蓝牙收（手机/上位机发给板子）---- */
#define BT_CMD_ENABLE               1        /* 1 = 收到的 '0'~'4'/'s' 直接当模式命令（和触摸屏同一套）
                                                手机蓝牙串口 App 发 1/2/3/4 = 例程1/平移/转弯/里程，0 = 停 */
#define BT_ECHO                     0        /* 1 = 把收到的整行原样回显（确认链路用；会让波形里混文本） */
#define BT_RX_BUF_SIZE              128      /* 接收环形缓冲（字节） */
#define BT_LINE_MAX                 96       /* 读一行的缓冲（BT_ReadLine 用） */
#define BT_TXT_MAX                  160      /* BT_Printf 的单次最大长度 */

/* ==========================================================================
 * ⑯ 调参命令通道（PC 在线整定 / 命令行调参）—— 详见 tune.h
 *
 *   为什么要它：三个 PID 的参数原来是编译期宏，改一个数就要重编译+重烧（约 1 分钟）；
 *   有了这条通道，PC 可以在线上改参数、触发小动作、收遥测，一轮试验只要几秒 ——
 *   自动整定（继电反馈 / Optuna）才有意义。
 *
 *   ⚠️ 走有线（TUNE_ON_BT=0）时**必须补一根线**：USB-TTL 的 TX → 板子 PA3。
 *      现在只接了 PA2（板→PC）单向，所以 PC 只能看不能发。
 *   ⚠️ 走蓝牙（TUNE_ON_BT=1）：先把模块 AT+BAUD=8 升到 115200（同时 BT_BAUD 改 115200u、
 *      BT_SET_BAUD 改 0），否则 960B/s 连 20Hz 遥测都喂不饱、判据不可信。
 * ========================================================================== */

#define TUNE_ENABLE                 1        /* 1 = 编译调参通道；0 = 整个模块变空函数 */
#define TUNE_ON_BT                  1        /* ★1 = 调参命令走蓝牙（现在是蓝牙链路，命令和回执都走这里）
                                                0 = 走有线串口2(PA2/PA3) */
#define TUNE_UART                   (TUNE_ON_BT ? (&huart4) : (&huart2))

#define TUNE_LINE_MAX               48       /* 单行命令最大长度 */
#define TUNE_TXT_MAX                160      /* 一行遥测最长 */
#define TUNE_TELEM_HZ               5        /* 文本遥测频率：蓝牙 9600 下一行约 90 字符，
                                                 5Hz = 450B/s 才稳；10Hz 会丢字节（实测过） */
#define TUNE_TELEM_ON_BOOT          1        /* ★1 = 开机就默认开文本遥测（并关掉二进制波形）
                                                 好处：板子欠压复位后不会"回到无数据"状态，
                                                       上位机永远有数据
                                                 比赛跑流程时可改 0（省串口带宽） */

#define TUNE_CRUISE_SPEED           250.0f   /* G 动作的巡航速度 mm/s（小动作别太快） */
#define TUNE_GO_TOL_MM              20.0f    /* 直线到位容差（轮式里程计判，打滑会偏） */
#define TUNE_STRAFE_TOL_MM          15.0f    /* 平移到位容差 */
#define TUNE_TURN_TOL_DEG           2.0f     /* 转角到位容差（IMU 判，是真值） */
#define TUNE_ACT_TIMEOUT_MS         8000     /* 任何一条动作的兜底超时：跑不到也自己停 */

/* ==========================================================================
 * ⑰ 夹爪 = ZP 总线舵机（#IDPxxxxTtttt!）—— 详见 servo.h
 *   ★已废弃 PWM S1/TIM3 与 MG370 直流夹爪。
 *   UART 脚位 TBD：暂用 USART2（PA2/PA3）；与有线 VOFA 冲突时改 BUS_SERVO_UART。
 *   不可占用 USART3（步进）/ UART7（视觉）。
 * ========================================================================== */

#define SERVO_ENABLE                1
#define BUS_SERVO_UART              (&huart2)
#define BUS_SERVO_ID                0
#define BUS_SERVO_OPEN_P            2480     /* ★张开 P（2026-11-06 网页定标） */
#define BUS_SERVO_CLOSE_P           1950     /* ★合紧 P（2026-11-05 网页定标） */
#define BUS_SERVO_INVERT            0
#define BUS_SERVO_MOVE_MS           1000
#define BUS_SERVO_SETTLE_EXTRA_MS   200
#define BUS_SERVO_P_MIN             500
#define BUS_SERVO_P_MAX             4000     /* 联调放宽；ZP 常见标称 500~2500，以舵机实收为准 */

/* ==========================================================================
 * ⑱ ★GM6020 旋转轴定标值（2026-10-21 实测，配合绕线保护）
 *
 *   实测角度（越往后面的盘角度越小）：
 *      初始位 344.5°  →  第一个盘 188.8°  →  第二个盘 162.2°  →  第三个盘 135.2°
 *
 *   ⚠️ 绕线约束：从第三个盘返回初始位，必须走【角度递增】方向
 *      （135.2 → 162.2 → 188.8 → 344.5，+209.3°），
 *      不能走最短路径（-150.7°，会穿过 0° 把线绕死）。
 *      → 用 GM6020_GoHome() 或命令 "Y344.5 1"
 * ========================================================================== */
#define ROT_HOME_DEG        344.5f      /* 初始位（收臂/待机） */
#define ROT_DISC1_DEG       188.8f      /* 第一个盘 */
#define ROT_DISC2_DEG       162.2f      /* 第二个盘 */
#define ROT_DISC3_DEG       135.2f      /* 第三个盘 */
#define ROT_TRAVEL_DEG      250.0f      /* ★行进时云台停这里（在安全区间内，不会绕线） */
#define ROT_SAFE_MIN_DEG    130.0f      /* 安全区间下限（低于它会开始绕线，仅作检查用） */
#define ROT_SAFE_MAX_DEG    350.0f      /* 安全区间上限 */
/* ==========================================================================
 * ⑲ ★视觉整合（对齐 成品/视觉 锁定契约，非手册 4.0）
 *
 *   收：B3 B3 B1/B2 … 5B（物料/圆环）
 *   发：FF cmd FE — 01 全色 / 02 圆环 / 03~08 单色
 *   到位：主板自判 dx∈[DX_MIN,DX_MAX] 且 dy∈[DY_MIN,DY_MAX]
 *         （相对相机中心 VISION_CAM_CX/CY；夹爪用此窗，不是上位机准星）
 * ========================================================================== */
#define VISION_ENABLE               1
#define VISION_UART                 (&huart7)
#define VISION_UART_INSTANCE        UART7
#define VISION_MON_UART             (&huart2)   /* 与 BUS_SERVO 暂共用 USART2 时勿同时占线 */
#define VISION_OFFLINE_MS           300
#define VISION_CAM_CX               160
#define VISION_CAM_CY               120
#define VISION_ALIGN_DEAD_PX        8           /* 云台跟瞄停转死区（对称）；夹爪到位见下 */
#define VISION_ARRIVE_DX_MIN      (-60)         /* 物料到位窗：dx = x-cx */
#define VISION_ARRIVE_DX_MAX        60
#define VISION_ARRIVE_DY_MIN      (-35)         /* dy = y-cy：上负下正 */
#define VISION_ARRIVE_DY_MAX        90

/* 1 = 把视觉译码结果显示/打印出来（联调用）；比赛跑流程时可关 */
#define VISION_LINK_TEST            1

/* ---- 底盘微调跟画面（vf1/0；死区 = 合爪到位窗 ARRIVE_*）----
   C：P 控制 + 最小速度；进窗停车。云台跟已停用。
   轴：dx→vy，dy→vx；符号跟反了只改 DX_DIR / DY_DIR。 */
#define VISION_CHASSIS_FOLLOW       1
#define VISION_FOLLOW_KX            2.5f    /* vy = DIR * KX * dx（mm/s / px） */
#define VISION_FOLLOW_KY            2.5f    /* vx = DIR * KY * dy */
#define VISION_FOLLOW_VMAX          120.0f  /* 单轴速度上限 mm/s（联调宜小） */
#define VISION_FOLLOW_VMIN           60.0f  /* 单轴最小速度（出窗才给） */
#define VISION_FOLLOW_PERIOD_MS      40u
#define VISION_FOLLOW_DX_DIR       -1.0f    /* dx>0（偏右）→ vy 符号 */
#define VISION_FOLLOW_DY_DIR       +1.0f    /* dy>0（偏下）→ vx 符号；相对原默认已取反 */

/* ---- 云台跟画面（已停用，改底盘微调；宏保留防旧工程引用）---- */
#define VISION_GIMBAL_FOLLOW        0
#define VISION_GIMBAL_DIR          -1.0f
#define VISION_GIMBAL_KP_DEG_PER_PX 0.12f
#define VISION_GIMBAL_MAX_STEP_DEG  3.0f
#define VISION_GIMBAL_MIN_STEP_DEG  0.3f
#define VISION_GIMBAL_PERIOD_MS     40u
#define VISION_GIMBAL_CLAMP_SAFE    1
#define VISION_GIMBAL_CLOSE_SCALE   0.4f
#define VISION_GIMBAL_TRAVEL_DEG    120.0f
/* ==========================================================================
 * ⑳ ★比赛路线执行（来自视觉屏幕修正版）
 *
 *   3×3 路口，屏幕上报障碍 → RoutePlan_Compute() 算字母序列 →
 *   主循环按字母走：F 前进 / L 左转 / R 右转 / U 掉头
 *                   C 扫码 / M 原料 / P 放下 / G 抓回 / B 暂存
 * ========================================================================== */
#define RUNMODE_ROUTE               5u      /* 启动键：按确认帧里的路线走 */
#define ROUTE_SEGMENT_MM            850.0f  /* 一段路的长度（现场按场地改） */
#define ROUTE_SPEED_MMPS            200.0f  /* 前进速度 mm/s */
#define ROUTE_TURN_WZ               60.0f   /* 转弯角速度 度/s */
#define ROUTE_TURN_TOL_DEG          5.0f    /* IMU 在线时，离目标角这个范围内就停 */
#define ROUTE_SCAN_MS               8000u   /* 扫码任务超时 */
#define ROUTE_VISION_MS             8000u   /* 等一次视觉到位/超时 */
#define ROUTE_ACT_MS                2000u   /* 到位后停留（定高，不升降） */
#define MATCH_FIXED_Z               1       /* 1 = 比赛中不跑升降任务，保持发车前高度 */
#define ROUTE_GAP_MS                200u    /* 两个动作之间的停顿；0 = 不停 */

#define ROUTE_FWD_MS                ((uint32_t)((ROUTE_SEGMENT_MM / ROUTE_SPEED_MMPS) * 1000.0f))
#define ROUTE_TURN90_MS             ((uint32_t)((90.0f  / ROUTE_TURN_WZ) * 1000.0f))
#define ROUTE_TURN180_MS            ((uint32_t)((180.0f / ROUTE_TURN_WZ) * 1000.0f))

/* ---- 夹爪 ---- */
#define GRIP_POWER_ON_OPEN          1       /* 1 = 上电回中位后立刻张开（发车前就是张开） */
#define GRIP_SETTLE_MS              1200u    /* 一次开/合之后的等待（脉宽 600→2000 约 0.93 秒） */

/* ---- 云台转到盘位 ---- */
#define ROT_MOVE_MS                 4000u   /* 转到目标角的最长等待，到点或超时都继续 */
/* ==========================================================================
 * ㉑ ★升降（同步带单轴）行程标定（脉冲/毫米）
 *   驱动器：51200 脉冲 = 一圈。导程按同步带实测改（下列为占位）。
 *   旧齿条轴已拆除，ARM_RACK_* 仅兼容旧 tune，勿在工位表使用 'R'。
 * ========================================================================== */
#define PULSE_PER_REV           51200.0f
#define ARM_SCREW_LEAD_MM       1.0f        /* ★同步带：电机一圈对应行程 mm（实测改） */
#define ARM_RACK_LEAD_MM        1.0f        /* 废弃 */

#define ARM_SCREW_PULSE_PER_MM  (PULSE_PER_REV / ARM_SCREW_LEAD_MM)
#define ARM_RACK_PULSE_PER_MM   (PULSE_PER_REV / ARM_RACK_LEAD_MM)

/* 升降方向与软限位（相对 VPOS；先把机构降到最低 → N1 置零，再量总行程改 MAX）
   DIR：网页「↑」发 +mm；跟机械反了改成 -1 */
#define ARM_LIFT_DIR           (-1.0f)
#define ARM_LIFT_SOFT_LIMIT     1
#define ARM_LIFT_MM_MIN         0.0f        /* 置零后的下限（通常 0=最低） */
#define ARM_LIFT_MM_MAX       150.0f        /* ★总行程 mm 占位：按实测改 */
/* ==========================================================================
 * ㉒ ★工位动作表（升降 S / 6020 T / 夹爪 G / 视觉 V）—— 无伸缩 R
 *      {'S', mm}   升降：正=升，负=降
 *      {'T', deg}  6020
 *      {'G', 1/0}  夹爪开/合（总线舵机）
 *      {'V', 0}    等视觉到位（须先按工位发模式，见 route_run）
 *      {'W', ms}   等待
 * ========================================================================== */
#define STATION_USE_TABLE           1

/* ---- 取料类（M 原料 / G 抓回）---- */
#define ST_PICK_STEPS   { {'V', 0.0f}, {'S', -15.0f}, {'G', 0.0f}, \
                          {'S', 15.0f}, \
                          {'T', ROT_DISC1_DEG}, {'G', 1.0f}, {'T', ROT_HOME_DEG} }

/* ---- 放置类（P 放下 / B 暂存）---- */
#define ST_PLACE_STEPS  { {'T', ROT_DISC1_DEG}, {'G', 0.0f}, {'T', ROT_HOME_DEG}, \
                          {'V', 0.0f}, {'S', -15.0f}, {'G', 1.0f}, \
                          {'S', 15.0f} }

/* ==========================================================================
 * 扫码相机（UART8，与物料视觉 UART7 分离）
 *   收：$156+123+516+231#   回：$ACK#
 *   查询：vs（不消费）→ CODE,<task> 或 CODE,NONE；vs0 清持有
 *   收码后经 TUNE 主动推 CODE,...
 * ========================================================================== */
#define SCAN_ENABLE                 1
#define SCAN_UART                   (&huart8)
#define SCAN_UART_INSTANCE          UART8
#define SCAN_BAUD                   115200u

/* ---- 扫码工位（C）：只等 UART8 任务码，不动机构 ---- */
#define ST_SCAN_STEPS   { {'W', 300.0f} }

/* 每步余量：动作发完后额外等这么久才算稳（ms） */
#define ST_STEP_SETTLE_MS           300u

/* ==========================================================================
 * ㉓ ★手机蓝牙遥控（备用方案）
 *   用手机串口助手（Serial Bluetooth Terminal）发单个字母控制，松手自动停：
 *     w 前进   s 后退   a 左转   d 右转   q 左前   e 右前
 *     x 停     + 加速   - 减速
 *     j 夹爪开   k 夹爪合
 *     1/2/3 6020 到盘1/2/3   0 6020 回初始位
 *     u 升降升5mm   n 升降降5mm   （旧齿条 l/m 已拆除，勿用）
 *   超过 REMOTE_TIMEOUT_MS 没收到按键 → 自动停车（防跑飞）
 * ========================================================================== */
#define REMOTE_ENABLE               1
#define REMOTE_SPEED_MMPS           200.0f   /* 前进/后退速度 */
#define REMOTE_TURN_DPS             45.0f    /* 原地转向角速度 */
#define REMOTE_SPEED_MIN            80.0f    /* 减速下限 */
#define REMOTE_SPEED_MAX            350.0f   /* 加速上限 */
#define REMOTE_TIMEOUT_MS           1500u    /* 多久没按键就自动停 */

#endif /* __CONFIG_H__ */