/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "gm6020.h"
#include "arm.h"
#include "vofa.h"
#include "pd42s1.h"
#include "chassis.h"
#include "imu.h"
#include "hmi.h"
#include "vision.h"
#include "route_run.h"   /* 比赛路线执行（按字母跑） */     /* MaixCam 视觉（UART7 = PE7/PE8） */
#include "bt.h"          /* 蓝牙 HC-05（UART4 = PA0/PA1）*/
#include "tune.h"        /* 调参命令通道（有线串口2 / 蓝牙）*/
#include "servo.h"    /* 夹爪 = ZP 总线舵机 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 云台三位置循环测试：位置① → ② → ③ → ① …（角度/停顿都在 config.h 的 3.7 区）
   正式写业务时把 config.h 的 GIMBAL_TEST_ENABLE 改成 0 */
#define MAIN_GIMBAL_SELFTEST     GIMBAL_TEST_ENABLE
#define MAIN_SELFTEST_PERIOD_MS  GIMBAL_TEST_DWELL_MS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- CAN 引脚 GPIO 自检：把 PA11/PA12 当普通推挽输出，每 250ms 反转 ----
   配合 main_can_pin_sample() 的软件示波器：正常的引脚低电平占比会 ≈0.5 */
#if CAN_PIN_GPIO_TEST
static void main_can_pin_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin   = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

static void main_can_pin_gpio_toggle(void)
{
    static uint32_t t_tick;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - t_tick) < 250u) return;
    t_tick = now;

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
}
#endif

/* ---- CAN 引脚"软件示波器"：高频采样 PA12(CAN_TX)/PA11(CAN_RX) 的电平，算低电平占比 ----
   为什么要有它：回环测试证明不了"引脚到底有没有真的输出波形"（尤其怀疑引脚被打坏时）。
   CAN 空闲时两根脚都是高电平（隐性）；发帧时会出现大量低电平（显性位）→ 低电平占比明显 >0。
     · ch12 = PA12(TX) 低电平占比 → 有值说明 **MCU 引脚在输出**
     · ch13 = PA11(RX) 低电平占比 → 有值说明 **收发器把总线状态送回来了**（模块+线是通的）
   判读：TX 有值 + RX 没值 = 引脚好的但收发器/线不通；两个都 0 = PA12 引脚死了 */
static uint32_t s_can_tx_low, s_can_rx_low, s_can_samp;

float Main_GetCanTxLowRatio(void) { return (s_can_samp != 0u) ? ((float)s_can_tx_low / (float)s_can_samp) : 0.0f; }
float Main_GetCanRxLowRatio(void) { return (s_can_samp != 0u) ? ((float)s_can_rx_low / (float)s_can_samp) : 0.0f; }

static void main_can_pin_sample(void)
{
    uint32_t i;

    for (i = 0u; i < 64u; i++)
    {
        uint32_t idr = GPIOA->IDR;

        if ((idr & (1u << 12)) == 0u) s_can_tx_low++;
        if ((idr & (1u << 11)) == 0u) s_can_rx_low++;
        s_can_samp++;
    }

    /* 每 100 万次采样（约几秒）把统计对半衰减一次，保证能反映"最近"的状态 */
    if (s_can_samp > 1000000u)
    {
        s_can_tx_low >>= 1;
        s_can_rx_low >>= 1;
        s_can_samp   >>= 1;
    }
}

/* ---- 主循环周期诊断（ch8/ch9 用）---- */
static uint32_t s_loop_tick;        /* 上一轮循环的时刻 */
static uint32_t s_loop_ms_now;      /* 最近一轮耗时(ms) */
static uint32_t s_loop_ms_max;      /* 开机以来最慢的一轮(ms)：一直很大 = 有阻塞点 */

float Main_GetLoopMs(void)    { return (float)s_loop_ms_now; }
float Main_GetLoopMaxMs(void) { return (float)s_loop_ms_max; }

/* ---- 主循环【分段】耗时诊断：哪一段"最慢一次"花了多久（ms，1ms 分辨率）----
   用途：循环被拖慢时，一眼看出是底盘、IMU、步进、还是 VOFA 发送 */
#define MAIN_SECT_STEP     0
#define MAIN_SECT_IMU      1
#define MAIN_SECT_CHASSIS  2
#define MAIN_SECT_VOFA     3
#define MAIN_SECT_REST     4
#define MAIN_SECT_N        5

static uint32_t s_sect_ms[MAIN_SECT_N];

#define MAIN_SECT(i, t0)                                             \
    do {                                                             \
        uint32_t d_ = (uint32_t)(HAL_GetTick() - (t0));               \
        if (d_ > s_sect_ms[i]) s_sect_ms[i] = d_;                     \
    } while (0)

float Main_GetSectMs(uint8_t i) { return (i < MAIN_SECT_N) ? (float)s_sect_ms[i] : 0.0f; }

#if CHASSIS_ENABLE
/* ---- 当前测试模式（触摸屏菜单可运行期切换；上电默认由 config.h 的开关决定）----
   ⚠️ 如果这里默认落在某个测试上（例：MISSION1_TEST=1 时默认 RUNMODE_M1），
      那么【一上电/一复位车就自己开跑】——烧录前先把车架空或留够场地 */
static uint8_t s_run_mode =
#if CHASSIS_MISSION1_TEST
    RUNMODE_M1
#elif CHASSIS_STRAFE_TEST
    RUNMODE_STRAFE
#elif CHASSIS_TURN_TEST
    RUNMODE_TURN
#elif CHASSIS_DIST_TEST
    RUNMODE_DIST
#else
    RUNMODE_NONE
#endif
;
#endif /* CHASSIS_ENABLE：底盘关掉时 s_run_mode 没人用，别留着报 warning */
#if CHASSIS_ENABLE
/* ---- 里程测试（config.h ⑧ 区）：状态 + VOFA 数据源，函数实现在下面 USER CODE 4 ----
       （⑨ 转弯测试打开时，里程测试整体不编译 → 两个测试不会互相干扰） */
static uint8_t  s_dtest_phase     = 0u;    /* 0=前进 1=停 2=后退 3=停 4=完成 */
static uint8_t  s_dtest_started   = 0u;    /* 本段是否已经下发过速度指令 */
static uint32_t s_dtest_tick      = 0u;    /* 本段开始时刻（判"跑够了没"） */
static float    s_dtest_expect_mm = 0.0f;  /* 本段跑完时"累计路程"应该是多少（VOFA 参考线） */
#endif

#if CHASSIS_ENABLE
/* ---- 转弯测试（config.h ⑨ 区）：状态 + VOFA 数据源，函数实现在下面 USER CODE 4 ----
   相位：偶数=动作段（0左A1 2右A1 4左A2 6右A2），奇数=停顿段，8=完成 */
static uint8_t  s_ttest_phase     = 0u;
static uint8_t  s_ttest_started   = 0u;    /* 本段是否已经下发过角速度指令 */
static uint32_t s_ttest_tick      = 0u;    /* 本段开始时刻 */
static float    s_ttest_wz        = 0.0f;  /* 本段目标角速度 度/s（+左 −右） */
static float    s_ttest_angle     = 0.0f;  /* 本段目标转角（度，正值） */
#endif

#if CHASSIS_ENABLE
/* ---- 平移测试（config.h ⑩ 区）：状态 + VOFA 数据源，函数实现在下面 USER CODE 4 ----
   相位：0=左平移 1=停 2=右平移 3=停 4=完成 */
static uint8_t  s_stest_phase   = 0u;
static uint8_t  s_stest_started = 0u;      /* 本段是否已经下发过速度指令 */
static uint32_t s_stest_tick    = 0u;      /* 本段开始时刻 */
static float    s_stest_vy      = 0.0f;    /* 本段目标 vy（+左 −右），VOFA 用 */
#endif

#if CHASSIS_ENABLE
/* ---- 测试例程1（config.h 11 区）：状态 + VOFA 数据源，实现在下面 USER CODE 4 ---- */
static uint8_t  s_m1_idx     = 0u;      /* 当前第几步（0 起） */
static uint8_t  s_m1_sub     = 0u;      /* 0 = 执行本步，1 = 步间停顿 */
static uint8_t  s_m1_started = 0u;      /* 本步是否已经下发过指令 */
static uint32_t s_m1_tick    = 0u;      /* 本步开始时刻 */
static float    s_m1_mm0     = 0.0f;    /* 本步起点的里程计读数 */
static float    s_m1_deg0    = 0.0f;    /* 本步起点的航向 */
static float    s_m1_moved   = 0.0f;    /* 本步已走（mm 或 度），给 VOFA 看进度 */
static float    s_m1_target  = 0.0f;    /* 本步目标（mm 或 度） */
#endif

#if MAIN_GIMBAL_SELFTEST
static uint32_t s_test_tick   = 0;     /* 本阶段（转向目标 / 停顿）开始时刻 */
static uint8_t  s_test_angle  = 0;     /* 当前去哪号位置：0/1/2（对应 config.h 3.7 区三个角度） */
static uint8_t  s_test_phase  = 0;     /* 0 = 正在转向这个点；1 = 已到位/超时，正在停顿 */
static uint8_t  s_test_issued = 0;     /* 本次目标角度是否已经下发 */
#endif

#if STEPPER_ENABLE && STEPPER_SELFTEST && !CHASSIS_ENABLE
static uint32_t s_step_start_tick = 0; /* 步进自检起始时刻 */
static uint32_t s_step_phase_tick = 0; /* 步进自检换相节拍 */
static uint32_t s_step_poll_tick  = 0; /* 读转速/位置的节拍 */
static uint8_t  s_step_started    = 0; /* 0 = 还没开始（先做启动序列） */
static uint8_t  s_step_boot       = 0; /* 启动序列步骤：清状态→设模式→使能→读参数 */
static uint8_t  s_step_phase      = 0; /* 当前处于第几步 */
static uint8_t  s_step_poll_sel   = 0; /* 轮询：转速/位置/状态/电压/误差 轮流读 */
static uint8_t  s_step_motor      = 0xFFu; /* 当前正在测的电机（config.h 表索引，0xFF = 还没选） */
static uint8_t  s_step_ol         = 0;     /* 1 = 用开环命令 0xE0；0 = 用闭环命令 0xF1 */
static uint8_t  s_step_auto       = 0;     /* 1 = 没读到驱动器模式，两种命令轮流试 */
static uint8_t  s_step_need_clear = 0;     /* 1 = 上一步发过 0xFC 刹车，下次动作前必须先 0xFB 清状态 */
#endif

#if !GIMBAL_USE_TIM6_TICK
static uint32_t s_ctrl_tick  = 0;      /* 软件节拍兜底用（TIM6 没配时才编译） */
#endif

#if MAIN_LED_HEARTBEAT
static uint32_t s_led_tick   = 0;      /* 心跳灯节拍（板载用户 LED = PD0，低电平点亮） */
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#if CHASSIS_ENABLE
static void App_TestReset(void);           /* 切测试模式时复位四个测试的状态 */
#endif
static void App_VofaSource(void);          /* VOFA+ 波形数据源（下面 USER CODE 4 里实现） */
#if STEPPER_ENABLE && STEPPER_SELFTEST && !CHASSIS_ENABLE
static void Stepper_TestTask(void);        /* 步进开环转动自检（底盘启用时不编译） */
#endif

#if CHASSIS_ENABLE
static void Chassis_DistTestTask(void);    /* 里程测试（config.h ⑧ 区），实现在 USER CODE 4 */
#endif

#if CHASSIS_ENABLE
static void Chassis_TurnTestTask(void);    /* 转弯测试（config.h ⑨ 区），实现在 USER CODE 4 */
#endif

#if CHASSIS_ENABLE
static void Chassis_StrafeTestTask(void);  /* 平移测试（config.h ⑩ 区），实现在 USER CODE 4 */
#endif

#if CHASSIS_ENABLE
static void Chassis_Mission1Task(void);    /* 测试例程1（config.h 11 区），实现在 USER CODE 4 */
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if ARM_ENABLE && ARM_TEXT_DIAG
#include <stdio.h>

/* CAN 诊断的【纯文本行】输出（config.h 里 ARM_TEXT_DIAG=1 时启用）
   为什么不用 VOFA 通道：蓝牙链路波特率还没确定，文本行最省事——
   哪一档波特率能读出完整一行，蓝牙模块就是那一档（顺便把链路波特率定死）。
   同时每 4 秒轮一次 UART4 波特率：9600 → 115200 → 38400 → 57600 */
static void Main_ArmTextDiag(void)
{
    static uint32_t t_tx;
    static uint32_t t_baud;
    static uint8_t  b_idx;
    uint32_t now = HAL_GetTick();
    uint16_t tec = 0u, rec = 0u;
    uint8_t  lec = 0u;
    char     line[128];
    int      n;

#if BT_BAUD_CYCLE
    if ((uint32_t)(now - t_baud) >= 4000u)
    {
        static const uint32_t kB[4] = { 9600u, 115200u, 38400u, 57600u };

        t_baud = now;
        b_idx  = (uint8_t)((b_idx + 1u) % 4u);
        BT_UART->Init.BaudRate = kB[b_idx];
        (void)HAL_UART_Init(BT_UART);
    }
#else
    (void)t_baud;
    (void)b_idx;
#endif

    if ((uint32_t)(now - t_tx) < 300u) return;
    t_tx = now;

    ARM_GetCanErrors(&tec, &rec, &lec);
#if ARM_CAN_BAUD_SCAN
    n = snprintf(line, sizeof(line),
                 "DIAG BT=%lu CAN=%u LISTEN=%u TEC=%u REC=%u LEC=%u EXT=%u ID=%lu D0=%u "
                 "SER1=%ld/%u SER2=%ld/%u\r\n",
                 (unsigned long)BT_UART->Init.BaudRate,
                 (unsigned)ARM_GetCanScanIdx(),
                 (unsigned)ARM_GetCanListenOnly(),
                 (unsigned)tec, (unsigned)rec, (unsigned)lec,
                 (unsigned)ARM_GetCanExtCount(),
                 (unsigned long)ARM_GetLastExtId(),
                 (unsigned)ARM_GetLastExtByte(0u),
                 (long)ARM_GetPosPulses(0u), (unsigned)ARM_GetPosValid(0u),
                 (long)ARM_GetPosPulses(1u), (unsigned)ARM_GetPosValid(1u));
#else
    n = snprintf(line, sizeof(line),
                 "DIAG BT=%lu CAN=1M TEC=%u REC=%u LEC=%u EXT=%u ID=%lu D0=%u "
                 "SER1=%ld/%u SER2=%ld/%u\r\n",
                 (unsigned long)BT_UART->Init.BaudRate,
                 (unsigned)tec, (unsigned)rec, (unsigned)lec,
                 (unsigned)ARM_GetCanExtCount(),
                 (unsigned long)ARM_GetLastExtId(),
                 (unsigned)ARM_GetLastExtByte(0u),
                 (long)ARM_GetPosPulses(0u), (unsigned)ARM_GetPosValid(0u),
                 (long)ARM_GetPosPulses(1u), (unsigned)ARM_GetPosValid(1u));
#endif

    if (n > 0)
    {
        if (n > (int)sizeof(line)) n = (int)sizeof(line);
        (void)HAL_UART_Transmit(BT_UART, (uint8_t *)line, (uint16_t)n, 40u);
    }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();           /* TIM3 仍初始化；夹爪已改总线舵机，不再依赖 TIM3 PWM */
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_UART4_Init();          /* ★蓝牙 HC-05 走 UART4 = PA0/PA1（原来老 IMU 用 PC10/PC11，见 config.h ⑮ 区） */
  MX_USART6_UART_Init();    /* ★IMU（YbImu）现在走 USART6 = PC6/PC7，必须在 IMU_Init() 之前 */
  MX_I2C1_Init();
  MX_UART7_Init();          /* ★视觉 MaixCam：PE8(TX)/PE7(RX)，115200 */
  /* USER CODE BEGIN 2 */
  /* ==================== 云台 GM6020 初始化 ==================== */
#if defined(GIMBAL_IFACE_PWM)
  /* PWM 舵机模式：绑定 PWM 定时器 + 启动 PWM 输出
     ⚠️ CubeMX 里要先配好 TIM1_CH1：PSC=179（计频 1MHz，1 tick = 1µs）、ARR=19999（50Hz） */
  GM6020_PwmInit(&htim1, TIM_CHANNEL_1);
#else
  /* CAN 模式：CAN 滤波器 + HAL_CAN_Start + 打开 CAN 接收中断 */
  GM6020_Init();
#endif

#if GIMBAL_USE_TIM6_TICK
  /* 启动 1kHz 控制中断：控制环 GM6020_Loop() 在 TIM6 中断里跑 */
  HAL_TIM_Base_Start_IT(&htim6);
#endif

  /* 上电目标角度：先给 0°（安全初始值，避免上电乱转） */
  GM6020_SetTargetPosDeg(0.0f);

  /* ==================== 步进电机 PD42S1 初始化 ==================== */
#if STEPPER_ENABLE
  /* 绑定 config.h 里的 STEPPER_UART（默认 &huart3 = PB10/PB11），开接收中断 */
  PD42S1_Init();
#endif

#if ARM_ENABLE
  /* ==================== 升降臂（两个直线轴）初始化 ====================
     必须在 PD42S1_Init() 之后：走串口时它会"失能"不该出力的轴
     （不失能的话闭环步进上电就抱住，定标阶段齿条手推不动）；
     ARM_PROBE_ENABLE=1 时顺便把 CAN 滤波器改成收所有 ID */
  ARM_Init();
#endif

  /* ==================== 陀螺仪（外接，没接就是空实现） ==================== */
  Vision_Init();            /* ★视觉串口解析器（UART7）
                             必须在 MX_UART7_Init 之后 */
  IMU_Init();

  /* ==================== 底盘（麦轮 4 轮运动学） ==================== */
#if CHASSIS_ENABLE
  Chassis_Init();
#endif

  /* VOFA+ 调试输出（绑定 config.h 里的 VOFA_UART，默认串口2 = PA2）
     ⚠️ 需要 CubeMX 里已经加好 USART2；没加时这里是空函数 */
  VOFA_Init();
  VOFA_SetSource(&App_VofaSource);

  /* ==================== 蓝牙 HC-05（config.h ⑮ 区）====================
     两个模式就是"同一份数据发到不同的口"：
       模式1 有线：串口2（PA2/PA3）+ USB-TTL   ← VOFA_WIRED_EN
       模式2 蓝牙：串口4（PA0/PA1）+ HC-05    ← VOFA_BT_EN（频率用 VOFA_BT_HZ，蓝牙要降速）
     两个都可以是 1（同时发）。BT_Init() 里会按 BT_BAUD / BT_SET_BAUD 对波特率。 */
#if BT_ENABLE
  BT_Init();
#endif
#if BT_ENABLE && VOFA_BT_EN
  VOFA_SetUart2(BT_UART, VOFA_BT_HZ);          /* 第二路输出 = 蓝牙 */
#endif
#if ARM_ENABLE
  /* ★蓝牙当 VOFA 主口（VOFA_UART=(&huart4)）时：BT_Init 里重配 9600 会把
     ARM_Init 挂好的接收断掉 → 这里重新挂一次，否则滑块命令收不到 */
  ARM_RearmRx();
#endif
#if BT_ENABLE
  /* ★蓝牙实际速率决定发多快：升级到 115200 → 用 VOFA_TX_HZ；
     没升级成功（还是 9600）→ 只发 10Hz，否则 44 字节一帧阻塞太久会把主循环拖死 */
  {
    uint32_t bk = BT_GetLinkBaud();

    if (bk != 0u)
    {
      VOFA_SetRate((bk >= 57600u) ? (uint16_t)VOFA_TX_HZ : 10u);
    }
  }
#endif
#if !VOFA_WIRED_EN
  VOFA_SetWiredEnable(0u);                     /* 只走蓝牙：有线那路不发 */
#endif

  /* ==================== 调参命令通道（config.h ⑯ 区）====================
     有线：串口2 的 PA3（RX）接 USB-TTL 的 TX；走蓝牙就 TUNE_ON_BT=1。
     PC 可以：Z 归零 / G 走 mm / T 转度 / S 平移 / P 改参数 / Q 读参数 / M1 开遥测 /
              U<P> 总线舵机位置 / O 张开 / C 夹紧 */
#if TUNE_ENABLE
  TUNE_Init();
#endif

  /* ==================== 夹爪 = 总线舵机（config.h ⑰）==================== */
#if SERVO_ENABLE
  SERVO_Init();
#endif

#if CAN_PIN_GPIO_TEST
  /* 引脚自检：把 PA11/PA12 抢成普通 GPIO 翻转（CAN 此时不能用） */
  main_can_pin_gpio_init();
#endif

#if HMI_ENABLE
  /* 触摸屏菜单（config.h ⑫ 区）：需要在 CubeMX 里先开 UART5，再把 HMI_ENABLE 改 1 */
  HMI_Init(HMI_UART);
#endif      /* 注册波形数据源，VOFA_Task 会自动调 */

#if MAIN_LED_HEARTBEAT
  /* 心跳灯初始化：板载用户 LED = PD0（低电平点亮），代码里配，不用改 CubeMX */
  {
    GPIO_InitTypeDef led = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    led.Pin   = GPIO_PIN_0;
    led.Mode  = GPIO_MODE_OUTPUT_PP;
    led.Pull  = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &led);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);     /* 先灭 */
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ★主循环周期测量（诊断用）：now/最大 都发到 VOFA 的 ch8/ch9。
       为什么要量：控制环（Chassis_Task）和三个 PID 都在主循环里跑，
       循环一慢，控制频率就跟着掉（曾经出现"设 100Hz 实测只有 20Hz"，
       于是 PID 的 dt、积分/微分全都对不上 → 调出来的参数不可信）。 */
    {
      uint32_t t_loop = HAL_GetTick();
      uint32_t d_loop = (uint32_t)(t_loop - s_loop_tick);

      s_loop_tick = t_loop;
      if ((d_loop > 0u) && (d_loop < 5000u))
      {
        s_loop_ms_now = d_loop;
        if (d_loop > s_loop_ms_max) s_loop_ms_max = d_loop;
      }
    }

#if !GIMBAL_USE_TIM6_TICK
    /* TIM6 未启用时的软件节拍兜底（1kHz，抖动大，只适合测试） */
    if (HAL_GetTick() - s_ctrl_tick >= 1)
    {
      s_ctrl_tick = HAL_GetTick();
      GM6020_Loop();
    }
#endif

#if MAIN_GIMBAL_SELFTEST
    /* ============ 云台三位置循环：① → ② → ③ → ① …（角度在 config.h 3.7 区） ============
       判据：① IsOnline()==1 才会动（CAN 通、有编码器反馈）
             ② 到位（IsArrived）或超时（GIMBAL_TEST_MOVE_TIMEOUT_MS）→ 进停顿阶段
             ③ 停顿 MAIN_SELFTEST_PERIOD_MS 后换下一个位置 */
    {
      static const float s_pos[3] =
      {
        GIMBAL_TEST_POS_1_DEG, GIMBAL_TEST_POS_2_DEG, GIMBAL_TEST_POS_3_DEG
      };
      uint32_t now = HAL_GetTick();

      if (GM6020_IsOnline() == 0)
      {
        GM6020_Stop();                    /* CAN 掉线 → 立即停止输出，等恢复后从头开始 */
        s_test_issued = 0u;
        s_test_phase  = 0u;
        s_test_tick   = now;
      }
      else if (s_test_angle >·2u)
      {
        s_test_angle  = 0u;               /* 防御：索引越界就回第一个点 */
        s_test_issued = 0u;
      }
      else if (s_test_issued == 0u)
      {
        GM6020_SetTargetPosDeg(s_pos[s_test_angle]);   /* 下发这个位置的目标角度 */
        s_test_issued = 1u;
        s_test_phase  = 0u;
        s_test_tick   = now;
      }
      else if (s_test_phase == 0u)
      {
        if ((GM6020_IsArrived() != 0u) ||
            ((uint32_t)(now - s_test_tick) >= (uint32_t)GIMBAL_TEST_MOVE_TIMEOUT_MS))
        {
          s_test_phase = 1u;              /* 到位 或 超时 → 开始停顿 */
          s_test_tick  = now;
        }
      }
      else if ((uint32_t)(now - s_test_tick) >= (uint32_t)MAIN_SELFTEST_PERIOD_MS)
      {
        s_test_angle = (uint8_t)((s_test_angle + 1u) % 3u);
        GM6020_SetTargetPosDeg(s_pos[s_test_angle]);   /* ★同一拍直接下发下一个点 */
        s_test_phase = 0u;               /* ★回到"转向"阶段：要等它到位（或超时）才停顿 */
        s_test_tick  = now;              /* issued 保持 1，继续"转向 → 停顿"循环 */
      }
    }
#else
    /* ============ 你的业务逻辑写在这里（状态机 / 通信解析 / 决策） ============
       注意：这里不要用 HAL_Delay 阻塞；控制环已经在 TIM6 中断里跑 */
#endif

#if ARM_ENABLE
    ARM_Task();               /* 升降臂：读两个编码器 + 处理 VOFA 拖条（很轻） */
#endif
#if ARM_ENABLE && ARM_PROBE_ENABLE
    ARM_ProbeTask();          /* 嗅探：定时发 CANopen 探针引驱动器应答 */
#endif

    /* ============ 步进电机：收应答 + 开环转动自检 ============ */
#if STEPPER_ENABLE
    {
      uint32_t t0 = HAL_GetTick();

      PD42S1_Update();        /* 解析驱动器应答（很轻，每轮都调） */
      MAIN_SECT(MAIN_SECT_STEP, t0);
    }
#if STEPPER_SELFTEST && !CHASSIS_ENABLE
    /* 注意：底盘启用时不做步进自检 —— 否则自检会和底盘抢同一批电机 */
    Stepper_TestTask();       /* 开环转动测试，见下面 USER CODE 4 */
#endif
#endif

    /* ============ 陀螺仪 + 底盘 ============ */
    {
      uint32_t t0 = HAL_GetTick();

      IMU_Update();             /* 没接 IMU 时是空函数 */
      MAIN_SECT(MAIN_SECT_IMU, t0);
    }

#if CHASSIS_ENABLE
    {
      uint32_t t0 = HAL_GetTick();

      Chassis_Task();           /* 底盘：运动学下发 + 反馈 + 里程计（自检序列在里面） */
      MAIN_SECT(MAIN_SECT_CHASSIS, t0);
    }
#endif

#if defined(GIMBAL_IFACE_CAN)
    /* ★行进姿态：车一动，云台就转到 ROT_TRAVEL_DEG（250°），避免挡住前方/绕线。
       只在"开始动"的那一拍下发一次，不动的时候云台保持不动（可以手动定标）。 */
    {
      static uint8_t s_was_moving;

#if CHASSIS_ENABLE
      uint8_t moving = Chassis_IsMoving();
#else
      uint8_t moving = 0u;
#endif

      if ((moving != 0u) && (s_was_moving == 0u))
      {
        GM6020_SetTargetPosDegDir((float)ROT_TRAVEL_DEG, 0);
      }
      s_was_moving = moving;
    }
#endif

#if HMI_ENABLE && HMI_DEBUG_LINE
    /* ★把关键数据直接显示在车载屏幕上：不依赖电脑/蓝牙（蓝牙一上电机就掉线）
       屏幕文本框很小 → 拆成两行短文字：
         t0 :  R<轮1>,<轮2>,<轮3>,<轮4>        ← 判断轮速反馈通不通，最主要的判据
         t1 :  Y<航向> E<误差>                 ← 航向/纠偏情况 */
    {
      static uint32_t s_hmi_tick;
      static char     s_hmi_rpm[28];
      static char     s_hmi_yaw[28];
      uint32_t now = HAL_GetTick();

      if ((uint32_t)(now - s_hmi_tick) >= 500u)
      {
        uint8_t on_cnt = 0u, rv_cnt = 0u;

        s_hmi_tick = now;
        Chassis_GetFeedbackHealth(&on_cnt, &rv_cnt);

        /* 每 2 秒轮换一页（屏幕一行放不下）：轮速 → 链路 → 6020 → 视觉 */
        {
          uint32_t phase = (now / 2000u) % 4u;

          if (phase == 0u)
          {
            (void)snprintf(s_hmi_rpm, sizeof(s_hmi_rpm),
                           "R%.0f,%.0f,%.0f,%.0f",
                           (double)Chassis_GetWheelRpm(0u),
                           (double)Chassis_GetWheelRpm(1u),
                           (double)Chassis_GetWheelRpm(2u),
                           (double)Chassis_GetWheelRpm(3u));
          }
          else if (phase == 1u)
          {
            (void)snprintf(s_hmi_rpm, sizeof(s_hmi_rpm),
                           "N%u/4 V%u/4",
                           (unsigned)on_cnt, (unsigned)rv_cnt);
          }
          else if (phase == 2u)
          {
            /* 6020（旋转）：G<在线> <当前角>  → 在线=1 表示 CAN 收到 0x205 了 */
            (void)snprintf(s_hmi_rpm, sizeof(s_hmi_rpm),
                           "G%u %.0f",
                           (unsigned)GM6020_IsOnline(),
                           (double)GM6020_GetCurrentDeg());
          }
          else
          {
            /* 视觉联调：V<字节数> <帧数> <dx>,<dy>  → 帧数在涨 = 相机在说话 */
            int16_t vdx = 0, vdy = 0;

            (void)Vision_GetOffset(&vdx, &vdy);
            (void)snprintf(s_hmi_rpm, sizeof(s_hmi_rpm),
                           "V%lu %lu %d,%d",
                           (unsigned long)Vision_RxByteCount(),
                           (unsigned long)Vision_FrameCount(),
                           (int)vdx, (int)vdy);
          }
        }

        (void)snprintf(s_hmi_yaw, sizeof(s_hmi_yaw),
                       "Y%.0f T%.0f",
                       (double)Chassis_GetYawDeg(),
                       (double)Chassis_GetTurnErrDeg());   /* T = 转向误差（调 T90 时看这个） */
        HMI_SetTextObj("t0", s_hmi_rpm);
        HMI_SetTextObj("t1", s_hmi_yaw);
      }
    }
#endif

#if CHASSIS_ENABLE
    /* ===== 触摸屏菜单：有请求就切模式并复位测试状态 ===== */
    {
        uint8_t req;

#if HMI_ENABLE
        HMI_Poll();                        /* 屏幕命令解析（障碍/出发/取消）都在这里推进 */
#endif
        req = HMI_TakeRequest();

        if (req != 0u)
        {
            Chassis_Stop();
            App_TestReset();
            s_run_mode = req;
            RouteRun_Stop();
#if HMI_ENABLE && HMI_REPORT_STATUS
            HMI_ReportMode(s_run_mode);
#endif
        }

#if HMI_ENABLE
        if (HMI_TakeStop() != 0u)               /* 取消：停车 + 清路线 */
        {
            Chassis_Stop();
            App_TestReset();
            s_run_mode = RUNMODE_NONE;
            RouteRun_Stop();
#if HMI_REPORT_STATUS
            HMI_ReportLine("STOP");
#endif
        }

        if (HMI_TakeStart() != 0u)              /* 出发：按障碍算路并开始跑 */
        {
            Chassis_Stop();
            App_TestReset();

            if (RouteRun_Start() != 0u)
            {
                s_run_mode = RUNMODE_ROUTE;
#if HMI_REPORT_STATUS
                HMI_ReportLine("GO");
#endif
            }
            else
            {
                s_run_mode = RUNMODE_NONE;
#if HMI_REPORT_STATUS
                HMI_ReportLine("NOPATH");
#endif
            }
        }

        if (s_run_mode == RUNMODE_ROUTE)        /* 路线执行：每拍推进一步 */
        {
            RouteRun_Task();
            if (RouteRun_IsActive() == 0u) s_run_mode = RUNMODE_NONE;
        }
#endif
    }
    /* ===== 自动测试：按"当前模式"只跑其中一个（菜单可随时切）===== */
    if      (s_run_mode == RUNMODE_M1)     Chassis_Mission1Task();
    else if (s_run_mode == RUNMODE_STRAFE) Chassis_StrafeTestTask();
    else if (s_run_mode == RUNMODE_TURN)   Chassis_TurnTestTask();
    else if (s_run_mode == RUNMODE_DIST)   Chassis_DistTestTask();
#endif


    /* ============ VOFA+ 波形输出 ============
       按 config.h 的 VOFA_TX_HZ 限频（默认 100Hz），放主循环里不占控制中断。
       蓝牙那路有自己的频率（VOFA_BT_HZ），同一帧数据一起发 */
    {
      uint32_t t0 = HAL_GetTick();

      VOFA_Task();
      MAIN_SECT(MAIN_SECT_VOFA, t0);
    }

    /* 蓝牙收（手机/上位机发过来的文本：'0'~'4' 直接当模式命令）
       —— 这一段连同上面没单独计时的部分（屏幕 / 云台块）一起算"其他" */
    {
      uint32_t t0 = HAL_GetTick();

      BT_Task();
#if BT_ENABLE && BT_LINK_SCAN
      BT_LinkScanTask();          /* ★链路找波特率：每 1.5s 换一档发 "PING <波特率>" */
#endif
      MAIN_SECT(MAIN_SECT_REST, t0);
    }

#if TUNE_ENABLE
    /* 调参通道：解析 PC 命令 + 推进小动作 + 到点发遥测（顺便把它的耗时也记进"其他"段） */
    {
      uint32_t t0 = HAL_GetTick();

      TUNE_Task();
      MAIN_SECT(MAIN_SECT_REST, t0);
    }
#endif

#if ARM_ENABLE && ARM_TEXT_DIAG
    /* ★CAN 诊断的【纯文本行】输出：不依赖 VOFA 通道表，串口助手/网页直接读。
       顺便每 4 秒轮一次 UART4 波特率 → 哪一档能读出整行，蓝牙就是那一档 */
    Main_ArmTextDiag();
#endif

#if SERVO_ENABLE
    /* 夹爪舵机：每轮把脉宽朝目标平滑推进一点（限速，防猛甩/防堵转大电流） */
    SERVO_Task();
#endif

    /* CAN 引脚软件示波器：高频采样 PA12/PA11，统计低电平占比（ch12/ch13） */
    main_can_pin_sample();

#if CAN_PIN_GPIO_TEST
    /* 引脚自检模式：持续翻转两根脚（配合上面的软件示波器读占比） */
    main_can_pin_gpio_toggle();
#endif

#if MAIN_LED_HEARTBEAT
    /* ============ 心跳灯（PD0）============
       1Hz 慢闪 = 一切正常；5Hz 快闪 = 串口没初始化成功（配置丢了/没生成代码） */
    {
      uint32_t period = (uint32_t)MAIN_LED_HEARTBEAT_MS;

#if STEPPER_ENABLE
      if (huart3.Instance != STEPPER_UART_INSTANCE) period = 100u;   /* 报警：USART3 没初始化 */
      else
#endif 
      if (huart2.Instance != USART2) period = 100u;                  /* 报警：USART2 没初始化 */

      if ((uint32_t)(HAL_GetTick() - s_led_tick) >= period)
      {
        s_led_tick = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
      }
    }
#endif

  }   /* ⚠️ 关 while(1)。CubeMX 重新生成代码时容易把这行吃掉：
         症状 = SystemClock_Config / RCC_ClkInitStruct 那一堆 "expected a ;"。
         每次用 CubeMX 生成完代码，先检查这里有没有这个 } */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* 把四个自动测试的状态全部复位（切模式时调用 → 新选的测试从头跑）
   ⚠️ 底盘关掉（CHASSIS_ENABLE=0）时没人调用它 → 用 #if 包住，免得吃 #177-D warning */
#if CHASSIS_ENABLE
static void App_TestReset(void)
{
#if CHASSIS_ENABLE
    s_m1_idx = 0u;  s_m1_sub = 0u;  s_m1_started = 0u;  s_m1_tick = 0u;
    s_m1_mm0 = 0.0f; s_m1_deg0 = 0.0f; s_m1_moved = 0.0f; s_m1_target = 0.0f;

    s_stest_phase = 0u; s_stest_started = 0u; s_stest_tick = 0u; s_stest_vy = 0.0f;
    s_ttest_phase = 0u; s_ttest_started = 0u; s_ttest_tick = 0u;
    s_ttest_wz = 0.0f;  s_ttest_angle = 0.0f;
    s_dtest_phase = 0u; s_dtest_started = 0u; s_dtest_tick = 0u; s_dtest_expect_mm = 0.0f;

    /* ★把航向零点也重置：每次（从触摸屏）选一个测试，当前车头方向就算 0°
       → 转弯/走直测试每次都从 0° 开始，连测几次不会累积漂移
       （内部会调 IMU_ResetYaw()，没接 IMU 时是空操作，不影响底盘） */
    Chassis_ResetYaw();

    /* 下面这几个量只有通道全开的 VOFA 版本才会读；现在 VOFA 只发姿态，
       这里显式读一下，避免编译器报 set-but-never-used（将来恢复通道时真有用途） */
    (void)s_m1_target;
    (void)s_ttest_angle;
    (void)s_dtest_expect_mm;
    /* PID 版测试里这两个量只写不读（老的 VOFA 全量通道已经注释掉了）→ 显式读一下 */
    (void)s_ttest_wz;
    (void)s_stest_vy;
#endif
}
#endif /* CHASSIS_ENABLE（App_TestReset） */

/* ============================================================================
 *  VOFA+ 波形数据源
 *
 *  ★三套分支都写在这里，但【只有一个会被编译】，由 config.h 的 VOFA_SOURCE 决定：
 *        VOFA_SRC_ARM    → App_VofaArm()      现在 = 2 路（两个编码器）
 *        VOFA_SRC_GIMBAL → App_VofaGimbal()   云台 10 路
 *        VOFA_SRC_IMU    → App_VofaImu()      IMU 姿态 5 路
 *        VOFA_SRC_CHASSIS→ App_VofaChassis()  调底盘 PID 用 10 路
 *    ✗ 没被选中的三个函数在 Keil 里会显示成【灰色】（不参与编译、不进 bin）。
 *      想确认到底发了几路：① 看帧长 = 路数×4+4 字节（2 路=12B、4 路=20B、10 路=44B）
 *                          ② Keil 里右键那段代码，灰的就是没编译
 *  ⚠️ 采样率 = config.h 的 VOFA_TX_HZ（默认 100Hz），VOFA+ 里填一样的值；
 *     协议选 JustFloat；串口2 = PA2，115200。
 * ============================================================================ */

#if VOFA_SOURCE == VOFA_SRC_GIMBAL
/* ---- 云台 GM6020：10 路（ch0 目标角度 / ch1 当前角度 / ch2 ecd / ch3 累计 / ch4 误差 /
       ch5 输出 / ch6 转速 / ch7 转矩电流 / ch8 在线 / ch9 温度）
       ⚠️ PWM 舵机模式下没有反馈，ch2~ch9 会是 0 */
static void App_VofaGimbal(void)
{
    const gm6020_measure_t *m = GM6020_GetMeasure();
    float err_deg = GM6020_GetTargetDeg() - GM6020_GetCurrentDeg();

    VOFA_Send(10,
              GM6020_GetTargetDeg(),            /* ch0 目标角度(°，0~360) */
              GM6020_GetCurrentDeg(),           /* ch1 当前角度(°，0~360，机械角） */
              (float)m->ecd,                    /* ch2 编码器原始值(0~8191) */
              (float)m->total_ecd,              /* ch3 累计角度(编码器计数，多圈，仅供显示) */
              err_deg,                          /* ch4 角度误差(°)：一直不为 0 = 到不了位 */
              (float)GM6020_GetCurrentOutput(), /* ch5 输出指令(0x1FF 的 int16，±30000) */
              (float)m->speed_rpm,              /* ch6 转速(rpm) */
              (float)m->current,                /* ch7 实际转矩电流(反馈值) */
              (float)GM6020_IsOnline(),         /* ch8 在线：1=收到反馈 0=没有 */
              (float)m->temp);                  /* ch9 温度(℃) */
}

#elif VOFA_SOURCE == VOFA_SRC_ARM
/* ---- 升降臂：定标用（默认 2 路；开嗅探/开滑块调试时路数不同）---- */
static void App_VofaArm(void)
{
#if ARM_ENABLE && ARM_PROBE_ENABLE
    /* CAN 嗅探结果（12 路）：ch0 帧数 / ch1 几种 ID / ch2·ch3 ID0 低位·高位 /
       ch4 ID0 是否扩展帧 / ch5·ch6 ID0 数据字节0·1 / ch7 ID1 低位 / ch8 ID1 扩展 /
       ch9 探针数 / ch10 发送错误计数 TEC / ch11 接收错误计数 REC */
    {
        uint16_t tec = 0u, rec = 0u;
        uint8_t  lec = 0u;

        ARM_ProbeErrorCounters(&tec, &rec, &lec);

        VOFA_Send(12,
                  (float)ARM_ProbeFrameCount(),
                  (float)ARM_ProbeDistinct(),
                  (float)(ARM_ProbeId(0) & 0xFFFFu),
                  (float)((ARM_ProbeId(0) >> 16) & 0x1FFFu),
                  (float)ARM_ProbeIsExt(0),
                  (float)ARM_ProbeData(0, 0u),
                  (float)ARM_ProbeData(0, 1u),
                  (float)(ARM_ProbeId(1) & 0xFFFFu),
                  (float)ARM_ProbeIsExt(1),
                  (float)ARM_ProbeSent(),
                  (float)tec,                       /* ch10 发送错误计数 */
                  (float)rec);                      /* ch11 接收错误计数 */
    }
#elif ARM_ENABLE
#if ARM_VOFA_SHOW_SLIDER
    /* ★速率扫描版（6 路）——用来确定"臂驱动器到底挂在哪档速率上"：
         ch0 螺杆编码器位置 / ch1 位置有效 / ch2 扩展帧计数（当前档）
         ch3 TEC（发送错误） / ch4 hits 掩码（1=1M 收到过 2=500K 4=250K） / ch5 当前档位(0/1/2)
       用法：拔掉 GM6020 的 CAN 线（它只认 1M，会把别的档位搅乱），复位，看 15 秒：
         · ch4 从 0 变成 1  → 1 Mbps 对
         · ch4 变成 2       → 500 Kbps 对（手册默认值）
         · ch4 变成 4       → 250 Kbps 对
         · ch4 一直是 0     → 三档都不对：查驱动器 CAN 使能/MODBUS 是否关掉/120Ω/共地 */
    {
        uint16_t tec = 0u, rec = 0u;
        uint8_t  lec = 0u;

        ARM_GetCanErrors(&tec, &rec, &lec);

        VOFA_Send(6,
#if ARM_VOFA_CAN_DIAG
                  (float)tec,                    /* ch0 TEC */
#if ARM_CAN_BAUD_SCAN
                  (float)ARM_GetCanScanHits(),   /* ch1 hits 掩码：1=1M 2=500K 4=250K 8=125K 16=100K */
                  (float)ARM_GetCanScanIdx(),    /* ch2 当前档位 0..4 */
#else
                  (float)rec,                    /* ch1 REC */
                  (float)lec,                    /* ch2 LEC */
#endif
                  (float)ARM_GetCanExtCount(),   /* ch3 扩展帧计数 */
                  (float)(ARM_GetLastExtId() & 0xFFFFu),  /* ch4 最后收到的扩展帧 ID 低 16 位 */
                  (float)ARM_GetLastExtByte(0u));         /* ch5 该帧第一个数据字节 */
#else
                  ARM_GetPosPulses(0u),             /* ch0 轴①（螺杆）位置 脉冲 */
                  (float)ARM_GetPosValid(0u),       /* ch1 轴① 位置有效 */
                  ARM_GetPosPulses(1u),             /* ch2 轴②（齿条）位置 脉冲 */
                  (float)ARM_GetPosValid(1u),       /* ch3 轴② 位置有效 */
                  ARM_GetSlider(0u),                /* ch4 滑块①（A=）最近发来的值 */
                  ARM_GetSlider(1u));               /* ch5 滑块②（B=）最近发来的值 */
#endif
    }
#else
    /* ★2 路（当前就是这套）：ch0 = 轴①（螺杆）编码器，ch1 = 轴②（齿条）编码器
       单位 = 脉冲（51200 = 一圈）；想按角度看就把 ARM_GetPosPulses 换成 ARM_GetPosDeg */
    VOFA_Send(2,
              ARM_GetPosPulses(0u),       /* ch0 轴①（螺杆）编码器位置 */
              ARM_GetPosPulses(1u));      /* ch1 轴②（齿条）编码器位置 */
#endif
#else
    VOFA_Send(2, 0.0f, 0.0f);             /* 升降臂模块关了：发两路 0 占位 */
#endif
}

#elif VOFA_SOURCE == VOFA_SRC_CHASSIS
    /* ---- 调 PID 用：三个环的中间量（通道表见 config.h ⑥ 区）----
       ch0 yaw / ch1 航向误差 / ch2 航向环输出 / ch3 实测 wz
       ch4 前向位移 / ch5 横向位移 / ch6 旋转误差 / ch7 旋转环输出
       ch8 平移误差 / ch9 平移环输出 */
static void App_VofaChassis(void)
{
    float dbg[10];

    Chassis_GetPidDebug(dbg, 10u);
    VOFA_Send(10, dbg[0], dbg[1], dbg[2], dbg[3], dbg[4],
                  dbg[5], dbg[6], dbg[7], dbg[8], dbg[9]);
}
#elif VOFA_SOURCE == VOFA_SRC_IMU
/* ---- IMU 姿态：7 路（开 IMU_VOFA_STATUS_CH 时）或 3 路 ---- */
static void App_VofaImu(void)
{
#if IMU_VOFA_STATUS_CH
    VOFA_Send(17,
              IMU_GetYawDeg(),        /* ch0 航向角 yaw  度（逆时针为正，已扣漂移） */
              IMU_GetRollDeg(),       /* ch1 横滚角 roll 度 */
              IMU_GetPitchDeg(),      /* ch2 俯仰角 pitch 度 */
              (float)IMU_GetStatus(), /* ch3 状态：0=正常 1=本轮没收到帧 2=离线 */
              (float)IMU_GetScanResult(), /* ch4 最近收到的功能码：38=0x26 姿态帧正常 */
              IMU_GetGyroZDps(),      /* ch5 ★陀螺 Z（度/s）：静止时 |这个值|×60 = 零偏 */
              (float)IMU_GetCalibState(), /* ch6 ★校准回执：非 0 = 上电那次校准拿到了回执 */
              IMU_GetYawTrimDpm(),    /* ch7 ★测出的 yaw 漂移率（度/分），被扣掉的量 */
              Main_GetLoopMs(),       /* ch8 ★主循环周期(ms)：应 ≤10；一直几十 = 有阻塞点 */
              Main_GetLoopMaxMs(),    /* ch9 ★最慢的一轮(ms)：600 = 开机测漂移那 600ms，正常 */
              VOFA_GetTxMs(),         /* ch10 ★这一帧有线发送耗时(ms)：115200 下应 ≈3.5 */
              VOFA_GetUartBaudK(),    /* ch11 ★主口波特率÷1000：应 115.2（=9.6 就是被改小了） */
              Main_GetSectMs(0),      /* ch12 PD42S1_Update 最慢(ms) */
              Main_GetSectMs(1),      /* ch13 IMU_Update 最慢(ms) */
              Main_GetSectMs(2),      /* ch14 Chassis_Task 最慢(ms) ← 嫌疑最大 */
              Main_GetSectMs(3),      /* ch15 VOFA_Task 最慢(ms) */
              Main_GetSectMs(4));     /* ch16 其他(屏幕/蓝牙/云台块) 最慢(ms) */
#else
    VOFA_Send(4,
              IMU_GetYawDeg(),        /* ch0 航向角 yaw  度（逆时针为正，已扣漂移） */
              IMU_GetRollDeg(),       /* ch1 横滚角 roll 度 */
              IMU_GetPitchDeg(),      /* ch2 俯仰角 pitch 度 */
              IMU_GetGyroZDps());     /* ch3 ★陀螺 Z 角速度 度/s：静止应≈0，转动手感最直观 */
#endif
}

#else
#error "config.h: VOFA_SOURCE 只能填 VOFA_SRC_ARM / VOFA_SRC_GIMBAL / VOFA_SRC_IMU / VOFA_SRC_CHASSIS"
#endif

/* 真正被 VOFA 周期调用的入口：只调用上面【被编译的那个】函数
   ⚠️ 加新数据源时这里必须同步加分支：分派漏了而通道函数被 #if 排除掉，
      症状就是"identifier App_VofaXxx is undefined"这种莫名其妙的报错 */
static void App_VofaSource(void)
{
#if VOFA_SOURCE == VOFA_SRC_GIMBAL
    App_VofaGimbal();
#elif VOFA_SOURCE == VOFA_SRC_ARM
    App_VofaArm();
#elif VOFA_SOURCE == VOFA_SRC_CHASSIS
    App_VofaChassis();
#else
    App_VofaImu();
#endif
}

    /* ---- 原来的多模式版本（要恢复就把上面 3 行换成这一整段）----
    {
        float ch_vx = 0.0f, ch_vy = 0.0f, ch_wz = 0.0f;

#if CHASSIS_ENABLE
        Chassis_GetVelocityEstimate(&ch_vx, &ch_vy, &ch_wz);
#endif

        switch (s_run_mode)
        {
        case RUNMODE_M1:          // ⑪ 例程1：步骤号 / 本步进度
            VOFA_Send(5, (float)(s_m1_idx + 1u), s_m1_moved, s_m1_target,
                      Chassis_GetYawDeg(), Chassis_GetDistanceMm());
            break;
        case RUNMODE_STRAFE:      // ⑩ 平移
            VOFA_Send(4, ch_vy, ch_vx, Chassis_GetDistanceMm(), s_stest_vy);
            break;
        case RUNMODE_TURN:        // ⑨ 转弯
            VOFA_Send(5, s_ttest_wz, ch_wz, Chassis_GetYawDeg(), s_ttest_angle,
                      Chassis_GetDistanceMm());
            break;
        case RUNMODE_DIST:        // ⑧ 里程
            VOFA_Send(3, ch_vx, Chassis_GetDistanceMm(), s_dtest_expect_mm);
            break;
        default:                  // 正常模式（停止）：全量 12 路
            VOFA_Send(12,
                      (float)PD42S1_Get()->rpm,                              // ch0  步进转速
                      (float)PD42S1_Get()->pos / (float)PD42S1_PULSE_PER_REV, // ch1 位置 圈
                      (float)(PD42S1_GetActiveMotor() + 1u),                 // ch2  当前电机号
                      (float)PD42S1_Get()->mode,                             // ch3  驱动器模式
                      (float)PD42S1_Get()->rx_ok,                            // ch4  应答条数
                      (float)PD42S1_Get()->tx_fail,                          // ch5  失败次数
                      Chassis_GetYawDeg(),                                   // ch6  底盘航向
                      ch_vx, ch_wz,                                          // ch7/ch8 里程计
                      (float)IMU_IsOnline(),                                 // ch9  IMU 在线
                      IMU_GetGyroZDps(),                                     // ch10 陀螺 Z
                      IMU_GetYawDeg());                                      // ch11 IMU 航向
            break;
        }
    }
    ---- 原版结束 ---- */

#if CHASSIS_ENABLE
/* ============================================================================
 *  里程测试（config.h ⑧ 区）—— 前进 N 米 → 停 → 后退同样距离 → 停
 *
 *  · 判距离用"时间 × 速度"：每段跑 CHASSIS_DIST_TEST_MS 毫秒（= 距离 ÷ 速度）
 *    不依赖里程计反馈，所以方向符号还没调对时也不会失控乱跑
 *  · CHASSIS_DIST_TEST_LOOP = 1 时来回循环；= 0 时去、回各一次就停住不动
 *  · 里程计的数照样发给 VOFA 做对照（见上面 App_VofaSource 的测试分支）
 *
 *  ⚠️ 上电就会真的把车开出去：先架空，或留够 2 米以上直线空间
 * ============================================================================ */
static void Chassis_DistTestTask(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_dtest_phase)
    {
    /* ---------- ① 前进 CHASSIS_DIST_TEST_MM ---------- */
    case 0u:
        if (s_dtest_started == 0u)                  /* 本段第一次进来：下发速度 */
        {
            s_dtest_started   = 1u;
            s_dtest_tick      = now;
            s_dtest_expect_mm = CHASSIS_DIST_TEST_MM;
            Chassis_SetVelocity(CHASSIS_DIST_TEST_SPEED, 0.0f, 0.0f);
        }
        if ((uint32_t)(now - s_dtest_tick) >= (uint32_t)CHASSIS_DIST_TEST_MS)
        {
            Chassis_Stop();                         /* 跑够了：停车（保持使能，不刹车不发烫） */
            s_dtest_started = 0u;
            s_dtest_phase   = 1u;
            s_dtest_tick    = now;
        }
        break;

    /* ---------- ② 停 CHASSIS_DIST_TEST_PAUSE_MS ---------- */
    case 1u:
        if ((uint32_t)(now - s_dtest_tick) >= (uint32_t)CHASSIS_DIST_TEST_PAUSE_MS)
        {
            s_dtest_phase = 2u;
        }
        break;

    /* ---------- ③ 反方向走同样距离 ---------- */
    case 2u:
        if (s_dtest_started == 0u)
        {
            s_dtest_started   = 1u;
            s_dtest_tick      = now;
            s_dtest_expect_mm = CHASSIS_DIST_TEST_MM * 2.0f;
            Chassis_SetVelocity(-CHASSIS_DIST_TEST_SPEED, 0.0f, 0.0f);
        }
        if ((uint32_t)(now - s_dtest_tick) >= (uint32_t)CHASSIS_DIST_TEST_MS)
        {
            Chassis_Stop();
            s_dtest_started = 0u;
            s_dtest_phase   = 3u;
            s_dtest_tick    = now;
        }
        break;

    /* ---------- ④ 停一下 → 循环 或 结束 ---------- */
    case 3u:
        if ((uint32_t)(now - s_dtest_tick) >= (uint32_t)CHASSIS_DIST_TEST_PAUSE_MS)
        {
#if CHASSIS_DIST_TEST_LOOP
            s_dtest_phase = 0u;                     /* 接着来回跑 */
#else
            s_dtest_phase = 4u;                     /* 结束 */
#endif
        }
        break;

    /* ---------- ⑤ 完成：停止那一下已经发过了，这里不再重复下发 ---------- */
    default:
        break;
    }
}
#endif /* CHASSIS_DIST_TEST_ACTIVE */

#if CHASSIS_ENABLE
/* ============================================================================
 *  转弯测试（config.h ⑨ 区）
 *    左转 A1 → 停 → 右转 A1 → 停 → 左转 A2 → 停 → 右转 A2 → 停
 *    判角度用"时间 × 角速度"：每段跑 角度 ÷ CHASSIS_TURN_TEST_WZ 秒
 *      · 好处：不需要任何转角反馈（没 IMU 也能跑）；wz 符号错会立刻看出来（转向反/变平移）
 *      · 局限：原地转打滑大，实际转角一般偏小；要精确转角得上 IMU 或位置模式
 *    CHASSIS_TURN_TEST_LOOP = 1 时四段跑完接着从头再来
 *
 *  ⚠️ 上电就会真的原地转，周围留出车长空间
 * ============================================================================ */
static void Chassis_TurnTestTask(void)
{
    uint32_t now = HAL_GetTick();

    if (s_ttest_phase >= 8u) return;               /* 完成：最后一下停车已经发过，不再重复下发 */

    if ((s_ttest_phase & 1u) == 0u)
    {
        /* ---------- 动作段：偶数相位 0/2/4/6 ---------- */
        uint8_t  leg = (uint8_t)(s_ttest_phase >> 1);          /* 0..3 */
        float    ang = (leg < 2u) ? CHASSIS_TURN_TEST_A1 : CHASSIS_TURN_TEST_A2;
        float    dir = ((leg & 1u) == 0u) ? 1.0f : -1.0f;      /* 0/2 左转(+)，1/3 右转(−) */
#if CHASSIS_TURN_USE_PID
        /* PID 版不用算时间：完成条件改成"yaw 到位" */
#else
        uint32_t dur = (uint32_t)(ang / CHASSIS_TURN_TEST_WZ * 1000.0f);
#endif

        if (s_ttest_started == 0u)                 /* 本段第一次进来：下发角速度 */
        {
            s_ttest_started = 1u;
            s_ttest_tick    = now;
            s_ttest_angle   = ang;
            s_ttest_wz      = dir * CHASSIS_TURN_TEST_WZ;   /* 只用于 VOFA 显示 */
#if CHASSIS_TURN_USE_PID
            /* PID 版：目标角交给旋转环，yaw 反馈到了才算转够（要用 IMU！）
               ⚠️ Chassis_TurnStart(deg) 是【绝对】目标角，这里要的是"从当前朝向再转 ang 度"
                  → 自己加上当前 yaw（测试序列是 左90 → 右90 → 左180 → 右180：
                    用绝对角的话第二段会变成"转到 −90°"而不是"转回来"） */
            Chassis_TurnStart(Chassis_GetYawDeg() + dir * ang);
#else
            Chassis_SetVelocity(0.0f, 0.0f, s_ttest_wz);
#endif
        }
#if CHASSIS_TURN_USE_PID
        /* 到位（转够 + 稳住 CHASSIS_TURN_SETTLE_MS）就停；超时兜底防死等 */
        if ((Chassis_TurnIsDone() != 0u) ||
            ((uint32_t)(now - s_ttest_tick) >= (uint32_t)CHASSIS_TURN_TEST_TIMEOUT_MS))
#else
        if ((uint32_t)(now - s_ttest_tick) >= dur) /* 转够了：停车 */
#endif
        {
            Chassis_TurnStop();
            Chassis_Stop();
            s_ttest_started = 0u;
            s_ttest_phase++;
            s_ttest_tick    = now;
        }
    }
    else
    {
        /* ---------- 停顿段：奇数相位 1/3/5/7 ---------- */
        if ((uint32_t)(now - s_ttest_tick) >= (uint32_t)CHASSIS_TURN_TEST_PAUSE_MS)
        {
            s_ttest_phase++;
            s_ttest_tick = now;
            if (s_ttest_phase >= 8u)
            {
#if CHASSIS_TURN_TEST_LOOP
                s_ttest_phase = 0u;                /* 循环：从头再来 */
#else
                s_ttest_phase = 8u;                /* 完成：以后什么都不做 */
#endif
            }
        }
    }
}
#endif /* CHASSIS_TURN_TEST_ACTIVE */
#if CHASSIS_ENABLE
/* ============================================================================
 *  平移测试（config.h ⑩ 区）
 *    左平移 N mm → 停 → 右平移 N mm → 停（LOOP=1 则来回不停）
 *    判距离："时间 × 速度"，每段跑 CHASSIS_STRAFE_TEST_MS 毫秒
 *    判对错：车【横着走】= 对；【前后走】= 辊子方向/符号错；【原地转】= 符号不成套
 * ============================================================================ */
static void Chassis_StrafeTestTask(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_stest_phase)
    {
    /* ---------- ① 左平移（vy > 0） ---------- */
    case 0u:
        if (s_stest_started == 0u)
        {
            s_stest_started = 1u;
            s_stest_tick    = now;
            s_stest_vy      = +CHASSIS_STRAFE_TEST_SPEED;
#if CHASSIS_STRAFE_USE_PID
            /* PID 版：目标横向位移(mm)交给平移环，到位才算走够（位移是轮式积分，打滑会差一点） */
            Chassis_StrafeStart(0.0f, +CHASSIS_STRAFE_TEST_MM);
#else
            Chassis_SetVelocity(0.0f, s_stest_vy, 0.0f);
#endif
        }
#if CHASSIS_STRAFE_USE_PID
        if ((Chassis_StrafeIsDone() != 0u) ||
            ((uint32_t)(now - s_stest_tick) >= (uint32_t)CHASSIS_STRAFE_TEST_TIMEOUT_MS))
#else
        if ((uint32_t)(now - s_stest_tick) >= (uint32_t)CHASSIS_STRAFE_TEST_MS)
#endif
        {
            Chassis_StrafeStop();
            Chassis_Stop();
            s_stest_started = 0u;
            s_stest_phase   = 1u;
            s_stest_tick    = now;
        }
        break;

    /* ---------- ② 停 ---------- */
    case 1u:
        if ((uint32_t)(now - s_stest_tick) >= (uint32_t)CHASSIS_STRAFE_TEST_PAUSE_MS)
        {
            s_stest_phase = 2u;
        }
        break;

    /* ---------- ③ 右平移（vy < 0） ---------- */
    case 2u:
        if (s_stest_started == 0u)
        {
            s_stest_started = 1u;
            s_stest_tick    = now;
            s_stest_vy      = -CHASSIS_STRAFE_TEST_SPEED;
#if CHASSIS_STRAFE_USE_PID
            Chassis_StrafeStart(0.0f, -CHASSIS_STRAFE_TEST_MM);   /* 相对当前位置往右走回来 */
#else
            Chassis_SetVelocity(0.0f, s_stest_vy, 0.0f);
#endif
        }
#if CHASSIS_STRAFE_USE_PID
        if ((Chassis_StrafeIsDone() != 0u) ||
            ((uint32_t)(now - s_stest_tick) >= (uint32_t)CHASSIS_STRAFE_TEST_TIMEOUT_MS))
#else
        if ((uint32_t)(now - s_stest_tick) >= (uint32_t)CHASSIS_STRAFE_TEST_MS)
#endif
        {
            Chassis_StrafeStop();
            Chassis_Stop();
            s_stest_started = 0u;
            s_stest_phase   = 3u;
            s_stest_tick    = now;
        }
        break;

    /* ---------- ④ 停 → 循环 或 结束 ---------- */
    case 3u:
        if ((uint32_t)(now - s_stest_tick) >= (uint32_t)CHASSIS_STRAFE_TEST_PAUSE_MS)
        {
#if CHASSIS_STRAFE_TEST_LOOP
            s_stest_phase = 0u;
#else
            s_stest_phase = 4u;
#endif
        }
        break;

    /* ---------- ⑤ 完成：停车已发过，不再重复下发 ---------- */
    default:
        break;
    }
}
#endif /* CHASSIS_ENABLE */

#if CHASSIS_ENABLE
/* ============================================================================
 *  测试例程 1（config.h ⑪ 区）
 *    前进 SIDE → 左转90 → … ×4（回到起点）→ 左平移 STRAFE → 右平移 STRAFE → 停
 *    每步：下发指令 → 判完成 → 停车 → 停顿 GAP_MS → 下一步
 *      距离类（前进/后退/平移）：看里程计【累计路程】增量 ≥ 目标 - 容差
 *      角度类（左转/右转）：接 IMU 时看航向角；没 IMU 时用 时间 × 角速度
 *    ⚠️ 每步都有超时保护，判据永远不满足也不会一直开下去
 * ============================================================================ */
typedef enum
{
    M1_FWD = 0,        /* 前进 */
    M1_BACK,           /* 后退（本路线没用，留着方便改路线） */
    M1_STRAFE_L,       /* 左平移 */
    M1_STRAFE_R,       /* 右平移 */
    M1_TURN_L,         /* 左转 */
    M1_TURN_R,         /* 右转 */
    M1_END             /* 结束 */
} m1_act_t;

typedef struct
{
    uint8_t act;       /* m1_act_t */
    float   value;     /* 距离(mm) 或 角度(度)；写 0 = 取 config.h 的边长/平移距离 */
} m1_step_t;

/* ---- 路线表：想改路线就改这里（前进/后退/平移/转弯随意组合） ---- */
static const m1_step_t s_m1_plan[] =
{
#if CHASSIS_MISSION1_NO_TURN
    /* ---- 无 IMU 闭合版：不转弯，用麦轮"平移"把 1×1 米方形走出来 ----
       为什么能闭合：① 全程不转 → 航向不累积误差；
                     ② 四段两两对称（前进/后退、左移/右移）→ 单段走短走长会互相抵消 */
    { M1_FWD,      0.0f },      /* 前进 1 米 */
    { M1_STRAFE_L, 0.0f },      /* 左移 1 米 */
    { M1_BACK,     0.0f },      /* 后退 1 米 → 回到起点 */
    { M1_STRAFE_R, 0.0f },      /* 右移 1 米（回起点那个角） */
#else
    /* ---- 转弯版：4 边 + 4 个左转（没 IMU 时闭不上，接 IMU 用这支）---- */
    { M1_FWD,      0.0f },      /* 第 1 边（0 = 用 CHASSIS_MISSION1_SIDE_MM） */
    { M1_TURN_L,  90.0f },
    { M1_FWD,      0.0f },      /* 第 2 边 */
    { M1_TURN_L,  90.0f },
    { M1_FWD,      0.0f },      /* 第 3 边 */
    { M1_TURN_L,  90.0f },
    { M1_FWD,      0.0f },      /* 第 4 边 → 回到起点 */
    { M1_TURN_L,  90.0f },      /* 把朝向转回起步方向 */
#endif
    { M1_STRAFE_L, 0.0f },      /* 左平移（0 = 用 CHASSIS_MISSION1_STRAFE_MM） */
    { M1_STRAFE_R, 0.0f },      /* 平移回来 */
    { M1_END,      0.0f }
};
#define M1_PLAN_NUM   ((uint8_t)(sizeof(s_m1_plan) / sizeof(s_m1_plan[0])))

/* 本步的目标值：表里写 0 就取 config.h 的边长/平移距离 */
static float m1_value(const m1_step_t *st)
{
    if (st->value > 0.0f) return st->value;
    if (st->act == M1_STRAFE_L || st->act == M1_STRAFE_R) return CHASSIS_MISSION1_STRAFE_MM;
    return CHASSIS_MISSION1_SIDE_MM;
}

static void Chassis_Mission1Task(void)
{
    const m1_step_t *st;
    uint32_t now = HAL_GetTick();
    float    v, d_mm, d_deg;
    uint8_t  done = 0u;

    if (s_m1_idx >= M1_PLAN_NUM) s_m1_idx = 0u;
    st = &s_m1_plan[s_m1_idx];

    /* ---------- 路线跑完 ---------- */
    if (st->act == M1_END)
    {
        Chassis_Stop();
#if CHASSIS_MISSION1_LOOP
        if ((uint32_t)(now - s_m1_tick) >= (uint32_t)CHASSIS_MISSION1_GAP_MS)
        {
            s_m1_idx = 0u; s_m1_sub = 0u; s_m1_started = 0u;
        }
#endif
        return;
    }

    /* ---------- 步间停顿 ---------- */
    if (s_m1_sub == 1u)
    {
        if ((uint32_t)(now - s_m1_tick) >= (uint32_t)CHASSIS_MISSION1_GAP_MS)
        {
            s_m1_idx++;
            s_m1_sub     = 0u;
            s_m1_started = 0u;
        }
        return;
    }

    v = m1_value(st);
    s_m1_target = v;

    /* ---------- 本步第一次进来：记起点 + 下发指令 ---------- */
    if (s_m1_started == 0u)
    {
        s_m1_started = 1u;
        s_m1_tick    = now;
        s_m1_mm0     = Chassis_GetDistanceMm();
        s_m1_deg0    = Chassis_GetYawDeg();
        s_m1_moved   = 0.0f;

        switch (st->act)
        {
        case M1_FWD:      Chassis_SetVelocity(+CHASSIS_MISSION1_SPEED, 0.0f, 0.0f); break;
        case M1_BACK:     Chassis_SetVelocity(-CHASSIS_MISSION1_SPEED, 0.0f, 0.0f); break;
        case M1_STRAFE_L: Chassis_SetVelocity(0.0f, +CHASSIS_MISSION1_SPEED, 0.0f); break;
        case M1_STRAFE_R: Chassis_SetVelocity(0.0f, -CHASSIS_MISSION1_SPEED, 0.0f); break;
        case M1_TURN_L:   Chassis_SetVelocity(0.0f, 0.0f, +CHASSIS_MISSION1_TURN_WZ); break;
        case M1_TURN_R:   Chassis_SetVelocity(0.0f, 0.0f, -CHASSIS_MISSION1_TURN_WZ); break;
        default: break;
        }
    }

    /* ---------- 本步完成没有 ---------- */
    d_mm  = Chassis_GetDistanceMm() - s_m1_mm0;      /* 累计路程增量（前后/平移都适用） */
    d_deg = Chassis_GetYawDeg() - s_m1_deg0;
    while (d_deg >  180.0f) d_deg -= 360.0f;
    while (d_deg < -180.0f) d_deg += 360.0f;

    switch (st->act)
    {
    case M1_FWD:
    case M1_BACK:
    case M1_STRAFE_L:
    case M1_STRAFE_R:
#if CHASSIS_MISSION1_JUDGE_BY_ODOM
        /* 用里程计判（需要里程计可靠；先看 VOFA ch4 会不会涨） */
        s_m1_moved = d_mm;
        done = (d_mm >= (v - CHASSIS_MISSION1_TOL_MM)) ? 1u : 0u;
#else
        /* 用时间判：本段该走 v 毫米，按 CHASSIS_MISSION1_SPEED 换算成时间
           （和 ⑧⑩ 测试同一套思路，不依赖任何反馈 → 里程计坏了也照样跑） */
        s_m1_moved = (float)((uint32_t)(now - s_m1_tick)) * CHASSIS_MISSION1_SPEED / 1000.0f;
        done = (s_m1_moved >= v) ? 1u : 0u;
        (void)d_mm;              /* 时间模式不看里程计（d_mm 只在里程计模式/保护里用） */
#endif
        break;

    case M1_TURN_L:
    case M1_TURN_R:
        if (IMU_IsOnline() != 0u)
        {
            s_m1_moved = (d_deg < 0.0f) ? -d_deg : d_deg;   /* IMU 在线：航向到点就停（真闭环） */
            done = (s_m1_moved >= (v - CHASSIS_MISSION1_TOL_DEG)) ? 1u : 0u;
        }
        else
        {
            /* 没 IMU（或 IMU_ENABLE=1 但没接上）：用 时间 × 角速度
               ⚠️ 这里用运行期判断而不是 #if IMU_ENABLE：
                  否则 IMU_ENABLE=1 但没接传感器时，航向恒为 0 → 判据永不满足 → 一直转 */
            s_m1_moved = (float)((uint32_t)(now - s_m1_tick)) * CHASSIS_MISSION1_TURN_WZ / 1000.0f;
            done = (s_m1_moved >= v) ? 1u : 0u;
        }
        break;

    default:
        break;
    }

#if CHASSIS_MISSION1_JUDGE_BY_ODOM
    /* ⚠️ 里程计"没动静"保护（只在"用里程计判"时才有意义）：
       距离类步骤跑了 1.5 秒、读数却几乎没涨 → 里程计/反馈有问题，别让它跑满超时 → 整条停 */
    if ((st->act == M1_FWD || st->act == M1_BACK ||
         st->act == M1_STRAFE_L || st->act == M1_STRAFE_R) &&
        (uint32_t)(now - s_m1_tick) >= 1500u && d_mm < 20.0f)
    {
        Chassis_Stop();
        s_m1_idx     = (uint8_t)(M1_PLAN_NUM - 1u);      /* 跳到 M1_END：整条停 */
        s_m1_sub     = 0u;
        s_m1_started = 0u;
        return;
    }
#endif

    /* 超时保护：判据永远不满足也不能一直开 */
    if (done == 0u && (uint32_t)(now - s_m1_tick) >= (uint32_t)CHASSIS_MISSION1_TIMEOUT_MS)
    {
        done = 1u;
    }

    if (done != 0u)
    {
        Chassis_Stop();                              /* 本步结束 */
        s_m1_started = 0u;
        if (CHASSIS_MISSION1_GAP_MS > 0)
        {
            s_m1_sub  = 1u;
            s_m1_tick = now;
        }
        else
        {
            s_m1_idx++;
        }
    }
}
#endif /* CHASSIS_ENABLE */



#if STEPPER_ENABLE && STEPPER_SELFTEST && !CHASSIS_ENABLE
/* ============================================================================
 *  步进电机自检（多台，按 config.h 的 STEPPER_MOTOR_TABLE 一台一台测）
 *
 *  每台电机的流程：
 *    ① 启动序列（每步隔 200ms，顺便满足手册"两条指令 ≥2ms"）：
 *         0xFB 清状态 → 0x62 设工作模式 → 0xFA 使能 → 0x32 读驱动参数
 *    ② 动作循环（周期 STEPPER_TEST_PERIOD_MS）：正转 → 停 → 反转 → 停
 *         位置模式则是：清零点 → 走一圈（STEPPER_TEST_PULSES）→ 回 0
 *    ③ 这台测完自动切下一台"启用"的电机
 *
 *  ⚠️ 判据：电机若"连续匀速转、从不停顿也不换向"→ 那是驱动器屏幕上的本地设置在跑，
 *     我们的串口命令没被执行 → 看 PD42S1_GetMotorState(i)->last_ack / mode / rx_ok
 * ============================================================================ */

/* 从 from 开始找下一台"启用"的电机，返回表索引；一台都没有则 0xFF */
static uint8_t Stepper_NextMotor(uint8_t from)
{
    uint8_t i;
    uint8_t n = PD42S1_GetMotorNum();

    for (i = 0u; i < n; i++)
    {
        uint8_t k = (uint8_t)((from + i) % n);
        const stepper_cfg_t *m = PD42S1_GetMotorCfg(k);

        if (m != 0 && m->use != 0u) return k;
    }
    return 0xFFu;
}

static void Stepper_TestTask(void)
{
    uint32_t now = HAL_GetTick();
    const stepper_cfg_t *m;
    uint8_t addr;

#if (STEPPER_TEST_MODE == 7)
    /* ===== 模式 7：地址扫描（不发运动指令）=====
       轮流给地址 1~7 发 0x32 读驱动参数，谁应答就说明：串口通了、地址是它
       看 VOFA：ch7 = 最后应答的电机号，ch9 = 收到应答条数（一直 0 = 串口没通） */
    if (s_step_start_tick == 0u)
    {
        s_step_start_tick = now;
        s_step_phase_tick = now;
        s_step_phase      = 1u;             /* 从地址 1 开始扫 */
    }
    if ((uint32_t)(now - s_step_phase_tick) < 500u) return;
    s_step_phase_tick = now;

    PD42S1_ReadDriveParams(s_step_phase);                    /* s_step_phase 当扫描地址用 */
    s_step_phase = (uint8_t)((s_step_phase >= 7u) ? 1u : (s_step_phase + 1u));
    return;
#endif

    /* ---------- 初始化：挑第一台启用的电机 ---------- */
    if (s_step_motor == 0xFFu) s_step_motor = Stepper_NextMotor(0u);
    m = PD42S1_GetMotorCfg(s_step_motor);
    if (m == 0) return;                     /* 表里一台都没启用 → 什么都不做 */
    addr = m->addr;

    /* ---------- 启动序列 ---------- */
    if (s_step_started == 0u)
    {
        if (s_step_start_tick == 0u)
        {
            s_step_start_tick = now;        /* 记录时刻，先等 200ms 让驱动器稳定 */
            return;
        }
        if ((uint32_t)(now - s_step_start_tick) < 200u) return;

        switch (s_step_boot)
        {
            case 0u:
                PD42S1_ClearState(addr);                    /* 0xFB：解除刹车/失能/堵转 */
                break;

            case 1u:
                PD42S1_ReadDriveParams(addr);               /* 0x32：先问清楚它现在是什么模式 */
                break;

            case 2u:
                /* ---- 决定用哪种命令（关键！别把驱动器原来的工作模式改坏）---- */
            {
#if   (STEPPER_TEST_MODE == 1) || (STEPPER_TEST_MODE == 2) || (STEPPER_TEST_MODE == 3)
                s_step_ol = 1u;                             /* 显式指定开环 */
#elif (STEPPER_TEST_MODE == 4) || (STEPPER_TEST_MODE == 5)
                s_step_ol = 0u;                             /* 显式指定闭环 */
                PD42S1_SetMode(addr, (STEPPER_TEST_MODE == 5) ? PD42S1_MODE_POS_COMM
                                                              : PD42S1_MODE_SPEED_COMM);
#else
                /* 模式 6：自动适配 —— 先看驱动器现在是什么模式 */
                {
                    const pd42s1_t *st = PD42S1_GetMotorState(s_step_motor);

                    if (st != 0 && st->mode_valid != 0u)
                    {
                        if (st->mode >= (uint8_t)PD42S1_MODE_OL_SPEED)
                        {
                            s_step_ol   = 1u;               /* 8/9/10 开环模式 → 发 0xE0，不动它的模式 */
                            s_step_auto = 0u;
                        }
                        else
                        {
                            s_step_ol   = 0u;               /* 通信类模式 → 发 0xF1 */
                            s_step_auto = 0u;

                            /* ⚠️ 关键：脉冲/脉宽/回零 这些模式下，串口运动指令是不生效的
                               （脉冲模式要 STP/DIR 引脚发脉冲，而我们板上没接 MCU），
                               所以必须先用 0x62 把它切到"通信速度模式" */
                            if (st->mode != (uint8_t)PD42S1_MODE_SPEED_COMM)
                            {
                                PD42S1_SetMode(addr, PD42S1_MODE_SPEED_COMM);
                            }
                        }
                    }
                    else
                    {
                        s_step_ol   = 0u;                   /* 读不到模式：先试闭环 */
                        s_step_auto = 1u;                   /* 一轮之后自动换开环再试 */
                    }
                }
#endif
            }
                break;

            case 3u:
                PD42S1_Enable(addr, 0u);                    /* 0xFA → 使能 */
                break;

            default:
                s_step_started    = 1u;                     /* 启动完成 */
                s_step_phase      = 0u;
                s_step_poll_tick  = now;
                s_step_phase_tick = now - (uint32_t)STEPPER_TEST_PERIOD_MS;
                return;                                     /* 下个循环立刻做第一个动作 */
        }

        s_step_boot++;
        s_step_start_tick = now;                            /* 下一步再等 200ms */
        return;
    }

    /* ---------- 指令调度：同一次循环里最多只发一条指令 ----------
       ⚠️ 底层有"两条指令间隔 ≥2ms"的手册保护。轮询(200ms) 和 动作(2000ms) 都是 200ms 的
          整数倍，会在同一次循环里连发两条 → 第二条（正好是"停/反转"这种动作指令）
          被间隔保护直接丢掉，表现就是【电机一直转、不停也不换向】。
          所以这里：动作优先，且发完就 return，绝不连发。 */
    if ((uint32_t)(now - s_step_phase_tick) >= (uint32_t)STEPPER_TEST_PERIOD_MS)
    {
        s_step_phase_tick = now;                    /* 到点：这一轮只发动作指令 */

        /* ⚠️ 手册 4.4.13：发了 0xFC 刹停后必须"及时清除电机状态"，否则电机严重发烫、
           而且会一直锁在刹车态（读回来 state=8），后面再发运动指令它也不会动！
           所以这里先补一条 0xFB，等 10ms（≥2ms 间隔）再发真正的动作指令。 */
        if (s_step_need_clear != 0u)
        {
            s_step_need_clear  = 0u;
            PD42S1_ClearState(addr);                /* 0xFB 解除刹车/堵转/失能 */
            s_step_phase_tick = (uint32_t)(now - (uint32_t)STEPPER_TEST_PERIOD_MS + 10u);
            return;
        }
    }
    else if ((uint32_t)(now - s_step_poll_tick) >= 200u)
    {
        s_step_poll_tick = now;                     /* 这一轮只发轮询读指令 */

        switch (s_step_poll_sel)
        {
            case 0u: s_step_poll_sel = 1u; PD42S1_ReadSpeed(addr);  break;
            case 1u: s_step_poll_sel = 2u; PD42S1_ReadPos(addr);    break;
            case 2u: s_step_poll_sel = 3u; PD42S1_ReadState(addr);  break;
            case 3u: s_step_poll_sel = 4u; PD42S1_ReadVolt(addr);   break;
            default: s_step_poll_sel = 0u; PD42S1_ReadPosErr(addr); break;
        }
        return;
    }
    else
    {
        return;                                     /* 什么都没到点 */
    }

#if (STEPPER_TEST_MODE == 1) || (STEPPER_TEST_MODE == 4) || (STEPPER_TEST_MODE == 6)
    /* ===== 速度模式：正转 → 停 → 反转 → 停 =====
       用闭环 0xF1 还是开环 0xE0 由 s_step_ol 决定（模式 6 按驱动器当前模式自动选） */
    {
        uint8_t nph = (STEPPER_TEST_PHASES >= 4u) ? 4u : 2u;

        switch (s_step_phase)
        {
            case 0u:                                        /* 正转 */
            case 2u:                                        /* 反转 */
            {
                uint8_t dir = (s_step_phase == 0u) ? 0u : 1u;   /* 逻辑方向：0 正 1 反 */

                if (s_step_ol != 0u)
                {
                    /* 开环路径：底层不做方向取反，这里自己套表里的 dir_inv */
                    uint8_t d = (m->dir_inv != 0u) ? (uint8_t)(dir ? 0u : 1u) : dir;
                    PD42S1_OpenLoopSpeed(addr, d, m->acc, m->speed_rpm);
                }
                else
                {
                    /* 闭环路径：PD42S1_MotorSpeed() 内部已经套过 dir_inv，这里传逻辑方向 */
                    PD42S1_MotorSpeed(s_step_motor, dir, m->speed_rpm);
                }
            }
                break;

            case 1u:                                        /* 停（2 相位模式下 = 反转） */
                if (nph >= 4u)
                {
                    PD42S1_MotorStop(s_step_motor);         /* 0xFC 刹车 */
                    s_step_need_clear = 1u;                 /* 下次动作前必须先 0xFB 清状态 */
                }
                else if (s_step_ol != 0u)
                {
                    uint8_t d = (m->dir_inv != 0u) ? 0u : 1u;
                    PD42S1_OpenLoopSpeed(addr, d, m->acc, m->speed_rpm);
                }
                else
                {
                    PD42S1_MotorSpeed(s_step_motor, 1u, m->speed_rpm);
                }
                break;

            default:                                        /* 停 */
                PD42S1_MotorStop(s_step_motor);             /* 0xFC 刹车 */
                s_step_need_clear = 1u;                     /* 下次动作前必须先 0xFB 清状态 */
                break;
        }

        s_step_phase = (uint8_t)((s_step_phase + 1u) % nph);
    }

#elif (STEPPER_TEST_MODE == 2)
    /* ===== 开环相对位置：每个周期走 N 个脉冲，方向交替 ===== */
    PD42S1_OpenLoopPosRel(addr,
                          (uint8_t)(s_step_phase & 1u),
                          m->acc,
                          (uint16_t)m->speed_rpm,
                          (uint32_t)m->pulses);
    s_step_phase = (uint8_t)((s_step_phase + 1u) % 2u);

#elif (STEPPER_TEST_MODE == 3) || (STEPPER_TEST_MODE == 5)
    /* ===== 位置模式（3 开环 / 5 闭环）：清零点 → 走一圈 → 回 0 ===== */
    switch (s_step_phase)
    {
        case 0u:
            PD42S1_AngleZero(addr);                         /* 0xF8：当前位置清零 */
            break;

        case 1u:
#if (STEPPER_TEST_MODE == 5)
            PD42S1_MotorPosAbs(s_step_motor, m->pulses);
#else
            PD42S1_OpenLoopPosAbs(addr, 0u, m->acc, (uint16_t)m->speed_rpm, m->pulses);
#endif
            break;

        default:
#if (STEPPER_TEST_MODE == 5)
            PD42S1_MotorPosAbs(s_step_motor, 0u);
#else
            PD42S1_OpenLoopPosAbs(addr, 0u, m->acc, (uint16_t)m->speed_rpm, 0u);
#endif
            break;
    }
    s_step_phase = (uint8_t)((s_step_phase + 1u) % 3u);

#elif (STEPPER_TEST_MODE == 7)
    /* ===== 模式 7：地址扫描（上面已经 return，这里不会执行）===== */
    (void)m;
    (void)addr;

#else
#error "config.h: STEPPER_TEST_MODE 只能是 1~7（1/4/6 速度，2 开环相对位置，3 开环绝对位置，5 闭环位置，7 地址扫描）"
#endif

    /* ---------- 一台测完 → 换下一台（相位回到 0 就算走完一圈） ---------- */
    if (s_step_phase == 0u)
    {
        uint8_t nxt = Stepper_NextMotor((uint8_t)(s_step_motor + 1u));

        /* 读不到驱动器模式时：这一轮用闭环，下一轮换开环，两种都试一遍 */
        if (s_step_auto != 0u) s_step_ol = (uint8_t)(s_step_ol ? 0u : 1u);

        if (nxt != 0xFFu && nxt != s_step_motor)
        {
            s_step_motor      = nxt;
            s_step_started    = 0u;         /* 新电机重新走一遍启动序列 */
            s_step_boot       = 0u;
            s_step_start_tick = 0u;
        }
    }
}
#endif /* STEPPER_ENABLE && STEPPER_SELFTEST && !CHASSIS_ENABLE */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
