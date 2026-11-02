/**
  ******************************************************************************
  * @file    gm6020.c
  * @brief   GM6020 CAN 驱动 + 位置环/速度环（1kHz）
  *
  *  防抽风措施（8 条：原有 7 条 + 新增的输出斜率限制）：
  *   1. 输出电流限幅            GIMBAL_CURRENT_MAX        （默认 3000 = 满量程 1/10）
  *   2. 积分限幅（抗饱和）       GIMBAL_POS/SPD_INTEGRAL_MAX
  *   3. 误差限幅                GIMBAL_POS/SPD_ERR_MAX
  *   4. 微分项低通              GIMBAL_D_LPF
  *   5. 目标角度缓变（斜率限制） GIMBAL_POS_SLEW_STEP      （默认 3 ecd/ms ≈ 79°/s）
  *   6. CAN 反馈超时 → 输出归零  GIMBAL_FEEDBACK_TIMEOUT_MS（30ms，CAN 回来自动继续）
  *   7. 上电延时 + 模式切换复位 PID  GIMBAL_STARTUP_DELAY_MS（300ms）
  *   8. 输出斜率限制（软启动/软刹车）GIMBAL_OUT_SLEW_STEP  （每 1ms 最多变 60）
  *  → 8 这条把"PID 一拍算出满输出"也压住了：任何情况下输出都是慢慢升上去的
  *  （过温/超速/堵转/软限位这些【故意没加】：GM6020 模块自带过温过流保护，
  *    机械行程限位等臂装好、知道实际限位之后再加更准 —— 需要时说一声，10 分钟能补）
  ******************************************************************************
  */

#include "gm6020.h"
#include "arm.h"        /* 升降臂 CAN 嗅探（ARM_PROBE_ENABLE=1 时在接收回调里转发一帧） */

/* ============================ 内部状态 ============================ */
static gm6020_measure_t s_meas;
static gm6020_pid_t     s_pos_pid;
static gm6020_pid_t     s_spd_pid;

static uint8_t  s_mode;

static uint16_t s_target_ecd;       /* 目标角度（ecd） */
static uint16_t s_target_ecd_now;   /* 缓变后的当前目标（ecd） */
static uint8_t  s_target_synced;    /* ★0 = 目标还没和实际位置对齐（上电后第一帧反馈时对齐，避免一复位就转） */
static int8_t   s_slew_dir;         /* ★强制旋转方向：0=最短路径, +1=角度递增, -1=角度递减
                                       （绕线保护：从大盘回初始位必须用 +1，不能让最短路径穿过 0°） */
static int16_t  s_target_rpm;       /* 目标转速（只有 CAN 闭环路径用到） */

static uint32_t s_start_tick;       /* 上电时刻 */
static int16_t  s_output;           /* 当前输出电流指令 */
static uint16_t s_tx_fail_cnt;      /* CAN 发送失败计数（调试用） */
static uint32_t s_rx_last_id;      /* ★诊断：最后收到的 6020 反馈 ID（0 = 还没收到过） */
static uint16_t s_rx_cnt;          /* ★诊断：收到过多少帧 0x205~0x20B */
static uint16_t s_tx_ok_cnt;       /* ★诊断：成功进邮箱的次数 */
static uint16_t s_loop_cnt;        /* ★诊断：Loop 被调用的次数（0 = TIM6 没跑） */
static uint8_t  s_can_start_ok;     /* HAL_CAN_Start() 是否成功（0 = 外设没起来，根本发不出去） */
static uint32_t s_target_tick;      /* 目标更新时刻（PWM 模式判断"到位"用） */
#if defined(GIMBAL_IFACE_CAN)
static float    s_out_prev;         /* 上一拍真实输出（输出斜率限制 / 软启动用） */
#endif

#if defined(GIMBAL_IFACE_PWM)
static TIM_HandleTypeDef *s_pwm_tim = 0;   /* PWM 定时器（main.c 里用 GM6020_PwmInit 绑定） */
static uint32_t           s_pwm_ch  = 0;
#endif

/* ============================ 小工具 ============================ */
static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* 角度(°) → PWM 脉宽(µs)：1080µs=0°，1920µs=360°（2.333µs/度） */
#if defined(GIMBAL_IFACE_PWM)
static uint16_t deg_to_pulse(float deg)
{
    float us;
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    us = GIMBAL_PWM_MIN_US + deg * (GIMBAL_PWM_MAX_US - GIMBAL_PWM_MIN_US) / 360.0f;
    us = clampf(us, GIMBAL_PWM_MIN_US, GIMBAL_PWM_MAX_US);
    return (uint16_t)(us + 0.5f);
}
#endif

static int16_t clamp_i16(int32_t v, int32_t lo, int32_t hi)
{
    if (v > hi) return (int16_t)hi;
    if (v < lo) return (int16_t)lo;
    return (int16_t)v;
}

/* 编码器误差（带过零点处理，结果 -4096 ~ +4095）
   例：目标 100、当前 8100，实际只需 -180 而不是 -8000 */
static int16_t ecd_error(uint16_t target, uint16_t now)
{
    int16_t err = (int16_t)((int32_t)target - (int32_t)now);
    if (err >  GIMBAL_ECD_HALF) err -= GIMBAL_ECD_RANGE;
    if (err < -GIMBAL_ECD_HALF) err += GIMBAL_ECD_RANGE;
    return err;
}

static void pid_reset(gm6020_pid_t *p)
{
    p->error      = 0.0f;
    p->last_error = 0.0f;
    p->integral   = 0.0f;
    p->d_state    = 0.0f;
    p->ff         = 0.0f;
    p->output     = 0.0f;
}

/* 通用 PID：位置式 + 误差限幅 + 积分限幅 + 微分低通 + 输出限幅
   err_max / i_max / out_max 传 <=0 表示不限（PWM 接口用不到，不参与编译） */
#if defined(GIMBAL_IFACE_CAN)
static float pid_calc(gm6020_pid_t *p, float error, float out_max, float err_max, float i_max)
{
    float d_raw, d_filt, out;

    /* --- 误差限幅：防止目标跳变时输出猛冲 --- */
    if (err_max > 0.0f) error = clampf(error, -err_max, err_max);
    p->error = error;

    /* --- 积分项 + 限幅（抗积分饱和） --- */
    p->integral += error;
    if (i_max > 0.0f) p->integral = clampf(p->integral, -i_max, i_max);

    /* --- 微分项 + 一阶低通（编码器噪声大时必须滤波，否则抖） --- */
    d_raw  = error - p->last_error;
    p->d_state = GIMBAL_D_LPF * p->d_state + (1.0f - GIMBAL_D_LPF) * d_raw;
    d_filt = p->d_state;

    /* --- 输出 --- */
    out = p->kp * p->error + p->ki * p->integral + p->kd * d_filt + p->ff;

    /* --- 输出限幅（最后一道闸） --- */
    if (out_max > 0.0f) out = clampf(out, -out_max, out_max);
    p->output = out;

    p->last_error = p->error;
    return out;
}

/* 输出斜率限制（★纯函数）：输出每 1ms 最多变化 step → 软启动 / 软刹车
   例：step=60、想给 3000，则要 50ms 才升满，任何一拍都不会甩出去 */
static float gimbal_slew_limit(float prev, float want, float step)
{
    if (step <= 0.0f) return want;
    if (want > (prev + step)) return prev + step;
    if (want < (prev - step)) return prev - step;
    return want;
}

#endif /* GIMBAL_IFACE_CAN */

/* ============================ CAN 收发 ============================ */
void GM6020_SendCurrent(int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t  data[8];
    uint32_t mailbox = 0;

    /* 双保险：再限一次幅，任何路径都不会把超范围的值发出去 */
    i1 = clamp_i16(i1, -GM6020_CURRENT_FULLSCALE, GM6020_CURRENT_FULLSCALE);
    i2 = clamp_i16(i2, -GM6020_CURRENT_FULLSCALE, GM6020_CURRENT_FULLSCALE);
    i3 = clamp_i16(i3, -GM6020_CURRENT_FULLSCALE, GM6020_CURRENT_FULLSCALE);
    i4 = clamp_i16(i4, -GM6020_CURRENT_FULLSCALE, GM6020_CURRENT_FULLSCALE);

    tx.StdId = GM6020_CTRL_ID_LOW;      /* 电机 ID 1~4 用 0x1FF */
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    data[0] = (uint8_t)((i1 >> 8) & 0xFF);
    data[1] = (uint8_t)(i1 & 0xFF);
    data[2] = (uint8_t)((i2 >> 8) & 0xFF);
    data[3] = (uint8_t)(i2 & 0xFF);
    data[4] = (uint8_t)((i3 >> 8) & 0xFF);
    data[5] = (uint8_t)(i3 & 0xFF);
    data[6] = (uint8_t)((i4 >> 8) & 0xFF);
    data[7] = (uint8_t)(i4 & 0xFF);

    /* 注意：官方 demo 这里写的是 (uint32_t*)CAN_TX_MAILBOX0 —— 那是把 0 当地址用，
       会往地址 0 写（Flash 别名），属于隐患。这里用真正的局部变量接收。 */
    if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) != HAL_OK)
    {
        s_tx_fail_cnt++;                /* 三个邮箱都满了才会失败，下次 1ms 再发 */
    }
    else
    {
        if (s_tx_ok_cnt < 0xFFFFu) s_tx_ok_cnt++;   /* ★诊断：成功进邮箱 */
    }
}

/* CAN 接收中断：解析 GM6020 反馈（0x205 ~ 0x20B） */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    if (hcan->Instance != CAN1) return;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) return;

#if ARM_ENABLE
    /* 升降臂模块也要看 CAN 上的帧：嗅探记录 / CANopen 位置(0x6064)应答解析都在它里面分派 */
    ARM_CanRx(&rx, data);
#endif

    if (rx.StdId >= GM6020_FEEDBACK_ID_BASE &&
        rx.StdId <  GM6020_FEEDBACK_ID_BASE + 7)
    {
        uint8_t idx = (uint8_t)(rx.StdId - GM6020_FEEDBACK_ID_BASE);   /* 0 = 电机ID1 */
        uint16_t ecd = (uint16_t)((data[0] << 8) | data[1]);

        /* ★诊断：不管 ID 是几，先记下来（这样能一眼看出 6020 的真实 ID） */
        s_rx_last_id = rx.StdId;
        if (s_rx_cnt < 0xFFFFu) s_rx_cnt++;

        if (1)     /* ★诊断阶段：任何 6020 ID 都采纳（拿到真实 ID 后再收紧） */
        {
            s_meas.last_ecd = s_meas.ecd;
            s_meas.ecd      = ecd;
            s_meas.speed_rpm = (int16_t)((data[2] << 8) | data[3]);
            s_meas.current   = (int16_t)((data[4] << 8) | data[5]);
            s_meas.temp      = data[6];

            /* 多圈累计（仅用于显示/记录） */
            {
                int16_t d = ecd_error(ecd, s_meas.last_ecd);
                s_meas.total_ecd += d;
            }

            s_meas.last_tick = HAL_GetTick();
            s_meas.online    = 1;
        }
    }
}

/* ============================ 初始化 ============================ */
void GM6020_Init(void)
{
    CAN_FilterTypeDef filter;

#if CAN_LOOPBACK_TEST
    /* ===== 回环自检模式（只在排查 CAN 硬件时打开）=====
       CAN_LOOPBACK_TEST = 1：内部回环（TX 引脚断开，不经过收发器）
            → 证明 MCU 侧外设/时序/收发/中断正常
       CAN_LOOPBACK_TEST = 2：静默回环（SILENT_LOOPBACK，**TX 引脚真的输出波形**，
            自己收自己的、不需要别人 ACK，总线上其它节点不会被打扰）
            → 用来判断"PA12 引脚到底有没有在输出"、"收发器和线通不通"
       ⚠️ 都会让真实节点收不到我们的帧，云台/驱动器不会动。查完改回 0。 */
    (void)HAL_CAN_Stop(&hcan1);
    hcan1.Init.Mode = (CAN_LOOPBACK_TEST == 2) ? CAN_MODE_SILENT_LOOPBACK : CAN_MODE_LOOPBACK;
    (void)HAL_CAN_Init(&hcan1);

    /* 回环也要"收所有 ID"，否则扩展帧（0x100x）过不了滤波 */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000u;
    filter.FilterIdLow          = 0x0000u;
    filter.FilterMaskIdHigh     = 0x0000u;
    filter.FilterMaskIdLow      = 0x0000u;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
#else
    /* 只接收 0x200~0x2FF（DJI 电机反馈区），其余 ID 不占中断 */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)((0x200 << 5) & 0xFFFF);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (uint16_t)((0x700 << 5) & 0xFFFF);
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;      /* 单 CAN 模式下无实际作用 */
#endif

    HAL_CAN_ConfigFilter(&hcan1, &filter);
    s_can_start_ok = (HAL_CAN_Start(&hcan1) == HAL_OK) ? 1u : 0u;
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 打开 CAN 接收中断（CubeMX 里没勾 NVIC 时，这里补上，否则收不到任何反馈）
       优先级设 1：比 TIM6 控制中断（0）低，控制环优先 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* 状态清零 */
    s_meas.ecd = 0;
    s_meas.last_ecd = 0;
    s_meas.total_ecd = 0;
    s_meas.speed_rpm = 0;
    s_meas.current = 0;
    s_meas.temp = 0;
    s_meas.last_tick = HAL_GetTick();
    s_meas.online = 0;
    s_meas.lost_cnt = 0;

    s_pos_pid.kp = GIMBAL_POS_KP;
    s_pos_pid.ki = GIMBAL_POS_KI;
    s_pos_pid.kd = GIMBAL_POS_KD;
    pid_reset(&s_pos_pid);

    s_spd_pid.kp = GIMBAL_SPD_KP;
    s_spd_pid.ki = GIMBAL_SPD_KI;
    s_spd_pid.kd = GIMBAL_SPD_KD;
    pid_reset(&s_spd_pid);

#if defined(GIMBAL_BASE_POSITION_MODE)
    s_mode = GM6020_MODE_POSITION;
#else
    s_mode = GM6020_MODE_SPEED;
#endif

    s_target_ecd     = 0;
    s_target_ecd_now = 0;
    s_target_rpm     = 0;
    s_output         = 0;
    s_tx_fail_cnt    = 0;
    s_start_tick     = HAL_GetTick();
}

/* ============================ PWM（舵机模式）初始化 ============================ */
#if defined(GIMBAL_IFACE_PWM)
void GM6020_PwmInit(TIM_HandleTypeDef *htim, uint32_t channel)
{
    s_pwm_tim = htim;
    s_pwm_ch  = channel;

    s_mode           = GM6020_MODE_POSITION;   /* PWM 只能位控 */
    s_target_ecd     = 0;
    s_target_ecd_now = 0;
    s_target_tick    = HAL_GetTick();
    s_start_tick     = HAL_GetTick();
    s_output         = 0;
    s_meas.ecd       = 0;
    s_meas.online    = 0;                      /* PWM 没有反馈，保持 0 */
    pid_reset(&s_pos_pid);
    pid_reset(&s_spd_pid);

    /* 先把脉宽设成 0°（1080µs）再启动，避免上电瞬间乱转 */
    __HAL_TIM_SetCompare(s_pwm_tim, s_pwm_ch, (uint32_t)deg_to_pulse(0.0f));
    HAL_TIM_PWM_Start(s_pwm_tim, s_pwm_ch);
}
#endif

/* ============================ 目标 / 模式 ============================ */
void GM6020_SetMode(uint8_t mode)
{
    if (mode == s_mode) return;
    s_mode = mode;
    /* 切模式必须复位 PID：否则切换瞬间的残余积分会让电机猛冲 */
    pid_reset(&s_pos_pid);
    pid_reset(&s_spd_pid);
    s_output = 0;
#if defined(GIMBAL_IFACE_CAN)
    s_out_prev = 0.0f;                  /* 输出斜率限制也重新从 0 起 */
#endif
}

uint8_t GM6020_GetMode(void)
{
    return s_mode;
}

/* ★带方向的转到某个角度（绕线保护）
   dir: 0 = 最短路径（默认）；+1 = 角度递增方向；-1 = 角度递减方向
   例：从 135.2° 回 344.5°，必须用 +1，否则最短路径会穿过 0° 把线绕死。 */
void GM6020_SetTargetPosDegDir(float deg, int8_t dir)
{
    if (dir > 0) dir = 1;
    if (dir < 0) dir = -1;
    s_slew_dir = dir;
    GM6020_SetTargetPosDeg(deg);
}

int8_t GM6020_GetSlewDir(void) { return s_slew_dir; }
void GM6020_SetTargetPos(uint16_t ecd)
{
    if (ecd >= GIMBAL_ECD_RANGE) ecd %= GIMBAL_ECD_RANGE;
    s_target_ecd  = ecd;
    s_target_tick = HAL_GetTick();          /* PWM 模式用它算"到位" */
}

void GM6020_SetTargetPosDeg(float deg)
{
    /* 角度归一化到 0~360 */
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    s_target_ecd  = (uint16_t)(deg * (float)GIMBAL_ECD_RANGE / 360.0f) % GIMBAL_ECD_RANGE;
    s_target_tick = HAL_GetTick();
}

void GM6020_SetTargetSpeed(int16_t rpm)
{
    s_target_rpm = rpm;

#if defined(GIMBAL_IFACE_PWM)
    /* PWM 舵机模式没有速度环，这个目标值用不到（写一行 (void) 只是为了消掉"未使用"警告） */
    (void)s_target_rpm;
#endif
}

void GM6020_Stop(void)
{
    pid_reset(&s_pos_pid);
    pid_reset(&s_spd_pid);
    s_output = 0;
    s_target_rpm = 0;
#if defined(GIMBAL_IFACE_CAN)
    s_out_prev = 0.0f;                  /* Stop 之后重新出力也是从 0 慢慢升 */
#endif

#if defined(GIMBAL_IFACE_PWM)
    /* PWM 模式：位置目标改成"当前角度"，即原地保持；
       想彻底掉力就 HAL_TIM_PWM_Stop(s_pwm_tim, s_pwm_ch)（整机安全还是靠断电/急停） */
    s_target_ecd     = s_target_ecd_now;
    s_target_tick    = HAL_GetTick();
#else
    GM6020_SendCurrent(0, 0, 0, 0);
#endif
}

/* ============================ 状态查询 ============================ */
uint8_t GM6020_IsOnline(void)
{
#if defined(GIMBAL_IFACE_PWM)
    /* PWM 模式没有角度反馈，这里返回"驱动是否就绪" */
    return (s_pwm_tim != 0) ? 1 : 0;
#else
    return s_meas.online;
#endif
}

int16_t GM6020_GetCurrentOutput(void) { return s_output; }
const gm6020_measure_t *GM6020_GetMeasure(void) { return &s_meas; }

/* 诊断（VOFA ch10/ch11）：
   tx_fail_cnt 一直在涨 = 三个发送邮箱一直是满的 → 帧根本没发出去（外设没启动/总线出错）
   can_ok = 0          = HAL_CAN_Start() 就失败了（时钟/引脚/外设配置问题）*/
uint16_t GM6020_GetTxFailCnt(void) { return s_tx_fail_cnt; }
uint8_t  GM6020_IsCanStarted(void) { return s_can_start_ok; }

float GM6020_GetCurrentDeg(void)
{
    return (float)s_meas.ecd * 360.0f / (float)GIMBAL_ECD_RANGE;
}

uint32_t GM6020_GetLastRxId(void) { return s_rx_last_id; }
uint16_t GM6020_GetRxCount(void)  { return s_rx_cnt; }
uint16_t GM6020_GetTxOkCount(void) { return s_tx_ok_cnt; }
uint16_t GM6020_GetTxFailCount(void){ return s_tx_fail_cnt; }
uint16_t GM6020_GetLoopCount(void)  { return s_loop_cnt; }
uint8_t  GM6020_GetCanStartOk(void) { return s_can_start_ok; }

float GM6020_GetTargetDeg(void)
{
    return (float)s_target_ecd_now * 360.0f / (float)GIMBAL_ECD_RANGE;
}

uint8_t GM6020_IsArrived(void)
{
#if defined(GIMBAL_IFACE_PWM)
    /* PWM 开环：发完目标等 GIMBAL_PWM_SETTLE_MS 就当到位（没法真的判断） */
    return ((HAL_GetTick() - s_target_tick) >= GIMBAL_PWM_SETTLE_MS) ? 1 : 0;
#else
    {
        int16_t err = ecd_error(s_target_ecd, s_meas.ecd);
        float e = (float)err;
        if (e < 0.0f) e = -e;
        return (e <= GIMBAL_POS_ARRIVE_ECD) ? 1 : 0;
    }
#endif
}

/* ============================ 1kHz 控制环 ============================ */
void GM6020_Loop(void)
{
    if (s_loop_cnt < 0xFFFFu) s_loop_cnt++;
#if defined(GIMBAL_IFACE_PWM)
    /* ================== PWM 舵机模式（无反馈，开环） ================== */
    uint16_t pulse;

    (void)s_target_rpm;      /* PWM 模式没有速度环（这个量只在 CAN 速控里读）→ 显式忽略 */

    if (s_pwm_tim == 0) return;                 /* 还没绑定定时器 */
    if ((HAL_GetTick() - s_start_tick) < GIMBAL_STARTUP_DELAY_MS) return;  /* 上电静默期 */

    /* 目标角度缓变（和 CAN 模式共用斜率参数） */
    if (GIMBAL_POS_SLEW_STEP > 0.0f)
    {
        int16_t d = ecd_error(s_target_ecd, s_target_ecd_now);
        float step = GIMBAL_POS_SLEW_STEP;
        if (d >  step) d =  (int16_t)step;
        if (d < -step) d = -(int16_t)step;
        {
            int32_t t = (int32_t)s_target_ecd_now + d;
            if (t < 0) t += GIMBAL_ECD_RANGE;
            if (t >= GIMBAL_ECD_RANGE) t -= GIMBAL_ECD_RANGE;
            s_target_ecd_now = (uint16_t)t;
        }
    }
    else
    {
        s_target_ecd_now = s_target_ecd;
    }

    /* 角度 → 脉宽 → 写 CCR（定时器 1µs 分辨率，所以 CCR 值就是 µs） */
    pulse = deg_to_pulse((float)s_target_ecd_now * 360.0f / (float)GIMBAL_ECD_RANGE);
    __HAL_TIM_SetCompare(s_pwm_tim, s_pwm_ch, (uint32_t)pulse);

    s_output = (int16_t)pulse;                  /* PWM 模式下这里存的是脉宽(µs)，方便 Watch/VOFA 看 */
    return;

#else
    /* ================== CAN 模式（有反馈，闭环） ================== */
    float out = 0.0f;        /* ★PID 输出（0x1FF 的电压/电流指令）
                                —— 之前这里漏了声明，CAN 分支从来没编译过所以没暴露 */
    (void)s_target_tick;     /* CAN 模式不读它（那是 PWM 模式判断"到位"用的）→ 显式忽略 */

    /* ---------- 1. 反馈超时检测 ---------- */
    if ((HAL_GetTick() - s_meas.last_tick) > GIMBAL_FEEDBACK_TIMEOUT_MS)
    {
        if (s_meas.online)
        {
            s_meas.online = 0;
            s_meas.lost_cnt++;
        }
    }
    else
    {
        s_meas.online = 1;
    }

    /* ---------- 2. 上电延时 / 失联 → 输出强制归零（CAN 一恢复就自动继续，不用手动清） ---------- */
    if ((HAL_GetTick() - s_start_tick) < GIMBAL_STARTUP_DELAY_MS || s_meas.online == 0)
    {
        pid_reset(&s_pos_pid);
        pid_reset(&s_spd_pid);
        s_output   = 0;
        s_out_prev = 0.0f;   /* 斜率限制的起点归零：恢复供电/重连后也是从 0 慢慢升 */
        GM6020_SendCurrent(0, 0, 0, 0);
        return;
    }

    /* ---------- 2.5 ★上电对齐：第一次拿到反馈时，把目标设成"当前角度" ----------
       目的：复位/上电后云台保持原地不动（以前是目标 0° → 一上电就往 0° 拽，可能绕线）。 */
    if (s_target_synced == 0u)
    {
        s_target_ecd     = s_meas.ecd;
        s_target_ecd_now = s_meas.ecd;
        s_target_synced  = 1u;
        pid_reset(&s_pos_pid);
        pid_reset(&s_spd_pid);
        s_output   = 0;
        s_out_prev = 0.0f;
        GM6020_SendCurrent(0, 0, 0, 0);
        return;
    }
    /* ---------- 3. 分模式计算 ---------- */
#if defined(GIMBAL_BASE_POSITION_MODE)
    /* ======== 位置环：目标角度 → 电流 ======== */
    s_mode = GM6020_MODE_POSITION;
    (void)s_target_rpm;                  /* 位控模式用不到目标转速 → 显式忽略（防 set-but-never-used） */

    /* 3.1 目标角度缓变（斜率限制）：防止目标一次跳几十度把电机甩起来 */
    if (GIMBAL_POS_SLEW_STEP > 0.0f)
    {
        if (s_slew_dir != 0)
        {
            /* ★定向缓变：只朝指定方向一格一格挪，走到（转过）目标就停。
               这样从 135° 回 344° 会走 +209° 的安全方向，而不是穿过 0° 的最短路径。 */
            int16_t d = ecd_error(s_target_ecd, s_target_ecd_now);

            if ((s_slew_dir > 0) && (d > 0))
            {
                int32_t t = (int32_t)s_target_ecd_now + (int32_t)GIMBAL_POS_SLEW_STEP;
                if (t >= GIMBAL_ECD_RANGE) t -= GIMBAL_ECD_RANGE;
                s_target_ecd_now = (uint16_t)t;
            }
            else if ((s_slew_dir < 0) && (d < 0))
            {
                int32_t t = (int32_t)s_target_ecd_now - (int32_t)GIMBAL_POS_SLEW_STEP;
                if (t < 0) t += GIMBAL_ECD_RANGE;
                s_target_ecd_now = (uint16_t)t;
            }
        }
        else
        {
            int16_t d = ecd_error(s_target_ecd, s_target_ecd_now);
            float step = GIMBAL_POS_SLEW_STEP;          /* 每 1ms 允许变化量 */
            if (d >  step) d =  (int16_t)step;
            if (d < -step) d = -(int16_t)step;
            {
                int32_t t = (int32_t)s_target_ecd_now + d;
                if (t < 0) t += GIMBAL_ECD_RANGE;
                if (t >= GIMBAL_ECD_RANGE) t -= GIMBAL_ECD_RANGE;
                s_target_ecd_now = (uint16_t)t;
            }
        }
    }
    else
    {
        s_target_ecd_now = s_target_ecd;
    }

    /* 3.2 位置误差（带过零点处理） */
    {
        int16_t err = ecd_error(s_target_ecd_now, s_meas.ecd);

        /* 3.3 前馈（默认 0，调好 PID 再考虑开） */
        s_pos_pid.ff = GIMBAL_POS_KFF_POS * (float)s_target_ecd_now;

        /* 3.4 PID → 电流指令 */
        out = pid_calc(&s_pos_pid, (float)err,
                       GIMBAL_CURRENT_MAX, GIMBAL_POS_ERR_MAX, GIMBAL_POS_INTEGRAL_MAX);
    }
#else
    /* ======== 速度环：目标转速 → 电流 ======== */
    s_mode = GM6020_MODE_SPEED;
    out = pid_calc(&s_spd_pid, (float)(s_target_rpm - s_meas.speed_rpm),
                   GIMBAL_CURRENT_MAX, GIMBAL_SPD_ERR_MAX, GIMBAL_SPD_INTEGRAL_MAX);
#endif

    /* ---------- 4. 输出限幅 + 斜率限制 + 发送 ---------- */
    out = clampf(out, -GIMBAL_CURRENT_MAX, GIMBAL_CURRENT_MAX);
    out = gimbal_slew_limit(s_out_prev, out, GIMBAL_OUT_SLEW_STEP);   /* ★软启动/软刹车：最后一道闸 */
    s_out_prev = out;
    s_output = (int16_t)out;

    /* 目前只有一台 6020（ID 1），占控制帧的第 1 个位置 */
    GM6020_SendCurrent(s_output, 0, 0, 0);
#endif  /* GIMBAL_IFACE_PWM */
}

/* ============================ CAN 接收中断入口 ============================ */
/* 你现在的 CubeMX 工程没有勾 "CAN1 RX0 interrupt"，所以 it.c 里没有这个函数，
   由本文件提供（否则中断向量会落到 startup 里的默认死循环，一收报文就卡死）。

   ⚠️ 如果以后在 CubeMX 里勾上了 CAN1 RX0 interrupt 并重新生成：
      it.c 会生成同名的 CAN1_RX0_IRQHandler → 必须把 config.h 里的
      GIMBAL_PROVIDE_CAN_IRQ_HANDLER 改成 0，否则链接期报重复定义。 */
#if GIMBAL_PROVIDE_CAN_IRQ_HANDLER
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
#endif

/* ============================ TIM6 中断回调 ============================ */
/* 说明：只有在 CubeMX 里启用了 TIM6 之后，HAL_TIM_MODULE_ENABLED 才会打开，
   这段回调才会参与编译。没启用 TIM6 时本文件依然能编译通过（只是没人调用 Loop）。
   如果以后还加了别的定时器中断，把它们的处理也合并到这里 */
#if defined(HAL_TIM_MODULE_ENABLED)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        GM6020_Loop();          /* 1kHz 控制环 */
    }
}
#endif
