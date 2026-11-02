/**
  ******************************************************************************
  * @file    arm.c
  * @brief   升降臂：编码器位置读取（串口 / CANopen）+ VOFA 拖条输入 + CAN 嗅探
  *
  *  三条职责，按 config.h ⑭ 区的开关编译：
  *    ① 位置：ARM_IFACE_UART=1 → 走串口问 PD42S1（0x2A 读位置）
  *            ARM_IFACE_UART=0 → 走 CAN，CANopen SDO 读 0x6064（位置实际值）
  *    ② 拖条：VOFA+ 控件发来的文本行里抓第一个数 → 当目标（限速下发，且只给"允许出力"的轴）
  *    ③ 嗅探：ARM_PROBE_ENABLE=1 时记录 CAN 上出现的所有 ID（诊断用）
  ******************************************************************************
  */
#include "arm.h"
#include "tune.h"       /* ARM_VofaRxIsr 把串口2 的字节转发给调参通道 */

#if ARM_ENABLE

#include "pd42s1.h"          /* 串口路径：直接用现成的驱动读反馈 */
#include "usart.h"           /* VOFA 那个串口（config.h 的 VOFA_UART，默认 huart2） */

static uint8_t s_vofa_rx_byte;   /* VOFA 拖条字节的落脚点（必须常驻内存） */

/* ============================ 状态 ============================ */
static int32_t  s_pos[ARM_AXIS_NUM];        /* 编码器位置（脉冲） */
static uint8_t  s_pos_valid[ARM_AXIS_NUM];
static float    s_slider_val[ARM_AXIS_NUM]; /* 每个滑块最近一次发来的值（轴①/轴②各一个） */
static uint16_t s_slider_cnt[ARM_AXIS_NUM]; /* 各收到过多少次 */
static int32_t  s_target[ARM_AXIS_NUM];     /* 各轴的目标脉冲 */
static uint32_t s_poll_tick;                /* 读位置的节拍 */
static uint32_t s_cmd_tick;                 /* 下发目标的节拍（限速） */
static char     s_line[24];                 /* 滑块文本行缓冲 */
static uint8_t  s_line_n;
static uint32_t s_line_tick;                /* 收到最后一个字节的时刻（兜底：控件不发换行也能解析） */
static uint16_t s_can_ext_cnt;              /* CAN 收到的扩展帧总数（诊断：驱动器到底答没答）
                                               GM6020 是标准帧、升降臂驱动器是扩展帧 → 只看扩展帧 */
static float    s_jog_rpm[2] = { (float)ARM_CAN_RPM, (float)ARM_CAN_RPM };  /* ★每轴运行速度（可在线改） */
static int32_t  s_virt_pos[2];      /* ★虚拟位置：累计【已下发的脉冲】（驱动器不回读时用它当位置） */
static uint16_t s_can_rx_total;             /* CAN 收到的【所有帧】总数（含标准帧）→ 判断"总线上有没有任何东西" */
static uint16_t s_can_tx_ok;                /* 成功放进发送邮箱的帧数（诊断：0 = 根本没在发） */
static uint16_t s_can_tx_fail;              /* 放邮箱失败的帧数 */
static uint8_t  s_can_filter_ok;            /* 扩展帧滤波器是否配置成功（0 = 配置失败，扩展帧进不来） */
static uint32_t s_last_ext_id;              /* 最后收到的扩展帧 ID（诊断） */
static uint8_t  s_last_ext_dlc;             /* 最后收到的扩展帧 DLC */
static uint8_t  s_last_ext_data[8];         /* 最后收到的扩展帧数据 */
#if ARM_CAN_BAUD_SCAN
/* 速率扫描状态：0 = 1Mbps / 1 = 500Kbps / 2 = 250Kbps */
static uint8_t  s_bscan_idx;                /* 当前档位 */
static uint8_t  s_bscan_hits;               /* bit0/1/2：该档位收到过扩展帧 */
static uint32_t s_bscan_tick;               /* 换档节拍 */
#endif
static uint8_t  s_listen_only;               /* 1 = 本段只收不发（纯监听） */
#if ARM_ADDR_SCAN
static uint16_t s_scan_mask;                /* 地址扫描结果：bit0 = 地址1 有应答 … */
#endif

/* ============ CAN 传输：正点原子自定义协议（★扩展帧，不是 CANopen）============
   依据 PD42S1 用户手册 6.2 节原文：
     · 仅支持扩展帧 (Extended Frame)
     · 报文 ID 必须符合 0x100x 格式（0x1001、0x100A、0x100F 均合法）
     · 协议内容（指令、数据格式、校验规则）与串口 TTL/RS485 版【完全一致】
     · 不支持 FDCAN，每条指令最多 8 字节
   ⇒ 做法：把【串口那套帧】原样塞进扩展帧的 8 字节数据区发出去；
           收到扩展帧就把数据区逐字节喂给现成的串口解析器（PD42S1_RxByte）。
   ★好处：位置/状态/转速这些解析全部沿用已经验证过的代码，不用重写。 */
#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART

/* CAN 发送样式（自动轮流试，每次换一种）：
     style 0：带串口外壳 C5…5C，DLC = 实际长度
     style 1：不带外壳，DLC = 实际长度
     style 2：带外壳，DLC = 8（后面补 0）
     style 3：不带外壳，DLC = 8（后面补 0）
   ID 的最后一位（0x100x 的 x）也一起轮 0x0~0xF：
   手册只说"0x1001、0x100A、0x100F 均合法"，没写哪一位有含义，
   所以 4 种格式 × 16 个 ID = 64 种组合全试（50ms 一发 → 3 秒扫完一轮）。 */
static void arm_can_send_serial(const uint8_t *frame, uint8_t len, uint8_t style, uint8_t id_digit)
{
    CAN_TxHeaderTypeDef tx;
    uint8_t  buf[16];                        /* 串口帧最长 13 字节（位置指令）*/
    uint8_t  d[8];
    uint32_t mb = 0u;
    uint8_t  i, n = 0u, dlc, off;
    uint8_t  strip = ((style == 1u) || (style == 3u)) ? 1u : 0u;
    uint8_t  dlc8  = ((style >= 2u)) ? 1u : 0u;

    if ((frame == 0) || (len == 0u)) return;

    /* ---- 1. 先按样式把要发的内容装进 buf（可能是去掉 C5/5C 外壳的） ---- */
    if (strip != 0u)
    {
        if ((len < 4u) || (frame[0] != 0xC5u) || (frame[len - 1u] != 0x5Cu)) return;
        for (i = 1u; (i + 1u) < len; i++) if (n < sizeof(buf)) buf[n++] = frame[i];
    }
    else
    {
        for (i = 0u; i < len; i++) if (n < sizeof(buf)) buf[n++] = frame[i];
    }

    if (n == 0u) return;

    /* ---- 2. ★分包发送：CAN 单帧最多 8 字节（手册 6.2「不支持 FDCAN」）
       位置(0xF2)/速度(0xF1) 这些指令串口帧是 11~13 字节，一帧塞不下 → 拆成多帧、
       同一 ID 连续发出去，让驱动器那边的字节流解析器拼起来。
       （以前这里遇到 n>8 直接 return，导致 CAN 路径下位置/速度指令根本发不出去） */
    for (off = 0u; off < n; off = (uint8_t)(off + 8u))
    {
        uint8_t chunk = (uint8_t)(n - off);

        if (chunk > 8u) chunk = 8u;

        for (i = 0u; i < chunk; i++) d[i] = buf[off + i];
        for (i = chunk; i < 8u; i++)  d[i] = 0u;

        dlc = (dlc8 != 0u) ? 8u : chunk;

        tx.StdId = 0u;
        tx.ExtId = ((uint32_t)ARM_CAN_TX_ID & 0xFFFFFFF0u) | (uint32_t)(id_digit & 0x0Fu);
        tx.IDE   = CAN_ID_EXT;
        tx.RTR   = CAN_RTR_DATA;
        tx.DLC   = dlc;

        /* ★计数：放进邮箱成功 = HAL_OK；失败（外设没启动/邮箱满）会返回其它值。
           诊断用：TX_OK 一直不涨 = CAN 根本没在发帧 */
        if (HAL_CAN_AddTxMessage(&hcan1, &tx, d, &mb) == HAL_OK)
        {
            s_can_tx_ok++;
        }
        else
        {
            s_can_tx_fail++;
        }

        /* 相邻两包之间略等一下，别把三个邮箱一次占满（1Mbps 下 8 字节 ≈ 100µs） */
        if ((uint8_t)(off + chunk) < n)
        {
            for (volatile uint32_t w = 0u; w < 2000u; w++) { }   /* ≈ 几十 µs 的空转 */
        }
    }
}

/* 用串口那套组帧函数组指令，再走 CAN 发（功能码/数据格式与串口完全一致） */
static void arm_can_cmd(uint8_t addr, uint8_t fct, const uint8_t *data, uint8_t dlen,
                        uint8_t style, uint8_t id_digit)
{
    uint8_t frame[16];
    uint8_t n = PD42S1_BuildFrame(addr, fct, data, dlen, frame);

    arm_can_send_serial(frame, n, style, id_digit);
}

static void arm_can_read_pos(uint8_t addr, uint8_t style, uint8_t id_digit)
{
    arm_can_cmd(addr, PD42S1_FCT_READ_POS, 0, 0u, style, id_digit);   /* 0x2A：读实时位置 */
}

/* ★CAN 版"相对走 N 个脉冲"（0xF3）—— 点动测试用：
   相对位置指令不需要知道当前绝对位置，正好用来验证"驱动器到底收不收我们的帧"。
   4 种样式全发（幂等：相对指令不是幂等的！所以只发一种最可能的 + 记录）
   ⚠️ 相对指令重复发会让它走两遍，所以这里只发一种样式（style 0）。*/
static void arm_can_move_rel(uint8_t addr, int32_t pulses, uint8_t acc, uint16_t rpm)
{
    uint8_t d[8];
    uint8_t dir = (pulses < 0) ? 1u : 0u;
    uint32_t u  = (uint32_t)((pulses < 0) ? -pulses : pulses);

    d[0] = dir;
    d[1] = acc;
    d[2] = (uint8_t)((rpm >> 8) & 0xFFu);
    d[3] = (uint8_t)((rpm >> 0) & 0xFFu);
    d[4] = (uint8_t)((u >> 24) & 0xFFu);
    d[5] = (uint8_t)((u >> 16) & 0xFFu);
    d[6] = (uint8_t)((u >> 8)  & 0xFFu);
    d[7] = (uint8_t)((u >> 0)  & 0xFFu);

    arm_can_cmd(addr, PD42S1_FCT_POS_REL, d, 8u, 0u, 0u);
}

/* ★CAN 版"走绝对位置"（0xF2）：定标时滑块/命令都走这个。
   数据布局与串口版一致：dir(1) acc(1) rpm(2,大端) pulses(4,大端)
   绝对位置指令是幂等的 → 4 种发送样式全发一遍也不会出错（谁被认就用谁）*/
static void arm_can_set_pos(uint8_t addr, int32_t pulses, uint8_t acc, uint16_t rpm)
{
    uint8_t d[8];
    uint8_t style;
    uint32_t u = (uint32_t)pulses;

    d[0] = 0u;                                   /* dir 由带符号的目标值决定，这里固定 0 */
    d[1] = acc;
    d[2] = (uint8_t)((rpm >> 8) & 0xFFu);
    d[3] = (uint8_t)((rpm >> 0) & 0xFFu);
    d[4] = (uint8_t)((u >> 24) & 0xFFu);
    d[5] = (uint8_t)((u >> 16) & 0xFFu);
    d[6] = (uint8_t)((u >> 8)  & 0xFFu);
    d[7] = (uint8_t)((u >> 0)  & 0xFFu);

    for (style = 0u; style < 4u; style++)
    {
        arm_can_cmd(addr, PD42S1_FCT_POS_ABS, d, 8u, style, 0u);
    }
}

/* ⚠️ 两个轴都允许出力（ALLOW_TORQUE=1）时没人调它 → 一起 #if 掉，免得吃 #177-D warning */
#if (!ARM_AXIS1_ALLOW_TORQUE) || (!ARM_AXIS2_ALLOW_TORQUE)
static void arm_can_enable(uint8_t addr, uint8_t en)
{
    uint8_t d[1];

    d[0] = en;                                              /* 0 = 使能，1 = 失能 */
    arm_can_cmd(addr, PD42S1_FCT_ENABLE, d, 1u, 0u, 0u);    /* 失能就用最常见的样式发一次 */
}
#endif
#endif /* ARM_IFACE_CAN_TOO || !ARM_IFACE_UART */

#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART || ARM_PROBE_ENABLE
/* 让 CAN1 收【所有】ID（掩码全 0）。
   ★走 CAN 读位置时这一步是必须的：CANopen 的应答是 0x580+节点号，
     落在 GM6020 自己那条 0x200~0x2FF 滤波之外 —— 不放行的话答复根本进不了中断，
     现象就是"VOFA 两个通道一直是 0，但接线全对"。
   GM6020 的解析不受影响（它自己判 ID 范围）。 */
static void arm_can_accept_all(void)
{
    CAN_FilterTypeDef f;

    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;

    /* ★必须"先停 → 改滤波 → 再启动"：F4 的 HAL 要求改滤波器时外设不处于运行态，
       否则 HAL_CAN_ConfigFilter 会静默失败（返回错误我们以前没检查）→
       滤波器还是 GM6020 那条 0x200~0x2FF → 【扩展帧 0x100x 全被硬件丢掉】，
       现象就是"驱动器明明回了、我们 ch5 一直是 0"。 */
    (void)HAL_CAN_Stop(&hcan1);
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
    {
        s_can_filter_ok = 0u;               /* 记下来：诊断通道可看 */
    }
    else
    {
        s_can_filter_ok = 1u;
    }
    (void)HAL_CAN_Start(&hcan1);
    (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

#if ARM_CAN_BAUD_SCAN
/* 换一档速率：重配 CAN1 的 Prescaler 并重建（APB1 = 45MHz，9 个时间片）
     档位 0 → PSC 5  = 1 Mbps（GM6020 那档）
     档位 1 → PSC 10 = 500 Kbps（PD42S1 CAN 版手册默认）
     档位 2 → PSC 20 = 250 Kbps
     档位 3 → PSC 40 = 125 Kbps（驱动器最低档）
     档位 4 → PSC 50 = 100 Kbps */
static void arm_can_set_baud(uint8_t idx)
{
    static const uint8_t k_psc[5] = { 5u, 10u, 20u, 40u, 50u };
    uint32_t psc = (idx < 5u) ? (uint32_t)k_psc[idx] : 5u;

    (void)HAL_CAN_Stop(&hcan1);
    (void)HAL_CAN_DeInit(&hcan1);           /* 会把 PA11/PA12/时钟一起复位，HAL_CAN_Init 里 MspInit 再配回来 */

    hcan1.Init.Prescaler = psc;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        s_can_filter_ok = 0u;
        return;
    }
    arm_can_accept_all();                   /* 重建后滤波器/中断都要重新挂（含 Start） */
}
#endif
#endif

/* ============================ CAN 嗅探 ============================ */
#if ARM_PROBE_ENABLE
#define ARM_PROBE_MAX   8u

typedef struct
{
    uint32_t id;
    uint8_t  ext;
    uint8_t  dlc;
    uint8_t  d[8];
    uint16_t cnt;
} arm_probe_slot_t;

static arm_probe_slot_t s_slot[ARM_PROBE_MAX];
static uint8_t  s_n;
static uint16_t s_total;
static uint8_t  s_overflow;
static uint16_t s_sent;
#if ARM_PROBE_SEND_CANOPEN
static uint32_t s_probe_tick;
#endif

/* GM6020 自己占用的 ID —— 嗅探时忽略，否则满屏都是它们 */
static uint8_t arm_is_gm6020(const CAN_RxHeaderTypeDef *rx)
{
    if (rx->IDE == CAN_ID_EXT) return 0u;

    if ((rx->StdId == 0x1FFu) || (rx->StdId == 0x2FFu) ||
        (rx->StdId == 0x1FEu) || (rx->StdId == 0x2FEu)) return 1u;
    if ((rx->StdId >= 0x205u) && (rx->StdId <= 0x20Bu)) return 1u;

    return 0u;
}

void ARM_ProbeInit(void)
{
    s_n        = 0u;
    s_total    = 0u;
    s_overflow = 0u;
    s_sent     = 0u;
#if ARM_PROBE_SEND_CANOPEN
    s_probe_tick = HAL_GetTick();
#endif

    arm_can_accept_all();          /* 嗅探就是要看全部 ID */
}

void ARM_ProbeRx(const CAN_RxHeaderTypeDef *rx, const uint8_t *data)
{
    uint8_t  i, b;
    uint32_t id  = (rx->IDE == CAN_ID_EXT) ? rx->ExtId : (uint32_t)rx->StdId;
    uint8_t  ext = (rx->IDE == CAN_ID_EXT) ? 1u : 0u;

    if (arm_is_gm6020(rx) != 0u) return;

    if (s_total < 0xFFFFu) s_total++;

    for (i = 0u; i < s_n; i++)
    {
        if ((s_slot[i].id == id) && (s_slot[i].ext == ext))
        {
            if (s_slot[i].cnt < 0xFFFFu) s_slot[i].cnt++;
            s_slot[i].dlc = (uint8_t)rx->DLC;
            for (b = 0u; b < 8u; b++) s_slot[i].d[b] = data[b];
            return;
        }
    }

    if (s_n < ARM_PROBE_MAX)
    {
        s_slot[s_n].id  = id;
        s_slot[s_n].ext = ext;
        s_slot[s_n].dlc = (uint8_t)rx->DLC;
        for (b = 0u; b < 8u; b++) s_slot[s_n].d[b] = data[b];
        s_slot[s_n].cnt = 1u;
        s_n++;
    }
    else
    {
        s_overflow = 1u;
    }
}

#if ARM_PROBE_SEND_CANOPEN && (ARM_IFACE_CAN_TOO || !ARM_IFACE_UART)
/* 主动探针：按【正点原子自定义协议】给两台驱动器发"读位置"请求（扩展帧 0x100x + 串口字节），
   这样只要驱动器在 CAN 模式收到了，就会回一帧 → 被嗅探记录到，就能看出它的应答 ID/格式。 */
static void arm_send_read_probe(uint8_t addr)
{
    arm_can_read_pos(addr, 0u, 0u);
    if (s_sent < 0xFFFFu) s_sent++;
}
#endif

void ARM_ProbeTask(void)
{
#if ARM_PROBE_SEND_CANOPEN && (ARM_IFACE_CAN_TOO || !ARM_IFACE_UART)
    static uint8_t s_which = 0u;

    if ((uint32_t)(HAL_GetTick() - s_probe_tick) >= 500u)
    {
        s_probe_tick = HAL_GetTick();
        arm_send_read_probe(s_which == 0u ? (uint8_t)ARM_NODE_1 : (uint8_t)ARM_NODE_2);
        s_which = (uint8_t)((s_which == 0u) ? 1u : 0u);
    }
#endif
}

void ARM_ProbeClear(void)
{
    uint8_t i, b;

    for (i = 0u; i < ARM_PROBE_MAX; i++)
    {
        s_slot[i].id = 0u; s_slot[i].ext = 0u; s_slot[i].dlc = 0u; s_slot[i].cnt = 0u;
        for (b = 0u; b < 8u; b++) s_slot[i].d[b] = 0u;
    }
    s_n = 0u; s_total = 0u; s_overflow = 0u;
}

uint16_t ARM_ProbeFrameCount(void) { return s_total; }
uint8_t  ARM_ProbeDistinct(void)   { return s_n; }
uint8_t  ARM_ProbeOverflow(void)   { return s_overflow; }
uint16_t ARM_ProbeSent(void)       { return s_sent; }
uint32_t ARM_ProbeId(uint8_t i)    { return (i < ARM_PROBE_MAX) ? s_slot[i].id : 0u; }
uint8_t  ARM_ProbeIsExt(uint8_t i) { return (i < ARM_PROBE_MAX) ? s_slot[i].ext : 0u; }
uint8_t  ARM_ProbeDlc(uint8_t i)   { return (i < ARM_PROBE_MAX) ? s_slot[i].dlc : 0u; }
uint16_t ARM_ProbeCount(uint8_t i) { return (i < ARM_PROBE_MAX) ? s_slot[i].cnt : 0u; }

uint8_t ARM_ProbeData(uint8_t i, uint8_t b)
{
    if ((i >= ARM_PROBE_MAX) || (b >= 8u)) return 0u;
    return s_slot[i].d[b];
}

/* CAN 错误计数器（诊断"速率不匹配"用）：
   模块传 1M，某个步进是 500K → 它发出的帧在我们这边全是错误帧 → REC/TEC 会涨
   ecr 返回: [0] = TEC(发送错误计数), [1] = REC(接收错误计数), [2] = LEC(最近错误码) */
void ARM_ProbeErrorCounters(uint16_t *tec, uint16_t *rec, uint8_t *lec)
{
    uint32_t esr = hcan1.Instance->ESR;

    if (tec != 0) *tec = (uint16_t)((esr >> 16) & 0xFFu);
    if (rec != 0) *rec = (uint16_t)((esr >> 24) & 0xFFu);
    if (lec != 0) *lec = (uint8_t)(esr & 0x07u);
}
#endif /* ARM_PROBE_ENABLE */

/* ============================ CAN 接收总入口 ============================ */
/* 由 gm6020.c 的接收中断回调转发进来（CAN1 收到的每一帧都会到这）
   在这里分发给：① 嗅探记录（若开着）② 驱动器应答 → 喂给串口驱动的解析器 */
void ARM_CanRx(const CAN_RxHeaderTypeDef *rx, const uint8_t *data)
{
    uint8_t i, n;

    /* ★"总线上有没有扩展帧"的总计数（诊断用，VOFA ch5 就发它）
       为什么要单独计：GM6020 用的是【标准帧 0x205】、升降臂驱动器用【扩展帧 0x100x】
       → 只数扩展帧才能判断"两台步进到底答没答"（GM6020 的帧不会污染这个数）*/
    /* ★"总线上有没有任何东西"的总计数（含标准帧）：GM6020 的 0x205 也算
       → 它涨 = 至少有一个节点在说话（不管是谁）；一直 0 = 总线完全没动静 */
    s_can_rx_total++;

    if (rx->IDE == CAN_ID_EXT)
    {
        s_can_ext_cnt++;
        /* 记下"最后收到的扩展帧"长什么样（诊断：驱动器到底回的是哪个 ID、什么载荷） */
        s_last_ext_id  = (uint32_t)(rx->ExtId & 0x1FFFFFFFu);
        s_last_ext_dlc = (uint8_t)((rx->DLC > 8u) ? 8u : rx->DLC);
        for (i = 0u; i < 8u; i++)
        {
            s_last_ext_data[i] = (i < s_last_ext_dlc) ? data[i] : 0u;
        }
#if ARM_CAN_BAUD_SCAN
        s_bscan_hits |= (uint8_t)(1u << s_bscan_idx);   /* 记下"这一档收到过帧" */
#endif
    }

#if ARM_PROBE_ENABLE
    ARM_ProbeRx(rx, data);
#endif

#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
    /* 驱动器回的是【扩展帧】，载荷是串口那套字节（手册：数据内容与串口完全一致）
       ★两种都可能：带串口外壳（C5 … 5C）或只带 `地址 功能码 数据… 校验`（正好 8 字节放得下）
         → 自动识别：判断"不带壳的这一帧是不是一条【完整】帧"（末尾校验 = C5+前面所有字节），
           是就补上 0xC5/0x5C 再喂给解析器；不是（多帧应答的续帧）就原样喂、让流式解析器接着拼。 */
    if (rx->IDE == CAN_ID_EXT)
    {
        n = (uint8_t)rx->DLC;
        if (n > 8u) n = 8u;

        if ((n > 0u) && (data[0] == 0xC5u))
        {
            for (i = 0u; i < n; i++) PD42S1_RxByte(data[i]);        /* 已带外壳：直接用 */
        }
        else if (n >= 3u)
        {
            uint8_t sum = 0xC5u;

            for (i = 0u; (i + 1u) < n; i++) sum = (uint8_t)(sum + data[i]);

            if (sum == data[n - 1u])
            {
                PD42S1_RxByte(0xC5u);                                   /* 完整"无壳"帧：补头 */
                for (i = 0u; i < n; i++) PD42S1_RxByte(data[i]);
                PD42S1_RxByte(0x5Cu);                                   /* 补尾 */
            }
            else
            {
                for (i = 0u; i < n; i++) PD42S1_RxByte(data[i]);        /* 续帧：原样喂 */
            }
        }
    }
#endif

    (void)rx;
    (void)data;
}

/* ============================ 拖条文本解析 ============================ */
/* 支持【两个滑块各控一个轴】：行的开头用字母区分
       `A=12000` 或 `a:12000`  → 轴①（螺杆）
       `B=8000`  或 `b:8000`   → 轴②（齿条）
       不带字母（如 `12000`）  → 两个轴都按这个值走
   数值本身随便：可以有负号、小数点；VOFA+ 控件里的命令写 `A={value}` 即可。
   ⚠️ 发送格式要在 VOFA+ 里选【文本/ASCII】，别选十六进制/二进制。 */
static void arm_parse_line(void)
{
    uint8_t i;
    uint8_t axis = 0xFFu;               /* 0xFF = 没指定 → 两个轴都发 */
    uint8_t started = 0u, neg = 0u, in_frac = 0u;
    float   val = 0.0f, div = 10.0f;

    /* 1) 先看前缀字母决定给哪个轴 */
    for (i = 0u; i < s_line_n; i++)
    {
        char c = s_line[i];

        if ((c == 'a') || (c == 'A')) { axis = 0u; break; }
        if ((c == 'b') || (c == 'B')) { axis = 1u; break; }
        if (((c >= '0') && (c <= '9')) || (c == '-')) break;   /* 先碰到数字 → 没有前缀 */
    }

    /* 2) 再抓这一行里第一个数 */
    for (i = 0u; i < s_line_n; i++)
    {
        char c = s_line[i];

        if (started == 0u)
        {
            if (c == '-') { neg = 1u; started = 1u; }
            else if ((c >= '0') && (c <= '9')) { started = 1u; val = (float)(c - '0'); }
            continue;
        }

        if ((c >= '0') && (c <= '9'))
        {
            if (in_frac != 0u) { val += (float)(c - '0') / div; div *= 10.0f; }
            else               { val = val * 10.0f + (float)(c - '0'); }
        }
        else if (c == '.')
        {
            in_frac = 1u;
        }
        else
        {
            break;                      /* 数字结束 */
        }
    }

    if (started == 0u) return;          /* 这行里没有数字 → 忽略 */

    /* ★安全：必须带 A=/B= 前缀才认（原来"不带字母 = 两个轴一起走"，
       有了调参通道后很危险：PC 发 `U1500`/`G400`/`T90` 这种命令会被这里
       当成"把螺杆目标设成 1500/400/90 脉冲" → 螺杆自己动！所以去掉那个兜底行为）*/
    if (axis == 0xFFu) return;

    if (neg != 0u) val = -val;

    {
        uint8_t from = (axis == 0xFFu) ? 0u : axis;
        uint8_t to   = (axis == 0xFFu) ? (uint8_t)(ARM_AXIS_NUM - 1u) : axis;
        uint8_t a;

        for (a = from; a <= to; a++)
        {
            s_slider_val[a] = val;
            if (s_slider_cnt[a] < 0xFFFFu) s_slider_cnt[a]++;
#if ARM_SLIDER_IS_DEG
            s_target[a] = (int32_t)(val * 51200.0f / 360.0f);   /* 滑块给的是角度 → 换成脉冲 */
#else
            s_target[a] = (int32_t)val;                         /* 滑块给的就是脉冲 */
#endif
        }
    }
}

void ARM_RxByte(uint8_t b)
{
    s_line_tick = HAL_GetTick();

    if ((b == (uint8_t)'\n') || (b == (uint8_t)'\r'))
    {
        if (s_line_n > 0u) arm_parse_line();
        s_line_n = 0u;
        return;
    }

    if (s_line_n < (uint8_t)(sizeof(s_line) - 1u))
    {
        s_line[s_line_n] = (char)b;
        s_line_n++;
        s_line[s_line_n] = '\0';
    }
    else
    {
        /* 缓冲满了：当一行结束处理，然后重开（避免一直卡死） */
        arm_parse_line();
        s_line_n = 0u;
    }
}

/* VOFA 串口收到一个字节（由 pd42s1.c 的接收回调按 VOFA_UART 转发进来）
   —— 收文本，喂给拖条解析，然后马上挂上下一个字节
   ★同时把字节转给调参通道（tune.c）：这样"串口2 一条线"既能发滑块命令（A=/B=）
     又能发调参命令（G/T/U/O/C...），接收只由这里一个人负责挂，不会两边抢缓冲。*/
void ARM_VofaRxIsr(UART_HandleTypeDef *huart)
{
    ARM_RxByte(s_vofa_rx_byte);

#if TUNE_ENABLE
    TUNE_FeedByte((char)s_vofa_rx_byte);
#endif

#if defined(HAL_UART_MODULE_ENABLED)
    /* ★按"实际是哪个口来的字节"重新挂接收：主口可能是串口2（有线）也可能是
       串口4（蓝牙），挂错了下一个字节就收不到了 */
    if (huart == 0) huart = VOFA_UART;
    (void)HAL_UART_Receive_IT(huart, &s_vofa_rx_byte, 1u);
#endif
}

/* 重新挂上 VOFA 主口的接收（一路 1 字节中断）
   ★用途：蓝牙当主口时，BT_Init() 里 HAL_UART_Init 重配 9600 会把已挂的接收断掉，
     所以 main 在 BT_Init() 之后必须再调一次这里 */
void ARM_RearmRx(void)
{
#if defined(HAL_UART_MODULE_ENABLED)
    (void)HAL_UART_Receive_IT(VOFA_UART, &s_vofa_rx_byte, 1u);
#endif
}

/* ============================ 初始化 / 周期任务 ============================ */
void ARM_Init(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)ARM_AXIS_NUM; i++)
    {
        s_pos[i]        = 0;
        s_pos_valid[i]  = 0u;
        s_slider_val[i] = 0.0f;
        s_slider_cnt[i] = 0u;
        s_target[i]     = 0;
    }
    s_line_n     = 0u;
    s_line_tick  = HAL_GetTick();
    s_poll_tick  = HAL_GetTick();
    s_cmd_tick   = s_poll_tick;

    /* VOFA 那个串口（默认串口2 = PA2/PA3）挂上接收：拖条发下来的文本从这进来
       注意：VOFA 平时是"只发不收"，这里开接收是为了收控件（拖条/按钮）的值 */
#if defined(HAL_UART_MODULE_ENABLED)
    (void)HAL_UART_Receive_IT(VOFA_UART, &s_vofa_rx_byte, 1u);
#endif

#if ARM_PROBE_ENABLE
    ARM_ProbeInit();
#endif

#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
    /* ★驱动器应答是扩展帧，ID 不在 GM6020 那条 0x200~0x2FF 滤波范围内
       → 必须把滤波器放开到"收所有 ID"，否则应答进不了中断（现象就是两路一直 0） */
    arm_can_accept_all();
#endif

#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
    /* ★定标阶段"不给力"：把不允许出力的轴【失能】（这样齿条能手推得动） */
#if !ARM_AXIS1_ALLOW_TORQUE
    arm_can_enable((uint8_t)ARM_NODE_1, 1u);
#endif
#if !ARM_AXIS2_ALLOW_TORQUE
    arm_can_enable((uint8_t)ARM_NODE_2, 1u);
#endif
#endif

#if ARM_IFACE_UART || ARM_IFACE_CAN_TOO
    /* ★先确保两台驱动器工作在【通信位置模式】(0x62 = 0)
       —— 新驱动器出厂可能是"脉冲模式"，那样串口发的位移命令会被直接忽略
          （现象：命令发出去了、驱动器也应答了，但电机一动不动） */
    PD42S1_SetMode((uint8_t)ARM_NODE_1, 0u);
    PD42S1_SetMode((uint8_t)ARM_NODE_2, 0u);

    /* 串口那条也发一遍同样的"失能"（PD42S1_Enable(addr, 1) = 失能，0 = 使能）
       —— 板子 5/6 口就是这条总线，臂电机本来就插在上面 */
#if !ARM_AXIS1_ALLOW_TORQUE
    PD42S1_Enable((uint8_t)ARM_NODE_1, 1u);
#else
    PD42S1_Enable((uint8_t)ARM_NODE_1, 0u);      /* ★允许出力的轴要显式【使能】 */
#endif
#if !ARM_AXIS2_ALLOW_TORQUE
    PD42S1_Enable((uint8_t)ARM_NODE_2, 1u);
#else
    PD42S1_Enable((uint8_t)ARM_NODE_2, 0u);
#endif
#endif
}

void ARM_Task(void)
{
    uint32_t now = HAL_GetTick();

#if ARM_CAN_BAUD_SCAN && (ARM_IFACE_CAN_TOO || !ARM_IFACE_UART || ARM_PROBE_ENABLE)
    /* ---------- 0. 速率扫描：每 ARM_CAN_SCAN_MS 换一档 ----------
       ★每档再切成两段：
           前半段 ARM_CAN_LISTEN_MS = 只收不发（纯监听，看驱动器会不会自己开口）
           后半段 = 正常发读位置指令（看它回不回）
       为什么要监听：一直发帧可能把速率不对的驱动器逼进 bus-off，它就不吭声了 */
    {
        if (s_bscan_tick == 0u)
        {
            s_bscan_tick = now;             /* 开机先按 1M 跑一会儿再换档 */
        }
        else if ((uint32_t)(now - s_bscan_tick) >= (uint32_t)ARM_CAN_SCAN_MS)
        {
            s_bscan_tick = now;
            s_bscan_idx  = (uint8_t)((s_bscan_idx >= 4u) ? 0u : (s_bscan_idx + 1u));
            s_can_ext_cnt = 0u;             /* 每档单独从 0 数 */
            arm_can_set_baud(s_bscan_idx);
        }

        s_listen_only = ((uint32_t)(now - s_bscan_tick) < (uint32_t)ARM_CAN_LISTEN_MS) ? 1u : 0u;
    }
#endif

    /* ---------- 0b. 兜底：控件要是没发换行，就按"串口静了 20ms"当一行结束 ---------- */
    if ((s_line_n > 0u) && ((uint32_t)(now - s_line_tick) >= 20u))
    {
        arm_parse_line();
        s_line_n = 0u;
    }

    /* ---------- 1. 读两个编码器 ----------
       ★两条路【同时发】（ARM_IFACE_CAN_TOO=1）：
         · 串口那条：板上 5/6 口就是这条总线，臂电机本来就插在上面
         · CAN 那条：扩展帧 0x100x + 串口字节；带壳/不带壳交替发（两种都可能）
       收到的字节都喂给同一个解析器，位置就填进驱动状态里 —— 谁有回应就用谁，
       不用猜"驱动器到底是 TTL 版还是 CAN 版"。 */
    {
        static uint8_t s_which = 0u;

        if ((uint32_t)(now - s_poll_tick) >= (uint32_t)ARM_POLL_MS)
        {
            static uint8_t s_style = 0u;      /* 载荷格式轮 0~3 */
            static uint8_t s_id_digit = 0u;   /* 0x100x 的 x 轮 0~F */
            uint8_t addr = (s_which == 0u) ? (uint8_t)ARM_NODE_1 : (uint8_t)ARM_NODE_2;

#if ARM_DIAG_SWEEP
            /* ★诊断模式：地址 0~10 连续轮（驱动器菜单 ⑦ 的 ID 范围就是 0~10），
               配合 4 种载荷格式一起轮 → 只要它认任何一种，就会回一帧，
               而且 ID/载荷会被记下来（VOFA 上看"最后一个扩展帧"那几路） */
            {
                static uint8_t s_diag_addr = 0u;

                addr = s_diag_addr;
                s_diag_addr = (uint8_t)((s_diag_addr >= 10u) ? 0u : (s_diag_addr + 1u));
            }
#elif ARM_ADDR_SCAN
            /* 地址扫描：开机先轮流问 1~7 两轮（≈0.7s），谁答谁进 ARM_ScanMask()；
               扫完就恢复正常，只问 5/6，这样定标时位置刷新还是快的 */
            static uint8_t s_scan_addr = 1u;
            static uint8_t s_scan_left = 14u;

            if (s_scan_left > 0u)
            {
                addr = s_scan_addr;
                s_scan_addr = (uint8_t)((s_scan_addr >= 7u) ? 1u : (s_scan_addr + 1u));
                s_scan_left--;
            }
#endif

            s_poll_tick = now;
            s_which = (uint8_t)((s_which == 0u) ? 1u : 0u);

#if ARM_IFACE_UART || ARM_IFACE_CAN_TOO
            PD42S1_ReadPos(addr);             /* 串口总线上点名读位置（0x2A） */
#endif
#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
            /* ★纯监听那一段不发任何 CAN 帧（否则可能把驱动器逼进 bus-off，它就不说话了） */
            if (s_listen_only == 0u)
            {
                arm_can_read_pos(addr, s_style, s_id_digit);   /* CAN：4 种格式 × 16 个 ID 全试 */
                s_style = (uint8_t)((s_style >= 3u) ? 0u : (s_style + 1u));
                if (s_style == 0u) s_id_digit = (uint8_t)((s_id_digit + 1u) & 0x0Fu);
            }
#endif
        }

        {
            const pd42s1_t *st;
#if ARM_ADDR_SCAN
            uint8_t i;                    /* 只有开地址扫描时才用得到 */
#endif

            st = PD42S1_GetMotorState((uint8_t)ARM_AXIS1_IDX);
            if ((st != 0) && (st->pos_valid != 0u)) { s_pos[0] = st->pos; s_pos_valid[0] = 1u; }

            st = PD42S1_GetMotorState((uint8_t)ARM_AXIS2_IDX);
            if ((st != 0) && (st->pos_valid != 0u)) { s_pos[1] = st->pos; s_pos_valid[1] = 1u; }

#if ARM_ADDR_SCAN
            /* 地址扫描：表里 7 台，谁的位置有效就把对应位置 1（bit0 = 地址 1） */
            for (i = 0u; i < 7u; i++)
            {
                const pd42s1_t *s2 = PD42S1_GetMotorState(i);

                if ((s2 != 0) && (s2->pos_valid != 0u)) s_scan_mask |= (uint16_t)(1u << i);
            }
#endif
        }
    }

    /* ---------- 2. 滑块：只给"允许出力"的轴下发，并限速 ----------
       两个滑块各控一个轴（A=… 轴①螺杆 / B=… 轴②齿条），互不影响；
       没允许出力的轴即使滑块在动也只记值、不下发（定标阶段就这样）
       ★原来这里被 #if ARM_IFACE_UART 包着 → 走 CAN 时滑块只记值、电机不动；
         现在两种接口都下发（串口走 PD42S1_MotorPosAbs，CAN 走 arm_can_set_pos）*/
    if ((uint32_t)(now - s_cmd_tick) >= (uint32_t)ARM_SLIDER_CMD_MS)
    {
        s_cmd_tick = now;

#if (ARM_AXIS1_ALLOW_TORQUE || ARM_AXIS2_ALLOW_TORQUE)
        if (s_slider_cnt[0] != 0u)
        {
#if ARM_AXIS1_ALLOW_TORQUE
#if ARM_IFACE_UART
            PD42S1_MotorPosAbs((uint8_t)ARM_AXIS1_IDX, (uint32_t)s_target[0]);   /* 串口：绝对位置 0xF2 */
#endif
#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
            arm_can_set_pos((uint8_t)ARM_NODE_1, s_target[0], ARM_CAN_ACC, ARM_CAN_RPM);
#endif
#endif
        }
        if (s_slider_cnt[1] != 0u)
        {
#if ARM_AXIS2_ALLOW_TORQUE
#if ARM_IFACE_UART
            PD42S1_MotorPosAbs((uint8_t)ARM_AXIS2_IDX, (uint32_t)s_target[1]);
#endif
#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
            arm_can_set_pos((uint8_t)ARM_NODE_2, s_target[1], ARM_CAN_ACC, ARM_CAN_RPM);
#endif
#endif
        }
#endif
    }
}

/* ============================ 查询 ============================ */float ARM_GetPosPulses(uint8_t axis)
{
    if (axis >= (uint8_t)ARM_AXIS_NUM) return 0.0f;
    return (float)s_pos[axis];
}

float ARM_GetPosDeg(uint8_t axis)
{
    float d;
    if (axis >= (uint8_t)ARM_AXIS_NUM) return 0.0f;

    d = (float)s_pos[axis] * 360.0f / 51200.0f;      /* 51200 脉冲 = 一圈 */
    while (d < 0.0f)     d += 360.0f;
    while (d >= 360.0f)  d -= 360.0f;
    return d;
}

uint8_t ARM_GetPosValid(uint8_t axis)
{
    return (axis < (uint8_t)ARM_AXIS_NUM) ? s_pos_valid[axis] : 0u;
}

float    ARM_GetSlider(uint8_t axis)       { return (axis < (uint8_t)ARM_AXIS_NUM) ? s_slider_val[axis] : 0.0f; }
uint16_t ARM_GetSliderCount(uint8_t axis)  { return (axis < (uint8_t)ARM_AXIS_NUM) ? s_slider_cnt[axis] : 0u; }
int32_t  ARM_GetTargetPulses(uint8_t axis) { return (axis < (uint8_t)ARM_AXIS_NUM) ? s_target[axis] : 0; }

/* 该轴在用的【从机地址】（走 CAN 和走串口都是这个地址；0 = 没配） */
uint8_t ARM_GetNode(uint8_t axis)
{
    if (axis == 0u) return (uint8_t)ARM_NODE_1;
    if (axis == 1u) return (uint8_t)ARM_NODE_2;
    return 0u;
}

/* 诊断：CAN 上收到的【扩展帧】累计数（驱动器应答 = 扩展帧；GM6020 是标准帧不计入）
   一直在涨 = 驱动器在说话；一直 0 = 驱动器根本没答（没上电 / 不在总线上 / 波特率 / 地址）*/
uint16_t ARM_GetCanExtCount(void) { return s_can_ext_cnt; }

/* 诊断：CAN 收到的【所有帧】总数（含标准帧，比如 GM6020 的 0x205）
   涨 = 总线上至少有一个节点在说话；恒 0 = 总线完全没动静 */
uint16_t ARM_GetCanRxTotal(void)  { return s_can_rx_total; }

/* ★虚拟位置（累计下发的脉冲）；清零后从 0 开始 */
void    ARM_SetJogRpm(uint8_t axis, float rpm)
{
    if (axis < 2u) { if (rpm < 1.0f) rpm = 1.0f; if (rpm > 300.0f) rpm = 300.0f; s_jog_rpm[axis] = rpm; }
}
float   ARM_GetJogRpm(uint8_t axis) { return (axis < 2u) ? s_jog_rpm[axis] : 0.0f; }

int32_t ARM_GetVirtPos(uint8_t axis) { return (axis < 2u) ? s_virt_pos[axis] : 0; }
void    ARM_SetVirtPos(uint8_t axis, int32_t v) { if (axis < 2u) s_virt_pos[axis] = v; }

/* 诊断：成功放进发送邮箱的帧数 / 失败数
   TX_OK 恒 0 → CAN 根本没在发（外设没启动 / 邮箱进不去） */
uint16_t ARM_GetCanTxOk(void)     { return s_can_tx_ok; }
uint16_t ARM_GetCanTxFail(void)   { return s_can_tx_fail; }

/* 诊断：最后收到的扩展帧（驱动器回过什么 ID / 什么载荷） */
uint32_t ARM_GetLastExtId(void)   { return s_last_ext_id; }
uint8_t  ARM_GetLastExtDlc(void)  { return s_last_ext_dlc; }
uint8_t  ARM_GetLastExtByte(uint8_t i)
{
    return (i < 8u) ? s_last_ext_data[i] : 0u;
}

/* 诊断：扩展帧滤波器是否配置成功（0 = 失败 → 0x100x 的扩展帧进不来） */
uint8_t  ARM_GetCanFilterOk(void) { return s_can_filter_ok; }

/* ★诊断：CAN 错误计数器（直接读 CAN1 的 ESR，不需要开嗅探）
   TEC = 发送错误计数：我们发帧没人 ACK 时会涨 → 说明"总线上的节点没接受到我们的帧/不认我们的帧"
   REC = 接收错误计数：**它涨得厉害 = 总线上有人在发，但波特率对不上**（我们按 1Mbps 解它的 500Kbps 帧）
   ★两者都 0 = 总线上除了 GM6020 没有任何别人在发东西 */
void ARM_GetCanErrors(uint16_t *tec, uint16_t *rec, uint8_t *lec)
{
    uint32_t esr = hcan1.Instance->ESR;

    if (tec != 0) *tec = (uint16_t)((esr >> 16) & 0xFFu);
    if (rec != 0) *rec = (uint16_t)((esr >> 24) & 0xFFu);
    if (lec != 0) *lec = (uint8_t)(esr & 0x07u);
}

#if ARM_CAN_BAUD_SCAN
/* 速率扫描结果：hits 位掩码（bit0 = 1M 收到过 / bit1 = 500K / bit2 = 250K）、当前档位 0/1/2 */
uint8_t ARM_GetCanScanHits(void) { return s_bscan_hits; }
uint8_t ARM_GetCanScanIdx(void)  { return s_bscan_idx; }
uint8_t ARM_GetCanListenOnly(void) { return s_listen_only; }
#endif

/* ★点动测试（调参通道 `J` 命令用）：让指定轴相对走 N 个脉冲
   · CAN 走 0xF3（相对位置）——不需要知道当前绝对位置，正好用来验证"收不收帧"
   · 串口那条也发一遍（如果电机其实接在板上 5/6 口的 T/R 上）
   返回 1 = 已下发 */
uint8_t ARM_Jog(uint8_t axis, int32_t pulses)
{
    if (pulses == 0) return 0u;
    if (pulses >  500000) pulses =  500000;      /* 兜底限幅：单次最多 ±9.7 圈 */
    if (pulses < -500000) pulses = -500000;

    /* ★虚拟位置累加：驱动器不回读位置时，用"下发过的脉冲总和"当位置 */
    if (axis < 2u) s_virt_pos[axis] += pulses;

    if (axis == 0u)
    {
#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
        arm_can_move_rel((uint8_t)ARM_NODE_1, pulses, ARM_CAN_ACC, (uint8_t)s_jog_rpm[0]);
#endif
#if ARM_IFACE_UART
        PD42S1_PosModeRel((uint8_t)ARM_NODE_1, (uint8_t)((pulses < 0) ? 1u : 0u),
                          ARM_CAN_ACC, (uint16_t)s_jog_rpm[0],
                          (uint32_t)((pulses < 0) ? -pulses : pulses));
#endif
        return 1u;
    }

    if (axis == 1u)
    {
#if ARM_IFACE_CAN_TOO || !ARM_IFACE_UART
        arm_can_move_rel((uint8_t)ARM_NODE_2, pulses, ARM_CAN_ACC, (uint8_t)s_jog_rpm[1]);
#endif
#if ARM_IFACE_UART
        PD42S1_PosModeRel((uint8_t)ARM_NODE_2, (uint8_t)((pulses < 0) ? 1u : 0u),
                          ARM_CAN_ACC, (uint16_t)s_jog_rpm[1],
                          (uint32_t)((pulses < 0) ? -pulses : pulses));
#endif
        return 1u;
    }

    return 0u;
}

#if ARM_ADDR_SCAN
uint16_t ARM_ScanMask(void) { return s_scan_mask; }
#endif

#endif /* ARM_ENABLE */
