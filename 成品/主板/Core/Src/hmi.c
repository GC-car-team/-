/**
  ******************************************************************************
  * @file    hmi.c
  * @brief   触摸屏菜单：按钮字节 → 请求模式
  *
 *  单字节（调试按钮，以及出发/取消）：
 *      0x02 / '2' → 平移测试     0x03 / '3' → 转弯测试
 *      0x04 / '4' → 里程测试     0x01 / '1' → 例程1（屏幕上该键已改为确认）
 *      0x10       → 出发：停掉测试，按已确认的路线发车
 *      0x00 / '0' / 's' / 'S' → 取消：停车，并丢掉已确认的路线
 *      0xA0 / 0xA1 → 右下 / 右上出发点，主板重算并涂屏幕
 *      0xC0+n / 0xD0+n → 路口 n 标上 / 取消（n = 0..8）
 *      0xE0+n / 0xF0+n → 路段 n 标上 / 取消（n = 0..11）
 *      0xAF → 清掉全部障碍并重算
 *  路线帧（确认键）：AA 5A，然后动作字母，最后 FE
 *      F 前进一段  L 左转  R 右转  U 掉头
 *      C 扫码  M 原料  P 放下  G 抓回  B 暂存
 *  帧内字节不再当成上面的单字节命令。帧外其它字节忽略。
  *
  *  往屏幕发状态：TJC 的串口指令必须以 3 个 0xFF 结尾，所以用
  *      t0.txt="MODE 1" <FF FF FF>
  ******************************************************************************
  */
#include "hmi.h"
#include "route_plan.h"

#if HMI_ENABLE

static UART_HandleTypeDef *s_uart;
static uint8_t s_rx;                 /* 接收单字节缓冲 */
static volatile uint8_t s_req;       /* 中断里写、主循环读 → 用 volatile */
static volatile uint8_t s_start;     /* 0x10 出发 */
static volatile uint8_t s_stop;      /* 0x00 取消 */
static volatile uint8_t s_frame;     /* 0 空闲，1 已见 AA，2 正在收动作 */
static uint8_t s_path[HMI_PATH_MAX];
static volatile uint8_t s_path_n;
static uint8_t s_path_keep;          /* 空帧 AA 5A FE 不把已有路线清掉 */
static volatile uint8_t s_plan_dirty;
static volatile uint8_t s_start_up;
static volatile uint16_t s_obs_n;
static volatile uint16_t s_obs_e;

static void hmi_clear_path(void)
{
    s_path_n = 0u;
    s_frame = 0u;
}

/* 屏幕上报的出发点 / 障碍。只改掩码，真正算路在主循环。 */
static void hmi_plan_byte(uint8_t b)
{
    if (b == 0xA0u) { s_start_up = 0u; s_plan_dirty = 1u; return; }
    if (b == 0xA1u) { s_start_up = 1u; s_plan_dirty = 1u; return; }
    if (b == 0xAFu)
    {
        s_obs_n = 0u;
        s_obs_e = 0u;
        s_plan_dirty = 1u;
        return;
    }
    if (b >= 0xC0u && b <= 0xC8u)
    {
        s_obs_n = (uint16_t)(s_obs_n | (uint16_t)(1u << (b - 0xC0u)));
        s_plan_dirty = 1u;
        return;
    }
    if (b >= 0xD0u && b <= 0xD8u)
    {
        s_obs_n = (uint16_t)(s_obs_n & (uint16_t)~(1u << (b - 0xD0u)));
        s_plan_dirty = 1u;
        return;
    }
    if (b >= 0xE0u && b <= 0xEBu)
    {
        s_obs_e = (uint16_t)(s_obs_e | (uint16_t)(1u << (b - 0xE0u)));
        s_plan_dirty = 1u;
        return;
    }
    if (b >= 0xF0u && b <= 0xFBu)
    {
        s_obs_e = (uint16_t)(s_obs_e & (uint16_t)~(1u << (b - 0xF0u)));
        s_plan_dirty = 1u;
    }
}

/* 不用 HAL_UART_Transmit。它会占着串口锁，接收中断里再挂接收会失败，之后启动键的字节就丢了。 */
static void hmi_send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (s_uart == 0 || s_uart->Instance == 0) return;
    for (i = 0u; i < len; i++)
    {
        uint32_t guard = 100000u;
        while (((s_uart->Instance->SR & USART_SR_TXE) == 0u) && (guard != 0u)) guard--;
        if (guard == 0u) return;
        s_uart->Instance->DR = buf[i];
    }
}

static void hmi_rx_arm(void)
{
    if (s_uart == 0) return;
    if (s_uart->RxState == HAL_UART_STATE_BUSY_RX) return;
    __HAL_UART_CLEAR_OREFLAG(s_uart);
    s_uart->ErrorCode = HAL_UART_ERROR_NONE;
    s_uart->RxState = HAL_UART_STATE_READY;
    (void)HAL_UART_Receive_IT(s_uart, &s_rx, 1u);
}

void HMI_Init(UART_HandleTypeDef *huart)
{
    s_uart = huart;
    s_req  = 0u;
    s_start = 0u;
    s_stop = 0u;
    s_plan_dirty = 0u;
    s_start_up = 0u;
    s_obs_n = 0u;
    s_obs_e = 0u;
    hmi_clear_path();
    if (s_uart != 0)
    {
        (void)HAL_UART_Receive_IT(s_uart, &s_rx, 1u);
    }
}

/* 单个字节 → 请求（中断里调，只做赋值） */
static void hmi_parse(uint8_t b)
{
    if (s_frame == 1u)
    {
        s_frame = (b == 0x5Au) ? 2u : 0u;
        if (s_frame == 2u)
        {
            s_path_keep = s_path_n;
            s_path_n = 0u;
        }
        return;
    }
    if (s_frame == 2u)
    {
        if (b == 0xFEu)
        {
            s_frame = 0u;
            /* plan.txt 是空的时，屏幕会发出没有字母的帧。那一帧不覆盖主板已算好的路线。 */
            if (s_path_n == 0u) s_path_n = s_path_keep;
        }
        else if (s_path_n < HMI_PATH_MAX)
        {
            s_path[s_path_n++] = b;
        }
        return;
    }
    if (b == 0xAAu)
    {
        s_frame = 1u;
        return;
    }
    if (b == 0xA0u || b == 0xA1u || b == 0xAFu ||
        (b >= 0xC0u && b <= 0xC8u) || (b >= 0xD0u && b <= 0xD8u) ||
        (b >= 0xE0u && b <= 0xEBu) || (b >= 0xF0u && b <= 0xFBu))
    {
        hmi_plan_byte(b);
        return;
    }
    switch (b)
    {
    case 0x00: case (uint8_t)'0': case (uint8_t)'s': case (uint8_t)'S':
        s_stop = 1u;
        s_start = 0u;
        hmi_clear_path();
        break;
    case 0x10:
        s_start = 1u;
        s_stop = 0u;
        break;
    case 0x01: case (uint8_t)'1': s_req = RUNMODE_M1;     s_stop = 0u; s_start = 0u; break;
    case 0x02: case (uint8_t)'2': s_req = RUNMODE_STRAFE; s_stop = 0u; s_start = 0u; break;
    case 0x03: case (uint8_t)'3': s_req = RUNMODE_TURN;   s_stop = 0u; s_start = 0u; break;
    case 0x04: case (uint8_t)'4': s_req = RUNMODE_DIST;   s_stop = 0u; s_start = 0u; break;
    default: break;             /* 忽略：屏幕的 0xFF 填充、噪声等 */
    }
}

void HMI_RxIsr(UART_HandleTypeDef *huart)
{
    hmi_parse(s_rx);
    (void)HAL_UART_Receive_IT(huart, &s_rx, 1u);      /* 重新挂上接收 */
}

/* 给蓝牙（bt.c）用的入口：字节 → 命令表（不碰 UART，也不用回挂接收） */
void HMI_FeedByte(uint8_t b)
{
    hmi_parse(b);
}

uint8_t HMI_TakeRequest(void)
{
    uint8_t r = s_req;
    s_req = 0u;
    return r;
}

uint8_t HMI_TakeStart(void)
{
    uint8_t r = s_start;
    s_start = 0u;
    return r;
}

uint8_t HMI_LoadPlannedPath(void)
{
    uint8_t up;
    uint16_t nodes;
    uint16_t edges;
    RoutePlan plan;
    uint8_t i;
    uint8_t n;

    __disable_irq();
    up = s_start_up;
    nodes = s_obs_n;
    edges = s_obs_e;
    __enable_irq();

    RoutePlan_Compute(up, nodes, edges, &plan);
    __disable_irq();
    if (s_frame != 0u)
    {
        __enable_irq();
        return 0u;
    }
    n = 0u;
    if (plan.status == ROUTE_PLAN_OK)
    {
        n = plan.length;
        if (n > HMI_PATH_MAX) n = (uint8_t)HMI_PATH_MAX;
        for (i = 0u; i < n; i++) s_path[i] = plan.letters[i];
    }
    s_path_n = n;
    __enable_irq();
    return n;
}

uint8_t HMI_TakeStop(void)
{
    uint8_t r = s_stop;
    s_stop = 0u;
    return r;
}

uint8_t HMI_PathLength(void)
{
    return s_path_n;
}

uint8_t HMI_PathAt(uint8_t index)
{
    if (index >= s_path_n) return 0u;
    return s_path[index];
}

uint8_t HMI_CopyPath(uint8_t *dst, uint8_t max)
{
    uint8_t n;
    uint8_t i;

    if (dst == 0) return 0u;
    __disable_irq();
    if (s_frame != 0u)
    {
        __enable_irq();
        return 0u;
    }
    n = s_path_n;
    if (n > max) n = max;
    for (i = 0u; i < n; i++) dst[i] = s_path[i];
    __enable_irq();
    return n;
}

void HMI_ReportMode(uint8_t mode)
{
    static const char *name[] = { "STOP", "M1", "STRAFE", "TURN", "DIST" };
    char buf[32];
    uint8_t n = 0u;
    const char *p;

    if (mode > RUNMODE_DIST) mode = RUNMODE_NONE;

    /* 拼 "t0.txt=\"MODE x\"" */
    for (p = "t0.txt=\""; *p != 0; p++) buf[n++] = *p;
    for (p = name[mode]; *p != 0; p++)   buf[n++] = *p;
    buf[n++] = '"';
    buf[n++] = 0xFF; buf[n++] = 0xFF; buf[n++] = 0xFF;   /* TJC 指令结束符 */

    hmi_send((const uint8_t *)buf, n);
}

void HMI_ReportLine(const char *text)
{
    HMI_ShowBox("t0.txt", text);
}

void HMI_ShowBox(const char *box, const char *text)
{
    char buf[48];
    uint8_t n = 0u;
    const char *p;

    if ((box == 0) || (text == 0)) return;
    for (p = box; (*p != 0) && (n < 16u); p++) buf[n++] = *p;
    if (n >= 44u) return;
    buf[n++] = '=';
    buf[n++] = '"';
    for (p = text; (*p != 0) && (n < 40u); p++) buf[n++] = *p;
    buf[n++] = '"';
    buf[n++] = 0xFF; buf[n++] = 0xFF; buf[n++] = 0xFF;
    hmi_send((const uint8_t *)buf, n);
}

void HMI_ShowPage(uint8_t page)
{
    char buf[12];
    uint8_t n = 0u;
    const char *p;

    for (p = "page "; *p != 0; p++) buf[n++] = *p;
    if (page > 9u) page = 9u;
    buf[n++] = (char)('0' + page);
    buf[n++] = 0xFF; buf[n++] = 0xFF; buf[n++] = 0xFF;
    hmi_send((const uint8_t *)buf, n);
}

static uint8_t s_paint[640];
static uint16_t s_paint_n;

static void paint_add(const uint8_t *p, uint16_t n)
{
    while (n > 0u && s_paint_n + 1u < (uint16_t)sizeof(s_paint))
    {
        s_paint[s_paint_n++] = *p++;
        n--;
    }
}

static void paint_text(const char *s)
{
    while (*s != 0 && s_paint_n + 1u < (uint16_t)sizeof(s_paint))
    {
        s_paint[s_paint_n++] = (uint8_t)*s++;
    }
}

static void paint_u16(uint16_t v)
{
    char tmp[5];
    uint8_t i = 0u;
    if (v == 0u)
    {
        paint_text("0");
        return;
    }
    while (v > 0u && i < 5u)
    {
        tmp[i++] = (char)('0' + (v % 10u));
        v = (uint16_t)(v / 10u);
    }
    while (i > 0u)
    {
        char c = tmp[--i];
        paint_add((const uint8_t *)&c, 1u);
    }
}

static void paint_end(void)
{
    static const uint8_t ff[3] = {0xFFu, 0xFFu, 0xFFu};
    paint_add(ff, 3u);
}

static void paint_map(uint16_t obs_n, uint16_t obs_e, const RoutePlan *plan)
{
    /* 与屏幕按钮同序。前 9 个是路口，后 12 个是路段。名字是 GBK。 */
    static const uint8_t name[21][4] = {
        {0xD7,0xF3,0xC9,0xCF}, {0xC9,0xCF,0xD6,0xD0}, {0xD3,0xD2,0xC9,0xCF},
        {0xD7,0xF3,0xD6,0xD0}, {0xD6,0xD0,0xD0,0xC4}, {0xD3,0xD2,0xD6,0xD0},
        {0xD7,0xF3,0xCF,0xC2}, {0xCF,0xC2,0xD6,0xD0}, {0xD3,0xD2,0xCF,0xC2},
        {0xBA,0xE1,0x30,0}, {0xBA,0xE1,0x31,0}, {0xBA,0xE1,0x32,0},
        {0xBA,0xE1,0x33,0}, {0xBA,0xE1,0x34,0}, {0xBA,0xE1,0x35,0},
        {0xCA,0xFA,0x30,0}, {0xCA,0xFA,0x31,0}, {0xCA,0xFA,0x32,0},
        {0xCA,0xFA,0x33,0}, {0xCA,0xFA,0x34,0}, {0xCA,0xFA,0x35,0}
    };
    static const uint8_t nlen[21] = {
        4,4,4,4,4,4,4,4,4, 3,3,3,3,3,3, 3,3,3,3,3,3
    };
    /* 路段两端路口。标了路口时，相连的路一起涂红，但不另算障碍。 */
    static const uint8_t edge_a[12] = {0, 1, 3, 4, 6, 7, 0, 3, 1, 4, 2, 5};
    static const uint8_t edge_b[12] = {1, 2, 4, 5, 7, 8, 3, 6, 4, 7, 5, 8};
    static const uint8_t none_road[] = {0xC3,0xBB,0xD3,0xD0,0xCA,0xA3,0xD3,0xE0,0xB5,0xC4,0xC2,0xB7};
    static const uint8_t start_block[] = {0xB3,0xF6,0xB7,0xA2,0xC2,0xB7,0xBF,0xDA,0xD2,0xD1,0xB7,0xE2};
    uint8_t i;

    s_paint_n = 0u;
    for (i = 0u; i < 21u; i++)
    {
        uint16_t path = (i < 9u) ? plan->nodes : plan->edges;
        uint8_t bit = (i < 9u) ? i : (uint8_t)(i - 9u);
        uint16_t color = (i < 9u) ? 50712u : 33808u;
        uint8_t blocked = 0u;
        if (i < 9u) {
            blocked = (obs_n & (uint16_t)(1u << bit)) != 0u;
        } else if ((obs_e & (uint16_t)(1u << bit)) != 0u
                   || (obs_n & (uint16_t)(1u << edge_a[bit])) != 0u
                   || (obs_n & (uint16_t)(1u << edge_b[bit])) != 0u) {
            blocked = 1u;
        }
        if (blocked) color = 63488u;
        else if ((path & (uint16_t)(1u << bit)) != 0u) color = 2016u;
        paint_add(name[i], nlen[i]);
        paint_text(".bco=");
        paint_u16(color);
        paint_end();
    }
    paint_text("plan.txt=\"");
    if (plan->status == ROUTE_PLAN_OK)
    {
        paint_add(plan->letters, plan->length);
    }
    paint_text("\"");
    paint_end();
    paint_text("seq.txt=\"");
    if (plan->status == ROUTE_PLAN_OK)
    {
        paint_add(plan->letters, plan->length);
    }
    else if (plan->status == ROUTE_PLAN_START_BLOCK)
    {
        paint_add(start_block, (uint16_t)sizeof(start_block));
    }
    else
    {
        paint_add(none_road, (uint16_t)sizeof(none_road));
    }
    paint_text("\"");
    paint_end();
    if (s_paint_n > 0u) hmi_send(s_paint, s_paint_n);
}

void HMI_Poll(void)
{
    hmi_rx_arm();
}

void HMI_PlanTask(void)
{
    uint8_t up;
    uint16_t nodes;
    uint16_t edges;
    RoutePlan plan;

    if (s_plan_dirty == 0u) return;
    __disable_irq();
    s_plan_dirty = 0u;
    up = s_start_up;
    nodes = s_obs_n;
    edges = s_obs_e;
    __enable_irq();
    nodes = (uint16_t)(nodes & (uint16_t)~170u);

    RoutePlan_Compute(up, nodes, edges, &plan);
    __disable_irq();
    if (s_frame == 0u)
    {
        uint8_t i;
        uint8_t n = plan.length;
        if (plan.status != ROUTE_PLAN_OK) n = 0u;
        if (n > HMI_PATH_MAX) n = (uint8_t)HMI_PATH_MAX;
        for (i = 0u; i < n; i++) s_path[i] = plan.letters[i];
        s_path_n = n;
    }
    __enable_irq();
    paint_map(nodes, edges, &plan);
}

#else   /* ---------------- 没开屏幕菜单：空实现（main.c 不用改） ---------------- */

void    HMI_Init(UART_HandleTypeDef *huart) { (void)huart; }
void    HMI_RxIsr(UART_HandleTypeDef *huart) { (void)huart; }
void    HMI_FeedByte(uint8_t b)              { (void)b; }
uint8_t HMI_TakeRequest(void)               { return 0u; }
uint8_t HMI_TakeStart(void)                 { return 0u; }
uint8_t HMI_LoadPlannedPath(void)           { return 0u; }
uint8_t HMI_TakeStop(void)                  { return 0u; }
uint8_t HMI_PathLength(void)                { return 0u; }
uint8_t HMI_PathAt(uint8_t index)           { (void)index; return 0u; }
uint8_t HMI_CopyPath(uint8_t *dst, uint8_t max) { (void)dst; (void)max; return 0u; }
void    HMI_ReportMode(uint8_t mode)        { (void)mode; }
void    HMI_ReportLine(const char *text)    { (void)text; }
void    HMI_ShowBox(const char *box, const char *text) { (void)box; (void)text; }
void    HMI_ShowPage(uint8_t page)          { (void)page; }
void    HMI_Poll(void)                      {}
void    HMI_PlanTask(void)                  {}

#endif /* HMI_ENABLE */

/* ★调试用：往指定组件写一行 ASCII 文本（TJC: obj.txt="..." FF FF FF）
   默认 HMI_DEBUG_LINE=0 不调用；需要屏幕调试时打开它，并换成没被占用的组件名。 */
void HMI_SetTextObj(const char *obj, const char *s)
{
    char buf[128];
    int  n = 0;

    if ((obj == 0) || (s == 0)) return;
    n = snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, s);
    if (n <= 0) return;
    hmi_send((const uint8_t *)buf, (uint16_t)n);
    hmi_send((const uint8_t *)"\xFF\xFF\xFF", 3u);
}

void HMI_SetText(const char *s)
{
    HMI_SetTextObj("t0", s);
}
/* ★给上位机/调试用：读出当前出发点与障碍掩码 */
void HMI_GetObs(uint8_t *start_up, uint16_t *obs_n, uint16_t *obs_e)
{
    if (start_up != 0) *start_up = s_start_up;
    if (obs_n != 0)    *obs_n    = s_obs_n;
    if (obs_e != 0)    *obs_e    = s_obs_e;
}