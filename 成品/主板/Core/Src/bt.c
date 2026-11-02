/**
  ******************************************************************************
  * @file    bt.c
  * @brief   HC-05 蓝牙串口实现（UART4 = PA0/PA1）
  *
  *  移植说明（来自 D:\...\sentinel_mouse\bsp\hc05.c）：
  *    · 那版自己写了 USART1_IRQHandler 和 HAL_UART_RxCpltCallback —— 本工程这两个
  *      已经存在（stm32f4xx_it.c / pd42s1.c），直接照抄会【重复定义】；
  *      所以这里只保留"字节处理"那一层，中断入口改成 BT_RxIsr() 由回调分发。
  *    · 那版 init 里 DeInit + 改波特率 + Init；本工程用更轻的做法：
  *      直接改 huart->Init.BaudRate 再 HAL_UART_Init()（GPIO/时钟不用重配）。
  *    · 环形缓冲 / 读一行 / printf 的写法与原版一致。
  *
  *  ⚠️ 这里的发送是【阻塞】的：115200 下 44 字节约 3.8ms，放主循环里发可以，
  *     别在中断里调。
  ******************************************************************************
  */
#include "bt.h"

#if BT_ENABLE && defined(HAL_UART_MODULE_ENABLED)

#include "usart.h"
#include "hmi.h"            /* 蓝牙收到的 '0'~'4' 复用触摸屏那套命令表 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ====================================================================
  *  Part 1: 环形接收缓冲（中断里写、主循环读）
  * ==================================================================== */
static UART_HandleTypeDef *s_uart = BT_UART;
static uint8_t  s_rx_byte;
static uint8_t  s_rxbuf[BT_RX_BUF_SIZE];
static volatile uint16_t s_head, s_tail, s_count;
static volatile uint32_t s_rx_total, s_rx_drop, s_lines, s_cmds;
static uint8_t  s_ready;
static uint32_t s_link_baud;                /* BT_Init 里最终定下来的波特率（AT 升级成功=115200，否则 BT_BAUD） */

static void rx_push(uint8_t b)
{
    if (s_count < BT_RX_BUF_SIZE)
    {
        s_rxbuf[s_head] = b;
        s_head = (uint16_t)((s_head + 1u) % BT_RX_BUF_SIZE);
        s_count++;
    }
    else
    {
        s_rx_drop++;                       /* 满了就丢，别覆盖没读的数据 */
    }
}

static uint8_t rx_pop(void)
{
    uint8_t b = 0u;

    if (s_count > 0u)
    {
        b = s_rxbuf[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) % BT_RX_BUF_SIZE);
        s_count--;
    }
    return b;
}

/* ====================================================================
  *  Part 2: 初始化
  * ==================================================================== */
/* 改波特率：CubeMX 生成的是 115200，模块可能是别的（出厂 9600 / 改过 38400）
   ⚠️ 直接改 Init.BaudRate 再 HAL_UART_Init 就行（GPIO/时钟/NVIC 不用重配，
      而且 HAL 只在 gState=RESET 时才调 MspInit；已挂上的接收中断也不会掉）
   ⚠️ 只有"按宏重配"(BT_SET_BAUD) 或"自动找波特率"(AUTOBAUD) 时才需要它 → 一起 #if，
      否则 BT_SET_BAUD=0 时会吃 #177-D warning */
#if BT_SET_BAUD || (BT_ALIVE_MS > 0 && BT_AUTOBAUD)
static void bt_apply_baud(uint32_t baud)
{
    if ((s_uart == 0) || (baud == 0u)) return;
    if (s_uart->Init.BaudRate == baud)  return;

    s_uart->Init.BaudRate = baud;
    (void)HAL_UART_Init(s_uart);
}
#endif

#if BT_AT_UPGRADE
/* 把 HC-05 升到 115200（AT 模式自动识别，失败自动回落）
   用法：按住模块上的小按键不放 → 给板子断电再上电（模块要跟着一起断电）→ 松开
   原理：AT 模式下模块固定在 38400；这里按 38400 发 "AT"，只有收到 "OK" 才认为进了 AT 模式，
         再发 AT+BAUD8 把它改成 115200（永久保存）；收不到就什么都不改，按 BT_BAUD 原样跑。
   返回：1 = 已升到 115200；0 = 没进 AT 模式，保持 BT_BAUD */
static uint8_t bt_at_upgrade(void)
{
    uint8_t b;
    uint8_t i, ok = 0u;

    s_uart->Init.BaudRate = 38400u;
    (void)HAL_UART_Init(s_uart);
    HAL_Delay(30);

    for (i = 0u; (i < 5u) && (ok == 0u); i++)
    {
        (void)HAL_UART_Transmit(s_uart, (uint8_t *)"AT\r\n", 4u, 60u);

        /* 等应答：先收到 'O'、紧接着 'K' 才算进了 AT 模式 */
        if (HAL_UART_Receive(s_uart, &b, 1u, 80u) == HAL_OK)
        {
            if ((b == (uint8_t)'O') &&
                (HAL_UART_Receive(s_uart, &b, 1u, 80u) == HAL_OK) &&
                (b == (uint8_t)'K'))
            {
                ok = 1u;
            }
        }
    }

    if (ok != 0u)
    {
        /* 两种固件写法都发一遍，谁认识算谁的 */
        (void)HAL_UART_Transmit(s_uart, (uint8_t *)"AT+BAUD8\r\n", 11u, 60u);
        HAL_Delay(150);
        (void)HAL_UART_Transmit(s_uart, (uint8_t *)"AT+UART=115200,0,0\r\n", 21u, 60u);
        HAL_Delay(150);
    }

    s_link_baud = (ok != 0u) ? (uint32_t)BT_BAUD_HI : (uint32_t)BT_BAUD;
    s_uart->Init.BaudRate = s_link_baud;
    (void)HAL_UART_Init(s_uart);
    return ok;
}
#endif

void BT_Init(void)
{
    if (s_uart == 0) return;

#if IMU_ENABLE && IMU_IF_UART
    /* 撞车保护：UART4 现在归蓝牙。万一有人按《IMU接线与使用》把 IMU_UART 改回 (&huart4)，
       这一路字节会被"谁先匹配谁解析"抢走（回调里 IMU 分支在蓝牙前面）→ 干脆不挂接收，
       让 IMU 独占那个口；现象比"两边都半死不活"好排查 */
    if (s_uart->Instance == IMU_UART->Instance) return;
#endif

    s_head = s_tail = s_count = 0u;
    s_rx_total = s_rx_drop = s_lines = s_cmds = 0u;

#if BT_AT_UPGRADE
    (void)bt_at_upgrade();                 /* 先试着把模块升到 115200（失败就保持 BT_BAUD） */
#elif BT_SET_BAUD
    bt_apply_baud((uint32_t)BT_BAUD);      /* 按 config.h 的 BT_BAUD 重配串口 */
    s_link_baud = (uint32_t)BT_BAUD;
#endif

    s_ready = 1u;

#if ARM_ENABLE
    /* ★蓝牙当 VOFA 主口（VOFA_UART = BT_UART）时：这一路的接收归 ARM 的滑块解析，
       这里不能再挂一次 —— 两个"1 字节缓冲"抢同一个串口会互相读错。
       （ARM_Init 已经挂好，main 在 BT_Init 之后还会再调 ARM_RearmRx()） */
    if (s_uart->Instance == VOFA_UART->Instance) return;
#endif

    (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

/* 当前蓝牙实际在用的波特率（0 = 没初始化）。main 据此决定 VOFA 发多快 */
uint32_t BT_GetLinkBaud(void) { return s_link_baud; }

#if BT_LINK_SCAN
/* ====================================================================
  *  链路找波特率（BT_LINK_SCAN=1 时启用，查完改回 0）
  *  每 1.5 秒换一档：9600 → 115200 → 38400 → 57600 → 19200 → 4800
  *  每档每 120ms 发一行 "PING <波特率>\r\n"
  *  上位机抓 12 秒，哪一档能读出人类可读的 "PING xxxx" → 模块就是那个波特率
  *  （配 VOFA_OUTPUT_ENABLE=0 用，避免二进制帧混进来）
  * ==================================================================== */
static const uint32_t k_bt_scan_baud[] = { 9600u, 115200u, 38400u, 57600u, 19200u, 4800u };
static uint8_t  s_bt_scan_idx;
static uint32_t s_bt_scan_tick;
static uint32_t s_bt_scan_line_tick;

void BT_LinkScanTask(void)
{
    uint32_t now = HAL_GetTick();
    const uint8_t kN = (uint8_t)(sizeof(k_bt_scan_baud) / sizeof(k_bt_scan_baud[0]));
    char msg[32];
    int  n;

    if (s_bt_scan_tick == 0u)
    {
        s_bt_scan_tick = now;
        s_bt_scan_idx  = 0u;
        s_uart->Init.BaudRate = k_bt_scan_baud[0];
        (void)HAL_UART_Init(s_uart);
        s_bt_scan_line_tick = now;
    }
    else if ((uint32_t)(now - s_bt_scan_tick) >= 1500u)
    {
        s_bt_scan_tick = now;
        s_bt_scan_idx  = (uint8_t)((s_bt_scan_idx + 1u) % kN);
        s_uart->Init.BaudRate = k_bt_scan_baud[s_bt_scan_idx];
        (void)HAL_UART_Init(s_uart);
        s_bt_scan_line_tick = now;
    }

    if ((uint32_t)(now - s_bt_scan_line_tick) >= 120u)
    {
        s_bt_scan_line_tick = now;
        n = snprintf(msg, sizeof(msg), "PING %lu\r\n",
                     (unsigned long)k_bt_scan_baud[s_bt_scan_idx]);
        if (n > 0)
        {
            (void)HAL_UART_Transmit(s_uart, (uint8_t *)msg, (uint16_t)n, 25u);
        }
    }
}
#endif

/* 中断里调（pd42s1.c 的 HAL_UART_RxCpltCallback 分发过来）：收字节 + 重新挂接收 */
void BT_RxIsr(UART_HandleTypeDef *huart)
{
    uint8_t b = s_rx_byte;

    s_rx_total++;
    rx_push(b);

    if (b == (uint8_t)'\n') s_lines++;

#if BT_CMD_ENABLE
    /* 和触摸屏完全同一套命令：'0'/'s' = 停, '1'~'4' = 四个测试 */
    {
        uint8_t before = b;

        if ((before == (uint8_t)'0') || (before == (uint8_t)'1') ||
            (before == (uint8_t)'2') || (before == (uint8_t)'3') ||
            (before == (uint8_t)'4') || (before == (uint8_t)'s') ||
            (before == (uint8_t)'S') || (before <= 0x04u))
        {
            s_cmds++;
        }
        HMI_FeedByte(b);
    }
#endif

    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}

/* ====================================================================
  *  Part 3: 发送（阻塞）
  * ==================================================================== */
void BT_Send(const uint8_t *buf, uint16_t len)
{
    if ((s_uart == 0) || (buf == 0) || (len == 0u)) return;
    (void)HAL_UART_Transmit(s_uart, (uint8_t *)buf, len, 50u);
}

void BT_SendText(const char *str)
{
    if (str == 0) return;
    BT_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

void BT_Printf(const char *fmt, ...)
{
    char buf[BT_TXT_MAX];
    va_list ap;
    int len;

    if ((s_uart == 0) || (fmt == 0)) return;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if (len > (int)sizeof(buf)) len = (int)sizeof(buf);

    BT_Send((const uint8_t *)buf, (uint16_t)len);
}

/* ====================================================================
  *  Part 4: 接收侧对外接口
  * ==================================================================== */
uint8_t BT_DataAvailable(void)
{
    return (s_count > 0u) ? 1u : 0u;
}

uint8_t BT_ReadByte(void)
{
    return rx_pop();
}

/* 读一行（\r\n 或 \n 结尾）；没有完整行返回 0 */
uint16_t BT_ReadLine(char *buf, uint16_t size)
{
    uint16_t i = 0u;
    uint16_t scan_count, scan_tail;
    uint8_t  found = 0u;

    if ((buf == 0) || (size == 0u)) return 0u;

    scan_count = s_count;
    scan_tail  = s_tail;

    while ((scan_count > 0u) && (i < (uint16_t)(size - 1u)))
    {
        uint8_t c = s_rxbuf[scan_tail];

        scan_tail = (uint16_t)((scan_tail + 1u) % BT_RX_BUF_SIZE);
        scan_count--;

        if (c == (uint8_t)'\n') { found = 1u; break; }
        if (c != (uint8_t)'\r') buf[i++] = (char)c;
    }

    if (found == 0u) return 0u;

    /* 把扫过的（含 \n）消费掉 */
    s_tail  = scan_tail;
    s_count = scan_count;
    buf[i]  = '\0';
    return i;
}

void BT_FlushRx(void)
{
    s_head = s_tail = s_count = 0u;
}

uint8_t  BT_IsReady(void)     { return s_ready; }
uint32_t BT_GetRxBytes(void)  { return s_rx_total; }
uint32_t BT_GetRxDrop(void)   { return s_rx_drop; }
uint32_t BT_GetLines(void)    { return s_lines; }
uint32_t BT_GetCmds(void)     { return s_cmds; }

/* 主循环里调：
   ① BT_ALIVE_MS > 0 时，每隔这么久往蓝牙发一行【文本】心跳
      —— 这是查"板子→蓝牙"这条路最快的手段：JustFloat 是二进制，串口助手里看着像乱码，
         文本心跳能一眼区分"没数据 / 波特率不对(乱码) / 通了"。
         文本会混进波形里，所以查完把 BT_ALIVE_MS 改回 0。
   ② BT_AUTOBAUD=1 时，跳到下一档波特率（9600/19200/38400/57600/115200 轮流），
      不知道模块波特率时用：串口助手固定在 9600，等它转到某一档时出现能读的文字，
      那行里的 baud= 就是模块的波特率。
   ③ BT_ECHO=1 时把收到的整行原样回显 */
void BT_Task(void)
{
#if BT_ECHO
    char line[BT_LINE_MAX];

    while (BT_ReadLine(line, (uint16_t)sizeof(line)) > 0u)
    {
        BT_Printf("RX: %s\r\n", line);
    }
#endif

#if BT_ALIVE_MS > 0
    {
        static uint32_t s_alive_tick;
        uint32_t now = HAL_GetTick();

#if BT_AUTOBAUD
        static const uint32_t k_baud[5] = { 9600u, 19200u, 38400u, 57600u, 115200u };
        static uint32_t s_baud_tick;
        static uint8_t  s_baud_idx;

        if ((uint32_t)(now - s_baud_tick) >= (uint32_t)BT_AUTOBAUD_MS)
        {
            s_baud_tick = now;
            s_baud_idx  = (uint8_t)((s_baud_idx + 1u) % 5u);
            bt_apply_baud(k_baud[s_baud_idx]);
        }
#endif

        if ((uint32_t)(now - s_alive_tick) >= (uint32_t)BT_ALIVE_MS)
        {
            s_alive_tick = now;
            BT_Printf("[BT] baud=%lu ready=%u rx=%lu lines=%lu cmds=%lu\r\n",
                      (unsigned long)((s_uart != 0) ? s_uart->Init.BaudRate : 0u),
                      (unsigned)s_ready,
                      (unsigned long)s_rx_total,
                      (unsigned long)s_lines,
                      (unsigned long)s_cmds);
        }
    }
#endif
}

#else   /* ---------------- 没开蓝牙：全部空实现（main.c 不用改） ---------------- */

void    BT_Init(void)                       {}
void    BT_RxIsr(UART_HandleTypeDef *huart) { (void)huart; }
void    BT_Task(void)                       {}
void    BT_Send(const uint8_t *b, uint16_t l) { (void)b; (void)l; }
void    BT_SendText(const char *s)          { (void)s; }
void    BT_Printf(const char *fmt, ...)     { (void)fmt; }
uint8_t BT_DataAvailable(void)              { return 0u; }
uint8_t BT_ReadByte(void)                   { return 0u; }
uint16_t BT_ReadLine(char *b, uint16_t s)   { (void)b; (void)s; return 0u; }
void    BT_FlushRx(void)                    {}
uint8_t BT_IsReady(void)                    { return 0u; }
uint32_t BT_GetRxBytes(void)                { return 0u; }
uint32_t BT_GetRxDrop(void)                 { return 0u; }
uint32_t BT_GetLines(void)                  { return 0u; }
uint32_t BT_GetCmds(void)                   { return 0u; }

#endif /* BT_ENABLE */
