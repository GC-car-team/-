#ifndef ROUTE_RUN_H
#define ROUTE_RUN_H

#include <stdint.h>

/* ★比赛路线执行（障碍→算路→按字母跑）
   用法：屏幕"出发" → RouteRun_Start()；主循环每拍 RouteRun_Task()；"取消" → RouteRun_Stop() */

uint8_t RouteRun_Start(void);      /* 1 = 有路可走，已开始 */
void    RouteRun_Stop(void);
uint8_t RouteRun_IsActive(void);   /* 1 = 正在跑 */
uint8_t RouteRun_Remaining(void);  /* 还剩几步 */
void    RouteRun_Task(void);

/* ★调试/可视化：读路线内容与进度 */
void    RouteRun_Info(uint8_t *n, uint8_t *idx, uint8_t *phase, const uint8_t **letters);

#endif /* ROUTE_RUN_H */