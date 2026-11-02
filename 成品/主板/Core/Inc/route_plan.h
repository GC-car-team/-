/**
 * 比赛路线规划。规则与 pingmu/build_screen.py 的 plan() 相同：
 * 3×3 路口。先少走路，再少 90° 转弯，最后少掉头。
 * 最多两个障碍。工位动作只产出字母，车上各停 2 秒。
 */
#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H

#include <stdint.h>

#define ROUTE_PLAN_MAX 96u

enum {
    ROUTE_PLAN_OK = 0,
    ROUTE_PLAN_NONE = 1,
    ROUTE_PLAN_START_BLOCK = 2,
    ROUTE_PLAN_TOO_MANY = 3
};

typedef struct {
    uint8_t letters[ROUTE_PLAN_MAX];
    uint8_t length;
    uint16_t nodes;
    uint16_t edges;
    uint8_t status;
} RoutePlan;

/* start_up: 0 右下朝北，1 右上朝南。node_mask / edge_mask 的位和屏幕按钮一致。 */
void RoutePlan_Compute(uint8_t start_up, uint16_t node_mask, uint16_t edge_mask,
                       RoutePlan *out);

#endif
