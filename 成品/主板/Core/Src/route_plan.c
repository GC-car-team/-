/**
 * 与 pingmu/build_screen.py 的 search()/plan() 对齐。
 * 改代价或路点时，两处一起改，并用底部 PLAN_TEST 对一遍。
 */
#include "route_plan.h"

#define PLAN_INF 1000000000
/* 一段路 10000，一次 90° 转弯 100，一次掉头 1。转弯再多也贵不过多走一段。 */

static const int8_t EDGES[12][2] = {
    {0, 1}, {1, 2}, {3, 4}, {4, 5}, {6, 7}, {7, 8},
    {0, 3}, {3, 6}, {1, 4}, {4, 7}, {2, 5}, {5, 8}
};

static int edge_bit(int a, int b)
{
    int i;
    if (a > b) {
        int t = a;
        a = b;
        b = t;
    }
    for (i = 0; i < 12; i++) {
        if (EDGES[i][0] == a && EDGES[i][1] == b) return i;
    }
    return -1;
}

static int neighbor(int node, int heading)
{
    int row = node / 3;
    int col = node % 3;
    if (heading == 0) row -= 1;
    else if (heading == 1) col += 1;
    else if (heading == 2) row += 1;
    else col -= 1;
    if (row < 0 || row > 2 || col < 0 || col > 2) return -1;
    return row * 3 + col;
}

static int popcount16(uint16_t v)
{
    int n = 0;
    while (v) {
        n += (int)(v & 1u);
        v = (uint16_t)(v >> 1);
    }
    return n;
}

/* 成功时把这一段的动作字母追加进 out，并写回到达该工位时的朝向。 */
static int search_leg(uint16_t node_mask, uint16_t edge_mask,
                      int *node, int *heading, int goal, RoutePlan *out)
{
    int cost[36];
    uint8_t parent[36];
    int state;
    int pass;
    int start;
    int best;
    int best_state;
    int guard;
    uint8_t stack[40];
    int sp;
    int prev;

    if ((node_mask & (uint16_t)(1u << *node)) != 0) return 0;
    if ((node_mask & (uint16_t)(1u << goal)) != 0) return 0;

    for (state = 0; state < 36; state++) {
        cost[state] = PLAN_INF;
        parent[state] = 255u;
    }
    start = *node * 4 + *heading;
    cost[start] = 0;

    for (pass = 0; pass < 36; pass++) {
        for (state = 0; state < 36; state++) {
            int cur = cost[state];
            int node_now;
            int head_now;
            int k;
            if (cur >= PLAN_INF) continue;
            node_now = state / 4;
            head_now = state - node_now * 4;
            for (k = 0; k < 3; k++) {
                static const int add[3] = {3, 1, 2};
                static const int extra[3] = {100, 100, 1};
                int nh = head_now + add[k];
                int dest;
                int nxt;
                if (nh >= 4) nh -= 4;
                dest = node_now * 4 + nh;
                nxt = cur + extra[k];
                if (nxt < cost[dest]) {
                    cost[dest] = nxt;
                    parent[dest] = (uint8_t)state;
                }
            }
            {
                int nid = neighbor(node_now, head_now);
                int bit;
                int dest;
                int nxt;
                if (nid < 0) continue;
                bit = edge_bit(node_now, nid);
                if (bit < 0) continue;
                if ((edge_mask & (uint16_t)(1u << bit)) != 0) continue;
                if ((node_mask & (uint16_t)(1u << nid)) != 0) continue;
                dest = nid * 4 + head_now;
                nxt = cur + 10000;
                if (nxt < cost[dest]) {
                    cost[dest] = nxt;
                    parent[dest] = (uint8_t)state;
                }
            }
        }
    }

    best = PLAN_INF;
    best_state = -1;
    for (state = 0; state < 4; state++) {
        int s = goal * 4 + state;
        if (cost[s] < best) {
            best = cost[s];
            best_state = s;
        }
    }
    if (best_state < 0 || best >= PLAN_INF) return 0;

    sp = 0;
    state = best_state;
    guard = 0;
    while (state != start && guard < 40) {
        if (sp >= 40) return 0;
        stack[sp++] = (uint8_t)state;
        state = parent[state];
        if (state == 255) return 0;
        guard += 1;
    }
    if (state != start) return 0;

    prev = start;
    while (sp > 0) {
        int cur = stack[--sp];
        int pn = prev / 4;
        int ph = prev - pn * 4;
        int cn = cur / 4;
        int ch = cur - cn * 4;
        char letter;
        if (out->length + 1u >= ROUTE_PLAN_MAX) return 0;
        if (pn == cn) {
            int dh = ch - ph;
            if (dh < 0) dh += 4;
            letter = (dh == 1) ? 'R' : (dh == 2) ? 'U' : 'L';
        } else {
            letter = 'F';
            out->nodes = (uint16_t)(out->nodes | (uint16_t)(1u << cn));
            {
                int bit = edge_bit(pn, cn);
                if (bit >= 0) {
                    out->edges = (uint16_t)(out->edges | (uint16_t)(1u << bit));
                }
            }
        }
        out->letters[out->length++] = (uint8_t)letter;
        prev = cur;
    }
    *heading = best_state % 4;
    *node = goal;
    return 1;
}

static int append_letter(RoutePlan *out, char letter)
{
    if (out->length + 1u >= ROUTE_PLAN_MAX) return 0;
    out->letters[out->length++] = (uint8_t)letter;
    return 1;
}

void RoutePlan_Compute(uint8_t start_up, uint16_t node_mask, uint16_t edge_mask,
                       RoutePlan *out)
{
    /* 比赛顺序：右中扫码、上中取料、下中放下并抓回、左中暂存，两批后回出发路口。
       这些工位字母在车上都是停车 2 秒。走路仍按最短、再少转弯、再少掉头。 */
    static const int goals[8] = {5, 1, 7, 3, 1, 7, 3, 0};
    /* 下标就是路段号。2 和 5 在下面单独写成放下+抓回，这里的字母不会用到。 */
    static const char mark_of[7] = {'C', 'M', 'P', 'B', 'M', 'P', 'B'};
    int node;
    int heading;
    int leg;
    int home;

    out->length = 0;
    out->nodes = 0;
    out->edges = 0;
    out->status = ROUTE_PLAN_NONE;
    /* 上中、左中、右中、下中是停车点，发来的路口位直接丢掉。 */
    node_mask = (uint16_t)(node_mask & (uint16_t)~170u);

    if (popcount16(node_mask) + popcount16(edge_mask) > 2) {
        out->status = ROUTE_PLAN_TOO_MANY;
        return;
    }

    if (start_up) {
        node = 2;
        heading = 2;
        home = 2;
    } else {
        node = 8;
        heading = 0;
        home = 8;
    }
    out->nodes = (uint16_t)(1u << node);

    if ((node_mask & (uint16_t)(1u << node)) != 0) {
        out->nodes = 0;
        out->status = ROUTE_PLAN_START_BLOCK;
        return;
    }

    for (leg = 0; leg < 8; leg++) {
        int goal = (leg == 7) ? home : goals[leg];
        if (!search_leg(node_mask, edge_mask, &node, &heading, goal, out)) {
            out->length = 0;
            out->nodes = 0;
            out->edges = 0;
            out->status = ROUTE_PLAN_NONE;
            return;
        }
        if (leg < 7) {
            if (leg == 2 || leg == 5) {
                if (!append_letter(out, 'P') || !append_letter(out, 'G')) {
                    out->length = 0;
                    out->nodes = 0;
                    out->edges = 0;
                    out->status = ROUTE_PLAN_NONE;
                    return;
                }
            } else if (!append_letter(out, mark_of[leg])) {
                out->length = 0;
                out->nodes = 0;
                out->edges = 0;
                out->status = ROUTE_PLAN_NONE;
                return;
            }
        }
    }
    out->letters[out->length] = 0;
    out->status = ROUTE_PLAN_OK;
}

#ifdef PLAN_TEST
#include <stdio.h>
int main(void)
{
    unsigned start, n, e;
    for (start = 0; start < 2; start++) {
        for (n = 0; n < 512; n++) {
            for (e = 0; e < 4096; e++) {
                int bits = 0;
                unsigned x;
                RoutePlan p;
                unsigned i;
                x = n;
                while (x) { bits += (int)(x & 1u); x >>= 1; }
                x = e;
                while (x) { bits += (int)(x & 1u); x >>= 1; }
                if (bits > 2) continue;
                RoutePlan_Compute((uint8_t)start, (uint16_t)n, (uint16_t)e, &p);
                printf("%u %u %u %u ", start, n, e, p.status);
                for (i = 0; i < p.length; i++) putchar((int)p.letters[i]);
                printf(" %u %u\n", p.nodes, p.edges);
            }
        }
    }
    return 0;
}
#endif
