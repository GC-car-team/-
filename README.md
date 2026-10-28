# 搬运机器人比赛 · 视觉端

## 文件导航

| 文件 | 内容 | 什么时候看 |
|---|---|---|
| [`docs/desktop-mode.md`](docs/desktop-mode.md) | **桌面模式清单**（无场地/无物料/仅手机照时怎么推进） | **先看这个** |
| [`docs/gaps.md`](docs/gaps.md) | 现在测不了的缺口登记 | 避免假进度 |
| [`docs/next-steps.md`](docs/next-steps.md) | 模型接入的三个坑、精修修正、处理顺序 | 模型相关细节 |
| [`docs/rules-extract.md`](docs/rules-extract.md) | **赛题解析**：视觉 5 项任务、计分环换算、规则缺口（R1-R10） | 想知道规则怎么落到代码 |
| [`docs/arch-v1.md`](docs/arch-v1.md) | **架构 v1**：拓扑、契约、6+3 骨架、两步标定、无车应对、风险登记 | 想知道为什么这么设计 |
| [`vision_place/`](vision_place/) | **MaixCAM Pro 视觉端**：骨架 + 标定 + 模型接入 | 开始写代码时 |
| [`vision_qr/`](vision_qr/) | **K210 扫码节点**（已对齐你 `saoma.py` 的串口协议） | 搞扫码时 |
| [`saoma.py`](saoma.py) | 你的原始扫码程序，**保留作参照，未改动** | 对比新旧逻辑 |
| [`docs/qa-log.md`](docs/qa-log.md) | 方法论：三层松紧、验证阶梯 L0-L5、黄金数据、延迟测量 | 想搞清楚底层逻辑 |
| [`docs/template-v1.md`](docs/template-v1.md) | 通用模板（**已被 arch-v1 取代，留作参考**） | 想扩展到其他项目 |

## 关键结论（免得每次重推）

0. **两个模型的 class_id 都不是业务ID。** 颜色 `[1,4,2,5,6,3]`、圆环 `[3,2,1]`。
   直接用 class_id 会静默出错。一律走 `model_meta.id_of()`。
1. **±4mm（2环）的瓶颈是机械，不是视觉**。但注意：**YOLO bbox 中心只够 2 环**，
   冲 1 环必须做 bbox 内亚像素精修（`RING_REFINE`）。
2. **必须闭环视觉伺服**。底盘误差数 cm，目标 ±4mm，开环差一个数量级。
3. **契约输出增量误差 `ex/ey`，不输出绝对坐标**。因为没车，完整手眼标定做不了。
4. **视觉端不知道任务内容**。颜色→位置映射在主控。
5. **循迹建议用灰度传感器**，不用相机。
6. **K210 扫码距离**：QVGA 约 0.3~0.4m，VGA 约 0.6m（**都是估算，必须实测**）。

## 当前进度

- [x] 赛题解析 / 架构 v1 / 契约草案
- [x] L0 骨架通过（假数据源输出契约消息）
- [x] `calib_task.py` 标定工具（数学自测通过）
- [x] K210 扫码校验逻辑（17 用例通过）
- [x] **`.mud` 类别映射解析（18 项自测通过）**
- [x] **模型接入薄壳 `detectors.py`**
- [x] **`vision_qr` 对齐你 `saoma.py` 的串口协议**
- [ ] **填 `MODEL_CONF_TH` / `MODEL_IOU_TH`，板上加载模型**
- [ ] **肉眼验证 class_id 映射（红/蓝/浅蓝三个物料）**
- [ ] **实测扫码距离（QVGA vs VGA）**
- [ ] `data/gate/` 首批 + `eval_task.py`
- [ ] `RING_REFINE` 亚像素精修（冲 1 环才需要）

## 现在就做的事（桌面模式 · 无场地无物料）

详见 [`docs/desktop-mode.md`](docs/desktop-mode.md)。摘要：

1. **1:1 打印**：环台图（图二）+ 80mm 任务二维码 + 100mm 刻度条  
2. **板上跑** `python verify_map.py ring`，确认数字 1/2/3 → 业务ID 1/2/3  
3. **K210 实测扫码距离**（打印二维码即可）  
4. **手机圆环照**只归档到 `vision_place/data/gate/ring_phone/`，只验环号不验毫米  
5. 颜色 / ±4mm / 精修：**等真实物料与环台**，桌面不要假装完成  
