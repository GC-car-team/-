# 现在看这里（唯一日常文档）

> 其它长文已挪到 `docs/archive/`，需要时再翻。日常只维护本页。

---

## 你现在的条件

- 无场地、无打印、无标准物料
- 有：MaixCAM Pro、K210、两个模型、手机圆环照（可选）、代码骨架

**所以先不做**：打印靶标、颜色调参、精修冲 1 环、上车精度验收。

---

## 不能打印时，可以做的事（按顺序）

| # | 做什么 | 怎么验收 | 要不要板子 |
|---|---|---|---|
| 1 | PC 自测全绿 | 见下方命令 | 否 |
| 2 | 整理目录：模型放进 `vision_place/models/` | 路径能加载 | 否 |
| 3 | 手机圆环照归档到 `data/gate/ring_phone/`，按 `ring1_01.jpg` 命名 | 文件名可读出期望环号 | 否 |
| 4 | 写/跑 `eval_task.py`（只验环号，不验 mm） | 对文件名期望 vs 映射表逻辑 | 否→板上更好 |
| 5 | 给主控的一页契约说明 | 队友能按字段写解析 | 否 |
| 6 | 板上加载模型 + `verify_map.py ring` | 屏幕 `ID=` 与数字一致 | **要板子**；可用手机屏显示环图凑合看映射 |

第 6 步若暂时只有手机照：把照片拷到板子上用文件回放，或把手机贴在镜头前（只验「能不能出框+ID」，不可靠但比完全不跑强）。

---

## PC 自测命令（现在就能跑）

在仓库根目录：

```bat
cd vision_place
python contract.py
python model_meta.py
python calib_task.py --selftest
python verify_map.py ring
cd ..\vision_qr
python qr_validate.py
```

全绿 = 软件骨架可信。然后去做「整理 + 归档照片 + 契约一页」。

---

## 目录：日常只碰这些

```
总结1/
  README.md                 ← 入口（指向本页）
  docs/NOW.md               ← 你正在看的
  docs/contract-mcu.md      ← 给主控的一页契约
  docs/arch-v1.md           ← 架构（需要时再看）
  docs/rules-extract.md     ← 赛题要点（需要时再看）
  docs/archive/             ← 旧长文，默认别翻
  vision_place/             ← MaixCAM 代码 + models/
  vision_qr/                ← K210 扫码
  saoma.py                  ← 你的原始扫码，对照用
  bisai.mdc                 ← 规则摘要
```

---

## 进度（只保留未完成项）

- [x] 骨架 L0、契约、映射解析、扫码校验、模型薄壳
- [x] 模型已复制到 `vision_place/models/`（根目录旧文件可稍后手动删）
- [x] `eval_task.py`（`--list` / `--run-ring`）
- [x] 主控契约一页 `docs/contract-mcu.md`
- [ ] 手机圆环照归档到 `data/gate/ring_phone/`（有则做）
- [ ] 板上 `verify_map`（有板就做；可暂缓）
- [ ] 打印 / 颜色精调 / 精修 / 场地 → 条件具备后再做

---

## 三条不要忘

1. `class_id` ≠ 业务ID，必须走 `model_meta.id_of`
2. 数字模型只给环号+粗位置；精修要外扩找环线（现在不做）
3. 桌面通过 ≠ 赛场通过；缺口见 `docs/archive/gaps.md`
