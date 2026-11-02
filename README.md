# 搬运机器人 · 视觉端

## 先看这一份

**[`docs/NOW.md`](docs/NOW.md)** —— 当前条件、能做什么、进度、目录说明。

其它文档默认先别翻。

---

## 代码

| 目录 | 平台 | 作用 |
|---|---|---|
| [`vision_place/`](vision_place/) | MaixCAM Pro | 下视：颜色 + 环数字 + 增量误差 |
| [`vision_qr/`](vision_qr/) | K210 | 扫码（协议对齐 `saoma.py`） |
| [`saoma.py`](saoma.py) | K210 | 你的原始扫码，对照用 |

## 需要时再看

| 文件 | 内容 |
|---|---|
| [`docs/contract-mcu.md`](docs/contract-mcu.md) | 给主控的一页契约 |
| [`docs/arch-v1.md`](docs/arch-v1.md) | 架构设计理由 |
| [`docs/rules-extract.md`](docs/rules-extract.md) | 赛题要点 |
| [`docs/archive/`](docs/archive/) | 旧长文（默认别翻） |

## PC 自测（无打印也能跑）

```bat
cd vision_place
python contract.py
python model_meta.py
python calib_task.py --selftest
cd ..\vision_qr
python qr_validate.py
```
