# vision_place · MaixCAM Pro

日常说明见仓库 [`docs/NOW.md`](../docs/NOW.md)。

## 关键文件

| 文件 | 作用 |
|---|---|
| `contract.py` | 契约（FROZEN） |
| `config.py` | 全部参数；模型在 `models/` |
| `main.py` | 主循环 |
| `perceive.py` / `detectors.py` / `model_meta.py` | 检测与映射 |
| `verify_map.py` | 桌上核对业务ID |
| `eval_task.py` | gate 清单 / 环号回归 |
| `calib_task.py` | 标定 |

## 自测

```bat
python contract.py
python model_meta.py
python calib_task.py --selftest
python eval_task.py --list
```
