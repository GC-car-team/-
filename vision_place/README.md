# vision_place · MaixCAM Pro 视觉端

夹爪下视相机。输出**增量误差**（夹爪需要移动多少 mm）给主控，不输出绝对坐标。
架构与理由见 `../docs/arch-v1.md`。

## 当前状态：阶段 2 骨架（L0 已通过，PC 上验证）

| 文件 | 层 | 能否自由改 |
|---|---|---|
| `contract.py` | 1 接口层 | **不能**。改字段/单位/符号必须升 `CV` 并通知主控 |
| `config.py` | 1/2 | 只能填值，不能加业务逻辑 |
| `perceive.py` | 3 算法层 | **随便改**，但必须过 `eval_task.py` |
| `main.py` | 2 编排层 | 改要说明理由（影响回放与串口行为） |
| `calib_task.py` | 工具 | 标定。`scale` 不需要车，`dir` 需要 |
| `calib.json` | 数据 | 标定产物，**由 calib_task 写，禁止手改** |

## L0 验证（已完成）

```
python contract.py        # 契约自测
python calib_task.py --selftest   # 标定数学自测（含 30° 旋转往返验证）
python main.py            # 无 MaixPy 时自动降级为 stdout
```

预期输出：

```
{"cv":"1.0","seq":1,"t":673215122,"dt":33,"md":"IDLE","st":2,"n":0,"tg":[]}
```

`st:2` = LOST、`n:0` = 没目标。**这是骨架该有的样子，不是 bug。**
它证明了：帧循环在跑、契约编码正确、串口链路通了（或已降级）。

## 标定：为什么用 2×2 矩阵而不是「角度 + 符号」

```
[ex]   [m00 m01]   [px - cx]
[ey] = [m10 m11] @ [py - cy]
```

**不是** `theta + 两个 sign`。原因：

- 角度和符号**无法独立标定**：符号错会让角度看起来差 180°，你分不清是哪个错
- 无法表达 X 和 Y 像素尺度不同
- 需要你判断「图像 y 向下、机器人 y 向左」这种约定，**任何写死的约定都是猜测**

矩阵把旋转、各向异性、符号一次包含，**两次移动就能标**，不需要任何角度计算。

### 标定步骤

```bash
# 第 1 步：尺度（**不需要车**）
#   在目标平面上放两个标记，间距 100mm（用尺子量，别信打印）
python calib_task.py scale --len-mm 100

# 第 2 步：方向（需要机器人能动一次）
python calib_task.py dir --move-mm 10
#   流程：点特征 → 沿 +X 移 10mm → 再点同一特征 → 沿 +Y 移 10mm → 再点

# 第 3 步：记录物理安装量（纯写文件，不接相机也能跑）
python calib_task.py phys --height-mm 320 --pitch-deg 90

# 检查相机装正了没有
python calib_task.py show
```

> **只做 scale 不做 dir，伺服可能不收敛。** scale 只给出对角矩阵，
> 它假设了轴对齐且无旋转，真实装配几乎不是这样。

### 几何变化的自动拦截

`calib.json` 里存了 `cam_height_mm` 和 `cam_pitch_deg`。`config.py` 加载时
会比对当前 `CAM_HEIGHT_MM` / `CAM_PITCH_DEG`，**不一致就打印醒目警告**。

原因：支架一改，之前所有 `data/gate` 数据全部作废，而且**不会报错**，
只是指标莫名变差。这条校验就是为了让这件事无法被忽略。


## 板上验证清单（拿到 MaixCAM Pro 后按顺序做）

1. [ ] `config.py` 填 `UART_DEV` / `UART_TX_PIN` / `UART_RX_PIN` 及对应的 FUNC 名
       → 用串口助手看到输出，波特率 115200
2. [ ] 核对 `main.py` 里所有 `[VERIFY]` 注释行（引脚复用函数名、`uart.UART()` 参数、
       `read()` 是否阻塞）
3. [ ] 主控发 `SET:RING`，看 `md` 是否切换
4. [ ] `config.py` 改 `SRC="cam"`，确认取图不报错
5. [ ] 量 `CAM_HEIGHT_MM` 和 `CAM_PITCH_DEG`，**写死并记下日期**
6. [ ] 跑 `calib_task.py`（尚未编写）填 `MM_PER_PX_X/Y`、`THETA_DEG`、
       `SIGN_IMG_X_TO_ROBOT_X/Y`

## 两个不许违反的纪律

1. **`config` 之外不许出现业务数字。** 见到 `if cf > 50` 这种就是魔数，改配置无效且不报错。
2. **标定值缺失时报错，不许兜底。** `perceive.to_error()` 故意不设默认值——
   悄悄算出错误坐标比崩掉危险得多。

## 下一步（阶段 3-4）

1. **[完成]** `calib_task.py`：尺度 + 方向两步标定
2. **[待做]** 在板上跑 `calib_task.py scale --len-mm 100`，填第一版矩阵
3. **[待做]** `data/gate/`：需要先知道物料尺寸 φ 才能定圆环直径
4. **[待做]** `eval_task.py`：扫目录 → PASS/FAIL
5. **[待做]** `perceive.detect_ring()`：第一个真算法

> 顺序不能反：**先有 gate 集和 eval，再写算法。** 否则调参全靠上车跑一次。

## 阻塞项：需要物料尺寸 φ

`docs/rules-extract.md` 的 R7：初赛「统一形状」的具体尺寸是多少？
不知道 φ 就算不出计分环直径（φ+3 / φ+8 / φ+15 …），也就打印不出正确的靶标。

**暂时绕过办法**：尺度标定用的是你自己量的一对标记（100mm），不依赖 φ，
所以 `calib_task.py scale` 现在就能跑。但 `detect_ring` 必须等 φ 确定。
