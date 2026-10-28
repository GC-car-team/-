"""
感知层（层 3 · 算法自由区）。

【本文件是唯一允许 agent 自由发挥的地方】
  改这里不需要通知任何人，但改完必须跑 eval_task.py。
  只有 detect_ring / detect_material / _maybe_refine_ring 允许改算法。

【不要重写的部分】
  to_error()：像素坐标 → 机器人坐标系增量误差，契约数学，只写一次。
  标定值缺一不可，缺了就报错，不做默认值兜底。

【类别编号的坑】
  见 model_meta.py：两个模型的 class_id 都不是业务ID，必须走
  model_meta.id_of()。直接用 class_id 会静默出错且不报错。
"""

import config as C


class CalibMissing(Exception):
    """标定值未填写。故意不提供默认值，避免悄悄算出错误结果。"""


def _require(name, value):
    if value is None:
        raise CalibMissing(
            "config.%s 未标定。先跑 calib_task.py，不要用默认值。" % name
        )
    return value


def to_error(cx_px, cy_px, img_w=None, img_h=None):
    """目标像素坐标 → 夹爪需要移动的增量误差 (ex, ey)，单位 mm。

    换算用 2x2 矩阵 M（来自 calib_task.py）：
        [ex]   [m00 m01]   [px - cx]
        [ey] = [m10 m11] @ [py - cy]

    为什么用矩阵而不是「角度 + 符号」：
      旋转、各向异性、坐标轴符号一次全部包含，且两次移动即可标定。
      角度/符号分开标定会互相纠缠：符号错会让角度看起来差 180°，分不清哪个错。

    返回 (ex, ey, ea)
      ex : 夹爪沿机器人 +X 需移动 mm（正 = 向前）
      ey : 夹爪沿机器人 +Y 需移动 mm（正 = 向左）
      ea : 需旋转角度，本方案无旋转机构，恒 0.0
    """
    img_w = C.CAM_WIDTH if img_w is None else img_w
    img_h = C.CAM_HEIGHT if img_h is None else img_h

    m = _require("MM_PER_PX_MATRIX", C.MM_PER_PX_MATRIX)

    dx_px = cx_px - img_w / 2.0
    dy_px = cy_px - img_h / 2.0

    ex = m[0][0] * dx_px + m[0][1] * dy_px + (C.CAM_TO_GRIPPER_MM_X or 0.0)
    ey = m[1][0] * dx_px + m[1][1] * dy_px + (C.CAM_TO_GRIPPER_MM_Y or 0.0)

    return ex, ey, 0.0


# ============================================================
# 检测器：懒加载，只在真正需要时加载模型
# ============================================================
_det = {}


def _get_detector(kind):
    if kind in _det:
        return _det[kind]
    import detectors
    if kind == "color":
        d = detectors.make_color_detector()
    elif kind == "ring":
        d = detectors.make_ring_detector()
    else:
        raise ValueError(kind)
    _det[kind] = d
    return d


def detect_ring(img):
    """找圆环 1/2/3 的中心。

    返回 list[dict]：{"c": 环序号, "cx": px, "cy": px, "w","h": px, "cf": 0-100}

    【重要：精度分级】
      YOLO 的 bbox 中心只能给到「模型输入像素」级的精度。
      若模型输入 320 而原图 640、视野 346mm，则 1 个输入像素 ≈ 2.2mm。
      这个量级**最多够 2 环（±4mm），够不到 1 环（±1.5mm）**。

      模型只识别数字，bbox 里主要是数字笔画。精修绝不能对数字框做质心。
      正确做法：以外扩 ROI 找同心环（尤其外虚线环）再拟合圆心。
        "ring_centroid" : 对环线像素加权质心
        "ring_fit"      : 对环边缘点拟合圆（更推荐）
      **先不做精修也能跑通全流程**，只是成绩停在 2 环。

    实现要点：
      - 环不完整（部分出画/被物料遮挡）时仍应给中心，置信度降低
      - 不要把不同环当同心环：它们是三处独立位置
    """
    d = _get_detector("ring")
    dets = d.detect(img)
    out = []
    for o in dets:
        cf = int(round(o["score"] * 100))
        if C.RING_MIN_CONF is not None and cf < C.RING_MIN_CONF:
            continue
        cx, cy = _maybe_refine_ring(img, o)
        out.append({"c": o["cls_id"], "cx": cx, "cy": cy,
                    "w": o["w"], "h": o["h"], "cf": cf})
    return out


def _maybe_refine_ring(img, o):
    """可选精修：在数字框外扩区域找圆环线，拟合圆心。

    未配置时直接返回数字框中心（粗位置，约 2 环量级）。
    实现时禁止对数字笔画做质心/拟合 —— 见 docs/next-steps.md。
    """
    if C.RING_REFINE in (None, "none"):
        return o["cx"], o["cy"]
    # TBD: 外扩 ROI → 找虚线环/实心环边缘 → 拟合圆心
    # 精修必须用原图分辨率。
    return o["cx"], o["cy"]


def detect_material(img):
    """找物料并判颜色。

    返回 list[dict]：{"c": 颜色编号, "cx": px, "cy": px, "area": px2, "cf": 0-100}

    实现要点：
      - 颜色编号来自 .mud 的 labels（见 model_meta），**绝不能直接用 class_id**
      - 5黑物料 vs 黑障碍物 Φ50×100：颜色和俯视轮廓都可能一样。
        用 bbox 宽高比 + 面积上下限区分；更可靠的是靠高度差（需另加传感器）
      - 3蓝 vs 6浅蓝 由模型负责，但必须用真实物料验证，不能只看训练集准确率
    """
    d = _get_detector("color")
    dets = d.detect(img)
    out = []
    for o in dets:
        cf = int(round(o["score"] * 100))
        if C.COLOR_MIN_CONF is not None and cf < C.COLOR_MIN_CONF:
            continue
        area = o["w"] * o["h"]
        if C.COLOR_MIN_AREA_PX is not None and area < C.COLOR_MIN_AREA_PX:
            continue
        if C.COLOR_MAX_AREA_PX is not None and area > C.COLOR_MAX_AREA_PX:
            continue
        out.append({"c": o["cls_id"], "cx": o["cx"], "cy": o["cy"],
                    "area": area, "cf": cf})
    return out


def ask_mode_targets(mode, img):
    """按模式跑对应检测。主循环只调这一个，不关心细节。"""
    if mode == "RING":
        return detect_ring(img)
    if mode == "MAT":
        return detect_material(img)
    return []


def to_targets(dets, mode):
    """检测结果 → 契约目标列表。这是唯一把像素换算成 mm 的地方。"""
    out = []
    for d in dets:
        ex, ey, ea = to_error(d["cx"], d["cy"])
        cf = int(d.get("cf", 0))
        if C.RING_MIN_CONF is not None and mode == "RING" and cf < C.RING_MIN_CONF:
            continue
        out.append({"c": d["c"], "ex": ex, "ey": ey, "ea": ea,
                    "px": d["cx"], "py": d["cy"], "cf": cf})
    return out
