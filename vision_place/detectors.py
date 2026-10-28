"""
YOLOv5 检测器薄壳 —— 硬件相关，全部标 [VERIFY]

【本文件只做三件事】
  1) 加载模型
  2) 调 detect()
  3) 把 class_id 换成业务ID（用 model_meta，绝不用 class_id 当业务ID）

像素→mm 的换算在 perceive.to_error()，不在这里。
"""

import os

import config as C
import model_meta


class DetectorError(Exception):
    pass


def _get(obj, names):
    """从对象或 dict 里按多个候选名取属性。"""
    for n in names:
        if hasattr(obj, n):
            v = getattr(obj, n)
            if callable(v):
                try:
                    v = v()
                except TypeError:
                    continue
            return v
    for n in names:
        try:
            if n in obj:
                return obj[n]
        except TypeError:
            break
    return None


class YoloDetector:
    """一个 YOLOv5 模型的包装。"""

    def __init__(self, mud_path, name=""):
        self.name = name or os.path.basename(mud_path)
        self.mud_path = mud_path
        self.meta = model_meta.load_mud(mud_path)
        self.model = None
        self.input_w = None
        self.input_h = None
        self._load()

    def _load(self):
        if not C.USE_MODEL:
            print("[detector:%s] USE_MODEL=False，不加载" % self.name)
            return
        try:
            from maix import nn
        except Exception as e:
            raise DetectorError("无 maix.nn 环境: %r" % (e,))

        mf = self.meta.get("model_file")
        if not mf:
            raise DetectorError(".mud 里没有 model= 字段: %s" % self.mud_path)
        model_path = os.path.join(self.meta["dir"], mf)

        # [VERIFY] 加载方式。优先直接传 .mud（会自动读 labels/anchors）。
        #   若不支持，改成传 model_path 并显式给 labels/anchors/net_len。
        used = None
        for kwargs in (
            {"model": self.mud_path, "dual_buff": True},   # 首选：直接给 .mud
            {"model": self.mud_path},
            {"model": model_path, "dual_buff": True},      # 退路：给 .cvimodel
            {"model": model_path},
        ):
            try:
                self.model = nn.YOLOv5(**kwargs)
                used = kwargs
                break
            except Exception:
                continue
        if self.model is None:
            raise DetectorError(
                "所有加载方式都失败。请手动确认 nn.YOLOv5 的签名，可能需要显式传 "
                "labels=%s anchors=%s net_len=<边长>"
                % (self.meta["labels"], self.meta["anchors"])
            )
        print("[detector:%s] 已加载 %s，labels=%s ids=%s"
              % (self.name, used, self.meta["labels"], self.meta["ids"]))

        # [VERIFY] 输入尺寸属性名
        for attr in ("input_w", "input_width", "net_len"):
            if hasattr(self.model, attr):
                self.input_w = getattr(self.model, attr)
                break
        for attr in ("input_h", "input_height", "net_len"):
            if hasattr(self.model, attr):
                self.input_h = getattr(self.model, attr)
                break
        if self.input_w is None:
            print("[detector:%s] 取不到输入尺寸，ROI 换算禁用" % self.name)

    def detect(self, img):
        """返回 list[dict]，坐标是原图像素。

        每项: {"cls_id": 业务ID, "cls_name": 标签,
               "cx","cy","w","h" 原图像素, "score": float}
        """
        if self.model is None:
            return []

        conf, iou = C.MODEL_CONF_TH, C.MODEL_IOU_TH
        if conf is None or iou is None:
            raise DetectorError(
                "config.MODEL_CONF_TH / MODEL_IOU_TH 未填。这两个必须现场调，不许写死。"
            )

        # [VERIFY] detect 签名
        try:
            objs = self.model.detect(img, conf_th=conf, iou_th=iou)
        except TypeError:
            objs = self.model.detect(img, conf, iou)

        out = []
        for o in objs:
            cid = _get(o, ("class_id", "classid", "cls", "label"))
            if cid is None:
                continue
            bid = model_meta.id_of(self.meta, int(cid))
            if bid is None:
                print("[detector:%s] 越界 class_id=%s，丢弃" % (self.name, cid))
                continue

            x = _get(o, ("x",)) or 0.0
            y = _get(o, ("y",)) or 0.0
            w = _get(o, ("w", "width")) or 0.0
            h = _get(o, ("h", "height")) or 0.0
            score = _get(o, ("score", "conf", "prob")) or 0.0

            # [VERIFY-1] 若返回的是"模型输入坐标系"，需乘 原图/输入 比例。
            # 判断：拿 640x480 的图，若所有框的 x+w 明显 ≤ 320，就是没映射。
            out.append({
                "cls_id": bid,
                "cls_name": model_meta.name_of(self.meta, bid),
                "cx": float(x) + float(w) / 2.0,
                "cy": float(y) + float(h) / 2.0,
                "w": float(w),
                "h": float(h),
                "score": float(score),
            })
        return out


def make_color_detector():
    return YoloDetector(os.path.join(C.MODEL_DIR, C.MODEL_COLOR_MUD), "color")


def make_ring_detector():
    return YoloDetector(os.path.join(C.MODEL_DIR, C.MODEL_RING_MUD), "ring")
