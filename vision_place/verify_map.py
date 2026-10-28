"""
桌上验证：class_id → 业务ID 是否正确

用法（在 MaixCAM Pro 上）：
  1. 把 1:1 打印的环图平放桌上
  2. 填 config.MODEL_CONF_TH / MODEL_IOU_TH（可先 0.45 / 0.45）
  3. config.SRC = "cam"
  4. python verify_map.py ring     # 验环数字
     python verify_map.py color    # 验颜色（没有真实物料时意义有限）

屏幕上每个框旁会显示：
  ID=<业务ID>  raw=<class_id>  sc=<分数>
用肉眼核对：对着「1」必须看到 ID=1（不要看 raw）。

无 MaixPy 时（PC）：只打印模型映射表，不跑相机。
"""

import sys
import time

import config as C
import model_meta


def print_mapping(kind):
    mud = C.MODEL_RING_MUD if kind == "ring" else C.MODEL_COLOR_MUD
    path = __import__("os").path.join(C.MODEL_DIR, mud)
    info = model_meta.load_mud(path)
    print("=== %s 映射表 ===" % kind)
    print("mud: %s" % mud)
    print("labels: %s" % info["labels"])
    print("ids:    %s" % info["ids"])
    print()
    print("class_id -> 业务ID")
    for i, (lab, bid) in enumerate(zip(info["labels"], info["ids"])):
        print("  %d  %s  ->  %s" % (i, lab, bid))
    print()
    print("核对清单：")
    if kind == "ring":
        print("  画面里的数字 1 → 屏幕应显示 ID=1")
        print("  画面里的数字 2 → ID=2")
        print("  画面里的数字 3 → ID=3")
        print("  若 ID 与数字相反，说明映射没接上或接错模型")
    else:
        print("  红→1  黄→2  蓝→3  绿→4  黑→5  浅蓝→6")
        print("  桌面无真实物料时，本模式只做冒烟，不作为验收")


def run_cam(kind):
    if C.MODEL_CONF_TH is None or C.MODEL_IOU_TH is None:
        print("请先在 config.py 填 MODEL_CONF_TH / MODEL_IOU_TH，例如都设 0.45")
        return 1

    from maix import camera, display, image
    import detectors

    print_mapping(kind)
    det = detectors.make_ring_detector() if kind == "ring" else detectors.make_color_detector()
    cam = camera.Camera(C.CAM_WIDTH, C.CAM_HEIGHT)
    disp = display.Display()

    print("开始预览。Ctrl+C 结束。请用肉眼核对立牌/打印件上的数字与 ID= 是否一致。")
    while True:
        img = cam.read()
        objs = det.detect(img)
        # 需要 class_id 原文做对照：再跑一层 meta（detect 已映射）
        # 这里只能显示业务ID；raw class_id 需要从模型再取。
        # 简化：显示业务ID + score。映射表已在启动时打印。
        for o in objs:
            x = int(o["cx"] - o["w"] / 2)
            y = int(o["cy"] - o["h"] / 2)
            w, h = int(o["w"]), int(o["h"])
            img.draw_rect(x, y, w, h, color=(0, 255, 0), thickness=2)
            label = "ID=%s %s sc=%.2f" % (o["cls_id"], o["cls_name"], o["score"])
            img.draw_string(x, max(0, y - 16), label, color=(255, 255, 0))
        img.draw_string(4, 4, "verify %s  conf=%.2f" % (kind, C.MODEL_CONF_TH),
                        color=(0, 255, 255))
        disp.show(img)


def main(argv):
    kind = argv[1] if len(argv) > 1 else "ring"
    if kind not in ("ring", "color"):
        print("用法: python verify_map.py ring|color")
        return 2

    # PC 上也能跑映射表
    try:
        from maix import camera  # noqa: F401
        has = True
    except Exception:
        has = False

    if not has:
        print("[PC 模式] 只打印映射表。拷到板上再跑相机预览。\n")
        print_mapping(kind)
        return 0

    return run_cam(kind) or 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print("stopped")
