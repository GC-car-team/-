"""
桌面 / 板上通用的 gate 检查（环号级，不验毫米）

PC 上（无模型）：
  python eval_task.py --list
  → 扫描 data/gate/**，按文件名解析期望业务ID并打印清单

板上（有模型 + 图片路径可读时）：
  python eval_task.py --run-ring
  → 对每张图跑环数字模型，对比文件名期望，输出 PASS/FAIL

文件名规则（见 data/gate/README.md）：
  ring1_01.jpg → 期望业务ID = 1
  取文件名中第一个连续数字。
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(ROOT, "data", "gate")
FIRST_NUM = re.compile(r"(\d+)")


def expected_id(filename):
    m = FIRST_NUM.search(filename)
    return int(m.group(1)) if m else None


def iter_images(sub=None):
    base = os.path.join(GATE, sub) if sub else GATE
    if not os.path.isdir(base):
        return
    for dirpath, _, files in os.walk(base):
        for fn in files:
            low = fn.lower()
            if low.endswith((".jpg", ".jpeg", ".png", ".bmp")):
                yield os.path.join(dirpath, fn), expected_id(fn)


def cmd_list():
    rows = list(iter_images())
    if not rows:
        print("data/gate 下还没有图片。")
        print("把手机圆环照放到 data/gate/ring_phone/，命名如 ring1_01.jpg")
        return 0
    print("找到 %d 张：" % len(rows))
    bad = 0
    for path, exp in rows:
        rel = os.path.relpath(path, GATE)
        if exp is None:
            print("  ?  %s   ← 文件名里没有数字，无法推断期望ID" % rel)
            bad += 1
        else:
            print("  %s  expect=%s" % (rel, exp))
    print("可解析 %d / %d" % (len(rows) - bad, len(rows)))
    return 1 if bad else 0


def cmd_run_ring():
    try:
        from maix import image
        import detectors
        import config as C
    except Exception as e:
        print("需要在 MaixCAM 上跑 --run-ring。当前环境: %r" % (e,))
        print("PC 上请先用 --list 归档照片。")
        return 2

    if C.MODEL_CONF_TH is None or C.MODEL_IOU_TH is None:
        print("先填 config.MODEL_CONF_TH / MODEL_IOU_TH")
        return 2

    det = detectors.make_ring_detector()
    total = hit = miss = empty = 0
    for path, exp in iter_images():
        if exp is None:
            continue
        total += 1
        try:
            img = image.load(path)  # [VERIFY] API 名可能是 image.load / Image
        except Exception:
            # 退路：部分固件用 open
            try:
                img = image.Image(path)
            except Exception as e:
                print("FAIL load %s: %r" % (path, e))
                miss += 1
                continue
        objs = det.detect(img)
        if not objs:
            print("MISS  %s expect=%s  (无检出)" % (os.path.basename(path), exp))
            empty += 1
            continue
        # 取最高分
        best = max(objs, key=lambda o: o["score"])
        ok = best["cls_id"] == exp
        print("%s  %s expect=%s got=%s sc=%.2f" % (
            "PASS" if ok else "FAIL", os.path.basename(path),
            exp, best["cls_id"], best["score"]))
        if ok:
            hit += 1
        else:
            miss += 1
    print("---")
    print("total=%d pass=%d fail=%d empty=%d" % (total, hit, miss, empty))
    return 0 if total and miss == 0 and empty == 0 else 1


def main(argv):
    if len(argv) < 2 or argv[1] in ("-h", "--help"):
        print(__doc__)
        return 2
    if argv[1] == "--list":
        return cmd_list()
    if argv[1] == "--run-ring":
        return cmd_run_ring()
    print("未知参数。用 --list 或 --run-ring")
    return 2


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    sys.exit(main(sys.argv))
