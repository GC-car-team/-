"""
模型元数据解析 —— 从 .mud 读类别表，建立 class_id → 业务ID 的映射。

【为什么单独写这个文件】
  你的两个模型的类别编号都不是 0,1,2,3...

  model_320583（颜色）: labels = 1红, 4绿, 2黄, 5黑, 6浅蓝, 3蓝
      class_id:  0    1    2    3    4     5
      业务ID:    1    4    2    5    6     3

  model_321272（圆环）: labels = 3, 2, 1
      class_id:  0  1  2
      业务ID:    3  2  1        ← 正好是反的

  如果按 class_id 当业务ID用（最常见写法），结果会静默全错：
    - 颜色：检测到红色会报成 1（碰巧对），检测到绿色报成 4（碰巧因为顺序），
            但检测到黄色(class_id=2)会报成 2 → 碰巧对，黑色(class_id=3)报成 3 → 错！
            实际上 6 个类别里有 4 个会错。
    - 圆环：完全反了，1环报成3环，3环报成1环。
  而且**不会报错**，只会表现为「精度莫名变差」。

【做法】
  从 .mud 的 labels 里解析每个标签的首位数字作为业务ID。
  这样重新训练后标签变了，映射自动跟着变，不需要改代码。
"""

import os
import re

# ============================================================
# 解析
# ============================================================


class MudError(Exception):
    pass


def _read_text(path):
    raw = open(path, "rb").read()
    for enc in ("utf-8", "utf-8-sig", "gbk", "latin-1"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    raise MudError("无法解码 %s" % path)


def _parse_ini_like(text):
    """解析 [section] key = value 结构。"""
    out = {}
    section = None
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#") or s.startswith(";"):
            continue
        if s.startswith("[") and s.endswith("]"):
            section = s[1:-1].strip()
            out.setdefault(section, {})
            continue
        if "=" in s:
            k, v = s.split("=", 1)
            out.setdefault(section or "default", {})[k.strip()] = v.strip()
    return out


_LEADING_INT = re.compile(r"^\s*(\d+)")


def labels_to_ids(labels):
    """标签列表 → 业务ID列表。

    规则：取每个标签开头的数字。
      "1红" -> 1,  "6浅蓝" -> 6,  "3" -> 3
    取不到数字就报错，不猜。
    """
    ids = []
    for i, lab in enumerate(labels):
        m = _LEADING_INT.match(str(lab))
        if not m:
            raise MudError(
                "第 %d 个标签 %r 开头没有数字，无法推断业务ID。"
                "请把标签改成 '<数字><描述>' 形式，例如 '3蓝'。" % (i, lab)
            )
        ids.append(int(m.group(1)))
    if len(set(ids)) != len(ids):
        raise MudError("业务ID 有重复: %s。标签写错了" % (ids,))
    return ids


def load_mud(path):
    """读 .mud，返回 dict:
        model_file : 同目录下的 .cvimodel 文件名
        model_type : 'yolov5'
        input_type : 'rgb'
        mean, scale: 归一化参数
        anchors    : [float]
        labels     : [str]   原始标签
        ids        : [int]   业务ID，与 class_id 一一对应
    """
    if not os.path.exists(path):
        raise MudError("找不到 %s" % path)
    kv = _parse_ini_like(_read_text(path))
    basic = kv.get("basic", {})
    extra = kv.get("extra", {})

    labels_raw = extra.get("labels", "")
    labels = [x.strip() for x in labels_raw.split(",") if x.strip()]
    if not labels:
        raise MudError("%s 里没有 labels" % path)

    def _floats(s):
        return [float(x) for x in s.split(",") if x.strip()] if s else []

    info = {
        "path": path,
        "dir": os.path.dirname(os.path.abspath(path)),
        "model_file": basic.get("model"),
        "model_type": extra.get("model_type", "").lower(),
        "input_type": extra.get("input_type", "").lower(),
        "mean": _floats(extra.get("mean", "")),
        "scale": _floats(extra.get("scale", "")),
        "anchors": _floats(extra.get("anchors", "")),
        "labels": labels,
        "ids": labels_to_ids(labels),
    }

    # --- 一致性检查 ---
    if len(info["anchors"]) and len(info["anchors"]) % 2 != 0:
        raise MudError("anchors 数量 %d 是奇数" % len(info["anchors"]))
    if len(set(info["ids"])) != len(info["ids"]):
        raise MudError("业务ID 有重复: %s。标签写错了" % (info["ids"],))
    return info


def id_of(info, class_id):
    """class_id → 业务ID。越界返回 None（不猜）。"""
    ids = info["ids"]
    if 0 <= class_id < len(ids):
        return ids[class_id]
    return None


def name_of(info, business_id):
    """业务ID → 原始标签，用于日志。"""
    try:
        i = info["ids"].index(business_id)
        return info["labels"][i]
    except ValueError:
        return str(business_id)


# ============================================================
# 自测：读仓库里的两个真实 .mud
# ============================================================
def _selftest(base_dir=None):
    here = os.path.dirname(os.path.abspath(__file__))
    base = base_dir or os.path.dirname(here)  # 上一级 = 工作区根

    fails = 0

    def check(cond, msg):
        nonlocal fails
        print("  %-4s %s" % ("ok" if cond else "FAIL", msg))
        if not cond:
            fails += 1

    # ---- 颜色模型 ----
    p_color = os.path.join(base, "model_320583.mud")
    if os.path.exists(p_color):
        ci = load_mud(p_color)
        print("\n[颜色模型] %s" % os.path.basename(p_color))
        print("  labels = %s" % (ci["labels"],))
        print("  ids    = %s" % (ci["ids"],))
        check(ci["model_type"] == "yolov5", "model_type=yolov5")
        check(ci["input_type"] == "rgb", "input_type=rgb")
        check(len(ci["anchors"]) == 18, "anchors 18 个（9组）")
        check(len(ci["ids"]) == 6, "6 个类别")
        check(ci["ids"] == [1, 4, 2, 5, 6, 3], "ids 顺序 = [1,4,2,5,6,3]")
        check(set(ci["ids"]) == {1, 2, 3, 4, 5, 6}, "覆盖全部 6 种颜色编号")
        # 关键：证明不能直接用 class_id
        wrong = [i for i in range(6) if ci["ids"][i] != i + 1]
        check(len(wrong) > 0,
              "class_id 不能直接当业务ID（有 %d/%d 个不一致: 索引%s）"
              % (len(wrong), 6, wrong))
        # 逐项验证
        expect = {0: 1, 1: 4, 2: 2, 3: 5, 4: 6, 5: 3}
        ok = all(id_of(ci, k) == v for k, v in expect.items())
        check(ok, "id_of 逐项正确 %s" % expect)
        check(id_of(ci, 99) is None, "越界 class_id 返回 None")
        check(name_of(ci, 6) == "6浅蓝", "name_of(6) = 6浅蓝")
    else:
        print("  skip 颜色模型（%s 不存在）" % p_color)

    # ---- 圆环模型 ----
    p_ring = os.path.join(base, "model_321272.mud")
    if os.path.exists(p_ring):
        ri = load_mud(p_ring)
        print("\n[圆环模型] %s" % os.path.basename(p_ring))
        print("  labels = %s" % (ri["labels"],))
        print("  ids    = %s" % (ri["ids"],))
        check(len(ri["ids"]) == 3, "3 个类别")
        check(ri["ids"] == [3, 2, 1], "ids 顺序 = [3,2,1]（与 class_id 完全相反）")
        check(id_of(ri, 0) == 3, "class_id=0 -> 环3")
        check(id_of(ri, 2) == 1, "class_id=2 -> 环1")
    else:
        print("  skip 圆环模型（%s 不存在）" % p_ring)

    # ---- 异常用例 ----
    print("\n[异常用例]")
    check(_try(lambda: labels_to_ids(["a红"])) is None, "无数字标签 -> 抛错")
    check(_try(lambda: labels_to_ids(["1红", "1蓝"])) is None, "重复ID -> 抛错")

    print()
    if fails:
        print("model_meta selftest: %d FAILED" % fails)
        return 1
    print("model_meta selftest: OK")
    return 0


def _try(fn):
    try:
        r = fn()
        return r
    except MudError:
        return None


if __name__ == "__main__":
    import sys
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    sys.exit(_selftest())
