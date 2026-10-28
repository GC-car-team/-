"""
任务码校验 —— 纯逻辑，不依赖任何硬件。

单独成文件的原因：
  这是整个扫码节点最高风险的代码。find_qrcodes 在图像模糊时会"解出"
  一个错误的串，如果直接下发，一次误读就能让整场比赛报废。
  所以校验逻辑必须能在 PC 上逐个用例测，而不是等到板子上才发现。

自测: python qr_validate.py
"""

import config as C


def check_format(payload):
    """格式检查：4 组、每组 3 位、全数字、用 + 连接。返回 (ok, groups)"""
    if payload is None:
        return False, None
    s = payload.strip()
    if not s:
        return False, None
    parts = s.split("+")
    if len(parts) != C.GROUP_COUNT:
        return False, None
    for p in parts:
        if len(p) != C.GROUP_LEN or not p.isdigit():
            return False, None
    return True, parts


def check_semantic(groups):
    """语义检查：颜色组必须 1-6，位置组必须 1-3。返回 (ok, reason)"""
    if groups is None:
        return False, "format"
    for i, g in enumerate(groups):
        is_color = C.GROUP_IS_COLOR[i] if i < len(C.GROUP_IS_COLOR) else False
        lo, hi = (C.COLOR_MIN, C.COLOR_MAX) if is_color else (C.POS_MIN, C.POS_MAX)
        for ch in g:
            d = int(ch)
            if not (lo <= d <= hi):
                kind = "颜色" if is_color else "位置"
                return False, "第%d组 %s位=%d 超出 %d-%d" % (i + 1, kind, d, lo, hi)
    return True, ""


def classify(payload):
    """把一次解码结果归类。返回 (kind, text, reason)

    kind: "TASK"(可用) | "SUSPECT"(可能误读) | "RAW"(格式不符)
    """
    ok, groups = check_format(payload)
    if not ok:
        return "RAW", (payload or "").strip(), "格式不符"
    ok2, reason = check_semantic(groups)
    if not ok2:
        return "SUSPECT", "+".join(groups), reason
    return "TASK", "+".join(groups), ""


# ==================================================================
# 自测
# ==================================================================
def _selftest():
    cases = [
        # (输入, 期望kind, 说明)
        ("156+123+516+231", "TASK", "规则原文示例"),
        ("156+123+516+231\n", "TASK", "带换行"),
        ("  156+123+516+231  ", "TASK", "带空格"),

        ("156+123+516", "RAW", "只有3组"),
        ("156+123+516+231+1", "RAW", "5组"),
        ("15+123+516+231", "RAW", "某组只有2位"),
        ("1561+123+516+231", "RAW", "某组4位"),
        ("156+12a+516+231", "RAW", "含字母"),
        ("156+123+516+231x", "RAW", "尾部多余字符"),
        ("", "RAW", "空串"),
        (None, "RAW", "None"),

        ("157+123+516+231", "SUSPECT", "颜色组出现7 (误读)"),
        ("156+124+516+231", "SUSPECT", "位置组出现4 (误读)"),
        ("156+023+516+231", "SUSPECT", "位置0 越界"),
        ("166+123+516+231", "TASK", "颜色6 合法"),
        ("111+111+111+111", "TASK", "全1 合法"),
        ("666+333+666+333", "TASK", "全上界 合法"),
    ]
    fails = 0
    for inp, want, why in cases:
        kind, text, reason = classify(inp)
        ok = (kind == want)
        if not ok:
            fails += 1
        print("  %-4s %-28r -> %-8s %s" % ("ok" if ok else "FAIL", inp, kind, why))
        if not ok:
            print("       期望 %s，实际 %s (%s)" % (want, kind, reason))

    print()
    if fails:
        print("qr_validate selftest: %d FAILED" % fails)
        return 1
    print("qr_validate selftest: OK (%d cases)" % len(cases))
    return 0


if __name__ == "__main__":
    import sys
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # Windows 控制台中文乱码修正
    except Exception:
        pass
    sys.exit(_selftest())
