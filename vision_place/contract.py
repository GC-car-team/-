"""
契约 v1.0 —— MaixCAM Pro 视觉端 → 主控

【本文件是 FROZEN 层】
  - 字段名、单位、坐标系、符号约定一旦改动，必须升 CV 版本号并通知主控
  - encode() 是唯一的序列化入口。任何模块都不许自己拼字符串
  - 视觉端不保存任何业务映射（颜色→任务位置），那是主控的事

【单位与坐标约定（冻结）】
  长度      : mm，保留 1 位小数
  坐标系    : 机器人坐标系，X 向前、Y 向左、Z 向上
  ex 正方向 : 夹爪沿机器人 +X 移动
  ey 正方向 : 夹爪沿机器人 +Y 移动（左）
  ea 正方向 : 从上方看，逆时针为正（度）
  t 时基    : 设备启动后毫秒，单调递增（monotonic）
  失效判据  : 连续 200ms 无新消息，主控必须停止动作
  分帧      : '\\n' 结尾的 ASCII 行，不加密不压缩（出问题能肉眼读）
"""

import json

# ---- 版本：改任何字段语义都必须 +1 ----
CV = "1.0"

# ---- 状态（st）：接收方必须据此决定是否动作 ----
ST_OK = 0        # 数据可信，可全速动作
ST_DEGRADED = 1  # 有数据但质量低，应降速/保守
ST_LOST = 2      # 无可信目标，禁止沿用上一帧

# ---- 模式（md）：视觉端当前在看什么 ----
MD_IDLE = "IDLE"  # 不看，只发心跳
MD_RING = "RING"  # 看圆环 1/2/3
MD_MAT = "MAT"    # 看物料颜色

MODES = (MD_IDLE, MD_RING, MD_MAT)


def make_target(c, ex, ey, ea=0.0, px=0.0, py=0.0, cf=0):
    """单个目标。

    c   : RING 模式=环序号(1-3) / MAT 模式=颜色编号(1-6)
    ex  : mm，夹爪需沿机器人 +X 移动的量
    ey  : mm，夹爪需沿机器人 +Y 移动的量
    ea  : deg，需旋转量（无旋转机构则恒 0）
    px py: 像素坐标，仅调试用
    cf  : 置信度 0-100 整数（省字节，也避免浮点打印差异）
    """
    return {
        "c": int(c),
        "ex": round(float(ex), 1),
        "ey": round(float(ey), 1),
        "ea": round(float(ea), 1),
        "px": round(float(px), 1),
        "py": round(float(py), 1),
        "cf": int(cf),
    }


def make_msg(seq, t_ms, dt_ms, mode, status, targets):
    return {
        "cv": CV,
        "seq": int(seq),
        "t": int(t_ms),
        "dt": int(dt_ms),
        "md": mode,
        "st": int(status),
        "n": len(targets),
        "tg": targets,
    }


def encode(msg):
    """唯一序列化入口。返回 bytes，以 \\n 结尾。

    用紧凑分隔符：一行更短，串口和人工阅读都更容易。
    """
    return (json.dumps(msg, separators=(",", ":"), ensure_ascii=True) + "\n").encode("ascii")


def decode(text):
    """反向解析，只用于自测（check round-trip）。"""
    return json.loads(text)


def parse_set_command(line):
    """解析主控下发指令。返回模式字符串或 None。

    支持: 'SET:RING' / 'SET:MAT' / 'SET:IDLE'（大小写不敏感，允许 \\r）
    """
    s = line.strip().upper()
    if not s.startswith("SET:"):
        return None
    m = s[4:].strip()
    return m if m in MODES else None


# ---- 自测：python contract.py ----
if __name__ == "__main__":
    m = make_msg(1, 123456, 12, MD_RING, ST_OK, [make_target(2, -3.24, 1.71, 0.0, 412.3, 233.1, 91)])
    raw = encode(m)
    print("encoded:", raw)
    back = decode(raw.decode("ascii").strip())
    assert back["cv"] == CV
    assert back["n"] == 1
    assert back["tg"][0]["ex"] == -3.2
    assert parse_set_command("set:ring\r\n") == MD_RING
    assert parse_set_command("SET:XXX") is None
    assert parse_set_command("hello") is None
    print("contract self-test: OK")
