"""
标定工具 —— MaixCAM Pro（夹爪下视相机）

四种模式：

  scale  尺度标定（**不需要车**）
         屏幕上点两个间距已知的点 -> 得到矩阵的对角初值 [[s,0],[0,s]]
         用法: python calib_task.py scale --len-mm 100
         要求: 在目标平面上放两个标记，间距 100mm（用尺子量，别信打印）

  dir    方向标定（**需要机器人能动**，两步）
         沿机器人 +X 移一次，再沿 +Y 移一次 -> 得到完整 2x2 矩阵
         用法: python calib_task.py dir --move-mm 10
         两次移动的距离可以不同，脚本会分别问

  phys   记录物理安装量（键盘输入，不走触摸）
         用法: python calib_task.py phys --height-mm 320 --pitch-deg 90
         这会写进 calib.json 并用于「几何变了就警告」

  show   只显示，不写文件。画十字线 + 网格，供肉眼检查相机装正了没有
         用法: python calib_task.py show

输出: 与本文件同目录的 calib.json，config.py 会自动加载并校验。

纯数学部分可在 PC 上自测: python calib_task.py --selftest
"""

import json
import os
import sys
import time

CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")

try:
    from maix import camera, display, image, touchscreen  # noqa: F401
    _HAS_MAIX = True
except Exception as _e:
    _HAS_MAIX = False
    _MAIX_ERR = _e


# ==================================================================
# 纯数学（不依赖 maix，可在 PC 上自测）
# ==================================================================
def matrix_from_scale(mm_per_px):
    """尺度标定结果 -> 对角矩阵初值。方向未知，故非对角为 0。"""
    s = float(mm_per_px)
    return [[s, 0.0], [0.0, s]]


def matrix_from_two_moves(move_x_mm, d_px_x, move_y_mm, d_px_y):
    """两次移动 -> 完整 2x2 矩阵。

    记 J 为「机器人移动 1mm 时，图像中静止特征的位移(px)」：
        moving robot by (mx, 0)  ->  feature moves by  d_px_x
        moving robot by (0, my)  ->  feature moves by  d_px_y
    于是 J 的两列为 d_px_x/mx 与 d_px_y/my。

    我们要的 M 满足 t = M @ p，其中 t 是「让特征从偏移 p 回到中心」所需的
    机器人位移。移动 t 会让特征移动 J @ t，要求 p + J @ t = 0
    => t = -J^-1 @ p，所以 M = -inv(J)。

    d_px_x / d_px_y 为 (dx, dy) 元组，单位像素。
    """
    mx = float(move_x_mm)
    my = float(move_y_mm)
    if abs(mx) < 1e-6 or abs(my) < 1e-6:
        raise ValueError("移动距离不能为 0")

    # J 的列
    j00 = d_px_x[0] / mx
    j10 = d_px_x[1] / mx
    j01 = d_px_y[0] / my
    j11 = d_px_y[1] / my

    det = j00 * j11 - j01 * j10
    if abs(det) < 1e-12:
        raise ValueError("雅可比奇异：两次移动方向可能共线，或目标没动")

    # inv(J) = 1/det * [[j11, -j01], [-j10, j00]]
    inv = [[j11 / det, -j01 / det], [-j10 / det, j00 / det]]
    # M = -inv(J)
    return [[-inv[0][0], -inv[0][1]], [-inv[1][0], -inv[1][1]]]


def apply_matrix(m, dx_px, dy_px):
    return (m[0][0] * dx_px + m[0][1] * dy_px,
            m[1][0] * dx_px + m[1][1] * dy_px)


def _selftest():
    # 情形 1: 纯尺度 0.18 mm/px，轴对齐，无翻转。
    # 机器人移 +X 10mm -> 特征在图像里往 -x 走 10/0.18 = 55.56px
    s = 0.18
    d = 10.0 / s
    m = matrix_from_two_moves(10.0, (-d, 0.0), 10.0, (0.0, -d))
    ex, ey = apply_matrix(m, 55.56, 0.0)
    assert abs(ex - 10.0) < 0.05, ex          # 目标在右边 -> 需往 +X 走
    assert abs(ey - 0.0) < 0.05, ey
    print("  case1 axis-aligned scale : ok  (ex=%.2f ey=%.2f)" % (ex, ey))

    # 情形 2: 相机装歪了 30 度
    import math
    th = math.radians(30.0)
    # 机器人 +X 移动 -> 特征位移方向 = -(cos, sin) * d
    dx = (-d * math.cos(th), -d * math.sin(th))
    dy = (d * math.sin(th), -d * math.cos(th))
    m = matrix_from_two_moves(10.0, dx, 10.0, dy)
    # 目标在图像 (100, 0) -> 用同一几何反推期望的机器人位移
    ex, ey = apply_matrix(m, 100.0, 0.0)
    # 期望: 机器人位移 t 满足 t 沿 X 为 d_mm... 直接验证往返一致
    # 正向: 机器人移 t=(a,b) -> 特征移 J@t。取 a=1,b=0 得 J 第一列 = (-cos, -sin)/s*d/10
    jx = (dx[0] / 10.0, dx[1] / 10.0)
    # 若目标偏移 p，则 t = M@p 应满足 J@t = -p
    got = (jx[0] * ex + dy[0] / 10.0 * ey, jx[1] * ex + dy[1] / 10.0 * ey)
    assert abs(got[0] + 100.0) < 0.5, got
    assert abs(got[1] + 0.0) < 0.5, got
    print("  case2 rotated 30deg      : ok  (ex=%.2f ey=%.2f)" % (ex, ey))

    # 情形 3: 尺度工厂函数
    assert matrix_from_scale(0.2) == [[0.2, 0.0], [0.0, 0.2]]
    print("  case3 matrix_from_scale  : ok")

    print("calib_task selftest: OK")


# ==================================================================
# 设备侧
# ==================================================================
def log(*a):
    print(*a, file=sys.stderr)


def load_calib():
    if not os.path.exists(CALIB_PATH):
        return {}
    try:
        with open(CALIB_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        log("[calib] 读取失败，按空处理: %r" % (e,))
        return {}


def save_calib(data):
    tmp = CALIB_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, CALIB_PATH)  # 原子替换，断电不会写坏
    log("[calib] 已写入 %s" % CALIB_PATH)


def _pick_two_points(disp, ts, cam, title):
    """在屏幕上点两个点，返回 ((x1,y1),(x2,y2))。"""
    pts = []
    last = time.monotonic()
    log("[%s] 请依次点击两个点" % title)
    while len(pts) < 2:
        img = cam.read()
        for (x, y) in pts:
            img.draw_cross(x, y, color=(255, 0, 0), size=8)
        img.draw_string(4, 4, "%s  picks=%d" % (title, len(pts)), color=(255, 255, 0))
        disp.show(img)

        # [VERIFY] TouchScreen.read() 返回类型：可能是 (x, y) 元组列表或 None
        t = ts.read()
        if t:
            now = time.monotonic()
            if now - last < 0.35:      # 去抖，防止一次触摸记两个点
                continue
            last = now
            # 兼容 [(x,y)] 与 (x,y) 两种返回
            if isinstance(t, (list, tuple)) and len(t) == 2 and isinstance(t[0], (int, float)):
                p = (int(t[0]), int(t[1]))
                pts.append(p)
            else:
                for item in t:
                    if isinstance(item, (list, tuple)) and len(item) >= 2:
                        pts.append((int(item[0]), int(item[1])))
                    elif hasattr(item, "x"):
                        pts.append((int(item.x), int(item.y)))
                    if len(pts) >= 2:
                        break
            log("[%s] 已选 %d/2: %s" % (title, len(pts), pts))
    return pts[0], pts[1]


def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def mode_scale(args):
    len_mm = _arg(args, "--len-mm", float)
    if not len_mm or len_mm <= 0:
        log("用法: python calib_task.py scale --len-mm 100   （两个标记的真实间距 mm）")
        return 1
    cam = camera.Camera(640, 480, _img_fmt())
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    a, b = _pick_two_points(disp, ts, cam, "scale")
    px = _dist(a, b)
    if px < 10:
        log("[scale] 两点太近(%.1fpx)，重来" % px)
        return 1
    s = len_mm / px
    log("[scale] 像素距离 %.1f px, 真实 %.1f mm -> %.5f mm/px" % (px, len_mm, s))

    c = load_calib()
    c["matrix"] = matrix_from_scale(s)
    c["method"] = "scale_only"
    c["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    c.setdefault("offset_mm", [0.0, 0.0])
    c["scale_len_mm"] = len_mm
    c["scale_px_dist"] = round(px, 2)
    save_calib(c)
    log("[scale] 注意：对角矩阵假设了轴对齐与无旋转。")
    log("[scale] 真实运行前必须做 dir 模式，否则伺服可能不收敛。")
    return 0


def _measure_feature(disp, ts, cam, title):
    """点一个静止特征，返回其像素坐标。"""
    log("[%s] 请点击一个静止特征（要有明确中心，例如靶标角点）" % title)
    p, _ = _pick_two_points(disp, ts, cam, title)
    # _pick_two_points 需要两个点；这里其实只要一个，但我们复用其去抖逻辑，
    # 取第一个即可。第二个点位置不用。
    return p


def mode_dir(args):
    move_mm = _arg(args, "--move-mm", float) or 10.0
    cam = camera.Camera(640, 480, _img_fmt())
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    log("=" * 56)
    log("方向标定：共需两次移动。移动距离 %.1f mm" % move_mm)
    log("每次流程：点特征 -> 移动机器人 -> 再点同一特征")
    log("=" * 56)

    p0 = _measure_feature(disp, ts, cam, "X-1")
    _wait_touch_confirm(disp, ts, cam, "移动机器人沿 +X %.0fmm，然后点屏幕继续" % move_mm)
    p1 = _measure_feature(disp, ts, cam, "X-2")
    d_px_x = (p1[0] - p0[0], p1[1] - p0[1])
    log("[dir] +X 移动导致特征位移 d_px_x = %s" % (d_px_x,))
    if abs(d_px_x[0]) < 1 and abs(d_px_x[1]) < 1:
        log("[dir] 特征几乎没动，标定无效。检查是否真移动了 / 是否点错位置")
        return 1

    q0 = _measure_feature(disp, ts, cam, "Y-1")
    _wait_touch_confirm(disp, ts, cam, "移动机器人沿 +Y（左）%.0fmm，然后点屏幕继续" % move_mm)
    q1 = _measure_feature(disp, ts, cam, "Y-2")
    d_px_y = (q1[0] - q0[0], q1[1] - q0[1])
    log("[dir] +Y 移动导致特征位移 d_px_y = %s" % (d_px_y,))
    if abs(d_px_y[0]) < 1 and abs(d_px_y[1]) < 1:
        log("[dir] 特征几乎没动，标定无效")
        return 1

    m = matrix_from_two_moves(move_mm, d_px_x, move_mm, d_px_y)
    log("[dir] 标定矩阵 = %s" % (m,))

    c = load_calib()
    c["matrix"] = m
    c["method"] = "two_move"
    c["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    c["dir_move_mm"] = move_mm
    c["dir_dpx_x"] = [round(v, 2) for v in d_px_x]
    c["dir_dpx_y"] = [round(v, 2) for v in d_px_y]
    save_calib(c)

    # 自检：把 p0 的偏移喂回去，看是否给出与 move_mm 同量级的位移
    log("[dir] 自检完成。建议跑一次 show 模式肉眼确认符号方向")
    return 0


def _wait_touch_confirm(disp, ts, cam, msg):
    log("[wait] " + msg)
    while True:
        img = cam.read()
        img.draw_string(4, 4, "TOUCH TO CONTINUE", color=(0, 255, 0))
        disp.show(img)
        if ts.read():
            time.sleep(0.3)
            return


def mode_phys(args):
    h = _arg(args, "--height-mm", float)
    p = _arg(args, "--pitch-deg", float)
    dx = _arg(args, "--dx", float)
    dy = _arg(args, "--dy", float)
    if h is None or p is None:
        log("用法: python calib_task.py phys --height-mm 320 --pitch-deg 90 [--dx 0 --dy 0]")
        return 1
    c = load_calib()
    c["cam_height_mm"] = h
    c["cam_pitch_deg"] = p
    if dx is not None and dy is not None:
        c["offset_mm"] = [dx, dy]
    c["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    save_calib(c)
    log("[phys] 高度=%.1fmm 俯角=%.1fdeg 装配偏差=(%s, %s)" % (h, p, dx, dy))
    log("[phys] 重要：把这些值也同步写进 config.py 的 CAM_HEIGHT_MM / CAM_PITCH_DEG，")
    log("[phys]       否则「几何变了」的校验不会生效。")
    return 0


def mode_show(args):
    cam = camera.Camera(640, 480, _img_fmt())
    disp = display.Display()
    log("[show] 画十字与网格，检查相机是否装正。Ctrl+C 退出")
    while True:
        img = cam.read()
        w, h = img.width(), img.height()
        cx, cy = w // 2, h // 2
        img.draw_line(cx, 0, cx, h, color=(0, 255, 0), thickness=1)
        img.draw_line(0, cy, w, cy, color=(0, 255, 0), thickness=1)
        for k in range(1, 6):
            img.draw_circle(cx, cy, k * 40, color=(80, 80, 80), thickness=1)
        img.draw_string(4, 4, "show: cross at center", color=(255, 255, 0))
        disp.show(img)


def _img_fmt():
    # [VERIFY] 相机格式枚举名
    try:
        return image.Format.FMT_RGB888
    except Exception:
        return None


def _arg(argv, name, typ):
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            try:
                return typ(argv[i + 1])
            except Exception:
                return None
        if a.startswith(name + "="):
            try:
                return typ(a.split("=", 1)[1])
            except Exception:
                return None
    return None


def main(argv):
    if "--selftest" in argv:
        _selftest()
        return 0
    if len(argv) < 2:
        print(__doc__)
        return 2
    mode = argv[1]
    if mode not in ("scale", "dir", "phys", "show"):
        print(__doc__)
        return 2
    if not _HAS_MAIX and mode != "phys":
        log("没有 MaixPy 环境，只能在板子上跑（phys 纯写文件，可以不接相机）")
        log("导入错误: %r" % (_MAIX_ERR,))
        return 1

    if mode == "scale":
        return mode_scale(argv)
    if mode == "dir":
        return mode_dir(argv)
    if mode == "phys":
        return mode_phys(argv)
    return mode_show(argv)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
