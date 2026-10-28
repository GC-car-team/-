"""
全部参数与标定值。

【纪律】
  其它任何文件里都不许出现业务数字常量。阈值、比例、ROI 一律从这里读。
  见到 `if cf > 50` 这种就是魔数 bug：改配置无效，而且不报错。

【状态标记】
  FROZEN : 已定，改动要通知主控
  TBD    : 待实测。为 None 时代码必须显式报错或降级，不许自己编一个值

【标定值来源】
  calib_task.py 会写 calib.json；本文件在末尾自动加载并校验。
  校验不通过（相机高度/俯角与记录不符）会打印醒目警告 —— 因为这意味着
  之前的 gate 数据已作废。
"""

import json
import os

# ============================================================
# 1. 运行源
# ============================================================
SRC = "fake"          # "fake" | "cam"

# ============================================================
# 2. 串口（FROZEN：改这里必须通知主控）
# ============================================================
UART_DEV = None            # TBD 例: "/dev/ttyS0"
UART_BAUD = 115200         # FROZEN
UART_TX_PIN = None         # TBD 例: "A16"
UART_RX_PIN = None         # TBD 例: "A17"
UART_TX_FUNC = "UART1_TX"  # TBD 与引脚复用名匹配
UART_RX_FUNC = "UART1_RX"  # TBD

LINK_TIMEOUT_MS = 200      # FROZEN 与契约一致

# ============================================================
# 3. 相机与安装几何（造车前必须定死，改动 -> gate 数据作废）
# ============================================================
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 30
CAM_HEIGHT_MM = None       # TBD 相机光心到靶标面的垂直距离
CAM_PITCH_DEG = None       # TBD 垂直向下 = 90

# ============================================================
# 4. 标定值（由 calib_task.py 写入 calib.json）
# ============================================================
# 2x2 矩阵：图像像素偏移 -> 夹爪需移动的 mm（机器人坐标系）
#   [ex]   [m00 m01]   [px - cx]
#   [ey] = [m10 m11] @ [py - cy]
# None = 未标定，to_error() 会抛 CalibMissing
MM_PER_PX_MATRIX = None

# 相机光心与夹爪中心的装配偏差（mm，机器人坐标系）
# 标定法：把靶标对准图像中心，看工具中心实际落在哪，量偏差
CAM_TO_GRIPPER_MM_X = 0.0
CAM_TO_GRIPPER_MM_Y = 0.0

# ============================================================
# 5. 模型
# ============================================================
USE_MODEL = True
MODEL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MODEL_COLOR_MUD = "model_320583.mud"   # 6类颜色：ids=[1,4,2,5,6,3]
MODEL_RING_MUD = "model_321272.mud"    # 3类圆环：ids=[3,2,1] 与 class_id 相反

# 这两个必须现场调，不许写死。为 None 时 detectors 会抛错。
MODEL_CONF_TH = None    # TBD 建议从 0.5 起试
MODEL_IOU_TH = None     # TBD 建议从 0.45 起试

# ============================================================
# 6. 圆环检测
# ============================================================
RING_COUNT = 3                  # FROZEN 位置编号 1-3
# 图二：内实心环约 3.5mm，外虚线环约 2.5mm。早期「1.5mm」可能不准，以图二为准
RING_SOLID_WIDTH_MM = 3.5
RING_DASH_WIDTH_MM = 2.5
RING_CENTER_SPACING_MM = 150    # FROZEN 图二：相邻环心距
RING_PLATFORM_MM = (580, 150)   # FROZEN 图二：台面
RING_DIAMETERS_MM = None        # TBD 由物料尺寸 phi 算出（内径 φ+3 等）
RING_ROI = None                 # TBD (x, y, w, h)
RING_MIN_CONF = None            # TBD
RING_MIN_AREA_PX = None         # TBD

# 精修：数字模型只给粗位置。必须在数字框外扩区域找圆环线，不能对数字笔画做质心。
# 详见 docs/next-steps.md「精修：数字模型下的正确做法」
RING_REFINE = None              # TBD "none" | "ring_centroid" | "ring_fit"
RING_REFINE_EXPAND_MM = None    # TBD 外扩半径（约 φ/2 + 余量），按高度换算成 px

# ============================================================
# 7. 物料颜色
# ============================================================
COLOR_IDS = (1, 2, 3, 4, 5, 6)  # FROZEN 1红 2黄 3蓝 4绿 5黑 6浅蓝
COLOR_MIN_CONF = None           # TBD
COLOR_MIN_AREA_PX = None        # TBD
COLOR_MAX_AREA_PX = None        # TBD 用于排除黑障碍物

# ============================================================
# 8. 时序
# ============================================================
MAIN_LOOP_SLEEP_MS = 5
STATS_EVERY_N = 30

# ============================================================
# 9. 加载 calib.json 并校验（放在最后，覆盖上面的 TBD）
# ============================================================
CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib.json")


def _load_calib():
    if not os.path.exists(CALIB_PATH):
        return
    global MM_PER_PX_MATRIX, CAM_TO_GRIPPER_MM_X, CAM_TO_GRIPPER_MM_Y
    try:
        with open(CALIB_PATH, "r", encoding="utf-8") as f:
            c = json.load(f)
    except Exception as e:
        print("[config] calib.json 读取失败，忽略: %r" % (e,))
        return

    m = c.get("matrix")
    if m and len(m) == 2 and all(len(r) == 2 for r in m):
        MM_PER_PX_MATRIX = [[float(r[0]), float(r[1])] for r in m]
        print("[config] 已加载标定矩阵: %s" % (MM_PER_PX_MATRIX,))
    off = c.get("offset_mm")
    if off and len(off) == 2:
        CAM_TO_GRIPPER_MM_X = float(off[0])
        CAM_TO_GRIPPER_MM_Y = float(off[1])

    # --- 关键校验：几何一变，gate 数据全部作废 ---
    h = c.get("cam_height_mm")
    p = c.get("cam_pitch_deg")
    mismatch = []
    if h is not None and CAM_HEIGHT_MM is not None and abs(h - CAM_HEIGHT_MM) > 1.0:
        mismatch.append("高度 标定=%s 当前=%s" % (h, CAM_HEIGHT_MM))
    if p is not None and CAM_PITCH_DEG is not None and abs(p - CAM_PITCH_DEG) > 0.5:
        mismatch.append("俯角 标定=%s 当前=%s" % (p, CAM_PITCH_DEG))
    if mismatch:
        print("!" * 60)
        print("[config] 严重：相机几何与标定时不一致 -> " + "; ".join(mismatch))
        print("[config] 标定值和 data/gate 数据均已失效，必须重标 + 重录")
        print("!" * 60)


_load_calib()
