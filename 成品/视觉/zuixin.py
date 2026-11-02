from maix import camera, display, image, nn, app, time, touchscreen
from maix.v1.machine import UART
import struct
import math
import numpy as np
import cv2

# ==================== 串口初始化 ====================
uart = UART("/dev/ttyS0", 115200)
time.sleep_ms(100)

# ==================== 模型加载 ====================
model = nn.YOLO26(model="/root/model1/yolo26n.mud", dual_buff=True)

# ==================== 摄像头 & 显示 ====================
CAM_W, CAM_H = 320, 240
MAT_W, MAT_H = 160, 160
cam = camera.Camera(CAM_W, CAM_H)
dis = display.Display()
ts = touchscreen.TouchScreen()
cam.skip_frames(30)

# ==================== YOLO 过滤参数 ====================
MIN_AREA_BALL = 100
CONF_TH = 0.5
IOU_TH = 0.45

# ==================== 物料 HSV 阈值 ====================
MATERIAL_HSV_COLORS = [
     # 索引0  红色  (H 跨越 0，用两段)
     ([(0,   80, 80,  10, 255, 255),    
       (174, 135, 58, 180, 237, 198)],  "红",   1),
     # 索引1  绿色

     ([(41,  117,38,  70, 201, 183)],  "绿",   4),
     # 索引2  蓝色
     ([(81, 94, 10, 129, 255, 132)],  "蓝",   3),

     # 索引3  黑色  (低亮度，色相任意)
     ([(0,    0,  17, 135, 79,  40)],  "黑",   5),
     # 索引4  黄色

     ([(12, 117, 114, 34, 224, 209)],  "黄",   2),
     # 索引5  浅蓝

     ([(72, 71, 63, 180, 163, 209)],  "浅蓝", 6),
 ]

DEBUG_COLOR_BGR = [
    (0,   0,   255),   # 0 红
    (0,   255, 0),     # 1 绿
    (255, 0,   0),     # 2 蓝
    (255, 255, 255),   # 3 黑
    (0,   255, 255),   # 4 黄
    (255, 255, 0),     # 5 浅蓝
]

MATERIAL_PIXELS_MIN = 100
MATERIAL_ROI_Y0 = 15
MATERIAL_ROI_H = 120

# ==================== 圆环检测参数 ====================
RING_DARK_BINARY = [(0, 45, -30, 30, -30, 30)]
WHITE_THRESHOLD = [(100, 255)]
DILATE_SIZE = 4   # 膨胀 4
MEAN_SIZE = 1     # 平滑 1
ERODE_SIZE = 2    # 腐蚀 2
CENTER_USE_BBOX = False
BLOB_AREA_MIN = 100 

# ==================== 模式定义 & 触摸按钮 ====================
MODE_MATERIAL = 1                                   #初始模式
MODE_YOLO = 2
current_mode = MODE_MATERIAL

MODE_BTN = [230, 6, 84, 26]
ts_pressed_already = False
ts_last_x, ts_last_y = 0, 0

# ==================== 串口指令 ====================
CMD_MODE_MATERIAL = 0x01
CMD_MODE_YOLO     = 0x02
CMD_COLOR_MAP = {0x03: 0, 0x04: 1, 0x05: 2, 0x06: 3, 0x07: 4, 0x08: 5}
material_color_idx = -1

# ==================== 串口上报 ====================
def send_ring(x, y, digit):
    frame = struct.pack(">BBBHHBBB", 0xb3, 0xb3, 0xb2, x, y, digit, 0, 0x5b)
    uart.write(frame)

def send_material(x, y, r, color):
    frame = struct.pack(">BBBHHBBB", 0xb3, 0xb3, 0xb1, x, y, r, color, 0x5b)
    print(x, y, r, color)
    uart.write(frame)

def read_cmd():
    data = uart.read()
    if data and len(data) >= 3:
        if data[0] == 0xff and data[2] == 0xfe:
            return data[1]
    return None

def handle_cmd(cmd):
    global current_mode, material_color_idx
    if cmd == CMD_MODE_MATERIAL:
        material_color_idx = -1
        current_mode = MODE_MATERIAL
        print("串口切换 -> 物料(全部颜色)")
    elif cmd == CMD_MODE_YOLO:
        current_mode = MODE_YOLO
        print("串口切换 -> YOLO")
    elif cmd in CMD_COLOR_MAP:
        material_color_idx = CMD_COLOR_MAP[cmd]
        current_mode = MODE_MATERIAL
        name, _ = get_material_info(material_color_idx)
        print("串口切换 -> 物料(只识别 %s)" % name)
    else:
        print("未知串口指令: 0x%02x" % cmd)

# ==================== HSV 色块检测 (物料模式) ====================
class MatBlob:
    def __init__(self, x, y, w, h, cx, cy, area, color_idx):
        self._x, self._y, self._w, self._h = x, y, w, h
        self._cx, self._cy = cx, cy
        self._area = area
        self._color_idx = color_idx
    def x(self): return self._x
    def y(self): return self._y
    def w(self): return self._w
    def h(self): return self._h
    def cx(self): return self._cx
    def cy(self): return self._cy
    def area(self): return self._area
    def pixels(self): return self._area
    def code(self): return self._color_idx
    def rect(self): return (self._x, self._y, self._w, self._h)

def get_material_info(color_idx):
    if 0 <= color_idx < len(MATERIAL_HSV_COLORS):
        _, name, code = MATERIAL_HSV_COLORS[color_idx]
        return name, code
    return "?", 0

def find_material_blobs_hsv(img, color_filter_idx=-1, min_pixels=100):
    h, w = img.height(), img.width()
    arr = np.frombuffer(img.to_bytes(), dtype=np.uint8).reshape(h, w, 3)
    hsv = cv2.cvtColor(arr, cv2.COLOR_RGB2HSV)

    if color_filter_idx >= 0:
        color_list = [(color_filter_idx, MATERIAL_HSV_COLORS[color_filter_idx])]
    else:
        color_list = list(enumerate(MATERIAL_HSV_COLORS))

    results = []
    debug = np.zeros((h, w, 3), dtype=np.uint8)

    for idx, (ranges, name, code) in color_list:
        mask = np.zeros((h, w), dtype=np.uint8)
        for (h_lo, s_lo, v_lo, h_hi, s_hi, v_hi) in ranges:
            lo = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
            hi = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        if 0 <= idx < len(DEBUG_COLOR_BGR):
            debug[mask > 0] = DEBUG_COLOR_BGR[idx]

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_pixels:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            bx, by, bw, bh = cv2.boundingRect(cnt)
            results.append(MatBlob(bx, by, bw, bh, cx, cy, int(area), idx))
            cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (255, 255, 255), 1)

    debug_small = cv2.resize(debug, (80, 60), interpolation=cv2.INTER_NEAREST)
    mask_maix = image.cv2image(debug_small, copy=True)
    return results, mask_maix


# ==================== ★ YOLO 模式下精确找圆环中心 (带调试图) ====================
def detect_ring_in_box(img, box_x, box_y, box_w, box_h):
    pad_x = 5
    pad_y = 5
    x0 = max(0, box_x - pad_x)
    y0 = max(0, box_y - pad_y)
    x1 = min(img.width(), box_x + box_w + pad_x)
    y1 = min(img.height(), box_y + box_h + pad_y)
    roi_w, roi_h = x1 - x0, y1 - y0
    
    # 画出用于圆环检测的 ROI 框 (黄色)
    img.draw_rect(x0, y0, roi_w, roi_h, color=image.COLOR_YELLOW, thickness=1)
    
    if roi_w < 20 or roi_h < 20:
        return None, None
        
    try:
        roi = img.crop(x0, y0, roi_w, roi_h)
    except Exception as e:
        print("crop 失败:", e)
        return None, None

    work = roi.copy()
    
    # 形态学处理
    work.binary(RING_DARK_BINARY)
    work.dilate(DILATE_SIZE)
    work.mean(MEAN_SIZE)
    work.erode(ERODE_SIZE)

    blobs = work.find_blobs(WHITE_THRESHOLD, merge=False, area_threshold=BLOB_AREA_MIN)
    
    if not blobs:
        return None, work

    best = max(blobs, key=lambda b: b.area())
    
    if CENTER_USE_BBOX:
        rc_x = best.x() + best.w() // 2
        rc_y = best.y() + best.h() // 2
    else:
        rc_x, rc_y = best.cx(), best.cy()

    # ★ 在调试图上也画一个红点，方便你确认找的中心对不对
    work.draw_cross(rc_x, rc_y, color=image.COLOR_RED, size=5, thickness=1)

    return (x0 + rc_x, y0 + rc_y), work


# ==================== 触摸交互 ====================
def draw_mode_button(img):
    label = "-> YOLO" if current_mode == MODE_MATERIAL else "-> thing"
    img.draw_rect(MODE_BTN[0], MODE_BTN[1], MODE_BTN[2], MODE_BTN[3], image.COLOR_YELLOW, 2)
    img.draw_string(MODE_BTN[0] + 8, MODE_BTN[1] + 8, label, image.COLOR_YELLOW, scale=1)

def poll_touch():
    global ts_pressed_already, ts_last_x, ts_last_y, current_mode, material_color_idx
    x, y, pressed = ts.read()
    x, y = image.resize_map_pos(dis.width(), dis.height(), CAM_W, CAM_H, image.Fit.FIT_CONTAIN, x, y)
    if pressed:
        ts_pressed_already = True
        ts_last_x, ts_last_y = x, y
    elif ts_pressed_already:
        ts_pressed_already = False
        mb = MODE_BTN
        if (mb[0] <= ts_last_x < mb[0] + mb[2] and mb[1] <= ts_last_y < mb[1] + mb[3]):
            current_mode = (MODE_YOLO if current_mode == MODE_MATERIAL else MODE_MATERIAL)
            material_color_idx = -1
            print("触摸切换 ->", "thing" if current_mode == MODE_MATERIAL else "YOLO")


# ==================== 主循环 ====================
while not app.need_exit():
    t_start = time.ticks_ms()
    img_big = cam.read()

    poll_touch()

    cmd = read_cmd()
    if cmd is not None:
        handle_cmd(cmd)

    # ============================================================
    # YOLO 模式
    # ============================================================
    if current_mode == MODE_YOLO:
        img = img_big
        objs = model.detect(img, conf_th=CONF_TH, iou_th=IOU_TH)
        filtered_objs = [obj for obj in objs if (obj.w * obj.h) >= MIN_AREA_BALL]

        if filtered_objs:
            best_obj = max(filtered_objs, key=lambda obj: obj.score)
            x = int(best_obj.x)
            y = int(best_obj.y)
            w = int(best_obj.w)
            h = int(best_obj.h)
            
            # 1. 计算并画出 YOLO 框的中心（蓝色十字）
            yolo_cx = x + w // 2
            yolo_cy = y + h // 2
            img.draw_cross(yolo_cx, yolo_cy, color=image.Color.from_rgb(0, 0, 255), size=8, thickness=2)
            img.draw_string(yolo_cx + 10, yolo_cy - 10, "YOLO", color=image.Color.from_rgb(0, 0, 255), scale=0.8)

            # 2. 形态学找中心
            ring_result = detect_ring_in_box(img, x, y, w, h)
            
            final_cx, final_cy = yolo_cx, yolo_cy
            ring_used = False
            work_img = None
            
            if ring_result:
                ring, work_img = ring_result
                if ring is not None:
                    final_cx, final_cy = ring
                    ring_used = True
                    
                    # ★ 画出形态学找出的中心（红色十字）
                    img.draw_cross(final_cx, final_cy, color=image.COLOR_RED, size=10, thickness=2)
                    img.draw_string(final_cx + 10, final_cy - 10, "Morph", color=image.COLOR_RED, scale=0.8)

            img.draw_string(x, max(0, y - 10), "%.2f" % best_obj.score, image.COLOR_RED, scale=1)

            # ★ 把形态学处理后的二值图缩小贴到右下角
            if work_img is not None:
                debug_img = work_img.resize(80, 60)
                img.draw_image(img.width() - 82, img.height() - 62, debug_img)
                img.draw_rect(img.width() - 82, img.height() - 62, 82, 62, image.COLOR_WHITE, thickness=1)

            # 发送最终结果（如果找到了形态学中心就发形态学中心，否则发 YOLO 中心）
            send_ring(final_cx, final_cy, 2)

        show_img = img

    # ============================================================
    # 物料模式: HSV 阈值检测 + 调试图
    # ============================================================
    else:
        img = img_big.resize(MAT_W, MAT_H)

        # ★ 根据当前模式决定 ROI 裁剪
        if material_color_idx >= 0:
            # 单色模式：跳过上方（y < MATERIAL_ROI_Y0），识别下方所有区域
            roi = img.crop(0, MATERIAL_ROI_Y0, MAT_W, MAT_H - MATERIAL_ROI_Y0)
            roi_offset_y = MATERIAL_ROI_Y0
        else:
            # 全部颜色模式：只识别中间指定区域
            roi = img.crop(0, MATERIAL_ROI_Y0, MAT_W, MATERIAL_ROI_H - MATERIAL_ROI_Y0)
            roi_offset_y = MATERIAL_ROI_Y0

        materials, mask_debug = find_material_blobs_hsv(
            roi,
            color_filter_idx=material_color_idx,
            min_pixels=MATERIAL_PIXELS_MIN)

        # 画识别范围分界线
        # 上边界始终画 (两种模式上面都不识别)
        img.draw_line(0, MATERIAL_ROI_Y0, MAT_W, MATERIAL_ROI_Y0, image.COLOR_RED, 1)
        
        # 只有全部颜色模式才有下边界 (单色模式识别到底)
        if material_color_idx < 0:
            img.draw_line(0, MATERIAL_ROI_H, MAT_W, MATERIAL_ROI_H, image.COLOR_RED, 1)

        if materials:
            max_blob = max(materials, key=lambda b: b.area())
            mx = max_blob.cx()
            my = max_blob.cy() + roi_offset_y
            mr = min(max_blob.w(), max_blob.h()) // 2

            color_idx = max_blob.code()
            color_name, color_code = get_material_info(color_idx)

            rect = max_blob.rect()
            img.draw_rect(rect[0], rect[1] + roi_offset_y, rect[2], rect[3], color=image.COLOR_GREEN, thickness=1)
            img.draw_cross(mx, my, color=image.COLOR_GREEN, size=5, thickness=1)
            img.draw_string(0, 12, "mat:(%d,%d) r:%d %s" % (mx, my, mr, color_name), image.COLOR_GREEN, scale=0.8)

            send_material(mx, my, mr, color_code)

        img.draw_image(img.width() - 82, img.height() - 62, mask_debug)
        img.draw_rect(img.width() - 82, img.height() - 62, 82, 62, image.COLOR_WHITE, thickness=1)

        # 屏幕显示当前模式
        mode_str = "ONE (Down)" if material_color_idx >= 0 else "ALL (Mid)"
        img.draw_string(0, 22, mode_str, image.COLOR_YELLOW, scale=0.8)

        show_img = img.resize(CAM_W, CAM_H)

    # ---------- 按钮 + FPS ----------
    draw_mode_button(show_img)

    t_end = time.ticks_ms()
    fps = 1000 / max(1, t_end - t_start)
    show_img.draw_string(0, 0, f"FPS:{fps:.1f}", image.COLOR_RED, scale=1)

    dis.show(show_img)