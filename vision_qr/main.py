"""
K210 扫码节点 —— 在你的 saoma.py 基础上加了校验和多帧确认

【与 saoma.py 的差别，只有三处】
  1) 锁定前做语义校验：格式或取值不合法就不锁定、不发送（只在控制台打印）
  2) 锁定前要求连续 CONFIRM_FRAMES 帧解出同一结果
  3) 帧尺寸改为可配置（默认沿用你的 QVGA）

【没有改的】
  - 串口协议仍是 $<payload># 和 $ACK#
  - 引脚仍是 TX=8 / RX=6
  - 重发间隔 200ms、ACK 超时 3000ms
  所以主控那边一行都不用动。

【为什么加语义校验】
  find_qrcodes 在图像模糊时会"解出"一个错误的串。你的 saoma.py 单帧锁定，
  一帧误读就会被重发 3 秒；如果主控 ACK 了，整场比赛就按错误任务跑。
  规则天然给了约束：颜色只能 1-6、位置只能 1-3，所以 157 这种一眼就能
  判定为误读。把这类结果挡在发送之前，成本几乎为零。

  **注意：校验失败的信息只走 print（USB 控制台），不走串口。**
  不要往主控的协议里塞新消息类型，那会逼主控改代码。

【挡不住的情况】
  "3 被稳定读成 6" 这类错误，语义上完全合法，校验帮不了。
  只能靠提高图像质量（距离、光照、曝光锁定）和多帧确认。
"""

import time

import sensor
import image
import lcd
from fpioa_manager import fm
from machine import UART

import config as C
from qr_validate import classify


# ==================================================================
# 串口
# ==================================================================
class Link:
    def __init__(self):
        fm.register(C.UART_TX_PIN, fm.fpioa.UART1_TX, force=True)
        fm.register(C.UART_RX_PIN, fm.fpioa.UART1_RX, force=True)
        # 参数顺序沿用你实测可用的 saoma.py：(uart, baud, bits, parity, stop, ...)
        self.uart = UART(UART.UART1, C.UART_BAUD, 8, 0, 0,
                         timeout=1000, read_buf_len=256)
        self._rx = b""

    def send(self, text):
        raw = (C.FRAME_HEAD + text + C.FRAME_TAIL).encode("ascii", "ignore")
        try:
            self.uart.write(raw)
            return True
        except Exception as e:
            print("UART write error:", e)
            return False

    def poll_ack(self):
        """缓冲中出现 $ACK# 则返回 True。"""
        try:
            data = self.uart.read()
        except Exception:
            data = None
        if not data:
            return False
        if isinstance(data, str):
            data = data.encode("latin-1")
        self._rx += data
        if len(self._rx) > 64:
            self._rx = self._rx[-64:]
        return C.ACK_TOKEN in self._rx

    def reset_rx(self):
        self._rx = b""


# ==================================================================
# 相机
# ==================================================================
def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    fs = getattr(sensor, C.FRAMESIZE)   # [VERIFY] 常量名 QVGA/VGA
    sensor.set_framesize(fs)
    sensor.set_vflip(0)
    sensor.run(1)
    sensor.skip_frames(time=C.SKIP_FRAMES_MS)

    if C.LOCK_GAIN:
        sensor.set_auto_gain(False)
    if C.LOCK_WHITEBAL:
        sensor.set_auto_whitebal(False)
    if C.LOCK_EXPOSURE:
        try:
            sensor.set_auto_exposure(False, exposure_us=C.EXPOSURE_US)  # [VERIFY]
        except TypeError:
            sensor.set_auto_exposure(False)


# ==================================================================
# 主循环
# ==================================================================
def main():
    init_sensor()
    if C.SHOW_LCD:
        lcd.init(type=1)
    link = Link()

    locked = None       # 已锁定的任务码
    locked_frame = None # 用于显示
    acked = False
    last_send_ms = 0
    lock_ms = 0

    cand = None         # 多帧确认的候选
    cand_n = 0

    print("QR node ready (confirm=%d frame, %s)" % (C.CONFIRM_FRAMES, C.FRAMESIZE))

    while True:
        img = sensor.snapshot()
        now = time.ticks_ms()

        if locked is None:
            # ---------- 扫描阶段 ----------
            try:
                res = img.find_qrcodes()
            except Exception as e:
                res = []
                print("find_qrcodes err:", e)

            hit = None      # (kind, text, reason)
            for r in res:
                payload = r.payload()
                if not payload:
                    continue
                kind, text, reason = classify(payload)
                if kind == "TASK":
                    hit = (kind, text, reason)
                    break
                elif hit is None:
                    hit = (kind, text, reason)   # 记录第一个不合格的，仅用于打印

            if hit is not None and hit[0] == "TASK":
                code = hit[1]
                if code == cand:
                    cand_n += 1
                else:
                    cand = code
                    cand_n = 1

                if cand_n >= C.CONFIRM_FRAMES:
                    locked = code
                    locked_frame = (C.FRAME_HEAD + code + C.FRAME_TAIL)
                    acked = False
                    lock_ms = now
                    link.reset_rx()
                    link.send(code)
                    last_send_ms = time.ticks_ms()
                    print("LOCK %s (confirmed %d frames)" % (code, cand_n))
            elif hit is not None:
                # 校验没通过：只打印，**不发送**，继续扫
                print("REJECT [%s] %s (%s)" % (hit[0], hit[1], hit[2]))
                cand, cand_n = None, 0
            else:
                cand, cand_n = None, 0

        else:
            # ---------- 已锁定，等待 ACK ----------
            if not acked:
                if link.poll_ack():
                    acked = True
                    print("ACK ok, stop TX")
                elif time.ticks_diff(now, lock_ms) >= C.ACK_WAIT_MS:
                    print("ACK timeout %.0fms, rescan" % C.ACK_WAIT_MS)
                    locked = None
                    locked_frame = None
                    cand, cand_n = None, 0
                    link.reset_rx()
                elif time.ticks_diff(now, last_send_ms) >= C.SEND_EVERY_MS:
                    link.send(locked)
                    last_send_ms = time.ticks_ms()
                    print("TX", locked_frame)

        # ---------- 显示 ----------
        if C.SHOW_LCD:
            if locked_frame is not None:
                img.clear()
                img.draw_string(2, 2, locked_frame, color=(0, 160, 0), scale=2)
                if acked:
                    img.draw_string(2, 40, "ACK", color=(0, 255, 0), scale=2)
                else:
                    img.draw_string(2, 40, "TX...", color=(255, 128, 0), scale=2)
            elif cand is not None:
                img.draw_string(2, 2, "confirm %d/%d" % (cand_n, C.CONFIRM_FRAMES),
                                color=(255, 255, 0), scale=2)
            else:
                img.draw_string(2, 2, "scanning", color=(255, 255, 0), scale=2)
            lcd.display(img)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("stopped")
