"""
MaixCAM Pro 视觉端主循环 —— 骨架阶段（L0）

本文件只做四件事，不含任何算法与阈值：
  1) 从主控接收 SET:<MODE> 指令
  2) 按模式调用 perceive
  3) 组装契约消息并发送
  4) 记录时序（seq / t / dt）

带 [VERIFY] 注释的行与你板子上的 MaixPy 版本相关，首次运行前逐行核对。
核对方法：注释掉该行看是否报 ImportError/AttributeError，或查官方文档。

验收标准（L0）：
  运行后串口助手里能看到持续输出，形如
    {"cv":"1.0","seq":1,"t":1234,"dt":0,"md":"IDLE","st":2,"n":0,"tg":[]}
"""

import sys
import time

import config as C
import contract as K
import perceive

# ------------------------------------------------------------------
# [VERIFY] MaixPy 模块名与构造签名
# 若你的固件里没有 display，删掉这一行即可（骨架阶段只在调试时用）
# ------------------------------------------------------------------
try:
    from maix import camera, display, image, pinmap, uart  # noqa: F401
    _HAS_MAIX = True
except Exception as e:  # 在 PC 上跑自测时走这里
    _HAS_MAIX = False
    _MAIX_ERR = e


def log(*a):
    print(*a, file=sys.stderr)


# ==================================================================
# 串口：一个很薄的双向包装。失败时降级到 stdout，不崩
# ==================================================================
class Link:
    def __init__(self):
        self.dev = None
        self._buf = b""
        if not _HAS_MAIX:
            log("[Link] 未加载 MaixPy，降级为 stdout")
            return
        if C.UART_DEV is None:
            log("[Link] config.UART_DEV 未填，降级为 stdout")
            return
        try:
            # [VERIFY] 引脚复用函数名
            if C.UART_TX_PIN:
                pinmap.set_pin_function(C.UART_TX_PIN, C.UART_TX_FUNC)
            if C.UART_RX_PIN:
                pinmap.set_pin_function(C.UART_RX_PIN, C.UART_RX_FUNC)
            # [VERIFY] UART 构造参数顺序
            self.dev = uart.UART(C.UART_DEV, C.UART_BAUD)
            log("[Link] UART 已打开 %s @ %d" % (C.UART_DEV, C.UART_BAUD))
        except Exception as e:
            log("[Link] UART 打开失败，降级为 stdout: %r" % (e,))

    def send(self, raw):
        if self.dev is None:
            sys.stdout.write(raw.decode("ascii", "replace"))
            sys.stdout.flush()
            return
        self.dev.write(raw)  # [VERIFY] write 接受 bytes

    def poll_mode(self):
        """非阻塞读主控指令，返回模式字符串或 None。"""
        if self.dev is None:
            return None
        try:
            # [VERIFY] read() 的返回类型与是否阻塞；必要时改成 read(n) / 超时参数
            chunk = self.dev.read()
        except Exception as e:
            log("[Link] read 异常: %r" % (e,))
            return None
        if not chunk:
            return None
        if isinstance(chunk, str):
            chunk = chunk.encode("ascii", "ignore")
        self._buf += chunk
        mode = None
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            m = K.parse_set_command(line.decode("ascii", "ignore"))
            if m is not None:
                mode = m  # 取最后一条，忽略积压
            else:
                log("[Link] 无法解析的指令: %r" % (line,))
        # 缓冲过长说明没收到换行，丢弃防内存涨
        if len(self._buf) > 512:
            log("[Link] 接收缓冲溢出，丢弃")
            self._buf = b""
        return mode


# ==================================================================
# 图像源：fake / cam。两者接口一致，后续加回放也只需加一个分支
# ==================================================================
class FakeSource:
    """不接相机。只为了让串口、编码、主循环能被单独验证。"""

    def __init__(self):
        self.n = 0

    def read(self):
        self.n += 1
        time.sleep(1.0 / max(1, C.CAM_FPS))
        return None  # None = 没有有效帧，主循环据此输出 LOST

    def close(self):
        pass


class CamSource:
    def __init__(self):
        if not _HAS_MAIX:
            raise RuntimeError("无 MaixPy 环境，不能用 SRC='cam'")
        # [VERIFY] Camera 构造签名：(width, height) 或 (width, height, format)
        self.cam = camera.Camera(C.CAM_WIDTH, C.CAM_HEIGHT)

    def read(self):
        return self.cam.read()  # 返回 maix.image.Image

    def close(self):
        try:
            self.cam.close()
        except Exception:
            pass


def make_source():
    if C.SRC == "cam":
        return CamSource()
    if C.SRC == "fake":
        return FakeSource()
    raise ValueError("未知 SRC=%r，只能是 'fake' 或 'cam'" % (C.SRC,))


# ==================================================================
# 主循环
# ==================================================================
def main():
    src = make_source()
    link = Link()

    mode = K.MD_IDLE
    seq = 0
    calib_warned = False
    last_ok_ms = time.monotonic() * 1000.0

    log("[main] 启动：SRC=%s 契约 v%s" % (C.SRC, K.CV))
    log("[main] 契约约定：连续 %dms 无消息，主控必须停止动作" % C.LINK_TIMEOUT_MS)

    while True:
        t0 = time.monotonic()

        # ---- 1. 收指令 ----
        new_mode = link.poll_mode()
        if new_mode is not None and new_mode != mode:
            log("[main] 模式切换 %s -> %s" % (mode, new_mode))
            mode = new_mode

        # ---- 2. 取图 ----
        img = None
        try:
            img = src.read()
        except Exception as e:
            log("[main] 取图异常: %r" % (e,))

        # ---- 3. 感知 ----
        targets = []
        status = K.ST_LOST
        if img is not None and mode != K.MD_IDLE:
            try:
                if mode == K.MD_RING:
                    dets = perceive.detect_ring(img)
                else:
                    dets = perceive.detect_material(img)
                if dets:
                    for d in perceive.to_targets(dets, mode):
                        targets.append(K.make_target(**d))
                    status = K.ST_OK
                else:
                    status = K.ST_LOST  # 看见了但没找到目标
            except perceive.CalibMissing as e:
                # 标定没填：诚实地上报 LOST，绝不编一个坐标
                if not calib_warned:
                    log("[main] 标定缺失，输出 LOST: %s" % (e,))
                    calib_warned = True
                status = K.ST_LOST
            except Exception as e:
                log("[main] 感知异常: %r" % (e,))
                status = K.ST_DEGRADED
        elif img is None and mode != K.MD_IDLE:
            status = K.ST_LOST  # 没有帧

        # ---- 4. 发送 ----
        t_send = time.monotonic()
        seq += 1
        msg = K.make_msg(
            seq=seq,
            t_ms=int(t_send * 1000.0),
            dt_ms=int((t_send - t0) * 1000.0),
            mode=mode,
            status=status,
            targets=targets,
        )
        link.send(K.encode(msg))
        last_ok_ms = t_send * 1000.0

        # ---- 5. 汇总日志 ----
        if seq % C.STATS_EVERY_N == 0:
            dt = (t_send - t0) * 1000.0
            log("[stat] seq=%d md=%s st=%d n=%d loop=%.1fms" %
                (seq, mode, status, len(targets), dt))

        # ---- 6. 让出 CPU ----
        elapsed_ms = (time.monotonic() - t0) * 1000.0
        rest = C.MAIN_LOOP_SLEEP_MS - elapsed_ms
        if rest > 0:
            time.sleep(rest / 1000.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log("[main] 用户中断")
