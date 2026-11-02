# -*- coding: utf-8 -*-
"""桌面联调：双串口隔离 + 非阻塞 + 状态机，避免单侧故障拖死整页。"""
from __future__ import annotations

import socket
import threading
import time
from collections import deque
from typing import Any, Optional

import serial
import serial.tools.list_ports
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel, Field

app = FastAPI(title="cam-servo-bench")

LOG = deque(maxlen=50)
_lock = threading.Lock()

servo_ser: Optional[serial.Serial] = None
vision_link: Any = None  # Serial 或 TcpVisionLink
vision_thread: Optional[threading.Thread] = None
vision_stop = threading.Event()
vision_mode = "none"  # none|serial|wifi

state = {
    "servo": "idle",  # idle|moving|fault
    "servo_until": 0.0,
    "servo_pos": None,
    "vision": None,  # last frame dict
    "fault": "",
    "p3_auto": False,
    "heartbeat": time.time(),
    "vision_mode": "none",
}


class TcpVisionLink:
    """把 socket 伪装成带 read/write/in_waiting 的串口。"""

    def __init__(self, sock: socket.socket, peer: str):
        self._s = sock
        self._s.settimeout(0.05)
        self.is_open = True
        self.peer = peer
        self._buf = bytearray()

    @property
    def in_waiting(self) -> int:
        try:
            chunk = self._s.recv(4096)
            if chunk:
                self._buf.extend(chunk)
            elif chunk == b"":
                self.is_open = False
        except socket.timeout:
            pass
        except OSError:
            self.is_open = False
        return len(self._buf)

    def read(self, n: int = 1) -> bytes:
        self.in_waiting  # 先抽一下
        if n <= 0:
            n = len(self._buf)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def write(self, data: bytes) -> int:
        self._s.sendall(data)
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False
        try:
            self._s.close()
        except OSError:
            pass

OPEN_P = 2000
CLOSE_P = 600
MOVE_MS = 1000
SETTLE_EXTRA = 200


def log(msg: str) -> None:
    with _lock:
        LOG.appendleft({"t": time.strftime("%H:%M:%S"), "m": msg})


def servo_write(cmd: str) -> None:
    global servo_ser
    if servo_ser is None or not servo_ser.is_open:
        raise HTTPException(400, "舵机口未连接")
    data = cmd.encode("ascii")
    servo_ser.write(data)
    servo_ser.flush()
    log("TX " + cmd)


def bus_goto(pos: int, t_ms: int = MOVE_MS) -> None:
    pos = max(500, min(2500, int(pos)))
    t_ms = max(100, min(9999, int(t_ms)))
    cmd = f"#000P{pos:04d}T{t_ms:04d}!"
    servo_write(cmd)
    state["servo"] = "moving"
    state["servo_pos"] = pos
    state["servo_until"] = time.time() + (t_ms + SETTLE_EXTRA) / 1000.0
    state["fault"] = ""


def tick_servo() -> None:
    if state["servo"] == "moving" and time.time() >= state["servo_until"]:
        state["servo"] = "idle"


def parse_vision(buf: bytearray) -> None:
    """从缓冲抠 B3 B3 帧。"""
    while True:
        if len(buf) < 10:
            return
        try:
            i = buf.index(0xB3)
        except ValueError:
            buf.clear()
            return
        if i > 0:
            del buf[:i]
        if len(buf) < 10:
            return
        if buf[1] != 0xB3:
            del buf[0]
            continue
        if buf[2] not in (0xB1, 0xB2):
            del buf[0]
            continue
        if len(buf) < 10:
            return
        if buf[9] != 0x5B:
            del buf[0]
            continue
        frame = bytes(buf[:10])
        del buf[:10]
        x = (frame[3] << 8) | frame[4]
        y = (frame[5] << 8) | frame[6]
        if frame[2] == 0xB1:
            r, color = frame[7], frame[8]
            state["vision"] = {"kind": "mat", "x": x, "y": y, "r": r, "color": color}
            log(f"RX mat ({x},{y}) c={color}")
        else:
            digit = frame[7]
            state["vision"] = {"kind": "ring", "x": x, "y": y, "digit": digit}
            log(f"RX ring ({x},{y}) d={digit}")
            if state.get("p3_auto") and state["servo"] == "idle":
                # P3：仅示例——检测到环后不自动合，需显式开；此处留扩展口
                pass


def vision_loop() -> None:
    buf = bytearray()
    while not vision_stop.is_set():
        ser = vision_link
        if ser is None or not getattr(ser, "is_open", False):
            time.sleep(0.05)
            continue
        try:
            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))
                if len(buf) > 4096:
                    del buf[:-512]
                parse_vision(buf)
            else:
                time.sleep(0.02)
        except Exception as e:
            log(f"VISION ERR {e}")
            state["fault"] = f"vision:{e}"
            time.sleep(0.2)


class ConnectBody(BaseModel):
    servo_port: str = ""
    vision_port: str = ""
    baud: int = 115200
    # WiFi：填相机 IP（与电脑同一局域网）；优先于 vision_port
    vision_host: str = ""
    vision_tcp_port: int = 5555


class PosBody(BaseModel):
    pos: int = Field(1500, ge=500, le=2500)
    ms: int = Field(MOVE_MS, ge=100, le=9999)


APP_VER = "2026-11-01-wifi2"


@app.get("/", response_class=HTMLResponse)
def index():
    with open(__file__.replace("server.py", "index.html"), "r", encoding="utf-8") as f:
        return f.read()


@app.get("/api/health")
def health():
    """用来确认当前跑的是不是最新 server（浏览器打开 /api/health）。"""
    return {
        "ok": True,
        "ver": APP_VER,
        "routes": [
            "/api/health",
            "/api/ports",
            "/api/probe_vision",
            "/api/connect",
            "/api/status",
        ],
    }


@app.get("/api/ports")
def ports():
    return [{"device": p.device, "desc": p.description} for p in serial.tools.list_ports.comports()]


def _connect_vision_wifi(host: str, port: int) -> None:
    global vision_link, vision_thread, vision_mode
    host, port = _parse_host_port(host, port)
    try:
        sock = socket.create_connection((host, int(port)), timeout=5.0)
    except OSError as e:
        raise RuntimeError(
            f"相机 TCP 连不上 {host}:{port} ({e})。"
            "请确认：①相机已运行 maixcam_wifi_bench.py；②屏上 TCP 为 listen/online；"
            "③填的是 WiFi IP（Settings 里那个）；④电脑与相机同一网段；"
            "⑤顺序是先开相机再点网页连接。"
        ) from e
    vision_link = TcpVisionLink(sock, f"{host}:{port}")
    vision_mode = "wifi"
    vision_stop.clear()
    vision_thread = threading.Thread(target=vision_loop, daemon=True)
    vision_thread.start()
    log(f"视觉 WiFi {host}:{port}")


def _parse_host_port(host: str, port: int) -> tuple[str, int]:
    h = (host or "").strip()
    p = int(port or 5555)
    if h.lower().startswith("http://"):
        h = h[7:]
    if h.lower().startswith("https://"):
        h = h[8:]
    if "/" in h:
        h = h.split("/")[0]
    if ":" in h and h.count(":") == 1:
        a, b = h.split(":")
        h, p = a, int(b)
    return h, p


def _do_probe(vision_host: str, vision_tcp_port: int = 5555):
    if not (vision_host or "").strip():
        raise HTTPException(400, "请填相机 IP")
    h, p = _parse_host_port(vision_host, int(vision_tcp_port or 5555))
    try:
        s = socket.create_connection((h, p), timeout=5.0)
        s.close()
        log(f"PROBE OK {h}:{p}")
        return {"ok": True, "host": h, "port": p, "ver": APP_VER}
    except OSError as e:
        msg = (
            f"探测失败 {h}:{p} — {e}。"
            "相机需先跑 maixcam_wifi_bench.py，屏显 TCP listen；同一 WiFi；先相机后网页。"
        )
        log(msg)
        raise HTTPException(500, msg)


@app.post("/api/probe_vision")
def probe_vision(body: ConnectBody):
    """只测相机 TCP，不占舵机。"""
    return _do_probe(body.vision_host, int(body.vision_tcp_port or 5555))


@app.get("/api/probe_vision")
def probe_vision_get(host: str = "", port: int = 5555):
    """浏览器地址栏也可测：/api/probe_vision?host=192.168.x.x&port=5555"""
    return _do_probe(host, port)


@app.post("/api/connect")
def connect(body: ConnectBody):
    global servo_ser, vision_link, vision_thread, vision_mode
    disconnect()
    vision_err = ""
    try:
        if body.servo_port:
            servo_ser = serial.Serial(body.servo_port, body.baud, timeout=0.05)
            log(f"舵机口 {body.servo_port}")

        host = (body.vision_host or "").strip()
        if host:
            try:
                _connect_vision_wifi(host, int(body.vision_tcp_port or 5555))
            except Exception as e:
                vision_err = str(e)
                vision_mode = "none"
                log(f"视觉失败: {vision_err}")
        elif body.vision_port:
            vision_link = serial.Serial(body.vision_port, body.baud, timeout=0.05)
            vision_mode = "serial"
            vision_stop.clear()
            vision_thread = threading.Thread(target=vision_loop, daemon=True)
            vision_thread.start()
            log(f"视觉串口 {body.vision_port}")
        else:
            vision_mode = "none"

        state["servo"] = "idle"
        state["fault"] = vision_err
        state["vision_mode"] = vision_mode
        # 填了相机却没连上 → 仍返回 200，但 ok_vision=false，方便网页提示
        return {
            "ok": True,
            "vision_mode": vision_mode,
            "ok_vision": vision_mode in ("wifi", "serial"),
            "vision_error": vision_err,
        }
    except Exception as e:
        disconnect()
        state["fault"] = str(e)
        state["servo"] = "fault"
        log(f"CONNECT FAIL {e}")
        raise HTTPException(500, str(e))


@app.post("/api/disconnect")
def disconnect():
    global servo_ser, vision_link, vision_thread, vision_mode
    vision_stop.set()
    if vision_thread and vision_thread.is_alive():
        vision_thread.join(timeout=1.0)
    vision_thread = None
    for s in (servo_ser, vision_link):
        try:
            if s is not None and getattr(s, "is_open", False):
                s.close()
        except Exception:
            pass
    servo_ser = None
    vision_link = None
    vision_mode = "none"
    state["vision_mode"] = "none"
    log("断开")
    return {"ok": True}


def _reject_if_fault() -> None:
    if state["servo"] == "fault":
        raise HTTPException(400, "处于 fault，先点复位")


@app.post("/api/open")
def api_open():
    _reject_if_fault()
    tick_servo()
    bus_goto(OPEN_P)
    return {"ok": True, "pos": OPEN_P}


@app.post("/api/close")
def api_close():
    _reject_if_fault()
    tick_servo()
    bus_goto(CLOSE_P)
    return {"ok": True, "pos": CLOSE_P}


@app.post("/api/goto")
def api_goto(body: PosBody):
    _reject_if_fault()
    tick_servo()
    bus_goto(body.pos, body.ms)
    return {"ok": True}


@app.post("/api/stop")
def api_stop():
    """急停：标 fault，可选再张开一次。"""
    try:
        if servo_ser and servo_ser.is_open:
            bus_goto(OPEN_P, 500)
    except Exception:
        pass
    state["servo"] = "fault"
    state["fault"] = "STOP"
    log("STOP")
    return {"ok": True}


@app.post("/api/reset")
def api_reset():
    state["servo"] = "idle"
    state["fault"] = ""
    state["vision"] = None
    log("RESET")
    try:
        if servo_ser and servo_ser.is_open:
            bus_goto(OPEN_P)
    except Exception as e:
        state["fault"] = str(e)
        state["servo"] = "fault"
    return {"ok": True}


@app.post("/api/confirm_close")
def confirm_close():
    """P2：人工确认后合爪。"""
    if state["servo"] == "fault":
        raise HTTPException(400, "先复位")
    bus_goto(CLOSE_P)
    return {"ok": True}


class VisionCmdBody(BaseModel):
    cmd: int = Field(..., ge=0, le=255)


@app.post("/api/vision_cmd")
def vision_cmd(body: VisionCmdBody):
    """向视觉（串口或 WiFi）发 FF cmd FE。"""
    ser = vision_link
    if ser is None or not getattr(ser, "is_open", False):
        raise HTTPException(400, "视觉未连接（填相机 IP 或串口）")
    frame = bytes((0xFF, body.cmd & 0xFF, 0xFE))
    try:
        ser.write(frame)
        ser.flush()
    except Exception as e:
        state["fault"] = f"vision_tx:{e}"
        raise HTTPException(500, str(e))
    log(f"TX VIS FF {body.cmd:02X} FE ({vision_mode})")
    return {"ok": True}


class CalibBody(BaseModel):
    open_p: Optional[int] = None
    close_p: Optional[int] = None
    move_ms: Optional[int] = None


@app.post("/api/calib")
def calib(body: CalibBody):
    global OPEN_P, CLOSE_P, MOVE_MS
    if body.open_p is not None:
        OPEN_P = max(500, min(2500, int(body.open_p)))
    if body.close_p is not None:
        CLOSE_P = max(500, min(2500, int(body.close_p)))
    if body.move_ms is not None:
        MOVE_MS = max(100, min(9999, int(body.move_ms)))
    log(f"CALIB open={OPEN_P} close={CLOSE_P} ms={MOVE_MS}")
    return {"ok": True, "open_p": OPEN_P, "close_p": CLOSE_P, "move_ms": MOVE_MS}


@app.get("/api/status")
def status():
    tick_servo()
    state["heartbeat"] = time.time()
    with _lock:
        logs = list(LOG)
    return JSONResponse(
        {
            "servo_state": state["servo"],
            "servo_pos": state["servo_pos"],
            "fault": state["fault"],
            "vision": state["vision"],
            "servo_connected": bool(servo_ser and servo_ser.is_open),
            "vision_connected": bool(vision_link and getattr(vision_link, "is_open", False)),
            "vision_mode": state.get("vision_mode", vision_mode),
            "p3_auto": state["p3_auto"],
            "log": logs,
            "open_p": OPEN_P,
            "close_p": CLOSE_P,
        }
    )


if __name__ == "__main__":
    import uvicorn

    # 固定新端口，避开旧进程占 8765 却没有新接口的情况
    port = 8766
    print("=" * 60)
    print("桌面联调 server", APP_VER)
    print("请打开:  http://127.0.0.1:%d/" % port)
    print("查版本:  http://127.0.0.1:%d/api/health" % port)
    print("=" * 60)
    uvicorn.run(app, host="127.0.0.1", port=port)
