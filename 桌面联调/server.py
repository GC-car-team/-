# -*- coding: utf-8 -*-
"""蓝牙无线联调：PC ↔ HC-05 ↔ 主板 TUNE；可选相机 WiFi 看帧。"""
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

app = FastAPI(title="bt-car-bench")
APP_VER = "2026-11-04-bt3"

LOG = deque(maxlen=80)
_lock = threading.Lock()
_bt_tx = threading.Lock()

bt_ser: Optional[serial.Serial] = None
bt_thread: Optional[threading.Thread] = None
bt_stop = threading.Event()

vision_link: Any = None
vision_thread: Optional[threading.Thread] = None
vision_stop = threading.Event()
vision_mode = "none"

OPEN_P = 2000
CLOSE_P = 600

state = {
    "bt": False,
    "fault": "",
    "vision": None,  # WiFi 相机帧
    "vision_mode": "none",
    "mcu_vis": None,  # 主板 v 回包解析
    "coop": {
        "on": False,
        "auto_close": False,
        "closed": False,
        "cmd": 3,
        "kind": 1,
        "id": 1,
    },
    "last_tx": "",
    "last_rx": "",
    "heartbeat": time.time(),
}

coop_thread: Optional[threading.Thread] = None
coop_stop = threading.Event()


def _parse_mcu_vis(line: str) -> None:
    """解析 VIS,on=...arr=...dx=... 填入 state['mcu_vis']。"""
    if not line.startswith("VIS,"):
        return
    parts = {}
    for tok in line.split(","):
        if "=" in tok:
            k, v = tok.split("=", 1)
            parts[k.strip()] = v.strip()
    try:
        info = {
            "on": int(parts.get("on", "0")),
            "bytes": int(parts.get("bytes", "0")),
            "frames": int(parts.get("frames", "0")),
            "arr": int(parts.get("arr", "0")),
            "kind": int(parts.get("kind", "0")),
            "fol": int(parts.get("fol", "0")),
            "dx": int(parts.get("dx", "0")),
            "dy": int(parts.get("dy", "0")),
            "t": time.time(),
        }
    except ValueError:
        return
    state["mcu_vis"] = info
    # 配合测试：到位且开启自动合爪 → 只合一次
    coop = state["coop"]
    if (
        coop.get("on")
        and coop.get("auto_close")
        and not coop.get("closed")
        and info["arr"] == 1
        and bt_ser is not None
        and bt_ser.is_open
    ):
        try:
            bt_send("va")
            coop["closed"] = True
            log("COOP auto va (arr=1)")
        except Exception as e:
            log(f"COOP auto va fail {e}")


def coop_loop() -> None:
    """配合测试时周期性问主板 v。"""
    while not coop_stop.is_set():
        if state["coop"].get("on") and bt_ser is not None and bt_ser.is_open:
            try:
                bt_send("v", quiet=True)
            except Exception:
                pass
            time.sleep(0.55)
        else:
            time.sleep(0.15)


def _ensure_coop_thread() -> None:
    global coop_thread
    if coop_thread and coop_thread.is_alive():
        return
    coop_stop.clear()
    coop_thread = threading.Thread(target=coop_loop, daemon=True)
    coop_thread.start()


def log(msg: str) -> None:
    with _lock:
        LOG.appendleft({"t": time.strftime("%H:%M:%S"), "m": msg})


# ---------- 蓝牙收发 ----------
def bt_loop() -> None:
    buf = bytearray()
    while not bt_stop.is_set():
        ser = bt_ser
        if ser is None or not ser.is_open:
            time.sleep(0.05)
            continue
        try:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                buf.extend(chunk)
                while True:
                    i_n = buf.find(b"\n")
                    i_r = buf.find(b"\r")
                    if i_n < 0 and i_r < 0:
                        break
                    if i_n < 0:
                        i = i_r
                    elif i_r < 0:
                        i = i_n
                    else:
                        i = min(i_n, i_r)
                    line = bytes(buf[:i]).decode("utf-8", errors="replace").strip()
                    del buf[: i + 1]
                    if not line:
                        continue
                    state["last_rx"] = line
                    log("RX " + line)
                    _parse_mcu_vis(line)
                    if line.startswith("OK,va") or line.startswith("WAIT,va"):
                        if line.startswith("OK,va"):
                            state["coop"]["closed"] = True
            else:
                time.sleep(0.02)
            if len(buf) > 4096:
                del buf[:-256]
        except Exception as e:
            log(f"BT ERR {e}")
            state["fault"] = f"bt:{e}"
            time.sleep(0.2)


def bt_send(line: str, quiet: bool = False) -> None:
    global bt_ser
    if bt_ser is None or not bt_ser.is_open:
        raise HTTPException(400, "蓝牙未连接（先选 HC-05 的 COM）")
    text = line.strip()
    if not text:
        raise HTTPException(400, "空命令")
    data = (text + "\n").encode("ascii", errors="ignore")
    with _bt_tx:
        bt_ser.write(data)
        bt_ser.flush()
    state["last_tx"] = text
    if not quiet:
        log("TX " + text)


# ---------- 视觉 WiFi（监视用）----------
class TcpVisionLink:
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
        self.in_waiting
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


def parse_vision(buf: bytearray) -> None:
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
        if buf[1] != 0xB3 or buf[2] not in (0xB1, 0xB2):
            del buf[0]
            continue
        if buf[9] != 0x5B:
            del buf[0]
            continue
        frame = bytes(buf[:10])
        del buf[:10]
        x = (frame[3] << 8) | frame[4]
        y = (frame[5] << 8) | frame[6]
        if frame[2] == 0xB1:
            state["vision"] = {"kind": "mat", "x": x, "y": y, "r": frame[7], "color": frame[8]}
            log(f"CAM mat ({x},{y}) c={frame[8]}")
        else:
            state["vision"] = {"kind": "ring", "x": x, "y": y, "digit": frame[7]}
            log(f"CAM ring ({x},{y}) d={frame[7]}")


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
            time.sleep(0.2)


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


def _connect_vision_wifi(host: str, port: int) -> None:
    global vision_link, vision_thread, vision_mode
    host, port = _parse_host_port(host, port)
    try:
        sock = socket.create_connection((host, int(port)), timeout=5.0)
    except OSError as e:
        raise RuntimeError(f"相机 TCP 连不上 {host}:{port} ({e})") from e
    vision_link = TcpVisionLink(sock, f"{host}:{port}")
    vision_mode = "wifi"
    vision_stop.clear()
    vision_thread = threading.Thread(target=vision_loop, daemon=True)
    vision_thread.start()
    log(f"视觉 WiFi {host}:{port}")


# ---------- API models ----------
class ConnectBody(BaseModel):
    bt_port: str = ""
    baud: int = 9600
    vision_host: str = ""
    vision_tcp_port: int = 5555


class SendBody(BaseModel):
    line: str


class GoBody(BaseModel):
    mm: float = Field(500.0, ge=-3000, le=3000)


class TurnBody(BaseModel):
    deg: float = Field(90.0, ge=-360, le=360)


class PosBody(BaseModel):
    pos: int = Field(1500, ge=500, le=2500)


class VisionCmdBody(BaseModel):
    cmd: int = Field(..., ge=0, le=255)
    via: str = "bt"  # bt | wifi


class CalibBody(BaseModel):
    open_p: Optional[int] = None
    close_p: Optional[int] = None


class ExpectBody(BaseModel):
    kind: int = Field(0, ge=0, le=2)
    id: int = Field(0, ge=0, le=6)


# ---------- routes ----------
@app.get("/", response_class=HTMLResponse)
def index():
    with open(__file__.replace("server.py", "index.html"), "r", encoding="utf-8") as f:
        return f.read()


@app.get("/api/health")
def health():
    return {
        "ok": True,
        "ver": APP_VER,
        "mode": "bluetooth-tune",
        "default_baud": 9600,
    }


def _bt_name_by_mac() -> dict[str, str]:
    """从即插即用里挖「HC-05」等蓝牙名，按 MAC 映射到串口选择提示。"""
    out: dict[str, str] = {}
    try:
        import subprocess

        # FriendlyName + InstanceId，不引入额外依赖
        ps = (
            "Get-PnpDevice -PresentOnly | "
            "Where-Object { $_.InstanceId -like 'BTHENUM\\DEV_*' } | "
            "ForEach-Object { $_.FriendlyName + '|' + $_.InstanceId }"
        )
        r = subprocess.run(
            ["powershell", "-NoProfile", "-Command", ps],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=8,
        )
        for line in (r.stdout or "").splitlines():
            if "|" not in line:
                continue
            name, iid = line.split("|", 1)
            # ...\DEV_002504015B17\...
            mac = ""
            for part in iid.replace("/", "\\").split("\\"):
                if part.startswith("DEV_") and len(part) >= 4 + 12:
                    mac = part[4:16].upper()
                    break
            if mac:
                out[mac] = name.strip()
    except Exception:
        pass
    return out


@app.get("/api/ports")
def ports():
    names = _bt_name_by_mac()
    rows = []
    for p in serial.tools.list_ports.comports():
        desc = p.description or ""
        hwid = (p.hwid or "").upper()
        tag = ""
        # 蓝牙串口 hwid 里带 12 位 MAC
        for mac, name in names.items():
            if mac in hwid.replace(":", ""):
                tag = name
                break
        label = desc
        if tag:
            label = f"{tag} · {desc}"
        elif "BTHENUM" in hwid and "000000000000" in hwid:
            label = f"(空蓝牙口) {desc}"
        rows.append({"device": p.device, "desc": label, "hwid": p.hwid, "bt_name": tag})
    # HC-05 排前面
    rows.sort(key=lambda r: (0 if (r.get("bt_name") or "").upper().startswith("HC") else 1, r["device"]))
    return rows


@app.post("/api/probe_vision")
def probe_vision(body: ConnectBody):
    if not (body.vision_host or "").strip():
        raise HTTPException(400, "请填相机 IP")
    h, p = _parse_host_port(body.vision_host, int(body.vision_tcp_port or 5555))
    try:
        s = socket.create_connection((h, p), timeout=5.0)
        s.close()
        log(f"PROBE OK {h}:{p}")
        return {"ok": True, "host": h, "port": p}
    except OSError as e:
        raise HTTPException(500, f"探测失败 {h}:{p} — {e}")


@app.post("/api/connect")
def connect(body: ConnectBody):
    global bt_ser, bt_thread, vision_mode
    disconnect()
    vision_err = ""
    if not (body.bt_port or "").strip():
        raise HTTPException(400, "必须选择蓝牙 COM（电脑已配对的 HC-05）")
    try:
        baud = int(body.baud or 9600)
        bt_ser = serial.Serial(body.bt_port.strip(), baud, timeout=0.05, write_timeout=1.0)
        bt_stop.clear()
        bt_thread = threading.Thread(target=bt_loop, daemon=True)
        bt_thread.start()
        state["bt"] = True
        log(f"蓝牙 {body.bt_port} @ {baud}")
        _ensure_coop_thread()

        host = (body.vision_host or "").strip()
        if host:
            try:
                _connect_vision_wifi(host, int(body.vision_tcp_port or 5555))
            except Exception as e:
                vision_err = str(e)
                vision_mode = "none"
                log(f"视觉失败: {vision_err}")
        else:
            vision_mode = "none"

        state["fault"] = vision_err
        state["vision_mode"] = vision_mode
        return {
            "ok": True,
            "bt": True,
            "baud": baud,
            "vision_mode": vision_mode,
            "ok_vision": vision_mode == "wifi",
            "vision_error": vision_err,
            "ver": APP_VER,
        }
    except Exception as e:
        disconnect()
        state["fault"] = str(e)
        log(f"CONNECT FAIL {e}")
        raise HTTPException(500, str(e))


@app.post("/api/disconnect")
def disconnect():
    global bt_ser, bt_thread, vision_link, vision_thread, vision_mode
    state["coop"]["on"] = False
    bt_stop.set()
    vision_stop.set()
    if bt_thread and bt_thread.is_alive():
        bt_thread.join(timeout=1.0)
    if vision_thread and vision_thread.is_alive():
        vision_thread.join(timeout=1.0)
    bt_thread = None
    vision_thread = None
    for s in (bt_ser, vision_link):
        try:
            if s is not None and getattr(s, "is_open", False):
                s.close()
        except Exception:
            pass
    bt_ser = None
    vision_link = None
    vision_mode = "none"
    state["bt"] = False
    state["vision_mode"] = "none"
    state["mcu_vis"] = None
    log("断开")
    return {"ok": True}


class CoopBody(BaseModel):
    cmd: int = Field(3, ge=1, le=8)  # FF 模式：3红 / 2环…
    kind: int = Field(1, ge=1, le=2)  # 1 物料 2 环
    id: int = Field(1, ge=1, le=6)
    auto_close: bool = False
    follow: bool = True


@app.post("/api/coop/start")
def coop_start(body: CoopBody):
    """张开 → 切视觉模式 → 过滤 → 跟瞄 → 轮询 v。"""
    if bt_ser is None or not bt_ser.is_open:
        raise HTTPException(400, "请先连接蓝牙")
    _ensure_coop_thread()
    state["coop"] = {
        "on": True,
        "auto_close": bool(body.auto_close),
        "closed": False,
        "cmd": int(body.cmd),
        "kind": int(body.kind),
        "id": int(body.id),
    }
    state["mcu_vis"] = None
    bt_send("O")
    time.sleep(0.05)
    bt_send(f"vc{int(body.cmd):02X}")
    time.sleep(0.05)
    bt_send(f"ve{int(body.kind)},{int(body.id)}")
    time.sleep(0.05)
    if body.follow:
        bt_send("vf1")
    else:
        bt_send("vf0")
    log(
        f"COOP start cmd={body.cmd:02X} ve={body.kind},{body.id} "
        f"follow={int(body.follow)} auto_close={int(body.auto_close)}"
    )
    return {"ok": True, "coop": state["coop"]}


@app.post("/api/coop/stop")
def coop_stop_api():
    state["coop"]["on"] = False
    try:
        if bt_ser is not None and bt_ser.is_open:
            bt_send("vf0")
    except Exception:
        pass
    log("COOP stop")
    return {"ok": True}


@app.post("/api/send")
def api_send(body: SendBody):
    bt_send(body.line)
    return {"ok": True}


@app.post("/api/chassis/go")
def chassis_go(body: GoBody):
    mm = int(round(body.mm))
    bt_send(f"G{mm}")
    return {"ok": True, "cmd": f"G{mm}"}


@app.post("/api/chassis/turn")
def chassis_turn(body: TurnBody):
    deg = int(round(body.deg))
    bt_send(f"T{deg}")
    return {"ok": True, "cmd": f"T{deg}"}


@app.post("/api/chassis/stop")
def chassis_stop():
    bt_send("X")
    return {"ok": True}


@app.post("/api/chassis/zero")
def chassis_zero():
    bt_send("Z")
    return {"ok": True}


@app.post("/api/open")
def api_open():
    bt_send("O")
    return {"ok": True, "pos": OPEN_P}


@app.post("/api/close")
def api_close():
    bt_send("C")
    return {"ok": True, "pos": CLOSE_P}


@app.post("/api/goto")
def api_goto(body: PosBody):
    bt_send(f"U{int(body.pos)}")
    return {"ok": True, "pos": body.pos}


@app.post("/api/arrive_close")
def arrive_close():
    """到位合爪（主板 va）。"""
    bt_send("va")
    return {"ok": True}


@app.post("/api/follow")
def follow(on: int = 1):
    bt_send("vf1" if on else "vf0")
    return {"ok": True, "follow": bool(on)}


@app.post("/api/expect")
def expect(body: ExpectBody):
    if body.kind == 0:
        bt_send("ve0")
    else:
        bt_send(f"ve{int(body.kind)},{int(body.id)}")
    return {"ok": True}


@app.post("/api/vision_cmd")
def vision_cmd(body: VisionCmdBody):
    """via=bt → 主板 vcXX；via=wifi → 直发 FF cmd FE 到联调相机。"""
    if body.via == "wifi":
        ser = vision_link
        if ser is None or not getattr(ser, "is_open", False):
            raise HTTPException(400, "相机 WiFi 未连接")
        frame = bytes((0xFF, body.cmd & 0xFF, 0xFE))
        ser.write(frame)
        ser.flush()
        log(f"TX CAM FF {body.cmd:02X} FE")
        return {"ok": True, "via": "wifi"}
    bt_send(f"vc{body.cmd:02X}")
    return {"ok": True, "via": "bt"}


@app.post("/api/calib")
def calib(body: CalibBody):
    global OPEN_P, CLOSE_P
    if body.open_p is not None:
        OPEN_P = max(500, min(2500, int(body.open_p)))
    if body.close_p is not None:
        CLOSE_P = max(500, min(2500, int(body.close_p)))
    log(f"CALIB open={OPEN_P} close={CLOSE_P}（网页记忆；写入主板用 U/配置）")
    return {"ok": True, "open_p": OPEN_P, "close_p": CLOSE_P}


@app.get("/api/status")
def status():
    state["heartbeat"] = time.time()
    state["bt"] = bool(bt_ser and bt_ser.is_open)
    with _lock:
        logs = list(LOG)
    return JSONResponse(
        {
            "ver": APP_VER,
            "bt": state["bt"],
            "fault": state["fault"],
            "vision": state["vision"],
            "mcu_vis": state.get("mcu_vis"),
            "coop": state.get("coop"),
            "vision_connected": bool(vision_link and getattr(vision_link, "is_open", False)),
            "vision_mode": state.get("vision_mode", vision_mode),
            "last_tx": state["last_tx"],
            "last_rx": state["last_rx"],
            "open_p": OPEN_P,
            "close_p": CLOSE_P,
            "log": logs,
        }
    )


if __name__ == "__main__":
    import uvicorn

    port = 8766
    print("=" * 60)
    print("蓝牙联调 server", APP_VER, "(含相机+舵机配合)")
    print("请打开:  http://127.0.0.1:%d/" % port)
    print("查版本:  http://127.0.0.1:%d/api/health" % port)
    print("默认波特率 9600（与主板 BT_BAUD 一致）")
    print("=" * 60)
    uvicorn.run(app, host="127.0.0.1", port=port)
