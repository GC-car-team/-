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
APP_VER = "2026-11-06-bt11"

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

OPEN_P = 2480
CLOSE_P = 1950
P_MIN = 500
P_MAX = 4000

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
    "scan": {"code": "", "ok": False, "t": 0.0},
    "arm": {"pos": None, "vpos_mm": None, "online": None, "t": 0.0},
}

coop_thread: Optional[threading.Thread] = None
coop_stop = threading.Event()


def _parse_scan_line(line: str) -> None:
    """解析 CODE,<task> / CODE,NONE。"""
    raw = line.replace("\x00", "")
    idx = raw.find("CODE,")
    if idx < 0:
        return
    payload = raw[idx + 5 :].strip()
    if "," in payload:
        payload = payload.split(",", 1)[0].strip()
    if (not payload) or payload.upper() == "NONE":
        state["scan"] = {"code": "", "ok": False, "t": time.time()}
        return
    code = payload[:15]
    ok = (
        len(code) == 15
        and code[3] == "+"
        and code[7] == "+"
        and code[11] == "+"
    )
    state["scan"] = {"code": code, "ok": ok, "t": time.time()}


def _parse_arm_line(line: str) -> None:
    """解析 POS,1,... / VPOS,1,<pls>,<mm>mm。"""
    raw = line.replace("\x00", "")
    arm = state.get("arm") or {}
    if raw.startswith("POS,1,"):
        parts = raw.split(",")
        try:
            arm = {
                **arm,
                "pos": int(parts[2]),
                "online": int(parts[5]) if len(parts) > 5 else arm.get("online"),
                "t": time.time(),
            }
        except (ValueError, IndexError):
            return
        state["arm"] = arm
    elif raw.startswith("VPOS,1,"):
        parts = raw.split(",")
        try:
            mm_tok = parts[3] if len(parts) > 3 else ""
            mm = float(mm_tok.replace("mm", "").strip())
            arm = {**arm, "vpos_mm": mm, "t": time.time()}
        except (ValueError, IndexError):
            return
        state["arm"] = arm


def _parse_mcu_vis(line: str) -> None:
    """解析 VIS,on=...arr=...dx=... 填入 state['mcu_vis']。"""
    # 蓝牙上若混入 JustFloat 二进制，行首可能不是 VIS，要从中间切开
    raw = line.replace("\x00", "")
    idx = raw.find("VIS,")
    if idx < 0:
        return
    line = raw[idx:]
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
            "held": 0,
        }
    except ValueError:
        return
    # 无有效目标时 MCU 仍回 dx=0,dy=0：若用它刷新准星会「钉死在中心」或乱跳。
    # 保留上一拍有效偏差，仅更新 on/frames 等状态。
    prev = state.get("mcu_vis")
    if (
        info["on"] == 0
        and info["dx"] == 0
        and info["dy"] == 0
        and isinstance(prev, dict)
        and (prev.get("dx", 0) != 0 or prev.get("dy", 0) != 0 or prev.get("on") == 1)
    ):
        info["dx"] = int(prev.get("dx", 0))
        info["dy"] = int(prev.get("dy", 0))
        info["held"] = 1
        if prev.get("kind"):
            info["kind"] = int(prev.get("kind") or info["kind"])
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
            # 勿在蓝牙收包线程里 sleep/发一长串，改丢给后台，避免整页卡死
            coop["closed"] = True
            threading.Thread(
                target=_coop_auto_close_worker, daemon=True, name="coop-close"
            ).start()
        except Exception as e:
            log(f"COOP auto close fail {e}")


def _coop_auto_close_worker() -> None:
    try:
        servo_close()
        log(f"COOP auto close U{CLOSE_P} (arr=1)")
    except Exception as e:
        log(f"COOP auto close worker fail {e}")


def _coop_start_worker(cmd: int, kind: int, id_: int, follow: bool) -> None:
    """配合启动序列放后台：避免 /api/coop/start 同步 sleep 把网页卡住。"""
    try:
        bt_send("M0")
        time.sleep(0.08)
        servo_open()
        time.sleep(0.25)
        if not state["coop"].get("on"):
            return
        bt_send(f"vc{int(cmd):02X}")
        time.sleep(0.12)
        bt_send(f"ve{int(kind)},{int(id_)}")
        time.sleep(0.12)
        bt_send("vf1" if follow else "vf0")
        log(
            f"COOP ready open={OPEN_P} cmd={cmd:02X} ve={kind},{id_} follow={int(follow)}"
        )
    except Exception as e:
        log(f"COOP start worker fail {e}")
        state["coop"]["on"] = False


def coop_loop() -> None:
    """配合测试时周期性问主板 v（约 2Hz，减轻 9600 阻塞）。"""
    while not coop_stop.is_set():
        if state["coop"].get("on") and bt_ser is not None and bt_ser.is_open:
            try:
                bt_send("v", quiet=True)
            except Exception:
                pass
            time.sleep(0.45)
        else:
            time.sleep(0.2)


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
                    raw = bytes(buf[:i])
                    del buf[: i + 1]
                    # 去掉 JustFloat/噪声里的空字节，避免 VIS 行无法 startswith
                    raw = raw.replace(b"\x00", b"")
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    # 日志里只保留可打印段，避免刷屏
                    vis_i = line.find("VIS,")
                    show = line[vis_i:] if vis_i >= 0 else line[:80]
                    state["last_rx"] = show
                    log("RX " + show)
                    _parse_mcu_vis(line)
                    _parse_scan_line(line)
                    _parse_arm_line(line)
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


def servo_sync_mcu(quiet: bool = False) -> None:
    """把网页开合 P 写入 MCU 运行时（Sp），避免只靠未烧录的 config 默认值。"""
    bt_send(f"Sp{OPEN_P},{CLOSE_P}", quiet=quiet)


def servo_goto(pos: int, quiet: bool = False) -> None:
    p = max(P_MIN, min(P_MAX, int(pos)))
    bt_send(f"U{p}", quiet=quiet)


def servo_open(quiet: bool = False) -> None:
    """张开：先 Sp 再 U到张开角（不单发 O，避免旧固件仍用 2000/夹到 2500）。"""
    servo_sync_mcu(quiet=True)
    time.sleep(0.08)
    servo_goto(OPEN_P, quiet=quiet)


def servo_close(quiet: bool = False) -> None:
    servo_sync_mcu(quiet=True)
    time.sleep(0.08)
    servo_goto(CLOSE_P, quiet=quiet)


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
    pos: int = Field(1500, ge=500, le=4000)


class VisionCmdBody(BaseModel):
    cmd: int = Field(..., ge=0, le=255)
    via: str = "bt"  # bt | wifi


class CalibBody(BaseModel):
    open_p: Optional[int] = None
    close_p: Optional[int] = None
    apply_mcu: bool = True  # True → 发 Sp 写入主板运行时开合 P


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
    """立即返回；M0→张开→vc/ve/vf 在后台线程执行，避免网页按钮卡住约 1s+。"""
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
    # 不要清空 mcu_vis：否则点开始后准星会先消失，直到下一帧 VIS
    log(
        f"COOP queued open={OPEN_P} cmd={body.cmd:02X} ve={body.kind},{body.id} "
        f"follow={int(body.follow)} auto_close={int(body.auto_close)}"
    )
    threading.Thread(
        target=_coop_start_worker,
        args=(int(body.cmd), int(body.kind), int(body.id), bool(body.follow)),
        daemon=True,
        name="coop-start",
    ).start()
    return {"ok": True, "coop": state["coop"], "open_p": OPEN_P, "close_p": CLOSE_P}


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
    servo_open()
    return {"ok": True, "pos": OPEN_P, "via": f"Sp+U{OPEN_P}"}


@app.post("/api/close")
def api_close():
    servo_close()
    return {"ok": True, "pos": CLOSE_P, "via": f"Sp+U{CLOSE_P}"}


@app.post("/api/goto")
def api_goto(body: PosBody):
    p = max(P_MIN, min(P_MAX, int(body.pos)))
    servo_goto(p)
    return {"ok": True, "pos": p}


@app.post("/api/arrive_close")
def arrive_close():
    """到位合爪：网页标定合紧角 U（不单靠 va）。"""
    servo_close()
    return {"ok": True, "pos": CLOSE_P}


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
    """记住开合 P；默认发 Sp 写入 MCU 运行时（O/C/va 立即生效，掉电丢失）。"""
    global OPEN_P, CLOSE_P
    if body.open_p is not None:
        OPEN_P = max(P_MIN, min(P_MAX, int(body.open_p)))
    if body.close_p is not None:
        CLOSE_P = max(P_MIN, min(P_MAX, int(body.close_p)))
    applied = False
    if body.apply_mcu:
        bt_send(f"Sp{OPEN_P},{CLOSE_P}")
        applied = True
        log(f"CALIB Sp{OPEN_P},{CLOSE_P} → MCU runtime")
    else:
        log(f"CALIB open={OPEN_P} close={CLOSE_P}（仅网页记忆）")
    return {"ok": True, "open_p": OPEN_P, "close_p": CLOSE_P, "mcu": applied}


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
            "scan": state.get("scan"),
            "arm": state.get("arm"),
            "log": logs,
        }
    )


class ArmJogBody(BaseModel):
    mm: float = 5.0


class ArmRpmBody(BaseModel):
    rpm: float = 120.0


@app.post("/api/arm/jog")
def arm_jog(body: ArmJogBody):
    """同步带轴①相对移动 mm：I1 <mm>。"""
    mm = float(body.mm)
    if abs(mm) < 0.01:
        raise HTTPException(400, "mm 太小")
    bt_send(f"I1 {mm:.2f}")
    return {"ok": True, "mm": mm}


@app.post("/api/arm/zero")
def arm_zero():
    bt_send("N1")
    return {"ok": True}


@app.post("/api/arm/read")
def arm_read():
    bt_send("L")
    return {"ok": True}


@app.post("/api/arm/enable")
def arm_enable(on: int = 1):
    bt_send(f"E1 {1 if on else 0}")
    return {"ok": True, "en": bool(on)}


@app.post("/api/arm/rpm")
def arm_rpm(body: ArmRpmBody):
    bt_send(f"V1 {float(body.rpm):.0f}")
    return {"ok": True, "rpm": float(body.rpm)}


@app.post("/api/scan/query")
def scan_query():
    """vs：不消费查询主板持有任务码。"""
    bt_send("vs")
    return {"ok": True}


@app.post("/api/scan/clear")
def scan_clear():
    bt_send("vs0")
    state["scan"] = {"code": "", "ok": False, "t": time.time()}
    return {"ok": True}


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
