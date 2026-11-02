import sensor
import image
import lcd
import time
from fpioa_manager import fm
from machine import UART

fm.register(8, fm.fpioa.UART1_TX, force=True)
fm.register(6, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=256)

clock = time.clock()
lcd.init(type=1)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(0)
sensor.run(1)
sensor.skip_frames()

locked = None
frame = None
acked = False
rx_buf = b""
last_send_ms = 0
lock_ms = 0
SEND_EVERY_MS = 200
ACK_WAIT_MS = 3000
ACK_TOKEN = b"$ACK#"


def make_frame(payload):
    text = payload.decode() if isinstance(payload, bytes) else str(payload)
    text = text.strip()
    if text[:1] == "$":
        text = text[1:]
    if text[-1:] == "#":
        text = text[:-1]
    return "$" + text + "#"


def to_bytes(s):
    if isinstance(s, bytes):
        return s
    return s.encode("ascii")


def poll_ack():
    """读串口，缓冲中出现 $ACK# 则返回 True。"""
    global rx_buf
    try:
        data = uart.read()
    except Exception:
        data = None
    if data:
        if isinstance(data, str):
            data = data.encode("latin-1")
        rx_buf += data
        if len(rx_buf) > 64:
            rx_buf = rx_buf[-64:]
        if ACK_TOKEN in rx_buf:
            return True
    return False


def send_frame_once():
    global last_send_ms
    if frame is None:
        return
    raw = to_bytes(frame)
    try:
        uart.write(raw)
    except Exception as e:
        print("UART write error:", e)
        return
    last_send_ms = time.ticks_ms()
    print("TX", frame)


while True:
    clock.tick()
    img = sensor.snapshot()
    now = time.ticks_ms()

    if locked is None:
        res = img.find_qrcodes()
        if len(res) > 0:
            locked = res[0].payload()
            frame = make_frame(locked)
            acked = False
            rx_buf = b""
            lock_ms = now
            send_frame_once()
    else:
        if not acked:
            if poll_ack():
                acked = True
                print("ACK ok, stop TX")
            elif time.ticks_diff(now, lock_ms) >= ACK_WAIT_MS:
                print("ACK timeout, rescan")
                locked = None
                frame = None
                rx_buf = b""
                lcd.display(img)
                continue
            elif time.ticks_diff(now, last_send_ms) >= SEND_EVERY_MS:
                send_frame_once()
        shown = locked.decode() if isinstance(locked, bytes) else locked
        img.clear()
        img.draw_string(2, 2, shown, color=(0, 128, 0), scale=3)
        if acked:
            img.draw_string(2, 40, "ACK", color=(0, 255, 0), scale=2)
        else:
            img.draw_string(2, 40, "TX...", color=(255, 128, 0), scale=2)

    lcd.display(img)
