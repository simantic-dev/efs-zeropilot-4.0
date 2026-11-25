"""Dashboard GIF of the plane's MAVLink: usage make_gif.py out/out.txt screenshots/zeropilot.gif "<sim version>" [out/log.txt]
Everything is decoded from the MAVLink the firmware sends on USART1 (view.py, CRC-checked). Pillow + ffmpeg only (gifkit.py)."""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gifkit import *
from view import frames, decode, summary
from gcs_peer import NAMES

VER = sys.argv[3]
ev = [(t, mid, decode(mid, p) if mid in (0, 24, 27, 29, 30, 36, 65, 132) else None, p) for t, mid, p in frames(sys.argv[1])]
log, seen = [], set()
for t, mid, d, p in ev:
    if mid == 253:
        log.append((t, "WARN", "firmware STATUSTEXT: " + p[1:].split(b"\0")[0].decode(errors="replace")))
    if mid not in seen:
        seen.add(mid); log.append((t, "INFO", f"first MAVLink {NAMES.get(mid, mid)} (id {mid})"))
log.sort()
def series(mid, f): return [(t, f(d)) for t, m, d, _ in ev if m == mid]
pitch = series(30, lambda d: d["pitch"] * 57.2958)
elev = series(36, lambda d: d["s2"])
fix = series(24, lambda d: d["fix_type"])
armed = series(0, lambda d: bool(d["base_mode"] & 0x80))
def t_st(v): return next((a for a, m, d, _ in ev if m == 0 and d["system_status"] == v), None)   # MAV_STATE: 5 CRITICAL, 3 STANDBY, 4 ACTIVE
t_fix = next((t for t, v in fix if v >= 3), None)
t_arm = next((t for t, v in armed if v), None)
END = 20.0
MSG = [(0, "HEARTBEAT"), (24, "GPS_RAW_INT"), (30, "ATTITUDE"), (27, "RAW_IMU"), (29, "SCALED_PRESSURE"), (36, "SERVO_OUTPUT_RAW"), (65, "RC_CHANNELS"), (132, "DISTANCE_SENSOR")]

def last(mid, t):
    r = [(a, d) for a, m, d, _ in ev if m == mid and a <= t]
    return r[-1] if r else None

def draw(c, t):
    c.header("ZeroPilot 4.0 plane firmware (STM32H753) on a simulated bench", "team firmware, unmodified; all values are its own CRC-checked MAVLink. Sensors, RC and the safety button are scripted peers.", t)
    a, g, b, r, s = last(30, t), last(24, t), last(29, t), last(132, t), last(36, t)
    hb = last(0, t); arm = bool(hb and hb[1]["base_mode"] & 0x80)
    tw = 196; X = [24 + i * (tw + 12) for i in range(6)]
    c.tile(X[0], 90, tw, 92, "PITCH (ATTITUDE)", f"{a[1]['pitch'] * 57.2958:.1f}" if a else "--", "deg", CYAN)
    c.tile(X[1], 90, tw, 92, "GPS FIX / SATS", f"{g[1]['fix_type']} / {g[1]['sats']}" if g else "--", "", GREEN if g and g[1]["fix_type"] >= 3 else AMBER)
    c.tile(X[2], 90, tw, 92, "BARO (0.0 = known gap)", f"{b[1]['press_hPa']:.1f}" if b else "--", "hPa", CYAN)
    c.tile(X[3], 90, tw, 92, "RANGEFINDER", f"{r[1]['cur_cm'] / 100:.2f}" if r else "--", "m", CYAN)
    c.tile(X[4], 90, tw, 92, "ELEVATOR SERVO", f"{s[1]['s2']}" if s else "--", "us", AMBER)
    c.tile(X[5], 90, tw, 92, "ARMED", "YES" if arm else "NO", "", GREEN if arm else RED)
    c.panel(24, 196, 1232, 112, "Arming sequence: HEARTBEAT system_status / base_mode and GPS_RAW_INT, as the firmware sent them")
    names = ["HEARTBEAT: CRITICAL", "GPS FIX 3", "HEARTBEAT: STANDBY", "ACTIVE + ARMED"]
    times = [x if (x is not None and x <= t) else None for x in (t_st(5), t_fix, t_st(3), t_arm)]
    act = max((i for i, x in enumerate(times) if x is not None), default=None)
    c.chain(40, 226, 1200, 70, names, act, times, [RED, AMBER, BLUE, GREEN], sz=17)
    c.panel(24, 322, 520, 360, "MAVLink from the firmware, latest value")
    for i, (mid, nm) in enumerate(MSG):
        e = last(mid, t); y = 354 + i * 40
        c.text(38, y, nm, 15, FG if e else DIM, True)
        c.text(38, y + 19, (summary(e[1], mid) or "") if e else "not sent yet", 13, AMBER if e else DIM, False, True)
    c.strip(556, 322, 350, 168, pitch, t, END, -12, 4, "pitch (deg)", CYAN, "", ticks=4)
    c.strip(918, 322, 338, 168, elev, t, END, 1000, 2000, "elevator servo (us)", AMBER, "", step=True, ticks=4)
    c.log(556, 502, 700, 180, log, t, "firmware STATUSTEXT and first-seen MAVLink messages", sz=14)
    c.footer(VER, SPEED, ".  Peers' tilt and GPS are stimulus; the firmware computes what you see.")

SPEED = END / 15
print("frames", render(draw, END, SPEED, sys.argv[2]))
