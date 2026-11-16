"""Render PNGs from a finished run (needs matplotlib):
  decoded.png   the decoded-MAVLink table (out/view.txt) and the SD card listing, as text
  signals.png   attitude, GPS_RAW_INT, servo outputs and pressure vs virtual time, from the MAVLink on USART1
usage: python3 screenshots.py out/out.txt out/view.txt out/card.img <caption> <outdir> [name-suffix]"""
import subprocess, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from view import frames, decode

out, viewtxt, card, caption, outdir = sys.argv[1:6]
name = sys.argv[6] if len(sys.argv) > 6 else ""

# text screenshot: viewer table without the periodic task-timing STATUSTEXT, plus the card listing
lines = [l.rstrip() for l in open(viewtxt)]
table = lines[:11]
listing = subprocess.run(["mdir", "-/", "-i", card, "::"], capture_output=True, text=True).stdout.strip().splitlines()
text = "\n".join(table + ["", "SD card after the run (mdir -/):"] + listing + ["", caption])
fig = plt.figure(figsize=(10, 0.24 * (len(text.splitlines()) + 1)), facecolor="#111")
fig.text(0.01, 0.99, text, family="monospace", fontsize=8, color="#ddd", va="top")
fig.savefig(f"{outdir}/decoded{name}.png", dpi=90, facecolor="#111", bbox_inches="tight")

# time series
s = {k: ([], []) for k in "roll pitch yaw lat lon servo1 servo2 servo3 servo4 press".split()}
for t, mid, p in frames(out):
    if mid == 30:
        d = decode(mid, p)
        for k in ("roll", "pitch", "yaw"):
            s[k][0].append(t); s[k][1].append(d[k] * 57.2958)
    elif mid == 24:
        d = decode(mid, p); s["lat"][0].append(t); s["lat"][1].append(d["fix_type"] * 1.0)
    elif mid == 36:
        d = decode(mid, p)
        for k in ("servo1", "servo2", "servo3", "servo4"):
            s[k][0].append(t); s[k][1].append(d["s" + k[-1]])
fig, ax = plt.subplots(3, 1, figsize=(10, 6.5), sharex=True)
for k in ("roll", "pitch", "yaw"):
    ax[0].plot(*s[k], label=k)
ax[0].set_ylabel("ATTITUDE (deg)"); ax[0].legend(loc="right")
ax[1].step(*s["lat"], where="post", color="g"); ax[1].set_ylabel("GPS_RAW_INT fix_type"); ax[1].set_ylim(-0.2, 4)
for k, n in zip(("servo1", "servo2", "servo3", "servo4"), ("aileron", "elevator", "throttle", "rudder")):
    ax[2].step(*s[k], where="post", label=n)
ax[2].set_ylabel("SERVO_OUTPUT_RAW (us)"); ax[2].legend(loc="right"); ax[2].set_xlabel("virtual time (s)")
fig.suptitle(caption, fontsize=8)
fig.tight_layout()
fig.savefig(f"{outdir}/signals{name}.png", dpi=80)
