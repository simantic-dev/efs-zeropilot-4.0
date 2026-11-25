"""Animates a run's MAVLink (decoded from out/out.txt by view.py) into screenshots/zeropilot.gif.
usage: python3 make_gif.py out/out.txt "tag"   (matplotlib, pillow)"""
import sys, io
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
from view import frames, decode
s = {k: ([], []) for k in "roll pitch yaw fix servo1 servo2 servo3 servo4".split()}
for t, mid, p in frames(sys.argv[1]):
    d = decode(mid, p) if mid in (30, 24, 36) else None
    if mid == 30:
        for k in ("roll", "pitch", "yaw"): s[k][0].append(t); s[k][1].append(d[k] * 57.2958)
    elif mid == 24: s["fix"][0].append(t); s["fix"][1].append(d["fix_type"])
    elif mid == 36:
        for k in ("servo1", "servo2", "servo3", "servo4"): s[k][0].append(t); s[k][1].append(d["s" + k[-1]])
end = 20.0; frames_ = []
for k in range(1, 41):
    tc = end * k / 40
    fig, ax = plt.subplots(3, 1, figsize=(7, 5), dpi=70, sharex=True)
    cut = lambda kk: ([t for t in s[kk][0] if t <= tc], s[kk][1][:sum(1 for t in s[kk][0] if t <= tc)])
    for kk in ("roll", "pitch", "yaw"): ax[0].plot(*cut(kk), label=kk)
    ax[0].set_ylabel("ATTITUDE, deg", fontsize=7); ax[0].legend(loc="upper left", fontsize=6)
    ax[1].step(*cut("fix"), where="post", c="g"); ax[1].set_ylabel("GPS fix_type", fontsize=7); ax[1].set_ylim(-.2, 4)
    for kk, n in zip(("servo1", "servo2", "servo3", "servo4"), ("aileron", "elevator", "throttle", "rudder")): ax[2].step(*cut(kk), where="post", label=n)
    ax[2].set_ylabel("SERVO_OUTPUT_RAW, us", fontsize=7); ax[2].legend(loc="upper left", fontsize=6, ncol=4); ax[2].set_xlim(0, end); ax[2].set_xlabel("virtual time, s")
    for a in ax: a.grid(alpha=.3)
    ax[0].set_title(f"ZeroPilot 4.0 on simulated H753, MAVLink from the firmware, t={tc:.1f} s virtual, {sys.argv[2]}", fontsize=8)
    fig.tight_layout(); bf = io.BytesIO(); fig.savefig(bf, format="png"); plt.close(fig)
    frames_.append(Image.open(bf).convert("RGB").quantize(colors=64, dither=Image.Dither.NONE))
frames_[0].save("screenshots/zeropilot.gif", save_all=True, append_images=frames_[1:], duration=[120] * 39 + [1500], loop=0, optimize=True)
