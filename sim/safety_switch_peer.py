# ZeroPilot's hardware safety button on PH12 (active high,
# boardfiles/model/src/drivers.cpp). The firmware boots with safety engaged and
# toggles it after the button is held SM_SAFETY_SWITCH_HOLD_THRESHOLD_MS
# (2000 ms), so this peer holds it from PRESS_US to RELEASE_US, after the
# system-manager task is running (~5.2 s virtual), the way a pilot would.
PRESS_US = 6_000_000
RELEASE_US = 8_500_000
POLL_US = 10_000


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.level = None
        ctx.schedule_periodic(POLL_US)
        self.on_timer()

    def on_gpio(self, line, level):
        pass

    def on_timer(self):
        level = PRESS_US <= self.ctx.now_us < RELEASE_US
        if level != self.level:
            self.level = level
            self.ctx.gpio(0).set(level)
