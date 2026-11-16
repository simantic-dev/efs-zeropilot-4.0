# SD socket card-detect switch on PA15 (active low: the firmware, main.cpp and
# sd.cpp, treats PA15 high as "no card"). Holds the pin low from time 0.
class Slave:
    def __init__(self, ctx):
        ctx.gpio(0).set(False)

    def on_gpio(self, line, level):
        pass
