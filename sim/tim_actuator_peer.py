# Actuator-output tap for ZeroPilot 4.0's plane build (closed loop).
# Not bus-attached (size 0 in the .repl) --
# a pure `ctx.schedule_periodic` ticker that polls TIM1/TIM2's own PSC/ARR/
# CCR registers via `ctx.bus_read` and writes the decoded pulse widths into
# `zp_plant.PLANT`, which `icm42688_peer.py` reads back for its IMU packets.
# Same in-process import as the sensor peers -- see zp_plant.py's header.
#
# MOTOR_MAP (boardfiles/model/src/drivers.cpp, MOT_PWM_TYPE=0/PWM -- the
# plane build's default; DShot/TIM-DMA-burst is a separate track,
# never exercised by this firmware configuration):
#   TIM1 CH1/2/3/4 (PE9/11/13/14) = aileron/elevator/throttle/rudder
#   TIM2 CH1/2/3/4 (PA0-3)        = aileron/flap/flap/ground steering
# Both timers: PSC=75, ARR=63158 at the 240 MHz kernel clock -> 20ms period
# (confirmed by a PWM register test against the
# same --trace-memory register reads this script performs live). CCR is
# bounded to MotorControl's configured [5%, 10%] duty cycle, i.e. 1.0-2.0ms
# -- the standard hobby servo/ESC convention zp_plant.py's normalization
# already assumes.
#
# TIM1's aileron/elevator/throttle/rudder feed the attitude model. TIM2's
# CH1/2/3/4 (aileron/flap/flap/ground steering, boardfiles/model/src/
# drivers.cpp MOTOR_MAP) are read and logged on PLANT.tim2_pulse_us for
# visibility. TIM2 CH2 (flap1) also feeds PLANT.set_flap() when the backend
# supports it (zp_jsbsim_plant.JSBSimPlant has real lift/drag state for flap
# deflection to act on; zp_plant.Plant does not -- MODELED SIMPLIFICATION on
# the kinematic fallback only). TIM2 CH1 (aileron) duplicates TIM1 CH1's
# surface and is not fed a second time (two sources commanding the same
# fcs/aileron-cmd-norm would just race); ground steering doesn't affect an
# in-flight model at all.
#
# Startup-floor gate: MotorControl::init() (boardfiles/drivers/motor/
# motor.cpp) writes CCR=minCCR (hard-over, the low end of the configured
# duty range) and starts the timer *before* the firmware's control task
# (AttitudeManager::amUpdate) ever runs -- so the very first live CCR values
# this peer would see are not a real command, just that startup floor held
# until the scheduler gets to the control task. Rather than integrate that
# floor for a fitted length of time (the old ATTITUDE_LIMIT_DEG clamp
# papered over exactly this), we hold the plant still until we observe a
# channel's CCR actually change away from the value we first saw for it --
# that change can only be the control task's own first real `set()` call
# overwriting init()'s floor.
from zp_plant import PLANT

TIM1_BASE = 0x40010000
TIM2_BASE = 0x40000000
PSC_OFF, ARR_OFF = 0x28, 0x2C
CCR_OFF = {1: 0x34, 2: 0x38, 3: 0x3C, 4: 0x40}
KERNEL_CLOCK_HZ = 240_000_000

AILERON_CH, ELEVATOR_CH, THROTTLE_CH, RUDDER_CH = 1, 2, 3, 4
TIM2_AILERON_CH, TIM2_FLAP1_CH, TIM2_FLAP2_CH, TIM2_GROUND_STEER_CH = 1, 2, 3, 4

POLL_PERIOD_US = 5000  # 200Hz: several samples per 20ms PWM period


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        self.throttle_pulse_us = 0.0
        self._floor = {}          # channel -> first-observed CCR value (the init() floor)
        self.control_running = False
        self.ctx.schedule_periodic(POLL_PERIOD_US)

    def on_access(self, req):
        # No MMIO surface (size 0); only ever reached for the INIT access a
        # machine Reset() sends, which also stops our timer -- re-arm it.
        if req.is_init:
            self.ctx.schedule_periodic(POLL_PERIOD_US)

    def on_timer(self):
        aileron = self._pulse_us(TIM1_BASE, AILERON_CH)
        elevator = self._pulse_us(TIM1_BASE, ELEVATOR_CH)
        rudder = self._pulse_us(TIM1_BASE, RUDDER_CH)
        self.throttle_pulse_us = self._pulse_us(TIM1_BASE, THROTTLE_CH)
        flap1 = self._pulse_us(TIM2_BASE, TIM2_FLAP1_CH)
        PLANT.tim2_pulse_us = {
            "aileron": self._pulse_us(TIM2_BASE, TIM2_AILERON_CH),
            "flap1": flap1,
            "flap2": self._pulse_us(TIM2_BASE, TIM2_FLAP2_CH),
            "ground_steer": self._pulse_us(TIM2_BASE, TIM2_GROUND_STEER_CH),
        }
        if flap1 is not None and hasattr(PLANT, "set_flap"):
            PLANT.set_flap(flap1)

        if not self.control_running:
            self._check_control_running(AILERON_CH, aileron)
            self._check_control_running(ELEVATOR_CH, elevator)
            self._check_control_running(RUDDER_CH, rudder)
            self._check_control_running(THROTTLE_CH, self.throttle_pulse_us)

        if aileron:
            PLANT.set_channel(0, aileron)
        if elevator:
            PLANT.set_channel(1, elevator)
        if rudder:
            PLANT.set_channel(2, rudder)
        if self.throttle_pulse_us:
            PLANT.set_channel(3, self.throttle_pulse_us)

        if self.control_running:
            PLANT.step(POLL_PERIOD_US / 1e6)

    def _check_control_running(self, channel, pulse_us):
        if pulse_us is None:
            return
        if channel not in self._floor:
            self._floor[channel] = pulse_us
        elif pulse_us != self._floor[channel]:
            self.control_running = True

    def _pulse_us(self, base, channel):
        psc = self.ctx.bus_read(base + PSC_OFF, 4)
        ccr = self.ctx.bus_read(base + CCR_OFF[channel], 4)
        if ccr == 0:
            return None  # TIM_OC*_SetConfig's pre-init CCR=0, not a real command yet
        return ccr * (psc + 1) / KERNEL_CLOCK_HZ * 1e6
