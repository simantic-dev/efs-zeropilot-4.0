# JSBSim-backed plant for ZeroPilot 4.0's closed loop
# replacing zp_plant.py's 3-axis kinematic
# model with a real 6DOF flight dynamics model. Drop-in for the same public
# surface tim_actuator_peer.py and icm42688_peer.py already use (channel_us,
# set_channel, step, gyro_dps, accel_g, roll_deg/pitch_deg/yaw_deg), plus
# GPS/baro state the kinematic model never had. zp_plant.py selects this
# class automatically when the `jsbsim` pip package is importable and falls
# back to Plant (its own kinematic model) otherwise -- see its header.
#
# We import jsbsim as an ordinary library (LGPL-2.1: dynamic linking, no
# vendored/copied source) and fly one of its own stock aircraft; no GPLv3
# code from PX4/ArduPilot/etc. is used anywhere in this file.
#
# Aircraft choice: c172x (Cessna 172), the smallest, slowest, simplest
# fixed-wing aircraft the `jsbsim` pip package ships. The catalog is mostly
# airliners, military jets, a glider and a few pre-WWII/experimental
# airframes (737, 787, A320, F-15/16/22, Concorde, Shuttle, ...); c172x is
# the closest available stand-in for a small fixed-wing UAV -- conventional
# aileron/elevator/rudder/flap FCS (the same surfaces ZeroPilot's plane
# build drives), a simple piston engine, and a docile flight envelope. A
# dedicated small-UAV airframe (e.g. "rascal", common in ArduPilot SITL) is
# not bundled with this pip package, and vendoring one here is out of scope
# (this fixture ships no third-party aircraft config, only what pip
# installs). MODELED SIMPLIFICATION: mass, inertia and aerodynamics are a
# 1982 Cessna 172's, not an actual UAV airframe.
import math
import os

import jsbsim

from zp_plant import PITCH_TRIM_DEG, _normalize, _normalize_throttle

AIRCRAFT = "c172x"
CRUISE_ALT_FT = 2000.0
CRUISE_VC_KTS = 90.0
# the test field, Waterloo ON -- matches gps_peer.py's LAT_DEG/LON_DEG.
HOME_LAT_DEG = 43.4723
HOME_LON_DEG = -80.5449
# Virtual seconds of engine cranking before the piston engine's own combustion
# model settles at a steady RPM (checked against propulsion/engine/engine-rpm
# during development, not fitted to any test assertion) -- JSBSim's engine
# model needs starter+magneto+mixture held for a few seconds before it catches,
# the same as a real Lycoming needs real cranking time.
ENGINE_START_S = 10.0

FT_PER_M = 0.3048
PSF_PER_KPA = 1.0 / 0.0478803  # 1 lbf/ft^2 = 47.8803 Pa


def _spin_up_engine(fdm, dt_s):
    fdm["propulsion/magneto_cmd"] = 3  # both magnetos
    fdm["propulsion/starter_cmd"] = 1
    fdm["fcs/mixture-cmd-norm"] = 1
    fdm["fcs/throttle-cmd-norm"] = 0.8
    for _ in range(int(ENGINE_START_S / dt_s)):
        fdm.run()


class JSBSimPlant:
    """Same public surface as zp_plant.Plant (see its docstring), backed by
    a real flight dynamics model instead of 3-axis kinematics."""

    def __init__(self, initial_roll_deg=0.0, initial_pitch_deg=PITCH_TRIM_DEG):
        jsbsim.FGJSBBase().debug_lvl = 0  # silence JSBSim's startup banner on stdout
        self._dt = 0.005  # matches tim_actuator_peer.py's POLL_PERIOD_US (200 Hz)
        # jsbsim.get_default_root_dir() locates the package's own data files
        # (aircraft/, engine/, ...) from __file__ -- it fails to resolve under
        # the sim CLI's embedded pythonnet interpreter (no normal __main__
        # entry point), so compute the same path directly from the installed
        # package location instead.
        root_dir = os.path.dirname(os.path.abspath(jsbsim.__file__))
        fdm = self._fdm = jsbsim.FGFDMExec(root_dir)
        fdm.set_debug_level(0)
        fdm.set_dt(self._dt)
        fdm.load_model(AIRCRAFT)
        fdm["ic/h-sl-ft"] = CRUISE_ALT_FT
        fdm["ic/vc-kts"] = CRUISE_VC_KTS
        fdm["ic/lat-gc-deg"] = HOME_LAT_DEG
        fdm["ic/long-gc-deg"] = HOME_LON_DEG
        fdm.run_ic()
        _spin_up_engine(fdm, self._dt)
        # Trim for straight, level, wings-level cruise flight -- JSBSim's own
        # physics-based trim solver (FGTrim), not a fitted constant.
        fdm.do_trim(1)
        theta_trim = fdm["attitude/theta-deg"]
        trim_cmds = {p: fdm[p] for p in (
            "fcs/aileron-cmd-norm", "fcs/elevator-cmd-norm",
            "fcs/rudder-cmd-norm", "fcs/throttle-cmd-norm")}
        if initial_roll_deg:
            # A standing disturbance (a gust that rolled the airframe before
            # the run starts recording -- ZP4_PLANT_INITIAL_ROLL_DEG) --
            # re-seed the trimmed speed/altitude/pitch with the disturbance
            # applied to roll, holding the same trimmed control positions as
            # the aircraft's resting point. tim_actuator_peer.py's
            # control_running gate holds the plant still (step() is never
            # called) until the firmware's own control task overwrites these
            # with a live command, so this is the pre-control resting state,
            # exactly like zp_plant.Plant(initial_roll_deg=...).
            # No fdm.run() below: tim_actuator_peer.py's control_running gate
            # freezes the plant (step() is never called) until the firmware's
            # own control task takes over, so this must be the exact IC, not
            # a few seconds of unattended flight -- 10s of uncorrected bank
            # with straight-flight trim surfaces would drift the disturbance
            # itself, not hold it. The engine (starter/magneto/mixture stay
            # set below) re-spools for real, in virtual time, once the
            # firmware's own throttle command starts driving step().
            fdm["ic/phi-deg"] = initial_roll_deg
            fdm["ic/theta-deg"] = theta_trim
            fdm.run_ic()
            fdm["propulsion/magneto_cmd"] = 3
            fdm["propulsion/starter_cmd"] = 1
            fdm["fcs/mixture-cmd-norm"] = 1
            for prop, value in trim_cmds.items():
                fdm[prop] = value
        # run_ic() only sets the state vector (position/velocity/attitude);
        # derived outputs (accelerations/*, the specific force accel_g()
        # reads) are computed by the force model during Run() and are stale
        # (near-zero) until it executes at least once. One 5ms frame here
        # populates them correctly for the frozen state above, with FCS
        # commands still at their power-on default (neutral/zero) -- this is
        # not "starting the flight" (tim_actuator_peer.py's control_running
        # gate still holds real integration off until the firmware commands
        # something), just making the plant's outputs valid from t=0.
        fdm.run()
        self.channel_us = [None] * 8
        self.tim2_pulse_us = {}

    # --- zp_plant.Plant-compatible surface ----------------------------------

    def set_channel(self, ch, pulse_us):
        if 0 <= ch < len(self.channel_us):
            self.channel_us[ch] = pulse_us

    def set_flap(self, pulse_us):
        """TIM2 CH2/CH3 (flap1/flap2, MOTOR_MAP) -- unlike zp_plant.Plant,
        this model has real lift/drag state for flap deflection to act on,
        so it is wired here. Both flap channels command the same surface in
        practice (mirrored L/R), so tim_actuator_peer.py passes one of them."""
        self._fdm["fcs/flap-cmd-norm"] = _normalize_throttle(pulse_us)

    def step(self, dt_s):
        if dt_s <= 0:
            return
        if dt_s != self._dt:
            self._dt = dt_s
            self._fdm.set_dt(dt_s)
        self._fdm["fcs/aileron-cmd-norm"] = _normalize(self.channel_us[0])
        self._fdm["fcs/elevator-cmd-norm"] = _normalize(self.channel_us[1])
        self._fdm["fcs/rudder-cmd-norm"] = _normalize(self.channel_us[2])
        self._fdm["fcs/throttle-cmd-norm"] = _normalize_throttle(self.channel_us[3])
        self._fdm.run()  # one JSBSim frame per caller tick -- lockstep, no wall clock

    def gyro_dps(self):
        return tuple(math.degrees(self._fdm[p]) for p in (
            "velocities/p-rad_sec", "velocities/q-rad_sec", "velocities/r-rad_sec"))

    def accel_g(self):
        # JSBSim's Nx/Ny/Nz load factors are the body-frame specific force in
        # g, gravity included -- the same convention as zp_plant.Plant.accel_g()
        # and icm42688_peer.py's driver-side sign handling (verified against a
        # trimmed level-flight case: Nx=sin(theta), Nz=cos(theta), Ny=0).
        return tuple(self._fdm[p] for p in ("accelerations/Nx", "accelerations/Ny", "accelerations/Nz"))

    @property
    def roll_deg(self):
        return self._fdm["attitude/phi-deg"]

    @property
    def pitch_deg(self):
        return self._fdm["attitude/theta-deg"]

    @property
    def yaw_deg(self):
        return self._fdm["attitude/psi-deg"]

    # --- GPS/baro state -- not on the kinematic fallback --------------------

    @property
    def lat_deg(self):
        return self._fdm["position/lat-gc-deg"]

    @property
    def lon_deg(self):
        return self._fdm["position/long-gc-deg"]

    @property
    def alt_m(self):
        return self._fdm["position/h-sl-ft"] * FT_PER_M

    @property
    def vel_ned_mps(self):
        return tuple(self._fdm[p] * FT_PER_M for p in (
            "velocities/v-north-fps", "velocities/v-east-fps", "velocities/v-down-fps"))

    @property
    def pressure_kpa(self):
        return self._fdm["atmosphere/P-psf"] / PSF_PER_KPA

    @property
    def temp_c(self):
        return (self._fdm["atmosphere/T-R"] - 491.67) * 5.0 / 9.0
