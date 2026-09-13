"""Ladder step 5: closed-loop evaluation of the served pi0.5 policy in Isaac.

Deploy contract, all verified against the running server and robotics_lab (see CLAUDE.md):

  server   pi05_pika_umi_wrist_velgrip_k1_h24_80k on :8001
  obs      observation/{left,right}_wrist_0_rgb  (full 640x480 RGB; the SERVER does
           resize_with_pad to 224x224, so do not pre-resize here)
           observation/state = 14-D velocity_grip
               [pos_vel3, rot_vel3, grip] per arm, left block then right block
           prompt = the fixed task sentence
  actions  (24, 14) per-step ee_local deltas; gripper dims 6/13 in /100 units

Two choices that are easy to get wrong and are deliberate here:

  * velproprio_source = "command". The velocity fed to the policy comes from the
    history of EMITTED absolute TCP targets, NOT from measured motion. Hold ticks
    re-append the last target (ZOH) so a stationary command decays to zero velocity.
    Feeding measured motion instead would hand the policy a feedback channel the
    demonstrations never had.
  * chunk_anchor = "command". Chunk deltas integrate onto the last COMMANDED pose,
    not the measured one.

Control is the Ruckig chunk follower (not the CM controller), with the limits from
rb_servo_server/config/stack_real.yaml `ruckig_follower`.

Run:
  OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=$HOME/workspace/openpi/packages/openpi-client/src \\
    .venv-isaac/bin/python scripts/eval_closed_loop.py --episodes 20 --layout aligned
"""

import argparse
import json
import math
import pathlib
import time

import os
import sys
import numpy as np
import work_surface

ROOT = pathlib.Path(__file__).resolve().parent.parent
# FINGERTIP. The robot runs the v15 tip -- rigid PLA spine + TPU 95A blade printed in
# #F3E600 -- and `data_v2`, which every served checkpoint is trained on, was collected with
# it. The old one-piece dark-grey Pika tip is therefore not a neutral default: it is the
# wrong shape against the hardware AND the wrong pixels against the training distribution,
# in the one object the wrist camera never stops seeing. Default flipped 2026-09-03 for the
# same reason OUTPUT_SMD was: this rig matches the robot.
#   v15   assets/rb3_730e_pika_tip_v15   built by scripts/build_tip_v15.py
#   orig  the pre-2026-09-03 asset. EVERY published number on this project came from it, so
#         keep it reachable -- it is the A/B partner, not dead weight.
# NOTE this is stage 1 of the tip swap: geometry, colour, colliders and mass. The blade is
# still a RIGID body; stage 2 relaxes the contact parameters for 95A compliance.
PIKA_TIP = os.environ.get("PIKA_TIP", "v15").lower()
# 2026-09-05: RB3-730E -> RB5-850E, the arm and stand the robot now runs.
# The RB5 display URDF already draws the printed PLA+TPU tip natively, so `v15` here means
# the SDF blade collider and the measured 55.8 g finger mass, not a visual graft.
SIM_ARM = os.environ.get("SIM_ARM", "rb5_850e").lower()
ARM_USD = ROOT / (f"assets/{SIM_ARM}_pika_tip_v15/{SIM_ARM}_pika_articulated_sim.usda"
                  if PIKA_TIP == "v15" else
                  f"assets/{SIM_ARM}_pika_articulated_sim/{SIM_ARM}_pika_articulated_sim.usda")
if not ARM_USD.exists():
    raise SystemExit(f"ABORT: SIM_ARM={SIM_ARM} PIKA_TIP={PIKA_TIP} but {ARM_USD} is missing.\n"
                     f"Build it with:  .venv-isaac/bin/python scripts/make_rb5_sim_urdf.py\n"
                     f"                .venv-isaac/bin/python scripts/import_urdf.py "
                     f"--urdf assets/urdf/{SIM_ARM}_pika_articulated_sim.urdf\n"
                     f"                .venv-isaac/bin/python scripts/build_tip_v15.py")
STAND_USD = ROOT / "assets/dual_rb5_850e_stand_only/dual_rb5_850e_stand_only.usda"

PROMPT = (
    "pick up the black bolt with the right arm and put it in the right box, "
    "then pick up the gray bolt with the left arm and put it in the left box"
)

# InitMotion reset pose, deg. READ FROM THE SAVED FILE, not from rb_gui's code default:
# rb_gui persists the operator's taught pose to ~/.rb_servo_gui/init_motion.json and only
# falls back to _DEFAULT_INIT_*_JOINTS_DEG when it is absent -- and those defaults are still
# the RB3 values, missed by the 2026-09-02 RB5 pass that rotated every other stand-frame
# constant. Values below are the file as of 2026-09-03 20:56. FK cross-check against 594k
# ticks of real teleop: this pose puts left TCP z at -0.054 and right at -0.072, against a
# measured dwell median of -0.055 / -0.072.
RESET = {
    "left": [-85.721, 36.301, 125.914, -9.832, -123.706, 33.556],
    "right": [86.320, -28.371, -125.802, -1.274, 123.360, -42.350],
}
ARM_JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
]
MOUNT_FRAME = {"left": "stand_left_arm_base", "right": "stand_right_arm_base"}

# ---- scene (same numbers as build_scene.py) ---------------------------------
SHAFT_R, SHAFT_L = 0.006, 0.025
HEAD_R, HEAD_L = 0.0092, 0.012
# See scripts/build_scene.py for the derivation of every number in this block. Short version:
# z = 0 is the STAND origin and the table is 295 mm below it (a 280 mm riser under a stand
# whose base plate reaches 15 mm below its own origin), and the two bolt piles sit 92 mm
# apart, not 320. Both were measured off 594k ticks of real stand-frame TCP, 2026-09-04.
TABLE_Z = -0.295
WORK_SURFACE = work_surface.load('foam')
PICK_SURFACE_Z = work_surface.top_z(WORK_SURFACE, TABLE_Z)
RISER_H = 0.280
TABLE = dict(cx=0.55, cy=0.0, hx=0.50, hy=0.55, thick=0.012)
# 240 x 300 mm, the stand's floor-bolted base plate, centred on the stand axis (measured by
# slicing the stand STL at its bottom face). NOT the mesh bbox -- see build_scene.py.
RISER = dict(cx=0.0, cy=0.0, hx=0.120, hy=0.150)
PILE_X, PILE_DY = 0.455, 0.046
# 2026-09-05: boxes pushed out. At 0.72 / 0.215 the gray box's near faces sat at x 0.60 and
# y 0.025 -- and the RB5 piles are at x 0.455, y +-0.046, so the box lip was directly over
# the pick region and the arms fouled it while picking (operator). Moved onto the MEASURED
# release clusters instead of the montage estimate: x p50 0.762/0.767, |y| 0.253/0.310 over
# the 2026-09-04 teleop sessions. Near faces now x 0.645, y 0.070.
# CAVEAT: a release point is where the TCP was, not where the box centre is, so this is a
# better estimate than 0.72 was, not a measurement of the box. Confirm against the cell.
BOX_X, BOX_DY = 0.765, 0.260
BOX = dict(hw=0.120, hd=0.190, wall_h=0.0525, t=0.020, floor_t=0.0065, sponge_h=0.030)
ARM_OF_COLOR = {"gray": "left", "black": "right"}
# top-view left box is green (black bolts), right is gray (gray bolts)
BOX_CY = {"gray": +BOX_DY, "black": -BOX_DY}

# ---- deploy parameters (from the operator's flow_infer command) -------------
POLICY_DT = 0.0334          # --policy-dt-sec, 30 Hz
ACTION_HORIZON = 24
CHUNK_EXECUTE_STEPS = 4     # FLOW_INFER_CHUNK_EXECUTE_STEPS
SPEED_SCALE = 1.0
# FLOW_INFER_RTC=1 in the deploy command. NOTE: with RTC on, robotics_lab sets
# _chunk_crossfade_steps = 0 (openpi_remote.py:592) -- RTC replaces the crossfade,
# it is not used alongside it. RTC pins the new chunk's first `inference_delay`
# rows to the PREVIOUS plan's unexecuted tail, which is what keeps velocity
# continuous across the 133 ms chunk boundary. Without it every boundary is a
# fresh plan with no continuity constraint -- visible as residual tremor.
RESERVE_STEPS = 4   # stack_real.yaml reserve_steps; lookahead for the central
                    # difference that gives each knot a nonzero target VELOCITY
RTC_ENABLED = False   # isolate the follower fix first (operator: solve tremor before RTC)
RTC_INFERENCE_DELAY = 4     # = execute_steps - prefetch_at (4 - 0)
# JAW STROKE, per side. `finger_pos = (1 - grip/100) * FINGER_TRAVEL_M`, so this is what
# "closed" means: at grip 0 each jaw has travelled this far and the two faces meet.
# 0.047 was never a measurement. It was chosen so that the VENDOR CAD's finger pose would
# close to a zero gap -- and that pose is not the open stop, it is ~3.8 mm/side inboard of
# it (robotics_lab, 2026-09-04). A vernier across the two TPU faces reads a full-open gap
# of ~98 mm with the jaws closing to contact, so the stroke is 98/2 = 49 mm per side.
# The v15 tip is placed on that same measurement, so the two have to move together: at
# 0.047 the v15 jaw would stop 2 mm/side short of closed, which on an 18.4 mm bolt head is
# the difference between a grasp and a miss. The URDF prismatic limit is +-0.05 m, so
# 0.049 fits without touching the joint. `orig` keeps 0.047 so the old asset still closes
# the way every historical number was scored.
FINGER_TRAVEL_M = float(os.environ.get(
    "FINGER_TRAVEL_M", "0.049" if PIKA_TIP == "v15" else "0.047"))
# D405 wrist optics. Default = the intrinsics the collected episodes carry (fx 393.55,
# 78.2 deg horizontal); the datasheet 87 deg value every run before 2026-09-06 used is
# H 20.955 / V 15.716 (fx 335.96). Overridable because the wrist image is the policy's only
# spatial grounding: a 17% projection error is an experiment, and an experiment that needs a
# source edit to run cannot be A/B'd against its own control.
H_APERTURE = float(os.environ.get("EVAL_H_APERTURE", "17.8885"))
V_APERTURE = float(os.environ.get("EVAL_V_APERTURE", "13.4524"))
# Principal point, in PIXELS from the 640x480 centre, per arm ("dx,dy", +x right, +y down).
# Default 0,0 = the K-normalised virtual camera (fx=fy=393, pp=320/240). The units the
# checkpoints were TRAINED on are not centred -- collection left (319.08, 229.52) and right
# (316.93, 238.20), i.e. (-0.9, -10.5) px and (-3.1, -1.8) px -- and the pipeline neither
# undistorts nor aligns pp, so rendering a centred camera is itself a distribution shift.
# 10 px at 200 mm is ~5 mm of aim error against a grasp that succeeds at |dxy| p50 8.9 mm.
PP_PX = {side: tuple(float(v) for v in
                     os.environ.get(f"EVAL_PP_{side.upper()}", "0,0").split(","))
         for side in ("left", "right")}
# --- cell photometry preset ---------------------------------------------------------
# The lighting, camera-response and material knobs below are all plain env vars so that a sweep
# can move any one of them. A board must not depend on a hand-typed env line, so EVAL_PHOTOMETRY
# names a tracked JSON ({"env": {NAME: value}}) whose values become the DEFAULTS of those knobs:
# an explicit env var still wins, which is what keeps a single-knob A/B possible on top of a
# preset. Unset = the stock rig, bit-for-bit.
PHOTOMETRY_PRESET = os.environ.get("EVAL_PHOTOMETRY", "").strip()
PHOTOMETRY_PRESET_ENV = {}
PHOTOMETRY_PRESET_SHA256 = None
if PHOTOMETRY_PRESET:
    _preset_path = pathlib.Path(PHOTOMETRY_PRESET)
    if not _preset_path.is_absolute() and not _preset_path.exists():
        _preset_path = pathlib.Path(__file__).resolve().parents[1] / _preset_path
    import hashlib
    PHOTOMETRY_PRESET_SHA256 = hashlib.sha256(_preset_path.read_bytes()).hexdigest()
    PHOTOMETRY_PRESET_ENV = {k: str(v) for k, v in
                             json.loads(_preset_path.read_text())["env"].items()}
    for _k, _v in PHOTOMETRY_PRESET_ENV.items():
        os.environ.setdefault(_k, _v)
    PHOTOMETRY_PRESET = str(_preset_path.resolve())

# --- cell lighting ------------------------------------------------------------------
# The real cell has a large window behind the work area and direct sun comes through it; on the
# robot the fingertips and parts of the boxes rail at 255. The stock rig -- a soft key (600,
# 12 deg) plus a dome (520) and no sun -- produced essentially no saturation. Light level alone
# was not the reason (see camera response below), but the window is a real hard directional
# source, so it gets its own light. EVAL_SUN_INTENSITY=0 creates no sun prim at all.
KEY_INTENSITY = float(os.environ.get("EVAL_KEY_INTENSITY", "600"))
KEY_ANGLE = float(os.environ.get("EVAL_KEY_ANGLE", "12"))
DOME_INTENSITY = float(os.environ.get("EVAL_DOME_INTENSITY", "520"))
SUN_INTENSITY = float(os.environ.get("EVAL_SUN_INTENSITY", "0"))
SUN_ANGLE = float(os.environ.get("EVAL_SUN_ANGLE", "1.5"))        # small angular size = hard sun
# Direction: the prim gets rotateXYZ(elevation - 90, 0, azimuth) and a DistantLight shines along
# its local -Z, so at azimuth 0 the light ARRIVES FROM +y (stand frame) at the given elevation, and
# +azimuth swings that source counter-clockwise about +z (azimuth -90 = from +x, the far side of
# the table). Checked by hand against USD's rotateXYZ order (X first, then Z).
SUN_AZIMUTH = float(os.environ.get("EVAL_SUN_AZIMUTH", "0"))      # deg about +z
SUN_ELEVATION = float(os.environ.get("EVAL_SUN_ELEVATION", "35"))  # deg above the horizon
# There is deliberately no fingertip-finish knob. The tips live in an instanceable payload, so a
# runtime shader override reaches nothing, and patching the asset layer showed roughness does not
# matter: under a hard clip the tip band clipped 11.1 / 10.6 / 10.5 / 10.7 % at roughness
# 0.03 / 0.10 / 0.25 / 0.50 (deployment 10.8 %). The TPU albedo (0.896, 0.791, 0) is the brightest
# diffuse surface in the scene and rails on diffuse light alone (2026-09-11 sweep).


# --- camera response ----------------------------------------------------------------
# Isaac renders through Kit's Iray-Reinhard tonemapper (op 6, burnHighlights 0.7, whiteScale
# 40.2), which rolls highlights off instead of clipping them. The D405 is the opposite: its
# auto-exposure sits pinned at the 90 fps ceiling (9947 us, gain 0, a single value across
# 1.66 M camera-quality rows on 2026-09-10/11), so it behaves as a fixed linear gain and a hard
# clip at 255. Across tonemap ops 1-8 the tip band clipped exactly 0.00 % at any light level;
# only op 0 (clamp) can rail. Under op 0 the film/exposure settings (filmIso, fNumber,
# cameraShutter, cm2Factor) had no effect, so exposure is set by the light intensities instead --
# which is why a fitted preset carries intensities of order 1 rather than hundreds.
# EVAL_RTX is a semicolon-separated list of carb settings applied right after the app starts:
#   EVAL_RTX='/rtx/post/tonemap/op=0'
# Empty keeps Kit's defaults bit-for-bit.
RTX_SETTINGS = os.environ.get("EVAL_RTX", "").strip()


def _apply_rtx_settings():
    """Apply EVAL_RTX to carb. Must run after SimulationApp() and before the first render."""
    if not RTX_SETTINGS:
        return
    import carb

    st = carb.settings.get_settings()
    for item in RTX_SETTINGS.split(";"):
        item = item.strip()
        if not item:
            continue
        key, _, raw = item.partition("=")
        key, raw = key.strip(), raw.strip()
        old = st.get(key)
        # carb is typed: writing a float into an int setting silently does nothing, so the new
        # value is coerced to the type already there and the result is read back and printed.
        if isinstance(old, bool) or raw.lower() in ("true", "false"):
            val = raw.lower() == "true"
        elif isinstance(old, int) and not isinstance(old, bool):
            val = int(float(raw))
        else:
            val = float(raw)
        st.set(key, val)
        got = st.get(key)
        flag = "" if got == val else f"  !! read back {got!r}"
        print(f"  [rtx] {key} {old!r} -> {got!r}{flag}", flush=True)


# --- cell materials -----------------------------------------------------------------
# The cell's surfaces were authored by eye. Two were outside anything lighting could fix: the
# table's diffuseColor is a LINEAR albedo of 0.42 while the foam pad beside it comes from an sRGB
# texture that lands near 0.10 linear, so any exposure that puts the pad near the deployment's
# level railed the whole table top as one block; and the gray bolts were metallic 1.0 / rough
# 0.32, i.e. polished chrome, where the real bolts are matte zinc. EVAL_MAT overrides any
# (name, field) of the MAT table without touching the code:
#   EVAL_MAT='table.metallic=0;table.rgb=0.10,0.11,0.12;bolt_gray.metallic=0.14'
# Names: table riser green boxgray insert bolt_gray bolt_black. Fields: rgb (3 comma-separated
# floats), rough, metallic. Empty keeps the authored values.
_MAT_NAMES = ("table", "riser", "green", "boxgray", "insert", "bolt_gray", "bolt_black")
_MAT_OVERRIDE = {}
for _item in os.environ.get("EVAL_MAT", "").split(";"):
    _item = _item.strip()
    if not _item:
        continue
    _lhs, _, _rhs = _item.partition("=")
    _name, _, _field = _lhs.strip().partition(".")
    if _name not in _MAT_NAMES or _field not in ("rgb", "rough", "metallic"):
        # a typo here would silently render the authored material and read as a materials result
        raise SystemExit(f"EVAL_MAT: unknown target {_lhs.strip()!r}")
    _MAT_OVERRIDE.setdefault(_name, {})[_field] = (
        tuple(float(v) for v in _rhs.split(",")) if _field == "rgb" else float(_rhs))


def photometry_metadata() -> dict:
    """Everything that decides what the wrist cameras see, for run provenance."""
    return {"preset": PHOTOMETRY_PRESET or None, "preset_sha256": PHOTOMETRY_PRESET_SHA256,
            "preset_env": PHOTOMETRY_PRESET_ENV,
            "rtx": RTX_SETTINGS,
            "light": {"key": KEY_INTENSITY, "key_angle": KEY_ANGLE, "dome": DOME_INTENSITY,
                      "sun": SUN_INTENSITY, "sun_angle": SUN_ANGLE,
                      "sun_az": SUN_AZIMUTH, "sun_el": SUN_ELEVATION},
            "material_overrides": {k: dict(v) for k, v in _MAT_OVERRIDE.items()}}


MEASURED_OPTICS = (17.8885, 13.4524)
if (H_APERTURE, V_APERTURE) != MEASURED_OPTICS:
    # Scoring standard (2026-09-07): every bolt_v2 / data_v2 checkpoint is scored at the
    # measured optics. An override is legitimate for a stated A/B and illegitimate as a board,
    # so it says so on its way past rather than hiding in a settings dump.
    print("=" * 78 + "\n"
          f"  WARNING: wrist optics overridden to H {H_APERTURE} / V {V_APERTURE} "
          f"(fx {11.0 / H_APERTURE * 640:.2f} px).\n"
          f"  The scoring standard is the MEASURED {MEASURED_OPTICS[0]} / {MEASURED_OPTICS[1]} "
          "(fx 393.55). This run is an A/B arm,\n"
          "  not a board, and must not be compared to boards scored at the standard.\n"
          + "=" * 78, flush=True)

# "actual" = measured jaw (the deploy default), "command" = the value just sent (this rig's
# historical behaviour). See the observation builder for why the difference is not cosmetic.
GRIP_PROPRIO = os.environ.get("GRIP_PROPRIO", "command").lower()
# Steps the grip channel is shifted against its paired pose row, per arm. 0/0 is what every run
# to date used. Positive = the jaw command is taken from further ahead in the chunk, i.e. it
# leads the arm, compensating a jaw slower than the arm.
GRIP_LEAD = {"left": int(os.environ.get("GRIP_LEAD_L", "0")),
             "right": int(os.environ.get("GRIP_LEAD_R", "0"))}
# DEPLOY CONTRACT: the runner subtracts a close-bias from the commanded opening
# (--gripper-close-bias-left/right, code defaults 2/6, operator runs use ~4). The 8/19
# hardware logs show why it exists: the policy commands the BOLT'S WIDTH (right cmd p50 18.5%
# vs stall width 19.6%) because the UMI hand held exactly that width -- so without the bias the
# jaws sit at zero pinch force, and the measured +-3%/step command chatter breaks contact. The
# sim never modelled it; under a force-capped drive that omission alone explains the transport
# drops. 0/0 = the historical behaviour.
GRIP_BIAS = {"left": float(os.environ.get("GRIP_BIAS_L", "0")),
             "right": float(os.environ.get("GRIP_BIAS_R", "0"))}
# Jaw transport delay, per arm, in ms. Hardware measures 105 (left) and 209 (right); this rig
# has always had ~0 because the position drive is stiff. Without it a lead value tuned in sim
# has the WRONG SIGN for the robot: sim's jaw leads the arm, hardware's trails it.
GRIP_LAG_MS = {"left": float(os.environ.get("GRIP_LAG_L", "0")),
               "right": float(os.environ.get("GRIP_LAG_R", "0"))}
GRIP_FLOOR = float(os.environ.get("GRIP_FLOOR", "0"))
# Interface-experiment modes (2026-08-24). STATE_MODE=posegrip feeds reset-relative pose state
# (converter `_rel`: pose in the episode-start command frame) instead of 1-step velocity;
# ACTION_MODE=anchored interprets chunk rows as UMI t0-relative waypoints (PikaUmiInputs
# `_anchor_relative_chunk` inverse) instead of chained per-step deltas. Frame mapping to the
# sim's TCP frame is the R_ALIGN conjugation, which for both 3-vectors is the same linear
# R_ALIGN multiply used for deltas.
STATE_MODE = os.environ.get("STATE_MODE", "velocity").lower()
ACTION_MODE = os.environ.get("ACTION_MODE", "delta").lower()
# rtc_raw_actions is MODEL-SPACE (normalized). The anchored RTC re-anchor is SE(3) algebra,
# which is invalid on normalized values (real-robot 20260825: normalized rotvec -> garbage
# rotation -> 30-50 mm chunk-boundary jumps, base-ward drift; the sim ran the same bad math
# with milder symptoms -- likely part of anchored's 3x tremor). Unnormalize with the served
# checkpoint's action q01/q99, transform, renormalize. RTC_NORM_STATS overrides the default.
# This used to default to a HARDCODED path -- one specific checkpoint's norm stats. Serving
# any other anchored checkpoint then loaded the wrong normalisation SILENTLY, because that
# file exists and the "not found" warning below never fired. The stats are now taken from the
# SERVED checkpoint (see _checkpoint_contract); this variable is the override only.
RTC_NORM_STATS = os.environ.get("RTC_NORM_STATS", "")
_RTC_NORM_Q = None
_CONTRACT: dict = {}      # what the served checkpoint says; filled once the server is known


def _load_rtc_norm_stats(path: str) -> bool:
    """Load action q01/q99 for the anchored RTC re-anchor. Returns whether it took."""
    global _RTC_NORM_Q
    if not path or not os.path.exists(path):
        return False
    import json as _json_ns
    a = _json_ns.load(open(path))["norm_stats"]["actions"]
    _RTC_NORM_Q = (np.asarray(a["q01"], dtype=np.float64)[:14],
                   np.asarray(a["q99"], dtype=np.float64)[:14])
    print(f"[eval] anchored RTC norm stats: {path}")
    return True


def _die(msg: str) -> None:
    """Print loudly and EXIT. Not `raise SystemExit` -- that does not end this process.

    Measured 2026-09-04: the contract gate raised SystemExit at t+16 s, the message printed,
    and the process then sat there until an external timeout killed it at t+400 s. Isaac's
    SimulationApp keeps non-daemon threads alive, so an unhandled SystemExit unwinds main()
    and the interpreter never gets to leave. A guard that prints and hangs is only half a
    guard: nothing downstream (run_std20.sh reads PIPESTATUS) sees a failure, and a queued
    sweep stalls instead of moving on.

    So: flush both streams, then os._exit -- which cannot be blocked by a lingering thread.

    It does NOT call SimulationApp.close() first, and that is not an oversight. Isaac runs
    with `--/app/fastShutdown=True`, so close() ends the process itself, with status 0:
    the first version of this helper printed the abort, terminated in 18 s, and STILL
    reported success, which is the same silent pass wearing a shorter runtime. run_std20.sh
    keys off PIPESTATUS and would have gone on to write a t1 report for a run that never
    scored anything. The kernel reclaims the GPU context when the process dies, so there is
    nothing close() protects that is worth the wrong exit code.
    """
    # stderr only. Writing to both duplicated it in every log this repo keeps, because the
    # wrappers all merge (run_std20.sh: `2>&1 | tee`) and Isaac re-echoes stderr through its
    # own logger on top of that -- three copies of the same abort. stderr is the stream a
    # fatal belongs on and the one every wrapper already captures.
    try:
        sys.stdout.flush()
    except Exception:  # noqa: BLE001 - a dying process must not die harder
        pass
    try:
        sys.stderr.write(msg if msg.endswith("\n") else msg + "\n")
        sys.stderr.flush()
    except Exception:  # noqa: BLE001
        pass
    os._exit(1)


def _abort_unverified(what: str, why: str) -> None:
    """Kill the run over an interface the rig could not verify against the served model.

    The house rule, after 2026-09-04 (operator): a contract problem is a LOUD ERROR AND AN
    EXIT, never a warning the run continues past. Every failure in this class produces a
    summary that looks healthy -- blank_obs 0, close events logged, video written, latency
    normal -- and is scored as if it meant something. A warning in a 200-line startup log
    is indistinguishable from silence, and the number outlives the log.

    ONE escape hatch for the whole class, and it has to be typed rather than defaulted,
    because this rig does run deliberate interface experiments (section 18).
    """
    if os.environ.get("ALLOW_UNVERIFIED_CONTRACT") == "1":
        print(f"[eval] !! UNVERIFIED CONTRACT ALLOWED: {what}\n"
              f"       ({why})\n"
              f"       This run is an experiment, not a score.")
        return
    _die(f"ABORT: {what}.\n"
         f"  Why this is fatal: {why}.\n"
         f"  Fix the mismatch, or set ALLOW_UNVERIFIED_CONTRACT=1 if running it "
         f"knowingly IS the experiment.")


def _checkpoint_contract(policy_dir: str) -> dict:
    """What ACTION contract the served checkpoint was TRAINED under, read from the
    checkpoint itself.

    The rig cannot ask the server this -- openpi's websocket metadata does not carry it --
    and getting it wrong is not a subtle error. An `anchored` checkpoint emits 24 rows that
    are each an INDEPENDENT offset from the chunk anchor; the `delta` branch chains them, so
    the command accumulates ~24 waypoints' worth of offset per chunk and the arm leaves the
    workspace. Measured on :8002 (pi05_pika_umi_boltv2_anchAB_ph3_h24_40k, 2026-09-04):
    decoded as delta the gripper closed with the bolt 270.6 mm BELOW it; decoded as anchored,
    -5.9 mm. Same policy, same scene, same seed.

    The signal is the training dataset's name, which openpi writes into the checkpoint's own
    assets tree next to the norm stats:
        ..._tcp_anchored_...  -> anchored      ..._tcp_gripabs_...  -> delta
    Verified across every checkpoint on this machine (anchAB/anchored/boltv2/boltv2ph3 are
    anchored; pad/v2_nolang/velgrip_real are gripabs).
    """
    import glob
    hits = sorted(glob.glob(os.path.join(policy_dir, "assets", "*", "*", "norm_stats.json")))
    if not hits:
        return {}
    dataset = os.path.basename(os.path.dirname(hits[0]))
    mode = ("anchored" if "_tcp_anchored_" in dataset else
            "delta" if "_tcp_gripabs_" in dataset else "")
    return {"dataset": dataset, "action_mode": mode, "norm_stats": hits[0]}


def _rtc_unnorm(n, q01, q99):
    return (np.asarray(n, dtype=np.float64) + 1.0) / 2.0 * (q99 - q01 + 1e-6) + q01


def _rtc_renorm(x, q01, q99):
    return ((x - q01) / (q99 - q01 + 1e-6) * 2.0 - 1.0).astype(np.float32)
# ee_local r_align: pika_rz180 = diag(-1,-1,+1), applied to BOTH linear and angular
R_ALIGN = np.diag([-1.0, -1.0, 1.0])

# ---- Ruckig follower limits (stack_real.yaml: safety.ruckig_follower) -------
LIN_V, LIN_A = 0.45, 12.0
ANG_V, ANG_A = 0.90, 40.0
LIN_J = float(os.environ.get("LIN_JERK", "2000.0"))
ANG_J = float(os.environ.get("ANG_JERK", "4000.0"))
# JERK: 4000/8000 -> 2000/4000 on 2026-09-03, to follow stack_real.yaml. The robot halved this
# pair on 2026-08-28 "to soften the acceleration transients that excite the arm's 11-13 Hz
# mode": at dt 2 ms, 4000 m/s^3 lets acceleration swing 8 m/s^2 in ONE tick, so the 12 m/s^2
# ceiling is reachable in ~3 ms -- near-step acceleration, broadband, straight into that mode.
# Their measurement: Ruckig used only 58% of the jerk limit at p50 while acceleration ran at
# 45% p50 and touched the ceiling, and vibration correlated with acceleration (+0.61) more than
# with jerk (+0.42). The feasibility cost was checked on their own knot stream and is small
# (conv 93.1% -> 90.0% left, 95.4% -> 92.7% right). The accels 12/40 are NOT halved.
# NOTE the reference config is the RB3-730E one (fad2cd4^): stack_real.yaml and stack_sim.yaml
# both switched to the RB5-850E on 2026-09-02 and their live values are for a different arm.
# ORIENTATION REPRESENTATION IN THE FOLLOWER. `tangent` (default) mirrors
# cartesian_chunk_follower.cpp: a quaternion base R0_ref plus a SMALL rotation vector in its
# tangent, re-based at every knot (relinearizeAndReseed). `abs` is the historical rig
# behaviour -- the ABSOLUTE rotvec straight into Ruckig axes 3-5.
# Why this matters (measured 2026-09-03): the tool-down attitude this task uses sits at
# |rotvec| = 3.05-3.14 rad, i.e. ON the pi boundary, and mat_to_rotvec canonicalises to w>=0.
# Every crossing therefore flips the 3-vector to its antipode: the true rotation step between
# knots is 0.34-0.64 deg while the RAW rotvec difference reaches 357-360 deg, 12-44 times per
# 20 s episode. Ruckig interpolates its axes linearly, so each flip slews the reference the
# wrong way at up to ANG_V and poisons the central-difference target velocity. Joint wiggle in
# the 100 ms after a flip measured 1.9-2.6x the rest of the episode (5 of 6 arm-runs).
# This is a REPRESENTATION BUG, not command smoothing -- the tangent form tracks the same
# commanded orientation, it just stops the coordinates from jumping.
FOLLOWER_ROT = os.environ.get("FOLLOWER_ROT", "tangent").lower()
# RAINBOW CONTROL BOX TRANSPORT DELAY, in 2 ms servo ticks, applied to the JOINT command on
# its way to the drives. This models the one stage the rig has never had: on hardware the
# host streams move_servo_j into the box and the box replays it from a FIFO.
# Measured on the deployed firmware (omx_wiki rainbow-control-box-servo-j-latency-fw-v8-7-3,
# 334 s / 167,155 ticks, queue_sync target_fill 5, servo_alpha 10 = inner LPF OFF):
#     box delay = RBACK queue fill + 1 tick, exactly, both arms, all joints. Nothing else.
#     sent -> ref  8.17 / 8.03 tk (16.3 / 16.1 ms), a PURE DEAD TIME (residual 0.0001-0.003 deg)
#     ref -> actual 2.97 / 3.09 tk        end to end 11.14 / 11.13 tk (22.3 ms)
# So the box is a DELAY, not a filter -- `servo_t2_sec` is the controller hold time and is
# explicitly "not UR-style lookahead" (docs/servo_backend_contract.md). Modelling it as a
# blend would be command smoothing, which this rig forbids; modelling it as a delay is plant
# fidelity. The remaining ref->actual ~3 tk is already supplied by the PhysX drive, whose
# measured first-order lag is 7.7 ms (3.9 tk) against the box's 2.97-3.09 tk.
# 0 = off (every result before 2026-09-03 ran this way); 8 = the measured sent->ref stage.
BOX_DELAY_TICKS = int(os.environ.get("BOX_DELAY_TICKS", "0"))
# HOW THE JOINT COMMAND REACHES THE ROBOT.
#   drive     (default) PhysX position drive; q_actual is the dynamics solution and lags q_ref
#             by a first-order 7.7 ms (= kd/kp), and adds a near-Nyquist ring at 176-236 Hz.
#   kinematic the ARM joints are written straight into the articulation state every 2 ms, so
#             q_actual == q_ref exactly and no arm dynamics are solved at all.
# `kinematic` exists to settle one question and only one: is the visible shake made by the
# dynamics solve, or is it already in the command? It is a DIAGNOSTIC, not an eval mode --
# teleporting the arm means the links no longer push back on anything they touch, so grasping
# and every contact number from such a run are meaningless. The FINGERS deliberately stay on
# the drive so the jaw still stalls on a bolt instead of scything through it.
DRIVE_MODE = os.environ.get("DRIVE_MODE", "drive").lower()
# VELPROPRIO HISTORY vs CHUNK ANCHOR. This rig kept ONE list (`cmd_hist`) for two jobs: the
# emitted-command history the policy's velocity proprio is differenced from, AND the slot the
# chunk anchor is written into at every boundary. The anchor write therefore replaced the last
# emitted knot with FK(q_cmd), so the tick after each boundary fed the policy
# `knot - FK(q_cmd)` instead of `knot - knot`. Measured 2026-09-03: the injected error is
# 14-32% of a typical per-tick displacement, once per chunk boundary -- 7.35 Hz at execute 4,
# which is exactly where the measured command wiggle peaks (7.41 Hz).
# The robot keeps them apart: openpi_remote._record_command_pose_history appends the emitted
# TcpPoseTarget only (ZOH on hold ticks) and the velproprio takes a time-based one-step
# lookback from THAT deque; the anchor never touches it.
# 1 = the historical (shared) behaviour, for A/B. 0 = separated, matching the robot.
VELPROPRIO_ANCHOR_OVERWRITE = os.environ.get("VELPROPRIO_ANCHOR_OVERWRITE", "0") == "1"
# FOLLOWER OUTPUT SMD -- port of rb_servo_server/src/control/follower_output_smd.cpp, the one
# stage of the deployed controller this rig never had. It sits between the follower's 500 Hz
# emitted pose and the IK, runs continuously (no chunk-boundary state: a per-chunk FIR was
# tried on hardware and made a 5.74 Hz boundary comb), and is a second-order critically damped
# tracker with LOW-PASSED velocity feedforward. With zeta=1 and w_lpf=wn:
#     H(s) = wn^2 (3s + wn) / (s + wn)^3
# The feedforward cancels first-order lag, so it is NOT a plain low-pass: 0.5-3 Hz passes at
# 1.06-1.30x and attenuation only starts above ~4.3 Hz. Hardware acceptance 2026-07-31
# (offline replay of a recorded trembling knot stream): accel power attenuation 13-20 Hz
# 89-109x, 10-13 Hz 22-31x, 5-10 Hz 4.5-5.1x, task band 1-5 Hz kept at 1.15x, path deviation
# p50 0.42 / p95 3.5 / max 9.4 mm -> enabled on the robot ever since.
# Defaults are stack_real.yaml's own (nf 3.5 / 2.5 Hz, zeta 1.0, ff on, ff_lpf 0 = follow nf).
# OFF by default here so no historical eval number moves without being asked for.
# ON by default since 2026-09-03: `output_smd.enable: true` on the robot, operator decision
# to run the same controller here. OUTPUT_SMD=0 restores the pre-port rig for A/B.
OUTPUT_SMD = os.environ.get("OUTPUT_SMD", "1") == "1"
SMD_NF_LIN = float(os.environ.get("SMD_NF_LINEAR_HZ", "3.5"))
SMD_NF_ANG = float(os.environ.get("SMD_NF_ANGULAR_HZ", "2.5"))
SMD_ZETA = float(os.environ.get("SMD_DAMPING_RATIO", "1.0"))
SMD_FF = os.environ.get("SMD_VELOCITY_FF", "1") == "1"
SMD_FF_LPF = float(os.environ.get("SMD_VELOCITY_FF_LPF_HZ", "0"))
# ACCELERATION FEEDFORWARD. cartesian_chunk_follower hands its BVP the central-difference
# af of the flanking knots, damped by af_damping_beta_{lin,ang} (both 1.0 on the RB3 profile).
# This rig passed NO target acceleration at all, i.e. af_target = 0 at every knot -- the exact
# analogue of the target_velocity=0 bug fixed on 2026-08-18, one derivative up. stack_real.yaml
# spells out why zero is not the safe default: "the cost of a segment is not |af|, it is
# |af_target - a0| / dt -- the jerk needed to swing acceleration from the chained current state
# to the target. a0 already carries the honest curvature, so shrinking af_target ENLARGES that
# mismatch and spends jerk forcing the arm to arrive flatter than it is actually travelling.
# af is a boundary condition, not a demand; a smaller one is not a cheaper one."
AF_BETA_LIN = float(os.environ.get("AF_BETA_LIN", "1.0"))
AF_BETA_ANG = float(os.environ.get("AF_BETA_ANG", "1.0"))
# CORNER (direction-reversal) RING-DOWN GUARD, chunk_follower_core.hpp:276 --
#     if sign(d_k) and sign(d_kp1) are both non-zero and DIFFER on an axis, vf[axis] *= scale
# with a per-class deadband so a flanking step below it contributes sign 0 and cannot form a
# reversal pair. Values are stack_real.yaml's (RB3 profile): 0.3 mm / 0.0005 rad / 0.25.
# Ported 2026-09-03 on the operator's "match the deployed controller even at the cost of
# success rate" decision. It is NOT a sim-side damping guard invented to hide tremor -- that
# is what CLAUDE.md forbids -- it is a stage the robot runs on every segment.
# ---- the obstruction layers the robot runs and this rig did not -------------------------
# Ported 2026-09-05 after the RB5 cell showed both arms commanded ~150 mm THROUGH the table
# (right arm z -0.288 -> -0.438 in one second, tracking error 212 mm) while the operator
# reports the same checkpoint pick-and-placing cleanly on hardware. Nothing was wrong with
# PhysX: the rig simply had no way to stop asking for a pose the arm cannot reach, and with
# `chunk_anchor=command` the anchor sinks with the command, so the runaway feeds itself.
# rb_servo_server has three layers between the plan and the drives; none existed here.
#
# 1. ROI BOX + REACH SHELL (safety.roi_box / safety.reach_constraint). Hard geometric bounds
#    on the commanded TCP, enforced with a velocity damper on the real robot. Values are
#    stack_real.yaml's, in the stand frame.
# 0. LEAD CLAMP -- the one the delta_preview follower actually runs, and the simplest.
#    `delta_twist_follower.cpp:491` scaleLinearTo(&lead, cfg_.max_lead_m) with
#    stack_real.yaml `delta_twist_max_lead_m: 0.020` / `delta_twist_max_lead_rad: 0.060`:
#    THE PLAN MAY NEVER BE MORE THAN 20 mm AHEAD OF THE ROBOT. That single bound is why a
#    real command cannot wind 150 mm into the table, and why the real anchor cannot ratchet
#    -- with `chunk_anchor=command` the anchor is FK(q_sent), so bounding the sent command to
#    the arm bounds the anchor to the arm too. This rig had no such bound: the operator saw
#    the tool descend-rise-descend and walk DOWN a step each chunk (2026-09-05), which is
#    exactly an unbounded lead being re-anchored once per boundary.
LEAD_CLAMP_ENABLE = os.environ.get("LEAD_CLAMP_ENABLE", "1") == "1"
MAX_LEAD_M = float(os.environ.get("DELTA_TWIST_MAX_LEAD_M", "0.020"))
MAX_LEAD_RAD = float(os.environ.get("DELTA_TWIST_MAX_LEAD_RAD", "0.060"))
ROI_ENABLE = os.environ.get("ROI_ENABLE", "1") == "1"
ROI_MIN = np.array([0.300, -0.400, -0.400])
ROI_MAX = np.array([1.100, 0.400, 0.590])
REACH_R_MIN = float(os.environ.get("REACH_R_MIN", "0.175"))
REACH_R_MAX = float(os.environ.get("REACH_R_MAX", "1.150"))
# 2. CONTACT HOLD-BACK (cartesian_chunk_follower.cpp:313-331). Whatever the projection above
#    removed is a direction the plan must stop advancing along. The real follower projects the
#    plan advance onto that direction and subtracts the blocked component, bounded to one
#    segment of travel at the linear velocity limit so a large removal cannot encode stale lag
#    as fresh contact.
HOLDBACK_ENABLE = os.environ.get("HOLDBACK_ENABLE", "1") == "1"
# 3. PLAN GATE (safety.plan_gate, plan_gate.hpp planGateStep). A first-order gate on the PLAN
#    CLOCK: the ratio of the realized joint step projected onto the requested one,
#    (r . q)/|q|^2, low-passed with separate attack and release. While the arm is obstructed
#    the chunk stops flowing, so the command cannot outrun the robot. The gate input is
#    post-clamp / pre-projection on purpose -- only an obstruction may close it.
#    Constants are stack_real.yaml's.
PLAN_GATE_ENABLE = os.environ.get("PLAN_GATE_ENABLE", "1") == "1"
# WHAT CLOSES THE GATE. On the robot `setAdvanceGate` is fed by the FORCE controller -- the
# gate is a contact gate, and the geometric layers (floor_constraint) are switched OFF there
# (`enable: false`, 2026-07-12, so the damper cannot alter bolt-pick proprioception). So the
# thing that stops a real command from winding 100 mm into the table is measured CONTACT, not
# a geometric face. This rig has no F/T sensor but it has the contact itself, and the honest
# reading of it is how much of the commanded joint step the arm actually realized.
#   contact     (default) requested = IK output, realized = measured joints. The physical
#               analogue of the force gate.
#   projection  requested vs post-ROI/reach command, i.e. the literal C++ input. Only an
#               explicit geometric face can close it -- which in this cell almost never
#               happens, because the ROI floor sits 105 mm BELOW the table.
PLAN_GATE_SOURCE = os.environ.get("PLAN_GATE_SOURCE", "contact").lower()
# ...AND WHAT MUST NOT CLOSE IT. The robot compares two COMMANDS, so its ratio has no lag
# term. Comparing a command to a MEASUREMENT does: the PhysX position drive trails by a
# first-order 7.7 ms (= kd/kp, measured), which at 170 deg/s is 0.34 deg/tick of perfectly
# healthy lag -- seven times the 0.05 deg deadband. Fed raw it closed the gate to 0.03 in
# free space and would have paced the whole task down. Compare the measurement against the
# command from one drive lag ago, which is the command it is actually chasing.
PLAN_GATE_LAG_SUBSTEPS = int(os.environ.get("PLAN_GATE_LAG_SUBSTEPS", "4"))
# PERSISTENCE. This is the whole design on the robot and it was learned the hard way there:
# feeding TRANSIENT shortfalls (the joint-limit barrier, the acceleration clamp) into the plan
# gate was measured and REVERTED -- "the barrier's job is to hold ONE joint at its standoff,
# so realized < requested is permanently true there while it works correctly", and pacing all
# six joints off that produced a 4.8 Hz ripple at 2.1-2.8x the baseline tremble. Only a
# SUSTAINED condition may pace the plan (ikThrottlePlanGateStep engages on a run of throttled
# ticks, measured at 150 consecutive).
# Measured here 2026-09-05 without it: the right arm's gate averaged 0.46, i.e. its chunk
# advanced at HALF speed for the whole episode, and the operator's read of the render was
# "much slower than real". Every tick of ordinary tracking shortfall was being charged as an
# obstruction. 25 substeps = 50 ms of continuous shortfall before the gate may close.
PLAN_GATE_PERSIST = int(os.environ.get("PLAN_GATE_PERSIST_SUBSTEPS", "25"))
PLAN_GATE_ENGAGE = float(os.environ.get("PLAN_GATE_ENGAGE_RATIO", "0.8"))
PLAN_GATE_ATTACK = float(os.environ.get("PLAN_GATE_ATTACK", "0.1"))
PLAN_GATE_RELEASE = float(os.environ.get("PLAN_GATE_RELEASE", "0.02"))
PLAN_GATE_DEADBAND = np.deg2rad(float(os.environ.get("PLAN_GATE_DEADBAND_DEG", "0.05")))
PLAN_GATE_MIN = float(os.environ.get("PLAN_GATE_MIN", "0.0"))
# 4. PROJECTION-ERROR FAULT (ruckig_follower.preview_max_projection_error_*). The robot gives
#    up loudly when the solve cannot reach the knot for N consecutive segments. This rig used
#    to diverge in silence, which is how a 200 mm tracking error reached a summary file.
PROJ_ERR_M = float(os.environ.get("PREVIEW_MAX_PROJECTION_ERROR_M", "0.002"))
PROJ_ERR_RAD = float(os.environ.get("PREVIEW_MAX_PROJECTION_ERROR_RAD", "0.00436332313"))
PROJ_ERR_N = int(os.environ.get("PREVIEW_MAX_CONSECUTIVE_PROJECTION_ERRORS", "12"))

CORNER_SCALE = float(os.environ.get("CORNER_VELOCITY_SCALE", "0.25"))
CORNER_DB_LIN = float(os.environ.get("CORNER_DEADBAND_LIN_M", "0.0003"))
CORNER_DB_ANG = float(os.environ.get("CORNER_DEADBAND_ANG_RAD", "0.0005"))

PHYSICS_DT = 1.0 / 500.0    # real servo rate
# rb3_730e.urdf gives every arm joint velocity=3.14159 rad/s. Clipping the IK step
# to that budget per substep is what stops the command outrunning the drives:
# the previous +-0.05 rad per 2 ms was 25 rad/s, ~8x the limit, so the joints
# saturated, tracking error grew, and the chunk-boundary re-anchor then jumped --
# which shows up as visible tremor (real hardware does not shake).
# Physical joint travel of the sim robot (rb3_730e.urdf). PhysX enforces these, so a
# command that runs past them cannot be reached: the joint pins at the stop while the
# IK keeps integrating, and the command-anchored chunk drifts away without bound.
# Measured before this clamp existed: left j1 spent 74% of an episode past +6.2832
# (up to 6.841) and the command ended 635 mm from the robot.
# rb_servo_server does the same thing in SafetyFilter::clampJointLimits (safety_filter.cpp:251):
#   out[i] = std::clamp(out[i], q_min_deg[i], q_max_deg[i])
# RB5-850E: elbow +-165 deg (2.879793 rad), the catalog bound robotics_lab's URDF generator
# enforces and `safety.q_max_deg[2]` mirrors. RB3 was +-150.
JOINT_LO = np.array([-6.28319, -6.28319, -2.879793, -6.28319, -6.28319, -6.28319])
JOINT_HI = -JOINT_LO
# stack_real.yaml dq_max_deg_s [170,170,170,240,240,320] -> rad/s, per joint
# stack_real.yaml safety.dq_max_deg_s. The RB5 profile flattened the wrist: it was
# [170,170,170,240,240,320] on RB3 and is [170]*6 now.
JOINT_VEL_LIMIT = np.deg2rad([170.0] * 6)
DQ_MAX = JOINT_VEL_LIMIT * PHYSICS_DT
# DLS damping. 1e-4 is the historical value every published number used. Near the boxes the
# Jacobian's smallest singular value drops to ~2e-4 and the arm visibly shakes and sheds bolts;
# a heavier damping trades tracking for calm there, so it is a knob, not a new default.
IK_LAMBDA = float(os.environ.get("IK_LAMBDA", "1e-4"))
# IK: the robot's own solver, from the RB3-730E profile (fad2cd4^ stack_real.yaml `kinematics.ik`)
#     damping 0.02  singular_region_eps 0.10  damping_max 0.08
#     max_iterations 100  min_iterations 1  position_tolerance 20 um  orientation_tolerance 0.0002 rad
#     max_step_deg [2,2,2,3,3,4]
# pinocchio_kinematics.cpp does SELECTIVE, SVD-based damped least squares:
#     dq = V diag(sigma / (sigma^2 + lambda_i^2)) U^T err
#     lambda_i^2 = damping^2, plus damping_max^2 * (1 - (sigma/eps)^2) on directions with
#                  sigma < eps
# so the inverse gain is capped on the degenerate direction only, leaving well-conditioned
# directions at full tracking accuracy. This rig instead used a UNIFORM lambda^2 = 1e-4 with no
# ramp, i.e. 4x less damping at baseline and, in the singular region, up to 68x less
# (0.02^2 + 0.08^2 = 6.8e-3 against 1e-4). Measured here 2026-09-03: sigma_min p10 is 0.07-0.16,
# so this arm lives AT and below the eps=0.10 threshold where the robot ramps and the rig did
# not. At sigma 0.03 the rig's inverse gain is 30.0 against the robot's 4.2 -- a 7x larger
# amplification of the same command noise into joint motion, which is exactly the measured
# symptom (0.4 deg of joint wiggle producing only 0.43 mm of tool wiggle).
# IK_MODE=legacy restores the uniform IK_LAMBDA solver for A/B.
IK_MODE = os.environ.get("IK_MODE", "real").lower()
IK_DAMPING = float(os.environ.get("IK_DAMPING", "0.02"))
IK_DAMPING_MAX = float(os.environ.get("IK_DAMPING_MAX", "0.08"))
IK_SINGULAR_EPS = float(os.environ.get("IK_SINGULAR_EPS", "0.10"))
IK_MAX_ITERS = int(os.environ.get("IK_MAX_ITERS", "100"))
# Once the target is this far away the solve is not going to converge inside the budget and
# grinding through 100 SVDs per substep only buys wall clock. Stand in for ik.timeout_ms.
IK_FAR_M = float(os.environ.get("IK_FAR_M", "0.05"))
IK_FAR_ITERS = int(os.environ.get("IK_FAR_ITERS", "8"))
IK_MIN_ITERS = int(os.environ.get("IK_MIN_ITERS", "1"))
IK_POS_TOL = float(os.environ.get("IK_POS_TOL_M", "0.00002"))
IK_ORI_TOL = float(os.environ.get("IK_ORI_TOL_RAD", "0.0002"))
IK_MAX_STEP = np.deg2rad(np.array([2.0, 2.0, 2.0, 3.0, 3.0, 4.0]))
# Single-arm oracle. The oracle's job has narrowed to characterising PICK physics; bimanual
# choreography (arm-arm interference, races at the pile midline) is task realism the POLICY
# must face but pure noise for a physics instrument. ORACLE_ARM=left parks the other arm at
# all-joints-zero -- verified by offline FK to point straight up (lowest link z=0.73 m, TCP at
# z=1.71 m), fully clear of the table and the working arm -- and the active arm services BOTH
# colours, placing each bolt into the box of its own colour.
ORACLE_ARM = os.environ.get("ORACLE_ARM", "").lower() or None
# RETURN-EPISODE COLLECTION. Demos are 8.2 s single-cycle recordings -- "place done, go back
# for the next bolt" exists nowhere in them, and rollouts die after ~10 s exactly there. This
# mode generates that missing bridge: both arms START at a jittered place pose over their own
# boxes (jaws open, the demonstrated RX tilt) and SEQUENTIALLY return to a hover over an
# isolated bolt at the demo approach-entry state. Speeds are capped inside the demo action
# distribution (pos p90 6.4 mm/step, rot p99 1.5 deg/step). Recording rides the existing
# EVAL_PROBE_DUMP path; scripts/return_probe_to_hdf5.py converts dumps to per-episode HDF5.
ORACLE_RETURN = os.environ.get("ORACLE_RETURN") == "1"
_RET_Q = {}   # per-episode teleported start joints (filled at reset, consumed at q_cmd init)
ORACLE_PARKED = ({"left": "right", "right": "left"}[ORACLE_ARM] if ORACLE_ARM else None)
DIAG = os.environ.get("TREMOR_DIAG") == "1"
# EVAL_DUMP_OBS=<dir> writes the exact wrist frames handed to the policy, every 30th tick.
# The observation path is the one thing an aggregate score cannot audit -- black or stale
# frames score 0 while looking like a policy result (see CLAUDE.md section 20).
DUMP_OBS = os.environ.get("EVAL_DUMP_OBS") or ""
# OBS_PREV_FRAMES=1 attaches the wrist frames from OBS_PREV_DELTA policy ticks ago as
# observation/{left,right}_wrist_prev_rgb (obs-2 models). History holds only frames grabbed at
# replan boundaries, so the match is nearest-tick, not exact -- error <= half the replan interval.
OBS_PREV = os.environ.get("OBS_PREV_FRAMES") == "1"
OBS_PREV_DELTA = int(os.environ.get("OBS_PREV_DELTA", "8"))
if DUMP_OBS:
    os.makedirs(DUMP_OBS, exist_ok=True)
# EVAL_PROBE_DUMP=<dir> writes a PERCEPTION-PROBE dataset: every PROBE_EVERY-th policy tick,
# the exact wrist frames plus ground truth no real log has -- TCP pose, the nearest bolts in the
# TOOL frame, and the policy's own chunk-endpoint aim. Purpose: decompose the closed-loop aim
# error into "the encoder cannot locate the bolt from this (drifted) viewpoint" versus "the
# features suffice but the action side loses precision" by fitting a probe from frozen encoder
# features to the tool-frame bolt position and comparing probe error with aim error per stratum.
PROBE_DUMP = os.environ.get("EVAL_PROBE_DUMP") or ""
PROBE_EVERY = int(os.environ.get("EVAL_PROBE_EVERY", "10"))
# EVAL_PROBE_NOIMG=1 keeps every field EXCEPT the wrist PNGs. The approach-profile questions --
# how far the plan travels as the gap closes, and where the jaw command is while it does -- are
# answered by `chunk`, `bolts_tool` and `grip`, all of which are already recorded; only offline
# RE-QUERYING of the policy needs pixels. Dropping them turns a 20-episode every-tick trace from
# ~36k PNGs into ~150 MB of JSON, which is what makes the right arm's approach measurable at all
# (the 4-episode image dump yielded 1-11 right-arm approach ticks per distance bucket).
PROBE_NOIMG = os.environ.get("EVAL_PROBE_NOIMG") == "1"
if PROBE_DUMP:
    os.makedirs(PROBE_DUMP, exist_ok=True)
# Replay drives the recorded joint-command stream verbatim, so the bolts-vs-no-bolts
# comparison differs ONLY by contact. The free-space run without this was useless:
# with no bolts the policy commanded unreachable poses (141-350 mm off), so the two
# conditions were not running the same trajectory at all.
REPLAY = os.environ.get("TREMOR_REPLAY")
_REP = np.load(REPLAY) if REPLAY else None
if _REP is not None:
    # Fail loudly at startup: a diag npz recorded before joint logging existed has no
    # *_q, and the KeyError deep in the loop left Isaac hung until the 1200 s timeout.
    _missing = [k for k in ("left_q", "left_gq", "right_q", "right_gq") if k not in _REP]
    if _missing:
        raise SystemExit(f"TREMOR_REPLAY={REPLAY} lacks {_missing}; re-record with TREMOR_DIAG=1")
IK_ITERS = 3        # DLS iterations per substep; one was not converging
SUBSTEPS = int(round(POLICY_DT / PHYSICS_DT))   # 17


def rotvec_to_mat(r):
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def mat_to_rotvec(R):
    # Quaternion route. The direct formula divides by sin(theta), which near pi amplified
    # numerical noise into INVALID rotvecs (|r| up to 4.33 > pi) in the per-tick dumps --
    # tool-down attitudes sit exactly at that singularity, and the 1e-6 special-case band was
    # far too narrow. Quaternion extraction is stable over the whole range.
    t = float(np.trace(R))
    if t > 0.0:
        s_ = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s_
        x = (R[2, 1] - R[1, 2]) / s_
        y = (R[0, 2] - R[2, 0]) / s_
        z = (R[1, 0] - R[0, 1]) / s_
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s_ = math.sqrt(max(1e-12, 1.0 + R[i, i] - R[j, j] - R[k, k])) * 2.0
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s_
        q[j] = (R[j, i] + R[i, j]) / s_
        q[k] = (R[k, i] + R[i, k]) / s_
        w = (R[k, j] - R[j, k]) / s_
        x, y, z = q
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return np.zeros(3)
    th = 2.0 * math.atan2(n, abs(w))
    sign = 1.0 if w >= 0.0 else -1.0
    return np.array([x, y, z]) * (sign * th / n)


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])



# ---- analytic arm kinematics ------------------------------------------------
# Taken straight from rb3_730e.urdf. Using the URDF chain rather than PhysX
# Jacobians because SingleArticulation exposes none (only the batched Articulation
# class does), and an analytic Jacobian is both exact and far cheaper at 500 Hz.
# Each entry is (origin_xyz, origin_rpy, axis); the joint rotation applies AFTER
# the origin transform. Verified at runtime against the physics TCP pose.
# RB5-850E, straight out of rb5_850e_pika_articulated.urdf. This is NOT an RB3 chain with
# new lengths -- the wrist convention differs: J4/J5/J6 axes go Z/Y/Z -> Y/Z/Y and the
# flange normal in link6 moves from +Z to -Y. Any attempt to port the RB3 chain by editing
# offsets produces an arm that looks right at zero and is wrong everywhere else.
URDF_CHAIN = [
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None),               # link0_fixed
    ((0.0, 0.0, 0.1692), (0.0, 0.0, 0.0), (0, 0, 1)),       # base_joint
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 1, 0)),          # shoulder_joint
    ((0.0, 0.0, 0.425), (0.0, 0.0, 0.0), (0, 1, 0)),        # elbow_joint
    ((0.0, 0.0, 0.392), (0.0, 0.0, 0.0), (0, 1, 0)),        # wrist1_joint
    ((0.0, -0.1107, 0.1107), (0.0, 0.0, 0.0), (0, 0, 1)),   # wrist2_joint
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0, 1, 0)),          # wrist3_joint
]
# link6 -> tcp. On RB3 this was a pure +z translation (0.100 + 0.247642) and the code below
# could add it as a scalar. On RB5 `attachment_site` is at link6 + (0, -0.0967, 0) with
# rpy (1.57, 0, 0) -- the flange faces -Y -- so the tool transform carries a ROTATION and
# has to be a full matrix. The Pika length itself is unchanged: attachment_site -> tcp is
# 0.247642 on both arms.
def _tool_xf():
    T = np.eye(4)
    T[:3, :3] = _rpy(1.57, 0.0, 0.0)
    T[:3, 3] = (0.0, -0.0967, 0.0)
    Tt = np.eye(4)
    Tt[2, 3] = 0.247642
    return T @ Tt


TOOL_XF = None      # built after _rpy is defined, below


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


TOOL_XF = _tool_xf()


def project_command(p, mount_p):
    """Clamp a commanded TCP into the ROI box and the reach shell.

    The robot's `safety.roi_box` and `safety.reach_constraint`, in the stand frame. On
    hardware these run a velocity damper into the face; here the clamp is hard, which is the
    conservative half -- it cannot let the command through, only stop it early.

    Returns `(clamped, removed)` where `removed = requested - clamped` is the direction the
    obstruction took away. That vector is what the contact hold-back and the plan gate below
    both consume, exactly as `into_contact_dir_` and the pre/post-projection pair do on the
    robot.
    """
    if not ROI_ENABLE:
        return np.asarray(p, dtype=float), np.zeros(3)
    q = np.clip(np.asarray(p, dtype=float), ROI_MIN, ROI_MAX)
    d = q - np.asarray(mount_p, dtype=float)
    r = float(np.linalg.norm(d))
    if r > 1e-9:
        r_cl = min(max(r, REACH_R_MIN), REACH_R_MAX)
        if r_cl != r:
            q = np.asarray(mount_p, dtype=float) + d * (r_cl / r)
    return q, np.asarray(p, dtype=float) - q


def plan_gate_step(gate, requested_q, final_q, prev_q, persist_ok=True):
    """One tick of rb_servo_server's planGateStep (control/plan_gate.hpp).

    Ratio is the realized step PROJECTED onto the requested one, (r.q)/|q|^2 -- not a ratio
    of two independent per-joint maxima, which credited J1's request against J4's realization
    and read a preserved tangential slide as a full block. Attack and release never run on
    the same tick. The deadband is on the Euclidean step norm so idle and hold ticks cannot
    drive the gate.
    """
    req = np.asarray(requested_q) - np.asarray(prev_q)
    got = np.asarray(final_q) - np.asarray(prev_q)
    req_sq = float(req @ req)
    if math.sqrt(req_sq) > PLAN_GATE_DEADBAND and req_sq > 1e-18:
        inst = min(max(float(got @ req) / req_sq, PLAN_GATE_MIN), 1.0)
        if inst < gate and persist_ok:
            return gate + PLAN_GATE_ATTACK * (inst - gate)
    return gate + PLAN_GATE_RELEASE * (1.0 - gate)


def plan_gate_ratio(requested_q, final_q, prev_q):
    """The instantaneous realized/requested ratio, for the persistence counter."""
    req = np.asarray(requested_q) - np.asarray(prev_q)
    got = np.asarray(final_q) - np.asarray(prev_q)
    req_sq = float(req @ req)
    if math.sqrt(req_sq) <= PLAN_GATE_DEADBAND or req_sq <= 1e-18:
        return 1.0
    return min(max(float(got @ req) / req_sq, 0.0), 1.0)


def _axis_rot(axis, q):
    return rotvec_to_mat(np.array(axis, dtype=float) * q)


def fk_chain(q, T_mount):
    """Return (tcp_pos, tcp_R, joint_origins, joint_axes) in world."""
    T = np.array(T_mount, dtype=float)
    origins, axes = [], []
    qi = 0
    for xyz, rpy, ax in URDF_CHAIN:
        M = np.eye(4)
        M[:3, :3] = _rpy(*rpy)
        M[:3, 3] = xyz
        T = T @ M
        if ax is not None:
            origins.append(T[:3, 3].copy())
            axes.append((T[:3, :3] @ np.array(ax, dtype=float)).copy())
            R = np.eye(4)
            R[:3, :3] = _axis_rot(ax, q[qi])
            T = T @ R
            qi += 1
    T = T @ TOOL_XF
    return T[:3, 3].copy(), T[:3, :3].copy(), origins, axes


def jacobian(q, T_mount):
    p_e, _, origins, axes = fk_chain(q, T_mount)
    J = np.zeros((6, 6))
    for i, (o, a) in enumerate(zip(origins, axes)):
        J[:3, i] = np.cross(a, p_e - o)
        J[3:, i] = a
    return J


class RuckigArmFollower:
    """Jerk-limited 6-DOF Cartesian follower, one per arm.

    Mirrors rb_servo_server's cartesian_chunk_follower: a Ruckig p/v/a chain drives the
    Cartesian reference toward the active knot, re-targeted at each 33 ms boundary.
    Orientation is carried as a rotvec so the per-axis angular limits apply the same way
    the C++ AxisLimit does.
    """

    def __init__(self, pose0):
        from ruckig import InputParameter, OutputParameter, Ruckig

        self.otg = Ruckig(6, PHYSICS_DT)
        self.inp = InputParameter(6)
        self.out = OutputParameter(6)
        self.inp.max_velocity = [LIN_V] * 3 + [ANG_V] * 3
        self.inp.max_acceleration = [LIN_A] * 3 + [ANG_A] * 3
        self.inp.max_jerk = [LIN_J] * 3 + [ANG_J] * 3
        self.tangent = FOLLOWER_ROT == "tangent"
        # R0: the quaternion base of the orientation tangent (carried as a matrix; the
        # rig has no quaternion type and a matrix is equally free of the pi wrap).
        self.R0 = rotvec_to_mat(np.asarray(pose0)[3:]) if self.tangent else None
        p0 = list(pose0[:3]) + [0.0, 0.0, 0.0] if self.tangent else list(pose0)
        self.inp.current_position = list(p0)
        self.inp.current_velocity = [0.0] * 6
        self.inp.current_acceleration = [0.0] * 6
        self.inp.target_position = list(p0)
        self.inp.target_velocity = [0.0] * 6
        self.reference = np.array(pose0, dtype=float)
        self.reference_velocity = np.zeros(6)

    def _relinearize(self):
        """Roll the accumulated tangent into R0 and reset axes 3-5 to zero.

        cartesian_chunk_follower.cpp does this in stepToNextSegment(), BEFORE each new
        solve, so the tangent never grows past the few degrees one knot moves. Velocity and
        acceleration carry through unchanged -- the same small-angle transfer the C++ relies
        on -- so the re-base is not a velocity step.
        """
        th = np.asarray(self.inp.current_position[3:6], dtype=float)
        if float(np.linalg.norm(th)) > 0.0:
            self.R0 = self.R0 @ rotvec_to_mat(th)
            cp = list(self.inp.current_position)
            cp[3], cp[4], cp[5] = 0.0, 0.0, 0.0
            self.inp.current_position = cp

    def _to_tangent(self, pose):
        return mat_to_rotvec(self.R0.T @ rotvec_to_mat(np.asarray(pose)[3:]))

    def set_target(self, pose, vel=None, neighbors=None, span_dt=None, acc=None):
        """Target a knot WITH a velocity.

        chunk_window.hpp keeps `reserve_R` steps of lookahead precisely so each knot can
        be given a central-difference target velocity. Leaving target_velocity at zero
        makes Ruckig decelerate to a full stop at every 33 ms knot: the arm accelerates
        and brakes 30 times a second, which is exactly the tremor the real robot does not
        have (it flows through knots at speed).

        `neighbors` = the (lo, hi) knots the caller central-differenced for `vel`. In
        tangent mode the ANGULAR half of that difference has to be retaken in the current
        tangent, because the caller computed it on absolute rotvecs that flip sign at the
        pi boundary -- a flip there produces a bogus 2*pi/dt angular target, clipped to
        ANG_V, i.e. a full-speed slew in the wrong direction.
        """
        tgt = np.asarray(pose, dtype=float)
        if self.tangent:
            self._relinearize()
            self.inp.target_position = list(tgt[:3]) + list(self._to_tangent(tgt))
        else:
            self.inp.target_position = [float(v) for v in tgt]
        if vel is None or os.environ.get("TREMOR_BASELINE") == "1":
            self.inp.target_velocity = [0.0] * 6   # old behaviour: stop at every knot
            self.inp.target_acceleration = [0.0] * 6
            return
        tv = np.asarray(vel, dtype=float).copy()
        d_k = d_kp1 = None
        if neighbors is not None and span_dt:
            half = 0.5 * float(span_dt)          # one policy step of the three-point stencil
            lo6 = np.asarray(neighbors[0], dtype=float).copy()
            hi6 = np.asarray(neighbors[1], dtype=float).copy()
            if self.tangent:
                # rotation differences must be taken in the current tangent: an absolute-rotvec
                # difference straddling the pi flip is garbage, and it would both corrupt vf and
                # fabricate a sign reversal for the corner guard below.
                lo6[3:] = self._to_tangent(neighbors[0])
                hi6[3:] = self._to_tangent(neighbors[1])
                mid = np.concatenate([tgt[:3], self._to_tangent(tgt)])
            else:
                mid = tgt
            d_k, d_kp1 = mid - lo6, hi6 - mid
            tv = (d_k + d_kp1) / (2.0 * half)
            # CORNER GUARD (chunk_follower_core.hpp:276): ring down the target velocity on any
            # axis whose two flanking steps genuinely reverse direction.
            db = np.array([CORNER_DB_LIN] * 3 + [CORNER_DB_ANG] * 3)
            s_k = np.sign(np.where(np.abs(d_k) > db, d_k, 0.0))
            s_p = np.sign(np.where(np.abs(d_kp1) > db, d_kp1, 0.0))
            rev = (s_k != 0) & (s_p != 0) & (s_k != s_p)
            tv = np.where(rev, tv * CORNER_SCALE, tv)
        tv[:3] = np.clip(tv[:3], -LIN_V, LIN_V)
        tv[3:] = np.clip(tv[3:], -ANG_V, ANG_V)
        self.inp.target_velocity = [float(v) for v in tv]
        # af is a BOUNDARY CONDITION on the segment, not a demand (stack_real.yaml). In tangent
        # mode the angular half is retaken in the current tangent for the same reason the
        # velocity is: an absolute-rotvec second difference straddling the pi flip is garbage.
        if acc is None:
            self.inp.target_acceleration = [0.0] * 6
            return
        ta = np.asarray(acc, dtype=float).copy()
        if d_k is not None:
            ta = (d_kp1 - d_k) / ((0.5 * float(span_dt)) ** 2)
        ta[:3] = np.clip(ta[:3] * AF_BETA_LIN, -LIN_A, LIN_A)
        ta[3:] = np.clip(ta[3:] * AF_BETA_ANG, -ANG_A, ANG_A)
        self.inp.target_acceleration = [float(v) for v in ta]

    def step(self):
        from ruckig import Result

        # Ruckig raises on a strictly degenerate request: target == current to the last bit,
        # with zero velocity, gives "error in step 2 ... for t sync: 0.000000". A policy never
        # produces that (its knots always move a little), but a scripted planner that holds
        # still does, and it killed the first oracle run. Holding the reference is the correct
        # answer for a zero-length trajectory anyway.
        # 1 nm / 1 nrad is far below anything the arm can express, so treating it as "already
        # there" cannot change behaviour -- but it does cover the whole family of static-hold
        # requests a scripted planner emits, which is what crashed the first two oracle runs.
        if (np.allclose(self.inp.target_position, self.inp.current_position, atol=1e-9)
                and np.allclose(self.inp.current_velocity, 0.0, atol=1e-7)
                and np.allclose(self.inp.target_velocity, 0.0, atol=1e-7)):
            self.reference = self._as_absolute(self.inp.current_position)
            self.reference_velocity = np.zeros(6)
            return True

        res = self.otg.update(self.inp, self.out)
        self.out.pass_to_input(self.inp)
        self.reference = self._as_absolute(self.out.new_position)
        # xi_ref for the output SMD: the follower's own chained velocity, exactly what
        # dual_arm_servo_loop hands FollowerOutputSmd::step (core_.v0()). In tangent mode
        # channels 3-5 are about R0, which to first order is the body frame the SMD
        # integrates in -- the same approximation the C++ makes.
        self.reference_velocity = np.asarray(self.out.new_velocity, dtype=float).copy()
        return res in (Result.Working, Result.Finished)

    def _as_absolute(self, p):
        """The IK consumes an ABSOLUTE pose, so map the tangent back out (tangentPose())."""
        p = np.asarray(p, dtype=float)
        if not self.tangent:
            return p.copy()
        return np.concatenate([p[:3], mat_to_rotvec(self.R0 @ rotvec_to_mat(p[3:6]))])


class FollowerOutputSmd:
    """Port of rb_servo_server/src/control/follower_output_smd.cpp.

    Continuous 500 Hz post-follower pose conditioner: no command-goal integrator, no goal
    lead, no chunk-boundary concept. The follower's emitted pose is the reference every tick
    and all follower bookkeeping stays on the pre-filter stream.
    """

    RESEED_POS_M = 0.05
    RESEED_ROT_RAD = 0.10

    def __init__(self, nf_lin, nf_ang, zeta, ff, ff_lpf_hz):
        self.wn_lin = 2.0 * math.pi * nf_lin
        self.wn_ang = 2.0 * math.pi * nf_ang
        self.zeta = zeta
        self.ff = ff
        self.ff_lin_hz = ff_lpf_hz if ff_lpf_hz > 0.0 else nf_lin
        self.ff_ang_hz = ff_lpf_hz if ff_lpf_hz > 0.0 else nf_ang
        self.active = False

    def reset(self, pose, xi):
        pose = np.asarray(pose, dtype=float)
        xi = np.asarray(xi, dtype=float)
        self.p = pose[:3].copy()
        self.R = rotvec_to_mat(pose[3:])
        self.v = xi[:3].copy()
        self.w = xi[3:].copy()
        self.v_ff = self.v.copy()
        self.w_ff = self.w.copy()
        self.active = True

    def _pose(self):
        return np.concatenate([self.p, mat_to_rotvec(self.R)])

    def step(self, reference, xi_ref, dt):
        ref = np.asarray(reference, dtype=float)
        xi = np.asarray(xi_ref, dtype=float)
        if not self.active:
            self.reset(ref, xi)
            return self._pose()
        R_ref = rotvec_to_mat(ref[3:])
        e_rot = mat_to_rotvec(self.R.T @ R_ref)
        # A stale output state must never pull the command back toward an old pose: snap to
        # the live pre-filter reference and inherit its chained velocity.
        if (np.linalg.norm(ref[:3] - self.p) > self.RESEED_POS_M
                or np.linalg.norm(e_rot) > self.RESEED_ROT_RAD):
            self.reset(ref, xi)
            return self._pose()
        v_d, w_d = np.zeros(3), np.zeros(3)
        if self.ff:
            self.v_ff += 2.0 * math.pi * self.ff_lin_hz * (xi[:3] - self.v_ff) * dt
            self.w_ff += 2.0 * math.pi * self.ff_ang_hz * (xi[3:] - self.w_ff) * dt
            v_d, w_d = self.v_ff, self.w_ff
        a = (self.wn_lin ** 2) * (ref[:3] - self.p) + 2.0 * self.zeta * self.wn_lin * (v_d - self.v)
        self.v = self.v + a * dt
        self.p = self.p + self.v * dt
        al = (self.wn_ang ** 2) * e_rot + 2.0 * self.zeta * self.wn_ang * (w_d - self.w)
        self.w = self.w + al * dt
        self.R = self.R @ rotvec_to_mat(self.w * dt)
        return self._pose()


def bolt_poses(layout, n_per, seed):
    if 'size_m' in WORK_SURFACE:
        return work_surface.placed_bolts(WORK_SURFACE, layout, n_per, seed, PILE_X, PILE_DY,
                                         float(os.environ.get('BOLT_SPREAD', '1.0')))
    rng = np.random.default_rng(seed)
    out = []
    spread = float(os.environ.get("BOLT_SPREAD", "1.0"))
    if layout == "aligned":
        for color, sy in (("gray", +1.0), ("black", -1.0)):
            for _ in range(n_per):
                # BOLT_SPREAD scales the pile sigma. At 1.0 the piles match the real cell's
                # density (same-colour nearest-neighbour gap ~38 mm, well inside the 110 mm
                # jaw sweep) -- correct for POLICY scoring, but for grasp-physics calibration
                # it entangles contact parameters with neighbour interference. Sparse layouts
                # are an instrument setting only; std20/std40 stay at 1.0.
                out.append((color, PILE_X + rng.normal(0, 0.045 * spread),
                            sy * PILE_DY + rng.normal(0, 0.05 * spread), rng.uniform(0, math.pi)))
    else:
        colors = ["gray"] * n_per + ["black"] * n_per
        rng.shuffle(colors)
        for color in colors:
            out.append((color, PILE_X + rng.normal(0, 0.07),
                        rng.uniform(-0.30, 0.30), rng.uniform(0, math.pi)))
    return out


def _resolve_served_checkpoint(host: str, port: int) -> dict:
    """Recover WHICH checkpoint answered this run by reading the local server's cmdline.

    The websocket metadata carries only {action_horizon, action_dim} -- nothing identifying -- so
    every summary written so far is unattributable to a checkpoint. Scanning /proc is the only
    source that cannot drift from what actually served. Remote hosts are unresolvable; say so
    rather than leaving the field silently empty.
    """
    if host not in ("127.0.0.1", "localhost", "::1"):
        return {"resolved": False, "why": f"non-local host {host}"}
    hits = []
    for pd in pathlib.Path("/proc").iterdir():
        if not pd.name.isdigit():
            continue
        try:
            argv = (pd / "cmdline").read_bytes().decode("utf-8", "replace").split("\0")
        except OSError:
            continue
        if not any("serve_policy" in a for a in argv):
            continue
        if str(port) not in argv:
            continue
        got = {"pid": int(pd.name), "argv": [a for a in argv if a]}
        for flag, key in (("--policy.dir", "dir"), ("--policy.config", "config"),
                          ("--num-medoid-samples", "num_medoid_samples")):
            if flag in argv:
                got[key] = argv[argv.index(flag) + 1]
        hits.append(got)
    if len(hits) != 1:
        return {"resolved": False, "why": f"{len(hits)} serve_policy processes matched port {port}",
                "candidates": hits}
    return {"resolved": True, **hits[0]}


def _safe_provenance(args, server_metadata: dict | None) -> dict:
    """Never let provenance collection destroy a finished run.

    Learned the hard way on 2026-09-02: a NameError in here killed the summary write of three
    completed 20-episode A/B runs (~2.5 h of simulation) AFTER every episode had been scored. The
    numbers are the product; the metadata about them is not worth risking them for.
    """
    try:
        return _provenance(args, server_metadata)
    except Exception as e:  # noqa: BLE001 - deliberately broad; this must not raise
        return {"error": f"{type(e).__name__}: {e}"}


def _provenance(args, server_metadata: dict | None) -> dict:
    """Everything needed to re-run this exact configuration, recorded WITH the numbers.

    Written because the 2026-08 summaries cannot be compared: `DC_*` reports dz p50 ~140 mm and
    `G23*` ~20 mm, and nothing on disk says which knobs, rig revision, or checkpoint produced
    either. A number without its knobs is not a measurement.
    """
    import hashlib
    src = pathlib.Path(__file__).read_bytes()
    prefixes = ("GRIP_", "BOLT_", "FINGER_", "ORACLE_", "EVAL_", "TREMOR_", "RTC_", "BOX_",
                "SMD_", "AF_", "IK_", "LIN_", "ANG_", "CORNER_")
    names = ("STATE_MODE", "ACTION_MODE", "IK_LAMBDA", "CUDA_VISIBLE_DEVICES", "OMNI_KIT_ACCEPT_EULA",
             "FOLLOWER_ROT", "DRIVE_MODE", "OUTPUT_SMD", "VELPROPRIO_ANCHOR_OVERWRITE",
             "PIKA_TIP")
    return {
        "rig": {
            "script": str(pathlib.Path(__file__).resolve()),
            "sha256": hashlib.sha256(src).hexdigest()[:16],
            "argv": sys.argv[1:],
        },
        "policy_server": {
            "host": args.host, "port": args.port,
            "metadata": server_metadata,
            "checkpoint": _resolve_served_checkpoint(args.host, args.port),
        },
        # Vars the operator SET (verbatim) ...
        "env": {k: v for k, v in sorted(os.environ.items())
                if k.startswith(prefixes) or k in names},
        # ... and the values that actually APPLIED, defaults included.
        "effective": {
            "work_surface": WORK_SURFACE,
            "pick_surface_z_m": PICK_SURFACE_Z,
            "H_APERTURE": H_APERTURE, "V_APERTURE": V_APERTURE, "PP_PX": PP_PX,
            "PHOTOMETRY": photometry_metadata(),
            "fx_px": round(11.0 / H_APERTURE * 640, 2),
            "GRIP_PROPRIO": GRIP_PROPRIO, "GRIP_LEAD": GRIP_LEAD, "GRIP_BIAS": GRIP_BIAS,
            "GRIP_LAG_MS": GRIP_LAG_MS, "GRIP_FLOOR": GRIP_FLOOR,
            "STATE_MODE": STATE_MODE, "ACTION_MODE": ACTION_MODE, "IK_LAMBDA": IK_LAMBDA,
            "FOLLOWER_ROT": FOLLOWER_ROT, "BOX_DELAY_TICKS": BOX_DELAY_TICKS,
            "DRIVE_MODE": DRIVE_MODE, "OUTPUT_SMD": OUTPUT_SMD,
            "SMD": {"nf_lin": SMD_NF_LIN, "nf_ang": SMD_NF_ANG, "zeta": SMD_ZETA,
                    "ff": SMD_FF, "ff_lpf": SMD_FF_LPF},
            "VELPROPRIO_ANCHOR_OVERWRITE": VELPROPRIO_ANCHOR_OVERWRITE,
            "AF_BETA": {"lin": AF_BETA_LIN, "ang": AF_BETA_ANG},
            "CORNER": {"scale": CORNER_SCALE, "db_lin": CORNER_DB_LIN, "db_ang": CORNER_DB_ANG},
            "IK": {"mode": IK_MODE, "damping": IK_DAMPING, "damping_max": IK_DAMPING_MAX,
                   "eps": IK_SINGULAR_EPS, "max_iters": IK_MAX_ITERS,
                   "pos_tol_m": IK_POS_TOL, "ori_tol_rad": IK_ORI_TOL},
            "LIMITS": {"lin_v": LIN_V, "lin_a": LIN_A, "lin_j": LIN_J,
                       "ang_v": ANG_V, "ang_a": ANG_A, "ang_j": ANG_J},
            "ORACLE_ARM": ORACLE_ARM, "ORACLE_RETURN": ORACLE_RETURN,
            # WHICH GRIPPER WAS SCORED. Two runs on different fingertips otherwise look
            # identical on disk, and the v15 swap moves the jaw stroke (0.047 -> 0.049),
            # the collider (flat hull -> arched SDF) and what the wrist camera sees. The
            # asset path is recorded, not just the flag, because the flag only names a
            # directory that a rebuild can change underneath it.
            # What the SERVED checkpoint was trained under, next to what the rig decoded
            # it as. A summary that records only the latter cannot be audited.
            "checkpoint_contract": _CONTRACT,
            "PIKA_TIP": PIKA_TIP,
            "ARM_USD": str(ARM_USD),
            "FINGER_TRAVEL_M": FINGER_TRAVEL_M,
            # rtc / execute_steps / prefetch_at are recorded as top-level summary fields;
            # PREFETCH_AT in particular is a local of main(), not a module global.
            # Grasp accounting changed on 2026-09-02 (the detector fix). Summaries without this
            # marker undercount grasps and must not be compared against ones that have it.
            "grasp_detector": "fixed_20260902",
        },
    }


def main() -> int:
    global RTC_ENABLED, RTC_INFERENCE_DELAY, CHUNK_EXECUTE_STEPS
    global WORK_SURFACE, PICK_SURFACE_Z
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--shared-stack", action="store_true",
                    help="use robotics_lab policy_runner and full C++ servo loop (one episode)")
    ap.add_argument("--shared-config", default=str(ROOT / "config/shared_stack.json"))
    ap.add_argument("--shared-replay", help="Offline policy-input log directory; shared controller and F/T stay live")
    ap.add_argument("--shared-hold", action="store_true",
                    help="validate the shared PhysX plant with Hold, without policy inference")
    ap.add_argument("--layout", choices=["aligned", "random"], default="aligned")
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--n-per-color", type=int, default=10,
                    help="bolts per colour; the real cell has ~20 bolts piled up")
    ap.add_argument("--episode-sec", type=float, default=40.0)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--video", action="store_true", default=True)
    ap.add_argument("--no-video", dest="video", action="store_false")
    ap.add_argument("--tag", default="ep")
    # Deploy-runner timing/RTC knobs (flow_infer_sweep_run.sh equivalents). The runner
    # KICKS the next inference at chunk step `prefetch_at` and SWAPS at the execute
    # boundary, dropping the rows that elapsed since the observation (realized_delay =
    # execute - prefetch_at). Emulating that here is what makes the sim comparable to
    # the robot: a synchronous infer() at the boundary would give every executed row a
    # freshness the real robot never has.
    # RTC OFF by default since 2026-09-05: the operator's own rollout command for these
    # checkpoints carries FLOW_INFER_RTC=0, and the .meta of every 2026-09-04 boltv2 rollout
    # in robotics_lab/outputs/sweep agrees. The rig defaulting it on was scoring a different
    # deployment than the robot runs. --rtc still turns it on for an A/B.
    ap.add_argument("--rtc", action="store_true", default=False,
                    help="FLOW_INFER_RTC=1: freeze the first (execute-prefetch_at) rows to the "
                         "previous plan and inpaint the rest (server-side, zeros schedule)")
    ap.add_argument("--execute-steps", type=int, default=CHUNK_EXECUTE_STEPS)
    ap.add_argument("--prefetch-at", type=int, default=0)
    # THE fixed scoring set. Every model comparison is run under --protocol std20 so that two
    # arms can never differ by an evaluation knob; only the policy endpoint and the layout axis
    # may change. Video is ON even though it roughly doubles wall clock: recording changes the
    # number of rep.orchestrator.step() calls per tick, and an orchestrator step advances the
    # timeline (section 19's mechanism). Measured 2026-08-19 on seed 104, same code: video ON
    # reproduced the historical runs (2 placements, close |dxy| p50 10.6 mm) while video OFF did
    # not (0 placements, 260 mm) even with verified-correct wrist observations. Every validated
    # number this project has came from a video-ON run; the protocol keeps it that way.
    ap.add_argument("--protocol", choices=["std20", "std40", "free"], default="free",
                    help="std20 = fixed eval set: 20 episodes, seeds 100..119, 30 s each, "
                         "10 bolts per colour, video on. Overrides the knobs it pins.")
    ap.add_argument("--label", default=None,
                    help="model name recorded in the summary (defaults to --tag)")
    # PhysX settling is NOT reproducible run to run (measured: same seed, same layout,
    # 43 mm vs 48 mm nearest-neighbour gap in two arms). Freeze the settled world once and
    # load it, so every arm is scored on a bit-identical start state.
    ap.add_argument("--scene-states", default=None,
                    help="JSON of settled bolt poses to load instead of settling")
    ap.add_argument('--work-surface', choices=['foam','bare'], default='foam',
                    help='500x500x20mm dark foam pad, or historical bare table')
    ap.add_argument("--oracle", action="store_true", default=False,
                    help="replace the policy with a privileged-state scripted planner "
                         "(same scene, same controller, same scoring) to measure the CEILING")
    ap.add_argument("--dump-scene-states", default=None,
                    help="settle as usual, then write the settled poses here and exit")
    args = ap.parse_args()
    WORK_SURFACE = work_surface.load(args.work_surface)
    PICK_SURFACE_Z = work_surface.top_z(WORK_SURFACE, TABLE_Z)
    if args.shared_replay and (not args.shared_stack or args.shared_hold):
        ap.error('--shared-replay requires --shared-stack and cannot be combined with --shared-hold')
    if args.shared_stack:
        if args.episodes != 1 or args.oracle or args.protocol != "free":
            ap.error("--shared-stack requires --episodes 1, --protocol free, and no --oracle")
        if DRIVE_MODE != "drive":
            ap.error("--shared-stack requires DRIVE_MODE=drive")
        shared_config = json.loads(pathlib.Path(args.shared_config).read_text())
        if args.rtc or args.execute_steps != shared_config["execute_steps"]:
            ap.error("shared policy timing is selected by --shared-config; do not mix legacy --rtc/--execute-steps")
        if not math.isfinite(args.episode_sec) or args.episode_sec <= 0:
            ap.error("--episode-sec must be positive")
        os.environ["GRIP_MAXF"] = str(shared_config["gripper_max_force_n"])
    if args.protocol in ("std20", "std40"):
        n = 20 if args.protocol == "std20" else 40
        args.episodes, args.seed, args.episode_sec = n, 100, 30.0
        args.n_per_color, args.video = 10, True
        # std40 = seeds 100..139, a strict SUPERSET of std20's 100..119. A std40 run therefore
        # still contains the std20 answer, so widening the protocol does not orphan earlier
        # scores -- the first 20 seeds remain directly comparable.
        print(f"[eval] protocol={args.protocol}: {n} eps, seeds 100-{99+n}, 30 s, "
              "10 bolts/colour, video on")
    global _SCENE_STATES, _DUMP
    _SCENE_STATES, _DUMP = {}, {}
    if args.scene_states:
        _SCENE_STATES = json.loads(pathlib.Path(args.scene_states).read_text())
        for frozen in _SCENE_STATES.values():
            work_surface.check_frozen(frozen, WORK_SURFACE)
        print(f"[eval] frozen scenes: {len(_SCENE_STATES)} seeds from {args.scene_states}")
    RTC_ENABLED = bool(args.rtc)
    CHUNK_EXECUTE_STEPS = int(args.execute_steps)
    PREFETCH_AT = int(np.clip(args.prefetch_at, 0, CHUNK_EXECUTE_STEPS - 1))
    RTC_INFERENCE_DELAY = CHUNK_EXECUTE_STEPS - PREFETCH_AT
    print(f"[eval] rtc={RTC_ENABLED} execute={CHUNK_EXECUTE_STEPS} prefetch_at={PREFETCH_AT} "
          f"realized_delay={RTC_INFERENCE_DELAY}")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    _apply_rtx_settings()

    import imageio.v2 as imageio
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim, SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.sensors.camera import Camera
    from openpi_client import websocket_client_policy
    from PIL import Image
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    outdir = ROOT / "outputs/eval"
    outdir.mkdir(parents=True, exist_ok=True)

    # rendering_dt MUST be given. It defaults to 1/60 s, so with physics_dt=1/500 every
    # render step advanced the timeline by 8.33 physics steps: recording a frame once per
    # policy tick injected a ~6.6 mm TCP jump (measured max 6.58 mm = 3.3 m/s, impossible
    # for a 0.45 m/s command) at exactly 30 Hz. Replaying the identical joint stream with
    # --no-video showed none of it (max step 1.80 mm).
    world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT,
                  rendering_dt=PHYSICS_DT)
    stage = get_current_stage()

    def mat(path, rgb, rough=0.5, metallic=0.0, name=None):
        ov = _MAT_OVERRIDE.get(name or path.rsplit("/", 1)[-1], {})
        if ov:
            rgb, rough, metallic = ov.get("rgb", rgb), ov.get("rough", rough), ov.get("metallic", metallic)
            print(f"  [mat] {name}: rgb {tuple(round(c, 3) for c in rgb)} rough {rough:g} "
                  f"metallic {metallic:g}", flush=True)
        m = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/S")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m

    MAT = {
        "table": mat("/World/m/table", (0.42, 0.44, 0.45), 0.42, 0.65, name="table"),
        "riser": mat("/World/m/riser", (0.05, 0.05, 0.055), 0.55, 0.35, name="riser"),
        "green": mat("/World/m/green", (0.10, 0.38, 0.27), 0.45, name="green"),
        "boxgray": mat("/World/m/boxgray", (0.46, 0.47, 0.49), 0.50, 0.35, name="boxgray"),
        "insert": mat("/World/m/insert", (0.26, 0.26, 0.28), 0.85, name="insert"),
        "bolt_gray": mat("/World/m/bgray", (0.52, 0.53, 0.56), 0.32, 1.0, name="bolt_gray"),
        "bolt_black": mat("/World/m/bblack", (0.07, 0.07, 0.08), 0.42, 0.9, name="bolt_black"),
    }

    def sbox(path, c, h, key):
        cu = UsdGeom.Cube.Define(stage, path)
        cu.CreateSizeAttr(2.0)
        UsdGeom.Xformable(cu).AddTransformOp().Set(
            Gf.Matrix4d().SetScale(Gf.Vec3d(*h)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*c))
        )
        UsdPhysics.CollisionAPI.Apply(cu.GetPrim())
        UsdShade.MaterialBindingAPI(cu.GetPrim()).Bind(MAT[key])

    add_reference_to_stage(usd_path=str(STAND_USD), prim_path="/World/cell/stand")
    mount_xf = {}
    for prim in stage.Traverse():
        for side, frame in MOUNT_FRAME.items():
            if prim.GetName() == frame and side not in mount_xf:
                mount_xf[side] = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
    for side, m in mount_xf.items():
        p = f"/World/cell/{side}_arm"
        UsdGeom.Xform.Define(stage, p).MakeMatrixXform().Set(m)
        add_reference_to_stage(usd_path=str(ARM_USD), prim_path=f"{p}/robot")

    # FINGER EFFORT LIMIT. The real jaws close until they touch -- there is no positional stop at
    # the object, so what makes a real grasp work is that the motor STALLS on the bolt at a finite
    # force (operator, 2026-08-23). The sim drive has no such limit and keeps pushing toward the
    # commanded 0 mm gap, which squeezes the bolt out. SingleArticulation has no set_max_efforts
    # in Isaac 6.0, so set the drive attribute on the joint prims directly, at build time, before
    # PhysX parses them -- writing it later would be overwritten by the articulation.
    _maxf = os.environ.get("GRIP_MAXF")
    if _maxf:
        _n = 0
        for prim in stage.Traverse():
            if prim.GetName() in ("finger_left_joint", "finger_right_joint"):
                for tok in ("linear", "transX", "transY", "transZ"):
                    dr = UsdPhysics.DriveAPI.Get(prim, tok)
                    if dr:
                        dr.CreateMaxForceAttr().Set(float(_maxf))
                        _n += 1
                        break
                else:
                    dr = UsdPhysics.DriveAPI.Apply(prim, "linear")
                    dr.CreateMaxForceAttr().Set(float(_maxf))
                    _n += 1
        print(f"  [grip] finger drive maxForce = {_maxf} N on {_n} joint(s)")
        if _n == 0:
            _die("ABORT: GRIP_MAXF set but no finger joints found -- the limit would "
                 "have been silently ignored and the run would look like a physics result.")

    sbox("/World/scene/table", (TABLE["cx"], TABLE["cy"], TABLE_Z - TABLE["thick"]),
         (TABLE["hx"], TABLE["hy"], TABLE["thick"]), "table")
    work_surface.build(stage, WORK_SURFACE, TABLE_Z)
    # The riser the stand is bolted to. Black, stand footprint (operator, 2026-09-05). It
    # spans table top -> stand base plate, so the arms cannot swing through the one large
    # object that is now directly under them.
    sbox("/World/scene/riser", (RISER["cx"], RISER["cy"], TABLE_Z + RISER_H / 2),
         (RISER["hx"], RISER["hy"], RISER_H / 2), "riser")
    b = BOX
    for name, color, wall in (("box_gray", "gray", "boxgray"), ("box_green", "black", "green")):
        cy = BOX_CY[color]
        r = f"/World/scene/{name}"
        UsdGeom.Xform.Define(stage, r)
        sbox(f"{r}/floor", (BOX_X, cy, TABLE_Z + b["floor_t"] / 2),
             (b["hw"], b["hd"], b["floor_t"] / 2), wall)
        sbox(f"{r}/sponge", (BOX_X, cy, TABLE_Z + b["floor_t"] + b["sponge_h"] / 2),
             (b["hw"] - b["t"], b["hd"] - b["t"], b["sponge_h"] / 2), "insert")
        h = b["wall_h"]
        # hw/hd describe the OUTER box. t is full wall thickness, not a
        # half-extent: the old construction grew every outer face by 20 mm.
        for tag, off, half in (("xm", (-b["hw"]+b["t"]/2, 0), (b["t"]/2, b["hd"], h)),
                               ("xp", (+b["hw"]-b["t"]/2, 0), (b["t"]/2, b["hd"], h)),
                               ("ym", (0, -b["hd"]+b["t"]/2), (b["hw"]-b["t"], b["t"]/2, h)),
                               ("yp", (0, +b["hd"]-b["t"]/2), (b["hw"]-b["t"], b["t"]/2, h))):
            sbox(f"{r}/w_{tag}", (BOX_X + off[0], cy + off[1], TABLE_Z + h), half, wall)

    # Optional explicit contact material for the bolts (see the binding below).
    _PHYSMAT = None
    _mu = os.environ.get("BOLT_FRICTION")
    if _mu:
        _mp = "/World/physmat_bolt"
        _m = UsdShade.Material.Define(stage, _mp)
        _api = UsdPhysics.MaterialAPI.Apply(_m.GetPrim())
        _api.CreateStaticFrictionAttr().Set(float(_mu))
        _api.CreateDynamicFrictionAttr().Set(float(_mu))
        _api.CreateRestitutionAttr().Set(float(os.environ.get("BOLT_RESTITUTION", "0.0")))
        _PHYSMAT = _m.GetPrim()
        print(f"  [phys] bolt friction {_mu}, restitution "
              f"{os.environ.get('BOLT_RESTITUTION', '0.0')}")

    n_bolts = args.n_per_color * 2
    bolt_prims = []
    for i in range(n_bolts):
        path = f"/World/scene/bolt_{i:02d}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.MakeMatrixXform().Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.4, 0, 0.05 + 0.05 * i)))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.022)
        for tag, rr, hl, cx in (("shaft", SHAFT_R, SHAFT_L / 2, SHAFT_L / 2),
                                ("head", HEAD_R, HEAD_L / 2, -HEAD_L / 2)):
            cy_ = UsdGeom.Cylinder.Define(stage, f"{path}/{tag}")
            cy_.CreateRadiusAttr(rr)
            cy_.CreateHeightAttr(hl * 2)
            cy_.CreateAxisAttr("X")
            UsdGeom.Xformable(cy_).AddTranslateOp().Set(Gf.Vec3d(cx, 0, 0))
            UsdPhysics.CollisionAPI.Apply(cy_.GetPrim())
            # The bolts have never had a physics material -- friction and restitution have been
            # whatever PhysX defaults to (0.5/0.5/0). On a task that is entirely about pinching a
            # smooth cylinder, that is an unexamined parameter, so make it explicit and tunable.
            if _PHYSMAT is not None:
                UsdShade.MaterialBindingAPI.Apply(cy_.GetPrim())
                UsdShade.MaterialBindingAPI(cy_.GetPrim()).Bind(
                    UsdShade.Material(_PHYSMAT), bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                    materialPurpose="physics")
        bolt_prims.append(path)

    # FINGERTIP / TABLE FRICTION. (Placed AFTER scene build: the first version ran before
    # sbox() created the table, matched zero colliders, and the loud guard aborted the run
    # -- which is the guard doing its job; a silent version would have produced two fake
    # 'friction has no effect' results.) Operator observation from the montage: when the tips are
    # resting on the table the jaws stop closing, while on hardware they scrape along the
    # surface and still shut. The sim tips and the table have no physics material at all, so
    # they inherit PhysX's default 0.5 and the position-controlled arm presses hard enough for
    # that to lock the prismatic fingers. A bolt cannot be picked if touching down forbids
    # closing, so make the tip/table pair explicitly tunable.
    _fmu = os.environ.get("FINGER_FRICTION")
    if _fmu:
        _fm = UsdShade.Material.Define(stage, "/World/physmat_finger")
        _fa = UsdPhysics.MaterialAPI.Apply(_fm.GetPrim())
        _fa.CreateStaticFrictionAttr().Set(float(_fmu))
        _fa.CreateDynamicFrictionAttr().Set(float(_fmu))
        _fa.CreateRestitutionAttr().Set(0.0)
        _n = 0
        for prim in stage.Traverse():
            nm = prim.GetName()
            if ("finger" in nm or nm == "table") and prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdShade.MaterialBindingAPI.Apply(prim)
                UsdShade.MaterialBindingAPI(prim).Bind(
                    _fm, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                    materialPurpose="physics")
                _n += 1
        print(f"  [phys] finger/table friction {_fmu} on {_n} collider(s)")
        if _n == 0:
            _die("ABORT: FINGER_FRICTION set but no finger/table colliders matched "
                 "-- the run would look like a physics result while changing nothing.")


    key = UsdLux.DistantLight.Define(stage, "/World/key")
    key.CreateIntensityAttr(KEY_INTENSITY)
    key.CreateAngleAttr(KEY_ANGLE)
    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(DOME_INTENSITY)
    if SUN_INTENSITY > 0.0:
        # The window. A DistantLight shines along its local -Z, so tilt it up by
        # (elevation - 90) about X and swing it round by the azimuth about Z.
        sun = UsdLux.DistantLight.Define(stage, "/World/sun")
        sun.CreateIntensityAttr(SUN_INTENSITY)
        sun.CreateAngleAttr(SUN_ANGLE)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(
            Gf.Vec3f(float(SUN_ELEVATION - 90.0), 0.0, float(SUN_AZIMUTH)))
        print(f"  [light] key {KEY_INTENSITY:g}/{KEY_ANGLE:g}deg  dome {DOME_INTENSITY:g}  "
              f"sun {SUN_INTENSITY:g}/{SUN_ANGLE:g}deg az {SUN_AZIMUTH:g} el {SUN_ELEVATION:g}",
              flush=True)

    arts = {}
    if args.shared_stack:
        from shared_contact import configure_materials
        configure_materials(stage, shared_config)
    for side in MOUNT_FRAME:
        a = SingleArticulation(prim_path=f"/World/cell/{side}_arm/robot", name=f"{side}_arm")
        world.scene.add(a)
        arts[side] = a
    world.reset()
    for a in arts.values():
        a.initialize()
    # DOF order is the articulation's, not ARM_JOINTS'; resolve it once so the 500 Hz
    # diagnostic read is an array index instead of a list search per substep.
    _ARM_IDX = {sd: np.array([list(arts[sd].dof_names).index(j) for j in ARM_JOINTS], dtype=int)
                for sd in arts}

    bolt_views = {p: RigidPrim(prim_paths_expr=p, name=f"bv{i}")
                  for i, p in enumerate(bolt_prims)}
    for v in bolt_views.values():
        v.initialize()

    # ---- T1 instrumentation: read EVERY bolt pose once per policy tick -------------------
    # The multimodality probe needs all bolt positions at 30 Hz (to see which candidate the
    # chunk endpoint is aimed at). Per-bolt reads would be 20 x 900 round trips per episode,
    # so use one batched view and remap it into bolt_prims order. Falls back to per-bolt
    # reads rather than failing: a slower eval is better than no eval.
    bolts_all, _bolt_order = None, None
    try:
        _bv = RigidPrim(prim_paths_expr="/World/scene/bolt_.*", name="bolts_all")
        _bv.initialize()
        _paths = getattr(_bv, "prim_paths", None) or getattr(_bv, "paths", None)
        _bolt_order = [bolt_prims.index(str(p)) for p in list(_paths)]
        if sorted(_bolt_order) != list(range(len(bolt_prims))):
            raise RuntimeError(f"batched view covers {len(_bolt_order)}/{len(bolt_prims)} bolts")
        bolts_all = _bv
    except Exception as exc:
        print(f"  [warn] batched bolt view unavailable ({exc}); falling back to per-bolt reads")

    def bolt_all_poses():
        """(pos, quat) for every bolt, in bolt_prims order -- what freezing records."""
        P = np.stack([np.asarray(bolt_views[p].get_world_poses()[0])[0] for p in bolt_prims])
        Q = np.stack([np.asarray(bolt_views[p].get_world_poses()[1])[0] for p in bolt_prims])
        return P, Q

    def bolt_axis_xy(i):
        """In-plane direction of bolt i's shaft, as a world-XY angle in degrees.

        The demonstrations grasp bolts across the shaft, so the jaw line (tool X -- the fingers
        are prismatic +-X) has to end up perpendicular to this. Measured 2026-08-22: EVERY
        successful grasp sits above 60 deg of separation and no close below it has ever
        succeeded, which makes this an independent necessary condition alongside lateral aim.
        Recorded at the close so it no longer has to be recovered by matching positions back to
        the frozen scene -- that matching silently drops every bolt the arms have nudged.
        """
        q = np.asarray(bolt_views[bolt_prims[i]].get_world_poses()[1])[0]
        w, x, y, z = (float(v) for v in q)
        # bolt shaft is the prim's local X (build_scene.py sets CreateAxisAttr("X"))
        ax = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)])
        return float(np.degrees(np.arctan2(ax[1], ax[0]))), float(ax[2])

    def bolt_xyz():
        """(n_bolts, 3) world positions, in bolt_prims order."""
        if bolts_all is not None:
            P = np.asarray(bolts_all.get_world_poses()[0])
            out = np.empty_like(P)
            out[_bolt_order] = P
            return out
        return np.stack([np.asarray(bolt_views[p].get_world_poses()[0])[0] for p in bolt_prims])

    tcp_views, wrist_cams = {}, {}
    LENS = (0.00917, 0.04601, 0.11930)
    for side in MOUNT_FRAME:
        root = f"/World/cell/{side}_arm/robot"
        tcp_path = next(str(p.GetPath()) for p in stage.Traverse()
                        if str(p.GetPath()).startswith(root) and p.GetName() == "tcp"
                        and p.HasAPI(UsdPhysics.RigidBodyAPI))
        v = RigidPrim(prim_paths_expr=tcp_path, name=f"tcp_{side}")
        v.initialize()
        tcp_views[side] = v
        tool_path = next(str(p.GetPath()) for p in stage.Traverse()
                         if str(p.GetPath()).startswith(root) and p.GetName() == "tool"
                         and p.HasAPI(UsdPhysics.RigidBodyAPI))
        cam = Camera(prim_path=f"{tool_path}/wrist_cam", name=f"d405_{side}",
                     resolution=(640, 480))
        cam.initialize()
        cam.set_local_pose(translation=np.array(LENS),
                           orientation=np.array([0.0, 0.0, 0.0, 1.0]),  # 180 deg roll
                           camera_axes="ros")
        cam.prim.GetAttribute("focalLength").Set(11.0)
        cam.prim.GetAttribute("horizontalAperture").Set(H_APERTURE)
        cam.prim.GetAttribute("verticalAperture").Set(V_APERTURE)
        ppx, ppy = PP_PX[side]
        # Sign verified against Gf.Camera.ComputeProjectionMatrix, not guessed: the aperture
        # offset moves the frustum window, so the optical axis lands on the OPPOSITE side of
        # the frame. With these two negations, EVAL_PP_* = (+40, +40) puts the principal
        # point at pixel (360, 280), i.e. right and down, matching the +x-right/+y-down
        # convention the collected intrinsics are quoted in.
        cam.prim.GetAttribute("horizontalApertureOffset").Set(-ppx / 640.0 * H_APERTURE)
        cam.prim.GetAttribute("verticalApertureOffset").Set(ppy / 480.0 * V_APERTURE)
        print(f"  [cam] {side:5s} aperture H {H_APERTURE:.4f} V {V_APERTURE:.4f} "
              f"-> fx {11.0 / H_APERTURE * 640:.2f} px | pp offset {ppx:+.2f},{ppy:+.2f} px",
              flush=True)
        cam.prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.004, 100.0))
        wrist_cams[side] = cam

    # The overview render product is the video source only. It is 960x720 and renders on every
    # orchestrator step whether or not a frame is kept, so it is created ONLY when recording.
    ov_ann = top_ann = None
    if args.video:
        ov = rep.create.camera(position=(1.85, -1.35, 1.15), look_at=(0.55, 0.0, 0.15),
                               clipping_range=(0.01, 100.0))
        ov_rp = rep.create.render_product(ov, (960, 720))
        ov_ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ov_ann.attach([ov_rp])
        if os.environ.get("EVAL_TOPVIEW"):
            # A near-top view of the bolt piles, for judging grasps by eye. Not straight down:
            # a perfectly vertical camera is degenerate here (view axis parallel to world up,
            # sec.12), so it sits back on +x and looks down at the piles -- robot at the top of
            # frame, +y to the right, matching the montage convention.
            tv = rep.create.camera(position=(1.00, 0.0, 0.95), look_at=(0.58, 0.0, 0.0),
                                   clipping_range=(0.01, 100.0))
            top_rp = rep.create.render_product(tv, (960, 720))
            top_ann = rep.AnnotatorRegistry.get_annotator("rgb")
            top_ann.attach([top_rp])

    if args.shared_stack:
        from shared_stack import run_episode
        exit_code = 1
        try:
            exit_code = run_episode(args, shared_config, sys.modules[__name__], world, rep,
                                    arts, wrist_cams, tcp_views, bolt_views, bolt_prims,
                                    ov_ann, MAT)
            return exit_code
        except BaseException:
            import traceback
            traceback.print_exc()
            raise
        finally:
            app.close(exit_code=exit_code)

    class Oracle:
        """Scripted pick-and-place from privileged state, emitting the SAME chunk format.

        Why this exists. Every improvement arm so far sits below the control, and we have no
        upper bound: 42 placements might be 90% of what is physically achievable in 30 s or it
        might be 20%. Worse, sim converts only 11% of close commands into grasps where the
        hardware record converts ~47%, so the "precision is the bottleneck" diagnosis could be
        measuring sim contact physics rather than the model. An oracle with exact bolt poses
        separates the two: if perfect aim still rarely grasps, the rig's physics is the term to
        fix before any more training arms.

        It replaces ONLY the policy. Scene, controller (Ruckig follower + DLS IK), scoring and
        the T1 instrument are untouched, so the difference is attributable to the policy alone.

        Attitude is taken from what actually works rather than designed: successful grasps
        approach 8-9 deg off straight down with the jaw axis horizontal, and every one of the 36
        recorded grasps has the jaw within 30 deg of perpendicular to the bolt. So the template
        is tool z straight down, tool x perpendicular to the target bolt's shaft.
        """

        # heights (m) above the bolt, and the grip percentages the follower sees
        H_APPROACH = 0.080
        Z_GRASP_OFF = float(os.environ.get("ORACLE_ZOFF", "0.0026"))
        # Transit/carry height. The box walls top out at z=105 mm. The first oracle traversed
        # at TCP 89 mm (16 mm BELOW the wall) and carried at 150 mm, where a grasped bolt's
        # lowest point hangs at ~113 mm -- 8 mm of wall clearance, gone at the first shake.
        # Video review + the lifted/dropped counters (8 lifted, 4 dropped in one run) showed
        # both: wall strikes on the way to pick, and bolts clipped off on the way in.
        H_LIFT = 0.200
        TRANSIT_XY = 0.040       # descend only when this close, laterally, to the target
        # Place at the NEAR interior of the box (still inside: interior x 0.62..0.82), not the
        # centre. 60 mm less extension where the Jacobian's smallest singular value collapses
        # to 2e-4 (1/90th of its overall p10) -- the measured shake-and-drop zone.
        PLACE_X = 0.660
        # Partial opening (operator): at 50% the half-gap is 23.5 mm against an 18.4 mm
        # head -- still 2.5 mm of aim slack per side for an oracle that lands at 0.1 mm, and
        # the narrower sweep stops the descent snagging neighbouring bolts.
        OPEN = float(os.environ.get("GRIP_OPEN", "100"))
        SHUT = 0.0
        STEP_XY, STEP_Z = 0.010, 0.006          # per policy tick; well inside LIN_V*dt = 15 mm
        GAIN = 0.15                             # fraction of the remaining error per row
        # The jaws must not slam. The policy closes at ~0.8%/tick (measured over 80 transitions
        # in its own dumps) and the hardware record puts jaw lag at 105-209 ms; the first oracle
        # stepped 100 -> 0 in ONE tick, which launched properly-arrived bolts up to 237 mm. That
        # is a planner artifact, not a verdict on sim contact physics, so the rate is a knob and
        # the close phase waits for the ramp to finish.
        GRIP_RATE = float(os.environ.get("GRIP_RATE", "100"))     # percent per policy tick
        RELEASE_TICKS = 6

        @property
        def CLOSE_TICKS(self):
            return int(max(12, 100.0 / self.GRIP_RATE + 8))

        # A phase must not be able to stall forever. The first version used 4 mm XY / 10 mm Z
        # transition gates, which the offline replay clears instantly with perfect tracking but
        # the real follower never does -- so all three smoke episodes sat in "seek" and issued
        # ZERO gripper commands. Gates are now loose and every phase is force-advanced after
        # PHASE_TIMEOUT, because a ceiling measurement has to actually attempt grasps.
        PHASE_TIMEOUT = 105          # ticks (3.5 s)
        DEBUG = bool(int(os.environ.get("ORACLE_DEBUG", "0")))

        def __init__(self):
            self.phase = {"left": "seek", "right": "seek"}
            self.target = {"left": None, "right": None}
            self.timer = {"left": 0, "right": 0}
            self.age = {"left": 0, "right": 0}
            self.place_col = {"left": "gray", "right": "black"}
            self.claimed = set()
            self.clock = 0
            self.cooldown = {}          # bolt idx -> clock tick until which it is skipped
            # return-mode state: sequential arms, constant open jaws, demo-speed caps
            self.ret = {"order": None, "phase": {"left": "wait", "right": "wait"},
                        "start": {}, "goal": {}, "grip0": {}, "timer": {"left": 0, "right": 0}}

        def _go(self, side, ph, timer=0):
            if self.DEBUG:
                print(f"      [oracle] {side} {self.phase[side]} -> {ph} "
                      f"(after {self.age[side]} ticks)")
            self.phase[side], self.timer[side], self.age[side] = ph, timer, 0

        # Approach attitude, taken from what the policy's SUCCESSFUL grasps actually do rather
        # than from what looks ideal. Exactly-straight-down was tried first and is wrong twice
        # over: it is a 180 deg rotation, i.e. sitting on the rotvec representation singularity,
        # and it is at the edge of the reachable set -- the traced run converged to a 22.8 mm
        # lateral residual and then froze, because the IK was pushing into a joint limit that
        # the command clamp holds. Successful grasps sit 8-9 deg off vertical (failures 11.6 and
        # 22.5), so the oracle uses that, per arm, in the direction the measurement shows.
        TILT = {"left": np.array([0.14, 0.01, -0.99]),
                "right": np.array([0.01, -0.14, -0.99])}
        # PLACE attitude. The operator deliberately tilted RX when placing so the far reach
        # stays feasible, and the collected data carries it: measured over the control
        # policy's rollout dumps, the tool near the box (x>0.60) runs 20-26 deg off vertical
        # with tool z tipped toward +x [0.4, 0, -0.9], versus 6-9 deg over the pile. The
        # first oracle placed dead-vertical there -- exactly the ill-conditioned pose the
        # demos avoid on purpose.
        TILT_PLACE = {"left": np.array([0.42, 0.0, -0.91]),
                      "right": np.array([0.42, 0.0, -0.91])}

        @classmethod
        def _pose(cls, p_xyz, bolt_ang_deg, side="left", place=False, ref=None):
            """tool z at the measured approach tilt, tool x as perpendicular to the shaft as
            that tilt allows."""
            tilt = cls.TILT_PLACE if place else cls.TILT
            zt = tilt[side] / np.linalg.norm(tilt[side])
            b = np.array([np.cos(np.radians(bolt_ang_deg)), np.sin(np.radians(bolt_ang_deg)), 0.0])
            xt = np.cross(np.array([0.0, 0.0, 1.0]), b)
            n = np.linalg.norm(xt)
            xt = xt / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
            xt = xt - np.dot(xt, zt) * zt                  # re-orthogonalise against the tilt
            xt = xt / np.linalg.norm(xt)
            # A two-jaw gripper is 180-deg symmetric: "perpendicular to the shaft" has TWO
            # solutions a half-turn apart, and cross(up, shaft) picks between them by which
            # way the bolt HEAD points -- sending the wrist on needless half-turns between
            # picks (operator, from video). Take the solution nearer the current jaw axis;
            # the yaw move is then never more than 90 deg.
            if ref is not None and np.dot(xt, rotvec_to_mat(np.asarray(ref[3:]))[:, 0]) < 0.0:
                xt = -xt
            yt = np.cross(zt, xt)
            R = np.column_stack([xt, yt, zt])
            return np.concatenate([p_xyz, mat_to_rotvec(R)])

        def _pick(self, side, B, placed):
            """Nearest unclaimed, unplaced bolt of this arm's colour."""
            cols = (set(colors) if ORACLE_ARM == side
                    else {c for c, a_ in ARM_OF_COLOR.items() if a_ == side})
            cand = [i for i, c in enumerate(colors)
                    if c in cols and bolt_prims[i] not in placed
                    and bolt_prims[i] not in self.claimed
                    and self.cooldown.get(i, 0) <= self.clock]
            if not cand:                       # everything cooling down: take the coolest
                cand = [i for i, c in enumerate(colors)
                        if c in cols and bolt_prims[i] not in placed
                        and bolt_prims[i] not in self.claimed]
            if not cand:
                return None
            home = np.array([PILE_X, PILE_DY if side == "left" else -PILE_DY])

            def score(i):
                # Prefer ISOLATED bolts: overlapping pairs produced double-grasps and blocked
                # closes (operator, from video). Clearance to the nearest OTHER bolt counts
                # for, distance from this arm's pile counts against; clearance is capped at
                # 30 mm because beyond a jaw-width it buys nothing.
                d_others = [np.linalg.norm(B[i][:2] - B[j][:2])
                            for j in range(len(B)) if j != i]
                clear = min(min(d_others) if d_others else 1.0, 0.030)
                return 2.0 * clear - np.linalg.norm(B[i][:2] - home)

            k = max(cand, key=score)
            self.claimed.add(bolt_prims[k])
            if ORACLE_ARM != side:
                # bimanual: box = the bolt's own colour
                self.place_col[side] = colors[k]
            # single-arm: place_col stays this arm's OWN box (init value). The far box is at
            # the edge of the arm's comfortable reach and placing there contaminated the
            # measurement (operator). Cross-colour bolts then score as placed_wrong, so for
            # oracle runs the pick metric is correct+wrong combined -- read it that way.
            return k

        def plan(self, side, anchor, B, placed):
            """-> (goal pose (6,), grip pct). Called once per chunk boundary."""
            if ORACLE_RETURN:
                self.clock += 1
                return self.plan_return(side, anchor, B)
            self.age[side] += CHUNK_EXECUTE_STEPS
            self.clock += CHUNK_EXECUTE_STEPS
            stuck = self.age[side] > self.PHASE_TIMEOUT
            ph = self.phase[side]
            i = self.target[side]
            if ph == "seek" and i is None:
                i = self.target[side] = self._pick(side, B, placed)
                if i is None:
                    # Nothing left to fetch: hover over this arm's pile rather than commanding
                    # the anchor itself, which is the degenerate request Ruckig rejects.
                    return self._pose(np.array([PILE_X, PILE_DY if side == "left" else -PILE_DY,
                                                self.H_LIFT]), 0.0, side), self.OPEN
            if i is not None and ph in ("seek", "descend", "close"):
                b = B[i]
                ang = bolt_axis_xy(i)[0]
                if ph == "seek" or ph == "descend":
                    if stuck:
                        # Blocked (a wall in the descent path, an unreachable pose): abandon
                        # THIS bolt for 10 s and pick another, instead of force-advancing into
                        # a pointless close and then retrying the same bolt forever -- the
                        # operator-observed loop. Deliberately no path planning: a cooldown
                        # list is all a physics instrument needs.
                        self.cooldown[i] = self.clock + 300
                        self.claimed.discard(bolt_prims[i])
                        self.target[side] = None
                        self._go(side, "seek")
                        return self._pose(np.array([b[0], b[1], self.H_LIFT]),
                                          ang, side, ref=anchor), self.OPEN
                if ph == "seek":
                    lat = float(np.linalg.norm(anchor[:2] - b[:2]))
                    # stay at transit height until overhead; descending early is what dragged
                    # the fingers through the box walls (and through the pile itself)
                    tz = b[2] + self.H_APPROACH if lat < self.TRANSIT_XY else self.H_LIFT
                    goal = self._pose(np.array([b[0], b[1], tz]), ang, side, ref=anchor)
                    if self.DEBUG and self.age[side] % 20 == 0:
                        att = np.degrees(np.linalg.norm(mat_to_rotvec(
                            rotvec_to_mat(anchor[3:]).T @ rotvec_to_mat(goal[3:]))))
                        print(f"      [oracle] {side} seek t={self.age[side]:3d} "
                              f"dxy={np.linalg.norm(anchor[:2]-b[:2])*1e3:6.1f}mm "
                              f"dz={(anchor[2]-b[2]-self.H_APPROACH)*1e3:+7.1f}mm "
                              f"datt={att:5.1f}deg")
                    if (np.linalg.norm(anchor[:2] - b[:2]) < 0.012
                            and abs(anchor[2] - (b[2] + self.H_APPROACH)) < 0.020):
                        self._go(side, "descend")
                    return goal, self.OPEN
                if ph == "descend":
                    goal = self._pose(np.array([b[0], b[1], b[2] + self.Z_GRASP_OFF]), ang, side, ref=anchor)
                    if abs(anchor[2] - (b[2] + self.Z_GRASP_OFF)) < 0.008:
                        self._go(side, "close", self.CLOSE_TICKS)
                    return goal, self.OPEN
                # close: hold still and shut, ramping the jaws rather than stepping them
                self.timer[side] -= CHUNK_EXECUTE_STEPS
                if self.timer[side] <= 0:
                    self._go(side, "lift")
                closed_for = self.CLOSE_TICKS - max(0, self.timer[side])
                g = max(self.SHUT, self.OPEN - self.GRIP_RATE * closed_for)
                return (self._pose(np.array([b[0], b[1], b[2] + self.Z_GRASP_OFF]), ang, side, ref=anchor), g)
            if ph == "lift":
                goal = anchor.copy(); goal[2] = self.H_LIFT
                if stuck or anchor[2] > self.H_LIFT - 0.020:
                    held = (i is not None
                            and np.linalg.norm(B[i] - anchor[:3]) < 0.065)
                    if held:
                        self._go(side, "carry")
                    else:
                        # Lifted nothing: an 8 s empty round-trip to the box teaches us
                        # nothing. Cool the bolt down and move on.
                        self.cooldown[i] = self.clock + 300
                        self.claimed.discard(bolt_prims[i])
                        self.target[side] = None
                        self._go(side, "seek")
                return goal, self.SHUT
            if ph == "carry":
                col = self.place_col[side]
                goal = self._pose(np.array([self.PLACE_X, BOX_CY[col], self.H_LIFT]), 0.0, side, place=True)
                if stuck or np.linalg.norm(anchor[:2] - goal[:2]) < 0.035:
                    self._go(side, "release", self.RELEASE_TICKS)
                return goal, self.SHUT
            # release
            self.timer[side] -= CHUNK_EXECUTE_STEPS
            if self.timer[side] <= 0:
                # Release the claim so a MISSED bolt is retried. Holding it would make the
                # oracle walk away from every failure, which is exactly the wrong behaviour in
                # a ceiling measurement -- the ceiling has to include retries.
                if self.target[side] is not None:
                    self.claimed.discard(bolt_prims[self.target[side]])
                self.target[side] = None
                self._go(side, "seek")
            # Hold ABOVE THE BOX while the jaws open, not at the anchor: an exact self-target
            # is the degenerate Ruckig input, and this keeps the bolt over the box as it drops.
            col_ = self.place_col[side]
            return self._pose(np.array([self.PLACE_X, BOX_CY[col_], self.H_LIFT]), 0.0, side, place=True), self.OPEN

        def plan_return(self, side, anchor, B):
            r = self.ret
            if r["order"] is None:
                rng_ = np.random.default_rng(int(abs(B.sum()) * 1e6) % (2**31))
                r["order"] = ["left", "right"] if rng_.random() < 0.5 else ["right", "left"]
                for s2 in ("left", "right"):
                    r["grip0"][s2] = float(rng_.uniform(60.0, 100.0))
            if side not in r["start"]:
                r["start"][side] = anchor.copy()
                # v2 (operator, from the v1 mp4 review): v1 yawed toward a chosen bolt before
                # any bolt was even in view -- rotation the demos never show at this stage. The
                # return is now TRANSLATION ONLY: same attitude as the start (the slight place
                # tilt stays), same height (no descent), moving to the vicinity of the RESET
                # pose where picking normally begins. No bolt targeting, no rotation at all.
                rng_g = np.random.default_rng(int(abs(B.sum()) * 1e6 + (0 if side == "left" else 7)) % (2**31))
                rp = np.deg2rad(np.array(RESET[side], dtype=float))
                p_r, _, _, _ = fk_chain(rp, T_mount[side])
                gx = p_r[0] + rng_g.uniform(-0.03, 0.03)
                gy = p_r[1] + rng_g.uniform(-0.03, 0.03)
                r["goal"][side] = np.concatenate([[gx, gy, anchor[2]], anchor[3:]])
            first, second = r["order"]
            active = first if r["phase"][first] != "done" else second
            g = r["grip0"][side]
            if side != active:
                # The idle arm HOLDS -- but a human hold is not a machine hold: the demos'
                # idle arm drifts ~1.4 mm/step. A perfectly frozen pose would hand the model
                # a "this is sim" cue (and an idle-arm distribution it has never seen), so the
                # hold breathes: a smooth deterministic wander of ~2 mm amplitude.
                base = (r["goal"][side] if r["phase"][side] == "done"
                        else r["start"][side]).copy()
                ph = 0.0 if side == "left" else 1.7
                c = self.clock * CHUNK_EXECUTE_STEPS if False else self.clock
                base[0] += 0.002 * np.sin(0.11 * c + ph)
                base[1] += 0.002 * np.sin(0.07 * c + 2.1 + ph)
                base[2] += 0.001 * np.sin(0.13 * c + 0.8 + ph)
                return base, g
            goal = r["goal"][side].copy()
            lat = float(np.linalg.norm(anchor[:2] - goal[:2]))
            dz_ = abs(anchor[2] - goal[2])
            # v2: goal height == start height, so no transit clamp is needed; the whole
            # path stays above the walls by construction.
            if lat < 0.012 and dz_ < 0.020:
                r["timer"][side] += CHUNK_EXECUTE_STEPS
                if r["timer"][side] > 20:                       # 0.7 s settled hover
                    r["phase"][side] = "done"
            return goal.copy(), g

        def chunk(self, side, anchor, B, placed):
            """Absolute goal -> H rows of the sequential body-frame deltas the loop integrates."""
            goal, grip = self.plan(side, anchor, B, placed)
            rows = np.zeros((ACTION_HORIZON, 7), dtype=float)
            cur_p, cur_R = anchor[:3].copy(), rotvec_to_mat(anchor[3:])
            R_goal = rotvec_to_mat(goal[3:])
            for k in range(ACTION_HORIZON):
                d = goal[:3] - cur_p
                lim = np.array([self.STEP_XY, self.STEP_XY, self.STEP_Z])
                # PROPORTIONAL, not saturating-then-stopping. The runner drops the rows that
                # elapsed since the observation (`drop` = 4 here), so rows 4..7 are what actually
                # executes. A step that clips straight to the goal reaches it by row 2 and leaves
                # rows 4..7 as exact zeros -- the arm then never moves again, the anchor never
                # advances, and the next chunk reproduces the same zeros. That deadlock froze
                # three oracle runs at 17-29 mm lateral error, always below the 4 x 10 mm the
                # skipped rows could have covered. A geometric approach never emits zeros.
                if ORACLE_RETURN:
                    # NORM cap, not per-axis: axis clipping lets diagonals reach 8.7 mm/step,
                    # already past the demo p90. 5 mm on the vector keeps every step inside.
                    step = self.GAIN * d
                    n_s = float(np.linalg.norm(step))
                    if n_s > 0.005:
                        step = step * (0.005 / n_s)
                else:
                    step = np.clip(self.GAIN * d, -lim, lim)
                nxt_p = cur_p + step
                # slew the attitude a fixed fraction per tick so it arrives with the position
                dR = cur_R.T @ R_goal
                rv = mat_to_rotvec(dR)
                if ORACLE_RETURN:
                    n_ = np.linalg.norm(rv)
                    rv_step = rv * min(1.0, np.radians(1.0) / max(n_, 1e-9))
                    nxt_R = cur_R @ rotvec_to_mat(rv_step)
                else:
                    nxt_R = cur_R @ rotvec_to_mat(rv * 0.25)
                # invert the loop's composition: p += R_cur @ (R_ALIGN @ dp)
                rows[k, :3] = R_ALIGN.T @ (cur_R.T @ (nxt_p - cur_p))
                rows[k, 3:6] = R_ALIGN.T @ mat_to_rotvec(cur_R.T @ nxt_R)
                # rows 4..7 are what actually executes, so the ramp has to live in the rows.
                if ORACLE_RETURN:
                    rows[k, 6] = grip / 100.0          # constant open hold, no ramp
                else:
                    rows[k, 6] = max(0.0, min(1.0, (grip - self.GRIP_RATE * k) / 100.0)) \
                        if grip < self.OPEN else grip / 100.0
                cur_p, cur_R = nxt_p, nxt_R
            return rows

    oracle = Oracle() if args.oracle else None
    client = None if args.oracle else \
        websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
    print(f"connected to {args.host}:{args.port}")
    if client is not None:
        server_metadata = client.get_server_metadata()
        print(f"server metadata: {server_metadata}")
        # ---- ACTION CONTRACT GATE -----------------------------------------------------
        # The decode mode is a flag on this side and a training choice on that side, and
        # nothing was checking they agreed. On 2026-09-04 :8002 was restarted onto an
        # anchored checkpoint while the rig kept its `delta` default; the run looked
        # healthy (blank_obs 0, closes logged, video written) and was pure garbage -- the
        # gripper closed 270 mm above the bolts. Refuse to score that.
        _ck = _resolve_served_checkpoint(args.host, args.port)
        _con = _checkpoint_contract(_ck.get("dir", "")) if _ck.get("resolved") else {}
        global _CONTRACT
        _CONTRACT = _con
        if _con.get("action_mode"):
            print(f"[eval] served checkpoint trained on {_con['dataset']} "
                  f"-> action_mode={_con['action_mode']}")
            if _con["action_mode"] != ACTION_MODE:
                _abort_unverified(
                    f"ACTION_MODE={ACTION_MODE} but the served checkpoint was trained "
                    f"{_con['action_mode']} ({_con['dataset']}) -- re-run with "
                    f"ACTION_MODE={_con['action_mode']}",
                    "decoding anchored rows as chained deltas puts the command ~24 "
                    "waypoints past the anchor every chunk; measured 2026-09-04, the "
                    "gripper closed with the bolt 270.6 mm below it and the run still "
                    "wrote a clean-looking summary")
            if ACTION_MODE == "anchored" and not RTC_NORM_STATS:
                # From the SERVED checkpoint, never a hardcoded path -- the re-anchor is
                # SE(3) algebra and needs THIS model's action q01/q99 to unnormalise.
                _load_rtc_norm_stats(_con["norm_stats"])
        else:
            _abort_unverified(
                f"could not read the served checkpoint's action contract "
                f"(dir={_ck.get('dir') or _ck.get('why', 'unresolved')}), so "
                f"ACTION_MODE={ACTION_MODE} is UNVERIFIED against the model",
                "the whole point of the gate is that an unverified contract is the state "
                "the 2026-09-04 run was in")
        if RTC_NORM_STATS:
            _load_rtc_norm_stats(RTC_NORM_STATS)
        if ACTION_MODE == "anchored" and _RTC_NORM_Q is None:
            _abort_unverified(
                "ACTION_MODE=anchored but no action norm stats could be loaded",
                "the anchored RTC re-anchor is SE(3) algebra on UNNORMALISED values, so "
                "without them RTC prev-chunk conditioning silently degrades to vanilla "
                "sampling -- a different algorithm wearing the same --rtc flag")
        # The horizon is the one thing the server DOES tell us, and the rig hardcodes it
        # (ACTION_HORIZON, RESERVE_STEPS lookahead, the RTC delay). A checkpoint served at
        # a different horizon would be sliced against the wrong assumptions.
        _sh = (server_metadata or {}).get("action_horizon")
        if _sh is not None and int(_sh) != ACTION_HORIZON:
            _abort_unverified(
                f"the server serves action_horizon={_sh} but the rig is built for "
                f"ACTION_HORIZON={ACTION_HORIZON}",
                "chunk integration, the reserve lookahead and the RTC delay are all "
                "written against the rig's horizon")
        # 14 valid dims, read as two 7-wide arm blocks. Anything narrower is not this task.
        _sd = (server_metadata or {}).get("action_dim")
        if _sd is not None and int(_sd) < 14:
            _abort_unverified(
                f"the server serves action_dim={_sd}, below the 14 this task decodes",
                "the rig slices [left 7 | right 7] out of every row")
    else:
        print("ORACLE mode: privileged-state planner, no policy server")
        server_metadata = None

    def tcp_pose(side):
        pos, quat = tcp_views[side].get_world_poses()
        p = np.asarray(pos)[0].astype(float)
        R = quat_to_mat(np.asarray(quat)[0].astype(float))
        return p, R

    _kin_prev = {}

    def set_arm(side, q_arm, grip):
        art = arts[side]
        names = list(art.dof_names)
        q = np.array(art.get_joint_positions(), dtype=np.float32)
        for j, val in zip(ARM_JOINTS, q_arm):
            q[names.index(j)] = val
        fp = (1.0 - grip / 100.0) * FINGER_TRAVEL_M
        for jn, sgn in (("finger_left_joint", +1.0), ("finger_right_joint", -1.0)):
            if jn in names:
                q[names.index(jn)] = sgn * fp
        # The drive target is set in BOTH modes: in kinematic mode it keeps the fingers under
        # their force-limited drive (and stops the solver from fighting the arm state it is
        # about to be handed), while the arm state is overwritten below.
        art.apply_action(ArticulationAction(joint_positions=q))
        if DRIVE_MODE != "kinematic":
            return
        ix = _ARM_IDX[side]
        qa = np.asarray(art.get_joint_positions(), dtype=np.float32).copy()
        qv = np.asarray(art.get_joint_velocities(), dtype=np.float32).copy()
        prev = _kin_prev.get(side)
        qa[ix] = np.asarray(q_arm, dtype=np.float32)
        # Write the velocity the command implies, not zero: a zeroed velocity state would make
        # every contact read as a standing collision and would itself be a fiction.
        qv[ix] = ((np.asarray(q_arm) - prev) / PHYSICS_DT if prev is not None
                  else np.zeros(len(ix)))
        _kin_prev[side] = np.asarray(q_arm, dtype=float).copy()
        art.set_joint_positions(qa)
        art.set_joint_velocities(qv)

    T_mount = {}
    for side, m in mount_xf.items():
        T_mount[side] = np.array([[m[i][j] for j in range(4)] for i in range(4)]).T

    results = []
    for ep in range(args.episodes):
        ep_seed = args.seed + ep
        # ---- reset -----------------------------------------------------------
        for side, art in arts.items():
            names = list(art.dof_names)
            q = np.zeros(len(names), dtype=np.float32)
            for j, deg in zip(ARM_JOINTS, RESET[side]):
                q[names.index(j)] = np.deg2rad(deg)
            if side == ORACLE_PARKED:
                q[:] = 0.0                     # straight up, out of the working volume
            if ORACLE_RETURN:
                # Teleport to a jittered own-box place pose: offline DLS from the reset q --
                # pure kinematics, no physics stepping, so it cannot disturb the settled scene.
                rng_ = np.random.default_rng(ep_seed * 7 + (0 if side == "left" else 1))
                col_ = "gray" if side == "left" else "black"
                tgt_p = np.array([0.66 + rng_.uniform(-0.02, 0.02),
                                  BOX_CY[col_] + rng_.uniform(-0.02, 0.02),
                                  0.20 + rng_.uniform(-0.03, 0.03)])
                # operator: the start is a SLIGHTLY tilted post-place pose (padDCp t=11 s),
                # not the full 25-deg place tilt -- the full tilt held during the return put
                # the right arm's IK at its limits and it ran away (joint-limit clamp drift).
                zt = np.array([0.15 + rng_.uniform(-0.07, 0.07), rng_.uniform(-0.05, 0.05), -0.99])
                zt /= np.linalg.norm(zt)
                # Jaw yaw from the MEASURED post-place distribution, not uniform(0, pi): padDC
                # rollouts hovering open over the box sit at +79 deg +-5 (left) / +96 deg +-5
                # (right) world jaw angle -- a tight band, folded onto the reset pose's own jaw
                # direction. Random start yaws put the wrist somewhere the policy never is
                # after placing (operator, from the v3 preview review).
                psi = np.radians((79.0 if side == "left" else 96.0) + rng_.uniform(-6.0, 6.0))
                xt = np.array([np.cos(psi), np.sin(psi), 0.0])
                xt -= np.dot(xt, zt) * zt; xt /= np.linalg.norm(xt)
                R_t = np.column_stack([xt, np.cross(zt, xt), zt])
                qq = np.deg2rad(np.array(RESET[side], dtype=float))
                for _ in range(400):
                    p_f, R_f, _, _ = fk_chain(qq, T_mount[side])
                    err = np.concatenate([tgt_p - p_f,
                                          mat_to_rotvec(R_t @ R_f.T)])
                    if np.linalg.norm(err[:3]) < 5e-4 and np.linalg.norm(err[3:]) < 2e-3:
                        break
                    J = jacobian(qq, T_mount[side])
                    dq_ = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
                    qq = np.clip(qq + np.clip(dq_, -0.05, 0.05), JOINT_LO, JOINT_HI)
                for j, val in zip(ARM_JOINTS, qq):
                    q[names.index(j)] = val
                _RET_Q[side] = qq.copy()   # q_cmd does not exist yet at this point
            art.set_joint_positions(q)
            art.set_joint_velocities(np.zeros_like(q))
            # kp=1e7 at a 2 ms step is extremely stiff; expose it so the tremor can be
            # tested against drive stiffness instead of assumed innocent.
            _kp = float(os.environ.get("TREMOR_KP", 1.0e7))
            _kd = float(os.environ.get("TREMOR_KD", 1.0e5))
            kps, kds = np.full(len(names), _kp), np.full(len(names), _kd)
            # The FINGERS were inheriting the arm's gains: kp=1e7 on a prismatic joint is
            # 10 MN/m, and grip 0 commands 47 mm of travel -- straight through an 18.4 mm bolt
            # head. Real hardware stalls a finite-torque motor instead. Left at the inherited
            # value by default so nothing changes silently; GRIP_KP/GRIP_MAXF make it testable.
            _gkp = os.environ.get("GRIP_KP")
            _gmf = os.environ.get("GRIP_MAXF")
            fidx = [names.index(j) for j in ("finger_left_joint", "finger_right_joint")
                    if j in names]
            if _gkp and fidx:
                kps[fidx] = float(_gkp)
                kds[fidx] = float(os.environ.get("GRIP_KD", float(_gkp) * 1e-2))
            art.get_articulation_controller().set_gains(kps=kps, kds=kds)
            if _gmf and fidx:
                # Effort limits are the more physical knob (a real motor stalls), but the API
                # differs across Isaac versions. Never let it kill a run silently or otherwise;
                # GRIP_KP alone is enough to bound the force via penetration x stiffness.
                try:
                    eff = np.asarray(art.get_max_efforts(), dtype=np.float32).reshape(-1)
                    eff[fidx] = float(_gmf)
                    art.set_max_efforts(eff)
                except Exception as exc:                      # noqa: BLE001
                    print(f"  [grip] max-effort unavailable ({exc}); using GRIP_KP only")
            art.apply_action(ArticulationAction(joint_positions=q))

        poses = bolt_poses(args.layout, args.n_per_color, ep_seed)
        colors = [c for c, *_ in poses]
        for (color, x, y, yaw), path in zip(poses, bolt_prims):
            v = bolt_views[path]
            qz = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
            start_z = PICK_SURFACE_Z + (HEAD_R + 0.001 if 'size_m' in WORK_SURFACE else SHAFT_R + 0.002)
            v.set_world_poses(np.array([[x, y, start_z]]), np.array([qz]))
            v.set_velocities(np.zeros((1, 6)))
        for i, path in enumerate(bolt_prims):
            UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path + "/shaft")).Bind(
                MAT["bolt_gray" if colors[i] == "gray" else "bolt_black"])
            UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path + "/head")).Bind(
                MAT["bolt_gray" if colors[i] == "gray" else "bolt_black"])
        frozen = None
        if args.scene_states:
            frozen = _SCENE_STATES.get(str(ep_seed))
            if frozen is None:
                _die(f"ABORT: --scene-states has no entry for seed {ep_seed}; regenerate "
                     "it with --dump-scene-states for this layout/n-per-color")
        if frozen is not None:
            # Exact settled world: position + orientation + zero velocity per bolt.
            for path, st in zip(bolt_prims, frozen["bolts"]):
                v = bolt_views[path]
                v.set_world_poses(np.array([st["p"]], dtype=float),
                                  np.array([st["q"]], dtype=float))
                v.set_velocities(np.zeros((1, 6)))
            world.step(render=False)
        else:
            for _ in range(240):
                world.step(render=False)
        # T1 scene difficulty is a property of the CONDITION, so measure it on the settled
        # START scene. Measuring after the episode (as this first did) let the arms disturb the
        # pile, and the same seed reported different crowding in two runs.
        if ORACLE_RETURN:
            # Mid-task scene: k bolts per colour already sit in their boxes, as they would
            # after the first cycle(s). Settled first, then moved, then a short re-settle.
            rng_pp = np.random.default_rng(ep_seed * 13)
            for col_, side_ in (("gray", +1), ("black", -1)):
                k = int(rng_pp.integers(0, 3))
                idx = [i for i, c in enumerate(colors) if c == col_][:k]
                for j, i2 in enumerate(idx):
                    bolt_views[bolt_prims[i2]].set_world_poses(
                        positions=np.array([[0.66 + 0.04 * j,
                                             BOX_CY[col_] + rng_pp.uniform(-0.06, 0.06),
                                             0.05]]),
                        orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
            for _ in range(120):
                world.step(render=False)
        B0 = bolt_xyz()
        # A bolt that settles inside a box was never picked or placed -- it was born there.
        # Counting it inflated every arm's score (0.05/ep aligned, 0.40/ep random) and was
        # enough on its own to produce the bogus "random scores higher than aligned" reading.
        pre_in_box = {
            path for i, path in enumerate(bolt_prims)
            if any(abs(B0[i][0] - BOX_X) <= BOX["hw"] and abs(B0[i][1] - cy) <= BOX["hd"]
                   for cy in BOX_CY.values())}
        if pre_in_box:
            print(f"[ep {ep:02d}] {len(pre_in_box)} bolt(s) settled inside a box -> excluded")
        crowd = {}
        for col in set(colors):
            idx = [i for i, c in enumerate(colors) if c == col]
            if len(idx) > 1:
                P = B0[idx][:, :2]
                D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=-1)
                np.fill_diagonal(D, np.inf)
                crowd[col] = float(np.median(D.min(axis=1)))
        if args.dump_scene_states:
            P, Q = bolt_all_poses()
            _DUMP[str(ep_seed)] = dict(
                layout=args.layout, colors=colors, work_surface=WORK_SURFACE,
                bolts=[dict(p=[float(x) for x in P[i]], q=[float(x) for x in Q[i]])
                       for i in range(len(bolt_prims))])
            print(f"  [freeze] seed {ep_seed}: captured {len(bolt_prims)} bolt poses")
            continue
        # Warm the renderer BEFORE the first observation. The per-tick orchestrator step runs
        # at the END of a tick, so without this the very first _observe() reads unfilled
        # annotators and the episode's first chunk is computed from black images. Every run
        # before 2026-08-19 had that defect (harmless-looking: one bad 0.8 s chunk at reset).
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=1, pause_timeline=False)
        if ep == 0:
            for i, path in enumerate(bolt_prims):
                bp = np.asarray(bolt_views[path].get_world_poses()[0])[0]
                print(f"  bolt {i} {colors[i]:5s} -> ({bp[0]:+.3f},{bp[1]:+.3f},{bp[2]:+.3f})")

        # ---- follower + command state ---------------------------------------
        followers, cmd_hist, grip_cmd, q_cmd = {}, {}, {}, {}
        q_hist = {}
        smds = {}
        _sent_q = {}
        plan_gate = {s_: 1.0 for s_ in MOUNT_FRAME}     # safety.plan_gate, per arm
        plan_clock = {s_: 0.0 for s_ in MOUNT_FRAME}    # fractional chunk index, paced by it
        plan_shift = {s_: np.zeros(3) for s_ in MOUNT_FRAME}   # contact hold-back
        proj_run = {s_: 0 for s_ in MOUNT_FRAME}
        stall_run = {s_: 0 for s_ in MOUNT_FRAME}   # consecutive substeps of shortfall        # consecutive projection violations
        proj_fault = {s_: 0 for s_ in MOUNT_FRAME}      # what actually reached the drives this substep, per arm
        for side in MOUNT_FRAME:
            if ORACLE_RETURN and side in _RET_Q:
                q_cmd[side] = _RET_Q[side].copy()
            else:
                q_cmd[side] = (np.zeros(6) if side == ORACLE_PARKED
                               else np.deg2rad(np.array(RESET[side], dtype=float)))
            p_fk, R_fk, _, _ = fk_chain(q_cmd[side], T_mount[side])
            p_ph, _ = tcp_pose(side)
            d = float(np.linalg.norm(p_fk - p_ph))
            if ep == 0:
                print(f"  FK check {side:5s}: analytic vs physics TCP = {d*1000:.3f} mm")
            p, R = tcp_pose(side)
            pose6 = np.concatenate([p, mat_to_rotvec(R)])
            followers[side] = RuckigArmFollower(pose6)
            plan_gate[side], plan_clock[side] = 1.0, 0.0
            plan_shift[side], proj_run[side], stall_run[side] = np.zeros(3), 0, 0
            smds[side] = (FollowerOutputSmd(SMD_NF_LIN, SMD_NF_ANG, SMD_ZETA, SMD_FF,
                                            SMD_FF_LPF) if OUTPUT_SMD else None)
            if smds[side] is not None:
                smds[side].reset(pose6, np.zeros(6))
            q_hist[side] = []          # box FIFO: q_ref at the 500 Hz servo tick
            cmd_hist[side] = [pose6.copy(), pose6.copy()]
            grip_cmd[side] = 100.0

        frames = []
        n_ticks = int(args.episode_sec / POLICY_DT)
        chunk = {s: None for s in MOUNT_FRAME}
        grip_hist = {s: [] for s in MOUNT_FRAME}     # commanded grip per policy tick
        ep_start_pose = {}
        for s_ in ("left", "right"):
            p0_, R0_, _, _ = fk_chain(q_cmd[s_], T_mount[s_])
            ep_start_pose[s_] = (p0_.copy(), R0_.copy())
        if oracle is not None:
            oracle.__init__()                 # planner state is per-episode
        chunk_idx = 0
        knots = {s_: None for s_ in MOUNT_FRAME}
        tcp_log = {s_: [] for s_ in MOUNT_FRAME}
        sub_i = -1
        dg = {s_: {k: [] for k in ("dq", "res", "smin", "grip", "refz", "cmdtcp", "q", "gq",
                                   "qact", "qdact", "knot", "ref", "refpre")}
              for s_ in MOUNT_FRAME}
        rtc_prev_raw = None      # reset per episode, like _rtc_prev_raw_chunk
        pending = None           # (full chunk (H,14), obs_tick) kicked at prefetch_at, swapped at boundary
        prev_grip = {s_: 100.0 for s_ in MOUNT_FRAME}
        close_events = []        # per gripper-close command: TCP-vs-nearest-bolt error, grasp outcome
        grasp_checks = []        # [event_idx, bolt_idx, bolt_path, bolt_z0, side, deadline, latched]
        # T1: per tick, index of the bolt nearest this arm's 24-step-ahead chunk endpoint.
        # Changes in this series ARE target switches -- the "bolt hopping" the multimodal
        # conditional would produce at a chunk boundary.
        target_track = {s_: [] for s_ in MOUNT_FRAME}
        _bxy = None
        held = {}        # bolt path -> (side, grasp tick); who is carrying what
        placed_at = {}   # bolt path -> first box entry (time, arm, transport duration)
        eject_watch = []  # [close idx, bolt idx, due tick, max displacement, pos at close]
        # PICK vs TRANSPORT. Operator observation from the oracle videos: bolts are picked and
        # then dropped on the way to the box, and the arm shakes near the box where the IK is
        # ill-conditioned. Placements therefore mix "did it grasp" with "did it survive the
        # carry", and the two rank the arms differently (up to 5 places apart on the current
        # board). Track the lift directly off bolt height so the pick stage can be read alone;
        # a bolt above LIFT_Z was carried, whatever the close-event detector thought.
        LIFT_Z = PICK_SURFACE_Z + 0.060
        bolt_max_z = np.zeros(len(bolt_prims))
        smin_log = []
        held_last = {}   # bolt path -> most recent (arm, grasp tick)
        blank_obs = {s_: 0 for s_ in MOUNT_FRAME}   # empty wrist frames fed to the policy
        rtc_warned = False
        infer_ms = []
        _prev_frames = []   # (tick, imgs) at each _observe; obs-2 support

        for tick in range(n_ticks):
            # ---- observation (velproprio_source=command, fixed_step) ---------
            # velocity_grip proprio, exactly the training converter's _arm_velocity
            # (openpi_remote._proprio_state_velocity):
            #     pos_vel = R_cur^T . (p_next - p_cur)      BODY frame, previous pose
            #     rot_vel = rotvec(R_cur^T . R_next)
            # It is a PER-STEP displacement in the previous body frame -- NOT divided by
            # dt (the fixed_step window already is one ~policy_dt frame), and NOT a world
            # vector. Dividing by dt fed the policy ~30x inflated numbers. R_align maps
            # the RB TCP frame to the EE (pika tip) frame the checkpoint trained in.
            state = np.zeros(14, dtype=np.float32)
            for bi, side in enumerate(("left", "right")):
                h = cmd_hist[side]
                p_cur, p_next = h[-2][:3], h[-1][:3]
                R_cur, R_next = rotvec_to_mat(h[-2][3:]), rotvec_to_mat(h[-1][3:])
                pos_vel = R_cur.T @ (p_next - p_cur)
                rot_vel = mat_to_rotvec(R_cur.T @ R_next)
                if STATE_MODE == "posegrip":
                    # reset-relative pose in the episode-start COMMAND frame (deploy contract
                    # is command-sourced proprio), mapped to the tip frame via R_ALIGN.
                    p0_, R0_ = ep_start_pose[side]
                    pc_, Rc_, _, _ = fk_chain(q_cmd[side], T_mount[side])
                    state[bi * 7 + 0: bi * 7 + 3] = R_ALIGN @ (R0_.T @ (pc_ - p0_))
                    state[bi * 7 + 3: bi * 7 + 6] = R_ALIGN @ mat_to_rotvec(R0_.T @ Rc_)
                else:
                    state[bi * 7 + 0: bi * 7 + 3] = R_ALIGN @ pos_vel
                    state[bi * 7 + 3: bi * 7 + 6] = R_ALIGN @ rot_vel
                # GRIPPER PROPRIO SOURCE. The deployed runner defaults to `actual` -- the
                # MEASURED jaw, which lags its command by 105-209 ms on hardware. This rig has
                # always sent the COMMAND, which has zero lag, so the grip channel the policy
                # sees here is not the channel it was trained on. That matters more than it
                # looks: the measured grip-echo pathology means the model largely predicts
                # "current grip - epsilon", so feeding it its own command closes the loop on
                # itself. Left on `command` by default so eleven arms of history stay
                # comparable; GRIP_PROPRIO=actual switches to the deploy contract.
                if GRIP_PROPRIO == "actual":
                    names = list(arts[side].dof_names)
                    q_now = np.asarray(arts[side].get_joint_positions()).reshape(-1)
                    fp = np.mean([abs(float(q_now[names.index(j)]))
                                  for j in ("finger_left_joint", "finger_right_joint")
                                  if j in names])
                    state[bi * 7 + 6] = float(np.clip(1.0 - fp / FINGER_TRAVEL_M, 0.0, 1.0))
                else:
                    state[bi * 7 + 6] = grip_cmd[side] / 100.0

            def _observe():
                imgs = {}
                for side in ("left", "right"):
                    a = wrist_cams[side].get_rgba()
                    arr = np.asarray(a) if a is not None else None
                    if arr is None or arr.size == 0:
                        # NEVER let this pass quietly: a black observation is not a policy
                        # result, it is a broken harness, and it scores 0 while looking real.
                        # Until 2026-09-04 only the FIRST blank aborted; a blank appearing
                        # later was counted, flagged in the summary, and the run carried on
                        # scoring the policy on black frames for the rest of the episode.
                        # Operator rule: loud error and exit, not a marker on a finished
                        # number -- by the time anyone reads the marker the mp4 and the
                        # score already exist and look normal.
                        blank_obs[side] += 1
                        _die(f"ABORT: the {side} wrist camera returned an empty frame at "
                             f"episode {ep}, tick {tick}. The policy would be scored on a "
                             f"black image.\n"
                             f"  The renderer is not filling its annotators -- check that "
                             f"rep.orchestrator.step() runs every policy tick (it must not "
                             f"be gated on --video; see CLAUDE.md section 20).")
                    imgs[side] = arr[..., :3].astype(np.uint8)
                if DUMP_OBS and tick % 30 == 0:
                    for side in ("left", "right"):
                        imageio.imwrite(
                            pathlib.Path(DUMP_OBS) / f"obs_{args.tag}_{ep:02d}_{tick:04d}_{side}.png",
                            imgs[side])
                obs = {
                    "observation/left_wrist_0_rgb": imgs["left"],
                    "observation/right_wrist_0_rgb": imgs["right"],
                    "observation/state": state,
                    "prompt": PROMPT,
                }
                if OBS_PREV:
                    tgt = tick - OBS_PREV_DELTA
                    pimgs = (min(_prev_frames, key=lambda e_: abs(e_[0] - tgt))[1]
                             if _prev_frames else imgs)
                    obs["observation/left_wrist_prev_rgb"] = pimgs["left"]
                    obs["observation/right_wrist_prev_rgb"] = pimgs["right"]
                    _prev_frames.append((tick, imgs))
                    del _prev_frames[:-12]
                if RTC_ENABLED and rtc_prev_raw is not None:
                    # rtc_shift_prev_chunk: advance the cached raw chunk by the steps that
                    # will have executed when the new chunk takes over, so the freeze pins
                    # to the UNEXECUTED tail rather than replaying old actions.
                    steps = int(max(0, min(CHUNK_EXECUTE_STEPS, rtc_prev_raw.shape[0])))
                    if ACTION_MODE == "anchored" and steps > 0 and _RTC_NORM_Q is None:
                        pass   # no valid re-anchor possible: vanilla beats a corrupted freeze
                    elif ACTION_MODE == "anchored" and steps > 0:
                        # anchored rows are transforms rel the OLD chunk anchor; the freeze must
                        # pin rows re-expressed rel the row that will be the NEW anchor state:
                        # T'_k = T_s^-1 T_{k+s}, per arm -- computed in UNNORMALIZED space
                        # (rtc_prev_raw is normalized model space; see RTC_NORM_STATS above).
                        q01_, q99_ = _RTC_NORM_Q
                        un_ = _rtc_unnorm(rtc_prev_raw[:, :14], q01_, q99_)
                        shifted_un = un_[steps:].copy()
                        for b in (0, 7):
                            ps_ = un_[steps - 1, b:b+3]
                            Rs_ = rotvec_to_mat(un_[steps - 1, b+3:b+6])
                            for k_ in range(shifted_un.shape[0]):
                                pk_ = un_[steps + k_, b:b+3]
                                Rk_ = rotvec_to_mat(un_[steps + k_, b+3:b+6])
                                shifted_un[k_, b:b+3] = Rs_.T @ (pk_ - ps_)
                                shifted_un[k_, b+3:b+6] = mat_to_rotvec(Rs_.T @ Rk_)
                        shifted = rtc_prev_raw[steps:].copy()
                        shifted[:, :14] = _rtc_renorm(shifted_un, q01_, q99_)
                        pad = np.zeros((steps, rtc_prev_raw.shape[1]), dtype=rtc_prev_raw.dtype)
                        obs["prev_action_chunk"] = np.concatenate([shifted, pad], axis=0)
                    else:
                        pad = np.zeros((steps, rtc_prev_raw.shape[1]), dtype=rtc_prev_raw.dtype)
                        obs["prev_action_chunk"] = np.concatenate([rtc_prev_raw[steps:], pad], axis=0)
                    obs["inference_delay"] = int(np.clip(RTC_INFERENCE_DELAY, 0,
                                                         CHUNK_EXECUTE_STEPS))
                return obs

            def _infer(obs):
                nonlocal rtc_prev_raw, rtc_warned
                t0 = time.time()
                if oracle is not None:
                    # Same (H,14) contract the server returns: [left 7 | right 7].
                    B = bolt_xyz()
                    out = np.zeros((ACTION_HORIZON, 14), dtype=float)
                    # The parked arm must not PLAN either: its virtual planner was cycling
                    # grip ramps and polluting the close-event stream with ~30 phantom closes
                    # per run (found via the P_R5/P_RL0 side histogram). Hold it open.
                    if ORACLE_PARKED is not None:
                        pi_ = ("left", "right").index(ORACLE_PARKED)
                        out[:, pi_ * 7 + 6] = 1.0
                    for bi_o, side_o in enumerate(("left", "right")):
                        if side_o == ORACLE_PARKED:
                            continue
                        p_a, R_a, _, _ = fk_chain(q_cmd[side_o], T_mount[side_o])
                        anchor = np.concatenate([p_a, mat_to_rotvec(R_a)])
                        out[:, bi_o * 7: bi_o * 7 + 7] = oracle.chunk(
                            side_o, anchor, B, set(placed_at))
                    infer_ms.append((time.time() - t0) * 1e3)
                    return out
                res = client.infer(obs)
                infer_ms.append((time.time() - t0) * 1e3)
                if RTC_ENABLED:
                    raw = res.get("rtc_raw_actions")
                    if raw is not None:
                        rtc_prev_raw = np.asarray(raw, dtype=np.float32)
                    elif not rtc_warned:
                        print("  [rtc] server returned no 'rtc_raw_actions' -> staying vanilla")
                        rtc_warned = True
                return np.asarray(res["actions"], dtype=float)   # (H, 14)

            def _activate(act):
                for bi, side in enumerate(("left", "right")):
                    chunk[side] = act[:, bi * 7: bi * 7 + 7]

            if chunk["left"] is None:
                # first chunk of the episode: synchronous, no elapsed rows (runner does the same)
                _activate(_infer(_observe()))
                chunk_idx = 0
                if sum(blank_obs.values()):
                    # Belt and braces. _observe() now aborts on ANY blank frame, so this
                    # cannot fire; it stays because the first observation is the one that
                    # used to be checked and losing the check outright would be a
                    # regression if _observe's guard is ever loosened.
                    _die(
                        "ABORT: the wrist cameras returned empty frames on the FIRST observation "
                        "-- the policy would be scored on black images. The renderer is not "
                        "filling its annotators; check that rep.orchestrator.step() runs every "
                        "policy tick (it must not be gated on --video).")
            elif chunk_idx >= CHUNK_EXECUTE_STEPS:
                # boundary: swap in the prefetched plan, aligned by the ticks that elapsed
                # since its observation (flow_inference._activate_chunk source_start_index)
                if pending is None:
                    act, obs_tick = _infer(_observe()), tick   # stall fallback (never with sync infer)
                else:
                    act, obs_tick = pending
                    pending = None
                drop = int(np.clip(tick - obs_tick, 0, act.shape[0] - CHUNK_EXECUTE_STEPS))
                _activate(act[drop:])
                chunk_idx = 0
            if pending is None and chunk_idx == PREFETCH_AT:
                # kick the next inference now; it is swapped in at the next boundary
                pending = (_infer(_observe()), tick)

            # ---- integrate ee_local delta onto the COMMAND anchor -------------
            # flow_inference._target_payload_for_arm RE-ANCHORS at every chunk
            # boundary (steps_since_boundary == 0) instead of free-running the
            # accumulator. Without this the command drifts away from the robot --
            # measured here as x 0.359 -> 0.224 m over 4 s with 150-428 mm tracking
            # error. chunk_anchor="command" makes the anchor FK(q_sent).
            if chunk_idx == 0:
                for side in ("left", "right"):
                    p_a, R_a, _, _ = fk_chain(q_cmd[side], T_mount[side])
                    anchor_pose = np.concatenate([p_a, mat_to_rotvec(R_a)])
                    plan_clock[side] = 0.0        # new chunk, plan clock restarts with it
                    if VELPROPRIO_ANCHOR_OVERWRITE:
                        # historical behaviour: the anchor lands in the velproprio history
                        cmd_hist[side][-1] = anchor_pose
                    # Integrate the whole horizon into absolute knots so the follower can
                    # look ahead; knots[0] is the anchor itself.
                    ks = [anchor_pose.copy()]
                    cur_p, cur_R = anchor_pose[:3].copy(), rotvec_to_mat(anchor_pose[3:])
                    if ACTION_MODE == "anchored":
                        # each row composes INDEPENDENTLY onto the anchor -- no chaining, no
                        # within-chunk integration drift. Spacing between consecutive knots is
                        # still clamped so the follower never receives an infeasible jump.
                        pa, Ra = anchor_pose[:3], rotvec_to_mat(anchor_pose[3:])
                        prev = anchor_pose.copy()
                        for r in chunk[side]:
                            dl = R_ALIGN @ (np.asarray(r[:3]) * SPEED_SCALE)
                            da = R_ALIGN @ (np.asarray(r[3:6]) * SPEED_SCALE)
                            tp = pa + Ra @ dl
                            tR = Ra @ rotvec_to_mat(da)
                            step = tp - prev[:3]
                            nl = np.linalg.norm(step)
                            if nl > LIN_V * POLICY_DT:
                                tp = prev[:3] + step * (LIN_V * POLICY_DT / nl)
                            # ROTATION spacing, the twin of the position clamp above. The
                            # delta branch below has always clamped both; this branch clamped
                            # only position, so anchored chunks handed the follower knot-to-knot
                            # rotations the follower cannot execute (ANG_V*dt = 1.72 deg).
                            # Measured 2026-09-03: 3-39% of anchored knot steps exceeded that,
                            # p99 up to 10.2 deg, and the commanded attitude churned 13-63 deg/s
                            # against 9.6-9.9 deg/s on the real robot's own follower_replay knots
                            # (where p99 is 1.2-1.4 deg and >1.8 deg is 0.1-0.3% -- the signature
                            # of exactly this clamp). Saturating the follower's angular limit is
                            # what makes the arm churn while the tool tracks.
                            dR = mat_to_rotvec(rotvec_to_mat(prev[3:]).T @ tR)
                            na_ = float(np.linalg.norm(dR))
                            if na_ > ANG_V * POLICY_DT:
                                tR = rotvec_to_mat(prev[3:]) @ rotvec_to_mat(
                                    dR * (ANG_V * POLICY_DT / na_))
                            k_ = np.concatenate([tp, mat_to_rotvec(tR)])
                            ks.append(k_)
                            prev = k_
                        knots[side] = ks
                        continue
                    for r in chunk[side]:
                        dl = R_ALIGN @ r[:3] * SPEED_SCALE
                        da = R_ALIGN @ r[3:6] * SPEED_SCALE
                        nl, na = np.linalg.norm(dl), np.linalg.norm(da)
                        if nl > LIN_V * POLICY_DT:
                            dl = dl * (LIN_V * POLICY_DT / nl)
                        if na > ANG_V * POLICY_DT:
                            da = da * (ANG_V * POLICY_DT / na)
                        cur_p = cur_p + cur_R @ dl
                        cur_R = cur_R @ rotvec_to_mat(da)
                        ks.append(np.concatenate([cur_p, mat_to_rotvec(cur_R)]))
                    knots[side] = ks
            _bxy = bolt_xyz()                     # T1: all bolt positions, once per tick
            bolt_max_z = np.maximum(bolt_max_z, _bxy[:, 2])
            # Jacobian conditioning of the CURRENT command, once per policy tick. The place
            # boxes sit near the far edge of the reach envelope, so this is where an
            # ill-conditioned solve would shake the arm and shed the bolt.
            for _s in ("left", "right"):
                _p, _, _, _ = fk_chain(q_cmd[_s], T_mount[_s])
                smin_log.append((_s, float(np.linalg.svd(jacobian(q_cmd[_s], T_mount[_s]),
                                                         compute_uv=False)[-1]),
                                 float(np.linalg.norm(_p[:2] - np.array([BOX_X, BOX_CY[
                                     'gray' if _s == 'left' else 'black']])))))
            for side in ("left", "right"):
                ks = knots[side]
                # PLAN CLOCK, per arm. `chunk_idx` still counts policy ticks (it drives the
                # replan boundary, which is a wall-clock event), but WHICH KNOT this arm is
                # aiming at now advances at the rate the plan gate allows. While an arm is
                # obstructed its chunk stops flowing, so the command cannot outrun it -- and
                # the two arms pace independently, as two per-arm followers do on the robot.
                if PLAN_GATE_ENABLE:
                    plan_clock[side] += plan_gate[side]
                    k = min(int(plan_clock[side]) + 1, len(ks) - 1)
                else:
                    k = min(chunk_idx + 1, len(ks) - 1)      # knot for this policy step
                new_pose = ks[k]
                # central difference over the reserve lookahead -> target velocity
                lo = max(0, k - 1)
                hi = min(len(ks) - 1, k + min(RESERVE_STEPS, 1))
                tvel = (ks[hi] - ks[lo]) / (max(1, hi - lo) * POLICY_DT)
                tvel[:3] = np.clip(tvel[:3], -LIN_V, LIN_V)
                tvel[3:] = np.clip(tvel[3:], -ANG_V, ANG_V)
                # Only a genuine three-point stencil defines a second difference; at the data
                # edge the follower decelerates into the tail, which is af = 0 (the C++ clamps
                # its flanking indices the same way).
                tacc = ((ks[hi] - 2.0 * ks[k] + ks[lo]) / (POLICY_DT ** 2)
                        if (hi - lo) == 2 else None)
                cmd_hist[side].append(new_pose.copy())
                if len(cmd_hist[side]) > 64:
                    cmd_hist[side].pop(0)
                followers[side].set_target(new_pose, tvel,
                                           neighbors=(ks[lo], ks[hi]),
                                           span_dt=max(1, hi - lo) * POLICY_DT,
                                           acc=tacc)
                if DIAG:
                    # The 30 Hz knot the follower is aiming at, i.e. the POLICY's own path
                    # after chunk integration and re-anchoring but BEFORE Ruckig and IK.
                    # Logged so a wiggle can be attributed to the model output rather than
                    # to the rig's follower.
                    dg[side]["knot"].append(np.asarray(new_pose, dtype=np.float64).copy())
                row = chunk[side][min(chunk_idx, chunk[side].shape[0] - 1)]
                # GRIPPER LEAD. UMI records (pose_k, grip_k) as one synchronous measurement of a
                # human hand: there is no command/actual split and no latency, so grip_k means
                # "the jaw opening at the instant the tool is at pose_k". Reproducing that on a
                # robot needs the two to ARRIVE together, and they do not: the arm realises a
                # command in ~133 ms while the jaw takes 105 ms (left) and 209 ms (right). The
                # jaw command therefore has to lead the paired arm command by (L_grip - L_arm),
                # which is +2.3 steps on the right and -0.8 on the left. Issuing them on the same
                # tick -- what every run so far has done -- lands the right jaw ~76 ms late, and
                # the right arm is exactly the one with the documented 78% half-open pathology.
                # This is an EXECUTION fix, not a training one: the data contains no latency to
                # learn from.
                gi = int(np.clip(chunk_idx + GRIP_LEAD[side], 0, chunk[side].shape[0] - 1))
                grip_cmd[side] = float(np.clip(
                    chunk[side][gi][6] * 100.0 - GRIP_BIAS[side], 0.0, 100.0))
                # PRIVILEGED capture-funnel probe (GRIP_FLOOR, sim-only). Mined verdict: 84% of
                # contact entries arrive with the jaw already narrowed to <35% (capture margin
                # +-0-4 mm vs 15-18 mm aim scatter) and convert at 5-10%; the 16% that arrive
                # >=35% open convert at 55%. This gate forces the jaw open while still ABOVE
                # the nearest bolt, to causally test that premature narrowing -- not aim, not
                # depth -- is the pick bottleneck. Uses ground-truth bolt height, so it is an
                # instrument, not a deployable fix.
                if GRIP_FLOOR > 0.0:
                    p_gf, _ = tcp_pose(side)
                    if p_gf[0] < 0.56 and _bxy is not None:
                        j_gf = int(np.argmin(np.linalg.norm(_bxy[:, :2] - p_gf[:2], axis=1)))
                        if (p_gf[2] - _bxy[j_gf][2]) > 0.020:
                            grip_cmd[side] = max(grip_cmd[side], GRIP_FLOOR)
                grip_hist[side].append(grip_cmd[side])
                # ---- T1: which bolt is this arm's 24-step-ahead intent pointing at? ----
                # knots[-1] is the far end of the integrated chunk, i.e. the policy's stated
                # intent for the next 24 steps -- a much cleaner target read than the current
                # TCP, which lags it by the whole horizon.
                ep_xy = np.asarray(knots[side][-1][:3])[:2]
                target_track[side].append(
                    int(np.argmin(np.linalg.norm(_bxy[:, :2] - ep_xy, axis=1))))

                # ---- gripper-close command: where is the TCP relative to the nearest bolt?
                def _jaw_bolt(R, i):
                    b_ang, b_z = bolt_axis_xy(i)
                    jaw = R @ np.array([1.0, 0.0, 0.0])
                    d = abs(float(np.degrees(np.arctan2(jaw[1], jaw[0]))) - b_ang) % 180.0
                    return dict(bolt_axis_deg=b_ang, bolt_axis_z=b_z,
                                # folded to [0,90]: a cylinder and a two-jaw gripper are both
                                # 180-deg symmetric, so 10 deg and 170 deg are the same grasp.
                                jaw_bolt_deg=min(d, 180.0 - d))

                if prev_grip[side] >= 40.0 and grip_cmd[side] < 40.0:
                    p_t, R_t = tcp_pose(side)
                    d_all = np.linalg.norm(_bxy[:, :2] - p_t[:2], axis=1)
                    order = np.argsort(d_all)
                    bi_ = int(order[0])
                    bj_ = int(order[1]) if len(order) > 1 else bi_
                    bp, bq = _bxy[bi_], _bxy[bj_]
                    d1, d2 = float(d_all[bi_]), float(d_all[bj_])
                    # Mode-average signature: does the aim sit BETWEEN the two candidates?
                    # t_seg is the projection of the TCP onto the b1->b2 segment (0 = on the
                    # nearest bolt, 1 = on the second); perp is the offset from that line.
                    u = (bq - bp)[:2]
                    w = p_t[:2] - bp[:2]
                    gap = float(np.linalg.norm(u))
                    if gap > 1e-6:
                        t_seg = float(np.dot(w, u) / (gap * gap))
                        perp = float(abs(u[0] * w[1] - u[1] * w[0]) / gap)
                    else:
                        t_seg, perp = 0.0, 0.0
                    tgt_color = [c for c, a_ in ARM_OF_COLOR.items() if a_ == side][0]
                    same = [i for i, c in enumerate(colors) if c == tgt_color]
                    d_tc = float(np.min(d_all[same])) if same else float("nan")
                    trk = target_track[side]
                    def _switches(n):
                        seg = trk[-n:]
                        return int(sum(1 for a_, b_ in zip(seg, seg[1:]) if a_ != b_))
                    lead = 0
                    for q_ in range(len(trk) - 1, 0, -1):
                        if trk[q_ - 1] != trk[-1]:
                            break
                        lead += 1
                    ev = dict(side=side, tick=tick, t=tick * POLICY_DT,
                              tcp=[float(v) for v in p_t],
                              # Aim error has only ever been measured in XY. Roll/pitch drift
                              # would tilt the jaws off the bolt axis and is invisible there.
                              tcp_rotvec=[float(v) for v in mat_to_rotvec(R_t)],
                              **_jaw_bolt(R_t, bi_),
                              # Bolts OTHER than the target sitting inside the jaw sweep
                              # volume at the moment of closing. The jaws sweep x in +-55 mm
                              # (open half-gap 47 + finger body), are ~28 mm half-wide in y,
                              # and act near the tip plane in z. A neighbour in that box gets
                              # struck: the observed double-grasps and blocked closes.
                              # Was the DESCENT blocked? bolt_elev = target sits on other
                              # bolts instead of the table; z_cmd_gap = physics held the TCP
                              # above where the command wanted it (fingers resting on a
                              # neighbour or the pile). Either way the jaws close higher than
                              # intended, and dz alone cannot tell WHY.
                              bolt_elev_mm=float((bp[2] - (PICK_SURFACE_Z + 0.0091)) * 1e3),
                              z_cmd_gap_mm=float((p_t[2] - fk_chain(
                                  q_cmd[side], T_mount[side])[0][2]) * 1e3),
                              neighbors_in_jaw=int(sum(
                                  1 for j_ in range(len(_bxy)) if j_ != bi_
                                  and abs((v_ := R_t.T @ (_bxy[j_] - p_t))[0]) < 0.055
                                  and abs(v_[1]) < 0.028 and abs(v_[2]) < 0.035)),
                              bolt=[float(v) for v in bp], bolt_color=colors[bi_],
                              target_color=tgt_color,
                              dx=float(p_t[0] - bp[0]), dy=float(p_t[1] - bp[1]),
                              dz=float(p_t[2] - bp[2]), dxy=d1,
                              # --- T1 multimodality fields ---
                              d2=d2, margin=d2 - d1, pair_gap=gap,
                              t_seg=t_seg, perp=perp, second_color=colors[bj_],
                              d_target_color=d_tc,
                              nearest_is_target_color=bool(colors[bi_] == tgt_color),
                              switches_1s=_switches(30), switches_2s=_switches(60),
                              commit_lead_ticks=lead,
                              grasped=None)
                    close_events.append(ev)
                    # 2.5 s, not 1.5: the event fires when grip crosses 40%, and a partial
                    # opening (GRIP_OPEN=50) crosses ~17 ticks sooner relative to the lift, so
                    # at +45 the lift had barely begun and every real grasp scored False
                    # (orcP0: grasped 0/22 while PLACING 8 -- the placements are the proof).
                    # tick+75 is now a DEADLINE, not the sampling instant: the window below is
                    # polled every tick and latches on the first success. Carries the bolt INDEX
                    # so the poll can read the already-fetched `_bxy` instead of a per-bolt round
                    # trip. Fields: (event, bolt_idx, prim, z_at_close, side, deadline, latched).
                    grasp_checks.append([len(close_events) - 1, bi_, bolt_prims[bi_],
                                         float(bp[2]), side, tick + 75, False])
                    # EJECTION probe. The finger joints run the same kp=1e7 position drive as
                    # the arm, and grip 0 commands them 47 mm inward -- straight through an
                    # 18.4 mm bolt head. A real gripper stalls its motor at finite force; this
                    # one can shoot the bolt out. Watch the target bolt's speed for the next
                    # 0.5 s: a held bolt tracks the hand (<0.3 m/s), an ejected one does not.
                    eject_watch.append([len(close_events) - 1, bi_, tick + 15,
                                        float(np.linalg.norm(_bxy[bi_] - bp)), bp.copy()])
                prev_grip[side] = grip_cmd[side]
            if PROBE_DUMP and tick % PROBE_EVERY == 0 and _bxy is not None:
                # `state` is the exact 14-D vector handed to the policy this tick. Storing it makes
                # the dump REPLAYABLE offline as single-shot infer() calls, which is what separates
                # "the sim pixels are out of distribution" from "the closed loop drifts": same
                # image, same state, no loop.
                rec = dict(ep=ep, seed=ep_seed, tick=tick, t=tick * POLICY_DT,
                           state=[float(v) for v in state])
                for side in ("left", "right"):
                    a = wrist_cams[side].get_rgba()
                    arr = np.asarray(a) if a is not None else None
                    if arr is None or arr.size == 0:
                        continue
                    fn = f"pr_{args.tag}_{ep:02d}_{tick:04d}_{side}.png"
                    if PROBE_NOIMG:
                        fn = None
                    else:
                        imageio.imwrite(pathlib.Path(PROBE_DUMP) / fn, arr[..., :3].astype(np.uint8))
                    p_t, R_t = tcp_pose(side)
                    d_all = np.linalg.norm(_bxy[:, :2] - p_t[:2], axis=1)
                    # ALL bolts, not the nearest three: the probe that reads these has to work in
                    # clutter, and a per-patch bolt/no-bolt label needs every bolt or the
                    # unlabelled ones become false negatives.
                    order = [int(i) for i in np.argsort(d_all)]
                    rec[side] = dict(
                        img=fn,
                        tcp=[float(v) for v in p_t],
                        tcp_rotvec=[float(v) for v in mat_to_rotvec(R_t)],
                        grip=float(grip_cmd[side]),
                        endpoint=[float(v) for v in np.asarray(knots[side][-1][:3])],
                        bolts_world=[[float(v) for v in _bxy[i]] for i in order],
                        bolts_tool=[[float(v) for v in (R_t.T @ (_bxy[i] - p_t))] for i in order],
                        bolt_colors=[colors[i] for i in order],
                        # The (H,7) rows the knots above were integrated from. With this, an
                        # offline replay can tell "the policy emitted a different chunk" apart
                        # from "the chunk->knots conversion lost the motion" -- the two are
                        # indistinguishable from `endpoint` alone.
                        chunk=[[float(v) for v in row] for row in np.asarray(chunk[side])],
                    )
                with open(pathlib.Path(PROBE_DUMP) / f"probe_{args.tag}.jsonl", "a") as fh:
                    fh.write(json.dumps(rec) + "\n")

            # resolve the ejection probe: peak displacement of the bolt since the close
            still_e = []
            for w in eject_watch:
                w[3] = max(w[3], float(np.linalg.norm(_bxy[w[1]] - w[4])))
                if tick >= w[2]:
                    close_events[w[0]]["bolt_move_mm"] = w[3] * 1e3
                else:
                    still_e.append(w)
            eject_watch = still_e

            # resolve pending grasp checks: did the nearest bolt come up with the gripper?
            #
            # POLLED, not sampled at the deadline. Reading the condition only at close+2.5 s
            # scored a SUCCESSFUL pick-and-place as "not grasped": measured transport is 0.0-0.4 s,
            # so by +2.5 s the bolt is already in the box (low z) with the jaw reopened, and both
            # terms of the test are false. Worse, `held` was populated at the deadline while the
            # placement loop below reads it every tick, so the box entry was always recorded
            # BEFORE the grasp resolved -> grasp_t=None. Together these made the two arms that
            # actually pick look identical to arms that only shove bolts across the table
            # (measured: 15 placements, 0 grasp-linked). Latching on the first tick the bolt is
            # up-and-held fixes detection and attribution at once. The deadline survives as the
            # negative timeout, and achieved_gap_mm is still read there so its meaning is
            # unchanged.
            still = []
            for w in grasp_checks:
                ei, bi_g, path, z0, side_, due, latched = w
                if not latched and _bxy[bi_g][2] - z0 > 0.03 and grip_cmd[side_] < 40.0:
                    latched = w[6] = True
                    close_events[ei]["grasped"] = True
                    close_events[ei]["grasp_tick"] = int(tick)
                    # Remember who is carrying what, so a later box entry can be
                    # attributed to an arm and to a transport duration.
                    # Keep BOTH: a bolt that is dropped and re-grasped overwrites the
                    # latest entry, which made transport_s collapse to ~0.2 s. The first
                    # grasp measures the whole attempt, the last the actual carry.
                    held.setdefault(path, (side_, tick))
                    held_last[path] = (side_, tick)
                if tick >= due:
                    if not latched:
                        close_events[ei]["grasped"] = False
                    # What the jaw PHYSICALLY did 2.5 s after the command: 0 = fully shut
                    # (closed on nothing or crushed through), ~28 = stalled on a bolt head,
                    # large = never closed (jammed on the floor or a neighbour). grip_cmd
                    # cannot tell these apart; the measured finger joints can.
                    _nm = list(arts[side_].dof_names)
                    _q = np.asarray(arts[side_].get_joint_positions()).reshape(-1)
                    _fp = np.mean([abs(float(_q[_nm.index(j)]))
                                   for j in ("finger_left_joint", "finger_right_joint")
                                   if j in _nm])
                    # FINGER_TRAVEL_M, not a second literal 0.047: this is the same jaw
                    # model as set_arm and the grip proprio, and the v15 tip moves it to
                    # 0.049. A stale copy here would have reported every gap 4 mm too
                    # narrow -- straight through the 18.4 mm stall signature this field
                    # exists to read.
                    close_events[ei]["achieved_gap_mm"] = float(
                        2.0 * (FINGER_TRAVEL_M - _fp) * 1e3)
                else:
                    still.append(w)
            grasp_checks = still

            # ---- placement TIMING -------------------------------------------------
            # A count of placements says nothing about how long the robot took to get them.
            # Record the FIRST tick each bolt enters a box; that gives time-to-first-success,
            # the interval between successes (cycle time) and, via `held`, the grasp->place
            # transport time per arm. Costs nothing: _bxy is already read every tick.
            for bi_, path in enumerate(bolt_prims):
                if path in placed_at or path in pre_in_box:
                    continue
                p = _bxy[bi_]
                for col, cy in BOX_CY.items():
                    if abs(p[0] - BOX_X) <= BOX["hw"] and abs(p[1] - cy) <= BOX["hd"]:
                        side_, gt = held.get(path, (None, None))
                        side_l, gl = held_last.get(path, (None, None))
                        placed_at[path] = dict(
                            bolt=bi_, bolt_color=colors[bi_], box_color=col,
                            t=tick * POLICY_DT, tick=tick, side=(side_l or side_),
                            grasp_t=(gt * POLICY_DT if gt is not None else None),
                            attempt_s=((tick - gt) * POLICY_DT if gt is not None else None),
                            transport_s=((tick - gl) * POLICY_DT if gl is not None else None))
                        break
            chunk_idx += 1

            # ---- servo: Ruckig reference -> IK -> drives ----------------------
            for _ in range(SUBSTEPS):
                sub_i += 1
                for side in ("left", "right"):
                    if side == ORACLE_PARKED:
                        q_cmd[side] = np.zeros(6)
                        set_arm(side, q_cmd[side], 100.0)
                        continue
                    followers[side].step()
                    ref_pre = followers[side].reference
                    ref = (smds[side].step(ref_pre, followers[side].reference_velocity,
                                           PHYSICS_DT)
                           if smds[side] is not None else ref_pre)
                    # OBSTRUCTION PROJECTION. Everything downstream of here sees the clamped
                    # command; `removed` is what the box or the shell took away this tick.
                    _p_req = ref[:3] - plan_shift[side]
                    _p_fin, _removed = project_command(_p_req, T_mount[side][:3, 3])
                    ref = np.concatenate([_p_fin, ref[3:]])
                    # LEAD CLAMP. Scale the plan-ahead-of-robot offset back to max_lead, the
                    # way scaleLinearTo does. Applied to the pose the IK will chase, so the
                    # joint command -- and therefore FK(q_cmd), the chunk anchor -- inherits
                    # the bound and the anchor cannot walk away from the arm.
                    if LEAD_CLAMP_ENABLE:
                        _act_p, _act_R = tcp_pose(side)
                        _lead = ref[:3] - _act_p
                        _ln = float(np.linalg.norm(_lead))
                        if _ln > MAX_LEAD_M:
                            ref = np.concatenate([_act_p + _lead * (MAX_LEAD_M / _ln),
                                                  ref[3:]])
                        _lr = mat_to_rotvec(rotvec_to_mat(ref[3:]) @ _act_R.T)
                        _rn = float(np.linalg.norm(_lr))
                        if _rn > MAX_LEAD_RAD:
                            ref = np.concatenate([
                                ref[:3],
                                mat_to_rotvec(rotvec_to_mat(_lr * (MAX_LEAD_RAD / _rn))
                                              @ _act_R)])
                    # CONTACT HOLD-BACK. Accumulate the blocked component so the plan stops
                    # advancing into the face instead of winding up against it. Bounded to one
                    # segment of travel at the linear limit, as the C++ does: a larger removal
                    # encodes stale lag, not fresh contact.
                    if HOLDBACK_ENABLE and PLAN_GATE_SOURCE == "contact":
                        # Into-contact direction from the tracking error: where the command is
                        # and where the tool actually is. Same role as the F/T triad's contact
                        # normal on the robot. Scaled by its OWN persistence, not by the plan
                        # gate -- tying it to (1 - gate) meant PLAN_GATE_ENABLE=0 silently
                        # disabled the hold-back too, and the A/B that was supposed to isolate
                        # the gate measured neither layer (2026-09-05).
                        _err_v = ref[:3] - tcp_pose(side)[0]
                        _en = float(np.linalg.norm(_err_v))
                        if _en > 0.002 and stall_run[side] >= PLAN_GATE_PERSIST:
                            _cap = LIN_V * PHYSICS_DT
                            plan_shift[side] = plan_shift[side] + _err_v * min(1.0, _cap / _en)
                        else:
                            plan_shift[side] *= 0.999
                    elif HOLDBACK_ENABLE:
                        _n = float(np.linalg.norm(_removed))
                        if _n > 1e-12:
                            _cap = LIN_V * POLICY_DT
                            plan_shift[side] = plan_shift[side] + (
                                _removed * (_cap / _n) if _n > _cap else _removed)
                        else:
                            plan_shift[side] *= 0.999   # bleed off once the face releases
                    qc = q_cmd[side]
                    p_fk, R_fk, _, _ = fk_chain(qc, T_mount[side])
                    err = np.concatenate([ref[:3] - p_fk,
                                          mat_to_rotvec(rotvec_to_mat(ref[3:]) @ R_fk.T)])
                    # IK BUDGET. `ik.timeout_ms: 20.0` on the robot bounds the solve; this rig
                    # had no bound, so once the command ran away from the arm every substep
                    # spent all 100 SVD iterations and a 30 s episode took over 10 minutes
                    # (measured 2026-09-05 with the guards off). Iterations are the portable
                    # form of that timeout -- a wall-clock one would make runs nondeterministic.
                    _n_it = IK_MAX_ITERS if IK_MODE == "real" else IK_ITERS
                    if IK_MODE == "real" and np.linalg.norm(err[:3]) > IK_FAR_M:
                        _n_it = IK_FAR_ITERS
                    for _it in range(_n_it):
                        J = jacobian(qc, T_mount[side])
                        if IK_MODE == "real":
                            U_, S_, Vt_ = np.linalg.svd(J)
                            lam2 = np.full(6, IK_DAMPING * IK_DAMPING)
                            if IK_SINGULAR_EPS > 0.0 and IK_DAMPING_MAX > 0.0:
                                r_ = S_ / IK_SINGULAR_EPS
                                lam2 = lam2 + np.where(
                                    S_ < IK_SINGULAR_EPS,
                                    IK_DAMPING_MAX * IK_DAMPING_MAX * (1.0 - r_ * r_), 0.0)
                            dq = Vt_.T @ ((S_ / (S_ * S_ + lam2)) * (U_.T @ err))
                            dq = np.clip(dq, -IK_MAX_STEP, IK_MAX_STEP)
                        else:
                            dq = J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA * np.eye(6), err)
                        qc = np.clip(qc + np.clip(dq, -DQ_MAX, DQ_MAX), JOINT_LO, JOINT_HI)
                        p_fk, R_fk, _, _ = fk_chain(qc, T_mount[side])
                        err = np.concatenate([ref[:3] - p_fk,
                                              mat_to_rotvec(rotvec_to_mat(ref[3:]) @ R_fk.T)])
                        if (IK_MODE == "real" and (_it + 1) >= IK_MIN_ITERS
                                and np.linalg.norm(err[:3]) < IK_POS_TOL
                                and np.linalg.norm(err[3:]) < IK_ORI_TOL):
                            break
                    # PLAN GATE. requested = the IK solution as it stands, final = the same
                    # after the obstruction projection above already shrank the target it was
                    # solving toward. On the robot these are two commands; here the projection
                    # is the only thing that can separate them, which is the point -- a free
                    # arm leaves the gate at 1 and only an obstruction closes it.
                    # stall_run is maintained WHETHER OR NOT the gate is enabled: the
                    # hold-back reads it too, and the whole point of the knob is to A/B one
                    # layer at a time.
                    if PLAN_GATE_SOURCE == "contact":
                        _qa0 = np.asarray(
                            arts[side].get_joint_positions()).reshape(-1)[_ARM_IDX[side]]
                        _hq0 = q_hist[side]
                        _L0 = PLAN_GATE_LAG_SUBSTEPS
                        _r0 = _hq0[max(0, len(_hq0) - _L0)] if _hq0 else qc
                        _p0 = (_hq0[max(0, len(_hq0) - _L0 - 1)]
                               if len(_hq0) > _L0 else _r0)
                        stall_run[side] = (stall_run[side] + 1
                                           if plan_gate_ratio(_r0, _qa0, _p0) < PLAN_GATE_ENGAGE
                                           else 0)
                    if PLAN_GATE_ENABLE:
                        if PLAN_GATE_SOURCE == "contact":
                            _qa = np.asarray(
                                arts[side].get_joint_positions()).reshape(-1)[_ARM_IDX[side]]
                            _hq = q_hist[side]
                            _L = PLAN_GATE_LAG_SUBSTEPS
                            # requested/prev are the command the drive is chasing NOW, i.e.
                            # one drive lag back; realized is what the joints did.
                            _req = _hq[max(0, len(_hq) - _L)] if _hq else qc
                            _prv = _hq[max(0, len(_hq) - _L - 1)] if len(_hq) > _L else _req
                            plan_gate[side] = plan_gate_step(
                                plan_gate[side], _req, _qa, _prv,
                                persist_ok=stall_run[side] >= PLAN_GATE_PERSIST)
                        else:
                            _q_free = qc + (
                                np.zeros(6) if np.linalg.norm(_removed) < 1e-12
                                else jacobian(qc, T_mount[side]).T @ np.linalg.solve(
                                    jacobian(qc, T_mount[side])
                                    @ jacobian(qc, T_mount[side]).T
                                    + IK_DAMPING * IK_DAMPING * np.eye(6),
                                    np.concatenate([_removed, np.zeros(3)])))
                            plan_gate[side] = plan_gate_step(plan_gate[side], _q_free, qc,
                                                             q_cmd[side])
                    # PROJECTION-ERROR FAULT. How far the realized command is from the knot
                    # the follower was aiming at. The robot latches infeasible_fault after
                    # PROJ_ERR_N consecutive violations; this rig used to diverge in silence.
                    _perr = float(np.linalg.norm(ref[:3] - fk_chain(qc, T_mount[side])[0]))
                    if _perr > PROJ_ERR_M:
                        proj_run[side] += 1
                        if proj_run[side] == PROJ_ERR_N:
                            proj_fault[side] += 1
                    else:
                        proj_run[side] = 0
                    q_cmd[side] = qc
                    if _REP is not None:
                        gi = min(sub_i, len(_REP[f"{side}_q"]) - 1)
                        q_cmd[side] = _REP[f"{side}_q"][gi].copy()
                        grip_cmd[side] = float(_REP[f"{side}_gq"][gi])
                    # The JOINTS get the delayed value; grip_cmd stays the command, which is
                    # what the close event, the proprio "command" mode and the logs mean.
                    lag_t = int(round(GRIP_LAG_MS[side] / 1000.0 / POLICY_DT))
                    h = grip_hist[side]
                    g_apply = h[max(0, len(h) - 1 - lag_t)] if h else grip_cmd[side]
                    # The DRIVES get the delayed joint command; q_cmd stays the command, which
                    # is what chunk_anchor=command, the velocity proprio and the logs mean --
                    # the same split the grip lag above uses, and the same one the robot has
                    # (the host anchors on what it sent, not on what the box has replayed yet).
                    qh = q_hist[side]
                    qh.append(q_cmd[side].copy())
                    if len(qh) > max(BOX_DELAY_TICKS, PLAN_GATE_LAG_SUBSTEPS) + 3:
                        qh.pop(0)
                    q_apply = qh[max(0, len(qh) - 1 - BOX_DELAY_TICKS)]
                    set_arm(side, q_apply, g_apply)
                    _sent_q[side] = q_apply
                    if DIAG:
                        # sigma_min exposes a near-singular wrist, where DLS damping turns
                        # a small Cartesian error into a large, sign-flipping joint step.
                        dg[side]["dq"].append(float(np.abs(dq).max()))
                        dg[side]["res"].append(float(np.linalg.norm(err[:3])))
                        dg[side]["smin"].append(
                            float(np.linalg.svd(J, compute_uv=False)[-1]))
                        dg[side]["grip"].append(grip_cmd[side])
                        dg[side]["refz"].append(float(ref[2]))
                        # The decisive split: COMMANDED tcp (analytic FK of the joint
                        # command) vs MEASURED tcp. If commanded is smooth while measured
                        # shakes, the tremor is physics/contact, not the controller.
                        dg[side]["ref"].append(np.asarray(ref, dtype=np.float64).copy())
                        dg[side]["refpre"].append(np.asarray(ref_pre, dtype=np.float64).copy())
                        dg[side]["cmdtcp"].append(
                            fk_chain(q_cmd[side], T_mount[side])[0].copy())
                        dg[side]["q"].append(q_cmd[side].copy())
                        dg[side]["gq"].append(grip_cmd[side])
                world.step(render=False)
                if DRIVE_MODE == "kinematic":
                    # Re-assert AFTER the step. Writing the state before world.step() is not
                    # enough: the step still integrates one dt of velocity, gravity, drive and
                    # contact on top of it, which left FK(q_actual) jerk at 645 um against
                    # FK(q_ref) 35 um -- i.e. not the kinematic arm this mode is supposed to
                    # be. Re-asserting here makes q_actual == q_ref exactly, which is the
                    # whole point of the diagnostic; the step before it is what lets the
                    # fingers and the bolts still see the arm.
                    for side in ("left", "right"):
                        if side in _sent_q:
                            _art = arts[side]
                            _qa = np.asarray(_art.get_joint_positions(), dtype=np.float32).copy()
                            _qv = np.asarray(_art.get_joint_velocities(), dtype=np.float32).copy()
                            _ix = _ARM_IDX[side]
                            _qa[_ix] = np.asarray(_sent_q[side], dtype=np.float32)
                            _qv[_ix] = 0.0
                            _art.set_joint_positions(_qa)
                            _art.set_joint_velocities(_qv)
                # Sample at the PHYSICS rate, not the policy rate. The tremor lives INSIDE
                # the 33 ms knot interval (accelerate then brake across 17 substeps);
                # sampling once per knot aliases it away completely, which is why the first
                # version of this metric was blind to the very thing it was measuring.
                for side in ("left", "right"):
                    tcp_log[side].append(tcp_pose(side)[0].copy())
                    if DIAG:
                        # q_ref vs q_actual, sampled AFTER the step so it lines up with the
                        # measured TCP above. The commanded side (dg["q"]) is written before
                        # the step, so comparing the two raw streams costs one substep of
                        # skew; _ARM_IDX makes the joint read cheap enough to do at 500 Hz.
                        _art = arts[side]
                        _qa = np.asarray(_art.get_joint_positions()).reshape(-1)
                        _qv = np.asarray(_art.get_joint_velocities()).reshape(-1)
                        _ix = _ARM_IDX[side]
                        dg[side]["qact"].append(_qa[_ix].astype(np.float64).copy())
                        dg[side]["qdact"].append(_qv[_ix].astype(np.float64).copy())

            if ep == 0 and tick % 30 == 0:
                dbg = []
                for side in ("left", "right"):
                    p_ph, _ = tcp_pose(side)
                    ref = followers[side].reference[:3]
                    cmd = cmd_hist[side][-1][:3]
                    dbg.append(f"{side[0]}: cmd=({cmd[0]:+.3f},{cmd[1]:+.3f},{cmd[2]:+.3f}) "
                               f"track_err={np.linalg.norm(ref - p_ph)*1000:5.1f}mm "
                               f"grip={grip_cmd[side]:5.1f} gate={plan_gate[side]:4.2f} "
                               f"hold={np.linalg.norm(plan_shift[side])*1e3:4.0f}mm")
                print(f"  t{tick:04d} " + " | ".join(dbg))

            # The WRIST cameras are replicator render products too: their annotators only fill
            # when the orchestrator steps. This step must therefore run whether or not video is
            # being recorded. Gating it on --video (as this did until 2026-08-19) left
            # Camera.get_rgba() empty, _observe() silently substituted a black image, and ten
            # std20 episodes scored 0/0 while looking like a policy result.
            rep.orchestrator.step(rt_subframes=1, pause_timeline=False)
            if ov_ann is not None:
                fr = np.asarray(ov_ann.get_data())
                if not fr.size:                       # one retry, as before
                    rep.orchestrator.step(rt_subframes=1, pause_timeline=False)
                    fr = np.asarray(ov_ann.get_data())
                if fr.size:
                    im = fr[..., :3].astype(np.uint8)
                    if top_ann is not None:
                        tf = np.asarray(top_ann.get_data())
                        if tf.size:
                            im = np.concatenate([im, tf[..., :3].astype(np.uint8)], axis=1)
                    frames.append(im)

        # ---- score ------------------------------------------------------------
        placed, misplaced = 0, 0
        for i, path in enumerate(bolt_prims):
            if path in pre_in_box:
                continue
            pos, _ = bolt_views[path].get_world_poses()
            p = np.asarray(pos)[0]
            for col, cy in BOX_CY.items():
                inside = (abs(p[0] - BOX_X) <= BOX["hw"]) and (abs(p[1] - cy) <= BOX["hd"])
                if inside:
                    if col == colors[i]:
                        placed += 1
                    else:
                        misplaced += 1
        tremor, knotband, tremor_box = {}, {}, {}
        for side in ("left", "right"):
            P = np.asarray(tcp_log[side])
            if len(P) > 64:
                d2 = np.diff(P, n=2, axis=0)
                # micrometres per physics step^2
                tremor[side] = float(np.sqrt((d2 ** 2).sum(axis=1).mean()) * 1e6)
                # The operator sees the arm shake NEAR THE BOX specifically, where the
                # Jacobian conditioning collapses (smin ~2e-4). Bucket the same jerk metric
                # by region so "the controller trembles" stops being one number that averages
                # a calm pile approach with a shaking place.
                nb = P[1:-1, 0] > 0.60
                if nb.sum() > 64:
                    tremor_box[side] = float(np.sqrt((d2[nb] ** 2).sum(axis=1).mean()) * 1e6)
                # how much of the speed signal sits at the 30 Hz knot rate and its harmonic
                spd = np.linalg.norm(np.diff(P, axis=0), axis=1) / PHYSICS_DT
                spd = spd - spd.mean()
                freq = np.fft.rfftfreq(len(spd), PHYSICS_DT)
                mag = np.abs(np.fft.rfft(spd)) ** 2
                tot = mag[freq > 0.5].sum()
                f0 = 1.0 / POLICY_DT
                band = ((np.abs(freq - f0) < 3.0) | (np.abs(freq - 2 * f0) < 3.0))
                knotband[side] = float(mag[band].sum() / tot * 100.0) if tot > 0 else 0.0
        print("          tremor RMS |d2 TCP| : "
              + "  ".join(f"{k}={v:.1f} um" for k, v in tremor.items())
              + " | knot-rate energy: "
              + "  ".join(f"{k}={v:.1f}%" for k, v in knotband.items()))
        if DIAG:
            np.savez(outdir / f"diag_{args.tag}_{ep:02d}.npz",
                     **{f"{sd}_{k}": np.asarray(v)
                        for sd, d in dg.items() for k, v in d.items()},
                     **{f"{sd}_tcp": np.asarray(tcp_log[sd]) for sd in dg})
            print(f"          diag -> diag_{args.tag}_{ep:02d}.npz")
        for w in grasp_checks:   # unresolved at episode end: never latched inside the window
            close_events[w[0]]["grasped"] = bool(w[6])
        n_close = len(close_events)
        n_grasp = sum(1 for e in close_events if e["grasped"])
        # ---- T1: target-track compression ---------------------------------------------
        # (crowding was measured on the START scene, above)
        def _rle(tr):
            out = []
            for i, v in enumerate(tr):
                if not out or out[-1][1] != v:
                    out.append([i, int(v)])
            return out
        tswitch = {s_: max(0, len(_rle(target_track[s_])) - 1) for s_ in MOUNT_FRAME}
        rec = dict(episode=ep, seed=ep_seed, layout=args.layout, bolts=n_bolts,
                   rtc=RTC_ENABLED, execute_steps=CHUNK_EXECUTE_STEPS, prefetch_at=PREFETCH_AT,
                   tremor_um=tremor, knot_energy_pct=knotband,
                   placed_correct=placed, placed_wrong=misplaced,
                   pre_placed=len(pre_in_box),
                   lifted=int(sum(1 for i, pth in enumerate(bolt_prims)
                                  if pth not in pre_in_box and bolt_max_z[i] > LIFT_Z)),
                   dropped=int(sum(1 for i, pth in enumerate(bolt_prims)
                                   if pth not in pre_in_box and bolt_max_z[i] > LIFT_Z
                                   and pth not in placed_at)),
                   smin_p10={_s: (float(np.percentile([v for t, v, d in smin_log if t == _s], 10))
                                  if any(t == _s for t, _, _ in smin_log) else None)
                             for _s in ("left", "right")},
                   tremor_box_um=tremor_box,
                   smin_p10_near_box={_s: (float(np.percentile(
                       [v for t, v, d in smin_log if t == _s and d < 0.15], 10))
                       if any(t == _s and d < 0.15 for t, _, d in smin_log) else None)
                       for _s in ("left", "right")},
                   close_events=close_events, n_close=n_close, n_grasp=n_grasp,
                   same_color_crowding_m=crowd,
                   placements=sorted(placed_at.values(), key=lambda e: e["tick"]),
                   blank_obs=dict(blank_obs),
                   target_switches=tswitch,
                   target_track_rle={s_: _rle(target_track[s_]) for s_ in MOUNT_FRAME},
                   infer_ms_p50=float(np.median(infer_ms)) if infer_ms else None,
                   infers=len(infer_ms))
        if close_events:
            dxy = np.array([e["dxy"] for e in close_events]) * 1e3
            dz = np.array([e["dz"] for e in close_events]) * 1e3
            print(f"          close cmds={n_close} grasped={n_grasp} | TCP-vs-nearest-bolt at close: "
                  f"|dxy| p50 {np.median(dxy):.1f} mm, dz p50 {np.median(dz):+.1f} mm")
            mg = np.array([e["margin"] for e in close_events]) * 1e3
            ts = np.array([e["t_seg"] for e in close_events])
            pp = np.array([e["perp"] for e in close_events]) * 1e3
            btw = int(np.sum((ts > 0.15) & (ts < 0.85) & (pp < 40.0)))
            wrong = int(np.sum([not e["nearest_is_target_color"] for e in close_events]))
            print(f"          T1 ambiguity: margin p50 {np.median(mg):.0f} mm "
                  f"(p10 {np.percentile(mg, 10):.0f}) | between-candidates {btw}/{n_close} | "
                  f"wrong-colour-nearest {wrong}/{n_close} | switches/ep "
                  f"L{tswitch['left']} R{tswitch['right']} | crowding "
                  + " ".join(f"{k}={v*1e3:.0f}mm" for k, v in sorted(crowd.items())))
        results.append(rec)
        print(f"[ep {ep:02d}] correct={placed} wrong={misplaced} "
              f"infer_p50={rec['infer_ms_p50']:.0f}ms n_infer={len(infer_ms)}")

        if args.video and frames:
            vp = outdir / f"{args.tag}_{args.layout}_{ep:02d}.mp4"
            imageio.mimwrite(vp, frames, fps=30, quality=6)
            print(f"          video {vp.name} ({len(frames)} frames)")

    _ev = [e for r in results for e in r["close_events"]]
    def _p50(vals, scale=1.0):
        return float(np.median(vals)) * scale if len(vals) else None
    t1 = dict(
        close_events=len(_ev),
        margin_p50_mm=_p50([e["margin"] for e in _ev], 1e3),
        margin_p10_mm=(float(np.percentile([e["margin"] for e in _ev], 10)) * 1e3
                       if _ev else None),
        # aim sits between two candidates and off both: the mode-average signature
        between_candidates=sum(1 for e in _ev
                               if 0.15 < e["t_seg"] < 0.85 and e["perp"] < 0.040),
        wrong_colour_nearest=sum(1 for e in _ev if not e["nearest_is_target_color"]),
        dxy_p50_mm=_p50([e["dxy"] for e in _ev], 1e3),
        dz_p50_mm=_p50([e["dz"] for e in _ev], 1e3),
        switches_1s_p50=_p50([e["switches_1s"] for e in _ev]),
        commit_lead_ticks_p50=_p50([e["commit_lead_ticks"] for e in _ev]),
        target_switches_per_episode={
            s: _p50([r["target_switches"][s] for r in results]) for s in ("left", "right")},
    )
    if args.dump_scene_states:
        pathlib.Path(args.dump_scene_states).write_text(json.dumps(_DUMP, indent=1))
        print(f"[freeze] wrote {len(_DUMP)} seeds -> {args.dump_scene_states}")
        app.close()
        return 0
    summary = dict(
        protocol=args.protocol, label=args.label or args.tag,
        scene_states=args.scene_states,
        # Runs before 2026-08-22 counted bolts that SETTLED inside a box as placements. The
        # frozen sets make that a constant per protocol (aligned +3 correct over 40 eps, random
        # +9 correct / +5 wrong), so old summaries stay correctable -- this flag is how
        # t1_report tells which ones still need the correction applied.
        pre_placed_excluded=True,
        policy=f"{args.host}:{args.port}",
        episodes=args.episodes, layout=args.layout,
        seeds=[r["seed"] for r in results], episode_sec=args.episode_sec,
        total_correct=sum(r["placed_correct"] for r in results),
        total_wrong=sum(r["placed_wrong"] for r in results),
        episodes_with_any_correct=sum(1 for r in results if r["placed_correct"] > 0),
        rtc=RTC_ENABLED, execute_steps=CHUNK_EXECUTE_STEPS, prefetch_at=PREFETCH_AT,
        total_close=sum(r["n_close"] for r in results),
        total_grasp=sum(r["n_grasp"] for r in results),
        # Placements that came from a CONFIRMED grasp. `total_correct` above only asks where the
        # bolt ended up, so a policy that shoves bolts across the table into the box scores the
        # same as one that picks them -- measured on a real arm: 15 placements, 0 grasp-linked.
        # Rank models on this, not on total_correct.
        total_grasp_linked=sum(1 for r in results for p in r["placements"]
                               if p.get("grasp_t") is not None),
        # Always 0 in a run that finished: a blank frame now aborts on the spot. Kept as
        # the field older summaries are read through, and as the check that would catch a
        # future path which counts a blank without raising.
        blank_obs=sum(v for r in results for v in r["blank_obs"].values()),
        t1=t1,
        provenance=_safe_provenance(args, server_metadata),
        records=results,
    )
    if summary["blank_obs"]:
        print(f"!! INVALID RUN: {summary['blank_obs']} blank wrist observations were fed to "
              f"the policy. Do not report these numbers.")
    (outdir / f"summary_{args.tag}_{args.layout}.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "records"}, indent=1))
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
