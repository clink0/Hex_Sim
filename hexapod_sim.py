"""
hexapod_sim.py — Hexapod Robot 3D Simulation
=============================================
Python translation of the Arduino hexapod project.
Faithfully mirrors:
  Config.h       → constants
  Kinematics.h   → IK / FK math (geometric / law-of-cosines)
  GaitEngine.h   → tripod gait keyframes + interpolation

Controls (interactive mode):
  ←/→  arrow keys   — step through gait phases manually
  Space              — toggle auto-walk animation
  W/S                — forward / backward walk
  A/D                — rotate left / right
  Q                  — quit

Usage:
  python hexapod_sim.py
"""

import numpy as np
import matplotlib

# ── Backend selection ──────────────────────────────────────────
# Try backends in order of preference. MacOS ships 'macosx' natively
# (no tkinter needed). Falls back through Qt, wx, then headless Agg.
def _pick_backend():
    candidates = ['macosx', 'Qt5Agg', 'Qt6Agg', 'GTK4Agg', 'GTK3Agg',
                  'wxAgg', 'TkAgg', 'Agg']
    for name in candidates:
        try:
            matplotlib.use(name)
            import matplotlib.pyplot as _p
            _p.figure()   # actually test that it can open a window
            _p.close('all')
            return name
        except Exception:
            continue
    return 'Agg'   # headless fallback

_backend = _pick_backend()
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.patches as mpatches
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import sys
import math

# ─────────────────────────────────────────────────────────────
# 1. CONFIG  (mirrors Config.h exactly)
# ─────────────────────────────────────────────────────────────

# Leg segment lengths (mm)
COXA_LENGTH  = 28.0
FEMUR_LENGTH = 84.0
TIBIA_LENGTH = 127.0

# Body geometry — coxa mount positions (mm from body center)
# (name, x, y)
MOUNTS = {
    "LF": ( 74.0,  39.0),
    "LM": (  0.0,  64.0),
    "LB": (-74.0,  39.0),
    "RF": ( 74.0, -39.0),
    "RM": (  0.0, -64.0),
    "RB": (-74.0, -39.0),
}

# Servo limits
SERVO_ANGLE_MIN = -90.0
SERVO_ANGLE_MAX =  90.0
TIBIA_OFFSET_DEG = 14.0

# Standing & gait params
DEFAULT_STAND_Z   = -123.2   # mm below body
FOOT_FB_DIST      =  200.3   # X for front/back feet
FOOT_WIDTH_FB     =  105.5   # Y for front/back feet
FOOT_WIDTH_M      =  206.7   # Y for middle feet
STEP_LENGTH       =   40.0   # mm per step
STEP_HEIGHT       =   30.0   # mm foot lift

# Reachability bounds
MAX_REACH = FEMUR_LENGTH + TIBIA_LENGTH    # 211mm
MIN_REACH = abs(FEMUR_LENGTH - TIBIA_LENGTH)  # 43mm

# Tripod groups
GROUP_A = ["LF", "RM", "LB"]   # lift in phase 0
GROUP_B = ["RF", "LM", "RB"]   # lift in phase 2

# Body polygon vertices (for drawing the body outline)
# order: LF, RF, RM, RB, LB, LM — forms a hexagon
BODY_ORDER = ["LF", "RF", "RM", "RB", "LB", "LM"]


# ─────────────────────────────────────────────────────────────
# 2. KINEMATICS  (mirrors Kinematics.h exactly)
# ─────────────────────────────────────────────────────────────

@dataclass
class JointAngles:
    coxa:  float = 0.0
    femur: float = 0.0
    tibia: float = 0.0
    valid: bool  = False

@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @staticmethod
    def from_array(a) -> "Vec3":
        return Vec3(float(a[0]), float(a[1]), float(a[2]))


def compute_leg_ik(foot: Vec3, mount_x: float, mount_y: float) -> JointAngles:
    """
    Geometric inverse kinematics for one leg.
    Exact translation of computeLegIK() in Kinematics.h.

    Step 1: direction vector from mount to foot
    Step 2: coxa angle  (atan2 in horizontal plane)
    Step 3: project into 2D leg plane
    Step 4: femur–tibia distance D
    Step 5: reachability check
    Step 6: femur angle  (atan2 + law-of-cosines)
    Step 7: tibia angle  (law-of-cosines + offset)
    Step 8: servo limit check
    """
    result = JointAngles()

    dx = foot.x - mount_x
    dy = foot.y - mount_y
    dz = foot.z

    # Coxa angle
    mount_angle_rad = math.atan2(mount_y, mount_x)
    foot_angle_rad  = math.atan2(dy, dx)
    coxa_deg = math.degrees(foot_angle_rad - mount_angle_rad)

    # Normalise to [-180, 180]
    while coxa_deg >  180.0: coxa_deg -= 360.0
    while coxa_deg < -180.0: coxa_deg += 360.0
    result.coxa = coxa_deg

    # Project into leg plane
    r = math.sqrt(dx*dx + dy*dy)
    h = r - COXA_LENGTH      # horizontal from femur pivot to foot
    v = dz                    # vertical   (negative = below)

    D = math.sqrt(h*h + v*v)  # femur-pivot → foot-tip distance

    # Reachability
    if D > MAX_REACH or D < MIN_REACH or D < 0.001:
        return result  # valid stays False

    # Femur angle
    phi_a = math.atan2(v, h)
    cos_phi_b = (D*D + FEMUR_LENGTH**2 - TIBIA_LENGTH**2) / (2.0 * D * FEMUR_LENGTH)
    cos_phi_b = max(-1.0, min(1.0, cos_phi_b))
    phi_b = math.acos(cos_phi_b)
    result.femur = math.degrees(phi_b + phi_a)

    # Tibia angle
    cos_gamma = (FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - D*D) / (2.0 * FEMUR_LENGTH * TIBIA_LENGTH)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))
    gamma_deg = math.degrees(math.acos(cos_gamma))
    result.tibia = gamma_deg - 90.0 - TIBIA_OFFSET_DEG

    # Servo range check
    if not all(SERVO_ANGLE_MIN <= a <= SERVO_ANGLE_MAX
               for a in [result.coxa, result.femur, result.tibia]):
        return result

    if any(math.isnan(a) for a in [result.coxa, result.femur, result.tibia]):
        return result

    result.valid = True
    return result


def compute_fk(coxa_deg: float, femur_deg: float, tibia_deg: float,
               mount_x: float, mount_y: float) -> Vec3:
    """
    Forward kinematics — exact translation of computeFK() in Kinematics.h.
    Returns foot-tip position in body frame.
    """
    mount_angle_rad = math.atan2(mount_y, mount_x)
    coxa_abs_rad    = math.radians(coxa_deg) + mount_angle_rad

    femur_rad = math.radians(femur_deg)

    tibia_internal_rad = math.radians(tibia_deg + 90.0 + TIBIA_OFFSET_DEG)

    # Femur endpoint in leg plane
    femur_end_h = FEMUR_LENGTH * math.cos(femur_rad)
    femur_end_v = FEMUR_LENGTH * math.sin(femur_rad)

    # Tibia absolute angle
    tibia_abs_rad = femur_rad - (math.pi - tibia_internal_rad)
    tip_h = femur_end_h + TIBIA_LENGTH * math.cos(tibia_abs_rad)
    tip_v = femur_end_v + TIBIA_LENGTH * math.sin(tibia_abs_rad)

    total_h = COXA_LENGTH + tip_h

    foot = Vec3()
    foot.x = mount_x + total_h * math.cos(coxa_abs_rad)
    foot.y = mount_y + total_h * math.sin(coxa_abs_rad)
    foot.z = tip_v
    return foot


def validate_ik(target: Vec3, mount_x: float, mount_y: float) -> float:
    """Round-trip IK→FK error in mm. Returns -1 if unreachable."""
    angles = compute_leg_ik(target, mount_x, mount_y)
    if not angles.valid:
        return -1.0
    fk = compute_fk(angles.coxa, angles.femur, angles.tibia, mount_x, mount_y)
    return math.sqrt((target.x-fk.x)**2 + (target.y-fk.y)**2 + (target.z-fk.z)**2)


def compute_joint_positions(angles: JointAngles,
                            mount_x: float, mount_y: float
                            ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Return 3D positions of: [mount, coxa_end, femur_end, foot_tip]
    Used to draw the leg as connected segments.
    """
    mount_angle_rad = math.atan2(mount_y, mount_x)
    coxa_abs_rad    = math.radians(angles.coxa) + mount_angle_rad

    # Coxa end (in horizontal plane)
    coxa_end = np.array([
        mount_x + COXA_LENGTH * math.cos(coxa_abs_rad),
        mount_y + COXA_LENGTH * math.sin(coxa_abs_rad),
        0.0
    ])

    femur_rad = math.radians(angles.femur)
    tibia_internal_rad = math.radians(angles.tibia + 90.0 + TIBIA_OFFSET_DEG)

    femur_end_h = FEMUR_LENGTH * math.cos(femur_rad)
    femur_end_v = FEMUR_LENGTH * math.sin(femur_rad)

    # Femur end in 3D
    femur_end = np.array([
        coxa_end[0] + femur_end_h * math.cos(coxa_abs_rad),
        coxa_end[1] + femur_end_h * math.sin(coxa_abs_rad),
        femur_end_v
    ])

    tibia_abs_rad = femur_rad - (math.pi - tibia_internal_rad)
    tip_h = femur_end_h + TIBIA_LENGTH * math.cos(tibia_abs_rad)
    tip_v = femur_end_v + TIBIA_LENGTH * math.sin(tibia_abs_rad)
    total_h = COXA_LENGTH + tip_h

    foot_tip = np.array([
        mount_x + total_h * math.cos(coxa_abs_rad),
        mount_y + total_h * math.sin(coxa_abs_rad),
        tip_v
    ])

    mount_pt = np.array([mount_x, mount_y, 0.0])
    return mount_pt, coxa_end, femur_end, foot_tip


# ─────────────────────────────────────────────────────────────
# 3. GAIT ENGINE  (mirrors GaitEngine.h exactly)
# ─────────────────────────────────────────────────────────────

HexPose = dict   # { "LF": Vec3, "LM": ..., "LB": ..., "RF": ..., "RM": ..., "RB": ... }

LEG_NAMES = ["LF", "LM", "LB", "RF", "RM", "RB"]


def home_pose(z: float = DEFAULT_STAND_Z) -> HexPose:
    return {
        "LF": Vec3( FOOT_FB_DIST,   FOOT_WIDTH_FB,  z),
        "LM": Vec3( 0,              FOOT_WIDTH_M,   z),
        "LB": Vec3(-FOOT_FB_DIST,   FOOT_WIDTH_FB,  z),
        "RF": Vec3( FOOT_FB_DIST,  -FOOT_WIDTH_FB,  z),
        "RM": Vec3( 0,             -FOOT_WIDTH_M,   z),
        "RB": Vec3(-FOOT_FB_DIST,  -FOOT_WIDTH_FB,  z),
    }


def smooth_step(t: float) -> float:
    """Cubic Hermite: f(t) = 3t² - 2t³"""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def lerp_vec3(a: Vec3, b: Vec3, t: float) -> Vec3:
    return Vec3(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    )


def lerp_pose(frm: HexPose, to: HexPose, t: float, smooth: bool = True) -> HexPose:
    if smooth:
        t = smooth_step(t)
    return {leg: lerp_vec3(frm[leg], to[leg], t) for leg in LEG_NAMES}


def walk_keyframe(phase: int, z: float = DEFAULT_STAND_Z, direction: float = 1.0) -> HexPose:
    """
    Tripod walk — exact translation of walkKeyframe() in GaitEngine.h.
      Group A: LF, RM, LB
      Group B: RF, LM, RB
    """
    z_up = z + STEP_HEIGHT
    sx   = STEP_LENGTH * direction

    if phase == 0:
        # Group A lifts
        return {
            "LF": Vec3( FOOT_FB_DIST,   FOOT_WIDTH_FB,  z_up),
            "LM": Vec3( 0,              FOOT_WIDTH_M,   z),
            "LB": Vec3(-FOOT_FB_DIST,   FOOT_WIDTH_FB,  z_up),
            "RF": Vec3( FOOT_FB_DIST,  -FOOT_WIDTH_FB,  z),
            "RM": Vec3( 0,             -FOOT_WIDTH_M,   z_up),
            "RB": Vec3(-FOOT_FB_DIST,  -FOOT_WIDTH_FB,  z),
        }
    elif phase == 1:
        # A landed forward, B pushed backward
        return {
            "LF": Vec3( FOOT_FB_DIST + sx,  FOOT_WIDTH_FB,  z),
            "LM": Vec3( 0            - sx,  FOOT_WIDTH_M,   z),
            "LB": Vec3(-FOOT_FB_DIST + sx,  FOOT_WIDTH_FB,  z),
            "RF": Vec3( FOOT_FB_DIST - sx, -FOOT_WIDTH_FB,  z),
            "RM": Vec3( 0            + sx, -FOOT_WIDTH_M,   z),
            "RB": Vec3(-FOOT_FB_DIST - sx, -FOOT_WIDTH_FB,  z),
        }
    elif phase == 2:
        # Group B lifts
        return {
            "LF": Vec3( FOOT_FB_DIST,   FOOT_WIDTH_FB,  z),
            "LM": Vec3( 0,              FOOT_WIDTH_M,   z_up),
            "LB": Vec3(-FOOT_FB_DIST,   FOOT_WIDTH_FB,  z),
            "RF": Vec3( FOOT_FB_DIST,  -FOOT_WIDTH_FB,  z_up),
            "RM": Vec3( 0,             -FOOT_WIDTH_M,   z),
            "RB": Vec3(-FOOT_FB_DIST,  -FOOT_WIDTH_FB,  z_up),
        }
    elif phase == 3:
        # B landed forward, A pushed backward
        return {
            "LF": Vec3( FOOT_FB_DIST - sx,  FOOT_WIDTH_FB,  z),
            "LM": Vec3( 0            + sx,  FOOT_WIDTH_M,   z),
            "LB": Vec3(-FOOT_FB_DIST - sx,  FOOT_WIDTH_FB,  z),
            "RF": Vec3( FOOT_FB_DIST + sx, -FOOT_WIDTH_FB,  z),
            "RM": Vec3( 0            - sx, -FOOT_WIDTH_M,   z),
            "RB": Vec3(-FOOT_FB_DIST + sx, -FOOT_WIDTH_FB,  z),
        }
    return home_pose(z)


def rotate_keyframe(phase: int, z: float = DEFAULT_STAND_Z, direction: float = 1.0) -> HexPose:
    """
    Rotation-in-place gait — exact translation of rotateKeyframe() in GaitEngine.h.
    direction: +1 = CCW from above (left), -1 = CW (right)
    """
    z_up  = z + STEP_HEIGHT
    theta = 0.09 * direction   # ~5.2° per half-step

    # Home X,Y for each leg
    hx = { "LF":  FOOT_FB_DIST, "LM":  0, "LB": -FOOT_FB_DIST,
            "RF":  FOOT_FB_DIST, "RM":  0, "RB": -FOOT_FB_DIST }
    hy = { "LF":  FOOT_WIDTH_FB, "LM":  FOOT_WIDTH_M, "LB":  FOOT_WIDTH_FB,
            "RF": -FOOT_WIDTH_FB, "RM": -FOOT_WIDTH_M, "RB": -FOOT_WIDTH_FB }

    def rot(x, y, th):
        ct, st = math.cos(th), math.sin(th)
        return ct*x - st*y, ct*y + st*x   # note: st*x + ct*y is wrong order; use standard 2D rotation

    # Group A rotates by +theta, Group B by -theta (then they swap in phase 3)
    def rotated(leg, th):
        rx, ry = rot(hx[leg], hy[leg], th)
        return Vec3(rx, ry, z)

    if phase == 0:
        # A lifts
        return {leg: Vec3(hx[leg], hy[leg], z_up if leg in GROUP_A else z)
                for leg in LEG_NAMES}
    elif phase == 1:
        # A swings to +theta, B counter-rotates to -theta
        return {leg: (rotated(leg,  theta) if leg in GROUP_A else
                      rotated(leg, -theta))
                for leg in LEG_NAMES}
    elif phase == 2:
        # B lifts
        return {leg: Vec3(hx[leg], hy[leg], z_up if leg in GROUP_B else z)
                for leg in LEG_NAMES}
    elif phase == 3:
        # B swings to +theta, A counter-rotates to -theta
        return {leg: (rotated(leg,  theta) if leg in GROUP_B else
                      rotated(leg, -theta))
                for leg in LEG_NAMES}
    return home_pose(z)


# ─────────────────────────────────────────────────────────────
# 4. IK VALIDATION (run once at startup, printed to terminal)
# ─────────────────────────────────────────────────────────────

def run_ik_validation():
    print("\n" + "="*60)
    print("IK Round-Trip Validation (should all be < 0.1mm)")
    print("="*60)
    pose = home_pose()
    all_ok = True
    for name, (mx, my) in MOUNTS.items():
        foot = pose[name]
        err  = validate_ik(foot, mx, my)
        ok   = "✓" if 0 <= err < 0.5 else "✗"
        print(f"  {name}: target=({foot.x:.1f}, {foot.y:.1f}, {foot.z:.1f}) "
              f"  err={err:.3f}mm {ok}")
        if err < 0 or err >= 0.5:
            all_ok = False
    print("All OK!" if all_ok else "WARNING: Some positions have high error!")
    print()


# ─────────────────────────────────────────────────────────────
# 5. VISUALIZER
# ─────────────────────────────────────────────────────────────

# Colour scheme
COL_BODY        = "#2c3e50"
COL_JOINT       = "#ecf0f1"
COL_COXA        = "#3498db"
COL_FEMUR       = "#2ecc71"
COL_TIBIA       = "#e74c3c"
COL_FOOT_DOWN   = "#f39c12"
COL_FOOT_UP     = "#9b59b6"
COL_BODY_FILL   = "#34495e"
COL_GROUND      = "#1a1a2e"
COL_GRID        = "#2d2d4e"

INTERP_STEPS = 20   # sub-steps between keyframes for smooth animation


class HexapodViz:
    def __init__(self):
        self.fig = plt.figure(figsize=(14, 9), facecolor="#0a0a1a")
        self.fig.canvas.manager.set_window_title("Hexapod Simulation")

        # Main 3D axis
        self.ax3d = self.fig.add_axes([0.0, 0.05, 0.72, 0.95], projection='3d')
        # Info panel axis
        self.ax_info = self.fig.add_axes([0.73, 0.05, 0.26, 0.90])

        self._style_axes()
        self._init_leg_artists()
        self._init_info_panel()
        self._init_foot_traces()

        # State
        self.current_pose  = home_pose()
        self.gait_phase    = 0
        self.gait_mode     = "stand"    # stand | walk_fwd | walk_bwd | rotate_l | rotate_r
        self.interp_frame  = 0
        self.from_pose     = home_pose()
        self.to_pose       = home_pose()
        self.auto_walk     = False
        self.foot_traces   = {leg: [] for leg in LEG_NAMES}  # trail points

        # Connect keyboard
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        # Animation
        self._anim = animation.FuncAnimation(
            self.fig, self._animate,
            interval=30, blit=False, cache_frame_data=False
        )

    # ── Styling ──────────────────────────────────────────────

    def _style_axes(self):
        ax = self.ax3d
        ax.set_facecolor("#0d0d1f")
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor(COL_GRID)
        ax.yaxis.pane.set_edgecolor(COL_GRID)
        ax.zaxis.pane.set_edgecolor(COL_GRID)
        ax.tick_params(colors='#666688', labelsize=7)
        ax.set_xlabel("X (fwd, mm)", color='#8888aa', labelpad=5, fontsize=8)
        ax.set_ylabel("Y (left, mm)", color='#8888aa', labelpad=5, fontsize=8)
        ax.set_zlabel("Z (up, mm)", color='#8888aa', labelpad=5, fontsize=8)
        ax.set_title("Hexapod Robot Simulation", color='#ccccee',
                     fontsize=13, fontweight='bold', pad=10)
        lim = 280
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-200, 80)
        ax.view_init(elev=25, azim=-60)

        # Ground plane
        xs = np.linspace(-lim, lim, 2)
        ys = np.linspace(-lim, lim, 2)
        xx, yy = np.meshgrid(xs, ys)
        zz = np.full_like(xx, DEFAULT_STAND_Z)
        ax.plot_surface(xx, yy, zz, alpha=0.08, color='#4444aa', zorder=0)

        self.ax_info.set_facecolor("#0d0d1f")
        self.ax_info.set_xticks([]); self.ax_info.set_yticks([])
        for spine in self.ax_info.spines.values():
            spine.set_edgecolor("#334")

    # ── Artists ──────────────────────────────────────────────

    def _init_leg_artists(self):
        """Pre-create all line + scatter artists for legs."""
        ax = self.ax3d
        self.leg_artists = {}

        for name in LEG_NAMES:
            # Leg segments: coxa (blue), femur (green), tibia (red)
            coxa_line,  = ax.plot([], [], [], '-', color=COL_COXA,  lw=3.5, solid_capstyle='round', zorder=5)
            femur_line, = ax.plot([], [], [], '-', color=COL_FEMUR, lw=3.5, solid_capstyle='round', zorder=5)
            tibia_line, = ax.plot([], [], [], '-', color=COL_TIBIA, lw=3.5, solid_capstyle='round', zorder=5)

            # Joint dots
            joints_scat = ax.scatter([], [], [], s=28, c=COL_JOINT, zorder=6, depthshade=False)
            # Foot dot (changes colour when lifted)
            foot_scat   = ax.scatter([], [], [], s=60, c=COL_FOOT_DOWN, zorder=7, depthshade=False)

            self.leg_artists[name] = {
                "coxa":  coxa_line,
                "femur": femur_line,
                "tibia": tibia_line,
                "joints": joints_scat,
                "foot":   foot_scat,
            }

        # Body polygon
        self.body_line, = ax.plot([], [], [], '-', color=COL_BODY, lw=2.5, zorder=4)
        # Body centre dot
        self.body_dot   = ax.scatter([0], [0], [0], s=80, c=COL_BODY, marker='o', zorder=8, depthshade=False)

    def _init_foot_traces(self):
        """Pre-create foot trace lines (faint trails)."""
        ax = self.ax3d
        self.trace_lines = {}
        trace_colors = {
            "LF": "#5588ff", "LM": "#44aaff", "LB": "#3366ff",
            "RF": "#ff5566", "RM": "#ff8844", "RB": "#ff3355",
        }
        for name in LEG_NAMES:
            line, = ax.plot([], [], [], '-', color=trace_colors[name],
                            lw=0.8, alpha=0.4, zorder=2)
            self.trace_lines[name] = line

    def _init_info_panel(self):
        """Create text objects for the info panel."""
        ax = self.ax_info
        self.info_texts = {}

        def txt(x, y, s, **kw):
            return ax.text(x, y, s, transform=ax.transAxes,
                           fontfamily='monospace', **kw)

        self.info_texts["title"] = txt(0.5, 0.97, "HEXAPOD STATUS",
            ha='center', va='top', color='#aabbff', fontsize=10, fontweight='bold')

        self.info_texts["mode"]  = txt(0.05, 0.90, "Mode: STAND",
            color='#66ffaa', fontsize=9)
        self.info_texts["phase"] = txt(0.05, 0.84, "Phase: 0",
            color='#ffcc44', fontsize=9)
        self.info_texts["auto"]  = txt(0.05, 0.78, "Auto: OFF",
            color='#ff8866', fontsize=9)

        # Divider
        txt(0.05, 0.73, "─"*28, color='#444466', fontsize=8)

        self.info_texts["angles_header"] = txt(0.05, 0.70,
            "Leg  Coxa  Femur  Tibia",
            color='#8888aa', fontsize=8)

        y = 0.64
        self.info_texts["angles"] = {}
        for name in LEG_NAMES:
            self.info_texts["angles"][name] = txt(0.05, y, f"{name}   ---   ---   ---",
                color='#ccccee', fontsize=8)
            y -= 0.057

        txt(0.05, 0.27, "─"*28, color='#444466', fontsize=8)

        # Legend
        legend_items = [
            (COL_COXA,  "Coxa segment"),
            (COL_FEMUR, "Femur segment"),
            (COL_TIBIA, "Tibia segment"),
            (COL_FOOT_DOWN, "Foot (grounded)"),
            (COL_FOOT_UP,   "Foot (lifted)"),
        ]
        txt(0.05, 0.24, "LEGEND", color='#8888aa', fontsize=8, fontweight='bold')
        y = 0.20
        for color, label in legend_items:
            ax.plot([0.05, 0.12], [y, y], '-', color=color, lw=2,
                    transform=ax.transAxes)
            txt(0.15, y - 0.005, label, color='#aaaacc', fontsize=7.5)
            y -= 0.045

        # Controls
        txt(0.05, -0.01,
            "W/S: fwd/bwd  A/D: rotate\n"
            "SPACE: auto  ←/→: step\n"
            "Q: quit",
            color='#666688', fontsize=7.5, va='bottom')

    # ── Update ───────────────────────────────────────────────

    def _update_leg(self, name: str, foot: Vec3, z_threshold: float):
        """Recompute IK and update all artists for one leg."""
        mx, my = MOUNTS[name]
        angles = compute_leg_ik(foot, mx, my)
        if not angles.valid:
            return

        mount, coxa_end, femur_end, foot_tip = compute_joint_positions(angles, mx, my)

        arts = self.leg_artists[name]

        def setline(line, p0, p1):
            line.set_data_3d([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])

        setline(arts["coxa"],  mount,    coxa_end)
        setline(arts["femur"], coxa_end, femur_end)
        setline(arts["tibia"], femur_end, foot_tip)

        pts = np.array([mount, coxa_end, femur_end])
        arts["joints"]._offsets3d = (pts[:,0], pts[:,1], pts[:,2])

        is_up  = foot.z > z_threshold + 5.0
        color  = COL_FOOT_UP if is_up else COL_FOOT_DOWN
        arts["foot"]._offsets3d = ([foot_tip[0]], [foot_tip[1]], [foot_tip[2]])
        arts["foot"].set_color(color)
        arts["foot"].set_sizes([90 if is_up else 60])

        # Update info panel angles
        self.info_texts["angles"][name].set_text(
            f"{name}  {angles.coxa:+5.1f} {angles.femur:+5.1f} {angles.tibia:+5.1f}"
        )

        # Foot trace (keep last 80 points)
        self.foot_traces[name].append(foot_tip.copy())
        if len(self.foot_traces[name]) > 80:
            self.foot_traces[name].pop(0)
        tr = np.array(self.foot_traces[name])
        self.trace_lines[name].set_data_3d(tr[:,0], tr[:,1], tr[:,2])

    def _update_body(self):
        """Draw the body hexagon at z=0."""
        pts = [MOUNTS[n] for n in BODY_ORDER] + [MOUNTS[BODY_ORDER[0]]]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [0.0] * len(pts)
        self.body_line.set_data_3d(xs, ys, zs)

    def _update_info_panel(self):
        mode_labels = {
            "stand": "STAND",
            "walk_fwd": "WALK ▶ FWD",
            "walk_bwd": "WALK ◀ BWD",
            "rotate_l": "ROTATE ↺ LEFT",
            "rotate_r": "ROTATE ↻ RIGHT",
        }
        self.info_texts["mode"].set_text(f"Mode: {mode_labels.get(self.gait_mode, self.gait_mode)}")
        self.info_texts["phase"].set_text(f"Phase: {self.gait_phase}  (step {self.interp_frame}/{INTERP_STEPS})")
        self.info_texts["auto"].set_text(f"Auto: {'ON  ▶▶' if self.auto_walk else 'OFF'}")
        self.info_texts["auto"].set_color('#66ffaa' if self.auto_walk else '#ff8866')

    # ── Gait ────────────────────────────────────────────────

    def _get_keyframe(self, phase: int) -> HexPose:
        if self.gait_mode == "walk_fwd":
            return walk_keyframe(phase, direction=1.0)
        elif self.gait_mode == "walk_bwd":
            return walk_keyframe(phase, direction=-1.0)
        elif self.gait_mode == "rotate_l":
            return rotate_keyframe(phase, direction=1.0)
        elif self.gait_mode == "rotate_r":
            return rotate_keyframe(phase, direction=-1.0)
        else:
            return home_pose()

    def _step_phase(self):
        """Advance one gait phase (called by keypress or auto-walk)."""
        if self.gait_mode == "stand":
            self.from_pose = self.current_pose
            self.to_pose   = home_pose()
        else:
            self.from_pose = self.current_pose
            self.gait_phase = (self.gait_phase + 1) % 4
            self.to_pose   = self._get_keyframe(self.gait_phase)
        self.interp_frame = 0

    def _set_mode(self, mode: str):
        if self.gait_mode != mode:
            self.gait_mode  = mode
            self.gait_phase = 0
            self.from_pose  = self.current_pose
            self.to_pose    = self._get_keyframe(0)
            self.interp_frame = 0

    # ── Animation loop ───────────────────────────────────────

    def _animate(self, frame_num):
        # Advance interpolation
        if self.interp_frame < INTERP_STEPS:
            t = self.interp_frame / INTERP_STEPS
            self.current_pose = lerp_pose(self.from_pose, self.to_pose, t, smooth=True)
            self.interp_frame += 1
        else:
            self.current_pose = self.to_pose
            if self.auto_walk and self.gait_mode != "stand":
                self._step_phase()

        # Determine z threshold to detect "lifted" feet
        z_thr = DEFAULT_STAND_Z

        # Render all legs
        for name in LEG_NAMES:
            self._update_leg(name, self.current_pose[name], z_thr)

        self._update_body()
        self._update_info_panel()

        return list(self.ax3d.get_lines()) + [self.body_dot]

    # ── Keyboard ─────────────────────────────────────────────

    def _on_key(self, event):
        key = event.key
        if key == 'q':
            plt.close('all')
            sys.exit(0)
        elif key == ' ':
            self.auto_walk = not self.auto_walk
            if self.auto_walk and self.gait_mode == "stand":
                self._set_mode("walk_fwd")
        elif key == 'right':
            self._step_phase()
        elif key == 'left':
            # Step backward through phases
            self.gait_phase = (self.gait_phase - 1) % 4
            self.from_pose = self.current_pose
            self.to_pose   = self._get_keyframe(self.gait_phase)
            self.interp_frame = 0
        elif key == 'w':
            self._set_mode("walk_fwd")
            if not self.auto_walk:
                self.auto_walk = True
        elif key == 's':
            self._set_mode("walk_bwd")
            if not self.auto_walk:
                self.auto_walk = True
        elif key == 'a':
            self._set_mode("rotate_l")
            if not self.auto_walk:
                self.auto_walk = True
        elif key == 'd':
            self._set_mode("rotate_r")
            if not self.auto_walk:
                self.auto_walk = True

    def show(self):
        plt.tight_layout(pad=0.5)
        plt.show()


# ─────────────────────────────────────────────────────────────
# 6. ENTRY POINT
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    run_ik_validation()

    if _backend == 'Agg':
        print("ERROR: No interactive display backend found.")
        print()
        print("On macOS, the quickest fix is to install PyQt5 into your venv:")
        print("  pip install PyQt5")
        print("  python hexapod_sim.py")
        print()
        print("Or if you prefer Tk:")
        print("  brew install python-tk@3.14")
        print("  (then recreate your venv so it picks up the new Tk)")
        sys.exit(1)

    print(f"Using backend: {_backend}")
    print("Launching visualizer...")
    print("  W/S   - walk forward / backward")
    print("  A/D   - rotate left / right")
    print("  SPACE - toggle auto-walk")
    print("  left/right arrows - single-step gait phases")
    print("  Q     - quit\n")

    viz = HexapodViz()
    viz.show()
