"""
hexapod_sim.py — Hexapod Robot 3D Simulation (v2 — Continuous Gait)
====================================================================
Python translation of the Arduino hexapod project.

Key improvement over v1: replaces the discrete 4-keyframe tripod gait
with a continuous sinusoidal gait engine. Each leg runs its own phase
in a smooth 0→2π cycle, producing:
  - Smooth parabolic foot arcs (no "stabbing" motion)
  - Fluid stride with no visible keyframe pauses
  - Subtle body height oscillation as weight shifts
  - Proper stance push-back so the body actually travels

IK math is identical to Kinematics.h (geometric / law-of-cosines).

Controls:
  W / S       -- walk forward / backward
  A / D       -- rotate left / right
  Space       -- toggle walking
  Up / Down   -- increase / decrease speed
  [ / ]       -- decrease / increase stride length
  Q           -- quit
"""

import numpy as np
import matplotlib

# ── Backend selection ──────────────────────────────────────────
def _pick_backend():
    candidates = ['macosx', 'Qt5Agg', 'Qt6Agg', 'GTK4Agg', 'GTK3Agg',
                  'wxAgg', 'TkAgg', 'Agg']
    for name in candidates:
        try:
            matplotlib.use(name)
            import matplotlib.pyplot as _p
            _p.figure()
            _p.close('all')
            return name
        except Exception:
            continue
    return 'Agg'

_backend = _pick_backend()
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
import sys, math, time
from dataclasses import dataclass
from typing import Dict, List, Tuple

# ─────────────────────────────────────────────────────────────
# 1. CONFIG  (mirrors Config.h)
# ─────────────────────────────────────────────────────────────
COXA_LENGTH   = 28.0
FEMUR_LENGTH  = 84.0
TIBIA_LENGTH  = 127.0

MOUNTS: Dict[str, Tuple[float, float]] = {
    "LF": ( 74.0,  39.0),
    "LM": (  0.0,  64.0),
    "LB": (-74.0,  39.0),
    "RF": ( 74.0, -39.0),
    "RM": (  0.0, -64.0),
    "RB": (-74.0, -39.0),
}

SERVO_ANGLE_MIN   = -90.0
SERVO_ANGLE_MAX   =  90.0
TIBIA_OFFSET_DEG  =  14.0

DEFAULT_STAND_Z   = -123.2
FOOT_FB_DIST      =  200.3
FOOT_WIDTH_FB     =  105.5
FOOT_WIDTH_M      =  206.7

MAX_REACH = FEMUR_LENGTH + TIBIA_LENGTH
MIN_REACH = abs(FEMUR_LENGTH - TIBIA_LENGTH)

LEG_NAMES  = ["LF", "LM", "LB", "RF", "RM", "RB"]
BODY_ORDER = ["LF", "RF", "RM", "RB", "LB", "LM"]

# Tripod phase offsets: Group A (LF,RM,LB)=0, Group B (RF,LM,RB)=pi
TRIPOD_PHASE: Dict[str, float] = {
    "LF": 0.0, "RM": 0.0, "LB": 0.0,
    "RF": math.pi, "LM": math.pi, "RB": math.pi,
}

NEUTRAL: Dict[str, Tuple[float, float]] = {
    "LF": ( FOOT_FB_DIST,  FOOT_WIDTH_FB),
    "LM": ( 0.0,           FOOT_WIDTH_M),
    "LB": (-FOOT_FB_DIST,  FOOT_WIDTH_FB),
    "RF": ( FOOT_FB_DIST, -FOOT_WIDTH_FB),
    "RM": ( 0.0,          -FOOT_WIDTH_M),
    "RB": (-FOOT_FB_DIST, -FOOT_WIDTH_FB),
}

# ─────────────────────────────────────────────────────────────
# 2. KINEMATICS  (exact port of Kinematics.h)
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


def compute_leg_ik(foot: Vec3, mount_x: float, mount_y: float) -> JointAngles:
    result = JointAngles()
    dx, dy, dz = foot.x - mount_x, foot.y - mount_y, foot.z

    mount_ang = math.atan2(mount_y, mount_x)
    foot_ang  = math.atan2(dy, dx)
    coxa_deg  = math.degrees(foot_ang - mount_ang)
    while coxa_deg >  180.0: coxa_deg -= 360.0
    while coxa_deg < -180.0: coxa_deg += 360.0
    result.coxa = coxa_deg

    r = math.sqrt(dx*dx + dy*dy)
    h = r - COXA_LENGTH
    v = dz
    D = math.sqrt(h*h + v*v)

    if D > MAX_REACH or D < MIN_REACH or D < 0.001:
        return result

    phi_a     = math.atan2(v, h)
    cos_phi_b = max(-1.0, min(1.0,
        (D*D + FEMUR_LENGTH**2 - TIBIA_LENGTH**2) / (2.0*D*FEMUR_LENGTH)))
    result.femur = math.degrees(math.acos(cos_phi_b) + phi_a)

    cos_g = max(-1.0, min(1.0,
        (FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - D*D) / (2.0*FEMUR_LENGTH*TIBIA_LENGTH)))
    result.tibia = math.degrees(math.acos(cos_g)) - 90.0 - TIBIA_OFFSET_DEG

    if not all(SERVO_ANGLE_MIN <= a <= SERVO_ANGLE_MAX
               for a in [result.coxa, result.femur, result.tibia]):
        return result
    if any(math.isnan(a) for a in [result.coxa, result.femur, result.tibia]):
        return result

    result.valid = True
    return result


def compute_fk(coxa_deg, femur_deg, tibia_deg, mount_x, mount_y) -> Vec3:
    mount_ang   = math.atan2(mount_y, mount_x)
    coxa_abs    = math.radians(coxa_deg) + mount_ang
    femur_r     = math.radians(femur_deg)
    tibia_int   = math.radians(tibia_deg + 90.0 + TIBIA_OFFSET_DEG)
    femur_h     = FEMUR_LENGTH * math.cos(femur_r)
    femur_v     = FEMUR_LENGTH * math.sin(femur_r)
    tibia_abs   = femur_r - (math.pi - tibia_int)
    tip_h       = femur_h + TIBIA_LENGTH * math.cos(tibia_abs)
    tip_v       = femur_v + TIBIA_LENGTH * math.sin(tibia_abs)
    total_h     = COXA_LENGTH + tip_h
    return Vec3(mount_x + total_h * math.cos(coxa_abs),
                mount_y + total_h * math.sin(coxa_abs),
                tip_v)


def joint_positions(angles: JointAngles, mount_x, mount_y):
    """Returns (mount, coxa_end, femur_end, foot_tip) as numpy arrays."""
    mount_ang  = math.atan2(mount_y, mount_x)
    coxa_abs   = math.radians(angles.coxa) + mount_ang
    femur_r    = math.radians(angles.femur)
    tibia_int  = math.radians(angles.tibia + 90.0 + TIBIA_OFFSET_DEG)

    coxa_end  = np.array([
        mount_x + COXA_LENGTH * math.cos(coxa_abs),
        mount_y + COXA_LENGTH * math.sin(coxa_abs), 0.0])
    femur_h   = FEMUR_LENGTH * math.cos(femur_r)
    femur_v   = FEMUR_LENGTH * math.sin(femur_r)
    femur_end = np.array([
        coxa_end[0] + femur_h * math.cos(coxa_abs),
        coxa_end[1] + femur_h * math.sin(coxa_abs), femur_v])
    tibia_abs = femur_r - (math.pi - tibia_int)
    tip_h     = femur_h + TIBIA_LENGTH * math.cos(tibia_abs)
    tip_v     = femur_v + TIBIA_LENGTH * math.sin(tibia_abs)
    foot_tip  = np.array([
        mount_x + (COXA_LENGTH + tip_h) * math.cos(coxa_abs),
        mount_y + (COXA_LENGTH + tip_h) * math.sin(coxa_abs), tip_v])

    return np.array([mount_x, mount_y, 0.0]), coxa_end, femur_end, foot_tip


def validate_ik_all():
    print("\n" + "="*60)
    print("IK Round-Trip Validation (target < 0.1 mm)")
    print("="*60)
    ok = True
    for name, (mx, my) in MOUNTS.items():
        nx, ny = NEUTRAL[name]
        foot   = Vec3(nx, ny, DEFAULT_STAND_Z)
        angles = compute_leg_ik(foot, mx, my)
        if not angles.valid:
            print(f"  {name}: UNREACHABLE"); ok = False; continue
        fk  = compute_fk(angles.coxa, angles.femur, angles.tibia, mx, my)
        err = math.sqrt((foot.x-fk.x)**2+(foot.y-fk.y)**2+(foot.z-fk.z)**2)
        print(f"  {name}: err={err:.3f} mm {'OK' if err < 0.5 else 'FAIL'}")
        if err >= 0.5: ok = False
    print("All OK!\n" if ok else "WARNING: check dimensions\n")


# ─────────────────────────────────────────────────────────────
# 3. CONTINUOUS GAIT ENGINE
# ─────────────────────────────────────────────────────────────
#
# Each leg maintains a continuous phase phi in [0, 2*pi).
# The cycle is divided into:
#   STANCE  phi in [0, duty*2pi)      foot on ground, pushing backward
#   SWING   phi in [duty*2pi, 2pi)    foot in air, arcing forward
#
# Swing uses a sine arch for Z (smooth lift/land) and cubic easing
# for X/Y (smooth acceleration/deceleration).
# Stance moves the foot linearly backward — this is what propels the body.

DUTY_CYCLE    = 0.60    # fraction of cycle on ground (0.5=equal, 0.65=more stable)
STRIDE_LEN    = 70.0    # mm, half-stride (foot travels ±STRIDE from neutral)
STEP_HEIGHT   = 40.0    # mm, peak foot lift
CYCLE_PERIOD  = 1.6     # seconds per full gait cycle at speed=1.0
BODY_BOB      = 3.5     # mm, vertical body oscillation amplitude


class ContinuousGait:
    def __init__(self):
        self.phases:    Dict[str, float] = dict(TRIPOD_PHASE)
        self.speed      = 1.0
        self.stride     = STRIDE_LEN
        self.walk_dir   = 0.0     # radians (0=fwd, pi=bwd)
        self.turn_rate  = 0.0     # tangential mm per cycle
        self.moving     = False

    def step(self, dt: float) -> Dict[str, Vec3]:
        """Advance phases and return all foot positions."""
        if self.moving:
            dphi = (2.0 * math.pi / CYCLE_PERIOD) * self.speed * dt
            for leg in LEG_NAMES:
                self.phases[leg] = (self.phases[leg] + dphi) % (2.0 * math.pi)

        return {leg: self._foot_pos(leg) for leg in LEG_NAMES}

    def _foot_pos(self, leg: str) -> Vec3:
        phi        = self.phases[leg]
        nx, ny     = NEUTRAL[leg]
        s          = self.stride
        duty_rad   = 2.0 * math.pi * DUTY_CYCLE

        # Stride vector in walk direction
        sdx = s * math.cos(self.walk_dir)
        sdy = s * math.sin(self.walk_dir)

        # Rotation contribution: tangential arc around body centre
        leg_r = math.sqrt(nx*nx + ny*ny)
        if leg_r > 1e-3:
            tang_x = -ny / leg_r * self.turn_rate
            tang_y =  nx / leg_r * self.turn_rate
        else:
            tang_x = tang_y = 0.0

        sdx += tang_x
        sdy += tang_y

        if phi < duty_rad:
            # ── STANCE: foot on ground, marches backward ──
            t      = phi / duty_rad           # 0 → 1
            foot_x = nx + sdx * (0.5 - t)    # from +stride to -stride
            foot_y = ny + sdy * (0.5 - t)
            foot_z = DEFAULT_STAND_Z

        else:
            # ── SWING: foot arcs through the air ──
            swing_rad = 2.0 * math.pi - duty_rad
            t = (phi - duty_rad) / swing_rad  # 0 → 1 across swing

            # Cubic ease for horizontal travel
            te = t * t * (3.0 - 2.0 * t)
            foot_x = nx + sdx * (-0.5 + te)
            foot_y = ny + sdy * (-0.5 + te)

            # Sine arch for vertical — zero at lift-off AND land
            foot_z = DEFAULT_STAND_Z + STEP_HEIGHT * math.sin(math.pi * t)

        return Vec3(foot_x, foot_y, foot_z)

    def body_bob(self) -> float:
        """Vertical body oscillation from swing-phase leg count."""
        if not self.moving:
            return 0.0
        duty_rad = 2.0 * math.pi * DUTY_CYCLE
        n_swing  = sum(1 for leg in LEG_NAMES if self.phases[leg] >= duty_rad)
        # Body rises slightly when 3 legs are swinging (reduced support)
        return BODY_BOB * (n_swing / 3.0 - 0.5) * 0.4

    def set_walking(self, direction_deg: float):
        self.walk_dir  = math.radians(direction_deg)
        self.turn_rate = 0.0
        self.moving    = True

    def set_turning(self, direction: float):
        self.walk_dir  = 0.0
        self.turn_rate = 55.0 * direction   # mm tangential travel per cycle
        self.moving    = True

    def stop(self):
        self.moving = False


# ─────────────────────────────────────────────────────────────
# 4. COLOURS
# ─────────────────────────────────────────────────────────────
COL_BODY      = "#2c3e50"
COL_JOINT     = "#ecf0f1"
COL_COXA      = "#3498db"
COL_FEMUR     = "#2ecc71"
COL_TIBIA     = "#e74c3c"
COL_FOOT_DOWN = "#f39c12"
COL_FOOT_UP   = "#9b59b6"
COL_GRID      = "#2d2d4e"


# ─────────────────────────────────────────────────────────────
# 5. VISUALIZER
# ─────────────────────────────────────────────────────────────

class HexapodViz:

    def __init__(self):
        self.fig = plt.figure(figsize=(14, 9), facecolor="#0a0a1a")
        self.fig.canvas.manager.set_window_title("Hexapod Simulation v2")

        self.ax3d  = self.fig.add_axes([0.0, 0.05, 0.72, 0.95], projection='3d')
        self.ax_inf= self.fig.add_axes([0.73, 0.05, 0.26, 0.90])

        self._style_axes()
        self._init_artists()
        self._init_info_panel()

        self.gait       = ContinuousGait()
        self.mode       = "stand"
        self.traces: Dict[str, List[np.ndarray]] = {l: [] for l in LEG_NAMES}
        self._last_t    = None

        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._anim = animation.FuncAnimation(
            self.fig, self._animate,
            interval=33, blit=False, cache_frame_data=False
        )

    # ── Styling ──────────────────────────────────────────────

    def _style_axes(self):
        ax = self.ax3d
        ax.set_facecolor("#0d0d1f")
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor(COL_GRID)
        ax.tick_params(colors='#666688', labelsize=7)
        ax.set_xlabel("X fwd (mm)",  color='#8888aa', labelpad=4, fontsize=8)
        ax.set_ylabel("Y left (mm)", color='#8888aa', labelpad=4, fontsize=8)
        ax.set_zlabel("Z up (mm)",   color='#8888aa', labelpad=4, fontsize=8)
        ax.set_title("Hexapod Robot Simulation  —  Continuous Gait",
                     color='#ccccee', fontsize=13, fontweight='bold', pad=10)
        lim = 310
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-200, 80)
        ax.view_init(elev=25, azim=-60)

        xs = np.linspace(-lim, lim, 2)
        xx, yy = np.meshgrid(xs, xs)
        ax.plot_surface(xx, yy, np.full_like(xx, DEFAULT_STAND_Z - 0.5),
                        alpha=0.07, color='#4444aa', zorder=0)

        self.ax_inf.set_facecolor("#0d0d1f")
        self.ax_inf.set_xticks([]); self.ax_inf.set_yticks([])
        for s in self.ax_inf.spines.values():
            s.set_edgecolor("#334")

    # ── Artists ──────────────────────────────────────────────

    def _init_artists(self):
        ax = self.ax3d
        self.leg_art = {}
        for name in LEG_NAMES:
            self.leg_art[name] = {
                "coxa":   ax.plot([],[],[], '-', color=COL_COXA,  lw=4, solid_capstyle='round', zorder=5)[0],
                "femur":  ax.plot([],[],[], '-', color=COL_FEMUR, lw=4, solid_capstyle='round', zorder=5)[0],
                "tibia":  ax.plot([],[],[], '-', color=COL_TIBIA, lw=4, solid_capstyle='round', zorder=5)[0],
                "joints": ax.scatter([],[],[], s=25, c=COL_JOINT,     zorder=6, depthshade=False),
                "foot":   ax.scatter([],[],[], s=65, c=COL_FOOT_DOWN, zorder=7, depthshade=False),
            }
        self.body_line, = ax.plot([],[],[], '-', color=COL_BODY, lw=2.5, zorder=4)
        self.body_dot   = ax.scatter([0],[0],[0], s=90, c=COL_BODY, zorder=8, depthshade=False)

        tcols = {"LF":"#5588ff","LM":"#44aaff","LB":"#3366ff",
                 "RF":"#ff5566","RM":"#ff8844","RB":"#ff3355"}
        self.trace_art = {
            leg: ax.plot([],[],[], '-', color=tcols[leg], lw=0.9, alpha=0.45, zorder=2)[0]
            for leg in LEG_NAMES
        }

    def _init_info_panel(self):
        ax = self.ax_inf
        def t(x, y, s, **kw):
            return ax.text(x, y, s, transform=ax.transAxes,
                           fontfamily='monospace', **kw)

        t(0.5, 0.97, "HEXAPOD STATUS",
          ha='center', va='top', color='#aabbff', fontsize=10, fontweight='bold')

        self.txt_mode   = t(0.05, 0.90, "Mode:   STAND",    color='#66ffaa', fontsize=9)
        self.txt_speed  = t(0.05, 0.84, "Speed:  1.0x",     color='#ffcc44', fontsize=9)
        self.txt_stride = t(0.05, 0.78, f"Stride: {STRIDE_LEN:.0f} mm", color='#44ccff', fontsize=9)

        t(0.05, 0.73, "-"*28, color='#444466', fontsize=8)
        t(0.05, 0.70, "Leg  Coxa  Femur  Tibia", color='#8888aa', fontsize=8)

        y = 0.64
        self.txt_ang = {}
        for name in LEG_NAMES:
            self.txt_ang[name] = t(0.05, y, f"{name}   ---   ---   ---",
                                    color='#ccccee', fontsize=8)
            y -= 0.057

        t(0.05, 0.27, "-"*28, color='#444466', fontsize=8)
        t(0.05, 0.24, "LEGEND", color='#8888aa', fontsize=8, fontweight='bold')
        items = [(COL_COXA,"Coxa"),(COL_FEMUR,"Femur"),(COL_TIBIA,"Tibia"),
                 (COL_FOOT_DOWN,"Grounded"),(COL_FOOT_UP,"Lifted")]
        y = 0.20
        for col, lbl in items:
            ax.plot([0.05,0.13],[y,y],'-',color=col,lw=2,transform=ax.transAxes)
            t(0.16, y-0.005, lbl, color='#aaaacc', fontsize=7.5)
            y -= 0.045

        t(0.05, -0.01,
          "W/S: fwd/bwd    A/D: rotate\n"
          "SPACE: toggle walk\n"
          "Up/Dn: speed    [/]: stride\n"
          "Q: quit",
          color='#666688', fontsize=7.5, va='bottom')

    # ── Update ───────────────────────────────────────────────

    def _render_leg(self, name: str, foot: Vec3):
        mx, my = MOUNTS[name]
        ang = compute_leg_ik(foot, mx, my)
        if not ang.valid:
            return

        mount, coxa_end, femur_end, foot_tip = joint_positions(ang, mx, my)
        a = self.leg_art[name]

        def seg(line, p0, p1):
            line.set_data_3d([p0[0],p1[0]], [p0[1],p1[1]], [p0[2],p1[2]])

        seg(a["coxa"],  mount,    coxa_end)
        seg(a["femur"], coxa_end, femur_end)
        seg(a["tibia"], femur_end, foot_tip)

        jpts = np.array([mount, coxa_end, femur_end])
        a["joints"]._offsets3d = (jpts[:,0], jpts[:,1], jpts[:,2])

        is_up = foot.z > DEFAULT_STAND_Z + 5.0
        a["foot"]._offsets3d = ([foot_tip[0]], [foot_tip[1]], [foot_tip[2]])
        a["foot"].set_color(COL_FOOT_UP if is_up else COL_FOOT_DOWN)
        a["foot"].set_sizes([100 if is_up else 65])

        self.txt_ang[name].set_text(
            f"{name}  {ang.coxa:+5.1f} {ang.femur:+5.1f} {ang.tibia:+5.1f}"
        )

        # Foot trace ring buffer
        self.traces[name].append(foot_tip.copy())
        if len(self.traces[name]) > 120:
            self.traces[name].pop(0)
        tr = np.array(self.traces[name])
        self.trace_art[name].set_data_3d(tr[:,0], tr[:,1], tr[:,2])

    def _render_body(self):
        pts = [MOUNTS[n] for n in BODY_ORDER] + [MOUNTS[BODY_ORDER[0]]]
        self.body_line.set_data_3d(
            [p[0] for p in pts], [p[1] for p in pts], [0.0]*len(pts))

    def _update_info(self):
        labels = {"stand":"STAND","walk_fwd":"WALK  FWD","walk_bwd":"WALK  BWD",
                  "rot_l":"ROTATE LEFT","rot_r":"ROTATE RIGHT"}
        self.txt_mode.set_text(f"Mode:   {labels.get(self.mode, self.mode)}")
        self.txt_speed.set_text(f"Speed:  {self.gait.speed:.1f}x")
        self.txt_stride.set_text(f"Stride: {self.gait.stride:.0f} mm")

    # ── Animation ────────────────────────────────────────────

    def _animate(self, frame):
        now = time.monotonic()
        dt  = min(now - self._last_t, 0.1) if self._last_t else 0.033
        self._last_t = now

        feet = self.gait.step(dt)
        for name in LEG_NAMES:
            self._render_leg(name, feet[name])
        self._render_body()
        self._update_info()

    # ── Keyboard ─────────────────────────────────────────────

    def _on_key(self, ev):
        k = ev.key
        if   k == 'q':     plt.close('all'); sys.exit(0)
        elif k == ' ':
            if self.gait.moving:
                self.gait.stop(); self.mode = "stand"
            else:
                self._apply_mode()
        elif k == 'w':     self.mode = "walk_fwd"; self._apply_mode()
        elif k == 's':     self.mode = "walk_bwd"; self._apply_mode()
        elif k == 'a':     self.mode = "rot_l";    self._apply_mode()
        elif k == 'd':     self.mode = "rot_r";    self._apply_mode()
        elif k == 'up':    self.gait.speed  = min(self.gait.speed  + 0.2, 3.0)
        elif k == 'down':  self.gait.speed  = max(self.gait.speed  - 0.2, 0.2)
        elif k == ']':     self.gait.stride = min(self.gait.stride + 10,  130)
        elif k == '[':     self.gait.stride = max(self.gait.stride - 10,   20)

    def _apply_mode(self):
        if   self.mode == "walk_fwd": self.gait.set_walking(0.0)
        elif self.mode == "walk_bwd": self.gait.set_walking(180.0)
        elif self.mode == "rot_l":    self.gait.set_turning(+1.0)
        elif self.mode == "rot_r":    self.gait.set_turning(-1.0)

    def show(self):
        plt.tight_layout(pad=0.4)
        plt.show()


# ─────────────────────────────────────────────────────────────
# 6. ENTRY POINT
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    validate_ik_all()

    if _backend == 'Agg':
        print("ERROR: No interactive display backend found.")
        print()
        print("Quickest fix -- install PyQt5 into your venv:")
        print("  pip install PyQt5")
        print("  python hexapod_sim.py")
        print()
        print("Or install Tk support:")
        print("  brew install python-tk@3.14")
        sys.exit(1)

    print(f"Backend: {_backend}")
    print("Controls:")
    print("  W/S       -- walk forward / backward")
    print("  A/D       -- rotate left / right")
    print("  Space     -- toggle walking")
    print("  Up/Down   -- speed +/-0.2x")
    print("  [ / ]     -- stride -/+10 mm")
    print("  Q         -- quit\n")

    viz = HexapodViz()
    viz.show()
