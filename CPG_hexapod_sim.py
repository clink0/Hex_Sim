"""
hexapod_cpg.py — Hexapod CPG Simulation
=========================================
6 coupled Kuramoto phase oscillators drive hexapod locomotion.
Gait patterns (wave, ripple, tripod) emerge automatically from
coupling dynamics — no hard-coded phase offsets.

The CPG equation for each leg i:
    dphi_i/dt = omega + sum_j K_ij * sin(phi_j - phi_i - theta_ij) + noise

Where:
    phi_i   = phase of leg i  (0 to 2*pi)
    omega   = natural frequency (drive speed)
    K_ij    = coupling strength between legs i and j
    theta_ij = desired phase offset between legs i and j
    noise   = small perturbations (makes emergence visible)

Speed-adaptive gait emergence:
    Low omega  -> weak coupling dominates -> wave gait  (beta ~0.83)
    Mid omega  -> balanced               -> ripple gait (beta ~0.67)
    High omega -> system stabilises at   -> tripod gait (beta ~0.50)

Controls:
    W / S       -- walk forward / backward
    A / D       -- rotate left / right
    Space       -- stop / start
    Up / Down   -- speed (drives omega and gait transition)
    P           -- perturb one leg (watch re-synchronisation)
    R           -- reset oscillators to random phases
    Q           -- quit
"""

import numpy as np
import matplotlib
import matplotlib.gridspec as gridspec

def _pick_backend():
    candidates = ['macosx', 'Qt5Agg', 'Qt6Agg', 'GTK4Agg',
                  'GTK3Agg', 'wxAgg', 'TkAgg', 'Agg']
    for name in candidates:
        try:
            matplotlib.use(name)
            import matplotlib.pyplot as _p
            _p.figure(); _p.close('all')
            return name
        except Exception:
            continue
    return 'Agg'

_backend = _pick_backend()
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import sys, math, time
from dataclasses import dataclass
from typing import Dict, List, Tuple
from collections import deque

# ─────────────────────────────────────────────────────────────
# 1. ROBOT CONFIG
# ─────────────────────────────────────────────────────────────
COXA_LENGTH  = 28.0
FEMUR_LENGTH = 84.0
TIBIA_LENGTH = 127.0

MOUNTS = {
    "LF": ( 74.0,  39.0), "LM": (  0.0,  64.0), "LB": (-74.0,  39.0),
    "RF": ( 74.0, -39.0), "RM": (  0.0, -64.0), "RB": (-74.0, -39.0),
}
NEUTRAL = {
    "LF": ( 200.3,  105.5), "LM": (0.0,  206.7), "LB": (-200.3,  105.5),
    "RF": ( 200.3, -105.5), "RM": (0.0, -206.7), "RB": (-200.3, -105.5),
}

SERVO_MIN, SERVO_MAX = -90.0, 90.0
TIBIA_OFFSET = 14.0
DEFAULT_Z    = -123.2
STEP_HEIGHT  = 38.0
MAX_REACH    = FEMUR_LENGTH + TIBIA_LENGTH
MIN_REACH    = abs(FEMUR_LENGTH - TIBIA_LENGTH)

LEGS      = ["LF", "LM", "LB", "RF", "RM", "RB"]
BODY_POLY = ["LF", "RF", "RM", "RB", "LB", "LM"]
N = 6

# ─────────────────────────────────────────────────────────────
# 2. KINEMATICS
# ─────────────────────────────────────────────────────────────
@dataclass
class JointAngles:
    coxa: float = 0.
    femur: float = 0.
    tibia: float = 0.
    valid: bool = False

@dataclass
class Vec3:
    x: float = 0.
    y: float = 0.
    z: float = 0.

def ik(foot, mx, my):
    r = JointAngles()
    dx, dy, dz = foot.x - mx, foot.y - my, foot.z
    ma = math.atan2(my, mx)
    fa = math.atan2(dy, dx)
    c  = math.degrees(fa - ma)
    while c >  180.: c -= 360.
    while c < -180.: c += 360.
    r.coxa = c
    rr = math.sqrt(dx*dx + dy*dy)
    h = rr - COXA_LENGTH
    v = dz
    D = math.sqrt(h*h + v*v)
    if D > MAX_REACH or D < MIN_REACH or D < 1e-3:
        return r
    pa  = math.atan2(v, h)
    cpb = max(-1., min(1., (D*D + FEMUR_LENGTH**2 - TIBIA_LENGTH**2) / (2.*D*FEMUR_LENGTH)))
    r.femur = math.degrees(math.acos(cpb) + pa)
    cg = max(-1., min(1., (FEMUR_LENGTH**2 + TIBIA_LENGTH**2 - D*D) / (2.*FEMUR_LENGTH*TIBIA_LENGTH)))
    r.tibia = math.degrees(math.acos(cg)) - 90. - TIBIA_OFFSET
    if not all(SERVO_MIN <= a <= SERVO_MAX for a in [r.coxa, r.femur, r.tibia]):
        return r
    if any(math.isnan(a) for a in [r.coxa, r.femur, r.tibia]):
        return r
    r.valid = True
    return r

def joint_pts(ang, mx, my):
    ma  = math.atan2(my, mx)
    ca  = math.radians(ang.coxa) + ma
    fr  = math.radians(ang.femur)
    ti  = math.radians(ang.tibia + 90. + TIBIA_OFFSET)
    ce  = np.array([mx + COXA_LENGTH * math.cos(ca),
                    my + COXA_LENGTH * math.sin(ca), 0.])
    fh  = FEMUR_LENGTH * math.cos(fr)
    fv  = FEMUR_LENGTH * math.sin(fr)
    fe  = np.array([ce[0] + fh * math.cos(ca),
                    ce[1] + fh * math.sin(ca), fv])
    ta  = fr - (math.pi - ti)
    th  = fh + TIBIA_LENGTH * math.cos(ta)
    tv  = fv + TIBIA_LENGTH * math.sin(ta)
    ft  = np.array([mx + (COXA_LENGTH + th) * math.cos(ca),
                    my + (COXA_LENGTH + th) * math.sin(ca), tv])
    return np.array([mx, my, 0.]), ce, fe, ft

# ─────────────────────────────────────────────────────────────
# 3. CPG NETWORK
# ─────────────────────────────────────────────────────────────

class CPGNetwork:
    """
    6 Kuramoto coupled oscillators.

    Coupling topology:
        Ipsilateral  (same side, adjacent): LF-LM, LM-LB, RF-RM, RM-RB
        Contralateral (across body, same position): LF-RF, LM-RM, LB-RB
        Diagonal     (cross-body, offset): LF-RM, LB-RM, RF-LM, RB-LM

    Phase targets theta_ij are interpolated between:
        Wave gait   (omega low):  60 deg spacing front-to-back
        Tripod gait (omega high): 0 or 180 deg (two groups)

    This makes gait type a continuous function of omega.
    """

    EDGES = [
        (0,1),(1,2),          # ipsilateral left
        (3,4),(4,5),          # ipsilateral right
        (0,3),(1,4),(2,5),    # contralateral
        (0,4),(2,4),(3,1),(5,1),  # diagonal
    ]

    def __init__(self):
        self.phi    = np.random.uniform(0, 2*math.pi, N)
        self.omega  = 2.0
        self.K_base = 3.5
        self.noise  = 0.04
        self.moving = False
        self.walk_dir  = 0.0
        self.turn_rate = 0.0
        self.stride    = 65.0
        self.history   = deque(maxlen=600)
        self.t = 0.0
        self._build_coupling()

    def _build_coupling(self):
        # Tripod targets: Group A={LF=0,RM=4,LB=2}=0 deg, Group B={RF=3,LM=1,RB=5}=180 deg
        group = np.array([0, 1, 0, 1, 0, 1])
        self._theta_tripod = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                if group[i] != group[j]:
                    self._theta_tripod[i, j] = math.pi

        # Wave targets: evenly spaced 60 deg offsets (front-to-back propagation)
        # LF=0, LM=60, LB=120, RF=180, RM=240, RB=300
        wave_phase = np.array([0, 1, 2, 3, 4, 5]) * (2*math.pi / 6)
        self._theta_wave = np.zeros((N, N))
        for i in range(N):
            for j in range(N):
                d = wave_phase[j] - wave_phase[i]
                d = (d + math.pi) % (2*math.pi) - math.pi  # wrap to (-pi, pi]
                self._theta_wave[i, j] = d

        # Coupling strength matrix
        self.K = np.zeros((N, N))
        for (i, j) in self.EDGES:
            self.K[i, j] = self.K_base
            self.K[j, i] = self.K_base

        self._update_theta()

    def _update_theta(self):
        """Interpolate coupling targets between wave and tripod based on omega."""
        alpha = np.clip((self.omega - 1.0) / 4.0, 0., 1.)
        alpha = alpha * alpha * (3. - 2. * alpha)   # smoothstep
        self.theta = (1. - alpha) * self._theta_wave + alpha * self._theta_tripod

    def step(self, dt):
        if not self.moving:
            return self._feet_from_phases()

        self._update_theta()

        # RK2 (midpoint) integration for stability
        def dphi(ph):
            d = np.zeros(N)
            for i in range(N):
                coupling = 0.
                for j in range(N):
                    if self.K[i, j] > 0:
                        coupling += self.K[i, j] * math.sin(ph[j] - ph[i] - self.theta[i, j])
                d[i] = self.omega + coupling
            return d

        k1 = dphi(self.phi)
        k2 = dphi(self.phi + 0.5 * dt * k1)
        self.phi += dt * k2
        self.phi += np.random.randn(N) * self.noise * dt
        self.phi %= 2 * math.pi

        self.t += dt
        self.history.append((self.t, self.phi.copy()))

        return self._feet_from_phases()

    def _feet_from_phases(self):
        feet = {}
        # Duty factor decreases as omega increases (more swing time at speed)
        duty = np.clip(0.70 - 0.08 * (self.omega - 1.0) / 4.0, 0.50, 0.72)
        duty_rad = 2 * math.pi * duty

        for idx, leg in enumerate(LEGS):
            phi    = self.phi[idx]
            nx, ny = NEUTRAL[leg]
            s      = self.stride

            sdx = s * math.cos(self.walk_dir)
            sdy = s * math.sin(self.walk_dir)

            leg_r = math.sqrt(nx*nx + ny*ny)
            if leg_r > 1e-3:
                tx = -ny / leg_r * self.turn_rate
                ty =  nx / leg_r * self.turn_rate
            else:
                tx = ty = 0.
            sdx += tx
            sdy += ty

            if phi < duty_rad:
                t      = phi / duty_rad
                foot_x = nx + sdx * (0.5 - t)
                foot_y = ny + sdy * (0.5 - t)
                foot_z = DEFAULT_Z
            else:
                swing = 2 * math.pi - duty_rad
                t      = (phi - duty_rad) / swing
                te     = t * t * (3. - 2. * t)
                foot_x = nx + sdx * (-0.5 + te)
                foot_y = ny + sdy * (-0.5 + te)
                foot_z = DEFAULT_Z + STEP_HEIGHT * math.sin(math.pi * t)

            feet[leg] = Vec3(foot_x, foot_y, foot_z)

        return feet

    def gait_name(self):
        alpha = np.clip((self.omega - 1.0) / 4.0, 0., 1.)
        alpha = alpha * alpha * (3. - 2. * alpha)
        if alpha < 0.25:   return "Wave"
        elif alpha < 0.65: return "Ripple"
        else:              return "Tripod"

    def phase_coherence(self):
        """
        Pattern coherence in [0,1].
        Measures how closely actual phase differences match target offsets
        across all coupled pairs. 1.0 = perfect gait lock.
        (Global Kuramoto r is misleading for antiphase gaits like tripod.)
        """
        if not self.moving:
            return 0.0
        errors = []
        for i in range(N):
            for j in range(N):
                if self.K[i, j] > 0:
                    actual = (self.phi[j] - self.phi[i] + math.pi) % (2*math.pi) - math.pi
                    target = self.theta[i, j]
                    err = abs(actual - target)
                    if err > math.pi:
                        err = 2*math.pi - err
                    errors.append(err)
        if not errors:
            return 0.0
        return float(1.0 - np.mean(errors) / math.pi)

    def perturb(self, leg_idx=0, amount=math.pi):
        self.phi[leg_idx] = (self.phi[leg_idx] + amount) % (2 * math.pi)

    def set_speed(self, omega):
        self.omega = np.clip(omega, 0.5, 6.0)

    def set_walking(self, direction_deg):
        self.walk_dir  = math.radians(direction_deg)
        self.turn_rate = 0.
        self.moving    = True

    def set_turning(self, direction):
        self.walk_dir  = 0.
        self.turn_rate = 55. * direction
        self.moving    = True

    def stop(self):
        self.moving = False


# ─────────────────────────────────────────────────────────────
# 4. COLOURS
# ─────────────────────────────────────────────────────────────
BG       = "#0a0a1a"
PANEL_BG = "#0d0d1f"
GRID_COL = "#1e1e3a"

LEG_COLS = {
    "LF": "#4fc3f7", "LM": "#29b6f6", "LB": "#0288d1",
    "RF": "#ef9a9a", "RM": "#e57373", "RB": "#c62828",
}
COL_COXA      = "#3498db"
COL_FEMUR     = "#2ecc71"
COL_TIBIA     = "#e74c3c"
COL_JOINT     = "#ecf0f1"
COL_FOOT_DOWN = "#f39c12"
COL_FOOT_UP   = "#9b59b6"
COL_BODY      = "#2c3e50"

def gait_colour(omega):
    alpha = np.clip((omega - 1.0) / 4.0, 0., 1.)
    alpha = float(alpha * alpha * (3. - 2. * alpha))
    # blue -> green -> orange
    if alpha < 0.5:
        t = alpha * 2.
        r = int(0   + t * 50);  g = int(100 + t * 156); b = int(220 - t * 20)
    else:
        t = (alpha - 0.5) * 2.
        r = int(50  + t * 205); g = int(256 - t * 106); b = int(200 - t * 200)
    return "#{:02x}{:02x}{:02x}".format(min(r,255), min(g,255), max(b,0))


# ─────────────────────────────────────────────────────────────
# 5. VISUALIZER
# ─────────────────────────────────────────────────────────────

class CPGViz:
    HISTORY_SECS = 4.0

    def __init__(self):
        self.fig = plt.figure(figsize=(16, 9), facecolor=BG)
        self.fig.canvas.manager.set_window_title("Hexapod CPG Simulation")

        gs = gridspec.GridSpec(
            3, 2, figure=self.fig,
            width_ratios=[1.55, 1.0],
            height_ratios=[1.1, 1.0, 0.85],
            hspace=0.40, wspace=0.07,
            left=0.01, right=0.97, top=0.94, bottom=0.05
        )

        self.ax3d  = self.fig.add_subplot(gs[:, 0], projection='3d')
        self.ax_whl = self.fig.add_subplot(gs[0, 1])
        self.ax_gdt = self.fig.add_subplot(gs[1, 1])
        self.ax_inf = self.fig.add_subplot(gs[2, 1])

        self._style_all()
        self._init_3d()
        self._init_wheel()
        self._init_gait_diagram()
        self._init_info()

        self.cpg    = CPGNetwork()
        self.mode   = "stand"
        self.traces = {l: [] for l in LEGS}
        self._last_t = None

        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self._anim = animation.FuncAnimation(
            self.fig, self._animate,
            interval=33, blit=False, cache_frame_data=False
        )

    # ── Style ─────────────────────────────────────────────────

    def _style_all(self):
        ax = self.ax3d
        ax.set_facecolor(PANEL_BG)
        for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor(GRID_COL)
        ax.tick_params(colors='#556', labelsize=6)
        ax.set_xlabel("X fwd",  color='#556', labelpad=2, fontsize=7)
        ax.set_ylabel("Y left", color='#556', labelpad=2, fontsize=7)
        ax.set_zlabel("Z up",   color='#556', labelpad=2, fontsize=7)
        ax.set_xlim(-310, 310); ax.set_ylim(-310, 310); ax.set_zlim(-200, 90)
        ax.view_init(elev=24, azim=-55)
        lim = 310
        xs = np.linspace(-lim, lim, 2)
        xx, yy = np.meshgrid(xs, xs)
        ax.plot_surface(xx, yy, np.full_like(xx, DEFAULT_Z - 0.5),
                        alpha=0.06, color='#4444aa', zorder=0)
        for a in [self.ax_whl, self.ax_gdt, self.ax_inf]:
            a.set_facecolor(PANEL_BG)
            for s in a.spines.values():
                s.set_edgecolor(GRID_COL)

    # ── 3D artists ────────────────────────────────────────────

    def _init_3d(self):
        ax = self.ax3d
        self.leg_art = {}
        for leg in LEGS:
            self.leg_art[leg] = {
                "coxa":  ax.plot([], [], [], '-', color=COL_COXA,  lw=4, solid_capstyle='round', zorder=5)[0],
                "femur": ax.plot([], [], [], '-', color=COL_FEMUR, lw=4, solid_capstyle='round', zorder=5)[0],
                "tibia": ax.plot([], [], [], '-', color=COL_TIBIA, lw=4, solid_capstyle='round', zorder=5)[0],
                "joints": ax.scatter([], [], [], s=22, c=COL_JOINT,     zorder=6, depthshade=False),
                "foot":   ax.scatter([], [], [], s=60, c=COL_FOOT_DOWN, zorder=7, depthshade=False),
            }
        self.body_line, = ax.plot([], [], [], '-', color=COL_BODY, lw=2.5, zorder=4)
        self.title_3d   = ax.set_title("Hexapod CPG",
                                        color='#ccccee', fontsize=11, fontweight='bold', pad=6)
        self.trace_art  = {
            leg: ax.plot([], [], [], '-', color=LEG_COLS[leg],
                         lw=0.8, alpha=0.5, zorder=2)[0]
            for leg in LEGS
        }

    # ── Phase wheel ────────────────────────────────────────────

    def _init_wheel(self):
        ax = self.ax_whl
        ax.set_aspect('equal')
        ax.set_xlim(-1.65, 1.65)
        ax.set_ylim(-1.65, 1.65)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title("Phase Wheel", color='#aabbff', fontsize=9, fontweight='bold', pad=4)

        # Outer circle guide
        ta = np.linspace(0, 2*math.pi, 300)
        ax.plot(np.cos(ta), np.sin(ta), '-', color='#333355', lw=1.5)

        # Sector fills (redrawn each frame)
        self._stance_patch = ax.fill([], [], color='#0d2a0d', alpha=0.6, zorder=1)[0]
        self._swing_patch  = ax.fill([], [], color='#1a0a2a', alpha=0.5, zorder=1)[0]

        # Leg dots + labels on the wheel
        self._whl_dots = {}
        self._whl_lbls = {}
        for leg in LEGS:
            dot, = ax.plot([], [], 'o', color=LEG_COLS[leg], markersize=11, zorder=5)
            lbl  = ax.text(0, 0, leg, color=LEG_COLS[leg],
                           fontsize=6.5, fontweight='bold', ha='center', va='center', zorder=6)
            self._whl_dots[leg] = dot
            self._whl_lbls[leg] = lbl

        # Order parameter ring (radius = coherence)
        self._coh_ring = Circle((0, 0), 0., fill=False, lw=3, zorder=3,
                                 edgecolor='#44ff88')
        ax.add_patch(self._coh_ring)

        # Text annotations
        self._coh_txt   = ax.text(0, -1.50, 'r = 0.00',
                                   color='#aaffaa', fontsize=8, ha='center')
        self._gait_txt  = ax.text(0,  1.48, 'STAND',
                                   color='#ffcc44', fontsize=9, ha='center', fontweight='bold')

        # Add compass labels
        for angle, label in [(0,'0'), (math.pi/2,'pi/2'), (math.pi,'pi'), (3*math.pi/2,'3pi/2')]:
            ax.text(1.58*math.cos(angle), 1.58*math.sin(angle), label,
                    color='#444466', fontsize=6, ha='center', va='center')

    def _update_wheel(self):
        cpg = self.cpg
        duty = np.clip(0.70 - 0.08*(cpg.omega - 1.0)/4.0, 0.50, 0.72)
        duty_rad = 2*math.pi * duty

        # Stance sector
        ta = np.linspace(0, duty_rad, 120)
        sx = np.concatenate([[0], np.cos(ta), [0]])
        sy = np.concatenate([[0], np.sin(ta), [0]])
        self._stance_patch.set_xy(np.column_stack([sx, sy]))

        # Swing sector
        tb = np.linspace(duty_rad, 2*math.pi, 120)
        swx = np.concatenate([[0], np.cos(tb), [0]])
        swy = np.concatenate([[0], np.sin(tb), [0]])
        self._swing_patch.set_xy(np.column_stack([swx, swy]))

        # Leg positions on wheel
        r_dot = 1.18
        for idx, leg in enumerate(LEGS):
            phi = cpg.phi[idx]
            x = r_dot * math.cos(phi)
            y = r_dot * math.sin(phi)
            self._whl_dots[leg].set_data([x], [y])
            lx = 1.40 * math.cos(phi)
            ly = 1.40 * math.sin(phi)
            self._whl_lbls[leg].set_position((lx, ly))

        # Coherence ring
        r = cpg.phase_coherence()
        self._coh_ring.set_radius(r * 0.88)
        self._coh_ring.set_edgecolor(gait_colour(cpg.omega))
        self._coh_txt.set_text(f'r = {r:.2f}')

        gname = cpg.gait_name() if cpg.moving else 'STAND'
        self._gait_txt.set_text(gname)
        self._gait_txt.set_color(gait_colour(cpg.omega) if cpg.moving else '#888888')

    # ── Gait diagram ──────────────────────────────────────────

    def _init_gait_diagram(self):
        ax = self.ax_gdt
        ax.set_facecolor('#080812')
        ax.set_xlim(0, self.HISTORY_SECS)
        ax.set_ylim(-0.5, N - 0.5)
        ax.set_yticks(range(N))
        ax.set_yticklabels(LEGS, color='#aabbff', fontsize=8)
        ax.set_xlabel("Time (s)", color='#8888aa', fontsize=7, labelpad=2)
        ax.set_title("Gait Diagram  (bright=swing, dark=stance)",
                     color='#aabbff', fontsize=8, fontweight='bold', pad=3)
        ax.tick_params(axis='x', colors='#666688', labelsize=7)
        ax.tick_params(axis='y', length=0)

        for idx in range(N):
            ax.axhline(idx, color='#12122a', lw=10, zorder=1)

        self._gdt_scats = {}
        for idx, leg in enumerate(LEGS):
            scat = ax.scatter([], [], s=14, marker='|',
                              linewidths=2.5, zorder=3,
                              c=LEG_COLS[leg])
            self._gdt_scats[leg] = scat

    def _update_gait_diagram(self):
        cpg  = self.cpg
        hist = list(cpg.history)
        if len(hist) < 2:
            return
        t_now = hist[-1][0]
        t_min = t_now - self.HISTORY_SECS
        duty  = np.clip(0.70 - 0.08*(cpg.omega - 1.0)/4.0, 0.50, 0.72)
        duty_rad = 2*math.pi * duty

        for idx, leg in enumerate(LEGS):
            xs, ys, cs = [], [], []
            for (t, phases) in hist:
                if t < t_min:
                    continue
                tx  = t - t_min
                phi = phases[idx]
                xs.append(tx)
                ys.append(idx)
                cs.append(LEG_COLS[leg] if phi >= duty_rad else '#1a1a3a')

            if xs:
                self._gdt_scats[leg].set_offsets(np.column_stack([xs, ys]))
                self._gdt_scats[leg].set_color(cs)
            else:
                self._gdt_scats[leg].set_offsets(np.zeros((0, 2)))

    # ── Info panel ────────────────────────────────────────────

    def _init_info(self):
        ax = self.ax_inf
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.set_xticks([]); ax.set_yticks([])

        def t(x, y, s, **kw):
            return ax.text(x, y, s, transform=ax.transAxes,
                           fontfamily='monospace', **kw)

        t(0.5, 0.97, "CPG STATUS", ha='center', va='top',
          color='#aabbff', fontsize=9, fontweight='bold')

        self.txt_mode  = t(0.04, 0.86, "Mode:  STAND",     color='#66ffaa', fontsize=8)
        self.txt_omega = t(0.04, 0.76, "omega: 2.00 rad/s", color='#ffcc44', fontsize=8)
        self.txt_gait  = t(0.04, 0.66, "Gait:  ---",        color='#44ccff', fontsize=8)
        self.txt_coh   = t(0.04, 0.56, "r:     0.00",       color='#ff9944', fontsize=8)

        t(0.04, 0.46, "-"*32, color='#222244', fontsize=7)
        t(0.04, 0.40, "Leg  Coxa  Femur  Tibia", color='#555577', fontsize=7)

        y = 0.33
        self.txt_ang = {}
        for leg in LEGS:
            self.txt_ang[leg] = t(0.04, y, f"{leg}   ---   ---   ---",
                                   color='#888899', fontsize=7)
            y -= 0.054

        t(0.04, -0.03,
          "W/S: fwd/bwd   A/D: rotate\n"
          "Up/Dn: omega -> gait changes\n"
          "P: perturb leg  R: randomise\n"
          "SPACE: stop/go   Q: quit",
          color='#333355', fontsize=7, va='bottom')

    def _update_info(self):
        cpg = self.cpg
        labels = {
            "stand": "STAND", "walk_fwd": "WALK FWD",
            "walk_bwd": "WALK BWD", "rot_l": "ROTATE LEFT", "rot_r": "ROTATE RIGHT"
        }
        self.txt_mode.set_text(f"Mode:  {labels.get(self.mode, '---')}")
        self.txt_omega.set_text(f"omega: {cpg.omega:.2f} rad/s")
        self.txt_gait.set_text( f"Gait:  {cpg.gait_name()}")
        self.txt_coh.set_text(  f"r:     {cpg.phase_coherence():.2f}")
        self.txt_gait.set_color(gait_colour(cpg.omega))
        self.title_3d.set_text(
            f"Hexapod CPG  |  {cpg.gait_name()}  |  "
            f"omega={cpg.omega:.1f}  |  r={cpg.phase_coherence():.2f}"
        )
        self.title_3d.set_color(gait_colour(cpg.omega))

    # ── 3D render ─────────────────────────────────────────────

    def _render_leg(self, leg, foot):
        mx, my = MOUNTS[leg]
        ang    = ik(foot, mx, my)
        if not ang.valid:
            return
        mount, ce, fe, ft = joint_pts(ang, mx, my)
        a = self.leg_art[leg]

        a["coxa"].set_data_3d([mount[0],ce[0]], [mount[1],ce[1]], [mount[2],ce[2]])
        a["femur"].set_data_3d([ce[0],fe[0]],   [ce[1],fe[1]],   [ce[2],fe[2]])
        a["tibia"].set_data_3d([fe[0],ft[0]],   [fe[1],ft[1]],   [fe[2],ft[2]])

        jp = np.array([mount, ce, fe])
        a["joints"]._offsets3d = (jp[:,0], jp[:,1], jp[:,2])

        is_up = foot.z > DEFAULT_Z + 5.
        a["foot"]._offsets3d = ([ft[0]], [ft[1]], [ft[2]])
        a["foot"].set_color(COL_FOOT_UP if is_up else COL_FOOT_DOWN)
        a["foot"].set_sizes([95 if is_up else 60])

        self.txt_ang[leg].set_text(
            f"{leg}  {ang.coxa:+5.1f} {ang.femur:+5.1f} {ang.tibia:+5.1f}"
        )

        self.traces[leg].append(ft.copy())
        if len(self.traces[leg]) > 130:
            self.traces[leg].pop(0)
        tr = np.array(self.traces[leg])
        self.trace_art[leg].set_data_3d(tr[:,0], tr[:,1], tr[:,2])

    def _render_body(self):
        pts = [MOUNTS[n] for n in BODY_POLY] + [MOUNTS[BODY_POLY[0]]]
        self.body_line.set_data_3d(
            [p[0] for p in pts], [p[1] for p in pts], [0.]*len(pts))

    # ── Animation loop ────────────────────────────────────────

    def _animate(self, frame):
        now = time.monotonic()
        dt  = min(now - self._last_t, 0.08) if self._last_t else 0.033
        self._last_t = now

        feet = self.cpg.step(dt)
        for leg in LEGS:
            self._render_leg(leg, feet[leg])
        self._render_body()
        self._update_wheel()
        self._update_gait_diagram()
        self._update_info()

    # ── Keyboard ──────────────────────────────────────────────

    def _on_key(self, ev):
        k = ev.key
        if k == 'q':
            plt.close('all'); sys.exit(0)
        elif k == ' ':
            if self.cpg.moving:
                self.cpg.stop(); self.mode = "stand"
            else:
                self._apply_mode()
        elif k == 'w':  self.mode = "walk_fwd"; self._apply_mode()
        elif k == 's':  self.mode = "walk_bwd"; self._apply_mode()
        elif k == 'a':  self.mode = "rot_l";    self._apply_mode()
        elif k == 'd':  self.mode = "rot_r";    self._apply_mode()
        elif k == 'up':
            self.cpg.set_speed(self.cpg.omega + 0.3)
            if not self.cpg.moving and self.mode != "stand":
                self._apply_mode()
        elif k == 'down':
            self.cpg.set_speed(self.cpg.omega - 0.3)
        elif k == 'p':
            idx = np.random.randint(0, N)
            self.cpg.perturb(idx, math.pi * np.random.uniform(0.7, 1.3))
            print(f"  >> Perturbed {LEGS[idx]} — watch re-synchronisation")
        elif k == 'r':
            self.cpg.phi = np.random.uniform(0, 2*math.pi, N)
            print("  >> Phases randomised — watch CPG converge to gait")

    def _apply_mode(self):
        if   self.mode == "walk_fwd": self.cpg.set_walking(0.)
        elif self.mode == "walk_bwd": self.cpg.set_walking(180.)
        elif self.mode == "rot_l":    self.cpg.set_turning(+1.)
        elif self.mode == "rot_r":    self.cpg.set_turning(-1.)

    def show(self):
        plt.show()


# ─────────────────────────────────────────────────────────────
# 6. ENTRY POINT
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n" + "="*62)
    print("  Hexapod CPG Simulation  --  Kuramoto Coupled Oscillators")
    print("="*62)
    print("""
  Each leg = one phase oscillator  phi_i in [0, 2*pi)

  Coupling equation:
    dphi_i/dt = omega + sum_j K_ij * sin(phi_j - phi_i - theta_ij)

  Gait emerges automatically from omega (the drive frequency):
    omega ~ 1.0  -->  Wave gait   (5 legs down, one lifts at a time)
    omega ~ 3.0  -->  Ripple gait (4 legs down)
    omega ~ 5.0  -->  Tripod gait (3 legs down, classic fast gait)

  The PHASE WHEEL shows each leg as a dot on a circle.
  Watch them cluster into groups as the CPG synchronises.

  The GAIT DIAGRAM shows stance/swing history over time --
  you can read the coordination pattern directly off it.

  The ORDER PARAMETER r = |mean(exp(i*phi))| measures sync:
    r -> 1  means the network has locked into a stable gait
    r -> 0  means phases are scattered (transitioning)

  Controls:
    W/S         walk forward / backward
    A/D         rotate left / right
    Up/Down     change omega  (watch gait transition on diagram)
    P           kick one oscillator (watch re-synchronisation)
    R           randomise all phases (watch CPG converge from scratch)
    Space       stop / start
    Q           quit
""")

    if _backend == 'Agg':
        print("ERROR: No interactive display backend found.")
        print("  pip install PyQt5  then re-run")
        sys.exit(1)

    print(f"  Backend: {_backend}\n")
    viz = CPGViz()
    viz.show()
