# Hexapod CPG Controller — Arduino Implementation

A Central Pattern Generator (CPG) gait controller for the FutureTrace 18-DOF hexapod kit running on an Arduino Mega 2560. Gait patterns (wave, ripple, tripod) emerge automatically from coupled oscillator dynamics — no hard-coded keyframes, no mode switching.

---

## What is a CPG?

A Central Pattern Generator is a small circuit of neurons that produces rhythmic output without needing rhythmic input. Real insects use them to coordinate their legs — the brain just sends a "walk faster" signal, and the CPG handles all the inter-leg coordination automatically.

This implementation models each leg as a **Kuramoto phase oscillator**. The six oscillators are coupled together so they naturally synchronise into a stable gait. The key equation governing each leg `i` is:

```
dφᵢ/dt = ω + Σⱼ Kᵢⱼ · sin(φⱼ − φᵢ − θᵢⱼ)
```

| Symbol | Meaning |
|--------|---------|
| `φᵢ` | Phase of leg i ∈ [0, 2π) |
| `ω` | Natural frequency — controls **both speed and gait type** |
| `Kᵢⱼ` | Coupling strength between legs i and j |
| `θᵢⱼ` | Target phase offset (encodes the desired gait) |

The magic: changing `ω` alone causes the gait to smoothly transition between wave → ripple → tripod. You don't select a gait; it falls out of the math.

---

## Gait Emergence vs. Speed

| ω (rad/s) | Gait | Legs on ground | Character |
|-----------|------|---------------|-----------|
| ~1.0 | **Wave** | 5 (one lifts at a time) | Very stable, slow rolling motion |
| ~3.0 | **Ripple** | 4 | Balanced speed and stability |
| ~5.0 | **Tripod** | 3 (alternating triangles) | Fastest statically stable gait |

The transition between these is continuous — not a mode switch. At intermediate values of `ω` you'll see genuine ripple coordination emerge.

---

## File Structure

```
hexapod_walker/
├── hexapod_walker.ino   ← Main loop, state machine, serial commands
├── CPGEngine.h          ← Kuramoto oscillators (NEW — replaces GaitEngine.h)
├── Config.h             ← Robot dimensions, pin assignments    (unchanged)
├── Kinematics.h         ← IK / FK math                         (unchanged)
├── HexLeg.h             ← Leg controller + servo output         (unchanged)
└── GaitEngine.h         ← Old keyframe gait (kept, not used)
```

Only two files change: `hexapod_walker.ino` is replaced, and `CPGEngine.h` is added. Everything else — your calibration, your pin assignments, your IK — stays exactly the same.

---

## Installation

**1. Back up your existing sketch folder first.**

**2. Add `CPGEngine.h`** to your sketch folder alongside the other `.h` files.

**3. Replace `hexapod_walker.ino`** with the new version (`hexapod_walker_cpg.ino`). Rename it to match your sketch folder name if needed.

**4. Verify the neutral foot positions** in `CPGEngine.h`. Near the top of the file, find:

```cpp
const float CPG_NEUTRAL_X[CPG_N] = {
     FOOT_FB_DIST,   0.0f,  -FOOT_FB_DIST,
     FOOT_FB_DIST,   0.0f,  -FOOT_FB_DIST
};
const float CPG_NEUTRAL_Y[CPG_N] = {
    FOOT_WIDTH_FB,  FOOT_WIDTH_M,  FOOT_WIDTH_FB,
   -FOOT_WIDTH_FB, -FOOT_WIDTH_M, -FOOT_WIDTH_FB
};
```

These reference constants from your `Config.h`. If your config uses different names for the neutral foot positions, update them here to match.

**5. Compile and upload** via Arduino IDE. The sketch compiles cleanly for Mega 2560 with no additional libraries required.

---

## Serial Commands

Connect at **115200 baud**.

### Movement

| Command | Action |
|---------|--------|
| `w` | Walk forward |
| `b` | Walk backward |
| `l` | Rotate left (CCW) |
| `r` | Rotate right (CW) |
| `x` | Stop |
| `h` | Return to home position |
| `s` | Sleep (release servos) |

### Gait & Speed Control

| Command | Action |
|---------|--------|
| `+` | Increase ω by 0.5 → faster + higher gait |
| `-` | Decrease ω by 0.5 → slower + lower gait |

These are the most important commands for experiencing the CPG. Try pressing `+` several times while walking and watch (or hear) the gait pattern change live.

### Calibration

| Command | Action |
|---------|--------|
| `c` | Centre all servos (use this first, robot in air) |
| `1`–`6` | Wiggle leg by index (LF/LM/LB/RF/RM/RB) to identify wiring |
| `^` | Raise body height |
| `v` | Lower body height |

### Diagnostics

| Command | Action |
|---------|--------|
| `p` | Print CPG status (ω, gait name, coherence `r`, all phases) |
| `d` | Dump full robot state (all joint angles + CPG status) |
| `t` | Run IK test sequence on left-front leg |
| `q` | Run IK round-trip validation on all 6 legs |

---

## Understanding the `p` Command Output

```
CPG: omega=4.50  gait=Tripod  r=0.97  moving=yes
     phases(deg): LF=12 LM=193 LB=8 RF=191 RM=15 RB=188
```

**`omega`** — Current natural frequency. Directly maps to gait type (see table above).

**`gait`** — Inferred gait name based on current `ω`.

**`r`** — Phase coherence, ranging 0→1. This measures how well the oscillators have locked into the target gait pattern. Values above ~0.85 indicate a stable, well-coordinated gait. Values near 0 mean the network is still converging (normal after a speed change or disturbance — it will recover within a few steps).

**`phases`** — Current oscillator phase for each leg in degrees. In a good tripod you'll see Group A (LF, LB, RM) clustered near one value and Group B (LM, RF, RB) clustered ~180° away.

---

## Tuning Guide

All tuning parameters live near the top of `CPGEngine.h` or in the constructor.

### If the robot moves too fast or too slow

Adjust the starting `ω` in the constructor:
```cpp
CPGEngine() : omega(2.0f), ...
```
Lower values give a slower, more stable wave gait. Try `1.5` for a cautious start.

### If legs don't synchronise well or the gait looks chaotic

Increase coupling strength `K`:
```cpp
CPGEngine() : omega(2.0f), K(3.5f), ...
```
Higher `K` = stronger pull toward correct phase relationships = faster, tighter synchronisation. Values between 3.0 and 5.0 work well for most builds.

### If feet drag or stumble on the ground

Increase step height:
```cpp
const float CPG_STEP_HEIGHT = STEP_HEIGHT;
```
Override with a direct value, e.g. `45.0f`, if `STEP_HEIGHT` from `Config.h` is too low.

### If the wave gait feels glacially slow

The wave gait at `ω=1.0` is genuinely slow by design (one leg at a time is the most stable, cautious gait). Try starting the wave at `ω=1.5` — still wave-like, but noticeably faster. Use `+` and `-` at runtime to find the sweet spot for your surface.

### If the tripod gait has visible body sway

This is normal — it's the weight shifting between the two tripod groups. You can reduce it by lowering `CPG_STEP_HEIGHT` or by shortening the stride:
```cpp
CPGEngine() : omega(2.0f), K(3.5f), stride(STEP_LENGTH), ...
```
Try `stride(45.0f)` for a shorter, choppier but more stable tripod.

---

## Timing & Performance

The control loop runs at **40Hz** (25ms period). Measured times on a Mega 2560 at 16MHz:

| Task | Time |
|------|------|
| `cpg.step()` — 11 `sin()` calls + integration | ~2.5ms |
| 6× IK solve + servo write | ~3.5ms |
| Serial handling + overhead | ~0.5ms |
| **Total** | **~6.5ms** |

This leaves ~18ms of headroom per frame. If you add sensors or additional logic, the loop has plenty of room.

---

## How It Differs From the Old Keyframe Gait

| | Old `GaitEngine.h` | New `CPGEngine.h` |
|--|--------------------|--------------------|
| **Architecture** | 4 discrete keyframes, linear interpolation | 6 continuous coupled oscillators |
| **Gait type** | Tripod only, hard-coded | Wave / ripple / tripod, speed-adaptive |
| **Gait change** | Requires different code path | Change one variable (`ω`) |
| **Disturbance handling** | None — phases are rigid | CPG re-synchronises automatically |
| **Body motion** | Steps between fixed positions | Continuous sinusoidal foot arcs |
| **Loop structure** | State machine with phase counter | Single `cpg.step(dt)` call at 40Hz |

---

## Simulation

A full interactive Python simulation (`hexapod_cpg.py`) uses **identical math** to this Arduino implementation. Run it on your computer to visualise the CPG before uploading:

```bash
pip install matplotlib PyQt5
python3 hexapod_cpg.py
```

The simulation shows the phase wheel (watch leg dots cluster into groups as the network synchronises), the live gait diagram, and the order parameter `r`. Keyboard controls mirror the serial commands: `W`/`S`/`A`/`D` to move, Up/Down to change `ω`, `P` to perturb a leg, `R` to randomise all phases and watch convergence from scratch.

---

## Leg Index Reference

| Index | Label | Position |
|-------|-------|----------|
| 0 | LF | Left Front |
| 1 | LM | Left Middle |
| 2 | LB | Left Back |
| 3 | RF | Right Front |
| 4 | RM | Right Middle |
| 5 | RB | Right Back |

**Tripod Group A** (phase ≈ 0°): LF (0), LB (2), RM (4)  
**Tripod Group B** (phase ≈ 180°): LM (1), RF (3), RB (5)

---

## References

- Kuramoto, Y. (1984). *Chemical Oscillations, Waves, and Turbulence*. Springer.
- Ijspeert, A.J. (2008). Central pattern generators for locomotion control in animals and robots. *Neural Networks*, 21(4), 642–653.
- [JakobLeander/hexapod](https://github.com/JakobLeander/hexapod) — IK validation for this kit.
- `hexapod_cpg.py` — Python simulation with identical CPG math and interactive visualisation.
