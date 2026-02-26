# Hexapod Robot Simulation

A 3D Python simulation of a 6-legged robot, faithfully porting the IK math and gait logic from the Arduino firmware.

## What It Simulates

- **Inverse kinematics** — geometric / law-of-cosines IK identical to `Kinematics.h`
- **Forward kinematics** — used to validate IK round-trip accuracy (all 6 legs at < 0.001mm error)
- **Tripod gait** — alternating Group A (LF, RM, LB) and Group B (RF, LM, RB) with cubic smoothstep easing
- **Rotation in place** — foot arc rotation matching `GaitEngine.h`
- **Live joint readout** — coxa/femur/tibia angles displayed in real time

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install matplotlib numpy

# matplotlib needs an interactive backend — one of these will work:
pip install PyQt5        # recommended (no system deps)
# OR: brew install python-tk@3.14
```

## Running

```bash
python hexapod_sim.py
```

IK validation runs first in the terminal, then the 3D window opens.

## Controls

| Key | Action |
|-----|--------|
| `W` / `S` | Walk forward / backward |
| `A` / `D` | Rotate left / right |
| `Space` | Toggle auto-walk |
| `←` / `→` | Step through gait phases manually |
| `Q` | Quit |

## File Structure

```
hexapod_sim.py   — full simulation (config, kinematics, gait, visualizer)
README.md        — this file
```

## Segment Colours

| Colour | Segment |
|--------|---------|
| Blue | Coxa (hip → knee) |
| Green | Femur (knee → ankle) |
| Red | Tibia (ankle → foot tip) |
| Orange dot | Foot on ground |
| Purple dot | Foot lifted (swing phase) |

## Robot Dimensions

Validated against the JakobLeander/hexapod reference implementation for the FutureTrace kit.

| Segment | Length |
|---------|--------|
| Coxa | 28 mm |
| Femur | 84 mm |
| Tibia | 127 mm |
| Max reach | 239 mm |
