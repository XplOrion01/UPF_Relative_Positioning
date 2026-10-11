# Single-Range Relative Pose Estimation with UWB + Odometry (2D, Two Robots)

Adapting the **Unscented Particle Filter (UPF)** from the paper *"Where are You? Unscented Particle Filter for Single Range Relative Pose Estimation in Unobservable Motion Using UWB and VIO"* to a simplified **2D, two-robot** setup, with the goal of running it on real hardware.

> **Status: 🧪 testing / modifying / iterating** to minimise estimation error. Everything below is validated in **simulation only**. Hardware results will be added here when available.

---

## 1. The problem

Two robots (A and B) have to find each other's **relative pose** (x, y, yaw) using only:

- **Odometry increments** shared between the robots (frame-to-frame motion in each robot's own body frame), and
- **One UWB range** (distance) between them.

**No positions or poses are ever shared**, and there is no GPS, no common reference frame and no external anchor. Ground truth is used only to score the error.

---

## 2. References

| | Link |
|---|---|
| Paper | Y. Durodié, B. Convens, G. Liu, T. Decoster, A. Munteanu, B. Vanderborght, *IEEE Robotics and Automation Letters*, vol. 9, no. 12, pp. 11754–11761, Dec. 2024. DOI: [10.1109/LRA.2024.3495592](https://doi.org/10.1109/LRA.2024.3495592) |
| Original code | <https://github.com/y2d2/UPF_RPE/tree/v1.0_dev> |

Please cite the paper if you use ideas from this work:

```bibtex
@article{durodie2024where,
  author  = {Durodi{\'e}, Yuri and Convens, Bryan and Liu, Gaoyuan and Decoster, Thomas and Munteanu, Adrian and Vanderborght, Bram},
  title   = {Where are You? Unscented Particle Filter for Single Range Relative Pose Estimation in Unobservable Motion Using {UWB} and {VIO}},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {9},
  number  = {12},
  pages   = {11754--11761},
  year    = {2024},
  doi     = {10.1109/LRA.2024.3495592}
}
```

> This repo is an independent adaptation. Check the license of the original repository before copying any of its code, and add your own `LICENSE` file here.

---

## 3. How this differs from the paper / original repo

| Aspect | Paper / original repo | This work (version 1) |
|---|---|---|
| Dimensions | 3D, 4 unknowns (x, y, z, yaw) | **2D**, 3 states (x, y, yaw) |
| Robots | UAVs in the paper; both robots moving in the repo's tests | **Static A + moving B**, and **both moving** |
| State | Trajectory of B + initial relative pose (separate states) + pseudo-state | **One 3-state pose** (B in A's frame) |
| Rotational drift | Pseudo-state in each particle | Modelled as **extra process noise** (both-moving codes); not needed for static A |
| Particle start | Grid over azimuth, elevation and heading | Static A: **heading only** (azimuth pinned). Both moving: grid over **bearing × heading** |
| Resampling | Branch-kill | **None**: all particles kept with cumulative likelihood weights |
| Update rate / UWB noise | 10 Hz; 0.1–1 m in simulation | 50 Hz; **0.01 m** or **0.1–1 m** (set in the code) |

### Key idea for the static-A case

If A never moves, rotating B's *whole* trajectory about A leaves every UWB range unchanged. So **B's start azimuth around A cannot be observed**. Instead of wasting particles on it, the static-A codes fix that azimuth by convention and spend all particles on the one observable unknown, B's initial heading. Consequence: positions are in a "gauge frame", and to compare with ground truth you must rotate by **one** angle (known in simulation, fitted once against e.g. Vicon on hardware). The fair accuracy measure for static A is the **A-in-B error** (where A appears from B's body frame) and the distance error, which do not depend on that rotation.

When both robots move, A tracks its own heading, the relative pose becomes observable, and no pinning is needed.

---

## 4. Repository layout

```
.
├── README.md
└── v1_simplified_2d_upf/
    ├── code1_static_A_lowNoise.py        # static A, B moves, UWB noise 0.01 m
    ├── code2_static_A_noisyUWB.py        # static A, B moves, UWB noise 0.1 - 1 m
    ├── code3_both_moving_lowNoise.py     # both move, A estimates B, UWB noise 0.01 m
    ├── code4_both_moving_noisyUWB.py     # both move, A estimates B, UWB noise 0.1 - 1 m
    ├── analyze_results.py                # universal analysis / plotting script
    ├── results/                          # one csv per run (see naming below)
    └── analysis_output/                  # plots + summary.csv written by analyze_results.py
```

---

## 5. The four codes

| # | File | Motion | UWB noise | Particles |
|---|---|---|---|---|
| 1 | `code1_static_A_lowNoise.py` | A static, B moves | 0.01 m | 32 heading hypotheses |
| 2 | `code2_static_A_noisyUWB.py` | A static, B moves | `UWB_NOISE_STD` (0.1 – 1 m, default 0.25) | `NUM_PARTICLES` (default 128) |
| 3 | `code3_both_moving_lowNoise.py` | both move | 0.01 m | `N_BEARING × N_HEADING` (default 16 × 16) |
| 4 | `code4_both_moving_noisyUWB.py` | both move | `UWB_NOISE_STD` (0.1 – 1 m, default 0.25) | `N_BEARING × N_HEADING` (default 16 × 16) |

Each file is a self-contained **PyQt6 dashboard** running a 50 Hz simulation: it simulates the motion, noisy odometry and noisy UWB, runs the filter, shows a live map and logs every step to `results.csv`.

---

## 6. Running

```bash
pip install numpy PyQt6 pandas matplotlib

cd v1_simplified_2d_upf
python code4_both_moving_noisyUWB.py      # runs until you close the window
```

Each run writes `results.csv` in the folder you launch it from, and **overwrites the previous one**. After a run, move and rename it, for example:

```
results/UPF_Code4_BothMoving_NoisyUWB_Result_30s.csv
```

Settings are plain constants at the top of each file (`UWB_NOISE_STD`, `NUM_PARTICLES`, `N_BEARING`, `N_HEADING`).

### CSV columns

| Codes 1–2 (static A) | Codes 3–4 (both moving) |
|---|---|
| `True_X, True_Y, Est_X, Est_Y` (A's frame, scoring only) | `True_Rel_X, True_Rel_Y, Est_Rel_X, Est_Rel_Y` (B in A's body frame) |
| `UWB_Distance, Position_Error, Heading_Error` | `UWB_Distance, Position_Error, Heading_Error` |
| `A_in_B_Error, Active_Modes` | `Active_Modes` |

---

## 7. Analysing results

```bash
python analyze_results.py results/          # all csv files in a folder
python analyze_results.py run1.csv run2.csv # specific files
```

It detects the file type automatically and writes to `analysis_output/`:

- `<run>_main.png`: error vs time, error vs distance, true vs estimated path, distances, heading error, active modes
- `<run>_extra.png`: error distribution (CDF), and relative or gauge-independent error
- `comparison.png`: all runs overlaid
- `summary.csv`: mean/median/RMSE/percentile errors, convergence time, heading error, measured UWB noise

---

## 8. Results so far (simulation only)

Random start poses, 50 Hz, odometry noise σ = (3 mm, 2 mm, 0.002 rad) per step. Random-motion simulation, mean position error.

**Both robots moving (codes 3 and 4), 6 seeds, 40 s**

| UWB noise | Mean position error | Worst seed | Mean heading error |
|---|---|---|---|
| 0.01 m | 0.44 m | 0.84 m | 0.05 rad |
| 0.1 m | 0.47 m | 0.74 m | 0.06 rad |
| 0.25 m | 0.51 m | 0.84 m | 0.07 rad |
| 0.5 m | 0.63 m | 0.93 m | 0.09 rad |
| 1.0 m | 0.77 m | 1.31 m | 0.11 rad |

**Static A, B moving (codes 1 and 2)**

| Code | UWB noise | Result |
|---|---|---|
| 1 | 0.01 m | ~1.7 m mean position error over 60 s (10 seeds) |
| 2 | 0.25 m | ~1.1 m over ~40 s with 128 particles (5 seeds); 6.1 m with 32 particles |
| 2 | 1.0 m | Unreliable (~14 m): range noise is comparable to the starting distance, so heading hypotheses cannot be separated |

For static A the position error includes an unobservable rotation about A; see the A-in-B and distance errors in `summary.csv` for the fair measure.

> The error never reaches zero even with perfect UWB. The remaining error is odometry drift of the robots, which UWB cannot remove, and it grows as the robots move apart.

---

## 9. Known limitations

- **Simulation only so far**: no hardware validation yet.
- **Simulation settings are optimistic for hardware**: 50 Hz and 0.5–2 m/s speeds (the paper uses 10 Hz and ≤ 1 m/s; a TurtleBot4 is much slower), and the robots' separation is not bounded.
- **UWB and odometry noise are modelled as white Gaussian noise**. Real VIO and UWB errors are correlated and biased (the paper makes the same observation).
- **Static A**: the rotation about A cannot be observed. Moving A even slightly (and sharing its odometry) makes it observable. Spinning A in place does not help, because the range does not change.
- **Straight-line relative motion** leaves the pose ambiguous (several modes stay alive, shown as `Active_Modes`).
- **Very noisy UWB (≈ 1 m) with static A** is unreliable at the start, when the robots are close.

---

## 10. Roadmap

- [x] 2D adaptation with static A and moving B
- [x] Both robots moving
- [x] Noisy-UWB variants (0.1 – 1 m)
- [x] Universal analysis script
- [ ] Re-run simulations with paper-like settings (10 Hz, ~0.3 m/s, bounded area)
- [ ] Hardware experiments with real odometry and UWB ranging
- [ ] Fit the gauge rotation against a motion-capture ground truth for static-A runs
- [ ] Test correlated / biased noise models
