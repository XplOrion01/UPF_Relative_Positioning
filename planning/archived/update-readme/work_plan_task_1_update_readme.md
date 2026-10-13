# Task Plan: Update README to match codebase

**Purpose:** Make `README.md` factually correct against the current repo (verified by orchestrator against code + data).

## Objective

`README.md` has drifted from the codebase in ~8 concrete places. Rewrite/update it so every factual claim is verifiable against the repo as it exists, keeping the existing structure, section numbering, and tone. Only `README.md` may change.

## Context

Repo = 2D UPF (Unscented Particle Filter) relative-pose estimation with UWB + odometry, two robots. Layout:

```
.
├── LICENSE                       # Apache-2.0 (full text, already in repo)
├── README.md                     # ← the file you update
└── v1_simplified_2d_upf/
    ├── code1_static_A_lowNoise.py        # 411 lines
    ├── code2_static_A_noisyUWB.py        # 417 lines
    ├── code3_both_moving_lowNoise.py     # 318 lines
    ├── code4_both_moving_noisyUWB.py     # 318 lines
    ├── analyze_results.py                # 335 lines, headless (Agg backend)
    ├── results/                          # 4 committed runs + Info.txt
    │   ├── Info.txt                      # "testing was done for a distance upto 15 meters"
    │   ├── UPF_Code1_Static_A_LowNoise_Result_15m.csv
    │   ├── UPF_Code2_Static_A_Noisy_Result_15m.csv
    │   ├── UPF_Code3_Both_moving_LowNoise_Resul_15m.csv   (note the "Resul" typo in the real name)
    │   └── UPF_Code4_Both_moving_Noisy_Result_15m.csv
    └── analysis_output/                  # output of analyze_results.py on those 4 CSVs
        ├── info.txt                      # "the analysis of all 4 codes have been saved"
        ├── summary.csv
        ├── comparison.png
        ├── UPF_Code1_Static_A_LowNoise_15m_main.png / _extra.png
        ├── UPF_Code2_Static_A_Noisy_15m_main.png / _extra.png
        ├── UPF_Code3_Both_moving_LowNoise_15m_main.png / _extra.png
        └── UPF_Code4_Both_moving_Noisy_15m_main.png / _extra.png
```

**Verified facts (authoritative — use these, re-read the code only if in doubt):**

1. **LICENSE** = Apache License 2.0, full text at repo root. The current README's line "add your own `LICENSE` file here" is stale.
2. **All four codes**: PyQt6 dashboard, `QTimer` at 20 ms → **50 Hz** simulation. B (and A in codes 3/4) start at **1.0 m** separation (`d0 = 1.0` / B at radius 1). Motion re-randomized every 100 steps: `v ∈ [0.01, 0.04]` per step (→ 0.5–2 m/s at 50 Hz), `w ∈ [−0.02, 0.02]` rad/step. Odometry noise σ = **(0.003 m, 0.002 m, 0.002 rad)** per step (identical in all four codes). All codes call `random.seed(42)` → **reproducible** runs (code comment: "Reproducible benchmark run. Change/remove this for random trials.").
3. **Code 1** (`code1_static_A_lowNoise.py`): static A, B moves; `GaugeFixedUPF(num_particles=32)` hardcoded; UWB noise `uwb_noise_std = 0.01` hardcoded inside `simulation_step` (not in a top-of-file settings block). Gauge frame: B's start azimuth pinned to A's +x; estimate rotated by `start_phi` for scoring only; writes `A_in_B_Error` (gauge-invariant error).
4. **Code 2** (`code2_static_A_noisyUWB.py`): same as code 1 but top-of-file settings `UWB_NOISE_STD = 0.25` (comment: "Try 0.1 ... 1.0") and `NUM_PARTICLES = 128` (heading hypotheses, 360/N deg apart).
5. **Code 3** (`code3_both_moving_lowNoise.py`): both move, A estimates B; settings `UWB_NOISE_STD = 0.01`, `N_BEARING = 16`, `N_HEADING = 16` → 256 particles on a bearing×heading grid. Rotational drift modeled as extra process noise from A's heading noise (plays the paper's pseudo-state role).
6. **Code 4** (`code4_both_moving_noisyUWB.py`): same as code 3 with `UWB_NOISE_STD = 0.25`.
7. **CSV columns** (all codes write `Timestamp, Step` first):
   - Codes 1–2: `Timestamp, Step, True_X, True_Y, Est_X, Est_Y, UWB_Distance, Position_Error, Heading_Error, A_in_B_Error, Active_Modes`
   - Codes 3–4: `Timestamp, Step, True_Rel_X, True_Rel_Y, Est_Rel_X, Est_Rel_Y, UWB_Distance, Position_Error, Heading_Error, Active_Modes` (no `A_in_B_Error` — no gauge to remove)
   - Each run writes `results.csv` into the launch directory, overwriting the previous one.
8. **`analyze_results.py`** usage: `python analyze_results.py` (no args → every `*.csv` next to the script), `... <file>.csv ...` (explicit files), `... <folder>/` (every `*.csv` in folder). Options: `--hz 50` (default 50), `--thr 1.0` (convergence threshold, default 1.0), `--out DIR`, `--show`. Output: `<run>_main.png` (6 panels), `<run>_extra.png` (CDF + gauge-independent/relative error), `comparison.png` (**only if >1 run**), `summary.csv`. Runs headless (matplotlib Agg) — no GUI needed.
9. **`analysis_output/summary.csv`** (the 4 committed runs; hz=50, thr=1.0):

| run (short) | type | duration_s | mean_err_m | median_err_m | rmse_m | p95_err_m | max_err_m | converged_<1m_at_s | mean_heading_err_deg | mean_true_dist_m | max_true_dist_m | measured_uwb_std_m | final_modes | mean_A_in_B_err_m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Code1 LowNoise | static A, B moving | 41.7 | 0.095 | 0.063 | 0.124 | 0.219 | 0.864 | 0.02 | 1.46 | 6.54 | 13.77 | 0.010 | 2 | 0.108 |
| Code2 Noisy (0.25 m) | static A, B moving | 21.9 | 3.586 | 3.912 | 3.873 | 5.542 | 5.665 | (never) | 21.86 | 8.83 | 13.77 | 0.248 | 6 | 0.279 |
| Code3 LowNoise | both bots moving | 30.0 | 0.325 | 0.179 | 0.568 | 1.952 | 2.078 | 2.06 | 2.85 | 6.67 | 13.10 | 0.010 | 3 | — |
| Code4 Noisy (0.25 m) | both bots moving | 34.9 | 0.327 | 0.178 | 0.590 | 1.965 | 3.974 | 2.56 | 4.27 | 7.61 | 16.36 | 0.254 | 5 | — |

10. **`results/Info.txt`**: "Here all the results of all four present codes are available, testing was done for a distance upto 15 meters" → the **`15m` suffix in run filenames = max A–B separation covered in the test**, not the start distance.
11. The existing README's multi-seed tables in §8 (both-moving: 6 seeds × 40 s; static A: 10 seeds / 5 seeds) describe runs **not present in the repo**.

**User decision (binding):** §8 must KEEP the multi-seed tables (numbers untouched) under an explicit label such as "earlier multi-seed study — run logs not included in this repo", and ADD a new table for the 4 committed runs using the summary.csv numbers above.

## Tasks

### Task 1: Update `README.md`
- **Files:** `README.md` (modify)
- **Do:** Apply every correction below, preserving the current 10-section structure, numbering, tables style, and voice. Where a section is already accurate (e.g. §1, §3 differences table, §9 limitations, §10 roadmap), leave it untouched except for the specific items listed.

  Required corrections:
  1. **§2 References** — remove the stale "add your own `LICENSE` file here" sentence; state this repo is licensed under **Apache-2.0** (see `LICENSE`). Keep the paper citation + bibtex + original-repo note (adjust wording: the license warning now refers to code *copied from* the original repo, since this repo has its own license).
  2. **§4 Repository layout** — tree must include `LICENSE`; show real `results/` (4 CSVs + `Info.txt`) and `analysis_output/` (8 PNGs + `comparison.png` + `summary.csv` + `info.txt`) contents. Add a one-line note explaining the `15m` filename suffix (max separation covered, per `results/Info.txt`) and that committed runs started at 1 m.
  3. **§5 The four codes** — keep the table (it is accurate). Add one sentence: all four call `random.seed(42)`, so a run is reproducible; remove the seed line for randomized trials. (Also note code 1's UWB noise is hardcoded at 0.01 m, not in a settings block, if the surrounding text implies settings exist in all four.)
  4. **§6 Running** — keep `pip install numpy PyQt6 pandas matplotlib`. Keep the "writes `results.csv` in the launch folder, overwrites" note. Add: the four codes open a PyQt6 window (GUI display required); `analyze_results.py` runs headless. Fix the rename example to the real convention, e.g. `results/UPF_Code4_Both_moving_Noisy_Result_15m.csv`, and state the suffix convention: `<max-separation>` or run tag, matching committed files. Add `Timestamp` and `Step` to the CSV-columns table (both column groups).
  5. **§7 Analysing results** — document the full CLI: no-arg default (all `*.csv` next to the script), explicit file list, folder, and options `--hz` (default 50), `--thr` (default 1.0 m), `--out`, `--show`. State `comparison.png` is written only when analyzing ≥2 runs.
  6. **§8 Results so far** — per the user decision: (a) keep the two multi-seed tables verbatim, wrapped in a clear label that those logs are not in the repo; (b) add a new table "Committed runs in this repo (one per code, see `results/` + `analysis_output/summary.csv`)" with columns at least: run, UWB noise used, duration (s), mean position error (m), median (m), max error (m), max A–B distance (m); for static-A runs also include mean A-in-B error (m); use the verified numbers in fact #9. Keep the existing closing notes (odometry-drift explanation, static-A gauge caveat).
  7. **§9 / §10** — unchanged unless a correction above forces a one-line touch (do not rewrite).
- **Notes:**
  - Tone: technical, plain, matches existing README (short paragraphs, tables, `>` callouts).
  - Do not invent numbers: only use values listed in "Verified facts".
  - Keep all existing correct content (paper table, bibtex, gauge explanation, differences table, limitations).
  - Markdown must stay valid (rendered tables, code fences).
- **Acceptance:**
  1. `grep -n "add your own" README.md` returns nothing; README states Apache-2.0 and points to `LICENSE`.
  2. §4 tree lists `LICENSE`, the 4 `results/` CSV names + `Info.txt`, and `analysis_output/` contents; a reader can find the `15m` explanation near the tree.
  3. §6 CSV-columns table includes `Timestamp` and `Step`; the rename example matches the committed naming style.
  4. §7 lists `--hz`, `--thr`, `--out`, `--show` with their defaults and the no-arg behavior; the ≥2-runs condition for `comparison.png` is stated.
  5. §8 contains both the original multi-seed tables (numbers unchanged) with a not-in-repo label, and a new 4-run table whose numbers match `analysis_output/summary.csv` (cross-check every cell).
  6. README mentions `random.seed(42)` reproducibility and the GUI vs headless distinction.
  7. Spot-check holds against code: 0.01 / 0.25 m UWB, 32 / 128 particles, 16×16 grid, 50 Hz, odometry σ (0.003, 0.002, 0.002) — no contradiction introduced.
  8. `git status --porcelain` shows exactly one modified file: `README.md`.

## Out of Scope

- Code files (including stale header comments in `code1`/`code2` about an "8×4 grid" and "systematic resampling").
- New files (no `requirements.txt`, no extra docs).
- Renaming CSVs or regenerating analysis output.

## Notes

- [Assumption] English-only README.
- [Assumption] §9/§10 need no changes beyond what corrections 1–6 force.
- [Assumption] multi-seed table numbers are kept verbatim (only relabeled).
