# Result Report — Task #1: Update README to match codebase

## Summary

Applied all 7 required corrections to `README.md`, making every factual claim verifiable against the repo as it exists. The existing 10-section structure, numbering, table style, and tone are preserved; §1, §3 (differences table), §9, and §10 were left untouched (no correction forced a one-line touch). All numbers in the new §8 committed-runs table were cross-checked cell-by-cell against `v1_simplified_2d_upf/analysis_output/summary.csv` and match the plan's authoritative fact #9 exactly. Only `README.md` was modified — no code edits, no new files, no renames.

## Files

| File | Action | Notes |
|---|---|---|
| `README.md` | Modified | Corrections 1–7 applied (§2, §4, §5, §6, §7, §8). §9/§10 untouched. |

Source files read (not modified) to verify facts: `v1_simplified_2d_upf/code{1..4}_*.py`, `analyze_results.py`, `analysis_output/summary.csv`, `results/Info.txt`, `LICENSE`.

## Build / Tests

No build step and no test suite exist for a documentation-only change. Verification was done via targeted `grep` checks, a programmatic cell-by-cell cross-check of the new §8 table against `summary.csv`, a markdown sanity check (balanced code fences, intact section numbering), and `git status --porcelain`.

## Acceptance Verification

| # | Criterion (verbatim from plan) | Result |
|---|---|---|
| 1 | "`grep -n "add your own" README.md` returns nothing; README states Apache-2.0 and points to `LICENSE`." | **PASS** — grep exits 1 (no match). Line 42: "This repo is licensed under **Apache-2.0** (see `LICENSE`)…"; LICENSE also listed in the §4 tree (line 70). |
| 2 | "§4 tree lists `LICENSE`, the 4 `results/` CSV names + `Info.txt`, and `analysis_output/` contents; a reader can find the `15m` explanation near the tree." | **PASS** — tree shows `LICENSE`, all 4 real CSV names (incl. the `Resul_15m` typo), `Info.txt`, and full `analysis_output/` (`info.txt`, `summary.csv`, `comparison.png`, 8 main/extra PNGs). The `15m` callout sits directly below the tree (line 94). |
| 3 | "§6 CSV-columns table includes `Timestamp` and `Step`; the rename example matches the committed naming style." | **PASS** — line 136: "All codes write `Timestamp, Step` first…". Rename example (line 125) is `results/UPF_Code4_Both_moving_Noisy_Result_15m.csv`, matching the committed file. |
| 4 | "§7 lists `--hz`, `--thr`, `--out`, `--show` with their defaults and the no-arg behavior; the ≥2-runs condition for `comparison.png` is stated." | **PASS** — line 149 documents no-arg (all `*.csv` next to script); line 154 lists all four options with defaults (`--hz` default 50, `--thr` default 1.0 m); line 160 states `comparison.png` is "written only when analysing ≥ 2 runs". |
| 5 | "§8 contains both the original multi-seed tables (numbers unchanged) with a not-in-repo label, and a new 4-run table whose numbers match `analysis_output/summary.csv` (cross-check every cell)." | **PASS** — both multi-seed tables kept verbatim under "### Earlier multi-seed study — run logs not included in this repo" (lines 169–171). New "Committed runs in this repo" table (lines 193–202) cross-checked cell-by-cell: every value equals the plan's authoritative fact #9, and each is a faithful 2-decimal rounding of `summary.csv` (e.g. max_true_dist 13.766→13.77, uwb 0.248/0.254→0.25). |
| 6 | "README mentions `random.seed(42)` reproducibility and the GUI vs headless distinction." | **PASS** — line 109: all four call `random.seed(42)`, reproducible, remove for randomised trials; line 130: codes open a PyQt6 window (GUI required), `analyze_results.py` runs headless (Agg). |
| 7 | "Spot-check holds against code: 0.01 / 0.25 m UWB, 32 / 128 particles, 16×16 grid, 50 Hz, odometry σ (0.003, 0.002, 0.002) — no contradiction introduced." | **PASS** — README values all match source: code1 `uwb_noise_std=0.01` + `num_particles=32`; code2 `UWB_NOISE_STD=0.25`/`NUM_PARTICLES=128`; codes 3–4 `N_BEARING=N_HEADING=16`; `QTimer.start(20)` → 50 Hz; odometry gauss σ (0.003, 0.002, 0.002). No contradiction introduced. |
| 8 | "`git status --porcelain` shows exactly one modified file: `README.md`." | **PASS** — ` M README.md` is the only tracked change (`git diff --name-only` → `README.md`). The untracked `?? planning/` entry is the pre-existing plan/result directory, not a modification. |

## Blockers

None.

## Next Steps

- Task #1 complete and all acceptance criteria pass. Nothing further required for this task.
- Note (out of scope, do not act without instruction): the untracked `planning/` directory contains the work plan and this result report; it is expected to remain uncommitted per the "do not commit" constraint.
