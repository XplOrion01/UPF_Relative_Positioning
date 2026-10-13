# Work Plan: Update README to match codebase

**Purpose:** Bring `README.md` into factual agreement with the current code, data, and repo state (verified by orchestrator review on `docs/update-readme`).

## Objective

The repo's README is mostly accurate but has drifted from the codebase in ~8 concrete places (missing LICENSE mention, stale "add your own LICENSE" note, missing CLI options, unexplained `15m` filename suffix, CSV columns omitted, results tables referencing runs not in the repo, etc.). End state: every factual claim in `README.md` is verifiable against the repo as it exists; no other file is modified.

## Context

- Repo: 2D UPF (Unscented Particle Filter) relative-pose estimation with UWB + odometry, two robots. Four self-contained PyQt6 dashboard simulation codes + one headless analysis script under `v1_simplified_2d_upf/`.
- Orchestrator has fully read: `README.md`, all four `code*.py`, `analyze_results.py`, `results/` (4 CSVs + `Info.txt`), `analysis_output/` (8 PNGs, `comparison.png`, `summary.csv`, `info.txt`), `LICENSE` (Apache-2.0), and git history.
- User decision (2026-10-12): for §8 "Results so far", **keep** the existing multi-seed tables (clearly labeled: run logs NOT in this repo) **and add** a new table for the 4 committed runs using numbers from `analysis_output/summary.csv`.

## Tasks

### Task 1: Rewrite/update `README.md` with verified corrections
- **Files:** `README.md` (modify — only file touched)
- **Spec:** see `work_plan_task_1_update_readme.md` (self-contained: full list of corrections + verified facts to use).
- **Acceptance:**
  1. §2 references: repo stated as Apache-2.0 licensed; "add your own `LICENSE` file here" sentence removed.
  2. §4 tree: includes `LICENSE`; `results/` and `analysis_output/` contents match the repo (4 CSVs + `Info.txt`; 8 PNGs + `comparison.png` + `summary.csv` + `info.txt`).
  3. README explains the `15m` suffix in committed run filenames (max A–B separation covered, per `results/Info.txt`) and states the initial separation is 1 m.
  4. §6 CSV-columns table lists `Timestamp` and `Step`; example renamed run follows the real convention (e.g. `UPF_Code4_Both_moving_Noisy_Result_15m.csv`).
  5. §7 documents all CLI options (`--hz`, `--thr`, `--out`, `--show`) and the no-arg default; states `comparison.png` is only written for ≥2 runs.
  6. §8: multi-seed tables kept with an explicit "earlier multi-seed study — run logs not in this repo" label; new table for the 4 committed runs (numbers from `analysis_output/summary.csv`); seeded reproducibility (`random.seed(42)`) mentioned.
  7. §5/§6 note: the four dashboard codes need a GUI (PyQt6); `analyze_results.py` runs headless.
  8. No factual claim contradicts the code (spot-check constants: UWB 0.01 / 0.25 m, 32 / 128 particles, 16×16 grid, 50 Hz, odometry σ = 3 mm / 2 mm / 0.002 rad).
  9. Existing structure, section numbering, and tone preserved; sections that are already accurate (1, 3, 5 core table, 9, 10) minimally touched.
  10. No other file modified (`git status` shows only `README.md` changed).

## Parallel (after above complete)

(none — single task)

## Out of Scope

- Any code change — including the stale header comments in `code1`/`code2` (claim an "8 position × 4 heading" grid and "systematic resampling"; actual implementation is heading-only hypotheses, no resampling). Flagged as a finding for the user, not fixed here.
- Creating `requirements.txt` / `pyproject.toml`.
- Renaming committed CSV files (e.g. the `Resul_15m` typo) or regenerating analysis output.
- Translations or a new top-level docs site.

## Notes

- [Assumption] English stays the only README language.
- [Assumption] the multi-seed tables' numbers themselves remain untouched — only labeled as not-in-repo.
- [Assumption] the 4 committed runs are the current best single-run reference set; §8 wording should not imply they are the multi-seed means.
- Risk: summary.csv numbers are from runs at UWB noise 0.25 m for codes 2/4 (measured 0.248 / 0.254 m); the new table must cite the noise setting actually used.
