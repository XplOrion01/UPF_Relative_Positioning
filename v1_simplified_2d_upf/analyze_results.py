"""
analyze_results.py  --  universal result analyser for the UPF relative-pose codes
=================================================================================
Works on the results.csv files written by ALL four codes:
  * static A + moving B   (columns True_X, True_Y, Est_X, Est_Y, ... A_in_B_Error, Active_Modes)
  * both bots moving      (columns True_Rel_X, True_Rel_Y, Est_Rel_X, Est_Rel_Y, ... Active_Modes)
The column layout is detected automatically.

USAGE
  python analyze_results.py                       -> analyses every *.csv next to this script
  python analyze_results.py run1.csv run2.csv     -> analyses the files you name
  python analyze_results.py folder/               -> analyses every *.csv in that folder
Options
  --hz 50          update rate used to turn Step into seconds (default 50)
  --thr 1.0        error threshold (m) used for the "converged" time (default 1.0)
  --out DIR        output folder (default: analysis_output next to the csv files / script)
  --show           also open the plots in a window

OUTPUT (in the output folder)
  <name>_main.png      6 panels: error vs time, error vs distance, path, distances, heading, modes
  <name>_extra.png     error distribution (CDF) + relative error / gauge-independent error
  comparison.png       (only if >1 file) all runs overlaid
  summary.csv          one row of numbers per run (copy straight into your report)
  The summary table is also printed in the terminal.

Needs: python 3, numpy, pandas, matplotlib
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib

if "--show" not in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------
# loading / schema detection
# ----------------------------------------------------------------------------
SCHEMAS = [
    # (true x, true y, est x, est y)
    ("True_Rel_X", "True_Rel_Y", "Est_Rel_X", "Est_Rel_Y"),
    ("True_X", "True_Y", "Est_X", "Est_Y"),
]


def load(path, hz):
    df = pd.read_csv(path)
    df.columns = [c.strip() for c in df.columns]
    cols = None
    for s in SCHEMAS:
        if all(c in df.columns for c in s):
            cols = s
            break
    if cols is None:
        raise ValueError(f"{os.path.basename(path)}: unrecognised columns {list(df.columns)}")
    tx, ty, ex, ey = (df[c].astype(float).to_numpy() for c in cols)
    d = pd.DataFrame({
        "step": df["Step"].astype(float).to_numpy(),
        "tx": tx, "ty": ty, "ex": ex, "ey": ey,
        "uwb": df["UWB_Distance"].astype(float).to_numpy(),
        "err": df["Position_Error"].astype(float).to_numpy(),
    })
    d["t"] = d["step"] / hz
    d["true_dist"] = np.hypot(tx, ty)
    d["est_dist"] = np.hypot(ex, ey)
    d["head_deg"] = np.degrees(df["Heading_Error"].astype(float).to_numpy()) if "Heading_Error" in df else np.nan
    d["modes"] = df["Active_Modes"].astype(float).to_numpy() if "Active_Modes" in df else np.nan
    d["a_in_b"] = df["A_in_B_Error"].astype(float).to_numpy() if "A_in_B_Error" in df else np.nan
    d["uwb_resid"] = d["uwb"] - d["true_dist"]
    kind = "static A, B moving" if cols[0] == "True_X" else "both bots moving"
    return d, kind


def label_of(path):
    name = os.path.splitext(os.path.basename(path))[0]
    return name.replace("_Result", "").replace("_Resul", "")


# ----------------------------------------------------------------------------
# metrics
# ----------------------------------------------------------------------------
def converged_time(t, err, thr, hold_s=2.0):
    """first time after which the error stays below thr for hold_s seconds (nan if never)."""
    ok = err < thr
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.02
    need = max(1, int(hold_s / dt))
    run = 0
    for i, o in enumerate(ok):
        run = run + 1 if o else 0
        if run >= need:
            return t[i - need + 1]
    return np.nan


def summarise(d, kind, name, thr):
    e = d["err"].to_numpy()
    n = len(e)
    last = e[int(0.9 * n):]
    warm = d[d["t"] >= 2.0]  # ignore first 2 s (filter still collapsing the initial hypotheses)
    ew = warm["err"].to_numpy() if len(warm) else e
    row = {
        "run": name,
        "type": kind,
        "steps": n,
        "duration_s": round(d["t"].iloc[-1], 1),
        "mean_err_m": e.mean(),
        "median_err_m": np.median(e),
        "rmse_m": np.sqrt((e ** 2).mean()),
        "p90_err_m": np.percentile(e, 90),
        "p95_err_m": np.percentile(e, 95),
        "max_err_m": e.max(),
        "mean_err_after2s_m": ew.mean(),
        "mean_err_last10pct_m": last.mean(),
        f"converged_below_{thr:g}m_at_s": converged_time(d["t"].to_numpy(), e, thr),
        "mean_heading_err_deg": np.nanmean(d["head_deg"]),
        "median_rel_err_pct": 100 * np.nanmedian(e / np.maximum(d["true_dist"], 0.2)),
        "mean_true_dist_m": d["true_dist"].mean(),
        "max_true_dist_m": d["true_dist"].max(),
        "uwb_noise_std_m": d["uwb_resid"].std(),
        "mean_dist_est_err_m": np.abs(d["est_dist"] - d["true_dist"]).mean(),
        "final_modes": d["modes"].iloc[-1],
    }
    if d["a_in_b"].notna().any():
        row["mean_A_in_B_err_m"] = d["a_in_b"].mean()
    return row


def rolling(x, w=50):
    return pd.Series(x).rolling(w, min_periods=1, center=True).median().to_numpy()


def binned(dist, err, nb=12):
    edges = np.linspace(dist.min(), dist.max() + 1e-9, nb + 1)
    idx = np.digitize(dist, edges) - 1
    cx, med, p90 = [], [], []
    for b in range(nb):
        m = idx == b
        if m.sum() >= 5:
            cx.append(dist[m].mean())
            med.append(np.median(err[m]))
            p90.append(np.percentile(err[m], 90))
    return np.array(cx), np.array(med), np.array(p90)


# ----------------------------------------------------------------------------
# plots
# ----------------------------------------------------------------------------
def main_figure(d, kind, name, out):
    fig, ax = plt.subplots(2, 3, figsize=(17, 9))
    fig.suptitle(f"{name}   ({kind})", fontsize=14, fontweight="bold")
    t = d["t"].to_numpy()

    a = ax[0, 0]
    a.plot(t, d["err"], color="#ffb74d", lw=0.8, label="error")
    a.plot(t, rolling(d["err"].to_numpy()), color="#d32f2f", lw=2, label="rolling median (1 s)")
    a.set_title("Position error vs time")
    a.set_xlabel("time [s]"); a.set_ylabel("error [m]"); a.legend(); a.grid(alpha=0.3)

    a = ax[0, 1]
    sc = a.scatter(d["true_dist"], d["err"], c=t, s=6, cmap="viridis", alpha=0.6)
    cx, med, p90 = binned(d["true_dist"].to_numpy(), d["err"].to_numpy())
    if len(cx):
        a.plot(cx, med, "r-o", lw=2, label="binned median")
        a.plot(cx, p90, "k--", lw=1.5, label="binned 90th pct")
    a.set_title("Position error vs true distance A-B")
    a.set_xlabel("true distance [m]"); a.set_ylabel("error [m]"); a.legend(); a.grid(alpha=0.3)
    plt.colorbar(sc, ax=a, label="time [s]")

    a = ax[0, 2]
    a.plot(d["tx"], d["ty"], color="#2e7d32", lw=2, label="true B")
    a.plot(d["ex"], d["ey"], color="#d32f2f", lw=1.2, ls="--", label="estimated B")
    a.plot(0, 0, "ks", ms=9, label="A")
    a.plot(d["tx"].iloc[0], d["ty"].iloc[0], "go", label="start (true)")
    a.plot(d["tx"].iloc[-1], d["ty"].iloc[-1], "g^", label="end (true)")
    a.plot(d["ex"].iloc[-1], d["ey"].iloc[-1], "r^", label="end (est.)")
    a.set_aspect("equal", adjustable="datalim")
    a.set_title("Path of B relative to A: true vs estimated")
    a.set_xlabel("x [m]"); a.set_ylabel("y [m]"); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = ax[1, 0]
    a.plot(t, d["uwb"], color="#90caf9", lw=0.8, label="UWB measured")
    a.plot(t, d["true_dist"], color="#2e7d32", lw=2, label="true distance")
    a.plot(t, d["est_dist"], color="#d32f2f", lw=1.2, ls="--", label="estimated distance")
    a.set_title(f"Distance A-B  (UWB noise measured = {d['uwb_resid'].std():.3f} m)")
    a.set_xlabel("time [s]"); a.set_ylabel("distance [m]"); a.legend(); a.grid(alpha=0.3)

    a = ax[1, 1]
    if d["head_deg"].notna().any():
        a.plot(t, d["head_deg"], color="#ce93d8", lw=0.8)
        a.plot(t, rolling(d["head_deg"].to_numpy()), color="#6a1b9a", lw=2)
    a.set_title("Heading error vs time")
    a.set_xlabel("time [s]"); a.set_ylabel("heading error [deg]"); a.grid(alpha=0.3)

    a = ax[1, 2]
    if d["modes"].notna().any():
        a.step(t, d["modes"], color="#00838f", where="post")
        a.set_ylim(bottom=0)
    a.set_title("Active modes (hypotheses with >5% weight)")
    a.set_xlabel("time [s]"); a.set_ylabel("modes"); a.grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    p = os.path.join(out, f"{name}_main.png")
    fig.savefig(p, dpi=130)
    return fig, p


def extra_figure(d, kind, name, out):
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(f"{name}   ({kind}) - extra", fontsize=13, fontweight="bold")
    e = np.sort(d["err"].to_numpy())
    cdf = np.arange(1, len(e) + 1) / len(e)
    a = ax[0]
    a.plot(e, cdf, lw=2)
    for q, c in ((0.5, "#2e7d32"), (0.9, "#ef6c00"), (0.95, "#c62828")):
        v = np.percentile(e, 100 * q)
        a.axvline(v, color=c, ls="--", lw=1)
        a.text(v, 0.05 + q * 0.1, f" {int(q*100)}%: {v:.2f} m", color=c)
    a.set_title("Error distribution (CDF)")
    a.set_xlabel("position error [m]"); a.set_ylabel("fraction of samples <= error"); a.grid(alpha=0.3)

    a = ax[1]
    t = d["t"].to_numpy()
    if d["a_in_b"].notna().any():
        a.plot(t, d["err"], color="#ffb74d", lw=1, label="Position error (A frame)")
        a.plot(t, d["a_in_b"], color="#1565c0", lw=1.5, label="A-in-B error (gauge-independent)")
        a.set_title("Static A: the rotation about A is not observable,\nso the A-in-B error is the fair measure")
        a.set_ylabel("error [m]"); a.legend()
    else:
        rel = 100 * d["err"] / np.maximum(d["true_dist"], 0.2)
        a.plot(t, rel, color="#8e24aa", lw=0.8, alpha=0.6)
        a.plot(t, rolling(rel.to_numpy()), color="#4a148c", lw=2)
        a.set_title("Error as % of the true distance")
        a.set_ylabel("error / distance [%]")
    a.set_xlabel("time [s]"); a.grid(alpha=0.3)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    p = os.path.join(out, f"{name}_extra.png")
    fig.savefig(p, dpi=130)
    return fig, p


def comparison_figure(runs, out):
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    fig.suptitle("Comparison of all runs", fontsize=14, fontweight="bold")
    for name, d, kind in runs:
        ax[0].plot(d["t"], rolling(d["err"].to_numpy()), lw=2, label=name)
        cx, med, _ = binned(d["true_dist"].to_numpy(), d["err"].to_numpy())
        if len(cx):
            ax[1].plot(cx, med, "-o", lw=2, label=name)
    ax[0].set_title("Position error vs time (rolling median)"); ax[0].set_xlabel("time [s]"); ax[0].set_ylabel("error [m]")
    ax[1].set_title("Median error vs true distance"); ax[1].set_xlabel("true distance [m]"); ax[1].set_ylabel("error [m]")
    names = [r[0] for r in runs]
    means = [r[1]["err"].mean() for r in runs]
    meds = [np.median(r[1]["err"]) for r in runs]
    x = np.arange(len(runs)); w = 0.38
    ax[2].bar(x - w / 2, means, w, label="mean")
    ax[2].bar(x + w / 2, meds, w, label="median")
    ax[2].set_xticks(x); ax[2].set_xticklabels(names, rotation=20, ha="right", fontsize=8)
    ax[2].set_title("Mean / median error"); ax[2].set_ylabel("error [m]")
    for a in ax:
        a.grid(alpha=0.3); a.legend(fontsize=8)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    p = os.path.join(out, "comparison.png")
    fig.savefig(p, dpi=130)
    return fig, p


# ----------------------------------------------------------------------------
def collect(paths):
    if not paths:
        paths = [os.path.dirname(os.path.abspath(__file__))]
    files = []
    for p in paths:
        if os.path.isdir(p):
            files += sorted(glob.glob(os.path.join(p, "*.csv")))
        else:
            files += sorted(glob.glob(p)) or [p]
    return [f for f in files if os.path.basename(f).lower() != "summary.csv"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--thr", type=float, default=1.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()

    files = collect(a.paths)
    if not files:
        sys.exit("No csv files found.")
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(files[0])), "analysis_output")
    os.makedirs(out, exist_ok=True)

    runs, rows = [], []
    for f in files:
        try:
            d, kind = load(f, a.hz)
        except Exception as ex:  # keep going if one file is not a results file
            print(f"skipped {os.path.basename(f)}: {ex}")
            continue
        name = label_of(f)
        runs.append((name, d, kind))
        rows.append(summarise(d, kind, name, a.thr))
        for fn in (main_figure, extra_figure):
            fig, p = fn(d, kind, name, out)
            print("saved", p)
            if not a.show:
                plt.close(fig)
    if not runs:
        sys.exit("Nothing to analyse.")
    if len(runs) > 1:
        fig, p = comparison_figure(runs, out)
        print("saved", p)
        if not a.show:
            plt.close(fig)

    summ = pd.DataFrame(rows)
    sp = os.path.join(out, "summary.csv")
    summ.to_csv(sp, index=False, float_format="%.3f")
    print("saved", sp)
    pd.set_option("display.width", 250, "display.max_columns", 50)
    print("\n================ SUMMARY ================")
    print(summ.set_index("run").T.to_string(float_format=lambda v: f"{v:.3f}"))
    if a.show:
        plt.show()


if __name__ == "__main__":
    main()
