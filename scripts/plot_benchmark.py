"""Figures and table of the trajectory-tracking benchmark (NMPC against the cascaded controller).

    python3 scripts/plot_benchmark.py            # reads results/benchmark/, writes docs/

docs/benchmark_paths.png   top: top view of the top-speed laps; bottom: position error over time
docs/benchmark_rmse.png    position RMSE against top speed, one panel per scenario
and a Markdown table on stdout for the README.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "quad_mission"))
from quad_mission.trajectory import Trajectory  # noqa: E402

CONTROLLERS = ("pid", "nmpc")
COLORS = {"nmpc": "#2a78d6", "pid": "#eb6834"}          # categorical slots 1 and 2
LABELS = {"nmpc": "NMPC (acados)", "pid": "Cascaded PID + geometric attitude"}
REF_COLOR, INK, MUTED, GRID = "#52514e", "#0b0b0b", "#52514e", "#e4e3df"
SCENARIOS = {"nominal": "Nominal", "wind": "Wind 1 N + gusts ±0.5 N", "payload": "Hidden payload, +20 % weight"}


def style(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def top_speed_laps(run, entry):
    """Rows of the laps flown at top speed: the window the RMSE is computed on.

    The path starts when the reference starts moving; the window then follows from the
    path parameters (Trajectory.top_speed_window), exactly as in the mission node."""
    tr = Trajectory(entry["shape"], entry["size"], entry["max_speed"], entry["laps"], [0.0, 0.0, 0.0])
    speed = np.sqrt(run["vx_ref"] ** 2 + run["vy_ref"] ** 2 + run["vz_ref"] ** 2)
    t0 = run["t"][np.argmax(speed > 1e-9)]
    t1, t2 = tr.top_speed_window()
    keep = (run["t"] >= t0 + t1) & (run["t"] <= t0 + t2)
    return run[keep], t0 + t1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--results", default=str(ROOT / "results" / "benchmark"))
    ap.add_argument("--docs", default=str(ROOT / "docs"))
    ap.add_argument("--path-speed", type=float, default=5.0)
    args = ap.parse_args()
    res, docs = Path(args.results), Path(args.docs)
    summary = {}
    for c in CONTROLLERS:
        p = res / f"benchmark_{c}.json"
        if p.exists():
            summary.update(json.loads(p.read_text()))
    speeds = sorted({e["max_speed"] for e in summary.values()})
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10})

    # Figure 1: path (top row) and error over time (bottom row) at the path speed.
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.2), gridspec_kw={"height_ratios": [1.15, 1]})
    for col, (scen, title) in enumerate(SCENARIOS.items()):
        ax_path, ax_err = axes[0, col], axes[1, col]
        style(ax_path)
        style(ax_err)
        ref_drawn = False
        notes = []
        for c in CONTROLLERS:
            key = f"{c}_lemniscate_{scen}_{args.path_speed:g}"
            path = res / "runs" / f"run_{key}.csv"
            if key not in summary or not path.exists():
                continue
            laps, t_start = top_speed_laps(np.genfromtxt(path, delimiter=",", names=True), summary[key])
            if not ref_drawn:
                ax_path.plot(laps["x_ref"], laps["y_ref"], "--", color=REF_COLOR, lw=1.2, label="Reference")
                ref_drawn = True
            ax_path.plot(laps["x"], laps["y"], color=COLORS[c], lw=1.6, label=LABELS[c])
            err = 100 * np.sqrt((laps["x"] - laps["x_ref"]) ** 2 + (laps["y"] - laps["y_ref"]) ** 2
                                + (laps["z"] - laps["z_ref"]) ** 2)
            ax_err.plot(laps["t"] - t_start, err, color=COLORS[c], lw=1.4)
            notes.append(f"{'NMPC' if c == 'nmpc' else 'PID'} {100 * summary[key]['rmse']:.1f} cm")
        ax_path.set_title(title, color=INK, fontsize=11, loc="left")
        ax_path.set_aspect("equal")
        ax_path.set_xlabel("x [m]", color=MUTED)
        ax_err.set_xlabel("time at top speed [s]", color=MUTED)
        ax_err.set_ylim(bottom=0)
        ax_err.set_title("RMSE  " + "   ".join(notes), color=INK, fontsize=9, loc="left")
    axes[0, 0].set_ylabel("y [m]", color=MUTED)
    axes[1, 0].set_ylabel("position error [cm]", color=MUTED)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.01, 0.955), ncol=3, frameon=False, fontsize=10)
    fig.suptitle(f"Figure eight, 4 m × 2 m, {args.path_speed:g} m/s top speed (Webots, DJI Mavic 2 Pro): "
                 "the two laps flown at top speed", x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(docs / "benchmark_paths.png", dpi=150, facecolor="white")

    # Figure 2: RMSE against speed, one panel per scenario (shared y axis).
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.8), sharey=True)
    width = 0.36
    for ax, (scen, title) in zip(axes, SCENARIOS.items()):
        style(ax)
        x = np.arange(len(speeds))
        for i, c in enumerate(CONTROLLERS):
            vals = [100 * summary.get(f"{c}_lemniscate_{scen}_{s:g}", {}).get("rmse", np.nan) for s in speeds]
            bars = ax.bar(x + (i - 0.5) * width, vals, width - 0.04, color=COLORS[c], label=LABELS[c])
            for b, val in zip(bars, vals):
                if np.isfinite(val):
                    ax.text(b.get_x() + b.get_width() / 2, val, f"{val:.1f}", ha="center", va="bottom",
                            fontsize=8, color=INK)
        ax.set_xticks(x, [f"{s:g} m/s" for s in speeds])
        ax.set_title(title, color=INK, fontsize=11, loc="left")
    axes[0].set_ylabel("position RMSE [cm]", color=MUTED)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(0.01, 0.9), ncol=2, frameon=False, fontsize=10)
    fig.suptitle("Tracking error over the top-speed laps against top speed (lower is better)",
                 x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.84))
    fig.savefig(docs / "benchmark_rmse.png", dpi=150, facecolor="white")

    # Table for the README.
    print("| Scenario | Top speed | Cascaded PID: RMSE / max error | NMPC: RMSE / max error | NMPC solve time: median / p99 / max |")
    print("|---|---|---|---|---|")
    for scen, title in SCENARIOS.items():
        for s in speeds:
            p = summary.get(f"pid_lemniscate_{scen}_{s:g}")
            n = summary.get(f"nmpc_lemniscate_{scen}_{s:g}")
            cell = lambda e: f"{100 * e['rmse']:.1f} / {100 * e['max_error']:.1f} cm" if e else "-"
            solve = (f"{n['solve_ms_median']:.2f} / {n['solve_ms_p99']:.2f} / {n['solve_ms_max']:.2f} ms"
                     if n and n.get("solve_ms_median") is not None else "-")
            print(f"| {title} | {s:g} m/s | {cell(p)} | {cell(n)} | {solve} |")


if __name__ == "__main__":
    main()
