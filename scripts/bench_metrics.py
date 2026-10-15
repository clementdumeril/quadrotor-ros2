"""Tracking metrics of the benchmark runs, against the exact reference.

    python3 scripts/bench_metrics.py            # recompute every run in results/benchmark/

The reference logged with each odometry sample is interpolated in the latest
/quad/reference message. When that message is stamped a few milliseconds after the
odometry sample, the interpolation clamps to its first sample, so about 0.1 % of the
logged reference points are off by up to 3 cm. The path is known in closed form, so the
metrics use it instead: reference = Trajectory.sample(t - t_start), where t_start is the
start time returned by the action, or, for runs logged without it, the start time that
best fits the logged reference (sub-millisecond).
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "quad_mission"))
from quad_mission.trajectory import Trajectory  # noqa: E402

MASS, G = 0.5, 9.81


def trajectory(entry, height=1.5):
    return Trajectory(entry["shape"], entry["size"], entry["max_speed"], entry["laps"], [0.0, 0.0, height])


def fit_start(rows, entry):
    """Start time that best matches the logged reference (median distance)."""
    tr = trajectory(entry)
    t, ref = rows[:, 0], rows[:, 11:14]
    speed = np.linalg.norm(rows[:, 14:17], axis=1)
    guess = t[np.argmax(speed > 1e-9)]
    sub = slice(None, None, 10)

    def cost(t0):
        m = (t[sub] >= t0) & (t[sub] <= t0 + tr.duration)
        P = np.array([tr.sample(x - t0)[0] for x in t[sub][m]])
        return np.median(np.linalg.norm(P - ref[sub][m], axis=1))

    coarse = min(np.arange(guess - 0.1, guess + 0.05, 0.001), key=cost)
    return float(min(np.arange(coarse - 0.001, coarse + 0.001, 0.0001), key=cost))


def apply_exact_reference(rows, entry, t_start):
    """Overwrite the reference columns with the exact path while it is being flown."""
    tr = trajectory(entry)
    for i, t in enumerate(rows[:, 0]):
        if t_start <= t <= t_start + tr.duration:
            p, v, _ = tr.sample(t - t_start)
            rows[i, 11:14], rows[i, 14:17] = p, v
    return rows


def metrics(rows, entry, t_start):
    tr = trajectory(entry)
    t1, t2 = tr.top_speed_window()
    win = (rows[:, 0] >= t_start + t1) & (rows[:, 0] <= t_start + t2)
    e = np.linalg.norm(rows[win, 1:4] - rows[win, 11:14], axis=1)
    exy = np.linalg.norm(rows[win, 1:3] - rows[win, 11:13], axis=1)
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (rows[win, 8] ** 2 + rows[win, 9] ** 2), -1, 1)))
    solve = rows[win, 21][np.isfinite(rows[win, 21])]
    return {
        "t_start": t_start,
        "rmse": float(np.sqrt(np.mean(e ** 2))), "rmse_xy": float(np.sqrt(np.mean(exy ** 2))),
        "max_error": float(e.max()), "max_tilt_deg": float(tilt.max()),
        "thrust_saturated": float(np.mean(rows[win, 17] >= 0.99 * 2 * MASS * G)),
        "solve_ms_median": float(np.median(solve)) if solve.size else None,
        "solve_ms_p99": float(np.percentile(solve, 99)) if solve.size else None,
        "solve_ms_max": float(solve.max()) if solve.size else None,
        "samples": int(win.sum()),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--results", default=str(ROOT / "results" / "benchmark"))
    ap.add_argument("--controllers", nargs="+", default=["pid", "nmpc"])
    args = ap.parse_args()
    res = Path(args.results)
    for summary_path in [res / f"benchmark_{c}.json" for c in args.controllers]:
        if not summary_path.exists():
            continue
        summary = json.loads(summary_path.read_text())
        for tag, entry in summary.items():
            path = res / "runs" / f"run_{tag}.csv"
            if not path.exists():
                continue
            with open(path, newline="") as f:
                reader = csv.reader(f)
                header = next(reader)
                rows = np.array([[float(x) for x in r] for r in reader])
            t_start = entry.get("t_start")
            if t_start is None:
                t_start = fit_start(rows, entry)
            rows = apply_exact_reference(rows, entry, t_start)
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(rows.tolist())
            old = entry["rmse"]
            entry.update(metrics(rows, entry, t_start))
            print(f"{tag}: rmse {100 * old:.2f} -> {100 * entry['rmse']:.2f} cm, max {100 * entry['max_error']:.1f} cm")
        summary_path.write_text(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
