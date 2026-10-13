"""Plot a flight log written by flight_test.py.

    python3 scripts/plot_flight.py logs/flight.csv docs/flight.png
"""
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

src, dst = sys.argv[1], sys.argv[2]
rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(src))]
t0 = next(r["t"] for r in rows if not math.isnan(r["sp_z"]))
rows = [r for r in rows if r["t"] >= t0]
t = [r["t"] - t0 for r in rows]

fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
for k, c in (("x", "C0"), ("y", "C1"), ("z", "C2")):
    ax[0].plot(t, [r[k] for r in rows], c, label=k)
    ax[0].plot(t, [r["sp_" + k] for r in rows], c + "--", lw=1)
ax[0].set(xlabel="time [s]", ylabel="position [m]", title="Position (solid) and setpoint (dashed)")
ax[0].legend(loc="best")
ax[1].plot([r["x"] for r in rows], [r["y"] for r in rows], "C0")
ax[1].plot([r["sp_x"] for r in rows], [r["sp_y"] for r in rows], "k--", lw=1)
ax[1].set(xlabel="x [m]", ylabel="y [m]", title="Ground track", aspect="equal")
ax[2].plot(t, [math.degrees(r["roll"]) for r in rows], label="roll")
ax[2].plot(t, [math.degrees(r["pitch"]) for r in rows], label="pitch")
ax[2].set(xlabel="time [s]", ylabel="angle [deg]", title="Attitude")
ax[2].legend(loc="best")
fig.tight_layout()
fig.savefig(dst, dpi=130)
print("saved", dst)
