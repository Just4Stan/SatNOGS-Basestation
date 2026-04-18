"""RSSI distribution by elevation bin.

Reads every station.py per-pass CSV in captures/20260409 + 20260412 and
compares the RSSI distribution at low elevation (<5°, antenna looking at
the horizon / ground) vs high elevation (>40°, antenna looking at cold sky).
The difference is the antenna-temperature swing and the key empirical
evidence for the board-noise-floor analysis in the thesis.
"""
import csv
import glob
import numpy as np
import matplotlib.pyplot as plt
from _plot_style import COL, save

ROOT = "../../captures"
SESSIONS = ["20260409", "20260412"]


def load_rssi():
    """Returns (rssi_low, rssi_high) arrays in dBm, binned by elevation."""
    low, high = [], []
    for sess in SESSIONS:
        for path in glob.glob(f"{ROOT}/{sess}/*.csv"):
            with open(path) as f:
                for row in csv.DictReader(f):
                    try:
                        r  = float(row["rssi"])
                        el = float(row["el_act"])
                    except (KeyError, ValueError):
                        continue
                    if r <= -140 or r >= 0:          # drop obvious invalid
                        continue
                    if el < 5:
                        low.append(r)
                    elif el > 40:
                        high.append(r)
    return np.array(low), np.array(high)


rssi_low, rssi_high = load_rssi()
bins = np.arange(-120, -70, 1.0)

fig, ax = plt.subplots(figsize=(8.5, 4.6))

ax.hist(rssi_low,  bins=bins, density=True, alpha=0.55,
        color=COL["c4"], edgecolor=COL["c4"],
        label=f"Horizon (EL < 5°, N={len(rssi_low):,})")
ax.hist(rssi_high, bins=bins, density=True, alpha=0.55,
        color=COL["c2"], edgecolor=COL["c2"],
        label=f"High passes (EL > 40°, N={len(rssi_high):,})")

# Median markers — one-number summary per group
for data, col, label in [(rssi_low, COL["c4"], "low"),
                         (rssi_high, COL["c2"], "high")]:
    if len(data):
        ax.axvline(np.median(data), color=col, ls="--", lw=1.2, alpha=0.8)

# Annotate ΔRSSI between medians
if len(rssi_low) and len(rssi_high):
    delta = np.median(rssi_low) - np.median(rssi_high)
    ax.text(0.02, 0.95,
            f"Δmedian = {delta:+.1f} dB\n"
            f"(horizon − high)",
            transform=ax.transAxes, va="top", ha="left",
            fontsize=9, color=COL["text"],
            bbox=dict(boxstyle="round,pad=0.4",
                      facecolor=COL["fill_soft"],
                      edgecolor=COL["grid"], linewidth=0.6))

ax.set_xlabel("RSSI (dBm)")
ax.set_ylabel("Density")
ax.set_title("CC1200 RSSI distribution — horizon vs high-elevation pointing")
ax.legend(loc="upper right")

save(fig, "rssi_indoor_outdoor")
