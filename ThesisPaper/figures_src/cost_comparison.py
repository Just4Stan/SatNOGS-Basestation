"""Ground-station cost comparison — Yaesu / SPID / SatNOGS v3 / this work.

Static data: public retail prices (Nov 2025) for rotator-only and for complete
station (rotator + controller + ~3 m coax + basic antenna). 'This work' column
uses v2 MOTOR_RF_HAT BOM from the thesis cost analysis.
"""
import numpy as np
import matplotlib.pyplot as plt
from _plot_style import COL, save

LABELS = ["Yaesu\nG-5500", "SPID\nRAS", "SatNOGS\nv3", "This\nwork"]
ROT    = [830, 1250, 400, 45]   # rotator only (EUR)
STATION = [1200, 1800, 600, 95] # complete station (EUR)
ACCURACY = ["~1°", "<1°", "~1°", "<1° EL\n0–3° AZ"]

x = np.arange(len(LABELS))
w = 0.38

fig, ax = plt.subplots(figsize=(8.5, 4.8))
b1 = ax.bar(x - w/2, ROT,     w, color=COL["c1"], label="Rotator only")
b2 = ax.bar(x + w/2, STATION, w, color=COL["c4"], label="Complete station")

# Value labels on top of bars
for rect, v in zip(b1, ROT):
    ax.text(rect.get_x() + rect.get_width()/2, v + 40,
            f"€{v}", ha="center", fontsize=9, color=COL["c1"], weight="bold")
for rect, v in zip(b2, STATION):
    ax.text(rect.get_x() + rect.get_width()/2, v + 40,
            f"€{v}", ha="center", fontsize=9, color=COL["c4"], weight="bold")

# Accuracy annotations along a dedicated bottom row (not colliding with bars)
ymin = -190
for xi, acc in zip(x, ACCURACY):
    ax.text(xi, ymin, acc, ha="center", va="top", fontsize=8.5,
            color=COL["muted"], style="italic")
ax.text(-0.85, ymin, "Accuracy:", ha="left", va="top", fontsize=8.5,
        color=COL["muted"], style="italic")

# "13× cheaper" callout — offset arrow to a clean area above the bars
cheapest_station = STATION[-1]
reference        = STATION[0]
ratio            = reference / cheapest_station
ax.annotate(f"{ratio:.0f}× cheaper\nthan Yaesu",
            xy=(x[-1] + w/2, cheapest_station + 30),
            xytext=(x[-1] - 0.4, 900),
            ha="center", fontsize=9.5, color=COL["c5"], weight="bold",
            arrowprops=dict(arrowstyle="->", color=COL["c5"], lw=1.2,
                            connectionstyle="arc3,rad=0.2"))

ax.set_xticks(x)
ax.set_xticklabels(LABELS)
ax.set_ylabel("Cost (€)")
ax.set_ylim(ymin - 60, 2100)
ax.set_title("Ground-station cost comparison")
ax.legend(loc="upper right")
ax.spines["bottom"].set_visible(False)
ax.tick_params(axis="x", length=0, pad=28)  # push x labels below accuracy row

save(fig, "cost_comparison")
