"""Doppler shift over a real tracked pass — reads station.py CSV log."""
import csv
from datetime import datetime
import matplotlib.pyplot as plt
from _plot_style import COL, save

CSV = "../../captures/20260412/20260412_184526_MIMAN.csv"
SAT_NAME = "MIMAN"


def parse(path):
    t0 = None
    t, dop, el, freq = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                ts = datetime.fromisoformat(row["t"])
                d = float(row["doppler_hz"])
                e = float(row["el_act"])
                fq = float(row["freq_hz"])
            except (KeyError, ValueError):
                continue
            if t0 is None:
                t0 = ts
            t.append((ts - t0).total_seconds() / 60.0)  # minutes
            dop.append(d / 1000.0)                     # kHz
            el.append(e)
            freq.append(fq)
    return t, dop, el, freq


t, dop, el, freq = parse(CSV)
f0_mhz = freq[0] / 1e6 if freq else 0
swing_khz = max(dop) - min(dop)

fig, ax1 = plt.subplots(figsize=(9, 4.5))

# Doppler trace (primary axis, KUL blue)
ax1.plot(t, dop, color=COL["c2"], lw=1.8, label="Measured Doppler")
ax1.set_xlabel("Time from AOS (min)")
ax1.set_ylabel("Doppler shift Δf (kHz)", color=COL["c2"])
ax1.tick_params(axis="y", labelcolor=COL["c2"])
ax1.axhline(0, color=COL["muted"], lw=0.7, ls=":")

# Elevation envelope (twin axis, KUL dark)
ax2 = ax1.twinx()
ax2.fill_between(t, 0, el, color=COL["c1"], alpha=0.12, linewidth=0)
ax2.plot(t, el, color=COL["c1"], lw=1.0, alpha=0.7)
ax2.set_ylabel("Elevation (°)", color=COL["c1"])
ax2.tick_params(axis="y", labelcolor=COL["c1"])
ax2.set_ylim(0, max(90, max(el) * 1.05))
ax2.grid(False)
ax2.spines["top"].set_visible(False)

# TCA marker
if el:
    t_tca = t[el.index(max(el))]
    ax1.axvline(t_tca, color=COL["c4"], lw=1.0, ls="--", alpha=0.7)
    ax1.annotate("TCA", xy=(t_tca, 0), xytext=(t_tca + 0.15, max(dop) * 0.55),
                 fontsize=9, color=COL["c4"], weight="bold")

# Title
ax1.set_title(f"Doppler shift — {SAT_NAME} pass   "
              f"(max EL = {max(el):.0f}°,  total swing {swing_khz:.1f} kHz at {f0_mhz:.1f} MHz)")

ax1.legend(loc="upper right")
save(fig, "doppler_real_pass")
