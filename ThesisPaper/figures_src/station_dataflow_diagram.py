"""station.py runtime dataflow — TLE → ephem → AZ/EL + freq → CC1200 → decoders → SiDS."""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from _style import OUT, COL


def box(ax, xy, w, h, title, sub="", face="white", edge=COL["fg"], title_size=9.5):
    x, y = xy
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                       linewidth=1.0, edgecolor=edge, facecolor=face, zorder=2)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h - 0.3, title, ha="center", va="top",
            fontsize=title_size, weight="bold", color=COL["fg"])
    if sub:
        ax.text(x + w / 2, y + h / 2 - 0.25, sub, ha="center", va="center",
                fontsize=8, color=COL["muted"])


def arrow(ax, p1, p2, text="", color=COL["bus"], lw=1.3, off=(0, 0.15), dashed=False):
    ls = "--" if dashed else "-"
    a = FancyArrowPatch(p1, p2, arrowstyle="->", mutation_scale=12,
                        linewidth=lw, color=color, linestyle=ls, zorder=1.5)
    ax.add_patch(a)
    if text:
        mx = (p1[0] + p2[0]) / 2 + off[0]
        my = (p1[1] + p2[1]) / 2 + off[1]
        ax.text(mx, my, text, ha="center", va="center", fontsize=7.8,
                color=color, style="italic",
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.95),
                zorder=3)


fig, ax = plt.subplots(figsize=(13, 8))
ax.set_xlim(0, 14)
ax.set_ylim(0, 9.2)
ax.set_aspect("equal")
ax.axis("off")

ax.text(7, 8.9, "station.py   runtime dataflow",
        ha="center", fontsize=12, weight="bold", color=COL["pi"])
ax.text(7, 8.55, "daemon loop @ 2 Hz   ·   ~/.station_status.json written @ 1 Hz",
        ha="center", fontsize=9, color=COL["muted"])

# External inputs (top row)
box(ax, (0.3, 7.05), 2.6, 1.1, "CelesTrak",
    sub="TLE catalogue\nHTTP fetch (daily)",
    face=COL["bg_soft"], edge=COL["muted"])
box(ax, (3.3, 7.05), 2.6, 1.1, "SatNOGS DB",
    sub="transmitter profile\nlookup + fallback",
    face=COL["bg_soft"], edge=COL["muted"])
box(ax, (6.3, 7.05), 2.9, 1.1, "Operator phone",
    sub="HTTPS dashboard\nlat  /  lon  /  elev",
    face=COL["bg_soft"], edge=COL["muted"])

# Core: ephem + pass scheduler
box(ax, (2.3, 5.0), 4.7, 1.5,
    "PyEphem   +   pass scheduler",
    sub="Body.compute(observer)  @ 2 Hz\n→  AZ, EL, range-rate ṙ,  next AOS / LOS",
    face=COL["bg_pi"], edge=COL["pi"], title_size=10)

# rf_hat.CC1200Link
box(ax, (8.1, 5.0), 4.8, 1.5,
    "rf_hat.CC1200Link",
    sub="doppler_shift()   Δf = f₀ · ṙ / c\nfreq_to_regs(f)  →  FREQ2 / FREQ1 / FREQ0",
    face=COL["bg_rf"], edge=COL["rf"], title_size=10)

# Arrows: external → core
arrow(ax, (1.6, 7.05), (3.5, 6.5), "TLE", color=COL["muted"], off=(-0.4, 0.15))
arrow(ax, (4.6, 7.05), (5.0, 6.5), "sat profile\n(freq, mod, baud)",
      color=COL["muted"], off=(0.7, 0.0))
arrow(ax, (7.7, 7.05), (6.3, 6.5), "observer\nposition",
      color=COL["muted"], off=(0.7, 0.1))

# Core inter-arrow: ephem → rf_hat
arrow(ax, (7.0, 5.75), (8.1, 5.75), "ṙ,  f₀",
      color=COL["pi"], off=(0, 0.20))

# Rotator path
box(ax, (0.3, 2.7), 2.6, 1.3, "rotctld",
    sub="TCP :4533\nEasyComm   \"M AZ EL\"",
    face="white", edge=COL["mcu"])
box(ax, (3.3, 2.7), 2.6, 1.3, "MotorPCB",
    sub="PID 100 Hz\nTB6642FG  →  motors",
    face=COL["bg_mcu"], edge=COL["mcu"])
arrow(ax, (3.8, 5.0), (1.6, 4.0), "AZ, EL (°)",
      color=COL["mcu"], off=(-0.8, 0.20))
arrow(ax, (2.9, 3.35), (3.3, 3.35), "USB CDC",
      color=COL["bus"], off=(0, 0.2))

# RF path
box(ax, (7.9, 2.7), 2.6, 1.3, "RF HAT Pico",
    sub="COBS + CRC-16\nSPI1 → UHF CC1200",
    face=COL["bg_mcu"], edge=COL["mcu"])
box(ax, (10.7, 2.7), 2.2, 1.3, "UHF CC1200",
    sub="432 MHz ± Δf\nRX_FIFO / TX_FIFO",
    face=COL["bg_rf"], edge=COL["rf"])
arrow(ax, (10.5, 5.0), (9.2, 4.0), "SET_FREQ_WORD\n(0x34)",
      color=COL["rf"], off=(0.6, 0.15))
arrow(ax, (10.5, 3.35), (10.7, 3.35), "SPI 5 MHz",
      color=COL["bus"], off=(0, 0.2))

# RX path
box(ax, (10.7, 0.6), 2.6, 1.4, "Decoders",
    sub="G3RUH (firmware)\nAX.25  ·  AX.100  ·  USP",
    face="white", edge=COL["rf"])
arrow(ax, (12.0, 2.7), (12.0, 2.0), "RX_READ\n(0x20)",
      color=COL["rf"], off=(0.8, 0))

# Outputs
box(ax, (6.8, 0.6), 3.2, 1.4, "satnogs.py",
    sub="SiDS POST\ndb.satnogs.org/api/telemetry",
    face=COL["bg_pi"], edge=COL["pi"])
arrow(ax, (10.7, 1.3), (10.0, 1.3), "decoded frames",
      color=COL["bus"], off=(0, 0.2))

box(ax, (3.1, 0.6), 3.2, 1.4, "dashboard.py",
    sub="HTTPS :5000   ·   live AZ / EL\nRSSI   ·   pass timeline",
    face=COL["bg_pi"], edge=COL["pi"])
arrow(ax, (4.7, 2.0), (4.7, 2.7), "status.json\n(1 Hz)",
      color=COL["pi"], off=(1.0, 0), dashed=True)

# Footer legend
ax.text(0.3, 0.15,
        "Arrows: solid = command / data flow     ·     dashed = status / feedback",
        fontsize=8, color=COL["muted"], style="italic")

fig.savefig(OUT / "station_dataflow_diagram.png")
print(f"wrote {OUT / 'station_dataflow_diagram.png'}")
