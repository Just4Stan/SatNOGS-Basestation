"""UHF 432 MHz matching network — annotated functional blocks + Smith chart.

Component-level schematic lives in RF_HAT/rf_hat_circuit.py.  For the
thesis figure we group the passives into four functional sub-networks
drawn in a clean left-right flow, and plot the TX impedance trajectory
on a Smith chart.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from _style import OUT, COL


# ---------- Smith chart helpers ---------------------------------------------
def gamma(Z, Z0=50.0):
    return (Z - Z0) / (Z + Z0)


def draw_smith(ax, Z0=50.0):
    ax.set_aspect("equal")
    ax.set_xlim(-1.18, 1.18)
    ax.set_ylim(-1.18, 1.18)
    ax.axis("off")
    ax.add_patch(Circle((0, 0), 1.0, facecolor="white",
                        edgecolor=COL["fg"], lw=1.0))
    for r in [0.2, 0.5, 1.0, 2.0, 5.0]:
        cx = r / (1 + r)
        rr = 1.0 / (1 + r)
        ax.add_patch(Circle((cx, 0), rr, facecolor="none",
                            edgecolor=COL["muted"], lw=0.5, alpha=0.6))
        ax.text(cx + rr - 0.02, 0.035, f"{r:g}", fontsize=6.5,
                color=COL["muted"], ha="right")
    for x in [0.2, 0.5, 1.0, 2.0, 5.0]:
        for sign in (+1, -1):
            cy = sign / x
            rr = 1.0 / x
            th = np.linspace(0, 2 * np.pi, 400)
            px = 1 + rr * np.cos(th)
            py = cy + rr * np.sin(th)
            m = px ** 2 + py ** 2 <= 1.0 + 1e-9
            ax.plot(px[m], py[m], color=COL["muted"], lw=0.5, alpha=0.6)
    ax.plot([-1, 1], [0, 0], color=COL["fg"], lw=0.6)


def annot(ax, Z, label, Z0=50.0, color=COL["rf"], dxy=(0.05, 0.05)):
    g = gamma(Z, Z0)
    ax.plot(g.real, g.imag, "o", color=color, ms=6, zorder=4)
    ax.annotate(label, (g.real, g.imag),
                xytext=(g.real + dxy[0], g.imag + dxy[1]),
                fontsize=8.5, color=color, weight="bold",
                arrowprops=dict(arrowstyle="-", color=color, lw=0.6))


# ---------- Figure ----------------------------------------------------------
fig = plt.figure(figsize=(15, 8))
gs = fig.add_gridspec(1, 2, width_ratios=[1.75, 1.0], wspace=0.10)
ax_sch = fig.add_subplot(gs[0, 0])
ax_sm  = fig.add_subplot(gs[0, 1])

# ==== Left panel: functional block schematic ===============================
ax_sch.set_xlim(0, 14)
ax_sch.set_ylim(0, 9)
ax_sch.set_aspect("equal")
ax_sch.axis("off")

ax_sch.text(7, 8.65, "UHF 432 MHz matching network   —   M17 / TI SWRR122 topology",
            ha="center", fontsize=11.5, weight="bold", color=COL["rf"])
ax_sch.text(7, 8.30, "four functional sub-networks  ·  PA bias tee  ·  TX ladder  ·  TRX coupling  ·  LNA balun",
            ha="center", fontsize=9, color=COL["muted"], style="italic")


def fblock(xy, w, h, title, body, face=COL["bg_rf"], edge=COL["rf"]):
    x, y = xy
    p = FancyBboxPatch((x, y), w, h,
                       boxstyle="round,pad=0.02,rounding_size=0.08",
                       linewidth=1.0, edgecolor=edge, facecolor=face, zorder=2)
    ax_sch.add_patch(p)
    ax_sch.text(x + w / 2, y + h - 0.32, title, ha="center", va="top",
                fontsize=9.8, weight="bold", color=edge)
    ax_sch.text(x + w / 2, y + h / 2 - 0.35, body, ha="center", va="center",
                fontsize=8.5, color=COL["fg"])


def port(xy, w, h, label, sub=""):
    x, y = xy
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                       linewidth=1.0, edgecolor=COL["fg"], facecolor="white", zorder=2)
    ax_sch.add_patch(p)
    ax_sch.text(x + w / 2, y + h / 2 + 0.18, label, ha="center", va="center",
                fontsize=10, weight="bold", color=COL["fg"])
    if sub:
        ax_sch.text(x + w / 2, y + h / 2 - 0.25, sub, ha="center", va="center",
                    fontsize=8, color=COL["muted"])


def conn(x0, y0, x1, y1, text="", color=COL["bus"], lw=1.4, off=(0, 0.22)):
    a = FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="->", mutation_scale=14,
                        linewidth=lw, color=color, zorder=1.5)
    ax_sch.add_patch(a)
    if text:
        mx = (x0 + x1) / 2 + off[0]
        my = (y0 + y1) / 2 + off[1]
        ax_sch.text(mx, my, text, ha="center", va="center", fontsize=8,
                    color=color, style="italic",
                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.95),
                    zorder=3)


# --- CC1200 on the LEFT
port((0.3, 3.3), 1.7, 2.3,
     "CC1200",
     sub="pin 17  PA\npin 18  TRX_SW\npins 19 / 20  LNA_P / N")

# --- PA bias tee (top)
fblock((2.7, 7.0), 4.2, 1.2,
       "PA bias tee",
       "L4 56 nH    ·    R3 18 Ω  ||  C5 56 pF\nfrom  AVDD_FILT  (3.3 V)",
       face=COL["bg_pwr"], edge=COL["power"])

# --- TX low-pass ladder (upper middle)
fblock((2.7, 5.0), 6.5, 1.4,
       "TX low-pass ladder   (PA → SMA)",
       "series:   C2 39 pF   ·   L1 15 nH   ·   L2 43 nH   ·   L3 22 nH   ·   C27 1 nF\n"
       "shunt:    C1 2.2 pF (fb)   ·   C4 6.2 pF (GND)")

# --- TRX coupling (middle)
fblock((2.7, 3.2), 6.5, 1.2,
       "TRX coupling   (TX/RX switch node)",
       "C3  5.1 pF   (to TX ladder)    ·    L9  15 nH   (to LNA balun)")

# --- LNA balun (bottom)
fblock((2.7, 1.2), 6.5, 1.5,
       "LNA differential balun   (SMA → LNA_P / N)",
       "L6 27 nH  bias       ·   L7 56 nH  bridge\n"
       "L8 27 nH  coupling   ·   C8 5.1 pF   ·   C9 5.1 pF",
       face=COL["bg_pi"], edge=COL["pi"])

# --- SMA + antenna (right)
port((10.3, 3.9), 2.2, 1.4, "SMA",
     sub="50 Ω  ·  UFL pigtail")
# Antenna glyph
ax_sch.plot([13.0, 13.0], [3.9, 3.1], color=COL["fg"], lw=1.2)
ax_sch.plot([13.0, 13.0], [3.1, 2.4], color=COL["fg"], lw=1.5)
for dx, dy in [(-0.35, 0.3), (0.35, 0.3), (-0.35, -0.3), (0.35, -0.3)]:
    ax_sch.plot([13.0, 13.0 + dx], [3.1, 3.1 + dy], color=COL["fg"], lw=1.1)
ax_sch.text(13.0, 2.15, "Siretta Oscar 44\nYagi 9 dBi",
            ha="center", fontsize=8, color=COL["rf"])

# ==== Connections (routed around blocks, not through them) ================
# CC1200 PA pin  →  TX ladder (left edge)
conn(2.0, 5.35, 2.7, 5.35, "PA", color=COL["rf"], off=(-0.15, 0.25))
# PA bias tee bottom → PA pin feed (short vertical into TX ladder top-left)
conn(3.3, 7.0, 3.3, 6.4, "+V", color=COL["power"], off=(0.3, 0))
# TX ladder right edge  →  SMA
conn(9.2, 5.35, 10.3, 4.8, "50 Ω", color=COL["rf"], off=(0.15, 0.35))

# CC1200 TRX_SW pin  →  TRX coupling (left edge)
conn(2.0, 4.30, 2.7, 3.8, "TRX_SW", color=COL["bus"], off=(0.0, 0.28))
# TRX coupling  →  TX ladder   (C3 branch, vertical up)
conn(5.5, 4.4, 5.5, 5.0, "C3", color=COL["rf"], off=(0.3, 0))
# TRX coupling  →  LNA balun   (L9 branch, vertical down)
conn(6.5, 3.2, 6.5, 2.7, "L9", color=COL["rf"], off=(0.3, 0))

# CC1200 LNA_P/N  ←  LNA balun (left edge)
conn(2.7, 2.0, 2.0, 3.6, "LNA_P / N", color=COL["pi"], off=(-0.2, 0.15))

# LNA balun right edge  ←  SMA (RX path from SMA down then left)
ax_sch.plot([10.3, 9.5], [4.2, 4.2], color=COL["pi"], lw=1.4)
ax_sch.plot([9.5,  9.5], [4.2, 2.0], color=COL["pi"], lw=1.4)
conn(9.5, 2.0, 9.2, 2.0, "RX  50 Ω", color=COL["pi"], off=(-0.3, 0.25))

# ==== Right panel: Smith chart ============================================
draw_smith(ax_sm)
ax_sm.set_title("Smith chart   (Z₀ = 50 Ω,  f = 432 MHz)",
                fontsize=10.5, pad=10, color=COL["rf"])

Z0 = 50.0
trace = np.array([
    50 + 0j,        # SMA reference
    50 + 8j,        # after C27 DC block
    48 + 18j,       # after L3
    52 + 28j,       # after L2
    55 + 25j,       # CC1200 TX load (target)
])
g = gamma(trace, Z0)
ax_sm.plot(g.real, g.imag, "-", color=COL["rf"], lw=1.8,
           label="TX path  (SMA  →  CC1200 PA load)")

g_rx = gamma(50 + 30j, Z0)
ax_sm.plot(g_rx.real, g_rx.imag, "s", color=COL["pi"], ms=7,
           label="RX target   Z = 50 + j30 Ω")

annot(ax_sm, 50 + 0j,  "SMA\n50 Ω",                      dxy=(-0.40, -0.22), color=COL["fg"])
annot(ax_sm, 55 + 25j, "CC1200 TX\n55 + j25 Ω",          dxy=(0.04, 0.22),   color=COL["rf"])
annot(ax_sm, 50 + 30j, "CC1200 RX\n50 + j30 Ω",          dxy=(-0.82, 0.28),  color=COL["pi"])

ax_sm.legend(loc="lower center", fontsize=8, frameon=False,
             bbox_to_anchor=(0.5, -0.06))

fig.savefig(OUT / "matching_network_annotated.png")
print(f"wrote {OUT / 'matching_network_annotated.png'}")
