"""Power distribution — 24V → 5V buck → 3.3V chain with decoupling."""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
from _style import OUT, COL


def rail(ax, x0, x1, y, label, color, lw=2.8):
    ax.plot([x0, x1], [y, y], color=color, lw=lw, solid_capstyle="round", zorder=1)
    ax.text(x0 - 0.2, y, label, ha="right", va="center", fontsize=10.5,
            weight="bold", color=color)


def block(ax, xy, w, h, title, body="", face="#ffffff", edge=COL["fg"]):
    x, y = xy
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                       linewidth=1.0, edgecolor=edge, facecolor=face, zorder=3)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h - 0.3, title, ha="center", va="top",
            fontsize=9.5, weight="bold", color=COL["fg"])
    if body:
        ax.text(x + w / 2, y + h / 2 - 0.25, body, ha="center", va="center",
                fontsize=8.2, color=COL["muted"])


def cap(ax, x, y_rail, y_gnd, label, dy=-0.12):
    """IEEE cap symbol rail→GND with label placed on the side (dy offset of rail)."""
    ax.plot([x, x], [y_rail, y_rail - 0.22], color=COL["fg"], lw=1.0)
    ax.plot([x - 0.14, x + 0.14], [y_rail - 0.22, y_rail - 0.22],
            color=COL["fg"], lw=1.2)
    ax.plot([x - 0.11, x + 0.11], [y_rail - 0.30, y_rail - 0.30],
            color=COL["fg"], lw=1.2)
    ax.plot([x, x], [y_rail - 0.30, y_gnd], color=COL["fg"], lw=1.0)
    ax.text(x, y_rail + dy, label, fontsize=7.8, ha="center", va="top",
            color=COL["muted"],
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.95))


def varrow(ax, x, y0, y1, color=COL["power"], lw=1.6):
    a = FancyArrowPatch((x, y0), (x, y1), arrowstyle="->", mutation_scale=12,
                        linewidth=lw, color=color, zorder=2)
    ax.add_patch(a)


fig, ax = plt.subplots(figsize=(13, 8))
ax.set_xlim(0, 14)
ax.set_ylim(-0.6, 9.5)
ax.set_aspect("equal")
ax.axis("off")

# Title — above all content
ax.text(7, 9.15, "Power distribution",
        ha="center", fontsize=12, weight="bold", color=COL["pi"])
ax.text(7, 8.80, "RF HAT v1 (field-deployed)  ·  topology identical on v2 MOTOR_RF_HAT",
        ha="center", fontsize=9, color=COL["muted"], style="italic")

# Rail Y-positions
y_24  = 7.2
y_5   = 4.7
y_33  = 2.3
y_gnd = 0.4

# Rails
rail(ax, 1.3, 13.5, y_24, "+24 V",  COL["power"])
rail(ax, 4.8, 13.5, y_5,  "+5 V",   COL["power"])
rail(ax, 8.2, 12.0, y_33, "+3.3 V", COL["power"])
rail(ax, 0.5, 13.8, y_gnd, "GND",   COL["muted"], lw=1.8)

# --- 24 V source (far left)
block(ax, (0.3, y_24 - 0.5), 1.0, 1.0, "24 V",
      body="XT30\nbattery",
      face=COL["bg_pwr"], edge=COL["power"])
# source to rail
ax.plot([0.8, 0.8], [y_24 - 0.5, y_gnd], color=COL["muted"], lw=1.2)

# --- Motor drivers (consumer of 24 V) — placed BELOW the 24 V rail, above 5 V rail
block(ax, (1.8, 5.9), 2.8, 0.9, "Motor drivers  (24 V direct)",
      body="v1 TB6642FG   ·   v2 A4950",
      face=COL["bg_mot"], edge=COL["motor"])
# tap from 24 V rail down into motor drivers (on top edge)
ax.plot([3.2, 3.2], [y_24, 6.8], color=COL["power"], lw=1.4)

# --- LMR51420 buck — straddles the 24 V (input) and 5 V (output) rails
bx, by, bw, bh = 5.2, y_5 + 0.45, 2.6, 2.05
block(ax, (bx, by), bw, bh, "LMR51420 buck",
      body="24 → 5 V / 2 A\nLCSC C7296200\nSOT-23-6",
      face=COL["bg_pwr"], edge=COL["power"])
# input from 24 V rail (top of buck)
varrow(ax, bx + 0.5, y_24, by + bh, lw=1.6)
# output to 5 V rail (bottom of buck)
varrow(ax, bx + bw - 0.5, by, y_5, lw=1.6)

# Input / output caps on the buck (placed off to the sides where labels are clear)
cap(ax, 4.6, y_24, y_gnd, "4.7 µF\n50 V  (C1)")
cap(ax, 8.3, y_5,  y_gnd, "22 µF\n(C3)")

# --- Pi 3A+ consuming 5 V
block(ax, (8.9, y_5 + 1.0), 2.4, 1.1, "Raspberry Pi 3A+",
      body="5 V on pins 2 / 4\ninternal LDO  →  3.3 V\n(logic only)",
      face=COL["bg_pi"], edge=COL["pi"])
ax.plot([10.1, 10.1], [y_5, y_5 + 1.0], color=COL["power"], lw=1.2)
cap(ax, 9.5, y_5, y_gnd, "100 nF")

# --- RP2040 Pico (RF HAT) — 5 V in, 3.3 V out via RT6150
px, py, pw, ph = 11.6, y_5 + 0.45, 1.9, 2.05
block(ax, (px, py), pw, ph, "RP2040 Pico",
      body="VSYS = 5 V\nRT6150 buck-boost\n→ 3V3_OUT  (pin 36)",
      face=COL["bg_mcu"], edge=COL["mcu"])
# 5 V in (top)
varrow(ax, px + 0.4, y_5, py, lw=1.4)
# 3.3 V out (bottom)
varrow(ax, px + pw - 0.4, py, y_33, lw=1.6)
cap(ax, px + 0.2, y_33, y_gnd, "10 nF\n+ 100 nF")

# --- Ferrite bead between 3.3 V rail and AVDD_FILT
fx = 10.5
ax.add_patch(Rectangle((fx - 0.22, y_33 - 0.12), 0.44, 0.24,
                       facecolor="#fde68a", edgecolor=COL["fg"], lw=1.0, zorder=4))
ax.plot([10.0, fx - 0.22], [y_33, y_33], color=COL["power"], lw=1.4)
ax.plot([fx + 0.22, 9.3],  [y_33, y_33], color=COL["power"], lw=1.4)
ax.text(fx, y_33 + 0.35, "FB  1 kΩ @ 100 MHz",
        ha="center", fontsize=7.8, color=COL["muted"])

# AVDD_FILT branch goes LEFT from the FB to feed CC1200 block
ax.plot([9.3, 9.3], [y_33, 1.45], color=COL["power"], lw=1.4)
ax.text(9.1, 1.85, "AVDD_FILT", ha="right", va="center", fontsize=8.5,
        weight="bold", color=COL["power"])

# --- CC1200 ×2 block (consumer of 3.3 V via AVDD_FILT)
block(ax, (4.4, 0.7), 4.6, 1.2, "CC1200  ×2   (UHF + VHF)",
      body="per-rail decoupling (×9):\n"
           "10 nF  ·  47 nF  ·  100 nF  ·  220 nF       ·       40 MHz XO, 15 pF load",
      face=COL["bg_rf"], edge=COL["rf"])
# arrow from AVDD_FILT drop to CC1200 block top
varrow(ax, 8.95, 1.45, 1.9, color=COL["power"], lw=1.4)

# Footnote
ax.text(7, -0.35,
        "Decoupling values shown are orders of magnitude; refer to "
        "RF_HAT/rf_hat_circuit.py for exact designators.",
        ha="center", fontsize=8, color=COL["muted"], style="italic")

fig.savefig(OUT / "power_distribution_diagram.png")
print(f"wrote {OUT / 'power_distribution_diagram.png'}")
