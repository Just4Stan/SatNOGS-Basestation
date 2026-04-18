"""System block diagram — Pi center, buses, power topology."""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from _style import OUT, COL


def box(ax, xy, w, h, text, *, face, edge=COL["fg"], fontsize=9, weight="normal"):
    x, y = xy
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06",
                       linewidth=1.0, edgecolor=edge, facecolor=face, zorder=2)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=fontsize, color=COL["fg"], weight=weight, zorder=3)
    return (x, y, w, h)


def hlabel(ax, x, y, text, color=COL["bus"]):
    ax.text(x, y, text, ha="center", va="center", fontsize=8, color=color,
            style="italic",
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.95),
            zorder=4)


def harrow(ax, x0, x1, y, color=COL["bus"], lw=1.2, dashed=False, double=True):
    ls = "--" if dashed else "-"
    head = "<->" if double else "->"
    a = FancyArrowPatch((x0, y), (x1, y), arrowstyle=head, mutation_scale=12,
                        linewidth=lw, color=color, linestyle=ls, zorder=1.5)
    ax.add_patch(a)


def varrow(ax, x, y0, y1, color=COL["bus"], lw=1.2, dashed=False, double=False):
    ls = "--" if dashed else "-"
    head = "<->" if double else "->"
    a = FancyArrowPatch((x, y0), (x, y1), arrowstyle=head, mutation_scale=12,
                        linewidth=lw, color=color, linestyle=ls, zorder=1.5)
    ax.add_patch(a)


fig, ax = plt.subplots(figsize=(12, 8.2))
ax.set_xlim(0, 13)
ax.set_ylim(0, 9.2)
ax.set_aspect("equal")
ax.axis("off")

ax.text(6.5, 8.95, "Ground-station system block diagram",
        ha="center", fontsize=12, weight="bold", color=COL["pi"])

# Column layout
# | MotorPCB(A) | channelAB | Pi(B) | channelBC | RF HAT(C) |
# A : 0.3 - 3.2  (w 2.9)
# AB: 3.2 - 4.7
# B : 4.7 - 8.3  (w 3.6)
# BC: 8.3 - 9.8
# C : 9.8 - 12.7 (w 2.9)

# --- TOP STRIP (power sources) y = 7.5 .. 8.5 ------------------------------
box(ax, (0.3, 7.5), 2.9, 0.9,
    "24 V battery / PSU\nXT30 connector",
    face=COL["bg_pwr"], edge=COL["power"], weight="bold", fontsize=9)
box(ax, (9.8, 7.5), 2.9, 0.9,
    "LMR51420 buck  24 → 5 V / 2 A\nPico RT6150  5 → 3.3 V",
    face=COL["bg_pwr"], edge=COL["power"], fontsize=9)

# 24 V bus (top-strip) — routed ABOVE title of bottom row; clear of Pi/phone area
y24 = 7.95
ax.plot([3.2, 9.8], [y24, y24], color=COL["power"], lw=1.6, zorder=1)
a = FancyArrowPatch((9.6, y24), (9.8, y24), arrowstyle="->",
                    mutation_scale=12, color=COL["power"], lw=1.6)
ax.add_patch(a)
hlabel(ax, 6.5, y24 + 0.22, "24 V", color=COL["power"])

# --- PHONE (middle-top, centered above Pi) ---------------------------------
box(ax, (5.0, 6.1), 3.6, 0.9,
    "Operator phone\nHTTPS :5000 dashboard  ·  Geolocation API",
    face="white", edge=COL["pi"], fontsize=8.8)

# --- MIDDLE ROW (subsystem boxes) y = 3.5 .. 4.9 ---------------------------
box(ax, (0.3, 3.5), 2.9, 1.4,
    "MotorPCB (RP2040)\nEasyComm III\nPID 100 Hz  ·  IMU homing\nAZ / EL drivers",
    face=COL["bg_mcu"], edge=COL["mcu"], weight="bold", fontsize=8.8)
box(ax, (4.7, 3.5), 3.6, 1.4,
    "Raspberry Pi 3A+\nDebian 13 Trixie (armhf)\nstation.py  ·  dashboard.py\nrotctld  ·  sat_library",
    face=COL["bg_pi"], edge=COL["pi"], weight="bold", fontsize=9.2)
box(ax, (9.8, 3.5), 2.9, 1.4,
    "RF HAT (RP2040)\nCOBS + CRC-16/CCITT\nUHF CC1200 (432 MHz)\nVHF CC1200 (not pop.)",
    face=COL["bg_rf"], edge=COL["rf"], weight="bold", fontsize=8.8)

# --- BOTTOM ROW (actuators) y = 0.9 .. 1.9 ---------------------------------
box(ax, (0.3, 0.9), 1.35, 1.1,
    "AZ motor\nFAPG36-555\n244.6 tick/°",
    face=COL["bg_mot"], edge=COL["motor"], fontsize=8)
box(ax, (1.85, 0.9), 1.35, 1.1,
    "EL motor\nFAPG36-555\n126.1 tick/°",
    face=COL["bg_mot"], edge=COL["motor"], fontsize=8)
box(ax, (9.8, 0.9), 2.9, 1.1,
    "Siretta Oscar 44\nYagi 9 dBi  ·  432 MHz\nRG58 3 m  (0.9 dB)",
    face=COL["bg_rf"], edge=COL["rf"], fontsize=8.5)

# ============== BUSES ======================================================
# MotorPCB <-> Pi : USB CDC  (horizontal arrow in channel A-B)
yab = 4.2
harrow(ax, 3.25, 4.65, yab, color=COL["bus"], double=True)
# Label goes in the channel ABOVE the arrow
hlabel(ax, 3.95, yab + 0.35, "USB CDC\nhamlib  EasyComm III")

# Pi <-> RF HAT : UART  (horizontal arrow in channel B-C)
harrow(ax, 8.35, 9.75, yab, color=COL["bus"], double=True)
hlabel(ax, 9.05, yab + 0.35, "UART  /dev/serial0\n115 200 baud  ·  COBS")

# Pi <-> Phone : WiFi (vertical, dashed)
varrow(ax, 6.5, 4.9, 6.1, color=COL["pi"], dashed=True, double=True)
hlabel(ax, 6.5, 5.5, "WiFi hotspot\nSSID: SatNOGS-xxxx", color=COL["pi"])

# MotorPCB -> AZ motor (24 V PWM)
varrow(ax, 1.0, 3.5, 2.0, color=COL["motor"])
hlabel(ax, 1.05, 2.75, "24 V PWM\nTB6642FG / A4950", color=COL["motor"])
# MotorPCB -> EL motor (no label to avoid clutter)
varrow(ax, 2.55, 3.5, 2.0, color=COL["motor"])

# RF HAT -> antenna (SMA)
varrow(ax, 11.25, 3.5, 2.0, color=COL["rf"])
hlabel(ax, 11.25, 2.75, "SMA  50 Ω\nM17 matching", color=COL["rf"])

# ============== POWER RAILS ================================================
# 24 V spur down the left side from top-strip PSU to MotorPCB (motor supply)
ax.plot([1.0, 1.0], [7.5, 4.9], color=COL["power"], lw=1.4, zorder=1)
a = FancyArrowPatch((1.0, 5.15), (1.0, 4.9), arrowstyle="->",
                    mutation_scale=12, color=COL["power"], lw=1.4)
ax.add_patch(a)
ax.text(1.15, 6.25, "24 V", fontsize=8, color=COL["power"], style="italic")

# 5 V rail from right top-strip buck: goes up+across+down, bypassing phone
# Path: (11.25, 7.5) -> (11.25, 5.3) -> (2.55, 5.3) -> down to consumers
ax.plot([11.25, 11.25], [7.5, 5.3], color=COL["power"], lw=1.2, zorder=1)
ax.plot([2.55, 11.25], [5.3, 5.3], color=COL["power"], lw=1.2, zorder=1)
for tx in [2.55, 6.5, 11.25]:
    a = FancyArrowPatch((tx, 5.3), (tx, 4.9), arrowstyle="->",
                        mutation_scale=12, color=COL["power"], lw=1.2)
    ax.add_patch(a)
hlabel(ax, 8.9, 5.5, "5 V / 2 A   (Pi + both RP2040s)", color=COL["power"])

# ============== LEGEND =====================================================
lx, ly, lw_, lh = 4.1, 0.6, 5.1, 0.95
ax.add_patch(FancyBboxPatch((lx, ly), lw_, lh, boxstyle="round,pad=0.02",
             linewidth=0.6, edgecolor=COL["muted"], facecolor=COL["bg_soft"]))
ax.text(lx + 0.15, ly + lh - 0.2, "Legend",
        fontsize=8.5, weight="bold", color=COL["fg"])
# three rows
items = [
    (COL["bus"],   "-",  "wired bus  (USB, UART, SPI)"),
    (COL["pi"],    "--", "wireless  (WiFi)"),
    (COL["power"], "-",  "power rail  (24 V, 5 V)"),
]
for i, (c, ls, txt) in enumerate(items):
    yy = ly + lh - 0.4 - i * 0.22
    ax.plot([lx + 1.05, lx + 1.5], [yy, yy], color=c, lw=1.5, ls=ls)
    ax.text(lx + 1.6, yy, txt, fontsize=7.8, va="center", color=COL["fg"])

fig.savefig(OUT / "system_block_diagram.png")
print(f"wrote {OUT / 'system_block_diagram.png'}")
