"""COBS frame byte layout — pre-COBS structure and post-COBS wire format."""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from _style import OUT, COL


def byte_cell(ax, x, y, w, h, top, bottom="", face="#ffffff", edge=COL["fg"],
              title_color=None):
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.01,rounding_size=0.04",
                       linewidth=1.0, edgecolor=edge, facecolor=face, zorder=2)
    ax.add_patch(p)
    ax.text(x + w / 2, y + h * 0.7, top, ha="center", va="center",
            fontsize=9, weight="bold",
            color=title_color or COL["fg"])
    if bottom:
        ax.text(x + w / 2, y + h * 0.28, bottom, ha="center", va="center",
                fontsize=7.5, color=COL["muted"], family="monospace")


def brace(ax, x0, x1, y, label, up=True, color=COL["muted"]):
    off = 0.15 if up else -0.15
    ax.annotate("", xy=(x0, y), xytext=(x0, y + off),
                arrowprops=dict(arrowstyle="-", color=color, lw=0.8))
    ax.annotate("", xy=(x1, y), xytext=(x1, y + off),
                arrowprops=dict(arrowstyle="-", color=color, lw=0.8))
    ax.plot([x0, x1], [y + off, y + off], color=color, lw=0.8)
    ax.text((x0 + x1) / 2, y + off + (0.25 if up else -0.35), label,
            ha="center", va="center" if up else "top",
            fontsize=8, color=color, style="italic")


fig, ax = plt.subplots(figsize=(13, 6.2))
ax.set_xlim(0, 14)
ax.set_ylim(0, 7)
ax.set_aspect("equal")
ax.axis("off")

# Title
ax.text(7, 6.6, "RF HAT protocol — COBS + CRC-16/CCITT-FALSE frame",
        ha="center", fontsize=11, weight="bold")
ax.text(7, 6.25, "UART /dev/serial0  ·  115 200 baud  ·  Pi ↔ RF HAT (RP2040)",
        ha="center", fontsize=8.5, color=COL["muted"])

# === Pre-COBS frame (logical) ================================================
y = 4.3
h = 0.9

# Layout: type(1) seq(1) len(1) body(N) crc_hi(1) crc_lo(1)
cells = [
    (0.5, 1.1, "type",  "1 B",  COL["bg_mcu"], "msg_type_t"),
    (1.6, 1.1, "seq",   "1 B",  COL["bg_soft"], "0–255 wraps"),
    (2.7, 1.1, "len",   "1 B",  COL["bg_soft"], "body length N"),
    (3.8, 4.0, "body",  "N B",  COL["bg_pi"],  "payload  (N ≤ ~250)"),
    (7.9, 1.1, "crc_hi", "1 B", COL["bg_rf"],  "CRC-16 MSB"),
    (9.0, 1.1, "crc_lo", "1 B", COL["bg_rf"],  "CRC-16 LSB"),
]
for (x, w, top, sub, face, _foot) in cells:
    byte_cell(ax, x, y, w, h, top, sub, face=face)

# Footnotes under cells
for (x, w, _t, _s, _f, foot) in cells:
    ax.text(x + w / 2, y - 0.2, foot, ha="center", va="top",
            fontsize=7.5, color=COL["muted"], style="italic")

# CRC span bracket
brace(ax, 0.5, 7.9, y + h, "CRC-16/CCITT-FALSE   poly 0x1021  ·  init 0xFFFF  ·  xorout 0x0000",
      up=True)

ax.text(0.5, y + h + 0.85, "Logical frame (pre-COBS)", fontsize=10, weight="bold",
        color=COL["fg"])

# Example byte values row
yex = y - 0.9
ax.text(0.5, yex + 0.3, "Example  (RX_READ, seq=1, zero-length body):",
        fontsize=8.5, color=COL["fg"], style="italic")
example = "0x20  0x01  0x00  —  0xEC  0x49"
ax.text(0.5, yex, example, fontsize=10, family="monospace", color=COL["fg"])

# === Arrow: COBS encode ======================================================
arrow = FancyArrowPatch((7, 3.4), (7, 2.7), arrowstyle="->",
                        mutation_scale=16, color=COL["fg"], lw=1.4)
ax.add_patch(arrow)
ax.text(7.2, 3.05, "cobs_encode()  +  append 0x00 delimiter",
        fontsize=8.5, color=COL["fg"], style="italic", va="center")

# === Post-COBS wire format ===================================================
yw = 1.4
hw = 0.9
ax.text(0.5, yw + hw + 0.35, "On-wire frame (post-COBS)", fontsize=10,
        weight="bold", color=COL["fg"])

wire = [
    (0.5, 1.1, "OHB",   "1 B",  "#fde68a", "overhead byte\n(offset to next 0x00)"),
    (1.6, 6.3, "COBS-encoded frame bytes",  "",  COL["bg_pwr"],
         "original frame with 0x00 bytes escaped by segment-length prefixes"),
    (7.9, 1.1, "0x00", "1 B",  "#f3f4f6", "frame delimiter\n(always last byte)"),
]
for (x, w, top, sub, face, foot) in wire:
    byte_cell(ax, x, yw, w, hw, top, sub, face=face)
    ax.text(x + w / 2, yw - 0.2, foot, ha="center", va="top",
            fontsize=7.5, color=COL["muted"], style="italic")

# Legend / notes bottom
ax.text(0.5, 0.25,
        "COBS: worst-case overhead = ⌈N/254⌉ bytes  ·  receiver reads until 0x00, "
        "then cobs_decode() + CRC-16 verify.",
        fontsize=8, color=COL["muted"], style="italic")

fig.savefig(OUT / "cobs_frame_diagram.png")
print(f"wrote {OUT / 'cobs_frame_diagram.png'}")
