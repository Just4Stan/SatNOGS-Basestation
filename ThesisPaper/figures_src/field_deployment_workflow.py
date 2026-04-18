"""Field deployment 5-step workflow."""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
from _style import OUT, COL


def step(ax, x, y, w, h, n, title, lines):
    # Card background
    p = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                       linewidth=1.0, edgecolor=COL["pi"], facecolor=COL["bg_pi"],
                       zorder=2)
    ax.add_patch(p)
    # Step circle (top-left, pushed into the header strip)
    cx = x + 0.55
    cy = y + h - 0.5
    c = Circle((cx, cy), 0.33, facecolor=COL["pi"], edgecolor="none", zorder=4)
    ax.add_patch(c)
    ax.text(cx, cy, str(n), ha="center", va="center", fontsize=13,
            color="white", weight="bold", zorder=5)
    # Title — start after the circle, left-aligned, pinned to header line
    ax.text(cx + 0.55, cy, title, ha="left", va="center",
            fontsize=10.5, weight="bold", color=COL["pi"], zorder=5)
    # Horizontal separator under header
    ax.plot([x + 0.2, x + w - 0.2], [y + h - 1.05, y + h - 1.05],
            color=COL["pi"], lw=0.8, alpha=0.4)
    # Body lines
    for i, line in enumerate(lines):
        ax.text(x + 0.25, y + h - 1.45 - i * 0.38, "•  " + line,
                fontsize=9, color=COL["fg"], va="top")


fig, ax = plt.subplots(figsize=(15, 6.2))
ax.set_xlim(0, 16)
ax.set_ylim(0, 6.4)
ax.set_aspect("equal")
ax.axis("off")

ax.text(8, 5.9, "Field deployment  —  5 steps from unpack to tracking",
        ha="center", fontsize=12, weight="bold", color=COL["pi"])
ax.text(8, 5.55, "wall-clock ≈ 4 minutes",
        ha="center", fontsize=9, color=COL["muted"], style="italic")

# 5 cards
W, H = 2.85, 4.0
Y = 0.85
xs = [0.2, 3.3, 6.4, 9.5, 12.6]

steps = [
    (1, "Place & power",
     ["Mount on tripod", "Point rotator North",
      "Connect 24 V PSU  (XT30)", "Pi boots Debian 13",
      "EL homes via ADXL345 IMU"]),
    (2, "Boot complete",
     ["rotctld on TCP :4533", "dashboard.py HTTPS :5000",
      "station.py waits for location", "NeoPixel  →  green  =  ready"]),
    (3, "Connect phone",
     ["Join WiFi hotspot", "SSID: SatNOGS-xxxx",
      "Open  https://<pi>:5000", "Accept self-signed cert"]),
    (4, "GPS + wizard",
     ["Tap  'Use My GPS'", "Browser Geolocation API",
      "Confirm callsign / mode", "Calibrate North  (AZ zero)"]),
    (5, "Auto-tracking",
     ["station.py polls TLE", "Rotator tracks  AZ / EL",
      "CC1200 Doppler @ 2 Hz", "SiDS  →  SatNOGS DB"]),
]
for x, (n, title, lines) in zip(xs, steps):
    step(ax, x, Y, W, H, n, title, lines)

# Connecting arrows between cards
for i in range(len(xs) - 1):
    x0 = xs[i] + W
    x1 = xs[i + 1]
    a = FancyArrowPatch((x0 + 0.03, Y + H / 2), (x1 - 0.03, Y + H / 2),
                        arrowstyle="->", mutation_scale=18,
                        linewidth=1.6, color=COL["pi"], zorder=1.5)
    ax.add_patch(a)

# Bottom timeline
ax.plot([0.2, 15.45], [0.3, 0.3], color=COL["muted"], lw=0.8)
ax.text(0.2, 0.5, "t = 0 s",   fontsize=8, color=COL["muted"])
ax.text(15.45, 0.5, "t ≈ 4 min", fontsize=8, color=COL["muted"], ha="right")

fig.savefig(OUT / "field_deployment_workflow.png")
print(f"wrote {OUT / 'field_deployment_workflow.png'}")
