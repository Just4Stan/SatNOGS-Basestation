"""Shared matplotlib style for **data plots** in the thesis.

Import at the top of any plotting script:
    from _plot_style import apply, COL, OUT, save

This sets rcParams to match the FIIW / cmbright look of the thesis body
text and gives every plot the same KU Leuven palette.
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
from pathlib import Path

# Output directory (same place the diagrams land)
OUT = Path(__file__).resolve().parent.parent / "images" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

# KU Leuven FIIW palette (fiiw.sty)
KUL_DARK  = "#00407A"
KUL_BLUE  = "#1D8DB0"
KUL_TINT  = "#D4EEF7"

COL = {
    "primary":    KUL_DARK,
    "accent":     KUL_BLUE,
    "c1":         KUL_DARK,    # first data series
    "c2":         KUL_BLUE,    # second data series
    "c3":         "#2F8F9D",   # RF / teal
    "c4":         "#B8860B",   # warm / power
    "c5":         "#8A3A1E",   # motor / warm-dark
    "c6":         "#6B21A8",   # purple fallback
    "muted":      "#6B7280",
    "grid":       "#E5E7EB",
    "text":       "#111827",
    "bg":         "#FFFFFF",
    "fill_soft":  KUL_TINT,
}

CYCLE = [COL["c1"], COL["c2"], COL["c3"], COL["c4"], COL["c5"], COL["c6"]]


def apply():
    """Install rcParams. Call once at import time in every plot script."""
    mpl.rcParams.update({
        "figure.dpi":        150,
        "savefig.dpi":       240,
        "savefig.bbox":      "tight",
        "savefig.facecolor": COL["bg"],
        "figure.facecolor":  COL["bg"],
        "axes.facecolor":    COL["bg"],

        # Fonts — sans-serif (approximates cmbright body text)
        "font.family":       "sans-serif",
        "font.sans-serif":   ["CMU Bright", "DejaVu Sans", "Arial"],
        "font.size":         10,
        "axes.titlesize":    11,
        "axes.titleweight":  "bold",
        "axes.titlecolor":   COL["primary"],
        "axes.labelsize":    10,
        "axes.labelcolor":   COL["text"],
        "xtick.labelsize":   9,
        "ytick.labelsize":   9,
        "xtick.color":       COL["text"],
        "ytick.color":       COL["text"],
        "legend.fontsize":   9,
        "legend.frameon":    True,
        "legend.framealpha": 0.95,
        "legend.edgecolor":  COL["grid"],

        # Axes / spines
        "axes.edgecolor":    COL["muted"],
        "axes.linewidth":    0.8,
        "axes.spines.top":   False,
        "axes.spines.right": False,

        # Grid
        "axes.grid":         True,
        "grid.color":        COL["grid"],
        "grid.linewidth":    0.6,
        "grid.alpha":        0.9,

        # Lines
        "lines.linewidth":   1.5,
        "lines.markersize":  4,

        # Colour cycle
        "axes.prop_cycle":   mpl.cycler(color=CYCLE),
    })


def save(fig, name):
    """Save to images/figures/<name>.{png,svg}.

    The SVG is the drag-edit source — open in Inkscape / Figma / Illustrator,
    reposition any text/arrow/box by hand, export PNG over the top.
    The PNG is what LaTeX reads; the SVG is the editable master.
    """
    stem = name[:-4] if name.endswith(".png") else name
    png  = OUT / f"{stem}.png"
    svg  = OUT / f"{stem}.svg"
    fig.savefig(png)
    fig.savefig(svg)  # same rcParams, vector output
    print(f"wrote {png}")
    print(f"wrote {svg}")
    return png


# Convenience: apply automatically on import so scripts can just do
#   from _plot_style import COL, OUT, save
apply()
