"""Shared matplotlib style for thesis figures.

Matches the KU Leuven FIIW thesis template (fiiw.sty):
  kulblueDark = #00407A   (primary dark — emphasis, headings)
  kulblue     = #1D8DB0   (primary accent)
Body font is cmbright (sans-serif Computer Modern).  We approximate with
DejaVu Sans, matplotlib's default sans, which carries similar weight.
"""
import matplotlib as mpl
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "images" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

mpl.rcParams.update({
    "figure.dpi": 150,
    "savefig.dpi": 240,                # thesis-quality raster
    "savefig.bbox": "tight",
    "savefig.facecolor": "white",
    "font.family": "sans-serif",
    "font.sans-serif": ["CMU Bright", "DejaVu Sans", "Arial"],
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.labelsize": 10,
    "axes.linewidth": 0.8,
    "lines.linewidth": 1.2,
    "patch.linewidth": 0.8,
})

# --- KU Leuven FIIW palette (sourced from fiiw.sty) ------------------------
KUL_DARK  = "#00407A"   # kulblueDark
KUL_BLUE  = "#1D8DB0"   # kulblue (primary accent)
KUL_LIGHT = "#CFE0EC"   # derived tint of kulblueDark
KUL_TINT  = "#D4EEF7"   # derived tint of kulblue

COL = {
    # Primary / subsystem roles (all drawn from KUL family + distinguishable accents)
    "pi":      KUL_DARK,     # Raspberry Pi / software stack
    "mcu":     KUL_BLUE,     # RP2040 firmware
    "rf":      "#2F8F9D",    # RF subsystem (teal companion to kulblue)
    "power":   "#B8860B",    # power rails (warm gold, KUL secondary)
    "motor":   "#8A3A1E",    # motor drivers / actuators
    "bus":     "#374151",    # wired buses
    # Backgrounds (shaded variants)
    "bg_pi":   KUL_LIGHT,
    "bg_mcu":  KUL_TINT,
    "bg_rf":   "#CCE5E8",
    "bg_pwr":  "#FBF1D4",
    "bg_mot":  "#F2DACC",
    "bg_soft": "#F3F4F6",
    # Text / muted
    "fg":      "#111827",
    "muted":   "#6B7280",
}
