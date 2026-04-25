#!/usr/bin/env python3
"""
plot_freq_response.py  -  Plot RSSI vs frequency from a cc1200_rf_test frequency sweep report.

Usage:
    python plot_freq_response.py report_freq_response.json
    python plot_freq_response.py report_freq_response.json --out freq_response.png
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Plot RSSI vs frequency from sweep report")
    parser.add_argument("report", help="JSON report file from cc1200_rf_test.py")
    parser.add_argument("--out", metavar="FILE", help="Save figure to file instead of showing")
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    with open(args.report, encoding="utf-8") as f:
        report = json.load(f)

    sweep = report.get("frequency_sweep")
    if not sweep:
        print("No 'frequency_sweep' key found in report.", file=sys.stderr)
        sys.exit(1)

    freqs      = [e["freq_mhz"] for e in sweep]
    rssi_avg   = [e["overall"]["rssi_avg_dbm"] for e in sweep]
    rssi_min   = [e["overall"]["rssi_min_dbm"] for e in sweep]
    rssi_max   = [e["overall"]["rssi_max_dbm"] for e in sweep]
    passed     = [e["overall"]["passed"] for e in sweep]

    freqs_arr    = np.array(freqs, dtype=float)
    rssi_avg_arr = np.array([v if v is not None else float("nan") for v in rssi_avg])
    rssi_min_arr = np.array([v if v is not None else float("nan") for v in rssi_min])
    rssi_max_arr = np.array([v if v is not None else float("nan") for v in rssi_max])

    fig, ax = plt.subplots(figsize=(9, 5))

    # Min/max band
    ax.fill_between(freqs_arr, rssi_min_arr, rssi_max_arr,
                    alpha=0.20, color="steelblue", label="RSSI min–max range")

    # Average line
    ax.plot(freqs_arr, rssi_avg_arr, "o-", color="steelblue",
            linewidth=1.8, markersize=5, label="RSSI avg")

    # Mark failed frequencies in red
    for i, ok in enumerate(passed):
        if not ok and rssi_avg[i] is None:
            ax.axvline(freqs[i], color="red", linewidth=0.8, linestyle="--", alpha=0.5)

    # Design-center marker
    ax.axvline(435, color="orange", linewidth=1.2, linestyle="--", alpha=0.8, label="Design center (435 MHz)")

    ax.set_xlabel("Frequency (MHz)", fontsize=11)
    ax.set_ylabel("RSSI (dBm)", fontsize=11)
    ax.set_title("CC1200 Received Signal Strength vs Frequency\n"
                 f"({report.get('notes', '')})", fontsize=11)

    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.xaxis.set_minor_locator(ticker.MultipleLocator(0.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(5))
    ax.grid(True, which="major", linestyle="--", linewidth=0.5, alpha=0.6)
    ax.grid(True, which="minor", linestyle=":", linewidth=0.3, alpha=0.4)

    ax.legend(fontsize=9)
    fig.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=args.dpi)
        print(f"Saved to {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
