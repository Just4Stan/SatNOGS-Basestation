#!/usr/bin/env python3
"""
Post-pass waterfall generator — produces a spectrogram PNG from raw IQ data.

Designed for Pi 3A+ (512MB RAM): processes IQ data in chunks to keep memory
under ~50MB. Uses lazy imports so scipy/matplotlib aren't loaded at station
startup.

Input format: interleaved uint8 I/Q (same as rtl_sdr CLI output).

Usage:
    # As a module (called by rtlsdr_backend.py after a pass)
    from waterfall import generate_waterfall
    generate_waterfall("iq.raw", "waterfall.png", sample_rate=240000)

    # Standalone CLI
    python3 waterfall.py iq.raw waterfall.png --rate 240000 --freq 433e6
"""

import os
import sys
import argparse


def generate_waterfall(iq_path: str, output_path: str,
                       sample_rate: int = 240000,
                       center_freq: float = 0.0,
                       chunk_seconds: float = 10.0,
                       nfft: int = 1024):
    """Generate a waterfall spectrogram PNG from a raw IQ file.

    Processes the file in time chunks to keep RAM usage under ~50MB on a
    Pi 3A+ (512MB total).

    Args:
        iq_path:       Path to raw interleaved uint8 IQ file.
        output_path:   Output PNG path.
        sample_rate:   Sample rate in Hz.
        center_freq:   Center frequency in Hz (for axis label).
        chunk_seconds: Process this many seconds at a time (RAM control).
        nfft:          FFT size for spectrogram.
    """
    import numpy as np

    file_size = os.path.getsize(iq_path)
    if file_size < nfft * 2:
        return  # not enough data

    # Samples per chunk (IQ pairs = 2 bytes each)
    chunk_samples = int(sample_rate * chunk_seconds)
    chunk_bytes = chunk_samples * 2

    # First pass: compute spectrogram slices
    all_slices = []
    time_offset = 0.0

    with open(iq_path, "rb") as f:
        while True:
            raw = f.read(chunk_bytes)
            if len(raw) < nfft * 2:
                break

            # Convert uint8 IQ to complex float
            iq_uint8 = np.frombuffer(raw, dtype=np.uint8)
            n_samples = len(iq_uint8) // 2
            i_samples = (iq_uint8[0::2].astype(np.float32) - 127.5) / 127.5
            q_samples = (iq_uint8[1::2].astype(np.float32) - 127.5) / 127.5
            iq = i_samples + 1j * q_samples

            # Compute spectrogram for this chunk
            from scipy.signal import spectrogram
            freqs, times, Sxx = spectrogram(
                iq, fs=sample_rate, nperseg=nfft,
                noverlap=nfft // 2, return_onesided=False,
                mode='magnitude',
            )

            # fftshift so DC is in the center
            Sxx = np.fft.fftshift(Sxx, axes=0)
            freqs = np.fft.fftshift(freqs)

            all_slices.append(Sxx)
            time_offset += len(iq) / sample_rate

            # Free memory
            del iq, i_samples, q_samples, iq_uint8

    if not all_slices:
        return

    # Concatenate all time slices
    waterfall = np.concatenate(all_slices, axis=1)
    del all_slices

    # Convert to dB
    waterfall_db = 20 * np.log10(waterfall + 1e-10)
    del waterfall

    # Plot
    import matplotlib
    matplotlib.use('Agg')  # headless
    import matplotlib.pyplot as plt

    total_time = file_size / (2 * sample_rate)
    freq_extent_khz = sample_rate / 2000.0  # half-bandwidth in kHz

    fig, ax = plt.subplots(figsize=(12, 6), dpi=100)
    fig.patch.set_facecolor('#0a0e1a')
    ax.set_facecolor('#0a0e1a')

    im = ax.imshow(
        waterfall_db,
        aspect='auto',
        origin='lower',
        cmap='inferno',
        extent=[0, total_time, -freq_extent_khz, freq_extent_khz],
        interpolation='nearest',
    )

    if center_freq > 0:
        ax.set_ylabel(f'Offset from {center_freq/1e6:.3f} MHz (kHz)',
                       color='white', fontsize=10)
    else:
        ax.set_ylabel('Frequency offset (kHz)', color='white', fontsize=10)
    ax.set_xlabel('Time (s)', color='white', fontsize=10)
    ax.tick_params(colors='white', labelsize=8)

    cbar = fig.colorbar(im, ax=ax, pad=0.02)
    cbar.set_label('Power (dB)', color='white', fontsize=9)
    cbar.ax.tick_params(colors='white', labelsize=8)

    for spine in ax.spines.values():
        spine.set_edgecolor('#333')

    fig.tight_layout()
    fig.savefig(output_path, dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)
    del waterfall_db


def main():
    parser = argparse.ArgumentParser(
        description="Generate waterfall spectrogram from raw IQ file")
    parser.add_argument("input", help="Raw IQ file (interleaved uint8)")
    parser.add_argument("output", help="Output PNG path")
    parser.add_argument("--rate", type=int, default=240000,
                        help="Sample rate in Hz (default: 240000)")
    parser.add_argument("--freq", type=float, default=0,
                        help="Center frequency in Hz (for label)")
    parser.add_argument("--nfft", type=int, default=1024,
                        help="FFT size (default: 1024)")
    parser.add_argument("--chunk", type=float, default=10.0,
                        help="Chunk size in seconds (default: 10)")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: {args.input} not found")
        sys.exit(1)

    print(f"Generating waterfall: {args.input} -> {args.output}")
    print(f"  Sample rate: {args.rate} Hz, FFT: {args.nfft}, Chunk: {args.chunk}s")
    generate_waterfall(
        args.input, args.output,
        sample_rate=args.rate,
        center_freq=args.freq,
        nfft=args.nfft,
        chunk_seconds=args.chunk,
    )
    print(f"Done: {args.output}")


if __name__ == "__main__":
    main()
