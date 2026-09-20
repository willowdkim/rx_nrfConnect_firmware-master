#!/usr/bin/env python3
"""
Plot voltage (mV) vs time from a vlog_capture.py CSV.

Usage
-----
    python3 plot_voltage.py rev3_run1.csv
    python3 plot_voltage.py rev3_run1.csv --out fig.pdf
    python3 plot_voltage.py rev3_run1.csv --start 10 --end 40
    python3 plot_voltage.py rev3_run1.csv --raw          # ADC counts instead of mV
    python3 plot_voltage.py rev3_run1.csv --no-events    # hide touch shading / marks

Reads the standard capture schema:
    host_epoch_s, device_ms, kind, raw_adc, millivolts, seq
where `kind` is one of V | KEYDOWN | KEYUP | MARK | STATE.

Time on the x-axis is seconds since the first sample in the window, taken from
device_ms (the board's own clock), because that is the clock the 8 ms sampling
interval is defined on. host_epoch_s is used only to place events that come
from a separate file.
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Validated categorical palette (see dataviz references/palette.md).
BLUE = "#2a78d6"
ORANGE = "#eb6834"
INK = "#1a1a1a"
MUTED = "#8a8a8a"
GRID = "#e4e4e4"

STATE_NAMES = {0: "waiting for touch", 1: "calibrating", 2: "ready"}


def load(path):
    """Return (samples, events) from a capture CSV."""
    samples = []          # (device_s, raw, mv)
    keys = []             # (device_s, is_down)
    marks = []            # device_s
    states = []           # (device_s, state_code)

    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            kind = (row.get("kind") or "").strip()
            try:
                dev_s = float(row["device_ms"]) / 1000.0
            except (KeyError, TypeError, ValueError):
                continue

            if kind == "V":
                try:
                    samples.append(
                        (dev_s, float(row["raw_adc"]), float(row["millivolts"]))
                    )
                except (TypeError, ValueError):
                    pass
            elif kind == "KEYDOWN":
                keys.append((dev_s, True))
            elif kind == "KEYUP":
                keys.append((dev_s, False))
            elif kind == "MARK":
                marks.append(dev_s)
            elif kind == "STATE":
                try:
                    states.append((dev_s, int(float(row["raw_adc"]))))
                except (TypeError, ValueError):
                    pass

    return samples, keys, marks, states


def press_spans(keys, t_end):
    """Pair KEYDOWN/KEYUP into (start, stop) spans; close a dangling press."""
    spans, open_at = [], None
    for t, down in sorted(keys):
        if down and open_at is None:
            open_at = t
        elif not down and open_at is not None:
            spans.append((open_at, t))
            open_at = None
    if open_at is not None:
        spans.append((open_at, t_end))
    return spans


def main():
    ap = argparse.ArgumentParser(description="Plot voltage vs time from a capture CSV.")
    ap.add_argument("csv_path", help="CSV written by vlog_capture.py")
    ap.add_argument("--out", help="output file (default: <input>_voltage.pdf)")
    ap.add_argument("--start", type=float, help="window start, seconds into the run")
    ap.add_argument("--end", type=float, help="window end, seconds into the run")
    ap.add_argument("--raw", action="store_true", help="plot ADC counts, not mV")
    ap.add_argument("--no-events", action="store_true",
                    help="hide touch shading and marks")
    ap.add_argument("--title", help="override the plot title")
    ap.add_argument("--width", type=float, default=9.0, help="figure width, inches")
    ap.add_argument("--height", type=float, default=3.4, help="figure height, inches")
    args = ap.parse_args()

    samples, keys, marks, states = load(args.csv_path)
    if not samples:
        sys.exit(f"No V rows found in {args.csv_path} - nothing to plot.")

    # Clock origin: first sample in the file, so x starts at 0.
    t0 = samples[0][0]

    def rel(t):
        return t - t0

    t = [rel(s[0]) for s in samples]
    y = [s[1] if args.raw else s[2] for s in samples]

    lo = args.start if args.start is not None else t[0]
    hi = args.end if args.end is not None else t[-1]
    keep = [i for i, ti in enumerate(t) if lo <= ti <= hi]
    if not keep:
        sys.exit(f"No samples between {lo} s and {hi} s.")
    t = [t[i] for i in keep]
    y = [y[i] for i in keep]

    # Effective rate, as a sanity check on the 125 Hz target.
    span = t[-1] - t[0]
    rate = (len(t) - 1) / span if span > 0 else float("nan")

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    if not args.no_events:
        # Touch intervals behind the trace, so the line stays readable.
        for a, b in press_spans(keys, samples[-1][0]):
            a, b = rel(a), rel(b)
            if b >= lo and a <= hi:
                ax.axvspan(max(a, lo), min(b, hi), color=ORANGE, alpha=0.13, lw=0,
                           zorder=0)
        for m in marks:
            m = rel(m)
            if lo <= m <= hi:
                ax.axvline(m, color=ORANGE, lw=1.0, ls=(0, (3, 3)), alpha=0.75,
                           zorder=1)

    # Solid line only. A dashed trace reads as dropouts, which is the very
    # thing these captures exist to measure.
    ax.plot(t, y, color=BLUE, lw=1.0, solid_joinstyle="round", zorder=3)

    ax.set_xlabel("Time (s)", color=INK)
    ax.set_ylabel("ADC counts" if args.raw else "V$_{LOG}$ (mV)", color=INK)

    name = os.path.basename(args.csv_path)
    ax.set_title(args.title or f"{name}  -  {rate:.1f} Hz, {len(t)} samples",
                 color=INK, loc="left", fontsize=11, pad=10)

    ax.grid(True, color=GRID, lw=0.7, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.set_xlim(lo, hi)
    ax.set_ylim(400, 1400)

    if not args.no_events and (keys or marks):
        from matplotlib.patches import Patch
        from matplotlib.lines import Line2D
        handles = []
        if keys:
            handles.append(Patch(facecolor=ORANGE, alpha=0.13, label="touch held"))
        if marks:
            handles.append(Line2D([], [], color=ORANGE, lw=1.0, ls=(0, (3, 3)),
                                  label="mark"))
        # Placed above the axes: an in-plot legend collides with the trace,
        # and a box over the signal is exactly what we must not hide.
        ax.legend(handles=handles, frameon=False, fontsize=9, labelcolor=INK,
                  loc="lower right", bbox_to_anchor=(1.0, 1.005), ncol=2,
                  handlelength=1.6, borderpad=0, columnspacing=1.4)

    fig.tight_layout()

    out = args.out or os.path.splitext(args.csv_path)[0] + "_voltage.pdf"
    fig.savefig(out, dpi=200, facecolor="white")
    print(f"wrote {out}")
    print(f"  {len(t)} samples over {span:.2f} s  ({rate:.1f} Hz)")
    if not args.raw:
        print(f"  range {min(y):.0f} - {max(y):.0f} mV")
    for ts, code in states:
        ts = rel(ts)
        if lo <= ts <= hi:
            print(f"  state @ {ts:7.2f} s: {STATE_NAMES.get(code, code)}")


if __name__ == "__main__":
    main()
