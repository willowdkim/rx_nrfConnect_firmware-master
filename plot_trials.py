#!/usr/bin/env python3
"""
Overlay the repeat trials of a scenario on one voltage-vs-time plot.

Each trial is a separate capture on a separate board uptime, so every trial is
re-zeroed to its own first sample. The x-axis is "seconds into the trial",
which is the only sense in which three independent runs share a time base.

Usage
-----
    # one scenario, files given explicitly
    python3 plot_trials.py scenario3Rev3Try*.csv

    # every scenario in the folder, one figure each
    python3 plot_trials.py --all

    # a window, and a custom output name
    python3 plot_trials.py scenario1Rev3Test*.csv --start 0 --end 30 --out s1.pdf

Trials are drawn in the order given (shell glob order = Try, Try2, Try3).
"""

import argparse
import csv
import glob
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Validated categorical palette, assigned in fixed order - never cycled.
# Worst adjacent CVD separation dE 24.7, normal-vision 32.5 - validated, not eyeballed.
SERIES = ["#2a78d6", "#eb6834", "#6b4bbd"]
INK = "#1a1a1a"
MUTED = "#8a8a8a"
GRID = "#e4e4e4"

NOMINAL_HZ = 125.0


def load(path):
    """Return (times_s, mv, rate_hz) for the V rows of one capture."""
    t, y = [], []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if (row.get("kind") or "").strip() != "V":
                continue
            try:
                t.append(float(row["device_ms"]) / 1000.0)
                y.append(float(row["millivolts"]))
            except (KeyError, TypeError, ValueError):
                pass
    if not t:
        return [], [], float("nan")
    t0 = t[0]
    t = [x - t0 for x in t]
    span = t[-1] - t[0]
    rate = (len(t) - 1) / span if span > 0 else float("nan")
    return t, y, rate


def scenario_of(path):
    """'scenario3Rev3Try2.csv' -> 'scenario3Rev3'."""
    stem = os.path.splitext(os.path.basename(path))[0]
    m = re.match(r"(.*?(?:Rev\d+))(?:Test|Try)\d*$", stem)
    return m.group(1) if m else stem


def trial_label(path, i):
    stem = os.path.splitext(os.path.basename(path))[0]
    m = re.search(r"((?:Test|Try)\d*)$", stem)
    return m.group(1) if m else f"trial {i + 1}"


def plot_group(paths, args, out=None):
    trials = []
    for p in paths:
        t, y, rate = load(p)
        if not t:
            print(f"  ! {os.path.basename(p)}: no V rows, skipped")
            continue
        trials.append((p, t, y, rate))
    if not trials:
        return False

    name = scenario_of(trials[0][0])
    lo = args.start if args.start is not None else 0.0
    hi = args.end if args.end is not None else max(t[-1] for _, t, _, _ in trials)

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    for i, (p, t, y, rate) in enumerate(trials):
        keep = [j for j, tj in enumerate(t) if lo <= tj <= hi]
        # Solid lines only: a dashed trace reads as a dropout, which is the
        # very thing these captures exist to measure.
        ax.plot([t[j] for j in keep], [y[j] for j in keep],
                color=SERIES[i % len(SERIES)], lw=0.9, alpha=0.85,
                solid_joinstyle="round",
                label=f"{trial_label(p, i)}  ({rate:.1f} Hz)", zorder=3 + i)

    ax.set_xlabel("Time into trial (s)", color=INK)
    ax.set_ylabel("V$_{LOG}$ (mV)", color=INK)
    ax.set_title(f"{name}  -  {len(trials)} trials overlaid",
                 color=INK, loc="left", fontsize=11, pad=10)
    ax.grid(True, color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.set_xlim(lo, hi)

    # Above the axes - an in-plot legend would cover the signal.
    ax.legend(frameon=False, fontsize=9, labelcolor=INK, ncol=len(trials),
              loc="lower right", bbox_to_anchor=(1.0, 1.005),
              handlelength=1.6, borderpad=0, columnspacing=1.4)

    fig.tight_layout()
    out = out or args.out or f"{name}_trials.pdf"
    fig.savefig(out, dpi=200, facecolor="white")
    plt.close(fig)

    print(f"wrote {out}")
    for p, t, y, rate in trials:
        flag = ""
        if rate == rate and rate < NOMINAL_HZ * 0.97:
            flag = f"  <- {100 * (1 - rate / NOMINAL_HZ):.0f}% under {NOMINAL_HZ:.0f} Hz"
        print(f"  {os.path.basename(p):30s} {len(t):6d} samples  "
              f"{t[-1]:6.1f} s  {rate:6.1f} Hz  "
              f"{min(y):4.0f}-{max(y):4.0f} mV{flag}")
    return True


def main():
    ap = argparse.ArgumentParser(
        description="Overlay repeat trials of a scenario, voltage vs time.")
    ap.add_argument("csvs", nargs="*", help="capture CSVs for ONE scenario")
    ap.add_argument("--all", action="store_true",
                    help="group every scenario*.csv here and plot each")
    ap.add_argument("--out", help="output file (single-scenario runs only)")
    ap.add_argument("--start", type=float, help="window start, s into trial")
    ap.add_argument("--end", type=float, help="window end, s into trial")
    ap.add_argument("--width", type=float, default=9.0)
    ap.add_argument("--height", type=float, default=3.4)
    args = ap.parse_args()

    if args.all:
        groups = {}
        for p in sorted(glob.glob("scenario*.csv")):
            groups.setdefault(scenario_of(p), []).append(p)
        if not groups:
            sys.exit("No scenario*.csv files in this folder.")
        if args.out:
            sys.exit("--out makes no sense with --all (one file per scenario).")
        for name in sorted(groups):
            plot_group(groups[name], args)
        return

    if not args.csvs:
        sys.exit("Give some CSVs, or use --all.")

    names = {scenario_of(p) for p in args.csvs}
    if len(names) > 1:
        sys.exit("Those files span more than one scenario: "
                 + ", ".join(sorted(names)) + "\nUse --all, or one scenario at a time.")
    plot_group(args.csvs, args)


if __name__ == "__main__":
    main()
