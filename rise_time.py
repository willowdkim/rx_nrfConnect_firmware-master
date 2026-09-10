#!/usr/bin/env python3
"""
Measure 10-90% rise (and fall) times on VLOG edges from capture CSVs.

Why this metric
---------------
Three things changed between Rev 2 and Rev 3: the single-ended -> differential
front end, the AD8309 -> AD8310 log amp, and the output capacitor 0.01 -> 0.1 uF.
The capacitor is the live alternative explanation for any difference in dropout
behaviour. Edge speed is what separates them: a 10x larger output cap makes
edges SLOWER. So if Rev 3's edges are as fast or faster than Rev 2's, the cap
is not what is carrying the result.

Method
------
Per edge, levels are taken locally, not globally: the low level is the median of
a settled window before the edge, the high level the median of a settled window
after it. Rise time is the interval between the last crossing of
low + 10%*(high-low) and the first crossing of low + 90%*(high-low).

Edges are found by hysteresis on the 50% level, so noise around a threshold
cannot manufacture extra edges.

Usage
-----
    python3 rise_time.py scenario3Rev3Try*.csv
    python3 rise_time.py rev2.csv rev3.csv --compare       # A/B two groups
    python3 rise_time.py *.csv --start 20 --end 80         # window, s into trial
    python3 rise_time.py *.csv --falling                   # fall times instead
    python3 rise_time.py *.csv --csv edges.csv             # per-edge dump
"""

import argparse
import csv
import os
import statistics
import sys

NOMINAL_HZ = 125.0


# ---------------------------------------------------------------- loading

def load(path):
    """Return (t_seconds_from_trial_start, mv) for the V rows."""
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
        return [], []
    t0 = t[0]
    return [x - t0 for x in t], y


def sample_period(t):
    if len(t) < 2:
        return float("nan")
    return statistics.median(t[i + 1] - t[i] for i in range(len(t) - 1))


# ---------------------------------------------------------------- edges

def find_edges(y, lo, hi, rising=True):
    """Indices where the signal crosses the midpoint, with hysteresis.

    Returns the index of the first sample past the midpoint for each edge.
    """
    mid = (lo + hi) / 2.0
    band = 0.15 * (hi - lo)          # hysteresis: 15% of full swing
    hi_gate, lo_gate = mid + band, mid - band

    edges = []
    state = "low" if y[0] < mid else "high"
    for i, v in enumerate(y):
        if state == "low" and v > hi_gate:
            state = "high"
            if rising:
                edges.append(i)
        elif state == "high" and v < lo_gate:
            state = "low"
            if not rising:
                edges.append(i)
    return edges


def edge_time(t, y, idx, rising, settle_s, lo_global, hi_global):
    """10-90% transition time for the edge whose crossing is at index idx.

    Returns (duration_s, low_mv, high_mv) or None if the edge is not clean.
    """
    dt = sample_period(t)
    if dt != dt or dt <= 0:
        return None
    n_settle = max(3, int(settle_s / dt))

    # Local levels from settled windows either side of the crossing, held off
    # by a full settle window. The guard has to clear the transition itself:
    # if the post-edge window opens too early the signal is still arriving,
    # the measured high level comes out low, and every rise time is
    # under-reported. Raise --settle for a slow signal.
    guard = n_settle
    pre = y[max(0, idx - guard - n_settle): max(0, idx - guard)]
    post = y[idx + guard: idx + guard + n_settle]
    if len(pre) < 3 or len(post) < 3:
        return None

    a, b = statistics.median(pre), statistics.median(post)
    low, high = (a, b) if rising else (b, a)
    swing = high - low

    # Reject edges that are not a real transition of this signal.
    if swing < 0.4 * (hi_global - lo_global):
        return None

    p10 = low + 0.10 * swing
    p90 = low + 0.90 * swing

    def cross(i0, i1, level):
        """Linearly interpolated time at which the segment i0->i1 hits level.

        Without this the result snaps to sample boundaries and each end is
        biased outward by up to one period - about 9 ms here, which is a
        large fraction of the edges being measured.
        """
        y0, y1 = y[i0], y[i1]
        if y1 == y0:
            return t[i1]
        f = (level - y0) / (y1 - y0)
        f = min(1.0, max(0.0, f))
        return t[i0] + f * (t[i1] - t[i0])

    if rising:
        # Walk back from the crossing to the last sample below 10%,
        # then forward to the first at or above 90%.
        i = idx
        while i > 0 and y[i] > p10:
            i -= 1
        j = idx
        while j < len(y) - 1 and y[j] < p90:
            j += 1
        if y[i] > p10 or y[j] < p90:
            return None
    else:
        i = idx
        while i > 0 and y[i] < p90:
            i -= 1
        j = idx
        while j < len(y) - 1 and y[j] > p10:
            j += 1
        if y[i] < p90 or y[j] > p10:
            return None

    t_start = cross(i, min(i + 1, len(y) - 1), p10 if rising else p90)
    t_stop = cross(max(j - 1, 0), j, p90 if rising else p10)
    dur = t_stop - t_start
    if dur <= 0:
        return None
    return dur, low, high


def measure(path, args):
    """Return (edges, dt, note) for one file. edges: list of dicts."""
    t, y = load(path)
    if not t:
        return [], float("nan"), "no V rows"

    lo_w = args.start if args.start is not None else t[0]
    hi_w = args.end if args.end is not None else t[-1]
    keep = [i for i, ti in enumerate(t) if lo_w <= ti <= hi_w]
    if len(keep) < 20:
        return [], float("nan"), "too few samples in window"
    t = [t[i] for i in keep]
    y = [y[i] for i in keep]

    dt = sample_period(t)

    # Global levels set the midpoint and the "is this a real edge" test.
    ys = sorted(y)
    lo_g = statistics.median(ys[: max(1, len(ys) // 10)])
    hi_g = statistics.median(ys[-max(1, len(ys) // 10):])
    if hi_g - lo_g < args.min_swing:
        return [], dt, f"swing only {hi_g - lo_g:.0f} mV (< --min-swing)"

    rising = not args.falling
    out = []
    for idx in find_edges(y, lo_g, hi_g, rising=rising):
        r = edge_time(t, y, idx, rising, args.settle, lo_g, hi_g)
        if r is None:
            continue
        dur, low, high = r
        out.append({
            "file": os.path.basename(path),
            "t_s": round(t[idx], 3),
            "ms": dur * 1000.0,
            "low_mv": round(low, 1),
            "high_mv": round(high, 1),
            "swing_mv": round(high - low, 1),
            "samples": max(1, round(dur / dt)) if dt == dt else None,
        })
    return out, dt, ""


# ---------------------------------------------------------------- reporting

def summarize(edges):
    v = sorted(e["ms"] for e in edges)
    n = len(v)
    if n == 0:
        return None
    mean = statistics.fmean(v)
    return {
        "n": n,
        "mean": mean,
        "median": statistics.median(v),
        "sd": statistics.stdev(v) if n > 1 else 0.0,
        "min": v[0],
        "max": v[-1],
        "sem": (statistics.stdev(v) / (n ** 0.5)) if n > 1 else 0.0,
    }


MIN_EDGES = 5          # below this, report the spread but not an interval


def t_crit(df):
    """Two-sided 95% t critical value. Falls back to a table without scipy.

    The normal 1.96 is badly wrong for small n - at df=1 the true value is
    12.7, so using 1.96 there understates the interval by ~6x and can make
    pure noise look like a resolved difference.
    """
    if df <= 0:
        return float("inf")
    try:
        from scipy import stats
        return float(stats.t.ppf(0.975, df))
    except Exception:
        table = {1: 12.71, 2: 4.30, 3: 3.18, 4: 2.78, 5: 2.57, 6: 2.45,
                 7: 2.36, 8: 2.31, 9: 2.26, 10: 2.23, 12: 2.18, 15: 2.13,
                 20: 2.09, 30: 2.04, 60: 2.00}
        for k in sorted(table):
            if df <= k:
                return table[k]
        return 1.96


def print_summary(label, s, dt):
    if s is None:
        print(f"{label}: no clean edges found")
        return
    ci = t_crit(s["n"] - 1) * s["sem"]
    print(f"{label}")
    warn = "   ** too few edges to support an interval **" if s["n"] < MIN_EDGES else ""
    print(f"  n = {s['n']} edges{warn}")
    if s["n"] < MIN_EDGES:
        print(f"  mean   {s['mean']:7.1f} ms   (not summarisable at n={s['n']})")
    else:
        print(f"  mean   {s['mean']:7.1f} ms   (95% CI +/- {ci:.1f})")
    print(f"  median {s['median']:7.1f} ms")
    print(f"  sd     {s['sd']:7.1f} ms      range {s['min']:.1f} - {s['max']:.1f} ms")
    if dt == dt:
        floor = dt * 1000.0
        print(f"  sampling period {floor:.1f} ms  ->  resolution limit")
        if s["median"] < 3 * floor:
            print(f"  ** median is under 3 sampling periods: this measurement is")
            print(f"     quantization-limited, not a measurement of the circuit. **")


def main():
    ap = argparse.ArgumentParser(
        description="10-90% rise/fall time from VLOG capture CSVs.")
    ap.add_argument("csvs", nargs="+", help="capture CSV files")
    ap.add_argument("--compare", action="store_true",
                    help="split files into two groups by Rev2/Rev3 in the "
                         "filename and report the difference")
    ap.add_argument("--falling", action="store_true",
                    help="measure 90-10%% fall times instead of rise times")
    ap.add_argument("--start", type=float, help="window start, s into trial")
    ap.add_argument("--end", type=float, help="window end, s into trial")
    ap.add_argument("--settle", type=float, default=0.25,
                    help="settled window each side of an edge, s (default 0.25)")
    ap.add_argument("--min-swing", type=float, default=100.0,
                    help="ignore files whose full swing is under this, mV")
    ap.add_argument("--csv", help="write every measured edge to this CSV")
    ap.add_argument("--per-file", action="store_true",
                    help="also print a summary for each file")
    args = ap.parse_args()

    kind = "fall (90-10%)" if args.falling else "rise (10-90%)"
    print(f"VLOG {kind} time\n")

    all_edges, dts, groups = [], [], {}
    for p in args.csvs:
        edges, dt, note = measure(p, args)
        if dt == dt:
            dts.append(dt)
        base = os.path.basename(p)
        if note:
            print(f"  ! {base}: {note}")
            continue
        if not edges:
            print(f"  ! {base}: no clean edges")
            continue
        all_edges += edges
        key = ("Rev2" if "rev2" in base.lower() else
               "Rev3" if "rev3" in base.lower() else "unlabelled")
        groups.setdefault(key, []).extend(edges)
        if args.per_file:
            s = summarize(edges)
            print(f"  {base:32s} n={s['n']:3d}  mean {s['mean']:6.1f} ms  "
                  f"median {s['median']:6.1f} ms")

    if not all_edges:
        sys.exit("\nNo clean edges found in any file.")

    dt = statistics.median(dts) if dts else float("nan")
    print()

    if args.compare:
        have = [k for k in ("Rev2", "Rev3") if k in groups]
        if len(have) < 2:
            print("--compare needs files from both revisions "
                  "(matched on 'rev2'/'rev3' in the filename).")
            print(f"Found: {', '.join(groups) or 'nothing'}\n")
            print_summary("ALL FILES", summarize(all_edges), dt)
        else:
            a, b = summarize(groups["Rev2"]), summarize(groups["Rev3"])
            print_summary("Rev 2", a, dt)
            print()
            print_summary("Rev 3", b, dt)
            diff = b["mean"] - a["mean"]
            se = (a["sem"] ** 2 + b["sem"] ** 2) ** 0.5
            print()

            if min(a["n"], b["n"]) < MIN_EDGES:
                small = "Rev 2" if a["n"] < b["n"] else "Rev 3"
                print(f"difference (Rev3 - Rev2): {diff:+.1f} ms  -  NOT INTERPRETABLE")
                print(f"  {small} has only {min(a['n'], b['n'])} edges. That is a")
                print(f"  count, not a sample. Find out why so few edges were")
                print(f"  measured before reading anything into this number.")
                return

            # Welch-Satterthwaite: the two groups have their own variance and
            # their own n, so the pooled df is not n1+n2-2.
            df = (se ** 4) / ((a["sem"] ** 4) / (a["n"] - 1)
                              + (b["sem"] ** 4) / (b["n"] - 1))
            tc = t_crit(df)
            lo_ci, hi_ci = diff - tc * se, diff + tc * se
            print(f"difference (Rev3 - Rev2): {diff:+.1f} ms "
                  f"(95% CI {lo_ci:+.1f} to {hi_ci:+.1f}, Welch df={df:.1f})")

            if lo_ci <= 0 <= hi_ci:
                print("  The interval spans zero: no resolved difference in edge speed.")
            elif diff < 0:
                print("  Rev 3 edges are FASTER. The 0.1 uF output cap would predict")
                print("  slower edges, so the cap does not explain this - it argues")
                print("  against the cap being what drives the revisions apart.")
            else:
                print("  Rev 3 edges are SLOWER. This is the direction the larger")
                print("  0.1 uF output cap predicts, so the cap remains a live")
                print("  explanation and this does NOT close that confound.")
            print("  Note: hand-paced edges are ~100x slower than either log amp,")
            print("  so this mostly measures approach speed, not the front end.")
    else:
        print_summary("ALL EDGES", summarize(all_edges), dt)

    if args.csv:
        with open(args.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(all_edges[0].keys()))
            w.writeheader()
            w.writerows(all_edges)
        print(f"\nwrote {args.csv}  ({len(all_edges)} edges)")


if __name__ == "__main__":
    main()
