#!/usr/bin/env python3
"""
Rev 2 vs Rev 3 analysis for the Circle body-coupled touch receiver.

Consumes the CSVs written by vlog_capture.py and produces the figures and
numbers for the NRSM abstract.

    python3 analyze_vlog.py \\
        --rev2 "rev2_b*.csv" --rev3 "rev3_b*.csv" \\
        --rev2-quiet "rev2_quiet*.csv" --rev3-quiet "rev3_quiet*.csv" \\
        --outdir figs

Outputs into --outdir:

    fig1_timeseries.pdf   morphology comparison, one panel per revision
    fig2_roc.pdf          per-event ROC, both revisions
    fig3_stability.pdf    the four stability metrics, small multiples
    metrics.csv           every number, for the paper
    metrics.txt           the same, formatted for reading

Method notes that matter for the write-up
-----------------------------------------
* A "press episode" merges KEYDOWN/KEYUP pairs separated by less than
  --merge-gap. A release inside an episode is a SPURIOUS RELEASE - the game's
  real failure mode, and the metric that needs no ground truth at all.
* Pd matches each operator MARK to an episode onset within --tolerance.
* False alarms come from the quiet captures, reported per minute, because with
  a good detector the touch-block false-alarm count is often zero and a ROC
  built on it collapses into the corner.
* The ROC sweeps a fixed threshold over the RAW signal, which isolates the
  hardware change from the adaptive detector. The deployed detector's operating
  point is drawn on top as a marker.
* Edge times (10-90%) are reported per revision. If they are comparable, the
  Rev 3 output capacitor cannot have filtered away Rev 2's dips - this is the
  measurement that closes that objection.
"""

import argparse
import glob
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------- style ----
# Categorical slots 1 and 2 from the validated reference palette.
# node scripts/validate_palette.js "#2a78d6,#eb6834" --mode light -> ALL PASS
# (CVD dE 24.7 protan, normal-vision dE 33.6, contrast >= 3:1)
C_REV2 = "#2a78d6"
C_REV3 = "#eb6834"
INK = "#0b0b0b"
INK_SOFT = "#52514e"
GRID = "#d8d7d2"

# Linestyle is deliberate secondary encoding: conference proceedings get
# printed and photocopied in grayscale, where hue alone stops working.
LS = {"Rev 2": (0, (5, 2)), "Rev 3": "solid"}
COLOR = {"Rev 2": C_REV2, "Rev 3": C_REV3}

plt.rcParams.update({
    "figure.dpi": 150,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.titlesize": 9,
    "axes.titleweight": "bold",
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK,
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "axes.axisbelow": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "xtick.color": INK_SOFT,
    "ytick.color": INK_SOFT,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.frameon": False,
    "legend.fontsize": 7.5,
    "lines.linewidth": 1.4,
})


def despine(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ----------------------------------------------------------- data model ----

class Capture:
    """One CSV: signal samples, detector events, ground-truth marks."""

    def __init__(self, path: Path):
        self.path = path
        df = pd.read_csv(path)
        if "host_epoch_s" not in df.columns and "host_time_s" in df.columns:
            df = df.rename(columns={"host_time_s": "host_epoch_s"})
        for col in ("host_epoch_s", "device_ms", "raw_adc"):
            df[col] = pd.to_numeric(df[col], errors="coerce")

        sig = df[df["kind"] == "V"].dropna(subset=["device_ms", "raw_adc"])
        if len(sig) < 10:
            raise ValueError(f"{path.name}: only {len(sig)} signal samples")

        self.t = (sig["device_ms"].to_numpy() - sig["device_ms"].iloc[0]) / 1000.0
        self.v = sig["raw_adc"].to_numpy(dtype=float)

        # Events carry only a wall-clock timestamp. Map host -> device time with
        # a linear fit over the signal rows, so everything lands on one clock.
        # Centre the epoch first: raw Unix seconds are ~1.8e9 and would make the
        # fit numerically poor over a one-minute span.
        h = sig["host_epoch_s"].to_numpy()
        self.epoch0 = float(h[0])
        self.epoch_span = (self.epoch0, float(h[-1]))
        hc = h - self.epoch0
        if len(hc) >= 2 and np.ptp(hc) > 0:
            slope, intercept = np.polyfit(hc, self.t, 1)
        else:
            slope, intercept = 1.0, 0.0
        self._h2d = lambda x: slope * (np.asarray(x, dtype=float) - self.epoch0) + intercept

        marks = df[df["kind"] == "MARK"]
        self.marks = np.sort(self._h2d(marks["host_epoch_s"].to_numpy(dtype=float)))

        t0_ms = sig["device_ms"].iloc[0]
        kd = df[df["kind"] == "KEYDOWN"].dropna(subset=["device_ms"])
        ku = df[df["kind"] == "KEYUP"].dropna(subset=["device_ms"])
        self.keydown = np.sort((kd["device_ms"].to_numpy() - t0_ms) / 1000.0)
        self.keyup = np.sort((ku["device_ms"].to_numpy() - t0_ms) / 1000.0)

    def absorb_events(self, ev: pd.DataFrame) -> int:
        """Merge browser-logged HID keystrokes and marks that fall inside this
        capture's wall-clock span. Needed because battery-powered BLE captures
        carry no detector events - those only exist over RTT."""
        lo, hi = self.epoch_span
        sel = ev[(ev["host_epoch_s"] >= lo) & (ev["host_epoch_s"] <= hi)]
        if sel.empty:
            return 0
        for kind, attr in (("KEYDOWN", "keydown"), ("KEYUP", "keyup")):
            rows = sel[sel["kind"] == kind]
            if not rows.empty:
                t = self._h2d(rows["host_epoch_s"].to_numpy(dtype=float))
                setattr(self, attr, np.sort(np.concatenate([getattr(self, attr), t])))
        mk = sel[sel["kind"] == "MARK"]
        if not mk.empty:
            t = self._h2d(mk["host_epoch_s"].to_numpy(dtype=float))
            self.marks = np.sort(np.concatenate([self.marks, t]))
        return len(sel)

    @property
    def duration_s(self) -> float:
        return float(self.t[-1] - self.t[0]) if len(self.t) else 0.0


def load(patterns) -> list:
    out = []
    for pat in patterns:
        for p in sorted(glob.glob(pat)):
            try:
                out.append(Capture(Path(p)))
            except Exception as exc:  # noqa: BLE001
                print(f"  skipping {p}: {exc}", file=sys.stderr)
    return out


# -------------------------------------------------------------- episodes ---

def press_episodes(keydown, keyup, merge_gap: float):
    """Merge chattering KEYDOWN/KEYUP pairs into physical touch episodes.

    Returns (episodes, spurious_releases) where episodes is a list of
    (start_s, end_s). A release that gets merged away was spurious: the signal
    dipped far enough mid-touch to drop the key.
    """
    pairs = []
    for dn in keydown:
        later = keyup[keyup > dn]
        pairs.append((dn, later[0] if len(later) else math.inf))
    pairs.sort()

    episodes, spurious = [], 0
    for start, end in pairs:
        if episodes and start - episodes[-1][1] < merge_gap:
            if end > episodes[-1][1]:
                episodes[-1] = (episodes[-1][0], end)
            spurious += 1
        else:
            episodes.append((start, end))
    return episodes, spurious


def wilson(k: int, n: int, z: float = 1.96):
    """Wilson score interval - correct near 0 and 1, unlike normal approx."""
    if n == 0:
        return (float("nan"),) * 3
    p = k / n
    d = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0.0, centre - half), min(1.0, centre + half)


# --------------------------------------------------------------- metrics ---

def analyse(caps, quiet_caps, args):
    m = {}
    n_marks = n_hit = n_spurious = n_episodes = 0
    dip_counts, hold_frac, plateau_cv, edges = [], [], [], []
    hold_seconds = 0.0

    for c in caps:
        eps, spur = press_episodes(c.keydown, c.keyup, args.merge_gap)
        n_episodes += len(eps)
        n_spurious += spur
        n_marks += len(c.marks)

        starts = np.array([e[0] for e in eps]) if eps else np.array([])
        for mk in c.marks:
            if len(starts) and np.min(np.abs(starts - mk)) <= args.tolerance:
                n_hit += 1

        # Touch windows from the marks; baseline is everything outside them.
        in_touch = np.zeros_like(c.t, dtype=bool)
        for mk in c.marks:
            in_touch |= (c.t >= mk + args.pre) & (c.t <= mk + args.post)

        if in_touch.any() and (~in_touch).any():
            base = float(np.median(c.v[~in_touch]))
            plat = float(np.median(c.v[in_touch]))
            mid = base + args.dip_frac * (plat - base)

            for mk in c.marks:
                w = (c.t >= mk + args.pre) & (c.t <= mk + args.post)
                seg = c.v[w]
                if len(seg) < 5:
                    continue

                # Restrict to the HOLD region - first crossing above the
                # midpoint to the last one. Including the pre-touch idle and
                # the rising edge would count them as "dropouts" and wash the
                # metric out: both revisions would score identically.
                above = np.flatnonzero(seg >= mid)
                if len(above) < 3:
                    continue
                hold = seg[above[0]: above[-1] + 1]

                below = hold < mid
                dip_counts.append(int(np.sum(below)))
                hold_frac.append(float(np.mean(~below)))
                hold_seconds += len(hold) / args.fs
                if len(hold) > 2 and np.mean(hold) != 0:
                    plateau_cv.append(float(np.std(hold) / np.mean(hold)))
                e = edge_time(c.t[w], seg, base, plat)
                if e is not None:
                    edges.append(e)

    m["episodes"] = n_episodes
    m["marks"] = n_marks
    m["spurious_releases"] = n_spurious
    m["spurious_per_100"] = 100.0 * n_spurious / n_episodes if n_episodes else float("nan")
    pd_, lo, hi = wilson(n_hit, n_marks)
    m["Pd"], m["Pd_lo"], m["Pd_hi"] = pd_, lo, hi
    m["dropouts_per_s"] = (sum(dip_counts) / hold_seconds) if hold_seconds else float("nan")
    m["hold_integrity"] = float(np.mean(hold_frac)) if hold_frac else float("nan")
    m["plateau_cv"] = float(np.mean(plateau_cv)) if plateau_cv else float("nan")
    m["edge_10_90_ms"] = float(np.median(edges)) * 1000 if edges else float("nan")

    quiet_s = sum(c.duration_s for c in quiet_caps)
    quiet_fa = sum(len(press_episodes(c.keydown, c.keyup, args.merge_gap)[0])
                   for c in quiet_caps)
    m["quiet_minutes"] = quiet_s / 60.0
    m["false_alarms"] = quiet_fa
    m["fa_per_min"] = (quiet_fa / (quiet_s / 60.0)) if quiet_s > 0 else float("nan")
    return m


def edge_time(t, v, base, plat):
    """10-90% rise time of the first upward transition in a touch window."""
    span = plat - base
    if span <= 0:
        return None
    lo, hi = base + 0.1 * span, base + 0.9 * span
    i_lo = np.argmax(v >= lo) if (v >= lo).any() else None
    if i_lo is None:
        return None
    rest = v[i_lo:]
    if not (rest >= hi).any():
        return None
    return float(t[i_lo + int(np.argmax(rest >= hi))] - t[i_lo])


def roc(caps, quiet_caps, args, n_thresh=200):
    """Per-event ROC on the raw signal with a swept fixed threshold.

    A touch window counts as detected if any sample in it exceeds the
    threshold. Quiet captures are chopped into windows of the same length to
    supply the false-alarm opportunities.
    """
    windows_touch, windows_quiet = [], []
    for c in caps:
        for mk in c.marks:
            w = (c.t >= mk + args.pre) & (c.t <= mk + args.post)
            if w.sum() >= 3:
                windows_touch.append(c.v[w].max())

    win_len = args.post - args.pre
    for c in quiet_caps:
        n = int(c.duration_s // win_len)
        for i in range(n):
            w = (c.t >= i * win_len) & (c.t < (i + 1) * win_len)
            if w.sum() >= 3:
                windows_quiet.append(c.v[w].max())

    if not windows_touch or not windows_quiet:
        return None

    a, b = np.array(windows_touch), np.array(windows_quiet)
    lo, hi = min(a.min(), b.min()), max(a.max(), b.max())
    ths = np.linspace(lo, hi, n_thresh)
    pd_ = np.array([(a >= t).mean() for t in ths])
    pfa = np.array([(b >= t).mean() for t in ths])
    order = np.argsort(pfa)
    return pfa[order], pd_[order]


# --------------------------------------------------------------- figures ---

def fig_timeseries(caps2, caps3, args, out: Path):
    """Small multiples, not an overlay: the revisions sit at different absolute
    levels, so overlaying them would invite a level comparison the log-amp
    change does not support. Shared x, independent y, same window length."""
    fig, axes = plt.subplots(2, 1, figsize=(6.5, 4.0), sharex=True)

    for ax, caps, label in ((axes[0], caps2, "Rev 2"), (axes[1], caps3, "Rev 3")):
        despine(ax)
        if not caps:
            ax.text(0.5, 0.5, f"no {label} data", ha="center", va="center",
                    transform=ax.transAxes, color=INK_SOFT)
            continue
        c = caps[0]
        w = c.t <= args.window
        # SOLID in both panels, deliberately. A dashed trace reads as gaps in
        # the signal - the very artifact this figure exists to show. The panels
        # are separated and titled, so hue alone never has to carry identity.
        ax.plot(c.t[w], c.v[w], color=COLOR[label], linestyle="solid",
                linewidth=1.2, solid_capstyle="round", solid_joinstyle="round")
        for mk in c.marks[c.marks <= args.window]:
            ax.axvline(mk, color=INK_SOFT, linewidth=0.6, alpha=0.30, zorder=0)
        ax.set_ylabel("VLOG (counts)")
        ax.set_title(f"{label} — {'single-ended' if label == 'Rev 2' else 'differential'}",
                     loc="left", color=INK)
        ax.margins(y=0.12)

    axes[1].set_xlabel("Time (s)")
    handle = axes[1].plot([], [], color=INK_SOFT, linewidth=0.8, alpha=0.7,
                          label="ground-truth touch onset")[0]
    fig.legend(handles=[handle], loc="lower right",
               bbox_to_anchor=(1.0, -0.02), ncol=1)
    fig.suptitle("VLOG during paced touch / release", x=0.005, ha="left",
                 fontsize=10, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out)
    plt.close(fig)


def fig_roc(r2, r3, m2, m3, out: Path):
    fig, ax = plt.subplots(figsize=(3.4, 3.2))
    despine(ax)
    ax.plot([0, 1], [0, 1], color=GRID, linewidth=0.8, zorder=0)

    for r, m, label in ((r2, m2, "Rev 2"), (r3, m3, "Rev 3")):
        if r is None:
            continue
        pfa, pd_ = r
        ax.plot(pfa, pd_, color=COLOR[label], linestyle=LS[label], label=label)
        # Deployed detector operating point, ringed against the surface so it
        # stays legible where the two curves overlap.
        if not math.isnan(m["Pd"]):
            ax.plot([min(m["fa_per_min"] / 60.0, 1.0)], [m["Pd"]], marker="o",
                    markersize=6, color=COLOR[label],
                    markeredgecolor="white", markeredgewidth=1.5, zorder=5)

    ax.set_xlabel("Probability of false alarm")
    ax.set_ylabel("Probability of detection")
    ax.set_xlim(-0.02, 1.02)
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("Per-event detection", loc="left", color=INK)
    proxy = plt.Line2D([], [], marker="o", markersize=6, linestyle="none",
                       color=INK_SOFT, markeredgecolor="white",
                       markeredgewidth=1.5, label="deployed detector")
    handles, _ = ax.get_legend_handles_labels()
    ax.legend(handles=handles + [proxy], loc="lower right",
              bbox_to_anchor=(1.0, -0.02))
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def fig_stability(m2, m3, out: Path):
    """Four metrics with four different units, so four panels with four y-axes.
    One chart with a shared axis would be meaningless here."""
    specs = [
        ("spurious_per_100", "Spurious releases\nper 100 touches", "{:.1f}"),
        ("dropouts_per_s", "Dropout samples\nper second of hold", "{:.2f}"),
        ("hold_integrity", "Hold integrity\n(fraction above threshold)", "{:.3f}"),
        ("plateau_cv", "Plateau CV\n(lower is steadier)", "{:.4f}"),
    ]
    fig, axes = plt.subplots(1, 4, figsize=(7.0, 2.4))

    for ax, (key, title, fmt) in zip(axes, specs):
        despine(ax)
        ax.grid(axis="x", visible=False)
        vals = [m2.get(key, float("nan")), m3.get(key, float("nan"))]
        bars = ax.bar(["Rev 2", "Rev 3"], vals,
                      color=[C_REV2, C_REV3], width=0.6, zorder=2)
        for b in bars:
            b.set_linewidth(1.2)
            b.set_edgecolor("white")   # 2px surface gap between adjacent bars
        for b, v in zip(bars, vals):
            if not math.isnan(v):
                ax.annotate(fmt.format(v),
                            xy=(b.get_x() + b.get_width() / 2, v),
                            xytext=(0, 3), textcoords="offset points",
                            ha="center", fontsize=7, color=INK)
        ax.set_title(title, loc="left", fontsize=7.5, color=INK)
        ax.margins(y=0.22)

    fig.suptitle("Signal stability during sustained contact", x=0.005, ha="left",
                 fontsize=10, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out)
    plt.close(fig)


# ------------------------------------------------------------------ main ---

def report(m2, m3) -> str:
    rows = [
        ("Touch episodes", "episodes", "{:.0f}"),
        ("Ground-truth marks", "marks", "{:.0f}"),
        ("Spurious releases", "spurious_releases", "{:.0f}"),
        ("  per 100 touches", "spurious_per_100", "{:.1f}"),
        ("Pd", "Pd", "{:.3f}"),
        ("  95% CI low", "Pd_lo", "{:.3f}"),
        ("  95% CI high", "Pd_hi", "{:.3f}"),
        ("Dropouts / s of hold", "dropouts_per_s", "{:.3f}"),
        ("Hold integrity", "hold_integrity", "{:.4f}"),
        ("Plateau CV", "plateau_cv", "{:.5f}"),
        ("Edge 10-90% (ms)", "edge_10_90_ms", "{:.1f}"),
        ("Quiet minutes", "quiet_minutes", "{:.2f}"),
        ("False alarms", "false_alarms", "{:.0f}"),
        ("  per minute", "fa_per_min", "{:.2f}"),
    ]
    w = max(len(r[0]) for r in rows) + 2
    out = [f"{'':{w}}{'Rev 2':>12}{'Rev 3':>12}", "-" * (w + 24)]
    for label, key, fmt in rows:
        def cell(m):
            v = m.get(key, float("nan"))
            return "n/a" if (isinstance(v, float) and math.isnan(v)) else fmt.format(v)
        out.append(f"{label:{w}}{cell(m2):>12}{cell(m3):>12}")
    return "\n".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rev2", nargs="+", required=True)
    ap.add_argument("--rev3", nargs="+", required=True)
    ap.add_argument("--rev2-quiet", nargs="*", default=[])
    ap.add_argument("--rev3-quiet", nargs="*", default=[])
    ap.add_argument("--events", nargs="*", default=[],
                    help="key_logger.html CSV exports; merged into whichever "
                         "capture their wall-clock overlaps")
    ap.add_argument("--outdir", default="figs")
    ap.add_argument("--merge-gap", type=float, default=0.30,
                    help="s; releases closer than this are spurious chatter")
    ap.add_argument("--tolerance", type=float, default=1.0,
                    help="s; mark-to-detection matching window")
    ap.add_argument("--pre", type=float, default=-0.2, help="s before a mark")
    ap.add_argument("--post", type=float, default=1.5, help="s after a mark")
    ap.add_argument("--dip-frac", type=float, default=0.5,
                    help="dip threshold as a fraction of baseline->plateau")
    ap.add_argument("--fs", type=float, default=125.0, help="sample rate, Hz")
    ap.add_argument("--window", type=float, default=20.0,
                    help="s of trace to draw in figure 1")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    print("Loading...")
    c2, c3 = load(args.rev2), load(args.rev3)
    q2, q3 = load(args.rev2_quiet), load(args.rev3_quiet)
    print(f"  Rev 2: {len(c2)} touch, {len(q2)} quiet")
    print(f"  Rev 3: {len(c3)} touch, {len(q3)} quiet")

    if args.events:
        frames = []
        for pat in args.events:
            for f in sorted(glob.glob(pat)):
                d = pd.read_csv(f)
                if "host_epoch_s" not in d.columns and "host_time_s" in d.columns:
                    d = d.rename(columns={"host_time_s": "host_epoch_s"})
                d["host_epoch_s"] = pd.to_numeric(d["host_epoch_s"], errors="coerce")
                frames.append(d.dropna(subset=["host_epoch_s"]))
        if frames:
            ev = pd.concat(frames, ignore_index=True)
            total = 0
            for c in c2 + c3 + q2 + q3:
                total += c.absorb_events(ev)
            print(f"  merged {total} of {len(ev)} browser events by wall clock")
            if total == 0:
                print("  WARNING: no events overlapped any capture. Are the "
                      "browser and capture clocks from the same machine?")
    if not c2 or not c3:
        sys.exit("Need at least one touch capture per revision.")

    m2, m3 = analyse(c2, q2, args), analyse(c3, q3, args)
    r2, r3 = roc(c2, q2, args), roc(c3, q3, args)

    fig_timeseries(c2, c3, args, outdir / "fig1_timeseries.pdf")
    fig_roc(r2, r3, m2, m3, outdir / "fig2_roc.pdf")
    fig_stability(m2, m3, outdir / "fig3_stability.pdf")

    pd.DataFrame([dict(rev="Rev 2", **m2), dict(rev="Rev 3", **m3)]).to_csv(
        outdir / "metrics.csv", index=False)
    text = report(m2, m3)
    (outdir / "metrics.txt").write_text(text + "\n")

    print("\n" + text)
    print(f"\nFigures and metrics written to {outdir.resolve()}")
    if r2 is None or r3 is None:
        print("\nNOTE: no ROC - that needs quiet captures. Pass --rev2-quiet/--rev3-quiet.")


if __name__ == "__main__":
    main()
