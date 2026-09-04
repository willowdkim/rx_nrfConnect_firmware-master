# Experimental Protocol — Rev 2 vs Rev 3 Receiver Front End

**Circle body-coupled touch sensing.** Single-ended (Rev 2) vs differential
(Rev 3) RF front end.

Target venue: **NRSM 2027**, Boulder, 5–7 January 2027.
Abstract deadline: **Friday 11 September 2026** (will not be extended).
Format: one-page abstract (≥250 words) or two-page summary.

Draft v1 — revise after the pilot session.

---

## 1. Objective and hypothesis

**Objective.** Quantify whether the differential receiver front end introduced
in Rev 3 improves the *stability* of the received signal during sustained
contact, relative to the single-ended Rev 2 design.

**Primary hypothesis.** Rev 3 exhibits significantly fewer transient dropouts
during a held touch than Rev 2, and correspondingly fewer spurious key releases
in the deployed game input path.

**Explicitly not claimed.** Higher absolute signal level. Rev 2 and Rev 3 use
different logarithmic amplifiers with different slope and intercept, so a raw
voltage ratio between boards is not a signal-strength ratio (§4.1). Framing the
result as "6× higher voltage" invites an objection that has nothing to do with
the contribution.

---

## 2. Apparatus

### 2.1 Hardware

| | Rev 2 | Rev 3 |
|---|---|---|
| Front end | single-ended | differential |
| RF input | SMA + coax | FPC, IN+/IN− pair |
| Filter | single-ended LC ladder | balanced LC ladder, same resonances |
| Log amp | AD8309ARU (~20 mV/dB, 100 dB) | AD8310ARMZ (~24 mV/dB, 95 dB) |
| VLOG output cap | 0.01 µF | ~0.1 µF |
| MCU | MDBT50Q-1MV2 (nRF52840) | same |

Both boards: VLOG on **P0.03 / AIN1**; LED_A P1.11, LED_B P1.13, LED_C P1.15;
battery powered via MCP73831 charger and MAX1595 regulator.

### 2.2 Firmware

Common build, flashed to both boards. Relevant configuration:

- SAADC: **12-bit**, gain 1/6, internal 0.6 V reference → 0–3600 mV full scale
- Conversion: `mv = raw × 3600 / 4096` (divisor 2^N per the nRF52840 spec)
- **64× oversampling**, one-shot offset calibration at boot
- Sample period **8 ms** (125 Hz)
- Detector constants scaled for 8 ms; spike detection over a fixed 32 ms window
- Streams over BLE (Nordic UART Service), 6 samples per 18-byte packet, each
  carrying a device-side timestamp and a packet sequence number

### 2.3 Software

| Tool | Role |
|---|---|
| `vlog_capture.py` | BLE acquisition → CSV + `.meta.json` provenance sidecar |
| `key_logger.html` | HID keystroke capture and ground-truth marks in the browser |
| `analyze_vlog.py` | metrics, figures, ROC |
| nRF Connect app | bring-up and diagnosis only, **not** acquisition |

---

## 3. Variables

**Independent variable.** Receiver revision (Rev 2 / Rev 3). Within-subject,
interleaved.

**Primary dependent variables.**

1. Spurious releases per 100 touches
2. Dropout samples per second of hold
3. Hold integrity (fraction of hold above detection threshold)
4. Plateau coefficient of variation

**Secondary.** Pd, false alarms per minute, 10–90 % edge time.

**Controlled.** Participant, limb, electrode position (marked on skin),
transmitter, geometry and distance, posture, touch pacing (metronome 0.5 Hz),
firmware build, sample rate, room.

**Uncontrolled but recorded.** Room temperature, time of day, block order,
session notes.

---

## 4. Known confounds and how they are handled

### 4.1 Different log amplifier

The AD8309 was discontinued; the AD8310 is its replacement, not a specification
change. Slope and intercept shift the *level*, not the presence of transient
dropouts, so this does not threaten a stability claim.

It arguably strengthens it: the **AD8310 is the faster part**, so it should
resolve more transient structure, not less. Rev 3 showing fewer dips despite a
faster detector is evidence for the front end, not an artifact of it. State this
explicitly.

*Mitigation:* report stability, not level. If any cross-board level comparison
is made, first perform the calibration sweep in §8.

### 4.2 Different VLOG output capacitor

0.01 µF → ~0.1 µF, as part of the differential redesign. This is the one
competing explanation that must be closed, because a larger output capacitor
smooths transients — precisely the phenomenon under claim.

*Mitigation:* compare **10–90 % edge times** of touch transitions on both
boards (computed automatically by `analyze_vlog.py`). If Rev 3's edges are as
fast as Rev 2's, its output filter cannot be smoothing anything on the timescale
of the dips. Report both numbers in one sentence.

### 4.3 Measurement-induced grounding (critical)

A J-Link debug probe ties board ground to the host and therefore to mains earth.
In a body-coupled system the body-to-earth return path **is** the signal path,
so the probe changes the quantity being measured rather than observing it. The
same applies to a mains-powered oscilloscope's ground clip and to USB.

*Mitigation:* all dataset captures are **battery powered with no wired
connection to the host**, over BLE. The host laptop also runs on battery and is
kept ~1 m from the participant. A paired BLE/RTT capture (§6.4) quantifies the
effect and is reported.

### 4.4 Temporal drift

Skin hydration and contact impedance drift measurably over a session.

*Mitigation:* conditions are **interleaved in blocks**, never run as all-Rev-2
then all-Rev-3, with the starting condition randomised.

### 4.5 Sampling rate vs dip duration

At 125 Hz, nothing shorter than ~16 ms can be resolved. If Rev 2's dips are
faster than that, the sampled dataset systematically understates them.

*Mitigation:* oscilloscope capture at full bandwidth (§6.3) establishes the true
dip duration before the dataset is trusted. If dips prove to be sub-16 ms, the
125 Hz result becomes a lower bound and must be reported as such.

---

## 5. Ground truth

Counting touches and comparing to counts of detections **cannot work**: misses
and false alarms cancel in a count, and no ROC can be built from it. Ground truth
must be timestamped.

**Method.** An operator presses **SPACE** in `key_logger.html` at each touch
onset. Touches are 2 s apart, so ±200 ms operator latency is negligible for
per-event scoring.

**Detections.** The board's HID keystrokes (A / D) land in the same browser page
and are logged with the same clock. OS auto-repeat is filtered.

**Clock alignment.** Both the capture script and the browser record absolute
macOS system time. `analyze_vlog.py` assigns each browser event to whichever
capture its wall-clock timestamp falls inside, then maps it onto the device
clock via a linear fit over the signal rows.

*Upgrade if time allows:* a conductive-tape contact sensor into a spare GPIO
would give sub-millisecond hardware ground truth and remove operator latency
entirely.

---

## 6. Procedure

### 6.1 Setup (once, ~30 min)

- [ ] Flash the common firmware to both boards; verify build identity
- [ ] `pip install bleak`; grant macOS Bluetooth permission to Terminal
      (System Settings → Privacy & Security → Bluetooth)
- [ ] **Forget the board on any phone**, or disable that phone's Bluetooth —
      a bonded phone auto-connects as a HID keyboard and stops the board
      advertising
- [ ] `python3 vlog_capture.py --scan` to confirm the advertised name
- [ ] 30 s dry run; confirm **~125 Hz** and **<1 % drops**
- [ ] Mark electrode positions on the skin
- [ ] Metronome at 0.5 Hz (30 BPM)
- [ ] Laptop on battery, ~1 m from participant

**Roles.** *Participant* performs the paced touch motion. *Operator* presses
SPACE at each touch onset and watches the live display.

### 6.2 Session structure (~45 min)

Interleaved, starting condition randomised:

| Block | Board | Condition | Duration | File |
|---|---|---|---|---|
| 1 | Rev 2 | 25 touches | ~1 min | `rev2_b1.csv` |
| 2 | Rev 3 | 25 touches | ~1 min | `rev3_b1.csv` |
| 3 | Rev 2 | still, no touch | 2 min | `rev2_quiet1.csv` |
| 4 | Rev 3 | still, no touch | 2 min | `rev3_quiet1.csv` |
| 5 | Rev 3 | 25 touches | ~1 min | `rev3_b2.csv` |
| 6 | Rev 2 | 25 touches | ~1 min | `rev2_b2.csv` |
| 7 | Rev 3 | 25 touches | ~1 min | `rev3_b3.csv` |
| 8 | Rev 2 | 25 touches | ~1 min | `rev2_b3.csv` |
| 9 | Rev 2 | 25 touches | ~1 min | `rev2_b4.csv` |
| 10 | Rev 3 | 25 touches | ~1 min | `rev3_b4.csv` |

100 touches and 2 minutes of quiet per board.

**The quiet blocks are not optional.** They are the only source of false-alarm
rate. With a good detector the touch-block false-alarm count is often zero, which
collapses the ROC into the corner; "3.2 → 0.1 false alarms/min" stays legible.

### 6.3 Per block

```bash
python3 vlog_capture.py --name "Circle Rx Right 1" \
    --out rev2_b1.csv --note "Rev 2, participant A, block 1"
```

With `key_logger.html` open and **focused**, Start pressed.

- [ ] **J-Link unplugged. Board on battery.**
- [ ] Wait for LEDs: cycling → blinking → **steady on** (threshold locked)
- [ ] Metronome on; participant begins
- [ ] Operator presses SPACE at each touch onset
- [ ] `Ctrl+C`; export the browser CSV; record the summary in §10
- [ ] Keep geometry, distance and posture identical between blocks

### 6.4 Additional captures (~30 min)

**Oscilloscope, both boards.** Decides whether Rev 2's dips are real and whether
Rev 3's cleanliness is genuine or filtered.

- [ ] Rev 2 VLOG, ~10 touches, full bandwidth, trace saved
- [ ] Rev 3 VLOG, same settings, trace saved
- [ ] Record **dip duration** (Rev 2) and **10–90 % edge times** (both)
- [ ] Note that the scope ground clip is itself an earth connection; use a
      differential probe or battery-powered scope if available

**Grounding delta.** Same motion, same board, captured twice:

```bash
python3 vlog_capture.py --name "Circle Rx Right 1" --out rev3_ground_ble.csv
python3 vlog_capture.py --source rtt --out rev3_ground_rtt.csv    # do LAST
```

The difference is the probe's grounding effect, and justifies §4.3 in one
sentence. Run the RTT capture last so nothing after it is contaminated.

---

## 7. Metrics

| Metric | Definition |
|---|---|
| **Spurious releases** | KEYUP followed within 300 ms by another KEYDOWN — the key dropped mid-touch. Requires no ground truth. |
| Dropout rate | samples during a hold falling below the baseline↔plateau midpoint, per second of hold |
| Hold integrity | fraction of the hold spent above that midpoint |
| Plateau CV | σ/μ over the hold region |
| Pd | fraction of ground-truth marks matched by a detection within ±1 s, Wilson 95 % CI |
| False alarms / min | detections during quiet blocks |
| Edge time | 10–90 % rise of the touch transition (confound check, §4.2) |

Spurious releases are the headline: a dip only matters to the game if it drops
the key mid-touch, which is the actual failure mode.

---

## 8. Analysis

```bash
python3 analyze_vlog.py \
    --rev2 "rev2_b*.csv" --rev3 "rev3_b*.csv" \
    --rev2-quiet "rev2_quiet*.csv" --rev3-quiet "rev3_quiet*.csv" \
    --events "keys_*.csv" --outdir figs
```

**Figures.**

1. Time series, one panel per revision, ground-truth marks overlaid
2. Per-event ROC, both revisions, with the deployed detector's operating point
3. Stability metrics, four panels (four different units → four axes)

**Statistics.** Wilson score intervals for proportions. With N=100 per
condition, Pd = 0.95 carries roughly ±4 %. Report N for sessions, participants,
and events per session.

**Optional — log-amp calibration sweep.** Required only if any cross-board
*level* comparison is reported. Inject known power through a step attenuator,
record VLOG per level, fit slope and intercept per board, and report the
improvement in **dB of recovered signal** rather than volts.

---

## 9. Data management and provenance

Every capture writes a `.meta.json` sidecar recording peer BLE address,
advertised name, characteristic UUID, packet format, transport, grounding
condition, ADC channel mapping, resolution and full-scale, start time in UTC,
sample count and drop count.

This matters because a **UUID identifies the kind of characteristic, not the
source**. Provenance comes from the peer address (which board), the UUID (which
pipe), and the firmware plus schematic (which node in the circuit — AIN1 / P0.03
= VLOG). Only the first two are discoverable over the air; the third must be
asserted and cross-checked.

Raw ADC counts are the archival quantity. Millivolt conversion is lossy and is
recomputed in analysis, so a per-board calibration factor can be applied later
without re-collecting.

---

## 10. Limitations to state in the paper

1. **Absolute accuracy.** SAADC gain error is specified −3 % to +4 %; without a
   one-point calibration against a traceable source, absolute voltages carry
   roughly ±4 %. Relative and within-board comparisons are unaffected.
2. **Acquisition time.** 10 µs default, rated to 100 kΩ source resistance. If
   VLOG is not op-amp buffered, the sample-and-hold under-reads by an amount
   that varies with contact impedance — which correlates with the measurand.
   Verify the front end; raise to 40 µs if unbuffered.
3. **Two components changed** alongside the front-end topology (§4.1, §4.2).
   Attribute the improvement to the Rev 3 redesign as a whole.
4. **Temporal resolution.** 125 Hz cannot resolve sub-16 ms events; the scope
   measurement bounds this.
5. **Sample size.** Report participants and sessions honestly. One participant
   is acceptable for a two-page summary if stated; the detector adapts per
   player pair, so cross-participant variability is a known open question.

---

## 11. Session log

| Block | File | Marks | Detections | Rate (Hz) | Drops | Notes |
|---|---|---|---|---|---|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |
| 4 | | | | | | |
| 5 | | | | | | |
| 6 | | | | | | |
| 7 | | | | | | |
| 8 | | | | | | |
| 9 | | | | | | |
| 10 | | | | | | |

Participant(s): ______  Date: ______  Room temp: ______  Start order: ______

Record anything unusual — slipped electrode, participant paused, metronome
drift, unexpected LED state. An explained outlier is worth far more than an
unexplained one, and you will not remember in a week.
