#!/usr/bin/env python3
"""
Capture VLOG voltage data from a Circle Rx board to a CSV file.

Two sources, one output format:

  --source ble   Nordic UART Service, battery powered, floating ground. Full
                 125 Hz via 6-sample batched packets. THIS IS THE PRIMARY
                 DATASET: it is the representative measurement condition.
  --source rtt   J-Link RTT. Lossless and convenient, but attaching the probe
                 ties board ground to the PC and hence to mains earth. In a
                 body-coupled system the body-to-earth return path IS the
                 signal path, so the probe perturbs what you are measuring.
                 Use for debugging, and to quantify the grounding effect by
                 capturing the same motion both ways.

Output CSV columns:

    host_time_s   seconds since capture start, measured on this computer
    device_ms     uptime milliseconds from the board - the authoritative axis
    kind          V | KEYDOWN | KEYUP | MARK
    raw_adc       12-bit SAADC counts, unfiltered (kind=V only)
    millivolts    raw * 3600 / 4096, convenience only - recompute in analysis
                  with a per-board calibration factor
    seq           NUS sequence number (BLE only; -1 for RTT); label for MARK

Three row kinds share one device clock, which is what makes per-event scoring
possible:

    V         the signal
    KEYDOWN   the detector fired      -> compare against MARK for Pd / Pfa
    MARK      operator saw a real touch (press ENTER)

Counting touches and detections separately cannot separate a miss from a false
alarm - the two cancel. Timestamped MARK and KEYDOWN rows can.

Examples
--------
    python3 vlog_capture.py --source rtt --out rev3_run1.csv
    python3 vlog_capture.py --source ble --name "Circle Rx Right 1" --out ble_demo.csv

Setup
-----
    pip install bleak

macOS also needs Bluetooth permission for whatever runs the script:
System Settings -> Privacy & Security -> Bluetooth -> enable Terminal (or your
IDE). Without it, scanning silently finds nothing.

    python3 vlog_capture.py --scan          # find the board's exact name

RTT source additionally requires JLinkRTTClient on PATH, with RTT Viewer CLOSED.
"""

import argparse
import asyncio
import csv
import json
import platform
import socket
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

# NUS UUIDs (Nordic UART Service)
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # device -> host, notify

ADC_FULL_SCALE_MV = 3600
ADC_MAX_COUNTS = 4096  # 2^12, per the SAADC RESULT formula

# RTT lines look like:  VLOG,123456,1487
RTT_LINE = re.compile(r"^VLOG,(\d+),(-?\d+)\s*$")
# Detector decisions, same clock:  KEY,123456,1   (1 = down, 0 = up)
KEY_LINE = re.compile(r"^KEY,(\d+),([01])\s*$")
# BLE notifications are BINARY, 18 bytes, little endian:
#   [0..3]  uint32 uptime_ms of the first sample in the batch
#   [4..5]  uint16 packet sequence
#   [6..]   6 x uint16 raw SAADC counts, SAMPLE_PERIOD_MS apart
NUS_BATCH_SAMPLES = 6
NUS_PACKET_BYTES = 6 + 2 * NUS_BATCH_SAMPLES
SAMPLE_PERIOD_MS = 8


# Detector events arrive as a 6-byte packet on the same characteristic:
#   [0..3] uint32 uptime_ms   [4] 0x4B ('K')   [5] 1 = down, 0 = up
NUS_KEY_BYTES = 6       # 'K' - detector fired / released
NUS_STATE_BYTES = 12    # 'S' - calibration state + thresholds
NUS_KEY_TAG = 0x4B
NUS_STATE_TAG = 0x53

# What the board reports about its own detector.
STATE_NAMES = {
    0: "waiting for touch",
    1: "CALIBRATING",
    2: "READY",
}


def decode_key_packet(data: bytes):
    """-> (device_ms, down) or None."""
    if len(data) != NUS_KEY_BYTES or data[4] != NUS_KEY_TAG:
        return None
    return int.from_bytes(data[0:4], "little"), bool(data[5])


def decode_state_packet(data: bytes):
    """-> (device_ms, state, baseline_mv, on_mv, off_mv) or None.

    state: 0 = waiting, 1 = calibrating, 2 = locked. Thresholds are 0 until
    locked; they exist only on the board, so they ride with this packet."""
    if len(data) != NUS_STATE_BYTES or data[4] != NUS_STATE_TAG:
        return None
    sgn = lambda b: int.from_bytes(b, "little", signed=True)
    return (int.from_bytes(data[0:4], "little"), data[5],
            sgn(data[6:8]), sgn(data[8:10]), sgn(data[10:12]))


def decode_packet(data: bytes):
    """-> (seq, [(device_ms, raw), ...]) or None if not a VLOG packet."""
    if len(data) != NUS_PACKET_BYTES:
        return None
    t0 = int.from_bytes(data[0:4], "little")
    seq = int.from_bytes(data[4:6], "little")
    out = []
    for i in range(NUS_BATCH_SAMPLES):
        raw = int.from_bytes(data[6 + 2 * i: 8 + 2 * i], "little")
        out.append((t0 + i * SAMPLE_PERIOD_MS, raw))
    return seq, out


def raw_to_mv(raw: int) -> int:
    if raw < 0:
        return -1
    return (raw * ADC_FULL_SCALE_MV + ADC_MAX_COUNTS // 2) // ADC_MAX_COUNTS


class Writer:
    """CSV writer plus running statistics and drop detection."""

    def __init__(self, path: Path, marker_label: str, live: bool = True):
        self.path = path
        self.marker_label = marker_label
        self.fh = path.open("w", newline="")
        self.csv = csv.writer(self.fh)
        self.csv.writerow(
            ["host_epoch_s", "device_ms", "kind", "raw_adc", "millivolts", "seq"]
        )
        # Absolute wall clock, so a separate keystroke log recorded in the
        # browser on the same machine can be merged with this one. monotonic()
        # drives the elapsed time (immune to NTP steps); the epoch offset is
        # captured once.
        self.t0 = time.monotonic()
        self.epoch0 = time.time()
        self.n = 0
        self.markers = 0
        self.detections = 0
        self.drops = 0
        self._last_seq = None
        self._lock = threading.Lock()
        self.last_raw = 0
        self.last_mv = 0
        self.live = live
        self._last_live = 0.0
        # Connect and preview first; write nothing until armed. Keeps the
        # calibration touch and the setup fumbling out of the block's data.
        self.armed = False
        # What the board says its detector is doing. None until it reports.
        self.detector_state = None
        self.baseline_mv = self.on_mv = self.off_mv = None
        self.locked_device_ms = None

    def arm(self) -> None:
        """Begin recording. Resets the clocks and counters so the file starts
        clean at t=0."""
        if self.detector_state != 2:
            got = STATE_NAMES.get(self.detector_state, "not reported yet")
            print("\r" + " " * 78, end="")
            print(f"\r  !! Detector is '{got}', not READY.\n"
                  "     Touch once to trigger calibration and wait ~4.5 s for\n"
                  "     READY, or press ENTER again to record anyway.\n",
                  flush=True)
            self.detector_state = 2   # a second ENTER overrides
            return

        with self._lock:
            self.armed = True
            self.t0 = time.monotonic()
            self.epoch0 = time.time()
            self.n = self.markers = self.detections = self.drops = 0
            self._last_seq = None
        print("\r" + " " * 78, end="")
        print(f"\r  ---- RECORDING from {time.strftime('%H:%M:%S', time.localtime())}"
              f" ----   ENTER marks a touch, Ctrl+C stops\n", flush=True)

    def sample(self, device_ms: int, raw: int, seq: int = -1) -> None:
        with self._lock:
            self.last_raw = raw
            self.last_mv = raw_to_mv(raw)
            if not self.armed:
                self._live()
                return

            if seq >= 0 and self._last_seq is not None:
                gap = (seq - self._last_seq) % 65536
                if gap != 1:
                    # Each missing packet carries NUS_BATCH_SAMPLES samples.
                    self.drops += max(gap - 1, 0) * NUS_BATCH_SAMPLES
            if seq >= 0:
                self._last_seq = seq

            self.csv.writerow(
                [
                    f"{self.epoch0 + time.monotonic() - self.t0:.4f}",
                    device_ms,
                    "V",
                    raw,
                    raw_to_mv(raw),
                    seq,
                ]
            )
            self.n += 1
            if self.n % 200 == 0:
                self.fh.flush()
            self._live()

    def _live(self) -> None:
        """One updating line, ~5 Hz: local wall clock, latest reading, health.
        Called with the lock held."""
        if not self.live:
            return
        now = time.monotonic()
        if now - self._last_live < 0.2:
            return
        self._last_live = now
        wall = time.strftime("%H:%M:%S", time.localtime())
        if not self.armed:
            st = STATE_NAMES.get(self.detector_state, "detector: unknown")
            hint = ("READY - press ENTER to record" if self.detector_state == 2
                    else st)
            sys.stdout.write(
                f"\r  PREVIEW  {wall}   {self.last_mv:>5d} mV"
                f"  (raw {self.last_raw:>4d})   [{hint}]        "
            )
            sys.stdout.flush()
            return
        elapsed = now - self.t0
        rate = self.n / elapsed if elapsed > 0 else 0.0
        sys.stdout.write(
            f"\r  REC {wall}   {self.last_mv:>5d} mV  (raw {self.last_raw:>4d})"
            f"   {rate:5.1f} Hz   marks {self.markers:<3d} det {self.detections:<3d}"
            f" drop {self.drops:<4d}  "
        )
        sys.stdout.flush()

    def marker(self) -> None:
        """Ground-truth mark: operator saw a touch happen."""
        if not self.armed:
            return
        with self._lock:
            self.csv.writerow(
                [
                    f"{self.epoch0 + time.monotonic() - self.t0:.4f}",
                    "",
                    "MARK",
                    "",
                    "",
                    self.marker_label,
                ]
            )
            self.markers += 1
            self.fh.flush()
        print(f"\r  [mark {self.markers:>3d}] {self.marker_label}"
              f"   at {time.strftime('%H:%M:%S', time.localtime())}"
              + " " * 20, flush=True)

    def state_event(self, device_ms: int, state: int,
                    baseline_mv: int = 0, on_mv: int = 0,
                    off_mv: int = 0) -> None:
        """Detector calibration state, reported by the board itself - so the
        host does not have to infer it from watching the LEDs."""
        changed = state != self.detector_state
        self.detector_state = state
        if state == 2:
            self.baseline_mv, self.on_mv, self.off_mv = baseline_mv, on_mv, off_mv
            self.locked_device_ms = device_ms

        if self.armed:
            with self._lock:
                self.csv.writerow(
                    [f"{self.epoch0 + time.monotonic() - self.t0:.4f}",
                     device_ms, "STATE", "", "", state]
                )
                self.fh.flush()

        if not changed:
            return

        stamp = time.strftime("%H:%M:%S", time.localtime())
        print("\r" + " " * 78, end="")

        if state == 1:
            print(f"\r  [{stamp}] Calibration stage has started"
                  " ..." + " " * 20, flush=True)
        elif state == 2:
            print(f"\r  [{stamp}] Calibration stage has ended. "
                  f"Threshold value is: {on_mv} mV" + " " * 10)
            print(f"             (release {off_mv} mV, baseline {baseline_mv} mV)")
            print("             Press ENTER to collect data.\n", flush=True)
        else:
            print(f"\r  [{stamp}] Detector waiting - touch once to begin"
                  " calibration" + " " * 10, flush=True)

    def key_event(self, device_ms: int, down: bool) -> None:
        """Detector decision, on the device clock."""
        if not self.armed:
            return
        with self._lock:
            self.csv.writerow(
                [
                    f"{self.epoch0 + time.monotonic() - self.t0:.4f}",
                    device_ms,
                    "KEYDOWN" if down else "KEYUP",
                    "",
                    "",
                    "",
                ]
            )
            if down:
                self.detections += 1
            self.fh.flush()

    def write_meta(self, extra: dict) -> Path:
        """Provenance sidecar. A UUID identifies the KIND of characteristic, not
        the source, so the record has to carry the peer address, the advertised
        name and the firmware's channel mapping explicitly. Written next to the
        CSV as <name>.meta.json."""
        meta = {
            "csv": self.path.name,
            "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                         time.gmtime(self.epoch0)),
            "started_epoch_s": round(self.epoch0, 4),
            "duration_s": round(time.monotonic() - self.t0, 2),
            "samples": self.n,
            "marks": self.markers,
            "detections": self.detections,
            "dropped_samples": self.drops,
            "host": socket.gethostname(),
            "platform": platform.platform(),
            "capture_tool": "vlog_capture.py",
            # Signal chain, from the firmware. Not discoverable over BLE - it
            # has to be asserted here and cross-checked against the schematic.
            "adc_channel": "AIN1 (P0.03) = VLOG",
            "adc_resolution_bits": 12,
            "adc_full_scale_mv": ADC_FULL_SCALE_MV,
            "adc_max_counts": ADC_MAX_COUNTS,
            "sample_period_ms": SAMPLE_PERIOD_MS,
            # Detector thresholds for this block, as calibrated on the board.
            "threshold_on_mv": self.on_mv,
            "threshold_off_mv": self.off_mv,
            "threshold_baseline_mv": self.baseline_mv,
            "calibration_locked_device_ms": self.locked_device_ms,
        }
        meta.update(extra)
        out = self.path.with_suffix(".meta.json")
        out.write_text(json.dumps(meta, indent=2) + "\n")
        return out

    def close(self) -> None:
        with self._lock:
            self.fh.flush()
            self.fh.close()

    def summary(self) -> str:
        elapsed = time.monotonic() - self.t0
        rate = self.n / elapsed if elapsed > 0 else 0.0
        out = [
            f"Saved {self.n} samples to {self.path.resolve()}",
            f"  duration     {elapsed:.1f} s",
            f"  mean rate    {rate:.1f} Hz",
            f"  marks        {self.markers}   (ground truth)",
            f"  detections   {self.detections}   (KEYDOWN events)",
        ]
        if self._last_seq is not None:
            total = self.n + self.drops
            pct = (100.0 * self.drops / total) if total else 0.0
            out.append(f"  dropped      {self.drops} ({pct:.2f}%)")
            if pct > 1.0:
                out.append(
                    "  WARNING: >1% loss. Do not use a BLE capture with this"
                    " much loss as a dataset - use --source rtt."
                )
        return "\n".join(out)


def start_marker_thread(writer: Writer, stop: threading.Event) -> threading.Thread:
    def loop():
        while not stop.is_set():
            try:
                if sys.stdin.readline() == "":
                    break
            except Exception:
                break
            if stop.is_set():
                break
            if not writer.armed:
                writer.arm()      # first ENTER starts the recording
            else:
                writer.marker()   # subsequent ENTERs mark touches

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def capture_rtt(writer: Writer, stop: threading.Event, provenance: dict) -> None:
    provenance.update({
        "transport": "J-Link RTT (wired)",
        "grounding": "PERTURBED - debug probe ties board ground to host earth",
    })
    print("Starting JLinkRTTClient (close RTT Viewer/Terminal first)...")
    proc = subprocess.Popen(
        ["JLinkRTTClient"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    print("Capturing. ENTER = marker, Ctrl+C = stop.\n")
    try:
        for line in proc.stdout:
            if stop.is_set():
                break
            stripped = line.strip()
            m = RTT_LINE.match(stripped)
            if m:
                writer.sample(int(m.group(1)), int(m.group(2)))
                continue
            k = KEY_LINE.match(stripped)
            if k:
                writer.key_event(int(k.group(1)), k.group(2) == "1")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


async def capture_ble(writer: Writer, stop: threading.Event, name: str,
                      provenance: dict, address: str = "") -> None:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        sys.exit("BLE source needs bleak:  pip install bleak")

    # Use discover() - the same call --scan uses. find_device_by_filter() is
    # unreliable on macOS because BLEDevice.name is frequently None there: the
    # name arrives in a scan RESPONSE, not the advertisement, so a name filter
    # silently never matches even though the board is plainly advertising.
    what = address if address else f"'{name}'"
    print(f"Scanning for {what} ...")

    device = None
    for _ in range(3):                      # ~24 s total before giving up
        for d in await BleakScanner.discover(timeout=8.0):
            dev_name = (d.name or "").strip()
            if address:
                if (d.address or "").upper() == address.upper():
                    device = d
                    break
            elif dev_name.lower() == name.strip().lower():
                device = d
                break
        if device:
            break

    if device is None:
        sys.exit(
            f"\nNot found: {what}\n\n"
            "Run  python3 vlog_capture.py --scan  to list what IS visible.\n"
            "  - If the board is listed, copy the name verbatim (or pass\n"
            "    --address with the identifier shown next to it).\n"
            "  - If it is not listed, it is not advertising: something has it\n"
            "    connected (unpair on this Mac and any phone), or it is not\n"
            "    powered. A connected peripheral stops advertising."
        )

    peer_address = device.address
    peer_name = device.name or name
    target = device

    provenance.update({
        "transport": "BLE / Nordic UART Service",
        "peer_name": peer_name,
        "peer_address": peer_address,
        "characteristic_uuid": NUS_TX_UUID,
        "packet_bytes": NUS_PACKET_BYTES,
        "samples_per_packet": NUS_BATCH_SAMPLES,
        "grounding": "battery powered, no wired connection to host",
    })

    def on_notify(_sender, data: bytearray) -> None:
        raw = bytes(data)

        st = decode_state_packet(raw)
        if st is not None:
            writer.state_event(*st)
            return

        key = decode_key_packet(raw)
        if key is not None:
            writer.key_event(key[0], key[1])
            return

        decoded = decode_packet(raw)
        if decoded is None:
            return
        seq, samples = decoded
        for j, (device_ms, raw) in enumerate(samples):
            # Attribute the packet sequence to the first sample only, so drop
            # accounting counts PACKETS rather than samples.
            writer.sample(device_ms, raw, seq if j == 0 else -1)

    try:
        client_ctx = BleakClient(target)
        await client_ctx.__aenter__()
    except Exception as exc:                                   # noqa: BLE001
        sys.exit(
            f"\nCould not connect: {exc}\n\n"
            "If this says 'not found', the board is not advertising - almost\n"
            "always because this Mac or a phone has it paired and auto-connected.\n"
            "Unpair it everywhere, power-cycle the board, and use --name.\n"
            "Detector events now come over NUS, so the keyboard pairing is not\n"
            "needed for data collection."
        )

    async with client_ctx as client:
        await client.start_notify(NUS_TX_UUID, on_notify)

        # Show what we actually attached to. This is the provenance that
        # matters: the ADDRESS says which board, the UUID says which pipe.
        # A UUID alone identifies the kind of characteristic, not the source.
        print()
        print(f"  Connected   {peer_name}")
        print(f"  Address     {peer_address}   <- which board")
        print(f"  Subscribed  {NUS_TX_UUID}")
        print( "              NUS TX characteristic, notify   <- which pipe")
        print(f"  Packets     {NUS_PACKET_BYTES} bytes = {NUS_BATCH_SAMPLES}"
              f" samples @ {SAMPLE_PERIOD_MS} ms")
        print( "  Signal      AIN1 / P0.03 = VLOG, 12-bit, 0-3600 mV")
        print( "  Grounding   battery, no wired link to host")
        print()
        print("  Previewing. Calibrate the board (touch once, wait for steady LEDs),\n  then press ENTER to START RECORDING.")
        print()
        warned = False
        t_connect = time.monotonic()

        while not stop.is_set():
            await asyncio.sleep(0.2)

            # Connected but nothing decodes -> firmware/script format mismatch.
            if (not warned and writer.n == 0 and writer.last_raw == 0
                    and time.monotonic() - t_connect > 6.0):
                warned = True
                print("\r" + " " * 78)
                print("  Connected, but no valid packets in 6 s.\n"
                      "  The board is probably running firmware that predates the\n"
                      "  batched binary stream (it sent text like 'V=1687mV raw=479').\n"
                      "  Apply experiment_instrumentation.patch, pristine build, reflash.\n")

            if not client.is_connected:
                print("Link dropped.")
                break
        try:
            await client.stop_notify(NUS_TX_UUID)
        except Exception:
            pass


async def scan() -> None:
    """List nearby BLE devices, so you can confirm the exact advertised name."""
    try:
        from bleak import BleakScanner
    except ImportError:
        sys.exit("Scanning needs bleak:  pip install bleak")

    print("Scanning 8 s...\n")
    found = await BleakScanner.discover(timeout=8.0)
    named = [d for d in found if d.name]
    if not named:
        print("Nothing with a name found.\n"
              "  - Is the board powered and advertising (yellow LED flashing)?\n"
              "  - If a phone has it bonded, the phone may have auto-connected\n"
              "    as a HID keyboard, which stops it advertising. Turn off that\n"
              "    phone's Bluetooth, or forget the device on it.")
        return
    for d in sorted(named, key=lambda x: x.name or ""):
        star = "  <-- looks like a Circle board" if "circle" in (d.name or "").lower() else ""
        print(f"  {d.name!r}  {d.address}{star}")
    print("\nPass the name verbatim:  --name \"Circle Rx Right 1\"")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", action="store_true",
                    help="list nearby BLE devices and exit")
    ap.add_argument("--source", choices=["rtt", "ble"], default="ble")
    ap.add_argument("--out", help="output CSV path")
    ap.add_argument("--name", default="Circle Rx Right 1",
                    help="BLE advertised name (--source ble)")
    ap.add_argument("--marker-label", default="TOUCH",
                    help="label written when you press ENTER")
    ap.add_argument("--address", default="",
                    help="connect by address instead of matching the name. NOTE: "
                         "on macOS bleak still resolves the address by scanning, "
                         "so this does NOT work for a board the OS has already "
                         "claimed. Unpair the board instead.")
    ap.add_argument("--quiet", action="store_true",
                    help="suppress the live status line")
    ap.add_argument("--note", default="",
                    help="free text recorded in the provenance sidecar, e.g. "
                         "'Rev 3, participant A, block 2'")
    args = ap.parse_args()

    if args.scan:
        asyncio.run(scan())
        return

    if not args.out:
        ap.error("--out is required unless --scan is given")

    out = Path(args.out)
    if out.exists():
        sys.exit(f"Refusing to overwrite existing file: {out}")

    writer = Writer(out, args.marker_label, live=not args.quiet)
    stop = threading.Event()
    start_marker_thread(writer, stop)
    provenance = {"note": args.note} if args.note else {}

    try:
        if args.source == "rtt":
            capture_rtt(writer, stop, provenance)
        else:
            asyncio.run(capture_ble(writer, stop, args.name, provenance,
                                    args.address))
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        writer.close()
        meta = writer.write_meta(provenance)
        print("\n\n" + writer.summary())
        print(f"  provenance   {meta.name}")


if __name__ == "__main__":
    main()
