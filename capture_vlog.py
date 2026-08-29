import argparse
import time
from pathlib import Path

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", default="vlog_log.txt")
    args = parser.parse_args()

    out_path = Path(args.out)

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    print("Capturing VLOG data...")
    print("Press Ctrl+C to stop and save.")

    with out_path.open("w") as f:
        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()

                if line.startswith("VLOG_MV"):
                    print(line)
                    f.write(line + "\n")
                    f.flush()

        except KeyboardInterrupt:
            pass
        finally:
            ser.close()

    print(f"Saved to {out_path.resolve()}")


if __name__ == "__main__":
    main()