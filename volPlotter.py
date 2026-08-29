from importlib.resources import files
from operator import index
import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

DEFAULT_FOLDER = Path(r"/Users/willowkim/Library/CloudStorage/OneDrive-TheOhioStateUniversity/Circle/Vlog")


# Sampling rate from MCU/logger
SAMPLE_PERIOD_MS = 30
SAMPLE_PERIOD_S = SAMPLE_PERIOD_MS / 1000.0

# If True, x-axis is based on sample number * 30 ms.
# If False, x-axis uses the time_ms column from the file.
USE_FIXED_SAMPLE_RATE_TIME = True


def load_vlog(path: str) -> pd.DataFrame:
    expected = ["time_ms", "raw_adc", "millivolts"]

    df = pd.read_csv(path, comment="#", on_bad_lines="skip")

    if list(df.columns) != expected:
        df = pd.read_csv(path, header=None, names=expected, on_bad_lines="skip")

    for c in expected:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    df = df.dropna().copy()
    df = df.reset_index(drop=True)

    if USE_FIXED_SAMPLE_RATE_TIME:
        # Correct x-axis using known sample rate:
        # sample 0 = 0.00 s
        # sample 1 = 0.03 s
        # sample 2 = 0.06 s
        # etc.
        df["time_s"] = df.index * SAMPLE_PERIOD_S
    else:
        # Original method: use MCU-provided timestamps
        df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0

    return df


def list_files():
    files = [f for f in os.listdir(DEFAULT_FOLDER) if f.lower().endswith(".txt")]
    files.sort()
    return files


def choose_file(files, prompt):
    print(f"\n{prompt}")
    for i, f in enumerate(files):
        print(f"[{i}] {f}")

    selection = input("Enter file number: ").strip()

    if not selection.isdigit():
        print("Invalid input.")
        return None

    index = int(selection)

    if index < 0 or index >= len(files):
        print("Number out of range.")
        return None

    return DEFAULT_FOLDER / files[index]


def main():

    files = list_files()

    if not files:
        print("No .txt files found.")
        return

    path1 = choose_file(files, "Select FIRST file:")
    if not path1:
        return

    path2 = choose_file(files, "Select SECOND file:")
    if not path2:
        return

    df1 = load_vlog(path1)
    df2 = load_vlog(path2)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 7))

    # First file
    ax1.plot(df1["time_s"], df1["millivolts"], color="blue")
    ax1.set_title(os.path.basename(path1))
    ax1.set_ylabel("Voltage (mV)")
    ax1.grid(True)

    # Second file
    ax2.plot(df2["time_s"], df2["millivolts"], color="red")
    ax2.set_title(os.path.basename(path2))
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Voltage (mV)")
    ax2.grid(True)

    if USE_FIXED_SAMPLE_RATE_TIME:
        fig.suptitle(f"Voltage vs Time Using Fixed {SAMPLE_PERIOD_MS} ms Sample Period")
    else:
        fig.suptitle("Voltage vs Time Using Logged time_ms Column")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()