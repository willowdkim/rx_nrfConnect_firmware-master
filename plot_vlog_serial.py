import argparse
import time
from collections import deque

import matplotlib.pyplot as plt
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=float, default=10.0)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    stop = False
    t_plot = deque()
    v_plot = deque()
    start_time = time.time()

    def on_key(event):
        nonlocal stop
        if event.key == " ":
            stop = True

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect("key_press_event", on_key)

    line, = ax.plot([], [])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("VLOG (V)")
    ax.set_title("Live VLOG Voltage")
    ax.grid(True)

    print("Reading VLOG_MV lines...")
    print("Close RTT Terminal first. Press spacebar in plot window to stop.")

    while not stop:
        raw_line = ser.readline().decode(errors="ignore").strip()

        # Debug: show what Python actually receives
        print(raw_line)

        if raw_line.startswith("VLOG_MV,"):
            try:
                vlog_mv = float(raw_line.split(",")[1])
                vlog_v = vlog_mv / 1000.0
            except ValueError:
                continue

            t = time.time() - start_time
            t_plot.append(t)
            v_plot.append(vlog_v)

            while t_plot and (t - t_plot[0]) > args.window:
                t_plot.popleft()
                v_plot.popleft()

            line.set_data(t_plot, v_plot)
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.001)

    ser.close()


if __name__ == "__main__":
    main()