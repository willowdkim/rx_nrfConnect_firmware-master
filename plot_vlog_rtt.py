import subprocess
import time
from collections import deque

import matplotlib.pyplot as plt
import select

def main():
    stop = False
    t_plot = deque()
    v_plot = deque()
    start_time = time.time()
    window = 10.0

    def on_key(event):
        nonlocal stop
        if event.key == " ":
            stop = True

    print("Starting JLinkRTTClient...")
    print("Close RTT Terminal first.")
    print("Press spacebar in plot window to stop.")

    proc = subprocess.Popen(
        ["JLinkRTTClient"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect("key_press_event", on_key)

    line, = ax.plot([], [])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("VLOG (V)")
    ax.set_title("Live VLOG Voltage from RTT")
    ax.grid(True)

    try:
        while not stop:
            ready, _, _ = select.select([proc.stdout], [], [], 0.05)

            if ready:
                raw_line = proc.stdout.readline().strip()

                if raw_line.startswith("VLOG_MV,"):
                    try:
                        vlog_mv = float(raw_line.split(",")[1])
                        vlog_v = vlog_mv / 1000.0
                    except ValueError:
                        continue

                    t = time.time() - start_time
                    t_plot.append(t)
                    v_plot.append(vlog_v)

                    while t_plot and (t - t_plot[0]) > window:
                        t_plot.popleft()
                        v_plot.popleft()

                    line.set_data(t_plot, v_plot)
                    ax.relim()
                    ax.autoscale_view()

            plt.pause(0.01)

    finally:
        proc.terminate()

    print("Stopped.")


if __name__ == "__main__":
    main()