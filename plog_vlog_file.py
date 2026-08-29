import matplotlib.pyplot as plt

times = []
volts = []

with open("vlog_log.txt", "r") as f:
    for line in f:
        line = line.strip()

        if line.startswith("VLOG_MV,"):
            mv = float(line.split(",")[1])
            volts.append(mv / 1000.0)
            times.append(len(times) * 0.03)

plt.plot(times, volts)
plt.xlabel("Time (s)")
plt.ylabel("VLOG (V)")
plt.title("VLOG Voltage vs Time")
plt.grid(True)
plt.show()