import json
import numpy as np

import matplotlib.pyplot as plt

# === Load your path JSON ===
with open("src/main/deploy/4 new.polarauto", "r") as f:
    data = json.load(f)

# === Pull out the first path's sampled points ===
sampled_points = []
start_times = []
end_times = []
i = 0
for path in data["paths"]:
    for pt in path["sampled_points"]:
        pt["idx"] = i
    sampled_points.extend(path["sampled_points"])
    start_times.append(
        path["sampled_points"][0]["time"] + (end_times[-1] if len(end_times) > 0 else 0)
    )
    end_times.append(
        path["sampled_points"][-1]["time"]
        + (end_times[-1] if len(end_times) > 0 else 0)
    )
    i += 1
print(start_times)
print(end_times)

# === Generate motion profile ===
for point in sampled_points:
    point["position"] = np.sqrt(
        point["x"] ** 2 + point["y"] ** 2
    )  # Example position calculation
    point["velocity"] = np.sqrt(
        point["x_velocity"] ** 2 + point["y_velocity"] ** 2
    )  # Example velocity calculation
    point["acceleration"] = np.sqrt(
        point["x_acceleration"] ** 2 + point["y_acceleration"] ** 2
    )  # Example acceleration calculation
    point["time"] += start_times[point["idx"]]  # Adjust time to start from zero

# === Plot motion profile ===
times = [point["time"] for point in sampled_points]
positions = [point["position"] for point in sampled_points]
velocities = [point["velocity"] for point in sampled_points]
accelerations = [point["acceleration"] for point in sampled_points]

plt.figure(figsize=(12, 8))

# Position vs Time
plt.subplot(3, 1, 1)
plt.plot(times, positions, label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Position vs Time")
plt.grid()

# Velocity vs Time
plt.subplot(3, 1, 2)
plt.plot(times, velocities, label="Velocity", color="orange")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity vs Time")
plt.grid()

# Acceleration vs Time
plt.subplot(3, 1, 3)
plt.plot(times, accelerations, label="Acceleration", color="green")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.title("Acceleration vs Time")
plt.grid()

plt.tight_layout()
plt.savefig("motion_profile_plot.png")
