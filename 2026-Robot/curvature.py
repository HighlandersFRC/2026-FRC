import json
import matplotlib.pyplot as plt

# === Load your path JSON ===
with open("src/main/deploy/3PieceFeederSmart.polarauto", "r") as f:
    data = json.load(f)

# === Pull out the first path's sampled points ===
sampled_points = data["paths"][0]["sampled_points"]

# === Extract values ===
# Initialize lists for graphing
all_times = []
all_curvatures = []
prevTime = 0.0
# Iterate through all paths in data["paths"]
for path in data["paths"]:
    sampled_points = path["sampled_points"]

    # Extract values for the current path
    times = []
    curvatures = []

    for pt in sampled_points:
        t = pt["time"] + prevTime
        x_vel = pt["x_velocity"]
        y_vel = pt["y_velocity"]
        x_accel = pt["x_acceleration"]
        y_accel = pt["y_acceleration"]

        # Calculate curvature
        dx = x_vel
        dy = y_vel
        ddx = x_accel
        ddy = y_accel

        numerator = abs(dx * ddy - dy * ddx)
        denominator = (dx**2 + dy**2) ** 1.5

        curvature = numerator / denominator if denominator > 1e-6 else 0.0

        times.append(t)
        curvatures.append(curvature)
    prevTime = times[-1] if times else prevTime
    # Append current path's data to the overall lists
    all_times.extend(times)
    all_curvatures.extend(curvatures)

# Plot the graph
plt.figure(figsize=(10, 6))
plt.plot(all_times, all_curvatures, label="Curvature vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Curvature")
plt.title("Curvature vs Time for All Paths")
plt.legend()
plt.grid()
plt.ylim(0, 2)
plt.savefig("curvature_plot.png")
