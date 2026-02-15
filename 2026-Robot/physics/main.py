import numpy as np
import scipy.constants as const


def getVelocityVector(initalPoint, finalPoint, maxHeight):
    [xi, yi, zi] = initalPoint
    [xf, yf, zf] = finalPoint
    vzi = np.sqrt(2 * const.g * (maxHeight - zi))
    r = np.sqrt((xf - xi) ** 2 + (yf - yi) ** 2)
    vri = r / (vzi / const.g + np.sqrt(2 * (maxHeight - zf) / const.g))
    theta = np.arctan2(yf - yi, xf - xi)
    vxi = vri * np.cos(theta)
    vyi = vri * np.sin(theta)
    return np.array([vxi, vyi, vzi])


initialPoint = np.array([0, 0, 0.635])
finalPoint = np.array([0, 0, 1.83])
maxHeight = 4.0
import matplotlib.pyplot as plt

# Generate trajectories for varying x-coordinates of the final point
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
for i in range(10):
    x_offset = 0.5 + 0.5 * i
    maxIncrement = 0 * i
    updatedFinalPoint = np.array(
        [finalPoint[0] + x_offset, finalPoint[1], finalPoint[2]]
    )
    velocity = getVelocityVector(
        initialPoint, updatedFinalPoint, maxHeight + maxIncrement
    )
    angle = np.deg2rad(0 * i)
    rotation_matrix = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1],
        ]
    )
    rotatedFinalPoint = rotation_matrix @ updatedFinalPoint
    velocity = getVelocityVector(
        initialPoint, rotatedFinalPoint, maxHeight + maxIncrement
    )
    velocity[0] = velocity[0]
    print(np.arctan(velocity[2] / velocity[0]) * 180 / np.pi)
    time = np.linspace(0, 2 * velocity[2] / const.g, num=500)
    x = initialPoint[0] + velocity[0] * time
    y = initialPoint[1] + velocity[1] * time
    z = initialPoint[2] + velocity[2] * time - 0.5 * const.g * time**2
    ax.plot(x, y, z, label=f"Trajectory (rotation={18 * i:.1f}°)")
# Calculate the velocity vector
velocity = getVelocityVector(initialPoint, finalPoint, maxHeight)

# Time array for plotting
time = np.linspace(0, 2 * velocity[2] / const.g, num=500)

# Calculate trajectory
x = initialPoint[0] + velocity[0] * time
y = initialPoint[1] + velocity[1] * time
z = initialPoint[2] + velocity[2] * time - 0.5 * const.g * time**2

# Find the time at max height
t_max_height = velocity[2] / const.g
x_max_height = initialPoint[0] + velocity[0] * t_max_height
y_max_height = initialPoint[1] + velocity[1] * t_max_height
z_max_height = maxHeight

# Labels and legend
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
# Hide the Y-axis
ax.yaxis.set_tick_params(labelleft=False)
ax.yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))  # Make the Y-axis line invisible

# Show the plot
plt.savefig("trajectory.png")
