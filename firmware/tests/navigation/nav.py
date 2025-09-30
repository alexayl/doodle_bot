import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ----- Generate fake data -----
np.random.seed(0)

N = 50  # number of commands
time = np.arange(N) * 0.1  # 0.1s timesteps

# Fake corrections (dx, dy) in meters
dx = 0.05 * np.ones(N) + 0.01 * np.random.randn(N)
dy = 0.02 * np.sin(0.2 * np.arange(N)) + 0.005 * np.random.randn(N)

# Simulated wheel velocities (rad/s)
vL = 20 + 2 * np.sin(0.1 * np.arange(N))
vR = 20 + 2 * np.cos(0.1 * np.arange(N))

# Fake states cycling through [Idle, Draw, Erase]
states = np.array(["Idle", "Draw", "Erase"])
state = [states[i % 3] for i in range(N)]

# Build dataframe (like you would log to CSV)
df = pd.DataFrame({
    "time": time,
    "dx": dx,
    "dy": dy,
    "vL": vL,
    "vR": vR,
    "state": state
})

# ----- Visualization -----
plt.figure(figsize=(12, 5))

# Path reconstruction from dx/dy
plt.subplot(1, 2, 1)
x = np.cumsum(df["dx"])
y = np.cumsum(df["dy"])
plt.plot(x, y, 'o-', label="Path")
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Robot Path from Δx, Δy")
plt.axis("equal")
plt.legend()

# Wheel velocities over time
plt.subplot(1, 2, 2)
plt.plot(df["time"], df["vL"], label="Left wheel")
plt.plot(df["time"], df["vR"], label="Right wheel")
plt.xlabel("Time (s)")
plt.ylabel("Wheel velocity (rad/s)")
plt.title("Wheel Velocities Over Time")
plt.legend()

plt.tight_layout()
plt.show()
