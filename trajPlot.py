import pandas as pd
import matplotlib.pyplot as plt

# ==================================================
# Path
# ==================================================

ref = pd.read_csv("output/ref_line.csv")
obs = pd.read_csv("output/obstacles.csv")

dp = pd.read_csv("output/path_dp.csv")
qp = pd.read_csv("output/path_qp.csv")

plt.figure(figsize=(12,6))

plt.plot(
    ref["x"],
    ref["y"],
    label="Reference"
)

plt.plot(
    dp["x"],
    dp["y"],
    label="DP Path"
)

plt.plot(
    qp["x"],
    qp["y"],
    linewidth=3,
    label="QP Path"
)

for _,row in obs.iterrows():

    circle = plt.Circle(
        (row["x"],row["y"]),
        row["r"],
        fill=False
    )

    plt.gca().add_patch(circle)

plt.axis("equal")
plt.grid()
plt.legend()
plt.title("Path Planning")

# ==================================================
# ST
# ==================================================

speed_dp = pd.read_csv("output/speed_dp.csv")
speed_qp = pd.read_csv("output/speed_qp.csv")

plt.figure(figsize=(12,6))

plt.plot(
    speed_dp["t"],
    speed_dp["s"],
    'o-',
    label="Speed DP"
)

plt.plot(
    speed_qp["t"],
    speed_qp["s"],
    linewidth=3,
    label="Speed QP"
)

plt.grid()
plt.legend()
plt.title("ST Graph")

# ==================================================
# Trajectory
# ==================================================

traj = pd.read_csv("output/trajectory.csv")

plt.figure(figsize=(12,6))

plt.plot(
    traj["x"],
    traj["y"],
    linewidth=2
)

plt.axis("equal")
plt.grid()
plt.title("Final Trajectory")

# ==================================================
# Velocity
# ==================================================

plt.figure(figsize=(12,6))

plt.plot(
    traj["t"],
    traj["v"]
)

plt.grid()
plt.title("Velocity")

# ==================================================
# Acceleration
# ==================================================

plt.figure(figsize=(12,6))

plt.plot(
    traj["t"],
    traj["a"]
)

plt.grid()
plt.title("Acceleration")

plt.show()