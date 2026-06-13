import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Circle

# =========================
# Read CSV
# =========================

ref = pd.read_csv("output/reference_line.csv")

dp = pd.read_csv("output/dp_path.csv")

qp = pd.read_csv("output/qp_path.csv")

obs = pd.read_csv("output/obstacles.csv")

boundary = pd.read_csv("output/boundary.csv")

# =========================
# Figure 1
# Cartesian View
# =========================

fig1, ax1 = plt.subplots(figsize=(14,7))

# Reference Line

ax1.plot(
    ref["x"],
    ref["y"],
    '--',
    linewidth=2,
    label="Reference Line"
)

# DP Path

ax1.plot(
    dp["x"],
    dp["y"],
    'o--',
    linewidth=1.5,
    label="DP Path"
)

# QP Path

ax1.plot(
    qp["x"],
    qp["y"],
    linewidth=3,
    label="QP Smoothed Path"
)

# Obstacles

for _, row in obs.iterrows():

    obstacle_circle = Circle(
        (row["x"], row["y"]),
        radius=1.0,
        alpha=0.5
    )

    ax1.add_patch(obstacle_circle)

# fake legend point

ax1.scatter(
    [],
    [],
    s=100,
    label="Obstacles"
)

ax1.set_xlabel("x (m)")
ax1.set_ylabel("y (m)")

ax1.set_title("Cartesian View")

ax1.grid(True)

ax1.legend()

ax1.axis("equal")

# =========================
# Figure 2
# Frenet SL View
# =========================

fig2, ax2 = plt.subplots(figsize=(14,7))

# Boundary

ax2.plot(
    boundary["s"],
    boundary["l_min"],
    'r--',
    linewidth=2,
    label="Boundary Min"
)

ax2.plot(
    boundary["s"],
    boundary["l_max"],
    'b--',
    linewidth=2,
    label="Boundary Max"
)

# fill corridor

ax2.fill_between(
    boundary["s"],
    boundary["l_min"],
    boundary["l_max"],
    alpha=0.15,
    label="Path Boundary"
)

# DP Path in SL

ax2.plot(
    dp["s"],
    dp["l"],
    'o--',
    linewidth=1.5,
    label="DP Path"
)

# QP Path in SL

ax2.plot(
    qp["s"],
    qp["l"],
    linewidth=3,
    label="QP Smoothed Path"
)

# obstacle in SL

for _, row in obs.iterrows():

    circle = Circle(
        (row["s"], row["l"]),
        radius=1.0,
        alpha=0.5
    )

    ax2.add_patch(circle)

ax2.scatter(
    [],
    [],
    s=100,
    label="Obstacles"
)

ax2.set_xlabel("s (m)")
ax2.set_ylabel("l (m)")

ax2.set_title("Frenet SL View")

ax2.grid(True)

ax2.legend()

plt.show()