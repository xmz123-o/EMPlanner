import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import numpy as np

# ==========================================
# Load CSV
# ==========================================

ref_df = pd.read_csv(
    "output/[SIM]reference_line.csv"
)

ego_df = pd.read_csv(
    "output/[SIM]ego_history.csv"
)

try:
    static_df = pd.read_csv(
        "output/[SIM]static_obstacles.csv"
    )
except:
    static_df = None

try:
    dynamic_df = pd.read_csv(
        "output/[SIM]dynamic_obstacles.csv"
    )
except:
    dynamic_df = None

try:
    traj_df = pd.read_csv(
        "output/SIMtrajectory.csv"
    )
except:
    traj_df = None


# ==========================================
# Figure
# ==========================================

fig, ax = plt.subplots(figsize=(12, 8))

ax.set_title("Autonomous Driving Simulation")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

ax.set_aspect('equal')


# ==========================================
# Reference Line
# ==========================================

ax.plot(
    ref_df["x"],
    ref_df["y"],
    '--',
    color='gray',
    linewidth=2,
    label="Reference Line"
)

# ==========================================
# Planned Trajectory
# ==========================================

if traj_df is not None:

    ax.plot(
        traj_df["x"],
        traj_df["y"],
        '--',
        color='black',
        linewidth=1,
        label="Planned Trajectory"
    )


# ==========================================
# Static Obstacles
# ==========================================

if static_df is not None:

    for _, obs in static_df.iterrows():

        circle = Circle(
            (obs["x"], obs["y"]),
            obs["radius"],
            color='red',
            alpha=0.5
        )

        ax.add_patch(circle)


# ==========================================
# Ego Vehicle
# ==========================================

ego_vehicle = Rectangle(
    (0, 0),
    4.5,
    2.0,
    fill=True,
    alpha=0.8
)

ax.add_patch(ego_vehicle)

# 已走过轨迹

ego_traj, = ax.plot(
    [],
    [],
    'b-',
    linewidth=2,
    label="Ego History"
)

# 当前位置

ego_dot, = ax.plot(
    [],
    [],
    'bo',
    markersize=6
)


# ==========================================
# Dynamic Obstacles
# ==========================================

dyn_dots = []
obs_ids = []

if dynamic_df is not None:

    obs_ids = dynamic_df["id"].unique()

    colors = plt.cm.tab10.colors

    for i, obs_id in enumerate(obs_ids):

        dot, = ax.plot(
            [],
            [],
            'o',
            color=colors[i % len(colors)],
            markersize=8
        )

        dyn_dots.append(dot)


# ==========================================
# Init
# ==========================================

def init():

    ego_dot.set_data([], [])

    ego_traj.set_data([], [])

    return [ego_dot, ego_traj, *dyn_dots]


# ==========================================
# Animation
# ==========================================

def animate(frame):

    # -------------------------
    # Ego
    # -------------------------

    x = ego_df["x"].iloc[frame]
    y = ego_df["y"].iloc[frame]

    ego_dot.set_data([x], [y])

    ego_traj.set_data(
        ego_df["x"][:frame+1],
        ego_df["y"][:frame+1]
    )

    # 车辆矩形

    ego_vehicle.set_xy(
        (
            x - 2.25,
            y - 1.0
        )
    )

    # -------------------------
    # Dynamic Obstacles
    # -------------------------

    if dynamic_df is not None:

        for i, obs_id in enumerate(obs_ids):

            rows = dynamic_df[
                dynamic_df["id"] == obs_id
            ]

            if frame < len(rows):

                ox = rows.iloc[frame]["x"]
                oy = rows.iloc[frame]["y"]

                dyn_dots[i].set_data(
                    [ox],
                    [oy]
                )

    # -------------------------
    # Follow Vehicle
    # -------------------------

    ax.set_xlim(
        x - 30,
        x + 30
    )

    ax.set_ylim(
        y - 15,
        y + 15
    )

    return [
        ego_dot,
        ego_traj,
        ego_vehicle,
        *dyn_dots
    ]


# ==========================================
# Run
# ==========================================

frames = len(ego_df)

ani = animation.FuncAnimation(
    fig,
    animate,
    init_func=init,
    frames=frames,
    interval=50,
    blit=False
)

# ==========================================
# Run
# ==========================================

frames = len(ego_df)

ani = animation.FuncAnimation(
    fig,
    animate,
    init_func=init,
    frames=frames,
    interval=50,
    blit=False
)

ax.legend()

# 保存视频（新增）
ani.save('simulation3.mp4', writer='ffmpeg', fps=20)

plt.show()