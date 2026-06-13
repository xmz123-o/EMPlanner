import pandas as pd
import matplotlib.pyplot as plt

grid = pd.read_csv(
    "output/dp_grid_cost.csv")

path = pd.read_csv(
    "output/Test_speed_dp.csv")

boundary = pd.read_csv(
    "output/Test_st_obstacles.csv")

plt.figure(figsize=(12,8))

sc = plt.scatter(
    grid["t"],
    grid["s"],
    c=grid["total_cost"],
    cmap="jet",
    s=8)

for obs_id in boundary["obs_id"].unique():

    obs = boundary[
        boundary["obs_id"] == obs_id]

    plt.fill_between(
        obs["t"],
        obs["s_lower"],
        obs["s_upper"],
        color="black",
        alpha=0.3)

plt.plot(
    path["t"],
    path["s"],
    color="magenta",
    linewidth=3,
    label="Best Path")

plt.colorbar(
    sc,
    label="Total Cost")

plt.xlabel("Time [s]")
plt.ylabel("S [m]")

plt.legend()
plt.grid(True)

plt.show()