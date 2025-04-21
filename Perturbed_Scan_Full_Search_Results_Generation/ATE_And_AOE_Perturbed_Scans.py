import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import glob
import matplotlib.pyplot as plt
import seaborn as sns

gt_poses = []
with open("00.txt", "r") as f:
    for line in f:
        vals = list(map(float, line.strip().split()))
        pose = np.array(vals).reshape(3, 4)
        gt_poses.append(pose)

files = sorted(glob.glob("estimated_trajectory_perturb*.csv"))

print(f"{'Guess':<25} {'ATE (m)':<10} {'AOE (deg)':<10}")
print("-" * 45)

labels = []
ate_list = []
aoe_list = []

for file in files:
    df = pd.read_csv(file)
    frame = int(df.iloc[0]['frame'])

    t_est = df[["x", "y", "z"]].values[0]
    rpy_est = df[["roll", "pitch", "yaw"]].values[0]
    R_est = R.from_euler('xyz', rpy_est).as_matrix()

    gt_pose = gt_poses[frame]
    t_gt = gt_pose[:, 3]
    R_gt = gt_pose[:, :3]

    ate = np.linalg.norm(t_gt - t_est)

    rot_diff = R_gt.T @ R_est
    angle_error = np.arccos(np.clip((np.trace(rot_diff) - 1) / 2, -1.0, 1.0))
    aoe = np.degrees(angle_error)

    name = file.replace("estimated_trajectory_", "").replace(".csv", "")
    print(f"{name:<25} {ate:<10.4f} {aoe:<10.4f}")

    labels.append(name)
    ate_list.append(ate)
    aoe_list.append(aoe)

sns.set(style="whitegrid")
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

sns.lineplot(x=labels, y=ate_list, marker='o', ax=axes[0])
axes[0].set_ylabel("ATE (m)")
axes[0].set_title("Absolute Trajectory Error (ATE) vs Perturbation")
axes[0].tick_params(axis='x', rotation=45)

sns.lineplot(x=labels, y=aoe_list, marker='o', ax=axes[1])
axes[1].set_ylabel("AOE (deg)")
axes[1].set_xlabel("Perturbation")
axes[1].set_title("Angular Orientation Error (AOE) vs Perturbation")
axes[1].tick_params(axis='x', rotation=45)

plt.tight_layout()
plt.show()


