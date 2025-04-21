import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

est_df = pd.read_csv("estimated_trajectory_fullsearch.csv")

gt_poses = []
with open("00.txt", "r") as f:
    for line in f:
        vals = list(map(float, line.strip().split()))
        pose = np.array(vals).reshape(3, 4)
        gt_poses.append(pose)

print(f"{'Frame':<8} {'ATE (m)':<10} {'AOE (deg)':<10}")
print("-" * 32)

for idx, row in est_df.iterrows():
    frame = int(row['frame'])
    if frame >= len(gt_poses):
        continue

    t_est = np.array([row['x'], row['y'], row['z']])
    rpy_est = np.array([row['roll'], row['pitch'], row['yaw']])
    R_est = R.from_euler('xyz', rpy_est).as_matrix()

    gt_pose = gt_poses[frame]
    t_gt = gt_pose[:, 3]
    R_gt = gt_pose[:, :3]

    ate = np.linalg.norm(t_gt - t_est)

    rot_diff = R_gt.T @ R_est
    angle_error = np.arccos(np.clip((np.trace(rot_diff) - 1) / 2, -1.0, 1.0))
    aoe = np.degrees(angle_error)

    print(f"{frame:<8} {ate:<10.4f} {aoe:<10.4f}")

