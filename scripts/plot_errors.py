#!/usr/bin/env python3
"""
Compare estimated trajectories (SDK, LiDAR) against GT by nearest-timestamp matching.
Top row   : XY trajectory (full paths) | Shortest distance to GT over time
Bottom row: X error | Y error over time

Usage:
  python3 plot_errors.py \
      --gt    ~/ros2_ws/src/lidar_odometry/ground_truth/corridor_tum.txt \
      --sdk   ~/traj_data/v3_sdk.txt \
      --lidar ~/traj_data/v3_lidar.txt \
      [--lidar2 ~/traj_data/v2_lidar.txt]
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def load_poses(path: str) -> np.ndarray:
    """
    Returns (N, 4) array: [timestamp, x, y, z]
    Supports:
      CSV (header: index,time,trans_x,trans_y,trans_z,...) — gt_publisher CSV format
      TUM 8-col : timestamp tx ty tz qx qy qz qw
      stamped 4-col: timestamp x y z
      plain 3-col: x y z  (row index used as timestamp)
    Origin-normalizes xyz. Timestamps set to elapsed seconds from first.
    """
    import csv as _csv
    import os

    rows = []
    ext = os.path.splitext(path)[1].lower()

    if ext == '.csv':
        with open(path, newline='') as f:
            reader = _csv.DictReader(f)
            for row in reader:
                try:
                    rows.append([float(row['time']),
                                 float(row['trans_x']),
                                 float(row['trans_y']),
                                 float(row['trans_z'])])
                except (KeyError, ValueError):
                    continue
    else:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                v = line.split()
                n = len(v)
                if n >= 8:
                    rows.append([float(v[0]), float(v[1]), float(v[2]), float(v[3])])
                elif n >= 4:
                    rows.append([float(v[0]), float(v[1]), float(v[2]), float(v[3])])
                elif n >= 3:
                    rows.append([np.nan, float(v[0]), float(v[1]), float(v[2])])

    if not rows:
        return None
    arr = np.array(rows, dtype=np.float64)
    arr[:, 1:] -= arr[0, 1:]
    if np.isnan(arr[0, 0]):
        arr[:, 0] = np.arange(len(arr), dtype=np.float64)
    else:
        arr[:, 0] -= arr[0, 0]
    return arr


def smooth_z_ema(arr: np.ndarray, alpha: float = 0.1) -> np.ndarray:
    """EMA smoothing on the z column (index 3) of an (N,4) pose array."""
    out = arr.copy()
    for i in range(1, len(out)):
        out[i, 3] = alpha * arr[i, 3] + (1.0 - alpha) * out[i - 1, 3]
    return out


def apply_rotation_2d(poses: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Apply 2x2 rotation matrix R to the XY columns of poses (N,4)."""
    out = poses.copy()
    out[:, 1:3] = (R @ poses[:, 1:3].T).T
    return out


def match_to_gt(gt: np.ndarray, est: np.ndarray) -> np.ndarray:
    """
    For each pose in est, find the GT pose with the closest timestamp.
    Returns (M, 4): [t_est, ex, ey, ez]
    """
    gt_t = gt[:, 0]
    out = np.empty((len(est), 4))
    for i, row in enumerate(est):
        idx = np.argmin(np.abs(gt_t - row[0]))
        out[i, 0] = row[0]
        out[i, 1] = row[1] - gt[idx, 1]
        out[i, 2] = row[2] - gt[idx, 2]
        out[i, 3] = row[3] - gt[idx, 3]
    return out


def shortest_dist_to_gt(gt: np.ndarray, est: np.ndarray) -> np.ndarray:
    """
    For each pose in est, compute the minimum 2D Euclidean distance to any GT point.
    Returns (M, 2): [t_est, min_dist]
    """
    gt_xy = gt[:, 1:3]
    out = np.empty((len(est), 2))
    for i, row in enumerate(est):
        diffs = gt_xy - row[1:3]
        dists = np.sqrt((diffs ** 2).sum(axis=1))
        out[i, 0] = row[0]
        out[i, 1] = dists.min()
    return out


def rmse(err: np.ndarray) -> float:
    return float(np.sqrt(np.mean(err ** 2)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt',     required=True)
    parser.add_argument('--sdk',    default=None)
    parser.add_argument('--lidar',  default=None, help='LiDAR odometry (e.g. v3)')
    parser.add_argument('--lidar2', default=None, help='Additional LiDAR for comparison (e.g. v2)')
    parser.add_argument('--label_lidar',  default='ICP+EMA')
    parser.add_argument('--label_lidar2', default='Raw ICP')
    parser.add_argument('--no_align_sdk', action='store_true',
                        help='Disable SDK rotation alignment')
    parser.add_argument('--sdk_angle', type=float, default=90.0,
                        help='SDK rotation angle in degrees (default: 90)')
    parser.add_argument('--gt_z_alpha', type=float, default=0.1,
                        help='EMA alpha for GT z smoothing to remove gait oscillation (default: 0.1, 0=off)')
    parser.add_argument('-o', default='error_comparison.png')
    args = parser.parse_args()

    gt = load_poses(args.gt)
    if gt is None:
        print('[ERROR] GT file could not be loaded.')
        return
    print(f'[GT    ] poses={len(gt)}  elapsed={gt[-1,0]:.1f}s  z_range=[{gt[:,3].min():.3f}, {gt[:,3].max():.3f}]m')
    if args.gt_z_alpha > 0:
        gt = smooth_z_ema(gt, alpha=args.gt_z_alpha)
        print(f'[GT z  ] EMA smoothed  alpha={args.gt_z_alpha}  z_range=[{gt[:,3].min():.3f}, {gt[:,3].max():.3f}]m')

    sources = []
    if args.sdk:
        sources.append((args.sdk,    'GO1 Odom',         'blue',    '-'))
    if args.lidar:
        sources.append((args.lidar,  args.label_lidar,   'green',   '-'))
    if args.lidar2:
        sources.append((args.lidar2, args.label_lidar2,  'red',     '-'))

    if not sources:
        print('Specify at least one of --sdk / --lidar / --lidar2')
        return

    # ── layout: 2×3 ─────────────────────────────────────────────────────
    # Row 0: XY trajectory (wide, 2 cols) | 최단오차
    # Row 1: X error | Y error | Z error
    fig = plt.figure(figsize=(18, 11))
    gs = gridspec.GridSpec(2, 3, figure=fig, height_ratios=[1.3, 1], hspace=0.38, wspace=0.32)

    ax_xy = fig.add_subplot(gs[0, 0:2])  # XY trajectory (wide)
    ax_sd = fig.add_subplot(gs[0, 2])    # shortest distance to GT
    ax_ex = fig.add_subplot(gs[1, 0])    # X error
    ax_ey = fig.add_subplot(gs[1, 1])    # Y error
    ax_ez = fig.add_subplot(gs[1, 2])    # Z error

    gt_kw = dict(color='black', linestyle='--', linewidth=1.8, label='GT')
    ax_xy.plot(gt[:, 1], gt[:, 2], **gt_kw)

    print(f'\n{"Source":<14}  {"RMSE_X":>8}  {"RMSE_Y":>8}  {"RMSE_Z":>8}  {"RMSE_2D":>9}  {"Mean_SD":>9}')
    print('-' * 66)

    for path, label, color, ls in sources:
        est = load_poses(path)
        if est is None:
            print(f'[WARN] {path} — skipping')
            continue

        print(f'[{label:<12}] poses={len(est)}  elapsed={est[-1,0]:.1f}s  z_range=[{est[:,3].min():.3f}, {est[:,3].max():.3f}]m')

        if label == 'GO1 Odom' and not args.no_align_sdk:
            angle_rad = np.radians(args.sdk_angle)
            R_align = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                 [np.sin(angle_rad),  np.cos(angle_rad)]])
            est = apply_rotation_2d(est, R_align)
            print(f'[align_sdk] applied rotation +{args.sdk_angle:.0f}° to state_SDK')

        kw_traj = dict(color=color, linestyle=ls, label=label, linewidth=1.4, alpha=0.85)
        ax_xy.plot(est[:, 1], est[:, 2], **kw_traj)

        sd = shortest_dist_to_gt(gt, est)
        kw_sd = dict(color=color, linestyle=ls, label=label, linewidth=1.2, alpha=0.85)
        ax_sd.plot(sd[:, 0], sd[:, 1], **kw_sd)

        err = match_to_gt(gt, est)
        t = err[:, 0]
        ex, ey, ez = err[:, 1], err[:, 2], err[:, 3]

        rx, ry, rz = rmse(ex), rmse(ey), rmse(ez)
        r2d = float(np.sqrt(np.mean(ex**2 + ey**2)))
        mean_sd = float(sd[:, 1].mean())
        print(f'{label:<14}  {rx:>8.4f}  {ry:>8.4f}  {rz:>8.4f}  {r2d:>9.4f}  {mean_sd:>9.4f}  m')

        kw_err = dict(color=color, linestyle=ls, label=label, linewidth=1.2, alpha=0.85)
        ax_ex.plot(t, ex, **kw_err)
        ax_ey.plot(t, ey, **kw_err)
        ax_ez.plot(t, ez, **kw_err)

    print()

    ax_xy.set_title('XY Trajectory')
    ax_xy.set_xlabel('X (m)')
    ax_xy.set_ylabel('Y (m)')
    ax_xy.axis('equal')
    ax_xy.legend()
    ax_xy.grid(True)

    ax_sd.set_title('Shortest Distance to GT')
    ax_sd.set_xlabel('Time (s)')
    ax_sd.set_ylabel('Distance (m)')
    ax_sd.axhline(0, color='gray', linewidth=0.8, linestyle=':')
    ax_sd.legend()
    ax_sd.grid(True)

    for ax, title, yl in [
        (ax_ex, 'X Error (est - GT)', 'X err (m)'),
        (ax_ey, 'Y Error (est - GT)', 'Y err (m)'),
        (ax_ez, 'Z Error (est - GT)', 'Z err (m)'),
    ]:
        ax.set_title(title)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(yl)
        ax.axhline(0, color='gray', linewidth=0.8, linestyle=':')
        ax.legend()
        ax.grid(True)

    plt.suptitle('Trajectory Comparison & Position Error vs GT', fontsize=14, fontweight='bold')
    plt.savefig(args.o, dpi=150, bbox_inches='tight')
    print(f'Saved: {args.o}')

    plt.show()


if __name__ == '__main__':
    main()
