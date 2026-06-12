#!/usr/bin/env python3
"""
txt 파일로 저장된 궤적을 pyplot으로 비교.

사용법:
  # GT + SDK + lidar 비교
  python3 plot_trajectories.py \
      --gt   ~/traj_data/gt.txt \
      --sdk  ~/traj_data/v3_sdk.txt \
      --lidar ~/traj_data/v3_lidar.txt

  # ver2 lidar 추가 비교
  python3 plot_trajectories.py \
      --gt    ~/traj_data/gt.txt \
      --sdk   ~/traj_data/v3_sdk.txt \
      --lidar ~/traj_data/v3_lidar.txt \
      --lidar2 ~/traj_data/v2_lidar.txt

파일 포맷:
  x y z  (한 줄에 공백 구분, # 로 시작하는 줄은 주석)
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_txt(path: str):
    """
    Returns (timestamps_sec, xs, ys, zs) — all origin-normalized.
    Supported formats:
      TUM (8 col): timestamp tx ty tz qx qy qz qw
      stamped xyz (4 col): timestamp x y z
      plain xyz (3 col): x y z  (timestamp = row index)
    """
    rows = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = line.split()
            n = len(vals)
            if n >= 8:
                rows.append([float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])])
            elif n >= 4:
                rows.append([float(vals[0]), float(vals[1]), float(vals[2]), float(vals[3])])
            elif n >= 3:
                rows.append([0.0, float(vals[0]), float(vals[1]), float(vals[2])])
    if not rows:
        return None
    arr = np.array(rows)          # (N, 4): t x y z
    arr[:, 1:] -= arr[0, 1:]      # 원점 정규화 (xyz만)
    arr[:, 0]  -= arr[0, 0]       # 시간 0초부터
    return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt',     default=None)
    parser.add_argument('--sdk',    default=None)
    parser.add_argument('--lidar',  default=None, help='LiDAR odometry (v3 등)')
    parser.add_argument('--lidar2', default=None, help='비교용 LiDAR odometry (v2 등)')
    parser.add_argument('--label_lidar',  default='LiDAR v3')
    parser.add_argument('--label_lidar2', default='LiDAR v2')
    parser.add_argument('-o', default='comparison.png')
    args = parser.parse_args()

    sources = []
    if args.gt:
        sources.append((args.gt,    'GT',               'black',  '-'))
    if args.sdk:
        sources.append((args.sdk,   'state_SDK',        'blue',   '--'))
    if args.lidar:
        sources.append((args.lidar,  args.label_lidar,  'red',    '-'))
    if args.lidar2:
        sources.append((args.lidar2, args.label_lidar2, 'orange', '--'))

    if not sources:
        print('Specify at least one file.')
        return

    datasets = []
    for path, label, color, ls in sources:
        result = load_txt(path)
        if result is None:
            print(f'[WARN] {path} not found or empty — skipping')
            continue
        datasets.append((label, color, ls, *result))

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ax_xy, ax_z = axes[0, 0], axes[0, 1]
    ax_x,  ax_y = axes[1, 0], axes[1, 1]

    for label, color, ls, ts, xs, ys, zs in datasets:
        kw = dict(color=color, linestyle=ls, label=label, linewidth=1.5)
        ax_xy.plot(xs, ys, **kw)
        ax_z.plot(ts, zs, **kw)
        ax_x.plot(ts, xs, **kw)
        ax_y.plot(ts, ys, **kw)

    for ax, title, xl, yl in [
        (ax_xy, 'XY Trajectory', 'X (m)',   'Y (m)'),
        (ax_z,  'Z over time',   'Time (s)', 'Z (m)'),
        (ax_x,  'X over time',   'Time (s)', 'X (m)'),
        (ax_y,  'Y over time',   'Time (s)', 'Y (m)'),
    ]:
        ax.set_title(title); ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.legend(); ax.grid(True)
    ax_xy.axis('equal')

    plt.suptitle('Trajectory Comparison', fontsize=14, fontweight='bold', y=1.01)
    plt.tight_layout()
    plt.savefig(args.o, dpi=150)
    print(f'Saved: {args.o}')
    plt.show()


if __name__ == '__main__':
    main()
