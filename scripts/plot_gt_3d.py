#!/usr/bin/env python3
"""
GT 궤적 다각도 시각화 (XY / XZ / YZ / Z-time)
Usage:
  python3 plot_gt_3d.py --gt corridor_tum.txt [--gt2 other.txt]
"""
import sys
import os
import argparse
import csv as _csv
import numpy as np
import matplotlib.pyplot as plt


def load_poses(path: str) -> np.ndarray:
    """Returns (N, 4): [t, x, y, z], origin-normalized."""
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
                if len(v) >= 4:
                    rows.append([float(v[0]), float(v[1]), float(v[2]), float(v[3])])

    if not rows:
        return None
    arr = np.array(rows, dtype=np.float64)
    arr[:, 1:] -= arr[0, 1:]
    arr[:, 0]  -= arr[0, 0]
    return arr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt',    required=True)
    parser.add_argument('--gt2',   default=None)
    parser.add_argument('--gt3',   default=None)
    parser.add_argument('--label',  default='GT')
    parser.add_argument('--label2', default=None)
    parser.add_argument('--label3', default=None)
    parser.add_argument('--angle2', type=float, default=0.0, help='gt2 XY 회전각 (도)')
    parser.add_argument('--angle3', type=float, default=0.0, help='gt3 XY 회전각 (도)')
    parser.add_argument('-o', default='gt_views.png')
    args = parser.parse_args()

    files = [(args.gt,  args.label,
              'black', '-',  0.0)]
    if args.gt2:
        files.append((args.gt2,
                      args.label2 or os.path.basename(args.gt2),
                      'blue',  '--', args.angle2))
    if args.gt3:
        files.append((args.gt3,
                      args.label3 or os.path.basename(args.gt3),
                      'red',   ':',  args.angle3))

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ax_xy  = axes[0, 0]
    ax_xz  = axes[0, 1]
    ax_yz  = axes[1, 0]
    ax_zt  = axes[1, 1]

    for path, label, color, ls, angle in files:
        arr = load_poses(path)
        if arr is None:
            print(f'[WARN] 로드 실패: {path}')
            continue

        if angle != 0.0:
            rad = np.radians(angle)
            R = np.array([[np.cos(rad), -np.sin(rad)],
                          [np.sin(rad),  np.cos(rad)]])
            arr[:, 1:3] = (R @ arr[:, 1:3].T).T

        t, x, y, z = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        dist = np.cumsum(np.sqrt(dx**2 + dy**2))
        print(f'[{label}]  poses={len(arr)}  elapsed={t[-1]:.1f}s  '
              f'dist={dist[-1]:.1f}m  z=[{z.min():.4f}, {z.max():.4f}]m')

        kw = dict(color=color, linestyle=ls, linewidth=1.4, label=label, alpha=0.9)
        ax_xy.plot(x, y,    **kw)
        ax_xz.plot(x, z,    **kw)
        ax_yz.plot(y, z,    **kw)
        ax_zt.plot(dist, z, **kw)

    for ax, title, xl, yl in [
        (ax_xy, 'Top View (XY)',   'X (m)', 'Y (m)'),
        (ax_xz, 'Front View (XZ)', 'X (m)', 'Z (m)'),
        (ax_yz, 'Side View (YZ)',  'Y (m)', 'Z (m)'),
        (ax_zt, 'Z over Distance', 'Distance (m)', 'Z (m)'),
    ]:
        ax.set_title(title)
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.legend()
        ax.grid(True)
        if yl == 'Z (m)':
            ax.axhline(0, color='gray', linewidth=0.8, linestyle=':')

    ax_xy.axis('equal')
    for ax in [ax_xz, ax_yz, ax_zt]:
        ax.set_ylim(-5, 5)
    fig.suptitle('GT Trajectory — Multi-View Analysis', fontsize=13, fontweight='bold')
    plt.tight_layout()
    plt.savefig(args.o, dpi=150, bbox_inches='tight')
    print(f'Saved: {args.o}')
    plt.show()


if __name__ == '__main__':
    main()
