#!/usr/bin/env python3
"""
/state_SDK, /lidar_odom, /lidar_odom_v2 를 실시간으로 txt 파일에 저장.

사용법:
  python3 save_trajectories.py --out_dir ~/traj_data --prefix corridor
  # → ~/traj_data/corridor_sdk.txt
  #   ~/traj_data/corridor_lidar.txt
  #   ~/traj_data/corridor_lidar_v2.txt  (lidar_odometry_ver2 실행 중일 때만)

파일 포맷 (공백 구분):
  timestamp  x  y  z
"""
import os
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class TrajSaver(Node):
    def __init__(self, out_dir: str, prefix: str, v2_only: bool):
        super().__init__('traj_saver')
        os.makedirs(out_dir, exist_ok=True)

        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._sdk_f      = None
        self._lidar_f    = None
        self._lidar_v2_f = None

        if v2_only:
            lidar_v2_path = os.path.join(out_dir, f'{prefix}_lidar_v2.txt')
            self._lidar_v2_f = open(lidar_v2_path, 'w')
            self._lidar_v2_f.write('# timestamp x y z\n')
            self.create_subscription(Odometry, '/lidar_odom_v2', self._lidar_v2_cb, qos)
            self.get_logger().info(f'저장 중: {lidar_v2_path}')
        else:
            sdk_path   = os.path.join(out_dir, f'{prefix}_sdk.txt')
            lidar_path = os.path.join(out_dir, f'{prefix}_lidar.txt')
            self._sdk_f   = open(sdk_path,   'w')
            self._lidar_f = open(lidar_path, 'w')
            self._sdk_f.write('# timestamp x y z\n')
            self._lidar_f.write('# timestamp x y z\n')
            self.create_subscription(Odometry, '/state_SDK',  self._sdk_cb,   qos)
            self.create_subscription(Odometry, '/lidar_odom', self._lidar_cb, qos)
            self.get_logger().info(f'저장 중: {sdk_path}')
            self.get_logger().info(f'저장 중: {lidar_path}')

    def _sdk_cb(self, msg: Odometry):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        self._sdk_f.write(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f}\n')
        self._sdk_f.flush()

    def _lidar_cb(self, msg: Odometry):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        self._lidar_f.write(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f}\n')
        self._lidar_f.flush()

    def _lidar_v2_cb(self, msg: Odometry):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        self._lidar_v2_f.write(f'{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f}\n')
        self._lidar_v2_f.flush()

    def destroy_node(self):
        if self._sdk_f:
            self._sdk_f.close()
        if self._lidar_f:
            self._lidar_f.close()
        if self._lidar_v2_f:
            self._lidar_v2_f.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out_dir',  default=os.path.expanduser('~/traj_data'))
    parser.add_argument('--prefix',   default='run')
    parser.add_argument('--v2_only',  action='store_true',
                        help='ver2 노드만 실행 중일 때: lidar_odom_v2만 저장')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TrajSaver(args.out_dir, args.prefix, args.v2_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
