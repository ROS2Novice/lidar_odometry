#!/usr/bin/env python3
import os
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GtPublisher(Node):
    def __init__(self):
        super().__init__('gt_publisher')
        self.declare_parameter('file', '')
        self.declare_parameter('frame_id', 'sdk_odom')

        file_path = self.get_parameter('file').get_parameter_value().string_value
        frame_id  = self.get_parameter('frame_id').get_parameter_value().string_value

        if not file_path:
            self.get_logger().error('Use: --ros-args -p file:=<path_to_gt_file>')
            return

        if not os.path.exists(file_path):
            self.get_logger().error(f'File not found: {file_path}')
            return

        # TRANSIENT_LOCAL: RViz가 나중에 연결해도 path 수신 가능
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(Path, '/gt_path', qos)

        path_msg = self._load_path(file_path, frame_id)
        if path_msg is not None:
            self.pub.publish(path_msg)
            self.get_logger().info(
                f'Published /gt_path  poses={len(path_msg.poses)}  '
                f'file={os.path.basename(file_path)}'
            )

    # ------------------------------------------------------------------
    def _load_path(self, file_path: str, frame_id: str) -> Path:
        ext = os.path.splitext(file_path)[1].lower()
        if ext == '.csv':
            poses = self._load_csv(file_path)
        else:
            poses = self._load_tum(file_path)

        if not poses:
            self.get_logger().error('No poses loaded — check file format')
            return None

        # 원점 정규화 (첫 포즈를 0,0,0 기준으로)
        x0, y0, z0 = poses[0][0], poses[0][1], poses[0][2]

        now = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp    = now
        path_msg.header.frame_id = frame_id

        for (tx, ty, tz, qx, qy, qz, qw) in poses:
            ps = PoseStamped()
            ps.header.stamp    = now
            ps.header.frame_id = frame_id
            ps.pose.position.x = tx - x0
            ps.pose.position.y = ty - y0
            ps.pose.position.z = tz - z0
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)

        return path_msg

    # ------------------------------------------------------------------
    def _load_tum(self, file_path: str):
        # TUM format: timestamp tx ty tz qx qy qz qw
        poses = []
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                v = line.split()
                if len(v) < 8:
                    continue
                poses.append((
                    float(v[1]), float(v[2]), float(v[3]),   # tx ty tz
                    float(v[4]), float(v[5]), float(v[6]), float(v[7]),  # qx qy qz qw
                ))
        return poses

    def _load_csv(self, file_path: str):
        # CSV format: index, time, trans_x, trans_y, trans_z, roll, pitch, yaw
        poses = []
        with open(file_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                tx    = float(row['trans_x'])
                ty    = float(row['trans_y'])
                tz    = float(row['trans_z'])
                roll  = float(row['roll'])
                pitch = float(row['pitch'])
                yaw   = float(row['yaw'])
                qx, qy, qz, qw = self._euler_to_quat(roll, pitch, yaw)
                poses.append((tx, ty, tz, qx, qy, qz, qw))
        return poses

    @staticmethod
    def _euler_to_quat(roll: float, pitch: float, yaw: float):
        cr, sr = np.cos(roll  / 2), np.sin(roll  / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw   / 2), np.sin(yaw   / 2)
        qw =  cr * cp * cy + sr * sp * sy
        qx =  sr * cp * cy - cr * sp * sy
        qy =  cr * sp * cy + sr * cp * sy
        qz =  cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main():
    rclpy.init()
    node = GtPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
