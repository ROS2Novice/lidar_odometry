#pragma once
#include "lidar_odometry/point_types.hpp"
#include "lidar_odometry/sdk_pose_buffer.hpp"
#include "lidar_odometry/imu_buffer.hpp"
#include <rclcpp/rclcpp.hpp>

// SDK 포즈 기반 deskewing (rotation + translation)
CloudXYZ::Ptr deskewCloud(
  const CloudIRT::Ptr & cloud,
  const rclcpp::Time & scan_start,
  const SdkPoseBuffer & buffer,
  rclcpp::Logger logger,
  size_t & corrected_out);

// IMU orientation 기반 deskewing (rotation 전용, 더 정밀)
CloudXYZ::Ptr deskewCloudImu(
  const CloudIRT::Ptr & cloud,
  const rclcpp::Time & scan_start,
  const ImuBuffer & imu_buffer,
  rclcpp::Logger logger,
  size_t & corrected_out);
