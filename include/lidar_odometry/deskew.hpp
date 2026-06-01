#pragma once
#include "lidar_odometry/point_types.hpp"
#include "lidar_odometry/sdk_pose_buffer.hpp"
#include <rclcpp/rclcpp.hpp>

// 포인트클라우드의 모션 왜곡을 스캔 시작 시점 기준으로 보정
CloudXYZ::Ptr deskewCloud(
  const CloudIRT::Ptr & cloud,
  const rclcpp::Time & scan_start,
  const SdkPoseBuffer & buffer,
  rclcpp::Logger logger,
  size_t & corrected_out);
