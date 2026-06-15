#pragma once
#include "lidar_odometry/point_types.hpp"
#include <Eigen/Core>
#include <rclcpp/time.hpp>

struct KeyFrame
{
  int              id;
  rclcpp::Time     stamp;
  Eigen::Matrix4f  pose;   // global pose — updated by pose graph optimizer
  CloudXYZ::Ptr    cloud;  // ground-removed scan

  std::vector<float> scan_context;    // ScanContext descriptor (N_R×N_S, row-major)
  // int graph_node_id{-1};            // g2o / GTSAM vertex id (future)
};
