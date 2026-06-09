#pragma once
#include "lidar_odometry/point_types.hpp"
#include <rclcpp/rclcpp.hpp>

struct GroundResult {
  CloudXYZ::Ptr no_ground;
  float a{0.0f}, b{0.0f}, c{0.0f}, d{0.0f};
  bool  plane_valid{false};
};

GroundResult removeGround(const CloudXYZ::Ptr & cloud, rclcpp::Logger logger);
