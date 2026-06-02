#pragma once
#include "lidar_odometry/point_types.hpp"
#include <rclcpp/rclcpp.hpp>

CloudXYZ::Ptr removeGround(const CloudXYZ::Ptr & cloud, rclcpp::Logger logger);
