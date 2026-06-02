#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <vector>

struct ImuStamped {
  rclcpp::Time      stamp;
  Eigen::Quaternionf orientation;
};

class ImuBuffer {
public:
  void insert(const rclcpp::Time & stamp, const Eigen::Quaternionf & q);
  std::vector<ImuStamped> snapshot() const;

private:
  static constexpr double kMaxAgeSeconds = 1.0;
  mutable std::mutex       mutex_;
  std::deque<ImuStamped>   buffer_;
};

bool interpolateImuOrientation(
  const std::vector<ImuStamped> & buf,
  const rclcpp::Time & t,
  Eigen::Quaternionf & result);
