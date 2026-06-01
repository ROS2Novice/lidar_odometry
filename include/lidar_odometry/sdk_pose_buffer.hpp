#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <vector>

struct PoseStamped {
  rclcpp::Time    stamp;
  Eigen::Matrix4f pose;
};

class SdkPoseBuffer
{
public:
  static Eigen::Matrix4f odomToMatrix(
    const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  void insert(const rclcpp::Time & stamp, const Eigen::Matrix4f & pose);

  // 버퍼 전체를 로컬 복사본으로 반환 (lock 1회)
  std::vector<PoseStamped> snapshot() const;

private:
  static constexpr double kMaxAgeSeconds = 1.0;
  mutable std::mutex      mutex_;
  std::deque<PoseStamped> buffer_;
};

// 버퍼 스냅샷에서 시각 t의 포즈를 보간 (translation: 선형, rotation: slerp)
bool interpolatePose(
  const std::vector<PoseStamped> & buf,
  const rclcpp::Time & t,
  Eigen::Matrix4f & result);
