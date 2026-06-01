#pragma once
#include "lidar_odometry/point_types.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

struct IcpConfig {
  int   max_iterations              = 30;    // 100→30: 속도 우선
  float max_correspondence_distance = 2.0f;
  float transformation_epsilon      = 1e-4f; // 수렴 판정 완화 → 빠른 종료
  float voxel_leaf_size             = 0.4f;  // 0.2→0.4: 포인트 수 ~8배 감소
  float fitness_score_threshold     = 0.0f;
};

class IcpOdometry
{
public:
  explicit IcpOdometry(IcpConfig cfg = IcpConfig());

  // 새 스캔으로 포즈 업데이트. 첫 스캔 저장 시 false 반환, 수렴 시 true 반환
  bool update(
    const CloudXYZ::Ptr & cloud,
    const Eigen::Matrix4f & initial_guess,
    rclcpp::Logger logger);

  const Eigen::Matrix4f & currentPose() const { return current_pose_; }
  double lastFitnessScore() const { return last_score_; }

private:
  IcpConfig       cfg_;
  CloudXYZ::Ptr   prev_cloud_;
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  double          last_score_   = 0.0;
};
