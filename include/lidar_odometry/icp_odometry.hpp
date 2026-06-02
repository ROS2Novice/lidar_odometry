#pragma once
#include "lidar_odometry/point_types.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

struct IcpConfig {
  int   max_iterations              = 30;
  float max_correspondence_distance = 2.0f;
  float transformation_epsilon      = 1e-4f;
  float voxel_resolution            = 0.4f;
  float fitness_score_threshold     = 0.0f;
  int   num_threads                 = 4;

  // Local map 설정
  float local_map_radius            = 40.0f;   // 현재 위치 기준 보존 반경 (m)
  float local_map_leaf_size         = 0.3f;    // 반경 crop 후 추가 다운샘플링 leaf size
  int   local_map_max_points        = 30000;   // 이 수 초과 시 다운샘플링
};

class IcpOdometry
{
public:
  explicit IcpOdometry(IcpConfig cfg = IcpConfig());

  // initial_guess: 예측 절대 포즈 (센서 → odom frame)
  bool update(
    const CloudXYZ::Ptr & cloud,
    const Eigen::Matrix4f & initial_guess,
    rclcpp::Logger logger);

  const Eigen::Matrix4f & currentPose() const { return current_pose_; }
  double lastFitnessScore() const { return last_score_; }

private:
  IcpConfig       cfg_;
  CloudXYZ::Ptr   local_map_;
  Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
  double          last_score_   = 0.0;
};
