#include "lidar_odometry/icp_odometry.hpp"
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

IcpOdometry::IcpOdometry(IcpConfig cfg) : cfg_(cfg) {}

bool IcpOdometry::update(
  const CloudXYZ::Ptr & cloud,
  const Eigen::Matrix4f & initial_guess,
  rclcpp::Logger logger)
{
  if (!local_map_) {
    // 첫 스캔: initial_guess로 odom 프레임에 변환해서 local map 초기화
    local_map_.reset(new CloudXYZ);
    pcl::transformPointCloud(*cloud, *local_map_, initial_guess);
    current_pose_ = initial_guess;
    RCLCPP_INFO(logger, "[VGICP] local map initialized  pts=%zu", local_map_->size());
    return false;
  }

  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
  vgicp.setNumThreads(cfg_.num_threads);
  vgicp.setResolution(cfg_.voxel_resolution);
  vgicp.setMaximumIterations(cfg_.max_iterations);
  vgicp.setMaxCorrespondenceDistance(cfg_.max_correspondence_distance);
  vgicp.setTransformationEpsilon(cfg_.transformation_epsilon);

  // target: odom 프레임의 local map  /  source: 센서 프레임의 현재 스캔
  vgicp.setInputTarget(local_map_);
  vgicp.setInputSource(cloud);

  CloudXYZ aligned;
  vgicp.align(aligned, initial_guess);   // initial_guess = 예측 절대 포즈

  if (!vgicp.hasConverged()) {
    // SDK initial guess로 포즈 유지 → 다음 프레임 initial guess가 틀어지지 않음
    current_pose_ = initial_guess;
    RCLCPP_WARN(logger, "[VGICP] did not converge, falling back to SDK pose");
    return false;
  }

  last_score_ = vgicp.getFitnessScore();
  if (cfg_.fitness_score_threshold > 0.0f && last_score_ > cfg_.fitness_score_threshold) {
    RCLCPP_WARN(logger, "[VGICP] score=%.4f exceeds threshold=%.4f",
      last_score_, cfg_.fitness_score_threshold);
    return false;
  }

  // getFinalTransformation() = 센서→odom 절대 변환
  current_pose_ = vgicp.getFinalTransformation();

  // aligned 스캔(odom 프레임)을 local map에 추가
  *local_map_ += aligned;

  // 현재 위치 기준 반경 crop — 멀리 떨어진 포인트 제거
  {
    const Eigen::Vector3f pos = current_pose_.block<3, 1>(0, 3);
    const float r2 = cfg_.local_map_radius * cfg_.local_map_radius;
    CloudXYZ::Ptr in_range(new CloudXYZ);
    in_range->reserve(local_map_->size());
    for (const auto & pt : *local_map_) {
      const float d2 = (pt.x - pos.x()) * (pt.x - pos.x()) +
                       (pt.y - pos.y()) * (pt.y - pos.y()) +
                       (pt.z - pos.z()) * (pt.z - pos.z());
      if (d2 <= r2) in_range->push_back(pt);
    }
    local_map_ = in_range;
  }

  // 반경 crop 후에도 포인트 수 초과 시 추가 다운샘플링
  if (static_cast<int>(local_map_->size()) > cfg_.local_map_max_points) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(local_map_);
    vg.setLeafSize(
      cfg_.local_map_leaf_size,
      cfg_.local_map_leaf_size,
      cfg_.local_map_leaf_size);
    CloudXYZ::Ptr filtered(new CloudXYZ);
    vg.filter(*filtered);
    local_map_ = filtered;
  }

  RCLCPP_INFO(logger, "[VGICP] score=%.6f  map=%zu pts", last_score_, local_map_->size());
  return true;
}
