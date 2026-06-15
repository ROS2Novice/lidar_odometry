#include "lidar_odometry/icp_odometry.hpp"
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>

IcpOdometry::IcpOdometry(IcpConfig cfg) : cfg_(cfg) {}

// Radius crop → VoxelGrid → RandomSample 하드캡 순서로 맵 크기 보장
void IcpOdometry::trimMap(const Eigen::Vector3f & pos)
{
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

  if (static_cast<int>(local_map_->size()) > cfg_.local_map_max_points) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(local_map_);
    vg.setLeafSize(cfg_.local_map_leaf_size, cfg_.local_map_leaf_size, cfg_.local_map_leaf_size);
    CloudXYZ::Ptr filtered(new CloudXYZ);
    vg.filter(*filtered);
    local_map_ = filtered;
  }

  // sparse 환경에서 VoxelGrid 후에도 초과 시 RandomSample로 하드캡
  if (static_cast<int>(local_map_->size()) > cfg_.local_map_max_points) {
    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud(local_map_);
    rs.setSample(static_cast<unsigned int>(cfg_.local_map_max_points));
    CloudXYZ::Ptr sampled(new CloudXYZ);
    rs.filter(*sampled);
    local_map_ = sampled;
  }
}

bool IcpOdometry::update(
  const CloudXYZ::Ptr & cloud,
  const Eigen::Matrix4f & initial_guess,
  rclcpp::Logger logger)
{
  if (!local_map_) {
    local_map_.reset(new CloudXYZ);
    pcl::transformPointCloud(*cloud, *local_map_, initial_guess);
    current_pose_ = initial_guess;
    // RCLCPP_INFO(logger, "[VGICP] local map initialized  pts=%zu", local_map_->size());
    return false;
  }

  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
  vgicp.setNumThreads(cfg_.num_threads);
  vgicp.setResolution(cfg_.voxel_resolution);
  vgicp.setMaximumIterations(cfg_.max_iterations);
  vgicp.setMaxCorrespondenceDistance(cfg_.max_correspondence_distance);
  vgicp.setTransformationEpsilon(cfg_.transformation_epsilon);

  vgicp.setInputTarget(local_map_);
  vgicp.setInputSource(cloud);

  CloudXYZ aligned;
  vgicp.align(aligned, initial_guess);

  if (!vgicp.hasConverged()) {
    current_pose_ = initial_guess;
    RCLCPP_WARN(logger, "[VGICP] did not converge, falling back to SDK pose");

    // SDK 예측 포즈로 스캔을 맵에 추가 → 맵이 stale해지면 연쇄 실패 방지
    CloudXYZ::Ptr predicted_in_map(new CloudXYZ);
    pcl::transformPointCloud(*cloud, *predicted_in_map, initial_guess);
    *local_map_ += *predicted_in_map;
    trimMap(current_pose_.block<3, 1>(0, 3));
    return false;
  }

  last_score_ = vgicp.getFitnessScore();
  if (cfg_.fitness_score_threshold > 0.0f && last_score_ > cfg_.fitness_score_threshold) {
    RCLCPP_WARN(logger, "[VGICP] score=%.4f exceeds threshold=%.4f",
      last_score_, cfg_.fitness_score_threshold);
    return false;
  }

  current_pose_ = vgicp.getFinalTransformation();
  *local_map_ += aligned;
  trimMap(current_pose_.block<3, 1>(0, 3));

  // RCLCPP_INFO(logger, "[VGICP] score=%.6f  map=%zu pts", last_score_, local_map_->size());
  return true;
}
