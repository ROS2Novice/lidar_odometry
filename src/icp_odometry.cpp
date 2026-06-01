#include "lidar_odometry/icp_odometry.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

IcpOdometry::IcpOdometry(IcpConfig cfg) : cfg_(cfg) {}

bool IcpOdometry::update(
  const CloudXYZ::Ptr & cloud,
  const Eigen::Matrix4f & initial_guess,
  rclcpp::Logger logger)
{
  // VoxelGrid 다운샘플링
  CloudXYZ::Ptr downsampled(new CloudXYZ);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(cfg_.voxel_leaf_size, cfg_.voxel_leaf_size, cfg_.voxel_leaf_size);
  voxel.filter(*downsampled);

  RCLCPP_INFO(logger, "[voxel] input=%zu  downsampled=%zu",
    cloud->size(), downsampled->size());

  if (!prev_cloud_) {
    prev_cloud_ = downsampled;
    RCLCPP_INFO(logger, "[ICP] first scan saved, waiting for next");
    return false;
  }

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setMaximumIterations(cfg_.max_iterations);
  gicp.setMaxCorrespondenceDistance(cfg_.max_correspondence_distance);
  gicp.setTransformationEpsilon(cfg_.transformation_epsilon);
  gicp.setCorrespondenceRandomness(10);  // covariance 계산 neighbor 수 20→10
  gicp.setInputSource(downsampled);
  gicp.setInputTarget(prev_cloud_);

  CloudXYZ aligned;
  gicp.align(aligned, initial_guess);

  if (!gicp.hasConverged()) {
    RCLCPP_WARN(logger, "[GICP] did not converge, pose not updated");
    prev_cloud_ = downsampled;  // reference는 최신으로 유지
    return false;
  }

  last_score_ = gicp.getFitnessScore();

  if (cfg_.fitness_score_threshold > 0.0f && last_score_ > cfg_.fitness_score_threshold) {
    RCLCPP_WARN(logger, "[GICP] score=%.4f exceeds threshold=%.4f, pose not updated",
      last_score_, cfg_.fitness_score_threshold);
    return false;
  }

  current_pose_ = current_pose_ * gicp.getFinalTransformation();
  prev_cloud_ = downsampled;  // 좋은 프레임에서만 reference 갱신

  RCLCPP_INFO(logger, "[GICP] converged  score=%.6f  iter=%d",
    last_score_, gicp.nr_iterations_);
  return true;
}
