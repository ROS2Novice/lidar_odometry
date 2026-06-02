#include "lidar_odometry/ground_removal.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <cmath>
#include <vector>

CloudXYZ::Ptr removeGround(const CloudXYZ::Ptr & cloud, rclcpp::Logger logger)
{
  if (cloud->size() < 50) return cloud;

  // 하위 20% z값 포인트를 ground 후보로 사용
  std::vector<float> zvals;
  zvals.reserve(cloud->size());
  for (const auto & pt : *cloud) zvals.push_back(pt.z);
  const size_t cutoff_idx = zvals.size() / 5;
  std::nth_element(zvals.begin(), zvals.begin() + cutoff_idx, zvals.end());
  const float z_cutoff = zvals[cutoff_idx];

  CloudXYZ::Ptr candidates(new CloudXYZ);
  candidates->reserve(cloud->size() / 5);
  for (const auto & pt : *cloud) {
    if (pt.z <= z_cutoff) candidates->push_back(pt);
  }

  if (candidates->size() < 10) return cloud;

  // RANSAC 수평면 fitting
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2f);
  seg.setMaxIterations(100);
  seg.setInputCloud(candidates);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coeff;
  seg.segment(inliers, coeff);

  if (inliers.indices.empty() || coeff.values.size() < 4) return cloud;

  // 평면 법선이 수직에 가까운지 확인 (cos > 0.8 → 수평면)
  const float nx = coeff.values[0], ny = coeff.values[1], nz = coeff.values[2];
  const float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (norm < 1e-6f || std::abs(nz) / norm < 0.8f) {
    RCLCPP_DEBUG(logger, "[ground] plane not horizontal, skipping");
    return cloud;
  }

  // 평면 방정식: ax + by + cz + d = 0
  const float a = coeff.values[0], b = coeff.values[1];
  const float c = coeff.values[2], d = coeff.values[3];
  const float inv_norm = 1.0f / norm;
  constexpr float kRemoveThresh = 0.3f;

  // 지면 인라이어 제거
  CloudXYZ::Ptr no_ground(new CloudXYZ);
  no_ground->reserve(cloud->size());
  size_t removed = 0;
  for (const auto & pt : *cloud) {
    // 후보 높이 이하 && 평면 거리 이내 → 지면
    if (pt.z <= z_cutoff &&
        std::abs(a * pt.x + b * pt.y + c * pt.z + d) * inv_norm < kRemoveThresh) {
      ++removed;
    } else {
      no_ground->push_back(pt);
    }
  }

  RCLCPP_INFO(logger, "[ground] removed=%zu  kept=%zu  (%.1f%%)",
    removed, no_ground->size(),
    100.0f * removed / static_cast<float>(cloud->size()));
  return no_ground;
}
