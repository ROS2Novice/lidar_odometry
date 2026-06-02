#include "lidar_odometry/deskew.hpp"
#include <cmath>

CloudXYZ::Ptr deskewCloud(
  const CloudIRT::Ptr & cloud,
  const rclcpp::Time & scan_start,
  const SdkPoseBuffer & buffer,
  rclcpp::Logger logger,
  size_t & corrected_out)
{
  const auto buf = buffer.snapshot();

  Eigen::Matrix4f T_start;
  const bool has_start = interpolatePose(buf, scan_start, T_start);

  CloudXYZ::Ptr out(new CloudXYZ);
  out->reserve(cloud->size());
  corrected_out = 0;

  for (const auto & pt : *cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

    pcl::PointXYZ xyz;
    if (has_start) {
      const rclcpp::Time t_pt =
        scan_start + rclcpp::Duration::from_seconds(static_cast<double>(pt.time));
      Eigen::Matrix4f T_pt;
      if (interpolatePose(buf, t_pt, T_pt)) {
        const Eigen::Vector4f p_corr =
          (T_start.inverse() * T_pt) * Eigen::Vector4f(pt.x, pt.y, pt.z, 1.0f);
        xyz.x = p_corr.x();
        xyz.y = p_corr.y();
        xyz.z = p_corr.z();
        ++corrected_out;
      } else {
        xyz.x = pt.x; xyz.y = pt.y; xyz.z = pt.z;
      }
    } else {
      xyz.x = pt.x; xyz.y = pt.y; xyz.z = pt.z;
    }
    out->push_back(xyz);
  }

  RCLCPP_INFO(logger, "[deskew] total=%zu  corrected=%zu  skipped=%zu",
    out->size(), corrected_out, out->size() - corrected_out);
  return out;
}

CloudXYZ::Ptr deskewCloudImu(
  const CloudIRT::Ptr & cloud,
  const rclcpp::Time & scan_start,
  const ImuBuffer & imu_buffer,
  rclcpp::Logger logger,
  size_t & corrected_out)
{
  const auto buf = imu_buffer.snapshot();

  Eigen::Quaternionf q_start;
  const bool has_start = interpolateImuOrientation(buf, scan_start, q_start);

  CloudXYZ::Ptr out(new CloudXYZ);
  out->reserve(cloud->size());
  corrected_out = 0;

  for (const auto & pt : *cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

    pcl::PointXYZ xyz;
    if (has_start) {
      const rclcpp::Time t_pt =
        scan_start + rclcpp::Duration::from_seconds(static_cast<double>(pt.time));
      Eigen::Quaternionf q_pt;
      if (interpolateImuOrientation(buf, t_pt, q_pt)) {
        // 스캔 시작 기준 상대 회전으로 포인트 보정
        const Eigen::Vector3f p_corr =
          q_start.inverse() * q_pt * Eigen::Vector3f(pt.x, pt.y, pt.z);
        xyz.x = p_corr.x();
        xyz.y = p_corr.y();
        xyz.z = p_corr.z();
        ++corrected_out;
      } else {
        xyz.x = pt.x; xyz.y = pt.y; xyz.z = pt.z;
      }
    } else {
      xyz.x = pt.x; xyz.y = pt.y; xyz.z = pt.z;
    }
    out->push_back(xyz);
  }

  RCLCPP_INFO(logger, "[deskew_imu] total=%zu  corrected=%zu  skipped=%zu",
    out->size(), corrected_out, out->size() - corrected_out);
  return out;
}
