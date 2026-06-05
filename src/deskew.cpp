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
  const SdkPoseBuffer & sdk_buffer,
  rclcpp::Logger logger,
  size_t & corrected_out)
{
  const auto imu_buf = imu_buffer.snapshot();
  const auto sdk_buf = sdk_buffer.snapshot();

  Eigen::Quaternionf q_start;
  const bool has_imu_start = interpolateImuOrientation(imu_buf, scan_start, q_start);

  Eigen::Matrix4f T_start;
  const bool has_sdk_start = interpolatePose(sdk_buf, scan_start, T_start);

  CloudXYZ::Ptr out(new CloudXYZ);
  out->reserve(cloud->size());
  corrected_out = 0;

  for (const auto & pt : *cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

    pcl::PointXYZ xyz;
    if (has_imu_start && has_sdk_start) {
      const rclcpp::Time t_pt =
        scan_start + rclcpp::Duration::from_seconds(static_cast<double>(pt.time));
      Eigen::Quaternionf q_pt;
      Eigen::Matrix4f T_pt;
      if (interpolateImuOrientation(imu_buf, t_pt, q_pt) &&
          interpolatePose(sdk_buf, t_pt, T_pt)) {
        // 회전 보정: IMU 기반 (정밀)
        const Eigen::Matrix3f R_rel = (q_start.inverse() * q_pt).toRotationMatrix();
        // 이동 보정: SDK 기반, world frame 변위를 스캔 시작 시점 body frame으로 변환
        const Eigen::Vector3f t_rel =
          q_start.inverse() * (T_pt.block<3, 1>(0, 3) - T_start.block<3, 1>(0, 3));

        const Eigen::Vector3f p_corr =
          R_rel * Eigen::Vector3f(pt.x, pt.y, pt.z) + t_rel;
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
