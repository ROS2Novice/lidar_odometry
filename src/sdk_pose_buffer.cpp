#include "lidar_odometry/sdk_pose_buffer.hpp"

Eigen::Matrix4f SdkPoseBuffer::odomToMatrix(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const auto & q = msg->pose.pose.orientation;
  Eigen::Quaternionf quat(
    static_cast<float>(q.w), static_cast<float>(q.x),
    static_cast<float>(q.y), static_cast<float>(q.z));
  quat.normalize();

  const auto & p = msg->pose.pose.position;
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = quat.toRotationMatrix();
  T(0, 3) = static_cast<float>(p.x);
  T(1, 3) = static_cast<float>(p.y);
  T(2, 3) = static_cast<float>(p.z);
  return T;
}

void SdkPoseBuffer::insert(const rclcpp::Time & stamp, const Eigen::Matrix4f & pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.push_back({stamp, pose});
  while (buffer_.size() > 1 &&
         (buffer_.back().stamp - buffer_.front().stamp).seconds() > kMaxAgeSeconds) {
    buffer_.pop_front();
  }
}

std::vector<PoseStamped> SdkPoseBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<PoseStamped>(buffer_.begin(), buffer_.end());
}

bool interpolatePose(
  const std::vector<PoseStamped> & buf,
  const rclcpp::Time & t,
  Eigen::Matrix4f & result)
{
  if (buf.size() < 2) return false;

  // 범위 클램핑
  if (t <= buf.front().stamp) { result = buf.front().pose; return true; }
  if (t >= buf.back().stamp)  { result = buf.back().pose;  return true; }

  for (size_t i = 1; i < buf.size(); ++i) {
    if (buf[i - 1].stamp <= t && t <= buf[i].stamp) {
      const double dt    = (buf[i].stamp - buf[i - 1].stamp).seconds();
      const float  alpha = static_cast<float>((t - buf[i - 1].stamp).seconds() / dt);

      const Eigen::Vector3f trans =
        buf[i - 1].pose.block<3, 1>(0, 3) * (1.0f - alpha) +
        buf[i].pose.block<3, 1>(0, 3) * alpha;

      Eigen::Quaternionf q0(buf[i - 1].pose.block<3, 3>(0, 0));
      Eigen::Quaternionf q1(buf[i].pose.block<3, 3>(0, 0));

      result = Eigen::Matrix4f::Identity();
      result.block<3, 3>(0, 0) = q0.slerp(alpha, q1).toRotationMatrix();
      result.block<3, 1>(0, 3) = trans;
      return true;
    }
  }
  return false;
}
