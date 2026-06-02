#include "lidar_odometry/imu_buffer.hpp"
#include <algorithm>

void ImuBuffer::insert(const rclcpp::Time & stamp, const Eigen::Quaternionf & q)
{
  std::lock_guard<std::mutex> lock(mutex_);
  buffer_.push_back({stamp, q});
  while (!buffer_.empty() &&
    (stamp - buffer_.front().stamp).seconds() > kMaxAgeSeconds)
  {
    buffer_.pop_front();
  }
}

std::vector<ImuStamped> ImuBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {buffer_.begin(), buffer_.end()};
}

bool interpolateImuOrientation(
  const std::vector<ImuStamped> & buf,
  const rclcpp::Time & t,
  Eigen::Quaternionf & result)
{
  if (buf.empty()) return false;

  // 범위 클램프
  if (t <= buf.front().stamp) { result = buf.front().orientation; return true; }
  if (t >= buf.back().stamp)  { result = buf.back().orientation;  return true; }

  // 이진탐색으로 브라켓 찾기
  auto it = std::lower_bound(buf.begin(), buf.end(), t,
    [](const ImuStamped & a, const rclcpp::Time & b){ return a.stamp < b; });

  const auto & hi = *it;
  const auto & lo = *(it - 1);
  const double dt = (hi.stamp - lo.stamp).seconds();
  if (dt <= 0.0) { result = lo.orientation; return true; }

  const float alpha = static_cast<float>((t - lo.stamp).seconds() / dt);
  result = lo.orientation.slerp(alpha, hi.orientation);
  return true;
}
