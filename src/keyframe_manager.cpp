#include "lidar_odometry/keyframe_manager.hpp"
#include <fast_gicp/gicp/fast_vgicp.hpp>
#include <pcl/common/transforms.h>
#include <rclcpp/logging.hpp>
#include <cmath>
#include <optional>

KeyFrameManager::KeyFrameManager(float dist_thresh, float rot_thresh_deg)
: dist_thresh_(dist_thresh),
  rot_thresh_(rot_thresh_deg * M_PI / 180.0f)
{}

bool KeyFrameManager::tryAdd(const rclcpp::Time & stamp,
                             const Eigen::Matrix4f & pose,
                             const CloudXYZ::Ptr & cloud,
                             rclcpp::Logger logger)
{
  if (!shouldAdd(pose)) return false;

  const std::vector<float> desc = sc_manager_.compute(cloud);

  KeyFrame kf;
  kf.id           = next_id_++;
  kf.stamp        = stamp;
  kf.pose         = pose;
  kf.cloud        = cloud;
  kf.scan_context = desc;
  keyframes_.push_back(std::move(kf));

  sc_manager_.add(keyframes_.back().id, desc);

  RCLCPP_INFO(logger, "\033[1;36m<<<< [KEYFRAME] #%d  xyz=(%.2f, %.2f, %.2f)  total=%zu\033[0m",
    keyframes_.back().id,
    pose(0, 3), pose(1, 3), pose(2, 3),
    keyframes_.size());

  return true;
}

bool KeyFrameManager::shouldAdd(const Eigen::Matrix4f & pose) const
{
  if (keyframes_.empty()) return true;

  const Eigen::Matrix4f & prev = keyframes_.back().pose;
  const Eigen::Matrix4f   delta = prev.inverse() * pose;

  const float dist = delta.block<3, 1>(0, 3).norm();
  if (dist >= dist_thresh_) return true;

  // 회전각: delta 회전행렬 → 축각 변환
  const Eigen::AngleAxisf aa(Eigen::Matrix3f(delta.block<3, 3>(0, 0)));
  if (std::abs(aa.angle()) >= rot_thresh_) return true;

  return false;
}

std::vector<ScanContextManager::Candidate>
KeyFrameManager::findLoopCandidates(size_t exclude_recent, float dist_thresh,
                                     rclcpp::Logger * logger) const
{
  if (keyframes_.empty()) return {};
  return sc_manager_.findCandidates(
    keyframes_.back().scan_context, exclude_recent, dist_thresh, logger);
}

const KeyFrame * KeyFrameManager::findById(int id) const
{
  for (const auto & kf : keyframes_) {
    if (kf.id == id) return &kf;
  }
  return nullptr;
}

std::optional<KeyFrameManager::LoopEdge>
KeyFrameManager::verifyLoop(int kf_id_src, int kf_id_tgt,
                             rclcpp::Logger logger,
                             float score_thresh)
{
  const KeyFrame * src = findById(kf_id_src);
  const KeyFrame * tgt = findById(kf_id_tgt);
  if (!src || !tgt || !src->cloud || !tgt->cloud) return std::nullopt;

  // 오도메트리 기반 초기 추정 (tgt 좌표계에서 src 위치)
  const Eigen::Matrix4f init_guess = tgt->pose.inverse() * src->pose;

  fast_gicp::FastVGICP<pcl::PointXYZ, pcl::PointXYZ> vgicp;
  vgicp.setNumThreads(4);
  vgicp.setResolution(0.5f);
  vgicp.setMaximumIterations(100);
  vgicp.setMaxCorrespondenceDistance(5.0);  // 드리프트 보정을 위해 넓게
  vgicp.setTransformationEpsilon(1e-3);
  vgicp.setInputTarget(tgt->cloud);
  vgicp.setInputSource(src->cloud);

  // 오도메트리 초기 추정과 항등행렬 둘 다 시도, 더 낮은 score 채택
  CloudXYZ aligned_odom, aligned_identity;
  vgicp.align(aligned_odom, init_guess);
  const float score_odom = vgicp.hasConverged()
    ? static_cast<float>(vgicp.getFitnessScore()) : 1e9f;
  const Eigen::Matrix4f pose_odom = vgicp.getFinalTransformation();

  vgicp.align(aligned_identity, Eigen::Matrix4f::Identity());
  const float score_identity = vgicp.hasConverged()
    ? static_cast<float>(vgicp.getFitnessScore()) : 1e9f;
  const Eigen::Matrix4f pose_identity = vgicp.getFinalTransformation();

  const bool use_identity = (score_identity < score_odom);
  const float score = std::min(score_odom, score_identity);
  const Eigen::Matrix4f best_pose = use_identity ? pose_identity : pose_odom;
  (void)aligned_odom; (void)aligned_identity;

  if (score >= 1e8f) {
    // RCLCPP_INFO(logger, "[loop verify] kf#%d ↔ kf#%d  not converged",
    //   kf_id_src, kf_id_tgt);
    return std::nullopt;
  }

  if (score > score_thresh) {
    // RCLCPP_INFO(logger, "[loop verify] kf#%d ↔ kf#%d  score=%.4f > thresh=%.4f (rejected)",
    //   kf_id_src, kf_id_tgt, score, score_thresh);
    return std::nullopt;
  }

  const float icp_translation = best_pose.block<3, 1>(0, 3).norm();
  if (icp_translation > 3.0f) {
    // RCLCPP_INFO(logger,
    //   "[loop verify] kf#%d ↔ kf#%d  translation=%.2fm > 3.0m (rejected)",
    //   kf_id_src, kf_id_tgt, icp_translation);
    return std::nullopt;
  }

  LoopEdge edge;
  edge.kf_id_src     = kf_id_src;
  edge.kf_id_tgt     = kf_id_tgt;
  edge.relative_pose = best_pose;
  edge.fitness_score = score;
  return edge;
}

void KeyFrameManager::commitLoop(const LoopEdge & edge, rclcpp::Logger logger)
{
  loop_edges_.push_back(edge);
  RCLCPP_WARN(logger,
    "\033[1;32m<<<< [LOOP CONFIRMED] kf#%d ↔ kf#%d  score=%.4f  total_loops=%zu\033[0m",
    edge.kf_id_src, edge.kf_id_tgt, edge.fitness_score, loop_edges_.size());
}

void KeyFrameManager::updatePoses(const std::vector<Eigen::Matrix4f> & optimized_poses)
{
  const size_t n = std::min(optimized_poses.size(), keyframes_.size());
  for (size_t i = 0; i < n; ++i) {
    keyframes_[i].pose = optimized_poses[i];
  }
}
