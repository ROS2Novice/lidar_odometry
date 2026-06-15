#pragma once
#include "lidar_odometry/keyframe.hpp"
#include "lidar_odometry/scan_context.hpp"
#include <rclcpp/logger.hpp>
#include <optional>
#include <vector>

class KeyFrameManager
{
public:
  struct LoopEdge {
    int             kf_id_src;      // 최신 키프레임
    int             kf_id_tgt;      // 매칭된 과거 키프레임
    Eigen::Matrix4f relative_pose;  // tgt 좌표계에서 src 위치 (ICP 결과)
    float           fitness_score;
  };

  explicit KeyFrameManager(float dist_thresh = 1.0f, float rot_thresh_deg = 20.0f);

  bool tryAdd(const rclcpp::Time & stamp,
              const Eigen::Matrix4f & pose,
              const CloudXYZ::Ptr & cloud,
              rclcpp::Logger logger);

  // ScanContext 후보 검색
  std::vector<ScanContextManager::Candidate>
  findLoopCandidates(size_t           exclude_recent = 50,
                     float            dist_thresh    = 0.70f,
                     rclcpp::Logger * logger         = nullptr) const;

  // ICP 기하학적 검증 — 결과만 반환, 저장은 하지 않음
  std::optional<LoopEdge> verifyLoop(int kf_id_src, int kf_id_tgt,
                                     rclcpp::Logger logger,
                                     float score_thresh = 1.5f);

  // 검증된 루프 중 최선을 골라 포즈 그래프 엣지로 확정
  void commitLoop(const LoopEdge & edge, rclcpp::Logger logger);

  const std::vector<KeyFrame>  & keyframes()  const { return keyframes_; }
  const std::vector<LoopEdge>  & loopEdges()  const { return loop_edges_; }
  size_t size() const { return keyframes_.size(); }

  void updatePoses(const std::vector<Eigen::Matrix4f> & optimized_poses);

private:
  bool shouldAdd(const Eigen::Matrix4f & pose) const;
  const KeyFrame * findById(int id) const;

  std::vector<KeyFrame>  keyframes_;
  std::vector<LoopEdge>  loop_edges_;
  ScanContextManager     sc_manager_;
  float dist_thresh_;
  float rot_thresh_;
  int   next_id_{0};
};
