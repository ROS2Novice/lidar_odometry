#pragma once
#include "lidar_odometry/point_types.hpp"
#include <rclcpp/logger.hpp>
#include <vector>

class ScanContextManager
{
public:
  static constexpr int   N_R        = 20;    // rings  (distance bins)
  static constexpr int   N_S        = 60;    // sectors (angle bins)
  static constexpr float MAX_RADIUS = 30.0f; // meters (indoor: corridor/parking)

  struct Candidate {
    int   keyframe_id;
    float distance;   // 0 = identical, 1 = completely different
  };

  // Ground-removed cloud → N_R×N_S descriptor (row-major, max-z per cell)
  std::vector<float> compute(const CloudXYZ::Ptr & cloud) const;

  // Add descriptor to internal database
  void add(int keyframe_id, const std::vector<float> & desc);

  // Two-stage retrieval: ring-key prefilter → column-aligned distance
  // exclude_recent: skip last N keyframes (avoid comparing adjacent poses)
  std::vector<Candidate> findCandidates(const std::vector<float> & query,
                                        size_t           exclude_recent = 50,
                                        float            dist_thresh    = 0.15f,
                                        rclcpp::Logger * logger         = nullptr) const;

  size_t size() const { return database_.size(); }

private:
  std::vector<float> computeRingKey(const std::vector<float> & sc) const;

  // Rotation-invariant distance: minimum over all column shifts
  float columnAlignedDistance(const std::vector<float> & a,
                              const std::vector<float> & b) const;

  std::vector<std::vector<float>> database_;
  std::vector<std::vector<float>> ring_keys_;
  std::vector<int>                kf_ids_;
};
