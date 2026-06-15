#include "lidar_odometry/scan_context.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

std::vector<float> ScanContextManager::compute(const CloudXYZ::Ptr & cloud) const
{
  const float dr = MAX_RADIUS / N_R;
  const float ds = 2.0f * M_PI / N_S;

  std::vector<float> sc(N_R * N_S, -std::numeric_limits<float>::max());

  for (const auto & pt : cloud->points) {
    const float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (r < 1e-3f || r >= MAX_RADIUS) continue;

    const int ring = std::min(static_cast<int>(r / dr), N_R - 1);

    float angle = std::atan2(pt.y, pt.x);
    if (angle < 0.0f) angle += 2.0f * static_cast<float>(M_PI);
    const int sector = std::min(static_cast<int>(angle / ds), N_S - 1);

    float & cell = sc[ring * N_S + sector];
    if (pt.z > cell) cell = pt.z;
  }

  // Cells with no points → 0
  for (float & v : sc) {
    if (v == -std::numeric_limits<float>::max()) v = 0.0f;
  }

  return sc;
}

std::vector<float> ScanContextManager::computeRingKey(const std::vector<float> & sc) const
{
  std::vector<float> rk(N_R, 0.0f);
  for (int r = 0; r < N_R; ++r) {
    float sum = 0.0f;
    for (int s = 0; s < N_S; ++s) sum += sc[r * N_S + s];
    rk[r] = sum / N_S;
  }
  return rk;
}

float ScanContextManager::columnAlignedDistance(const std::vector<float> & a,
                                                 const std::vector<float> & b) const
{
  float min_dist = std::numeric_limits<float>::max();

  for (int shift = 0; shift < N_S; ++shift) {
    float dist = 0.0f;
    for (int r = 0; r < N_R; ++r) {
      float dot = 0.0f, na = 0.0f, nb = 0.0f;
      for (int s = 0; s < N_S; ++s) {
        const float av = a[r * N_S + s];
        const float bv = b[r * N_S + ((s + shift) % N_S)];
        dot += av * bv;
        na  += av * av;
        nb  += bv * bv;
      }
      const float denom   = std::sqrt(na * nb);
      const float cos_sim = (denom > 1e-6f) ? (dot / denom) : 0.0f;
      dist += 1.0f - cos_sim;
    }
    dist /= static_cast<float>(N_R);
    if (dist < min_dist) min_dist = dist;
  }

  return min_dist;
}

void ScanContextManager::add(int keyframe_id, const std::vector<float> & desc)
{
  database_.push_back(desc);
  ring_keys_.push_back(computeRingKey(desc));
  kf_ids_.push_back(keyframe_id);
}

std::vector<ScanContextManager::Candidate>
ScanContextManager::findCandidates(const std::vector<float> & query,
                                    size_t exclude_recent,
                                    float  dist_thresh,
                                    rclcpp::Logger * logger) const
{
  if (database_.size() <= exclude_recent) return {};

  const std::vector<float> qrk = computeRingKey(query);
  const size_t n = database_.size() - exclude_recent;

  // Stage 1: ring-key L2 distance → top-10 candidates
  std::vector<std::pair<float, size_t>> rk_dists;
  rk_dists.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    float d = 0.0f;
    for (int r = 0; r < N_R; ++r) {
      const float diff = qrk[r] - ring_keys_[i][r];
      d += diff * diff;
    }
    rk_dists.emplace_back(std::sqrt(d), i);
  }

  const size_t top_k = std::min(size_t(10), rk_dists.size());
  std::partial_sort(rk_dists.begin(), rk_dists.begin() + top_k, rk_dists.end());

  // Stage 2: full rotation-invariant distance on top-K
  std::vector<Candidate> candidates;
  float best_dist = std::numeric_limits<float>::max();
  int   best_id   = -1;
  for (size_t i = 0; i < top_k; ++i) {
    const size_t idx = rk_dists[i].second;
    const float  d   = columnAlignedDistance(query, database_[idx]);
    if (d < best_dist) { best_dist = d; best_id = kf_ids_[idx]; }
    if (d < dist_thresh) candidates.push_back({kf_ids_[idx], d});
  }

  // if (logger && candidates.empty() && best_id >= 0 &&
  //     (database_.size() % 50 == 0)) {
  //   RCLCPP_INFO(*logger, "[SC] best_dist=%.4f (kf#%d)  thresh=%.3f  db=%zu",
  //     best_dist, best_id, dist_thresh, database_.size());
  // }

  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & x, const Candidate & y) { return x.distance < y.distance; });

  return candidates;
}
