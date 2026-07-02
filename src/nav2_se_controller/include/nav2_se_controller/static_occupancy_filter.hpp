// Copyright (c) 2026 Jungmo Kang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SE_CONTROLLER__STATIC_OCCUPANCY_FILTER_HPP_
#define NAV2_SE_CONTROLLER__STATIC_OCCUPANCY_FILTER_HPP_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>  // NOLINT(build/include_order)

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_se_controller
{

/// Tunables for the per-cell static-evidence grid.
struct StaticFilterConfig
{
  unsigned char cost_threshold{253};  ///< occupied threshold (match the tracker's).
  int static_min_frames{10};          ///< occupied observations before a cell is static.
  int evidence_cap{20};               ///< saturation so freed cells forget quickly.
  int stale_prune_frames{100};        ///< forget cells not observed for this many frames.
};

/**
 * @class StaticOccupancyFilter
 * @brief Classical DOGM-lite: per-cell occupancy persistence, world-anchored.
 *
 * SE-Predict N1 (design §3.1a). Maintains a sparse grid of "static evidence"
 * keyed by world coordinates (not costmap indices, so it survives the rolling
 * local-costmap window). A cell observed occupied accumulates evidence; a cell
 * observed free decays it. Cells whose evidence reaches static_min_frames are
 * STATIC — persistent structure (walls, furniture) — regardless of any phantom
 * velocity the cluster tracker's data association may assign them. Unknown
 * (NO_INFORMATION) observations change nothing.
 *
 * This is intentionally evidence-of-occupancy, not motion estimation: an
 * obstacle that genuinely stops long enough becomes "static" and is handed
 * back to the MPPI/costmap path, which is the DOGM semantic (stationary ==
 * static) and the safe division of labour for the CBF (dynamic only). The
 * same mechanism means a VERY slow obstacle (cell dwell time exceeding
 * static_min_frames) drifts toward static — bounded by choosing
 * static_min_frames ~ 1 s of frames: anything slower than roughly its own
 * radius per second is then quasi-static and safely handled by the costmap.
 *
 * Cells never observed (or recently appeared) are NOT static — the
 * conservative direction: an unproven obstacle stays eligible for the CBF.
 */
class StaticOccupancyFilter
{
public:
  void configure(const StaticFilterConfig & cfg) {cfg_ = cfg;}
  const StaticFilterConfig & config() const {return cfg_;}

  /**
   * @brief Fold one costmap frame into the evidence grid.
   *
   * Occupied cells (cost >= cost_threshold, not NO_INFORMATION) gain one
   * evidence point (saturating); already-tracked cells observed free lose one
   * (erased at zero). Entries unseen for stale_prune_frames are pruned.
   */
  void update(const nav2_costmap_2d::Costmap2D & costmap);

  /// True if the world point lies in a cell with static-level evidence.
  bool isStatic(double wx, double wy) const;

  /// Fraction of the given world points that are static ([0, 1]; 0 if empty).
  double staticFraction(const std::vector<Eigen::Vector2d> & points) const;

  void reset()
  {
    grid_.clear();
    frame_ = 0;
    resolution_ = 0.0;
  }

  std::size_t trackedCells() const {return grid_.size();}

private:
  struct CellEvidence
  {
    std::int32_t evidence{0};
    std::uint32_t last_seen{0};
  };

  std::uint64_t key(double wx, double wy) const;

  StaticFilterConfig cfg_{};
  std::unordered_map<std::uint64_t, CellEvidence> grid_;
  double resolution_{0.0};   ///< quantization step, locked to the first costmap seen.
  std::uint32_t frame_{0};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__STATIC_OCCUPANCY_FILTER_HPP_
