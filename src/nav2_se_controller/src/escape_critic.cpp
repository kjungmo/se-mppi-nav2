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

#include "nav2_se_controller/escape_critic.hpp"

#include <cmath>

#include "nav2_se_controller/gap_search.hpp"
#include "nav2_se_controller/repulsion.hpp"

namespace mppi
{
namespace critics
{

void EscapeCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  nav2_se_controller::EntrapmentConfig cfg;
  getParam(cfg.progress_stall_window, "progress_stall_window", 30);
  detector_.configure(cfg);

  // Rendezvous with the controller's single entrapment source (keyed by the
  // parent controller name); falls back to detector_ when not driven.
  shared_ = nav2_se_controller::EntrapmentRegistry::get(parent_name_);

  getParam(always_on_, "always_on", false);
  getParam(use_apf_, "use_apf", true);
  getParam(repulsion_weight_, "repulsion_weight", 2.0f);
  getParam(repulsion_power_, "repulsion_power", 1);
  getParam(apf_influence_dist_, "apf_influence_dist", 0.6f);
  getParam(apf_eta_, "apf_eta", 1.0f);

  getParam(use_gap_search_, "use_gap_search", true);
  getParam(gap_weight_, "gap_weight", 4.0f);
  getParam(gap_power_, "gap_power", 1);
  getParam(gap_num_rays_, "gap_num_rays", 36);
  getParam(gap_max_range_, "gap_max_range", 2.0f);
  getParam(gap_min_clearance_, "gap_min_clearance", 0.6f);

  RCLCPP_INFO(
    logger_,
    "EscapeCritic[%s] initialized (M1.x): enabled=%d stall_window=%d mode=%s",
    name_.c_str(), enabled_, cfg.progress_stall_window,
    use_apf_ ? "APF" : "cost-proxy");
}

void EscapeCritic::score(mppi::CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // --- Entrapment: follow the controller's single source of truth when it is
  //     driving us; otherwise self-detect from path progress (stock MPPI). ---
  bool entrapped;
  if (shared_ && shared_->driven.load(std::memory_order_relaxed)) {
    entrapped = shared_->entrapped.load(std::memory_order_relaxed);
  } else {
    const std::size_t furthest = data.furthest_reached_path_point.value_or(0);
    entrapped = detector_.update(furthest);
  }

  // --- Conditional repulsive augmentation (detect-and-switch). ---
  // always_on_ (ablation baseline) injects the escape cost every cycle.
  const bool injecting = entrapped || always_on_;
  if (injecting != prev_injecting_) {
    // Runtime evidence for the live A/B: prove the escape cost actually enters
    // the MPPI objective (and when it stops).
    RCLCPP_INFO(
      logger_, "EscapeCritic[%s] escape-cost injection %s (entrapped=%d)",
      name_.c_str(), injecting ? "BEGIN" : "END", entrapped ? 1 : 0);
    prev_injecting_ = injecting;
  }
  if (!injecting) {
    return;
  }

  if (use_apf_) {
    data.costs += nav2_se_controller::computeApfRepulsionCosts(
      data.trajectories.x, data.trajectories.y, *costmap_,
      apf_influence_dist_, apf_eta_, repulsion_power_);
  } else {
    data.costs += nav2_se_controller::computeRepulsionCosts(
      data.trajectories.x, data.trajectories.y, *costmap_,
      repulsion_weight_, repulsion_power_);
  }

  // --- Free-space gap attraction: steer toward an opening that still heads
  //     roughly toward the goal (U-shaped / non-convex trap escape). ---
  if (use_gap_search_ && data.trajectories.x.shape(1) > 0) {
    const double rx = static_cast<double>(data.trajectories.x(0, 0));
    const double ry = static_cast<double>(data.trajectories.y(0, 0));
    const double goal_bearing =
      std::atan2(data.goal.position.y - ry, data.goal.position.x - rx);
    const nav2_se_controller::EscapeGap gap = nav2_se_controller::findEscapeGap(
      *costmap_, rx, ry, goal_bearing,
      gap_num_rays_, gap_max_range_, gap_min_clearance_);
    if (gap.found) {
      data.costs += nav2_se_controller::computeGapAttractionCosts(
        data.trajectories.x, data.trajectories.y, rx, ry, gap.bearing,
        gap_weight_, gap_power_);
    }
  }
}

}  // namespace critics
}  // namespace mppi

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mppi::critics::EscapeCritic, mppi::critics::CriticFunction)
