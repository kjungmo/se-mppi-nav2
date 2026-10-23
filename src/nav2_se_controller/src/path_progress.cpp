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

#include "nav2_se_controller/path_progress.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_se_controller
{

std::size_t nearestPathIndex(const nav_msgs::msg::Path & path, double x, double y)
{
  std::size_t best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - x;
    const double dy = path.poses[i].pose.position.y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

PathProgressTracker::Poly PathProgressTracker::build(const nav_msgs::msg::Path & path)
{
  Poly out;
  double arc = 0.0;
  for (const auto & pose : path.poses) {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    if (!out.x.empty()) {
      arc += std::hypot(x - out.x.back(), y - out.y.back());
    }
    out.x.push_back(x);
    out.y.push_back(y);
    out.arc.push_back(arc);
  }
  return out;
}

double PathProgressTracker::arcAtNearest(const Poly & poly, double x, double y)
{
  if (poly.x.size() < 2) {
    return 0.0;
  }
  double best_d2 = std::numeric_limits<double>::max();
  double best_arc = 0.0;
  for (std::size_t i = 0; i + 1 < poly.x.size(); ++i) {
    const double ax = poly.x[i];
    const double ay = poly.y[i];
    const double dx = poly.x[i + 1] - ax;
    const double dy = poly.y[i + 1] - ay;
    const double len2 = dx * dx + dy * dy;
    const double t_raw = len2 > 0.0 ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
    const double t = std::clamp(t_raw, 0.0, 1.0);
    const double ex = x - (ax + t * dx);
    const double ey = y - (ay + t * dy);
    const double d2 = ex * ex + ey * ey;
    if (d2 < best_d2) {
      best_d2 = d2;
      // Behind the first pose the arc continues backwards (negative), so that
      // motion away from the start of a path counts as negative advance.
      const double t_arc = (i == 0 && t_raw < 0.0) ? t_raw : t;
      best_arc = poly.arc[i] + t_arc * std::sqrt(len2);
    }
  }
  return best_arc;
}

void PathProgressTracker::reset()
{
  anchor_ = Poly{};
  candidate_ = Poly{};
  candidate_pending_base_ = false;
  candidate_base_ = 0.0;
  carried_ = 0.0;
  chain_ = 0.0;
  best_arc_ = 0.0;
  has_best_ = false;
  has_last_ = false;
  events_ = 0;
  adoptions_ = 0;
}

void PathProgressTracker::startTask(const nav_msgs::msg::Path & plan)
{
  reset();
  anchor_ = build(plan);
}

void PathProgressTracker::offerReplan(const nav_msgs::msg::Path & plan)
{
  Poly poly = build(plan);
  if (poly.x.empty()) {
    return;
  }
  // Close the old candidate: its signed advance up to the last known position
  // joins the chain.
  if (!candidate_.arc.empty() && !candidate_pending_base_ && has_last_) {
    carried_ += arcAtNearest(candidate_, last_x_, last_y_) - candidate_base_;
  }
  candidate_ = std::move(poly);
  candidate_pending_base_ = true;  // the new path starts where the robot is
}

std::size_t PathProgressTracker::update(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y) || anchor_.arc.empty()) {
    return events_;
  }
  last_x_ = x;
  last_y_ = y;
  has_last_ = true;
  const double arc = arcAtNearest(anchor_, x, y);
  if (!has_best_) {
    best_arc_ = arc;  // baseline: the first position is not progress
    has_best_ = true;
  }
  if (!candidate_.arc.empty()) {
    const double cand_arc = arcAtNearest(candidate_, x, y);
    if (candidate_pending_base_) {
      candidate_base_ = cand_arc;
      candidate_pending_base_ = false;
    }
    chain_ = carried_ + (cand_arc - candidate_base_);
    if (chain_ >= cfg_.adopt_advance) {
      anchor_ = std::move(candidate_);
      candidate_ = Poly{};
      candidate_pending_base_ = false;
      carried_ = 0.0;
      chain_ = 0.0;
      best_arc_ = arcAtNearest(anchor_, x, y);
      ++events_;
      ++adoptions_;
      return events_;
    }
  } else {
    chain_ = carried_;
  }
  if (arc - best_arc_ >= cfg_.min_advance) {
    best_arc_ = arc;
    ++events_;
    // The chain restarts from here.
    carried_ = 0.0;
    chain_ = 0.0;
    if (!candidate_.arc.empty()) {
      candidate_base_ = arcAtNearest(candidate_, x, y);
    }
  }
  return events_;
}

}  // namespace nav2_se_controller
