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

#ifndef NAV2_SE_CONTROLLER__MULTI_ROBOT_COORDINATOR_HPP_
#define NAV2_SE_CONTROLLER__MULTI_ROBOT_COORDINATOR_HPP_

#include <vector>

#include "nav2_se_controller/cbf_types.hpp"

namespace nav2_se_controller
{

/// A neighbor robot as known to this controller (from its odom topic).
struct NeighborRobot
{
  int id{0};                              ///< fleet priority id (lower passes).
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  bool valid{false};                      ///< received at least one message.
};

/// Tunables for the multi-robot coordination layer (Multi-SE-MPPI N2).
struct MultiRobotConfig
{
  double match_radius{0.5};        ///< neighbor pose -> tracked cluster match (m).
  double reciprocal_lambda{0.5};   ///< default budget share for neighbor robots.
  double pass_lambda{0.7};         ///< passer's share (more allowance)...
  double yield_lambda{0.3};        ///< ...yielder's (deference); pair sums to 1,
                                   ///< so the joint barrier decays at
                                   ///< lambda_p*alpha_escape + lambda_y*alpha_base
                                   ///< — still a valid class-K rate (> 0).
  double deadlock_range{1.6};      ///< both stalled within this range => deadlock (m).
  double deadlock_speed{0.12};     ///< "stalled" speed threshold (m/s).
  double clear_range{1.6};         ///< conflict over when no neighbor ahead within (m).
  double yield_v_max{0.10};        ///< forward-speed cap while yielding (m/s).
};

/**
 * @class MultiRobotCoordinator
 * @brief Neighbor marking + mutual-deadlock priority (Multi-SE-MPPI N2, 2/2).
 *
 * Pure logic, ported from the validated 2D prototype (multi_se_proto.py):
 *
 *  1. markNeighbors(): tracked clusters whose centroid lies within
 *     match_radius of a known neighbor robot get the reciprocal barrier-budget
 *     share (TrackedObstacle::responsibility) instead of the single-robot
 *     lambda = 1 — both robots run the same protocol, so the pair restores
 *     the joint barrier with no velocity exchange (see cbf_types.hpp).
 *
 *  2. update(): the deadlock state machine. A mutual deadlock is declared
 *     when this robot is entrapped, nearly stopped, and a neighbor within
 *     deadlock_range is also nearly stopped. Priority is the deterministic
 *     id convention (LOWER id passes — stands in for a fleet-priority
 *     broadcast or traffic norm). Roles LATCH until the conflict clears
 *     (every conflicting neighbor behind or out of range): an unlatched role
 *     would flicker off the moment the sidestep itself counts as progress —
 *     exactly the oscillation it exists to remove (prototype finding).
 *
 * The behavioral consequences are applied by the controller: the passer keeps
 * the escape pipeline with alpha_escape and pass_lambda; the yielder gets
 * yield_lambda and a forward-speed cap (hold/back-off primitive). Live
 * multi-robot validation runs on the workstation (N3 milestone).
 */
class MultiRobotCoordinator
{
public:
  enum class Role { kNone, kPass, kYield };

  void configure(const MultiRobotConfig & cfg) {cfg_ = cfg;}
  const MultiRobotConfig & config() const {return cfg_;}

  /// Stamp reciprocal responsibility onto tracked obstacles that ARE known
  /// neighbor robots. Returns how many obstacles were marked.
  int markNeighbors(
    std::vector<TrackedObstacle> & obstacles,
    const std::vector<NeighborRobot> & neighbors) const;

  /**
   * @brief Advance the deadlock/priority state machine one cycle.
   * @param my_id This robot's fleet id (lower passes).
   * @param state Current pose.
   * @param goal Current navigation goal (conflict geometry reference).
   * @param my_speed Last commanded forward speed (m/s).
   * @param entrapped Entrapment detector output.
   * @param neighbors Known neighbor robots.
   * @return The (latched) role for this cycle.
   */
  Role update(
    int my_id, const RobotState & state, const Eigen::Vector2d & goal,
    double my_speed, bool entrapped,
    const std::vector<NeighborRobot> & neighbors);

  Role role() const {return role_;}

  /// Barrier-budget share for neighbor-robot constraints under the role.
  double lambdaForRole() const
  {
    switch (role_) {
      case Role::kPass: return cfg_.pass_lambda;
      case Role::kYield: return cfg_.yield_lambda;
      default: return cfg_.reciprocal_lambda;
    }
  }

  void reset() {role_ = Role::kNone;}

private:
  bool conflictCleared(
    const RobotState & state, const Eigen::Vector2d & goal,
    const std::vector<NeighborRobot> & neighbors) const;

  MultiRobotConfig cfg_{};
  Role role_{Role::kNone};
};

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__MULTI_ROBOT_COORDINATOR_HPP_
