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

#ifndef NAV2_SE_CONTROLLER__NO_VALID_CONTROL_HPP_
#define NAV2_SE_CONTROLLER__NO_VALID_CONTROL_HPP_

#include <exception>
#include <stdexcept>
#include <string>

#ifndef NAV2_SE_CONTROLLER_HUMBLE_API
#include "nav2_core/controller_exceptions.hpp"
#endif

namespace nav2_se_controller
{

/**
 * The stock MPPI optimizer reports "no valid control" (every rollout collides)
 * with an exception. Jazzy throws nav2_core::NoValidControl. Humble's nav2_core
 * has no such class; its MPPI throws std::runtime_error("Optimizer fail to
 * compute path"). These two helpers hide the difference.
 */
inline bool isNoValidControl(const std::exception & e)
{
#ifdef NAV2_SE_CONTROLLER_HUMBLE_API
  return dynamic_cast<const std::runtime_error *>(&e) != nullptr &&
         std::string(e.what()).find("Optimizer fail to compute path") != std::string::npos;
#else
  return dynamic_cast<const nav2_core::NoValidControl *>(&e) != nullptr;
#endif
}

/// Throw what the optimizer throws when it finds no valid control (tests).
[[noreturn]] inline void throwNoValidControl()
{
#ifdef NAV2_SE_CONTROLLER_HUMBLE_API
  throw std::runtime_error("Optimizer fail to compute path");
#else
  throw nav2_core::NoValidControl("Optimizer fail to compute path");
#endif
}

}  // namespace nav2_se_controller

#endif  // NAV2_SE_CONTROLLER__NO_VALID_CONTROL_HPP_
