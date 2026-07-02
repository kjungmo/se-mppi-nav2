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

// Runtime smoke test: confirm the SE-MPPI plugins are discoverable and
// instantiable through pluginlib exactly as Nav2's servers load them. This
// catches plugin-XML, base-class, and symbol-export regressions without a full
// simulator.

#include <gtest/gtest.h>

#include "pluginlib/class_loader.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

TEST(PluginLoad, SafeEscapeControllerIsLoadable)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");
  std::shared_ptr<nav2_core::Controller> controller;
  ASSERT_NO_THROW(
    controller = loader.createSharedInstance("nav2_se_controller::SafeEscapeController"));
  EXPECT_NE(controller, nullptr);
}

TEST(PluginLoad, EscapeCriticIsLoadable)
{
  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
    "nav2_mppi_controller", "mppi::critics::CriticFunction");
  std::shared_ptr<mppi::critics::CriticFunction> critic;
  ASSERT_NO_THROW(
    critic = loader.createSharedInstance("mppi::critics::EscapeCritic"));
  EXPECT_NE(critic, nullptr);
}
