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

// Deployment test: the continuous-QP-failure rule behind the ERROR status.

#include <gtest/gtest.h>

#include <cstdint>

#include "nav2_se_controller/failure_streak.hpp"

using nav2_se_controller::FailureStreak;

namespace
{
constexpr std::int64_t kMs = 1000000;
constexpr std::int64_t kDuration = 2000 * kMs;
constexpr std::int64_t kStale = 500 * kMs;
}  // namespace

TEST(FailureStreak, ContinuousFailureLongerThanTheDurationTrips)
{
  FailureStreak s;
  for (std::int64_t t = 1; t <= 2100; t += 50) {
    s.record(true, t * kMs);
  }
  EXPECT_TRUE(s.longerThan(2101 * kMs, kDuration, kStale));
  EXPECT_FALSE(s.longerThan(1500 * kMs, kDuration, kStale));
}

TEST(FailureStreak, OneSuccessRestartsTheStreak)
{
  FailureStreak s;
  for (std::int64_t t = 1; t <= 1500; t += 50) {
    s.record(true, t * kMs);
  }
  s.record(false, 1550 * kMs);
  for (std::int64_t t = 1600; t <= 2600; t += 50) {
    s.record(true, t * kMs);
  }
  EXPECT_FALSE(s.longerThan(2601 * kMs, kDuration, kStale));
}

TEST(FailureStreak, IdleLoopDoesNotKeepAStaleStreak)
{
  FailureStreak s;
  for (std::int64_t t = 1; t <= 2500; t += 50) {
    s.record(true, t * kMs);
  }
  EXPECT_TRUE(s.longerThan(2501 * kMs, kDuration, kStale));
  EXPECT_FALSE(s.longerThan(4000 * kMs, kDuration, kStale));  // no cycle for 1.5 s
}
