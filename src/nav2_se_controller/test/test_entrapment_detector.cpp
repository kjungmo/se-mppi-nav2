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

#include <gtest/gtest.h>

#include "nav2_se_controller/entrapment_detector.hpp"

using nav2_se_controller::EntrapmentConfig;
using nav2_se_controller::EntrapmentDetector;

TEST(EntrapmentDetector, NoEntrapmentWhileProgressing)
{
  EntrapmentDetector d;
  EntrapmentConfig cfg;
  cfg.progress_stall_window = 5;
  d.configure(cfg);

  for (std::size_t i = 1; i <= 20; ++i) {
    EXPECT_FALSE(d.update(i));
  }
  EXPECT_FALSE(d.entrapped());
}

TEST(EntrapmentDetector, DetectsStall)
{
  EntrapmentDetector d;
  EntrapmentConfig cfg;
  cfg.progress_stall_window = 5;
  d.configure(cfg);

  d.update(10);  // seed observation
  bool entrapped = false;
  for (int i = 0; i < 5; ++i) {
    entrapped = d.update(10);  // no progress
  }
  EXPECT_TRUE(entrapped);
  EXPECT_TRUE(d.entrapped());
}

TEST(EntrapmentDetector, ClearsImmediatelyOnProgress)
{
  EntrapmentDetector d;
  EntrapmentConfig cfg;
  cfg.progress_stall_window = 3;
  d.configure(cfg);

  d.update(0);
  for (int i = 0; i < 3; ++i) {
    d.update(0);
  }
  ASSERT_TRUE(d.entrapped());

  // A single real progress step (furthest index increases) clears entrapment.
  EXPECT_FALSE(d.update(1));
}

TEST(EntrapmentDetector, StopAndGoEscapeDoesNotStayLatched)
{
  EntrapmentDetector d;
  EntrapmentConfig cfg;
  cfg.progress_stall_window = 3;
  d.configure(cfg);

  d.update(0);
  for (int i = 0; i < 3; ++i) {
    d.update(0);
  }
  ASSERT_TRUE(d.entrapped());

  // Intermittent progress: advance, stall a cycle, advance -> must not re-latch
  // (clears on the first advance and a single stall is well under the window).
  EXPECT_FALSE(d.update(1));
  EXPECT_FALSE(d.update(1));  // one stall
  EXPECT_FALSE(d.update(2));  // progress again
}

TEST(EntrapmentDetector, ResetClearsState)
{
  EntrapmentDetector d;
  EntrapmentConfig cfg;
  cfg.progress_stall_window = 2;
  d.configure(cfg);

  d.update(0);
  d.update(0);
  d.update(0);
  ASSERT_TRUE(d.entrapped());

  d.reset();
  EXPECT_FALSE(d.entrapped());
  EXPECT_EQ(d.stallCount(), 0);
}
