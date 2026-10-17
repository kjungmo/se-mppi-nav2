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

// Opening selection at the stall pose of the campaign-1 U-trap trials
// (recorded scenario s1_utrap; it is not a development geometry and no
// controller tuning uses it). The robot sits at map (0.29, 0.10), 0.34 m from
// the inflated back-wall band, in a 3 m x 3 m rolling window. The legacy rule
// (closest viable ray to the goal bearing) picks a sideways ray; the horizon
// rule picks the mouth.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/gap_search.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Box
{
  double x0, x1, y0, y1;
};

double boxDistance(const Box & b, double x, double y)
{
  const double dx = std::max({b.x0 - x, 0.0, x - b.x1});
  const double dy = std::max({b.y0 - y, 0.0, y - b.y1});
  return std::hypot(dx, dy);
}

// s1_utrap: back wall x in [0.85, 1.15], side walls y in +-[0.85, 1.15], all
// spanning the pocket (x in [-1, 1]); inner pocket x in [-1, 0.85], |y| <= 0.85.
nav2_costmap_2d::Costmap2D s1Window(double cx, double cy)
{
  const std::vector<Box> walls{
    {0.85, 1.15, -1.0, 1.0}, {-1.0, 1.0, 0.85, 1.15}, {-1.0, 1.0, -1.15, -0.85}};
  const double res = 0.05;
  const double ox = std::floor((cx - 1.5) / res + 0.5) * res;
  const double oy = std::floor((cy - 1.5) / res + 0.5) * res;
  nav2_costmap_2d::Costmap2D cm(60, 60, res, ox, oy, 0);
  for (int j = 0; j < 60; ++j) {
    for (int i = 0; i < 60; ++i) {
      const double x = ox + (i + 0.5) * res;
      const double y = oy + (j + 0.5) * res;
      double d = 1e9;
      for (const auto & w : walls) {
        d = std::min(d, boxDistance(w, x, y));
      }
      unsigned char c = 0;
      if (d <= 0.0) {
        c = 254;
      } else if (d <= 0.22) {
        c = 253;
      } else if (d < 0.45) {
        c = static_cast<unsigned char>(std::max(1.0, std::round(252.0 * (0.45 - d) / 0.23)));
      }
      if (c != 0) {
        cm.setCost(i, j, c);
      }
    }
  }
  return cm;
}

double deg(double rad) {return rad * 180.0 / kPi;}

double angDiff(double a, double b) {return std::atan2(std::sin(a - b), std::cos(a - b));}
}  // namespace

// Independent reference for the ray casting (same sampling rule as the
// implementation: half-cell steps, a cell at or above 253 stops the ray, the
// map edge ends it as free): used to print the clearances and to cross-check.
struct RefRay
{
  double bearing;
  double clearance;
  bool horizon;
};

std::vector<RefRay> refRays(
  const nav2_costmap_2d::Costmap2D & cm, double rx, double ry, int n, double max_range)
{
  std::vector<RefRay> out;
  const double step = 0.5 * cm.getResolution();
  for (int k = 0; k < n; ++k) {
    const double b = -kPi + 2.0 * kPi * k / n;
    RefRay ray{b, max_range, true};
    for (double r = step; r <= max_range; r += step) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!cm.worldToMap(rx + r * std::cos(b), ry + r * std::sin(b), mx, my)) {
        ray = {b, r, true};
        break;
      }
      if (cm.getCost(mx, my) >= 253) {
        ray = {b, r, false};
        break;
      }
    }
    out.push_back(ray);
  }
  return out;
}

void printRays(const std::vector<RefRay> & rays)
{
  for (const auto & r : rays) {
    std::printf(
      "GAP ray %4.0f deg  clearance %.2f m  %s\n", deg(r.bearing), r.clearance,
      r.horizon ? "horizon" : "blocked");
  }
}

// Legacy rule (kept for the C_gap ablation): the diagnosis of ROOT_CAUSE.md
// section 4. Prints all 36 clearances.
TEST(GapPocket, LegacySelectionPicksASidewaysRayAtTheStallPose)
{
  const auto cm = s1Window(0.29, 0.10);
  std::printf("GAP stall pose (0.29, 0.10), goal bearing 0 deg, 36 rays, max_range 2.0\n");
  printRays(refRays(cm, 0.29, 0.10, 36, 2.0));
  const auto gap = nav2_se_controller::findEscapeGap(cm, 0.29, 0.10, 0.0, 36, 2.0, 0.6);
  ASSERT_TRUE(gap.found);
  std::printf(
    "GAP legacy selection: bearing %.0f deg, clearance %.2f m\n", deg(gap.bearing), gap.clearance);
  EXPECT_NEAR(std::abs(deg(gap.bearing)), 60.0, 10.0)
    << "the legacy rule chooses a sideways ray, not the mouth";
}
