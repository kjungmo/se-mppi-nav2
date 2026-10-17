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

// Development geometries for the closed-loop U-pocket test. The rule is
// written in UTRAP_DEV_GEOMETRIES.md (same directory) and was committed before
// the first run of the test. Plain C++, no ROS.

#ifndef UTRAP_DEV_GEOMETRIES_HPP_
#define UTRAP_DEV_GEOMETRIES_HPP_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace utrap
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfThickness = 0.075;
constexpr double kRobotRadius = 0.22;
constexpr double kInflationRadius = 0.45;
constexpr double kStartX = -2.6;
constexpr double kStartY = 0.0;
constexpr double kGoalX = 3.0;
constexpr double kGoalY = 0.0;
constexpr double kBackFaceX = 0.9;

struct Pocket
{
  double w{2.2};       // inner width (m)
  double d{1.3};       // inner depth, mouth to back face (m)
  double theta{0.0};   // rotation about the back-face centre (deg)
  double ft{1.0};      // top side-wall length factor
  double fb{1.0};      // bottom side-wall length factor
};

struct Segment
{
  double x0, y0, x1, y1;
};

inline std::string name(const Pocket & p)
{
  char buf[64];
  std::snprintf(
    buf, sizeof(buf), "w%d_d%d_r%d_a%d",
    static_cast<int>(std::lround(p.w * 10.0)), static_cast<int>(std::lround(p.d * 10.0)),
    static_cast<int>(std::lround(p.theta)), static_cast<int>(std::lround(p.ft * 10.0)));
  return buf;
}

// Pocket-frame (u, v) to world.
inline void toWorld(const Pocket & p, double u, double v, double & x, double & y)
{
  const double t = p.theta * kPi / 180.0;
  x = kBackFaceX + std::cos(t) * u - std::sin(t) * v;
  y = std::sin(t) * u + std::cos(t) * v;
}

// World to pocket-frame (u, v).
inline void toPocket(const Pocket & p, double x, double y, double & u, double & v)
{
  const double t = p.theta * kPi / 180.0;
  const double dx = x - kBackFaceX;
  const double dy = y;
  u = std::cos(t) * dx + std::sin(t) * dy;
  v = -std::sin(t) * dx + std::cos(t) * dy;
}

inline std::vector<Segment> walls(const Pocket & p)
{
  const double h = kHalfThickness;
  const double hw = 0.5 * p.w;
  std::vector<Segment> out;
  auto add = [&](double u0, double v0, double u1, double v1) {
      Segment s;
      toWorld(p, u0, v0, s.x0, s.y0);
      toWorld(p, u1, v1, s.x1, s.y1);
      out.push_back(s);
    };
  add(h, -(hw + 2.0 * h), h, hw + 2.0 * h);                      // back wall
  add(-p.d * p.ft, hw + h, 2.0 * h, hw + h);                     // top side wall
  add(-p.d * p.fb, -(hw + h), 2.0 * h, -(hw + h));               // bottom side wall
  return out;
}

inline double distToSegment(const Segment & s, double x, double y)
{
  const double dx = s.x1 - s.x0;
  const double dy = s.y1 - s.y0;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0.0 ? ((x - s.x0) * dx + (y - s.y0) * dy) / len2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(x - (s.x0 + t * dx), y - (s.y0 + t * dy));
}

// Distance from (x, y) to the nearest wall surface (negative inside a wall).
inline double surfaceDistance(const std::vector<Segment> & ws, double x, double y)
{
  double best = 1e9;
  for (const auto & s : ws) {
    best = std::min(best, distToSegment(s, x, y) - kHalfThickness);
  }
  return best;
}

// Costmap cost of a cell centre (rule in UTRAP_DEV_GEOMETRIES.md).
inline unsigned char cellCost(double surface_distance)
{
  if (surface_distance <= 0.0) {
    return 254;
  }
  if (surface_distance <= kRobotRadius) {
    return 253;
  }
  if (surface_distance < kInflationRadius) {
    const double f = (kInflationRadius - surface_distance) / (kInflationRadius - kRobotRadius);
    return static_cast<unsigned char>(std::max(1.0, std::round(252.0 * f)));
  }
  return 0;
}

// True if (x, y) is inside the pocket rectangle u in [-d, 0], |v| <= w/2.
inline bool insidePocket(const Pocket & p, double x, double y)
{
  double u, v;
  toPocket(p, x, y, u, v);
  return u >= -p.d && u <= 0.0 && std::abs(v) <= 0.5 * p.w;
}

inline bool valid(const Pocket & p)
{
  // V4: not the live s1 pocket.
  if (std::abs(p.w - 1.7) < 0.1 && std::abs(p.d - 1.85) < 0.1) {
    return false;
  }
  // V1: the straight path enters through the mouth.
  const double t = p.theta * kPi / 180.0;
  if (std::abs(p.d * std::tan(t)) > 0.5 * p.w - 0.3) {
    return false;
  }
  const auto ws = walls(p);
  // V2: start and goal in free space.
  if (surfaceDistance(ws, kStartX, kStartY) < 0.8 || surfaceDistance(ws, kGoalX, kGoalY) < 0.8) {
    return false;
  }
  // V3: the straight line S to G is blocked.
  bool blocked = false;
  for (double x = kStartX; x <= kGoalX; x += 0.01) {
    if (surfaceDistance(ws, x, 0.0) < kRobotRadius) {
      blocked = true;
      break;
    }
  }
  return blocked;
}

// All valid geometries in lexicographic order of their names.
inline std::vector<Pocket> family()
{
  std::vector<Pocket> out;
  for (double w : {1.3, 1.6, 2.2, 2.6}) {
    for (double d : {1.1, 1.5, 1.8}) {
      for (double th : {-25.0, 0.0, 15.0, 30.0}) {
        for (double ft : {1.0, 0.6}) {
          Pocket p;
          p.w = w;
          p.d = d;
          p.theta = th;
          p.ft = ft;
          p.fb = 1.0;
          if (valid(p)) {
            out.push_back(p);
          }
        }
      }
    }
  }
  std::sort(
    out.begin(), out.end(),
    [](const Pocket & a, const Pocket & b) {return name(a) < name(b);});
  return out;
}

// The two geometries that gate the test (fixed in the rule file).
inline std::vector<Pocket> primary()
{
  Pocket a;
  a.w = 2.2; a.d = 1.3; a.theta = 30.0; a.ft = 1.0; a.fb = 1.0;
  Pocket b;
  b.w = 1.6; b.d = 1.5; b.theta = -15.0; b.ft = 0.6; b.fb = 1.0;
  return {a, b};
}

// Every 7th valid geometry starting at index 3 (informational sweep).
inline std::vector<Pocket> sweep()
{
  const auto all = family();
  std::vector<Pocket> out;
  for (std::size_t i = 3; i < all.size(); i += 7) {
    out.push_back(all[i]);
  }
  return out;
}

}  // namespace utrap

#endif  // UTRAP_DEV_GEOMETRIES_HPP_
