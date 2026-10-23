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

// Parity shim: lets the Python mirror (experiments/prototype/se_mppi_v2.py) call the C++
// escape logic that the plugin runs. It reads commands on stdin, one per line, and prints
// one answer line per command (numbers with 17 significant digits). The Python parity test
// (experiments/benchmark2d/tests/test_v2_cxx_parity.py) drives it.
//
//   SELECT <min_clearance> <hysteresis> <goal_bearing> <prev|nan> <n_avoid>
//          {<bearing> <distance> <radius>}*n_avoid
//          <n_rays> {<bearing> <clearance> <horizon>}*n_rays
//     -> found bearing clearance horizon kept_previous candidates
//        (selectHorizonGap: horizon rule, minimum clearance, avoid list with its retry,
//         hysteresis; the rays are given, so only the selection rule is under test)
//   CAST <res> <size> <ox> <oy> <rx> <ry> <n_rays> <max_range> <threshold>
//          then <size> lines of <size> integers (row y, columns x)
//     -> one line per ray: bearing clearance horizon   (castGapRays on that grid)
//   PROGRESS <min_advance> <adopt_advance> <stall_window>   (starts a new tracker + detector)
//   START <x0> <y0> <x1> <y1>      (PathProgressTracker::startTask on a two-pose path)
//   REPLAN <x0> <y0> <x1> <y1>     (PathProgressTracker::offerReplan)
//   UPDATE <x> <y>                 (tracker.update, then EntrapmentDetector::update(events))
//     -> events adoptions entrapped
//   QUIT

#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_se_controller/entrapment_detector.hpp"
#include "nav2_se_controller/opening_search.hpp"
#include "nav2_se_controller/path_progress.hpp"
#include "nav_msgs/msg/path.hpp"

namespace
{
nav_msgs::msg::Path twoPosePath(double x0, double y0, double x1, double y1)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  for (const auto & xy : {std::make_pair(x0, y0), std::make_pair(x1, y1)}) {
    geometry_msgs::msg::PoseStamped s;
    s.pose.position.x = xy.first;
    s.pose.position.y = xy.second;
    s.pose.orientation.w = 1.0;
    p.poses.push_back(s);
  }
  return p;
}
}  // namespace

int main()
{
  using namespace nav2_se_controller;  // NOLINT
  std::string line;
  PathProgressTracker tracker;
  EntrapmentDetector detector;
  std::printf("READY\n");
  std::fflush(stdout);
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "QUIT") {
      break;
    } else if (cmd == "SELECT") {
      double min_clearance, hysteresis, goal_bearing;
      std::string prev;
      int n_avoid = 0;
      in >> min_clearance >> hysteresis >> goal_bearing >> prev >> n_avoid;
      std::vector<AvoidPoint> avoid;
      for (int i = 0; i < n_avoid; ++i) {
        AvoidPoint a;
        in >> a.bearing >> a.distance >> a.radius;
        avoid.push_back(a);
      }
      int n_rays = 0;
      in >> n_rays;
      std::vector<GapRay> rays;
      for (int i = 0; i < n_rays; ++i) {
        GapRay r;
        int h = 0;
        in >> r.bearing >> r.clearance >> h;
        r.horizon = h != 0;
        rays.push_back(r);
      }
      std::optional<double> previous;
      if (prev != "nan") {
        previous = std::stod(prev);
      }
      const HorizonGap g =
        selectHorizonGap(rays, goal_bearing, min_clearance, previous, hysteresis, avoid);
      std::printf(
        "%d %.17g %.17g %d %d %zu\n", g.found ? 1 : 0, g.bearing, g.clearance, g.horizon ? 1 : 0,
        g.kept_previous ? 1 : 0, g.candidates);
    } else if (cmd == "CAST") {
      double res, ox, oy, rx, ry, max_range;
      int size, n_rays, threshold;
      in >> res >> size >> ox >> oy >> rx >> ry >> n_rays >> max_range >> threshold;
      nav2_costmap_2d::Costmap2D cm(size, size, res, ox, oy, 0);
      for (int y = 0; y < size; ++y) {
        std::string row;
        std::getline(std::cin, row);
        std::istringstream rs(row);
        for (int x = 0; x < size; ++x) {
          int c = 0;
          rs >> c;
          cm.setCost(x, y, static_cast<unsigned char>(c));
        }
      }
      const auto rays = castGapRays(
        cm, rx, ry, n_rays, max_range,
        static_cast<unsigned char>(threshold));
      for (const auto & r : rays) {
        std::printf("%.17g %.17g %d\n", r.bearing, r.clearance, r.horizon ? 1 : 0);
      }
    } else if (cmd == "PROGRESS") {
      ProgressConfig pc;
      int window = 30;
      in >> pc.min_advance >> pc.adopt_advance >> window;
      tracker = PathProgressTracker();
      tracker.configure(pc);
      detector = EntrapmentDetector();
      EntrapmentConfig ec;
      ec.progress_stall_window = window;
      detector.configure(ec);
      std::printf("OK\n");
    } else if (cmd == "START" || cmd == "REPLAN") {
      double x0, y0, x1, y1;
      in >> x0 >> y0 >> x1 >> y1;
      if (cmd == "START") {
        tracker.startTask(twoPosePath(x0, y0, x1, y1));
        detector.reset();
      } else {
        tracker.offerReplan(twoPosePath(x0, y0, x1, y1));
      }
      std::printf("OK\n");
    } else if (cmd == "UPDATE") {
      double x, y;
      in >> x >> y;
      const std::size_t ev = tracker.update(x, y);
      const bool ent = detector.update(ev);
      std::printf("%zu %zu %d\n", ev, tracker.adoptions(), ent ? 1 : 0);
    } else {
      std::printf("ERR unknown command\n");
    }
    std::fflush(stdout);
  }
  return 0;
}
