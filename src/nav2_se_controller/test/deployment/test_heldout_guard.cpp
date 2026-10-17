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

// Guard for the held-out campaign geometry (decision D-3): its world name must
// not appear in the repository outside experiments/campaign2/, so that no code,
// test, parameter or document is developed against it before the campaign. The
// name is assembled at run time so that this file does not contain it.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifndef SE_REPO_ROOT
#define SE_REPO_ROOT ""
#endif

namespace fs = std::filesystem;

namespace
{

std::string heldOutName()
{
  return std::string("s3") + "_" + "ctrap";
}

// Files under `root` that contain `needle`, skipping `.git`, build trees and
// the exempt subdirectory (a path relative to root, with a trailing slash).
std::vector<std::string> filesWithNeedle(
  const fs::path & root, const std::string & needle, const std::string & exempt)
{
  std::vector<std::string> hits;
  for (auto it = fs::recursive_directory_iterator(
      root, fs::directory_options::skip_permission_denied);
    it != fs::recursive_directory_iterator(); ++it)
  {
    const std::string rel = fs::relative(it->path(), root).generic_string();
    if (it->is_directory()) {
      const std::string base = it->path().filename().string();
      if (base == ".git" || base == "build" || base == "__pycache__") {
        it.disable_recursion_pending();
      }
      continue;
    }
    if (!it->is_regular_file()) {
      continue;
    }
    if (!exempt.empty() && rel.rfind(exempt, 0) == 0) {
      continue;
    }
    if (it->file_size() > (64u << 20)) {
      continue;  // data blobs
    }
    std::ifstream in(it->path(), std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    if (ss.str().find(needle) != std::string::npos) {
      hits.push_back(rel);
    }
  }
  return hits;
}

void writeFile(const fs::path & p, const std::string & text)
{
  fs::create_directories(p.parent_path());
  std::ofstream(p) << text;
}

}  // namespace

// The scanner itself: it must flag a file outside the exempt directory and
// ignore one inside it (this is the mutation check of the guard).
TEST(HeldOutGuard, ScannerFlagsOutsideAndIgnoresCampaignDirectory)
{
  const fs::path tmp = fs::temp_directory_path() / "se_heldout_guard_selftest";
  fs::remove_all(tmp);
  writeFile(tmp / "src/a.cpp", "// uses " + heldOutName() + "\n");
  writeFile(tmp / "experiments/campaign2/worlds/w.sdf", heldOutName());
  writeFile(tmp / "experiments/other/b.yaml", "world: " + heldOutName());
  writeFile(tmp / ".git/config", heldOutName());
  writeFile(tmp / "docs/clean.md", "nothing here");
  const auto hits = filesWithNeedle(tmp, heldOutName(), "experiments/campaign2/");
  fs::remove_all(tmp);
  ASSERT_EQ(hits.size(), 2u);
  std::vector<std::string> sorted = hits;
  std::sort(sorted.begin(), sorted.end());
  EXPECT_EQ(sorted[0], "experiments/other/b.yaml");
  EXPECT_EQ(sorted[1], "src/a.cpp");
}

TEST(HeldOutGuard, TheHeldOutWorldNameIsAbsentOutsideTheCampaignDirectory)
{
  const fs::path root = SE_REPO_ROOT;
  if (root.empty() || !fs::exists(root / "src" / "nav2_se_controller")) {
    GTEST_SKIP() << "repository root not available at SE_REPO_ROOT='" << root.string() << "'";
  }
  const auto hits = filesWithNeedle(root, heldOutName(), "experiments/campaign2/");
  for (const auto & h : hits) {
    ADD_FAILURE() << "the held-out world name appears outside experiments/campaign2/: " << h;
  }
  EXPECT_TRUE(hits.empty());
}
