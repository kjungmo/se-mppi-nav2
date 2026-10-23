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

// Guard for the held-out campaign geometries (decision D-3 and LEAD_DECISIONS_2 E-2).
//
// Before the campaign (until 2026-10-22) the world names could appear nowhere outside
// experiments/campaign2/, so that no code, test, parameter or document was developed against
// them. The campaign ended on 2026-10-22; the paper, the number guard and the committed results
// now name the worlds. The guard keeps the part of the rule that is still true: the names may
// appear only in the allowed paths below, and never in the controller sources, the development
// family generator, the headless test worlds, the tuning configs or the tests. A name found
// anywhere else fails the test. The names are assembled at run time so that this file does not
// contain them.

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

// The held-out worlds: s3 (a pocket inside the development family's range, with a twin there) and
// s4 (outside the range, authored blind).
std::vector<std::string> heldOutNames()
{
  return {std::string("s3") + "_" + "ctrap", std::string("s4") + "_" + "ltrap"};
}

// Paths (relative to the repository root) where the names may appear after the campaign. An entry
// that ends in '/' is a directory prefix; any other entry is one file.
//   experiments/campaign2/            the campaign: plan, worlds, scenarios, raw data, results
//   docs/papers/                      the paper sources (LaTeX, arXiv package) on the campaign
//   scripts/check_paper_numbers.py    the number guard that ties the paper to the committed results
std::vector<std::string> allowedPaths()
{
  return {"experiments/campaign2/", "docs/papers/", "scripts/check_paper_numbers.py"};
}

bool isAllowed(const std::string & rel, const std::vector<std::string> & allowed)
{
  return std::any_of(
    allowed.begin(), allowed.end(),
    [&rel](const std::string & a) {
      return a.back() == '/' ? rel.rfind(a, 0) == 0 : rel == a;
    });
}

// Files under `root` that contain any of `needles`, skipping `.git`, build trees and
// the allowed paths (see allowedPaths()).
std::vector<std::string> filesWithNeedle(
  const fs::path & root, const std::vector<std::string> & needles,
  const std::vector<std::string> & allowed)
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
    if (isAllowed(rel, allowed)) {
      continue;
    }
    if (it->file_size() > (64u << 20)) {
      continue;  // data blobs
    }
    std::ifstream in(it->path(), std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    if (std::any_of(
        needles.begin(), needles.end(),
        [&text](const std::string & n) {return text.find(n) != std::string::npos;}))
    {
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

// The scanner itself: it must flag a file outside the allowed paths and
// ignore one inside them (this is the mutation check of the guard).
TEST(HeldOutGuard, ScannerFlagsOutsideAndIgnoresCampaignDirectory)
{
  const fs::path tmp = fs::temp_directory_path() / "se_heldout_guard_selftest";
  fs::remove_all(tmp);
  const auto names = heldOutNames();
  ASSERT_EQ(names.size(), 2u);
  writeFile(tmp / "src/a.cpp", "// uses " + names[0] + "\n");
  writeFile(tmp / "docs/second.md", "the second pocket: " + names[1]);   // the second name alone
  writeFile(tmp / "experiments/campaign2/worlds/w.sdf", names[0]);
  writeFile(tmp / "experiments/campaign2/worlds/v.sdf", names[1]);
  writeFile(tmp / "experiments/other/b.yaml", "world: " + names[0]);
  writeFile(tmp / ".git/config", names[0] + names[1]);
  writeFile(tmp / "docs/clean.md", "nothing here");
  // Allowed after the campaign: the paper sources and the number guard (but not their lookalikes).
  writeFile(tmp / "docs/papers/latex/main.tex", names[0] + " " + names[1]);
  writeFile(tmp / "scripts/check_paper_numbers.py", names[0]);
  writeFile(tmp / "scripts/other.py", names[0]);
  writeFile(tmp / "docs/papers_notes.md", names[1]);
  writeFile(tmp / "experiments/sim/worlds/w.sdf", names[1]);
  const auto hits = filesWithNeedle(tmp, names, allowedPaths());
  fs::remove_all(tmp);
  ASSERT_EQ(hits.size(), 6u);
  std::vector<std::string> sorted = hits;
  std::sort(sorted.begin(), sorted.end());
  EXPECT_EQ(sorted[0], "docs/papers_notes.md");
  EXPECT_EQ(sorted[1], "docs/second.md");
  EXPECT_EQ(sorted[2], "experiments/other/b.yaml");
  EXPECT_EQ(sorted[3], "experiments/sim/worlds/w.sdf");
  EXPECT_EQ(sorted[4], "scripts/other.py");
  EXPECT_EQ(sorted[5], "src/a.cpp");
}

TEST(HeldOutGuard, TheHeldOutWorldNamesAreAbsentOutsideTheAllowedPaths)
{
  const fs::path root = SE_REPO_ROOT;
  if (root.empty() || !fs::exists(root / "src" / "nav2_se_controller")) {
    GTEST_SKIP() << "repository root not available at SE_REPO_ROOT='" << root.string() << "'";
  }
  const auto hits = filesWithNeedle(root, heldOutNames(), allowedPaths());
  for (const auto & h : hits) {
    ADD_FAILURE()
      << "a held-out world name appears outside the allowed paths "
      "(experiments/campaign2/, docs/papers/, scripts/check_paper_numbers.py): " << h;
  }
  EXPECT_TRUE(hits.empty());
}
