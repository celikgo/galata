// SPDX-License-Identifier: Apache-2.0
//
// The version surface is load-bearing for charter rule 3 (one source of
// version truth) and rule 9 (no number reaches the user without provenance).
// These tests catch a broken configure_file substitution, which otherwise
// shows up as a literal "@PROJECT_VERSION@" in a shipped result file.

#include "galata/version.hpp"

#include <gtest/gtest.h>

#include <charconv>
#include <string>
#include <string_view>
#include <vector>

namespace {

std::vector<std::string_view> split(std::string_view s, char sep) {
  std::vector<std::string_view> parts;
  std::size_t start = 0;
  while (true) {
    const std::size_t hit = s.find(sep, start);
    if (hit == std::string_view::npos) {
      parts.push_back(s.substr(start));
      return parts;
    }
    parts.push_back(s.substr(start, hit - start));
    start = hit + 1;
  }
}

bool parse_int(std::string_view s, int& out) {
  const char* first = s.data();
  const char* last = s.data() + s.size();
  const auto result = std::from_chars(first, last, out);
  return result.ec == std::errc{} && result.ptr == last;
}

}  // namespace

TEST(Version, StringIsSubstitutedNotLiteral) {
  const std::string_view v = galata::version_string();
  ASSERT_FALSE(v.empty());
  EXPECT_EQ(v.find('@'), std::string_view::npos)
      << "version string is '" << v << "' — configure_file did not substitute";
}

TEST(Version, StringIsThreeComponentSemver) {
  const std::string_view v = galata::version_string();
  const std::vector<std::string_view> parts = split(v, '.');
  ASSERT_EQ(parts.size(), 3U) << "version string '" << v << "' is not major.minor.patch";
  for (const std::string_view part : parts) {
    int value = 0;
    EXPECT_TRUE(parse_int(part, value)) << "component '" << part << "' is not an integer";
    EXPECT_GE(value, 0);
  }
}

TEST(Version, ComponentsReconstructTheString) {
  const std::string rebuilt = std::to_string(galata::version_major()) + "."
                              + std::to_string(galata::version_minor()) + "."
                              + std::to_string(galata::version_patch());
  EXPECT_EQ(rebuilt, std::string(galata::version_string()));
}

TEST(Version, AbiMajorIsOne) {
  // Pinned for the life of the 1.x series (docs/adr/0001-independent-c-abi.md).
  // A change here is a major-version event, not a refactor.
  EXPECT_EQ(galata::abi_version_major(), 1);
}

TEST(Version, BuildIdentificationCarriesTheVersionAndToolchain) {
  const std::string_view id = galata::build_identification();
  ASSERT_FALSE(id.empty());
  EXPECT_EQ(id.find('@'), std::string_view::npos)
      << "build identification is '" << id << "' — configure_file did not substitute";
  EXPECT_NE(id.find(galata::version_string()), std::string_view::npos)
      << "build identification '" << id << "' does not name the version";
  EXPECT_EQ(id.rfind("galata ", 0), 0U) << "build identification should start with the tool name";
}
