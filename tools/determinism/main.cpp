// SPDX-License-Identifier: Apache-2.0
//
// Prints the determinism fingerprint on stdout.
//
// The battery itself lives in fingerprint.cpp, so that the V&V report can count
// what this gate covers without that number being typed by hand. What remains
// here is the formatting and nothing else.

#include "galata/version.hpp"

#include "fingerprint.hpp"

#include <cstdio>
#include <string>

int main() {
  // The build identification is deliberately NOT in the fingerprint: it names
  // the compiler, and this file is compared across compilers.
  std::printf("# galata determinism fingerprint, version %s\n",
              std::string(galata::version_string()).c_str());

  // %.17g round-trips a double exactly, so a byte-identical fingerprint means
  // bit-identical values and not merely values that print the same.
  galata::determinism::fingerprint(
      GALATA_DETERMINISM_MODEL,
      [](const std::string& key, double value) { std::printf("%s\t%.17g\n", key.c_str(), value); });
  return 0;
}
