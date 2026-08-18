// SPDX-License-Identifier: Apache-2.0
//
// Deterministic generators for property-based tests.
//
// No property-testing framework, and no std:: distributions. std::mt19937_64
// is specified bit-exactly by the C++ standard, but std::uniform_real_distribution
// and friends are NOT — their outputs are implementation-defined, so the same
// seed produces different samples on libstdc++ and MSVC. That would make a
// property-test failure unreproducible across platforms, which is the one thing
// a property test must not be (ADR-0004).
//
// So the engine is the standard's, and everything built on it is here.

#ifndef GALATA_TESTS_PROPERTY_GENERATORS_HPP
#define GALATA_TESTS_PROPERTY_GENERATORS_HPP

#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <random>

namespace galata::testing {

// Number of samples each property is asserted over. Large enough that a sign
// error in one octant cannot hide, small enough that the suite stays fast.
inline constexpr int kPropertySamples = 2000;

class Generator {
 public:
  // The seed is part of the test's identity: a failure message prints it and
  // anyone can replay the exact sequence.
  explicit Generator(std::uint64_t seed) : engine_(seed), seed_(seed) {}

  [[nodiscard]] std::uint64_t seed() const noexcept {
    return seed_;
  }

  // Uniform in [0, 1). Built from the top 53 bits, which is the standard
  // construction and is exact in double.
  [[nodiscard]] double unit();

  // Uniform in [low, high).
  [[nodiscard]] double range(double low, double high);

  // Standard normal, via Box-Muller. Used only to build uniformly-distributed
  // rotations; the normal-ness itself is never asserted.
  [[nodiscard]] double normal();

  [[nodiscard]] Eigen::Vector3d vector3(double magnitude);

  // Uniformly distributed over the rotation group, not merely "some random
  // numbers normalised" — a rotation sampler biased toward the identity would
  // never exercise the large-angle branches this suite exists to cover.
  [[nodiscard]] galata::core::Quaternion rotation();

  [[nodiscard]] galata::core::State state();

 private:
  std::mt19937_64 engine_;
  std::uint64_t seed_;
};

}  // namespace galata::testing

#endif  // GALATA_TESTS_PROPERTY_GENERATORS_HPP
