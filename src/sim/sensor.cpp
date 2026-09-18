// SPDX-License-Identifier: Apache-2.0
//
// Deterministic sensor streams. The algorithm is intentionally implemented
// here rather than delegated to implementation-defined distribution classes.

#include "galata/sim/sensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace galata::sim {
namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;  // GALATA_SI_EXEMPT: algorithm constant

std::uint64_t splitmix64(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

std::uint64_t hash_text(const std::string& text) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const char character : text) {
    hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(character));
    hash *= 1099511628211ULL;
  }
  return hash;
}

double unit_open(std::mt19937_64& generator) {
  // The top 53 bits are the same construction on every implementation of
  // mt19937_64. The +0.5 avoids log(0) without relying on a distribution class.
  const std::uint64_t bits = generator() >> 11;
  return (static_cast<double>(bits) + 0.5) / 9007199254740992.0;
}

double normal_sample(std::mt19937_64& generator) {
  const double u = unit_open(generator);
  const double v = unit_open(generator);
  return std::sqrt(-2.0 * std::log(u)) * std::cos(kTwoPi * v);
}

void require_finite_vector(const Eigen::VectorXd& vector, const char* name) {
  if (!vector.allFinite()) {
    throw std::invalid_argument(std::string("sensor: ") + name + " must be finite");
  }
}

int lattice_count(double period_s, double integration_step_s, const char* name) {
  const double ratio = period_s / integration_step_s;
  const double nearest = std::round(ratio);
  if (!std::isfinite(ratio) || nearest < 1.0
      || std::fabs(ratio - nearest) > 1.0e-12 * std::fmax(1.0, std::fabs(ratio))) {
    std::ostringstream message;
    message << "sensor: " << name << " = " << period_s << " s is not a positive whole number of "
            << integration_step_s << " s integration steps; sampling is not rounded silently";
    throw std::invalid_argument(message.str());
  }
  if (nearest > static_cast<double>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument(std::string("sensor: ") + name + " is too large");
  }
  return static_cast<int>(nearest);
}

}  // namespace

DeterministicSensor::DeterministicSensor(SensorConfiguration configuration,
                                         double integration_step_s)
    : configuration_(std::move(configuration)) {
  const int width = static_cast<int>(configuration_.channel_names.size());
  if (width < 1) {
    throw std::invalid_argument("sensor: at least one named channel is required");
  }
  if (!(integration_step_s > 0.0) || !std::isfinite(integration_step_s)) {
    throw std::invalid_argument("sensor: integration step must be positive and finite");
  }
  if (!(configuration_.sample_period_s > 0.0) || !std::isfinite(configuration_.sample_period_s)) {
    throw std::invalid_argument("sensor: sample_period_s must be positive and finite");
  }
  if (!(configuration_.latency_s >= 0.0) || !std::isfinite(configuration_.latency_s)) {
    throw std::invalid_argument("sensor: latency_s must be finite and non-negative");
  }
  if (configuration_.algorithm_version != "mt19937_64_box_muller_v1") {
    throw std::invalid_argument(
        "sensor: algorithm_version must be `mt19937_64_box_muller_v1`; the generator is not an "
        "unversioned standard-library distribution");
  }
  if (configuration_.bias.size() == 0) {
    configuration_.bias = Eigen::VectorXd::Zero(width);
  }
  if (configuration_.white_noise_stddev.size() == 0) {
    configuration_.white_noise_stddev = Eigen::VectorXd::Zero(width);
  }
  if (configuration_.bias.size() != width || configuration_.white_noise_stddev.size() != width) {
    throw std::invalid_argument("sensor: bias and white_noise_stddev must match channel count");
  }
  require_finite_vector(configuration_.bias, "bias");
  require_finite_vector(configuration_.white_noise_stddev, "white_noise_stddev");
  if ((configuration_.white_noise_stddev.array() < 0.0).any()) {
    throw std::invalid_argument("sensor: white_noise_stddev must be non-negative");
  }
  if (!(configuration_.quantization_step >= 0.0)
      || !std::isfinite(configuration_.quantization_step)) {
    throw std::invalid_argument("sensor: quantization_step must be finite and non-negative");
  }
  if (!std::isfinite(configuration_.saturation_min)
      && configuration_.saturation_min != -std::numeric_limits<double>::infinity()) {
    throw std::invalid_argument("sensor: saturation_min must be finite or negative infinity");
  }
  if (!std::isfinite(configuration_.saturation_max)
      && configuration_.saturation_max != std::numeric_limits<double>::infinity()) {
    throw std::invalid_argument("sensor: saturation_max must be finite or positive infinity");
  }
  if (!(configuration_.saturation_max > configuration_.saturation_min)) {
    throw std::invalid_argument("sensor: saturation_max must exceed saturation_min");
  }
  if (configuration_.dropout_policy != "unavailable"
      && configuration_.dropout_policy != "hold_last") {
    throw std::invalid_argument("sensor: dropout_policy must be `unavailable` or `hold_last`");
  }
  sample_steps_ =
      lattice_count(configuration_.sample_period_s, integration_step_s, "sample_period_s");
  latency_samples_ =
      configuration_.latency_s == 0.0
          ? 0
          : lattice_count(configuration_.latency_s, configuration_.sample_period_s, "latency_s");
  std::sort(configuration_.dropout_samples.begin(), configuration_.dropout_samples.end());
  if (std::adjacent_find(configuration_.dropout_samples.begin(),
                         configuration_.dropout_samples.end())
      != configuration_.dropout_samples.end()) {
    throw std::invalid_argument("sensor: dropout_samples must not contain duplicates");
  }
  for (const int sample : configuration_.dropout_samples) {
    if (sample < 0) {
      throw std::invalid_argument("sensor: dropout_samples must be non-negative sample indices");
    }
  }
  stream_seeds_.reserve(static_cast<std::size_t>(width));
  for (const auto& channel : configuration_.channel_names) {
    stream_seeds_.push_back(
        splitmix64(configuration_.seed ^ hash_text(configuration_.name + "/" + channel)));
  }
  latest_.values = Eigen::VectorXd::Zero(width);
}

SensorReading DeterministicSensor::sample_if_due(int integration_step_index,
                                                 double time_s,
                                                 const Eigen::VectorXd& truth) {
  if (integration_step_index < 0 || !std::isfinite(time_s)) {
    throw std::invalid_argument("sensor: sample time and integration index must be valid");
  }
  if (truth.size() != static_cast<Eigen::Index>(configuration_.channel_names.size())
      || !truth.allFinite()) {
    throw std::invalid_argument("sensor: truth vector does not match the named channels");
  }
  if (integration_step_index % sample_steps_ != 0) {
    return latest_;
  }
  const int sample_index = integration_step_index / sample_steps_;
  if (sample_index != next_sample_index_) {
    throw std::logic_error("sensor: samples must be presented in increasing clock order");
  }
  Eigen::VectorXd measured(truth.size());
  for (Eigen::Index i = 0; i < truth.size(); ++i) {
    std::mt19937_64 generator(stream_seeds_[static_cast<std::size_t>(i)]
                              ^ splitmix64(static_cast<std::uint64_t>(sample_index)));
    measured(i) = truth(i) + configuration_.bias(i)
                  + configuration_.white_noise_stddev(i) * normal_sample(generator);
    if (configuration_.quantization_step > 0.0) {
      measured(i) = configuration_.quantization_step
                    * std::round(measured(i) / configuration_.quantization_step);
    }
    measured(i) =
        std::clamp(measured(i), configuration_.saturation_min, configuration_.saturation_max);
  }
  pending_.push_back(PendingSample{sample_index, time_s, std::move(measured)});
  ++next_sample_index_;
  if (static_cast<int>(pending_.size()) <= latency_samples_) {
    latest_.available = false;
    latest_.stale = false;
    latest_.sample_index = sample_index;
    latest_.time_s = time_s;
    return latest_;
  }
  const PendingSample delivered = std::move(pending_.front());
  pending_.erase(pending_.begin());
  const bool dropped = std::binary_search(configuration_.dropout_samples.begin(),
                                          configuration_.dropout_samples.end(),
                                          delivered.index);
  latest_.sample_index = delivered.index;
  latest_.time_s = delivered.time_s;
  if (dropped) {
    latest_.stale = true;
    if (configuration_.dropout_policy == "unavailable" || !latest_.available) {
      latest_.available = false;
    }
    return latest_;
  }
  latest_.available = true;
  latest_.stale = false;
  latest_.values = delivered.values;
  return latest_;
}

std::vector<std::string> DeterministicSensor::stream_ids() const {
  std::vector<std::string> result;
  result.reserve(configuration_.channel_names.size());
  for (const auto& channel : configuration_.channel_names) {
    result.push_back(configuration_.name + "/" + channel);
  }
  return result;
}

}  // namespace galata::sim
