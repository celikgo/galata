// SPDX-License-Identifier: Apache-2.0
//
// Deterministic sampled measurements for offline control studies.
//
// The generator, stream derivation and normal transform are deliberately
// named. A sensor sample is produced only when the caller asks at a declared
// sampling instant; ODE stages, numerical Jacobians and implicit iterations
// never consume the stream.
//
// WHAT THIS IS NOT. Not an estimator, a Kalman filter, a coloured-noise model
// or a claim about a real sensor. It is a reproducible measurement impairment
// primitive whose seed and configuration belong in the run record.
#ifndef GALATA_SIM_SENSOR_HPP
#define GALATA_SIM_SENSOR_HPP

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace galata::sim {

struct SensorConfiguration {
  std::string name = "sensor";
  std::vector<std::string> channel_names;
  std::uint64_t seed = 0;
  std::string algorithm_version = "mt19937_64_box_muller_v1";
  double sample_period_s = 0.0;  // s
  double latency_s = 0.0;        // s
  Eigen::VectorXd bias;
  Eigen::VectorXd white_noise_stddev;
  double quantization_step = 0.0;  // source units; zero disables quantisation
  double saturation_min = -std::numeric_limits<double>::infinity();
  double saturation_max = std::numeric_limits<double>::infinity();
  std::vector<int> dropout_samples;
  std::string dropout_policy = "unavailable";  // unavailable or hold_last
};

struct SensorReading {
  bool available = false;
  bool stale = false;
  int sample_index = -1;
  double time_s = 0.0;  // s
  Eigen::VectorXd values;
};

class DeterministicSensor {
 public:
  // The integration step is part of the construction contract so a sensor
  // period and latency cannot be rounded differently by two callers.
  DeterministicSensor(SensorConfiguration configuration, double integration_step_s);

  // Samples only when integration_step_index is on the declared sensor clock.
  // Calling this at every ODE stage is harmless: non-sample boundaries do not
  // advance any stream. The returned reading is the latest delivered sample.
  [[nodiscard]] SensorReading sample_if_due(int integration_step_index,
                                            double time_s,
                                            const Eigen::VectorXd& truth);

  [[nodiscard]] const SensorConfiguration& configuration() const noexcept {
    return configuration_;
  }

  [[nodiscard]] int sample_steps() const noexcept {
    return sample_steps_;
  }

  [[nodiscard]] int latency_samples() const noexcept {
    return latency_samples_;
  }

  [[nodiscard]] const SensorReading& latest() const noexcept {
    return latest_;
  }

  [[nodiscard]] std::vector<std::string> stream_ids() const;

 private:
  struct PendingSample {
    int index = 0;
    double time_s = 0.0;
    Eigen::VectorXd values;
  };

  SensorConfiguration configuration_;
  int sample_steps_ = 1;
  int latency_samples_ = 0;
  int next_sample_index_ = 0;
  SensorReading latest_;
  std::vector<PendingSample> pending_;
  std::vector<std::uint64_t> stream_seeds_;
};

}  // namespace galata::sim

#endif  // GALATA_SIM_SENSOR_HPP
