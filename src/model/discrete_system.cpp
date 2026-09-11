// SPDX-License-Identifier: Apache-2.0

#include "galata/model/discrete_system.hpp"

#include "galata/numerics/matrix_exponential.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace galata::model {

const char* to_string(InputHold hold) {
  switch (hold) {
    case InputHold::ZeroOrder:
      return "zero_order_hold";
  }
  return "unknown";
}

void DiscreteLinearSystem::validate() const {
  if (a.rows() == 0) {
    throw std::invalid_argument("discrete system: A is empty");
  }
  if (a.rows() != a.cols()) {
    std::ostringstream message;
    message << "discrete system: A is " << a.rows() << "x" << a.cols() << ", must be square";
    throw std::invalid_argument(message.str());
  }
  // The sample time is checked FIRST among the scalar fields because every
  // other reading of this model depends on it: a rate, a Nyquist frequency, a
  // simulated duration. A model that reached a report without one would put
  // sample indices where a reader expects seconds.
  if (!(sample_time_s > 0.0) || !std::isfinite(sample_time_s)) {
    std::ostringstream message;
    message << "discrete system: the sample time is " << sample_time_s
            << ", which is not a positive finite number of seconds. A discrete model "
               "without its sample time cannot be simulated at a rate or compared with a "
               "continuous one, so it is refused rather than defaulted.";
    throw std::invalid_argument(message.str());
  }
  if (static_cast<Eigen::Index>(state_names.size()) != a.rows()) {
    std::ostringstream message;
    message << "discrete system: " << state_names.size() << " state names for a " << a.rows()
            << "-state model. Every state must be named.";
    throw std::invalid_argument(message.str());
  }
  const std::set<std::string> unique(state_names.begin(), state_names.end());
  if (unique.size() != state_names.size()) {
    throw std::invalid_argument("discrete system: state names are not unique");
  }
  if (b.size() != 0) {
    if (b.rows() != a.rows()) {
      std::ostringstream message;
      message << "discrete system: B has " << b.rows() << " rows, A has " << a.rows();
      throw std::invalid_argument(message.str());
    }
    if (static_cast<Eigen::Index>(input_names.size()) != b.cols()) {
      std::ostringstream message;
      message << "discrete system: " << input_names.size() << " input names for " << b.cols()
              << " columns of B";
      throw std::invalid_argument(message.str());
    }
  }
  if (c.size() != 0 && c.cols() != a.rows()) {
    std::ostringstream message;
    message << "discrete system: C has " << c.cols() << " columns, A has " << a.rows();
    throw std::invalid_argument(message.str());
  }
  if (d.size() != 0) {
    if (c.size() != 0 && d.rows() != c.rows()) {
      std::ostringstream message;
      message << "discrete system: D has " << d.rows() << " rows, C has " << c.rows();
      throw std::invalid_argument(message.str());
    }
    if (b.size() != 0 && d.cols() != b.cols()) {
      std::ostringstream message;
      message << "discrete system: D has " << d.cols() << " columns, B has " << b.cols();
      throw std::invalid_argument(message.str());
    }
  }
  if (!a.allFinite() || (b.size() != 0 && !b.allFinite())) {
    throw std::invalid_argument("discrete system: A or B has a non-finite entry");
  }
}

Eigen::MatrixXd DiscreteLinearSystem::output_matrix() const {
  if (c.size() == 0) {
    return Eigen::MatrixXd::Identity(a.rows(), a.rows());
  }
  return c;
}

Eigen::MatrixXd DiscreteLinearSystem::feedthrough_matrix() const {
  if (d.size() != 0) {
    return d;
  }
  return Eigen::MatrixXd::Zero(output_count(), input_count());
}

std::vector<std::string> DiscreteLinearSystem::output_labels() const {
  if (!output_names.empty()) {
    return output_names;
  }
  return state_names;
}

Discretisation discretize_zoh(const LinearSystem& system, double sample_time_s) {
  system.validate();
  if (!(sample_time_s > 0.0) || !std::isfinite(sample_time_s)) {
    std::ostringstream message;
    message << "discretize_zoh: the sample time is " << sample_time_s
            << ", which is not a positive finite number of seconds";
    throw std::invalid_argument(message.str());
  }
  if (system.input_count() == 0) {
    throw std::invalid_argument(
        "discretize_zoh: the system has no inputs, so there is nothing for a hold to hold. "
        "Discretising the state transition alone is expm(A T) and is available directly from "
        "galata::numerics::matrix_exponential.");
  }

  const Eigen::Index n = system.state_count();
  const Eigen::Index m = system.input_count();

  // VAN LOAN'S BLOCK MATRIX. exp([[A, B], [0, 0]] T) = [[Ad, Bd], [0, I]], so
  // one exponential gives both blocks and gives Bd = (integral of exp(A s) ds) B
  // EXACTLY, rather than as a series whose truncation nobody stated. The zero
  // bottom rows are what make the top-right block the integral.
  Eigen::MatrixXd block = Eigen::MatrixXd::Zero(n + m, n + m);
  block.topLeftCorner(n, n) = system.a;
  block.topRightCorner(n, m) = system.b;
  block *= sample_time_s;

  const auto exponential = numerics::matrix_exponential(block);

  Discretisation result;
  DiscreteLinearSystem& discrete = result.system;
  discrete.a = exponential.value.topLeftCorner(n, n);
  discrete.b = exponential.value.topRightCorner(n, m);
  discrete.c = system.c;
  discrete.d = system.d;
  discrete.sample_time_s = sample_time_s;
  discrete.hold = InputHold::ZeroOrder;
  discrete.state_names = system.state_names;
  discrete.input_names = system.input_names;
  discrete.output_names = system.output_names;
  discrete.description = system.description;
  discrete.citation = system.citation;
  discrete.units = system.units;
  discrete.validate();

  DiscretisationEvidence& evidence = result.evidence;
  evidence.sample_time_s = sample_time_s;
  evidence.hold = InputHold::ZeroOrder;
  evidence.exponential_one_norm = exponential.one_norm;
  evidence.exponential_pade_order = exponential.pade_order;
  evidence.exponential_squarings = exponential.squarings;
  evidence.exponential_error_bound = exponential.backward_error_bound;

  const Eigen::EigenSolver<Eigen::MatrixXd> discrete_spectrum(discrete.a);
  double radius = 0.0;
  for (Eigen::Index i = 0; i < discrete_spectrum.eigenvalues().size(); ++i) {
    const std::complex<double> value = discrete_spectrum.eigenvalues()(i);
    evidence.discrete_eigenvalues.push_back(value);
    radius = std::max(radius, std::abs(value));
  }
  evidence.spectral_radius = radius;

  const Eigen::EigenSolver<Eigen::MatrixXd> continuous_spectrum(system.a);
  double fastest = 0.0;
  for (Eigen::Index i = 0; i < continuous_spectrum.eigenvalues().size(); ++i) {
    fastest = std::max(fastest, std::abs(continuous_spectrum.eigenvalues()(i)));
  }
  evidence.fastest_mode_rad_s = fastest;
  // GALATA_SI_EXEMPT: pi is the half-turn in radians, not a unit conversion.
  // The Nyquist angular frequency of a sampler is pi / T rad/s by definition.
  evidence.nyquist_rad_s = 3.14159265358979323846 / sample_time_s;

  std::ostringstream assumptions;
  assumptions << "Zero-order hold at " << sample_time_s << " s (" << discrete.sample_rate_hz()
              << " Hz), exact for a constant input across each interval, by the exponential "
                 "of Van Loan's block matrix. Describes the state at the ticks only and says "
                 "nothing about the response between them. No computational or transport "
                 "delay is included. Discretisation does not filter: the fastest continuous "
                 "mode is "
              << fastest << " rad/s against a Nyquist frequency of " << evidence.nyquist_rad_s
              << " rad/s, and a mode above that aliases to a slower one this model shows as "
                 "real dynamics.";
  evidence.assumptions = assumptions.str();

  return result;
}

}  // namespace galata::model
