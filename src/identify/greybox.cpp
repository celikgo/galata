// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the grey-box fit declared in
// include/galata/identify/greybox.hpp.

#include "galata/identify/greybox.hpp"

#include "galata/data/record.hpp"
#include "galata/numerics/integrator.hpp"

#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::identify {
namespace {

// Reads "name[i]" into its parts; a bare name has index -1.
int bracket_index(const std::string& path, std::string& base) {
  const auto open = path.find('[');
  if (open == std::string::npos) {
    base = path;
    return -1;
  }
  base = path.substr(0, open);
  return std::atoi(path.c_str() + open + 1);
}

// Points at the one double a parameter path names. Refused rather than ignored
// when it names nothing: a parameter silently not fitted is a run reporting
// success having estimated less than it was asked to.
double* resolve(model::Quadrotor& model, const std::string& path) {
  if (path == "mass.mass_kg") {
    return &model.mass.mass_kg;
  }
  std::string head;
  const auto dot = path.find('.');
  if (dot == std::string::npos) {
    return nullptr;
  }
  const std::string prefix = path.substr(0, dot);
  const std::string leaf = path.substr(dot + 1);
  if (prefix.rfind("rotors", 0) == 0) {
    std::string base;
    const int index = bracket_index(prefix, base);
    if (base != "rotors" || index < 0 || index >= model.rotor_count()) {
      return nullptr;
    }
    model::Rotor& rotor = model.rotors[static_cast<std::size_t>(index)];
    if (leaf == "thrust_coefficient_n_s2") {
      return &rotor.thrust_coefficient_n_s2;
    }
    if (leaf == "torque_coefficient_n_m_s2") {
      return &rotor.torque_coefficient_n_m_s2;
    }
    if (leaf == "speed_time_constant_s") {
      return &rotor.speed_time_constant_s;
    }
    return nullptr;
  }
  if (prefix == "drag") {
    std::string base;
    const int index = bracket_index(leaf, base);
    if (index < 0 || index > 2) {
      return nullptr;
    }
    if (base == "linear_n_s_m") {
      return &model.drag_linear_n_s_m(index);
    }
    if (base == "quadratic_n_s2_m2") {
      return &model.drag_quadratic_n_s2_m2(index);
    }
    if (base == "angular_n_m_s") {
      return &model.angular_drag_n_m_s(index);
    }
    return nullptr;
  }
  return nullptr;
}

const std::vector<double>& channel_of(const data::Record& record, const std::string& name) {
  const data::Channel* channel = record.find(name);
  if (channel == nullptr) {
    throw std::invalid_argument("identify.greybox: the record has no channel '" + name + "'");
  }
  return channel->samples;
}

}  // namespace

std::string to_string(StopReason reason) {
  switch (reason) {
    case StopReason::DeclaredIterationsCompleted:
      return "the declared iteration count completed";
  }
  return "unknown";
}

GreyboxResult fit_greybox(const model::Quadrotor& model,
                          const data::Record& record,
                          const GreyboxRequest& request) {
  if (request.parameters.empty()) {
    throw std::invalid_argument("identify.greybox: at least one parameter must be declared");
  }
  if (request.outputs.empty()) {
    throw std::invalid_argument("identify.greybox: at least one output match must be declared");
  }
  if (!(request.step_s > 0.0) || !std::isfinite(request.step_s)) {
    throw std::invalid_argument("identify.greybox: step_s must be positive and finite");
  }
  if (request.iterations <= 0) {
    throw std::invalid_argument("identify.greybox: iterations must be positive");
  }
  if (static_cast<int>(request.command_channels.size()) != model.rotor_count()) {
    throw std::invalid_argument(
        "identify.greybox: one command channel per rotor is required, in rotor order");
  }
  if (request.initial_extended_state.size() != model.extended_state_size()) {
    throw std::invalid_argument(
        "identify.greybox: the declared initial state does not match the model's width");
  }

  // Resolve every path against a scratch copy first, so a study naming
  // something the model does not have is refused before anything is simulated.
  {
    model::Quadrotor probe = model;
    for (const Parameter& parameter : request.parameters) {
      if (resolve(probe, parameter.path) == nullptr) {
        throw std::invalid_argument(
            "identify.greybox: '" + parameter.path
            + "' does not name a parameter of this model. Refused rather than ignored: a "
              "parameter silently not fitted is a run that reports success having estimated "
              "less than it was asked to");
      }
      if (!(parameter.upper > parameter.lower)) {
        throw std::invalid_argument("identify.greybox: '" + parameter.path
                                    + "' has bounds that do not bracket");
      }
      if (parameter.initial < parameter.lower || parameter.initial > parameter.upper) {
        throw std::invalid_argument("identify.greybox: '" + parameter.path
                                    + "' starts outside its own bounds");
      }
    }
  }

  const auto parameter_count = static_cast<int>(request.parameters.size());
  const auto sample_count = static_cast<int>(record.times_s.size());
  if (sample_count < 2) {
    throw std::invalid_argument("identify.greybox: the record has too few samples to simulate");
  }

  std::vector<const std::vector<double>*> commands;
  for (const std::string& name : request.command_channels) {
    commands.push_back(&channel_of(record, name));
  }
  std::vector<const std::vector<double>*> measured;
  std::vector<int> state_index;
  const std::vector<std::string> state_names = model.extended_state_names();
  for (const OutputMatch& match : request.outputs) {
    measured.push_back(&channel_of(record, match.channel));
    const auto found = std::find(state_names.begin(), state_names.end(), match.state_name);
    if (found == state_names.end()) {
      throw std::invalid_argument("identify.greybox: '" + match.state_name
                                  + "' is not a state of this model");
    }
    state_index.push_back(static_cast<int>(found - state_names.begin()));
    if (!(match.scale > 0.0)) {
      throw std::invalid_argument(
          "identify.greybox: output '" + match.channel
          + "' needs a positive scale. Without one, channels in different units are added "
            "together as though a metre and a radian per second were the same size, which is a "
            "weighting decision made by accident");
    }
  }

  // Simulate and return the scaled residual vector. This is the ONLY place the
  // plant is evaluated, and it uses `Quadrotor::derivative` and
  // `numerics::integrate_fixed_step` — the same pair `sim.plant` uses, so a fit
  // and a simulation cannot disagree about what the model does.
  const auto residuals = [&](const Eigen::VectorXd& theta) {
    model::Quadrotor candidate = model;
    for (int p = 0; p < parameter_count; ++p) {
      *resolve(candidate, request.parameters[static_cast<std::size_t>(p)].path) = theta(p);
    }
    // Validity is enforced at EVERY evaluation, not only at the end: an
    // optimiser that walks through an invalid model on its way somewhere has
    // evaluated an objective that means nothing.
    //
    // The rethrow names the point it failed at. Without it the caller is told
    // that some quadrotor somewhere was invalid, which is true of a model it
    // never wrote and cannot see; with it, the study can read which declared
    // parameter reached which value and widen or narrow the bound that let it.
    try {
      candidate.validate();
    } catch (const std::exception& error) {
      std::ostringstream message;
      message << "identify.greybox: the search reached a model the plant refuses — " << error.what()
              << ". The point was";
      for (int p = 0; p < parameter_count; ++p) {
        message << (p == 0 ? " " : ", ") << request.parameters[static_cast<std::size_t>(p)].path
                << " = " << theta(p);
      }
      message << ". Declared bounds admit this point, so narrow the bound that does";
      throw std::invalid_argument(message.str());
    }

    Eigen::VectorXd out(sample_count * static_cast<int>(request.outputs.size()));
    Eigen::VectorXd state = request.initial_extended_state;
    Eigen::VectorXd command(candidate.rotor_count());
    int row = 0;
    for (int k = 0; k < sample_count; ++k) {
      for (std::size_t o = 0; o < request.outputs.size(); ++o) {
        const double predicted = state(state_index[o]);
        const double observed = (*measured[o])[static_cast<std::size_t>(k)];
        out(row++) = (predicted - observed) / request.outputs[o].scale;
      }
      if (k + 1 < sample_count) {
        for (int r = 0; r < candidate.rotor_count(); ++r) {
          command(r) = (*commands[static_cast<std::size_t>(r)])[static_cast<std::size_t>(k)];
        }
        const double span = record.times_s[static_cast<std::size_t>(k) + 1]
                            - record.times_s[static_cast<std::size_t>(k)];
        const auto steps = static_cast<int>(std::llround(span / request.step_s));
        const numerics::DerivativeFunction derivative = [&](double, const Eigen::VectorXd& x) {
          return candidate.derivative(x, command);
        };
        const numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) {
          candidate.project(x);
        };
        state = numerics::integrate_fixed_step(derivative,
                                               state,
                                               0.0,
                                               request.step_s,
                                               std::max(1, steps),
                                               std::max(1, steps),
                                               projection)
                    .states.back();
      }
    }
    return out;
  };

  Eigen::VectorXd theta(parameter_count);
  for (int p = 0; p < parameter_count; ++p) {
    theta(p) = request.parameters[static_cast<std::size_t>(p)].initial;
  }

  Eigen::VectorXd residual = residuals(theta);
  double objective = residual.squaredNorm();
  const double initial_objective = objective;
  int accepted_steps = 0;
  int last_accepted_iteration = -1;
  int iterations_run = 0;
  Eigen::MatrixXd jacobian(residual.size(), parameter_count);
  double damping = 1e-3;
  double last_step = 0.0;

  // Levenberg-Marquardt, a DECLARED number of iterations. Not "until
  // converged": ADR-0004 wants the same bits every run, and an optimiser that
  // stops when it is close enough stops after a different number of steps on a
  // different machine.
  for (int iteration = 0; iteration < request.iterations; ++iteration) {
    ++iterations_run;
    for (int p = 0; p < parameter_count; ++p) {
      const double scale = std::fmax(std::fabs(theta(p)), 1e-8);
      const double step = 1e-6 * scale;
      Eigen::VectorXd forward = theta;
      forward(p) += step;
      jacobian.col(p) = (residuals(forward) - residual) / step;
    }
    const Eigen::MatrixXd normal =
        jacobian.transpose() * jacobian
        + damping * Eigen::MatrixXd::Identity(parameter_count, parameter_count);
    const Eigen::VectorXd delta = normal.ldlt().solve(-jacobian.transpose() * residual);
    Eigen::VectorXd trial = theta + delta;
    // Bounds are enforced on every trial point, so no evaluation ever happens
    // at a model the study declared impossible.
    for (int p = 0; p < parameter_count; ++p) {
      const Parameter& bound = request.parameters[static_cast<std::size_t>(p)];
      trial(p) = std::clamp(trial(p), bound.lower, bound.upper);
    }
    const Eigen::VectorXd trial_residual = residuals(trial);
    const double trial_objective = trial_residual.squaredNorm();
    if (trial_objective < objective) {
      ++accepted_steps;
      last_accepted_iteration = iteration;
      last_step = (trial - theta).norm();
      theta = trial;
      residual = trial_residual;
      objective = trial_objective;
      damping = std::fmax(damping * 0.5, 1e-12);
    } else {
      damping = std::fmin(damping * 4.0, 1e12);
      last_step = 0.0;
    }
  }

  // THE JACOBIAN IS RECOMPUTED AT THE POINT BEING REPORTED. The one the loop
  // left behind was taken at the START of the last iteration, before its step
  // was accepted or rejected, so a condition number, a covariance or a gradient
  // read off it describes a point the caller is not being handed. Every figure
  // below is therefore taken from a fresh Jacobian at the final `theta`, at the
  // cost of one more Jacobian evaluation — which is the right price for a
  // diagnostic that claims to be about the answer.
  for (int p = 0; p < parameter_count; ++p) {
    const double scale = std::fmax(std::fabs(theta(p)), 1e-8);
    const double step = 1e-6 * scale;
    Eigen::VectorXd forward = theta;
    forward(p) += step;
    jacobian.col(p) = (residuals(forward) - residual) / step;
  }
  const Eigen::VectorXd gradient = jacobian.transpose() * residual;

  GreyboxResult result;
  // The plant the optimiser stopped at, built by the same `resolve` the
  // objective used, so the model the caller receives and the model the last
  // residual was evaluated on cannot differ. Validated here for the same reason
  // every trial was: a fit must not hand back a model that would be refused if
  // somebody tried to load it.
  result.fitted_model = model;
  for (int p = 0; p < parameter_count; ++p) {
    *resolve(result.fitted_model, request.parameters[static_cast<std::size_t>(p)].path) = theta(p);
  }
  result.fitted_model.validate();

  result.iterations_declared = request.iterations;
  result.iterations_run = iterations_run;
  result.stop_reason = StopReason::DeclaredIterationsCompleted;
  result.objective = objective;
  result.initial_objective = initial_objective;
  result.accepted_steps = accepted_steps;
  result.objective_improved = objective < initial_objective;
  result.residual_count = static_cast<int>(residual.size());
  result.residual_rms = std::sqrt(objective / static_cast<double>(residual.size()));
  result.convergence.last_step_norm = last_step;
  result.convergence.last_accepted_iteration = last_accepted_iteration;
  result.convergence.gradient_infinity_norm =
      parameter_count > 0 ? gradient.cwiseAbs().maxCoeff() : 0.0;
  double scaled_gradient = 0.0;
  for (int p = 0; p < parameter_count; ++p) {
    const Parameter& bound = request.parameters[static_cast<std::size_t>(p)];
    scaled_gradient =
        std::fmax(scaled_gradient, std::fabs(gradient(p)) * (bound.upper - bound.lower));
  }
  result.convergence.gradient_over_bound_span_infinity_norm = scaled_gradient;
  result.identifiability_ratio = request.identifiability_ratio;
  result.value = theta;
  result.standard_error = Eigen::VectorXd::Zero(parameter_count);
  for (int p = 0; p < parameter_count; ++p) {
    const Parameter& bound = request.parameters[static_cast<std::size_t>(p)];
    result.names.push_back(bound.path);
    const double span = bound.upper - bound.lower;
    result.at_bound.push_back(std::fabs(theta(p) - bound.lower) < 1e-9 * span
                              || std::fabs(theta(p) - bound.upper) < 1e-9 * span);
  }

  // IDENTIFIABILITY IS A SEPARATE QUESTION FROM CONVERGENCE. A direction the
  // Jacobian cannot see is a parameter this record did not measure, however
  // small the residual became — the objective is flat along it, so the
  // optimiser is equally happy anywhere in that direction and the number it
  // returned is where it happened to stop.
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const double largest = svd.singularValues()(0);
  const double smallest = svd.singularValues()(parameter_count - 1);
  result.jacobian_condition_number =
      smallest > 0.0 ? largest / smallest : std::numeric_limits<double>::infinity();
  if (!(smallest > request.identifiability_ratio * largest)) {
    std::ostringstream message;
    message << "identify.greybox: the data does not constrain every declared parameter. The "
               "sensitivity matrix has condition number "
            << result.jacobian_condition_number
            << ", so at least one direction in parameter "
               "space changes the prediction by nothing this record can see. The optimiser "
               "finished — that is not the same thing, and reporting its answer for an "
               "unmeasured direction would be reporting where it happened to stop. Excite the "
               "unidentified parameter, or hold it at its declared value and fit the rest";
    throw std::invalid_argument(message.str());
  }

  const int degrees_of_freedom = static_cast<int>(residual.size()) - parameter_count;
  if (degrees_of_freedom > 0 && result.residual_rms > 0.0) {
    const double variance = objective / static_cast<double>(degrees_of_freedom);
    const Eigen::MatrixXd covariance =
        (jacobian.transpose() * jacobian)
            .ldlt()
            .solve(Eigen::MatrixXd::Identity(parameter_count, parameter_count))
        * variance;
    for (int p = 0; p < parameter_count; ++p) {
      result.standard_error(p) = std::sqrt(std::fmax(0.0, covariance(p, p)));
    }
    result.uncertainty_is_estimable = true;
    result.note =
        "standard errors are the Gauss-Newton approximation at the final point, valid to the "
        "extent the residual is locally linear in the parameters and the measurement errors are "
        "independent with equal variance in the SCALED units the study declared";
  } else {
    result.note =
        "no uncertainty: the residual is at the arithmetic floor or there are as many "
        "parameters as residuals, so the record demonstrates no scatter to estimate from";
  }

  // THE MODEL SAYS WHAT IT IS. A file written from this plant is byte-for-byte
  // the same kind of object as one written from a bench measurement, and nothing
  // in the format tells them apart — so the two free-text fields the format does
  // have are used to say it, at the moment the fitted model comes into
  // existence rather than at whatever later point somebody remembers to. A
  // library caller who wants different words can overwrite them; a library
  // caller who forgets does not thereby publish a fitted model wearing its base
  // model's description.
  {
    std::ostringstream described;
    described << model.description;
    if (!model.description.empty()) {
      described << " — ";
    }
    described << result.names.size() << " parameter(s) identified against a measured record";
    result.fitted_model.description = described.str();

    std::ostringstream cited;
    cited << "identified by identify.greybox from ";
    if (!record.source_path.empty()) {
      cited << "'" << record.source_path << "', ";
    }
    cited << "sha256 "
          << (record.source_sha256.empty() ? std::string("unrecorded") : record.source_sha256)
          << ", over " << record.sample_count() << " sample(s), fitting";
    for (std::size_t p = 0; p < result.names.size(); ++p) {
      cited << (p == 0 ? " " : ", ") << result.names[p];
    }
    cited << ". Every other parameter is the value the base model gave it";
    if (!model.citation.empty()) {
      cited << ", whose own source is: " << model.citation;
    }
    cited << ". A fitted model is not measured aircraft data and a completed fit is not a "
             "validation";
    result.fitted_model.citation = cited.str();
  }
  return result;
}

}  // namespace galata::identify
