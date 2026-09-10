// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the held-out validation declared in
// include/galata/identify/validate.hpp.

#include "galata/identify/validate.hpp"

#include "galata/data/record.hpp"
#include "galata/numerics/integrator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace galata::identify {
namespace {

const std::vector<double>& channel_of(const data::Record& record, const std::string& name) {
  const data::Channel* channel = record.find(name);
  if (channel == nullptr) {
    throw std::invalid_argument("identify.validate: the record has no channel '" + name + "'");
  }
  return channel->samples;
}

}  // namespace

ValidationResult validate_model(const model::Quadrotor& model,
                                const data::Record& record,
                                const ValidationRequest& request) {
  if (request.estimation_record_sha256.empty()) {
    throw std::invalid_argument(
        "identify.validate: the estimation record's digest is required. A caller who cannot say "
        "which record trained the model cannot claim anything was held out from it, and this "
        "will not accept the claim on trust");
  }
  if (request.outputs.empty()) {
    throw std::invalid_argument("identify.validate: at least one output must be declared");
  }
  if (!(request.step_s > 0.0) || !std::isfinite(request.step_s)) {
    throw std::invalid_argument("identify.validate: step_s must be positive and finite");
  }
  if (static_cast<int>(request.command_channels.size()) != model.rotor_count()) {
    throw std::invalid_argument(
        "identify.validate: one command channel per rotor is required, in rotor order");
  }
  if (request.initial_extended_state.size() != model.extended_state_size()) {
    throw std::invalid_argument(
        "identify.validate: the declared initial state does not match the model's width");
  }
  const auto samples = static_cast<int>(record.times_s.size());
  if (samples < 2) {
    throw std::invalid_argument("identify.validate: the record has too few samples");
  }

  std::vector<const std::vector<double>*> commands;
  for (const std::string& name : request.command_channels) {
    commands.push_back(&channel_of(record, name));
  }
  const std::vector<std::string> state_names = model.extended_state_names();
  std::vector<const std::vector<double>*> measured;
  std::vector<int> state_index;
  for (const auto& match : request.outputs) {
    measured.push_back(&channel_of(record, match.channel));
    const auto found = std::find(state_names.begin(), state_names.end(), match.state_name);
    if (found == state_names.end()) {
      throw std::invalid_argument("identify.validate: '" + match.state_name
                                  + "' is not a state of this model");
    }
    state_index.push_back(static_cast<int>(found - state_names.begin()));
  }

  // Simulate once, through the same plant and integrator the fit and `sim.plant`
  // use, so a validation cannot disagree with either about what the model does.
  const auto output_count = request.outputs.size();
  std::vector<std::vector<double>> predicted(output_count);
  Eigen::VectorXd state = request.initial_extended_state;
  Eigen::VectorXd command(model.rotor_count());
  for (int k = 0; k < samples; ++k) {
    for (std::size_t o = 0; o < output_count; ++o) {
      predicted[o].push_back(state(state_index[o]));
    }
    if (k + 1 < samples) {
      for (int r = 0; r < model.rotor_count(); ++r) {
        command(r) = (*commands[static_cast<std::size_t>(r)])[static_cast<std::size_t>(k)];
      }
      const double span = record.times_s[static_cast<std::size_t>(k) + 1]
                          - record.times_s[static_cast<std::size_t>(k)];
      const auto steps = std::max(1, static_cast<int>(std::llround(span / request.step_s)));
      const numerics::DerivativeFunction derivative = [&](double, const Eigen::VectorXd& x) {
        return model.derivative(x, command);
      };
      const numerics::ProjectionFunction projection = [&](Eigen::VectorXd& x) { model.project(x); };
      state = numerics::integrate_fixed_step(
                  derivative, state, 0.0, request.step_s, steps, steps, projection)
                  .states.back();
    }
  }

  ValidationResult result;
  result.sample_count = samples;
  result.estimation_record_sha256 = request.estimation_record_sha256;
  result.validation_record_sha256 = record.source_sha256;
  // The one comparison this capability exists to make. Equal digests mean the
  // model is being run on the data it was fitted to, which is a diagnostic and
  // not validation, and the difference is a label rather than a refusal.
  result.is_held_out = record.source_sha256 != request.estimation_record_sha256;
  result.assumptions =
      "errors are compared sample by sample against the record's own values; the fit fraction "
      "is 1 - ||y - yhat|| / ||y - mean(y)||, so zero means no better than predicting the "
      "channel's mean and negative means worse than that; the lag-one autocorrelation assumes "
      "uniformly spaced samples, which the record's timebase must have provided";

  for (std::size_t o = 0; o < output_count; ++o) {
    ValidationOutput out;
    out.channel = request.outputs[o].channel;
    out.state_name = request.outputs[o].state_name;

    const std::vector<double>& observed = *measured[o];
    if (static_cast<int>(observed.size()) != samples) {
      throw std::invalid_argument("identify.validate: channel '" + out.channel
                                  + "' has a different length from the record's timebase");
    }
    double sum_squared = 0.0;
    double sum_error = 0.0;
    double observed_mean = 0.0;
    for (int k = 0; k < samples; ++k) {
      observed_mean += observed[static_cast<std::size_t>(k)];
    }
    observed_mean /= static_cast<double>(samples);

    std::vector<double> residual(static_cast<std::size_t>(samples));
    double observed_variation = 0.0;
    for (int k = 0; k < samples; ++k) {
      const double error =
          predicted[o][static_cast<std::size_t>(k)] - observed[static_cast<std::size_t>(k)];
      residual[static_cast<std::size_t>(k)] = error;
      sum_squared += error * error;
      sum_error += error;
      out.max_absolute_error = std::fmax(out.max_absolute_error, std::fabs(error));
      const double spread = observed[static_cast<std::size_t>(k)] - observed_mean;
      observed_variation += spread * spread;
    }
    out.rmse = std::sqrt(sum_squared / static_cast<double>(samples));
    out.mean_error = sum_error / static_cast<double>(samples);

    // A channel that never moves has no variation to be compared against. Every
    // prediction is equally good by this measure, so the measure does not
    // apply; reporting zero or one would be inventing a verdict.
    if (observed_variation > 0.0) {
      out.fit_fraction = 1.0 - std::sqrt(sum_squared) / std::sqrt(observed_variation);
      out.fit_fraction_is_defined = true;
    } else {
      out.undefined_reason =
          "the observed channel does not vary, so there is no variation for a prediction to "
          "explain and no fit fraction to report";
    }

    // Lag-one autocorrelation of the residual. Structure left behind means the
    // model missed something the record contains. Reported, never gated on.
    double residual_mean = 0.0;
    for (const double value : residual) {
      residual_mean += value;
    }
    residual_mean /= static_cast<double>(samples);
    double numerator = 0.0;
    double denominator = 0.0;
    for (int k = 0; k < samples; ++k) {
      const double centred = residual[static_cast<std::size_t>(k)] - residual_mean;
      denominator += centred * centred;
      if (k + 1 < samples) {
        numerator += centred * (residual[static_cast<std::size_t>(k) + 1] - residual_mean);
      }
    }
    if (denominator > 0.0) {
      out.residual_lag_one_autocorrelation = numerator / denominator;
      out.autocorrelation_is_defined = true;
    }
    result.outputs.push_back(std::move(out));
  }
  return result;
}

}  // namespace galata::identify
