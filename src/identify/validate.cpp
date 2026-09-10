// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the held-out validation declared in
// include/galata/identify/validate.hpp.

#include "galata/identify/validate.hpp"

#include "galata/data/record.hpp"
#include "galata/numerics/integrator.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace galata::identify {
namespace {

const std::vector<double>& channel_of(const data::Record& record, const std::string& name) {
  const data::Channel* channel = record.find(name);
  if (channel == nullptr) {
    throw std::invalid_argument("identify.validate: the record has no channel '" + name + "'");
  }
  return channel->samples;
}

RecordLineage lineage_of(const data::Record& record) {
  RecordLineage lineage;
  lineage.source_path = record.source_path;
  lineage.source_sha256 = record.source_sha256;
  lineage.is_window = record.is_window;
  lineage.window_start_s = record.window_start_s;
  lineage.window_end_s = record.window_end_s;
  lineage.first_sample_s = record.first_time_s();
  lineage.last_sample_s = record.last_time_s();
  lineage.sample_count = static_cast<int>(record.sample_count());
  return lineage;
}

// The channel names both records carry, in the validation record's order so the
// result does not depend on which record was passed where. `std::set` rather
// than a hash: ADR-0004, ordered iteration reaching output.
std::vector<std::string> shared_channel_names(const data::Record& left, const data::Record& right) {
  std::set<std::string> in_right;
  for (const data::Channel& channel : right.channels) {
    in_right.insert(channel.name);
  }
  std::vector<std::string> shared;
  for (const data::Channel& channel : left.channels) {
    if (in_right.count(channel.name) != 0) {
      shared.push_back(channel.name);
    }
  }
  return shared;
}

// Observations found in BOTH records: a sample instant whose values agree
// exactly on every shared channel. Exact equality is the right test and its own
// limit — it finds a segment copied verbatim between files, however they were
// formatted, and it cannot see a copy that was rescaled or resampled. The basis
// sentence says so rather than leaving a reader to assume otherwise.
int shared_sample_count(const data::Record& left,
                        const data::Record& right,
                        const std::vector<std::string>& shared) {
  if (shared.empty()) {
    return 0;
  }
  std::vector<const std::vector<double>*> left_channels;
  std::vector<const std::vector<double>*> right_channels;
  for (const std::string& name : shared) {
    left_channels.push_back(&left.find(name)->samples);
    right_channels.push_back(&right.find(name)->samples);
  }

  std::vector<std::vector<double>> rows;
  rows.reserve(right.times_s.size());
  for (std::size_t k = 0; k < right.times_s.size(); ++k) {
    std::vector<double> row;
    row.reserve(shared.size() + 1);
    row.push_back(right.times_s[k]);
    for (const std::vector<double>* channel : right_channels) {
      row.push_back((*channel)[k]);
    }
    rows.push_back(std::move(row));
  }
  std::sort(rows.begin(), rows.end());

  int found = 0;
  for (std::size_t k = 0; k < left.times_s.size(); ++k) {
    std::vector<double> row;
    row.reserve(shared.size() + 1);
    row.push_back(left.times_s[k]);
    for (const std::vector<double>* channel : left_channels) {
      row.push_back((*channel)[k]);
    }
    if (std::binary_search(rows.begin(), rows.end(), row)) {
      ++found;
    }
  }
  return found;
}

std::string interval_text(const data::Record& record) {
  std::ostringstream out;
  out << "[" << record.first_time_s() << ", " << record.last_time_s() << "] s";
  return out.str();
}

}  // namespace

std::string to_string(RecordSeparation separation) {
  switch (separation) {
    case RecordSeparation::NotHeldOut:
      return "not held out";
    case RecordSeparation::VerifiedDisjoint:
      return "verified disjoint";
    case RecordSeparation::CallerDeclared:
      return "caller-declared";
    case RecordSeparation::Unknown:
      return "unknown";
  }
  return "unknown";
}

namespace {

// THE CLASSIFICATION. Ordered so that a proof of overlap always beats a claim
// of independence: every branch that can establish NotHeldOut is taken before
// any branch that can grant it.
void classify_separation(const data::Record& record,
                         const ValidationRequest& request,
                         ValidationResult& result) {
  const std::string& estimation_digest = result.estimation_record_sha256;

  // 1. The same bytes. Nothing else needs checking, and nothing else could
  //    overturn it.
  if (!estimation_digest.empty() && record.source_sha256 == estimation_digest
      && request.estimation_record == nullptr) {
    result.separation = RecordSeparation::NotHeldOut;
    result.separation_basis =
        "the validation record and the estimation record are the same bytes (sha256 "
        + estimation_digest.substr(0, 16)
        + "...), so this is the model scored on its own training data — a legitimate "
          "diagnostic, and not validation";
    return;
  }

  if (request.estimation_record == nullptr) {
    // Nothing to compare against. Whatever the digests say, an inequality of
    // hashes is an inequality of BYTES: the same observations reformatted, or a
    // segment copied between files, would pass it. So the verdict is the
    // caller's claim or silence, and it says which.
    if (request.caller_declares_different_data) {
      result.separation = RecordSeparation::CallerDeclared;
      result.separation_basis =
          "the study declared these to be different data and the estimation record was not "
          "supplied, so nothing here checked the claim. Digest inequality alone was NOT "
          "treated as evidence: the same observations reformatted, exported twice, or copied "
          "between files carry different digests. Supply the estimation record to have the "
          "claim verified";
    } else {
      result.separation = RecordSeparation::Unknown;
      result.separation_basis =
          "only the estimation record's digest was given, and a digest identifies bytes rather "
          "than observations. Nothing here establishes that these are different data, and "
          "nothing here establishes that they are not";
    }
    return;
  }

  const data::Record& estimation = *request.estimation_record;
  result.estimation_lineage = lineage_of(estimation);
  result.estimation_lineage_is_known = true;

  const std::vector<std::string> shared = shared_channel_names(record, estimation);
  result.shared_channel_count = static_cast<int>(shared.size());
  result.shared_sample_count = shared_sample_count(record, estimation, shared);
  result.intervals_overlap = record.first_time_s() <= estimation.last_time_s()
                             && estimation.first_time_s() <= record.last_time_s();

  const bool same_source =
      !record.source_sha256.empty() && record.source_sha256 == estimation.source_sha256;

  // 2. Same file, overlapping stretch of it. The observations are the same
  //    seconds of the same run whether or not any sample is bit-identical:
  //    two exports of one segment at two rates share no tuple and every
  //    measurement.
  if (same_source && result.intervals_overlap) {
    result.separation = RecordSeparation::NotHeldOut;
    std::ostringstream basis;
    basis << "both records are cut from one imported file (sha256 "
          << record.source_sha256.substr(0, 16) << "...) and their sample intervals overlap — "
          << "validation " << interval_text(record) << " against estimation "
          << interval_text(estimation)
          << ". The same stretch of one run is the same observations however it was resampled";
    result.separation_basis = basis.str();
    result.caller_declaration_was_contradicted = request.caller_declares_different_data;
    return;
  }

  // 3. Observations found in both, whatever the files were called.
  if (result.shared_sample_count > 0) {
    result.separation = RecordSeparation::NotHeldOut;
    std::ostringstream basis;
    basis << result.shared_sample_count << " of the validation record's " << record.sample_count()
          << " sample(s) occur in the estimation record too, matching exactly on all "
          << shared.size() << " shared channel(s)"
          << (same_source ? "" : " despite the two files having different digests")
          << ". A model scored on samples it was fitted to is not being validated on them";
    result.separation_basis = basis.str();
    result.caller_declaration_was_contradicted = request.caller_declares_different_data;
    return;
  }

  // 4. The one case that can be PROVEN. One file, two stretches of it that do
  //    not meet. Nothing outside those stretches is in either record, so no
  //    observation can be in both — and the scan above confirms none is.
  if (same_source && !result.intervals_overlap && record.is_window && estimation.is_window) {
    result.separation = RecordSeparation::VerifiedDisjoint;
    std::ostringstream basis;
    basis << "both records are windows of one imported file (sha256 "
          << record.source_sha256.substr(0, 16) << "...) over intervals that do not meet — "
          << "validation " << interval_text(record) << " against estimation "
          << interval_text(estimation) << " — and no sample instant occurs in both across the "
          << shared.size()
          << " shared channel(s). Disjoint by construction, not by an inequality of hashes. "
             "This is SAMPLE separation and not statistical independence: the two stretches "
             "share the aircraft, its trim, the air mass and every unmodelled effect that "
             "persists across the cut";
    result.separation_basis = basis.str();
    return;
  }

  // 5. Two different files, no shared sample found. That is the absence of a
  //    contradiction and not a proof: exact equality cannot see the same flight
  //    rescaled or resampled into a second file.
  if (request.caller_declares_different_data) {
    result.separation = RecordSeparation::CallerDeclared;
    std::ostringstream basis;
    basis << "the study declared these to be different data, and the checks that were possible "
             "did not contradict it: no sample instant occurs in both across the "
          << shared.size() << " shared channel(s)"
          << (result.intervals_overlap ? ", though their intervals do overlap in time"
                                       : ", and their intervals do not overlap")
          << ". This is not a proof — exact comparison cannot see the same run rescaled or "
             "resampled into a second file — so it stands as the caller's claim, unrefuted";
    result.separation_basis = basis.str();
    return;
  }

  result.separation = RecordSeparation::Unknown;
  std::ostringstream basis;
  basis << "the two records come from different files and share no sample instant across the "
        << shared.size()
        << " shared channel(s), but nothing here proves they hold different observations and "
           "the study claimed nothing. Cut both from one import with `data.window` to have the "
           "split verified, or declare the claim to have it recorded as yours";
  result.separation_basis = basis.str();
}

}  // namespace

ValidationResult validate_model(const model::Quadrotor& model,
                                const data::Record& record,
                                const ValidationRequest& request) {
  if (request.estimation_record_sha256.empty() && request.estimation_record == nullptr) {
    throw std::invalid_argument(
        "identify.validate: the estimation record's identity is required — either the record "
        "itself, or its digest. A caller who cannot say which record trained the model cannot "
        "claim anything was held out from it, and this will not accept the claim on trust");
  }
  // Both may be given, and then they must agree. Resolving a disagreement by
  // preferring one would be choosing which record trained the model on the
  // caller's behalf, and only one of them did.
  if (request.estimation_record != nullptr && !request.estimation_record_sha256.empty()
      && request.estimation_record->source_sha256 != request.estimation_record_sha256) {
    throw std::invalid_argument(
        "identify.validate: the declared estimation digest '" + request.estimation_record_sha256
        + "' is not the digest of the estimation record supplied, '"
        + request.estimation_record->source_sha256
        + "'. They cannot both be the data the model was fitted to, and which one is is not "
          "this routine's to decide");
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
  result.estimation_record_sha256 = request.estimation_record != nullptr
                                        ? request.estimation_record->source_sha256
                                        : request.estimation_record_sha256;
  result.validation_record_sha256 = record.source_sha256;
  result.validation_lineage = lineage_of(record);
  classify_separation(record, request, result);
  result.assumptions =
      "the separation label bounds what the fit could have seen and is not a claim of "
      "statistical independence; errors are compared sample by sample against the record's "
      "own values; the fit fraction "
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
