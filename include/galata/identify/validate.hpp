// SPDX-License-Identifier: Apache-2.0
//
// Held-out validation: running an identified model on a record it was not
// fitted to.
//
// WHY IT IS A SEPARATE CAPABILITY FROM THE FIT. A fit's own residual is not
// evidence that a model predicts anything. It is evidence that an optimiser
// found the best it could on the data it was given, and enough free parameters
// will drive it to zero on any record at all. The question that matters — does
// this model predict data it has never seen — can only be asked with a
// DIFFERENT record, and nothing about the fitting code can answer it.
//
// SO THE RECORDS' IDENTITIES TRAVEL WITH THE ANSWER. The estimation record's
// digest is declared by the caller, the validation record's is read from the
// record, and `is_held_out` is false when they match. A result computed on the
// estimation record is still returned — re-running a fit on its own data is a
// legitimate diagnostic — but it is labelled, and the label is the difference
// between a diagnostic and a claim. A good estimation fit presented as
// validation is the specific dishonesty this design exists to make awkward.
#pragma once

#include "galata/model/quadrotor.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::data {
struct Record;
}  // namespace galata::data

namespace galata::identify {

struct ValidationOutput {
  std::string channel;
  std::string state_name;
  double rmse = 0.0;
  double max_absolute_error = 0.0;
  double mean_error = 0.0;  // a nonzero mean is a bias the model did not capture

  // NRMSE as a FIT FRACTION, defined here rather than assumed:
  //
  //     fit = 1 - ||y - yhat|| / ||y - mean(y)||
  //
  // One is a perfect prediction; zero is no better than predicting the
  // channel's own mean; NEGATIVE is worse than that, and is reported as such
  // rather than clamped, because a model that loses to a constant is a finding.
  double fit_fraction = 0.0;
  bool fit_fraction_is_defined = false;
  // The denominator above vanishes for a channel that never moves. Such a
  // channel supports no fit fraction at all — every prediction is equally good
  // by that measure — and reporting zero or one would be inventing a verdict.
  std::string undefined_reason;

  // Lag-one autocorrelation of the residual. Structure left in a residual means
  // the model missed something the record contains; white residuals are the
  // shape of "what is left is noise". Reported, never gated on: this code does
  // not decide what is acceptable.
  double residual_lag_one_autocorrelation = 0.0;
  bool autocorrelation_is_defined = false;
};

struct ValidationResult {
  std::vector<ValidationOutput> outputs;
  std::string estimation_record_sha256;
  std::string validation_record_sha256;
  // False when the two digests match. See the header: the label is the
  // difference between a diagnostic and a claim.
  bool is_held_out = false;
  int sample_count = 0;
  std::string assumptions;
};

struct ValidationRequest {
  // The digest of the record the model was FITTED to. Required, and compared
  // rather than trusted: a caller who cannot say which record trained the model
  // cannot claim anything was held out from it.
  std::string estimation_record_sha256;
  std::vector<std::string> command_channels;

  struct Match {
    std::string channel;
    std::string state_name;
  };

  std::vector<Match> outputs;
  Eigen::VectorXd initial_extended_state;
  double step_s = 0.0;
};

[[nodiscard]] ValidationResult validate_model(const model::Quadrotor& model,
                                              const data::Record& record,
                                              const ValidationRequest& request);

}  // namespace galata::identify
