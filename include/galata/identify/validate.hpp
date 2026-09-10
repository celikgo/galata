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
// SO THE RECORDS' IDENTITIES TRAVEL WITH THE ANSWER, AND SO DOES WHAT
// ESTABLISHED THEM. A result computed on the estimation record is still
// returned — re-running a fit on its own data is a legitimate diagnostic — but
// it is labelled, and the label is the difference between a diagnostic and a
// claim. A good estimation fit presented as validation is the specific
// dishonesty this design exists to make awkward.
//
// AND UNEQUAL DIGESTS ARE NOT THE LABEL. An earlier version of this file said
// `is_held_out` was false when the two digests matched, and true otherwise.
// The first half is sound — the same bytes are the same observations — and the
// second half is not, because a digest is an identity of BYTES and held-out is
// a claim about DATA. A file reformatted, exported twice, or written with a
// different number of decimal places has a different digest and the same
// observations in it. A segment copied from one file into another has a
// different digest and shares every sample it copied. Two exports of one flight
// at two sample rates have different digests and describe the same seconds of
// the same aircraft. Every one of those would have passed as validation on an
// inequality of hashes, and each is a model being scored on its own training
// data with a label saying otherwise.
//
// So independence is now classified, and the classification says on what
// grounds. `Independence::VerifiedDisjoint` is reserved for the one case this
// code can actually prove: two windows of ONE imported file whose intervals do
// not overlap, cut by `data.window`, which keeps the file's identity and adds
// an interval to it. `CallerDeclared` is what an honest study gets when it
// knows two files are different flights and galata cannot check that —
// recorded as the caller's claim, in the caller's name, and downgraded to
// `NotHeldOut` the moment a check contradicts it. `Unknown` is what silence
// gets. The one thing that no longer happens is a claim of independence
// manufactured out of an inequality.
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

// On what grounds, if any, the validation record is independent of the record
// the model was fitted to. Ordered from the strongest negative to the weakest
// positive, and never inferred from a digest inequality alone.
enum class Independence {
  // The two records demonstrably share observations: the same record, the same
  // file over overlapping intervals, or samples found in both. Not a refusal —
  // the numbers are still computed and returned — but not validation either.
  NotHeldOut,
  // Proven. Both records are windows of ONE imported file and their intervals
  // do not overlap, so no observation can be in both, and a scan of the shared
  // channels confirms none is.
  VerifiedDisjoint,
  // The study asserted independence, and every check that could be made was
  // made and did not contradict it. This is a claim in the caller's name, not
  // a finding in galata's.
  CallerDeclared,
  // Nothing establishes it. No claim was made, or the records share no channel
  // by which a claim could be checked.
  Unknown,
};

[[nodiscard]] std::string to_string(Independence independence);

// Where a record came from, kept so a reader can see why the classification
// above came out as it did rather than taking it on trust.
struct RecordLineage {
  std::string source_path;
  std::string source_sha256;
  bool is_window = false;
  double window_start_s = 0.0;
  double window_end_s = 0.0;
  // The stretch the samples actually cover, which is what overlap is decided
  // on. A declared window may be wider than the samples inside it.
  double first_sample_s = 0.0;
  double last_sample_s = 0.0;
  int sample_count = 0;
};

struct ValidationResult {
  std::vector<ValidationOutput> outputs;
  std::string estimation_record_sha256;
  std::string validation_record_sha256;

  Independence independence = Independence::Unknown;
  // One sentence naming what established the classification, including what it
  // could NOT check. Written for a reader who will quote it, so it must be
  // true standing alone.
  std::string independence_basis;
  // Preserved where available, and empty where it is not: the estimation
  // record's lineage is only known when the record itself was supplied.
  RecordLineage validation_lineage;
  RecordLineage estimation_lineage;
  bool estimation_lineage_is_known = false;
  // What the comparison found, reported whether or not it changed the verdict.
  int shared_channel_count = 0;
  int shared_sample_count = 0;
  bool intervals_overlap = false;
  // True when the study declared independence and a check contradicted it. A
  // caller who wants to know that its own claim was overruled should not have
  // to parse a sentence to find out.
  bool caller_declaration_was_contradicted = false;

  int sample_count = 0;
  std::string assumptions;
};

struct ValidationRequest {
  // The digest of the record the model was FITTED to. Required unless
  // `estimation_record` is supplied, in which case it is read from that record
  // and this, if also given, must agree with it — a disagreement is refused
  // rather than resolved, because the two cannot both be the training data.
  //
  // A caller who cannot say which record trained the model cannot claim
  // anything was held out from it, and this will not accept the claim on trust.
  std::string estimation_record_sha256;

  // THE ESTIMATION RECORD ITSELF, when the study can supply it. This is what
  // turns the label from a claim into a check: with it, the two records'
  // lineage and their observations are compared, and `VerifiedDisjoint` becomes
  // reachable. Without it, the best an honest study can reach is
  // `CallerDeclared`. Not owned; must outlive the call.
  const data::Record* estimation_record = nullptr;

  // What the study asserts when it cannot supply the record — that these are
  // different data. Recorded as the caller's claim and checked as far as it can
  // be; a check that contradicts it wins.
  bool caller_declares_independent = false;

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
