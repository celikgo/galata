// SPDX-License-Identifier: Apache-2.0
//
// Linear least squares over declared terms: the bench maps.
//
// WHAT THIS FITS. A response that is linear in its PARAMETERS, however
// nonlinear it is in its measured quantities. Thrust against speed squared,
// reaction torque against speed squared, terminal voltage against current — all
// one form, because each is `y = sum_j c_j f_j(x)` with the f_j declared by the
// study.
//
// WHAT IT WILL NOT DO. It will not choose the model. A term that is not in the
// study is not in the fit, and there is no stepwise selection, no automatic
// intercept and no silent centring: every one of those changes what the
// coefficients MEAN, and a coefficient whose meaning was decided by software is
// not a measurement of anything.
//
// UNCERTAINTY IS REPORTED ONLY WHEN IT EXISTS. The usual standard error
// estimates the noise from what the fit could not explain, sigma^2 = RSS /
// (n - p). With as many samples as parameters that denominator is zero, and
// with noiseless data the numerator is too: the fit passes exactly through the
// points and has told you nothing about how much they scatter. Both cases
// return coefficients and NO uncertainty, rather than a zero or an infinity
// that a reader would take for precision. `uncertainty_is_estimable` says
// which, and why.
#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::identify {

// One term of the model: a channel raised to a power, e.g. rotor speed squared.
// The power is declared rather than inferred, because `thrust ~ speed^2` and
// `thrust ~ speed^3` fit the same data to different physics.
struct Term {
  std::string channel;
  double power = 1.0;
  std::string name;  // what the coefficient is called
  std::string unit;  // the coefficient's own unit, e.g. "N.s^2"
};

struct StaticFitRequest {
  std::string response_channel;
  std::vector<Term> terms;
  // A constant term. Off by default: an intercept that nobody asked for turns
  // `T = k w^2` into `T = k w^2 + b`, and a nonzero b is thrust at zero speed.
  bool intercept = false;
  std::string intercept_name = "intercept";
  std::string intercept_unit;
  // Beyond this the design matrix is treated as rank deficient and the fit is
  // refused. Conservative and stated, not universal.
  double maximum_condition_number = 1e8;
};

struct Coefficient {
  std::string name;
  std::string unit;
  double value = 0.0;
  // Standard error, valid only when `uncertainty_is_estimable`.
  double standard_error = 0.0;
};

struct StaticFit {
  std::vector<Coefficient> coefficients;
  double residual_rms = 0.0;
  int sample_count = 0;
  int parameter_count = 0;
  // Condition number of the design matrix, reported whether or not it passed:
  // a fit near the limit is a fit whose coefficients trade against each other.
  double condition_number = 0.0;
  bool uncertainty_is_estimable = false;
  std::string uncertainty_note;
  // The span of each term's CHANNEL over the record, in that channel's own unit —
  // NOT the span of the design-matrix column the term becomes. For a term
  // `omega^2` over a record reaching 2000 rad/s these hold 0 and 2000, not 0 and
  // 4e6. The channel's span is the more useful of the two, because it is the
  // quantity a bench operator set and a reader recognises, and the fields were
  // called `regressor_*` until 2026-09-10 — which named the design column and so
  // invited a reader to be wrong by a square.
  //
  // A coefficient is evidence about the range it was measured over and about
  // nothing outside it, and a reader extrapolating from it should have to ignore
  // this to do so.
  std::vector<double> term_channel_minimum;
  std::vector<double> term_channel_maximum;
  double response_minimum = 0.0;
  double response_maximum = 0.0;
};

}  // namespace galata::identify

namespace galata::data {
struct Record;
}  // namespace galata::data

namespace galata::identify {

// Refuses: a channel the record does not carry; fewer samples than parameters;
// a regressor that does not vary, which is a bench run that never excited the
// thing it was meant to measure; a design matrix past
// `maximum_condition_number`; and a non-finite value anywhere.
[[nodiscard]] StaticFit fit_static(const data::Record& record, const StaticFitRequest& request);

}  // namespace galata::identify
