// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the linear least squares declared in
// include/galata/identify/static_fit.hpp.
//
// The solve is a column-pivoting householder QR of the design matrix rather
// than the normal equations. Forming A^T A squares the condition number, which
// on a bench map spanning two decades of speed is the difference between a
// coefficient and a coefficient's rounding error.

#include "galata/identify/static_fit.hpp"

#include "galata/data/record.hpp"

#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace galata::identify {
namespace {

const std::vector<double>& samples_of(const data::Record& record, const std::string& name) {
  const data::Channel* channel = record.find(name);
  if (channel == nullptr) {
    throw std::invalid_argument("identify.static_fit: the record has no channel '" + name
                                + "'. It is refused rather than skipped: a fit missing a term "
                                  "the study asked for is a different model");
  }
  return channel->samples;
}

}  // namespace

StaticFit fit_static(const data::Record& record, const StaticFitRequest& request) {
  if (request.terms.empty() && !request.intercept) {
    throw std::invalid_argument("identify.static_fit: a fit needs at least one term");
  }
  const std::vector<double>& response = samples_of(record, request.response_channel);
  const auto samples = static_cast<int>(response.size());
  const auto parameters = static_cast<int>(request.terms.size()) + (request.intercept ? 1 : 0);
  if (samples < parameters) {
    std::ostringstream message;
    message << "identify.static_fit: " << samples << " sample(s) cannot determine " << parameters
            << " parameter(s). This is not a fit that came out badly; it is a system with more "
               "unknowns than equations";
    throw std::invalid_argument(message.str());
  }

  Eigen::MatrixXd design(samples, parameters);
  Eigen::VectorXd target(samples);
  for (int i = 0; i < samples; ++i) {
    if (!std::isfinite(response[static_cast<std::size_t>(i)])) {
      throw std::invalid_argument(
          "identify.static_fit: the response channel holds a non-finite value; a record with "
          "gaps must declare how they were handled at import, not have them absorbed here");
    }
    target(i) = response[static_cast<std::size_t>(i)];
  }

  StaticFit fit;
  fit.term_channel_minimum.assign(request.terms.size(), 0.0);
  fit.term_channel_maximum.assign(request.terms.size(), 0.0);
  for (std::size_t t = 0; t < request.terms.size(); ++t) {
    const Term& term = request.terms[t];
    const std::vector<double>& column = samples_of(record, term.channel);
    if (static_cast<int>(column.size()) != samples) {
      throw std::invalid_argument("identify.static_fit: channel '" + term.channel + "' has "
                                  + std::to_string(column.size()) + " sample(s) against the "
                                    "response's " + std::to_string(samples));
    }
    double lowest = column.front();
    double highest = column.front();
    for (int i = 0; i < samples; ++i) {
      const double raw = column[static_cast<std::size_t>(i)];
      if (!std::isfinite(raw)) {
        throw std::invalid_argument("identify.static_fit: channel '" + term.channel
                                    + "' holds a non-finite value");
      }
      lowest = std::fmin(lowest, raw);
      highest = std::fmax(highest, raw);
      design(i, static_cast<int>(t)) = std::pow(raw, term.power);
    }
    // A regressor that does not move carries no information about its own
    // coefficient. That is a bench run which never excited the thing it was
    // meant to measure, and it is a refusal rather than a large standard error,
    // because the number that comes out is not an estimate of anything.
    if (!(highest > lowest)) {
      throw std::invalid_argument(
          "identify.static_fit: channel '" + term.channel
          + "' does not vary across the record, so it carries no information about its own "
            "coefficient. The run never excited what it was meant to measure");
    }
    fit.term_channel_minimum[t] = lowest;
    fit.term_channel_maximum[t] = highest;
  }
  if (request.intercept) {
    design.col(parameters - 1).setOnes();
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(design);
  const double smallest = svd.singularValues()(svd.singularValues().size() - 1);
  fit.condition_number =
      smallest > 0.0 ? svd.singularValues()(0) / smallest : std::numeric_limits<double>::infinity();
  if (!(fit.condition_number <= request.maximum_condition_number)) {
    std::ostringstream message;
    message << "identify.static_fit: the design matrix has condition number "
            << fit.condition_number << ", past the declared limit "
            << request.maximum_condition_number
            << ". The terms cannot be told apart by this data: a fit would return coefficients "
               "that trade against each other, and their individual values would mean nothing";
    throw std::invalid_argument(message.str());
  }

  const Eigen::VectorXd solution = design.colPivHouseholderQr().solve(target);
  const Eigen::VectorXd residual = target - design * solution;
  const double residual_sum_of_squares = residual.squaredNorm();
  fit.sample_count = samples;
  fit.parameter_count = parameters;
  fit.residual_rms = std::sqrt(residual_sum_of_squares / static_cast<double>(samples));
  fit.response_minimum = target.minCoeff();
  fit.response_maximum = target.maxCoeff();

  // WHEN AN UNCERTAINTY EXISTS AT ALL. sigma^2 = RSS / (n - p) estimates the
  // noise from what the fit could not explain. With n == p the denominator
  // vanishes; with a noiseless record the numerator does. Neither is a small
  // uncertainty — both are an absence of evidence about scatter, and returning
  // zero would be reporting infinite precision from data that demonstrated
  // none.
  const int degrees_of_freedom = samples - parameters;
  const double response_scale =
      std::fmax(std::fabs(fit.response_maximum), std::fabs(fit.response_minimum));
  const double noise_floor = 1e-12 * std::fmax(1.0, response_scale);
  if (degrees_of_freedom <= 0) {
    fit.uncertainty_is_estimable = false;
    fit.uncertainty_note =
        "exactly determined: as many parameters as samples, so the fit passes through every "
        "point and nothing is left over to estimate the scatter from";
  } else if (fit.residual_rms <= noise_floor) {
    fit.uncertainty_is_estimable = false;
    fit.uncertainty_note =
        "residual at the arithmetic floor: the data is noiseless as far as this fit can tell, "
        "so it demonstrates no scatter and supports no uncertainty";
  } else {
    fit.uncertainty_is_estimable = true;
    fit.uncertainty_note =
        "standard errors assume independent, identically distributed, zero-mean errors on the "
        "response and none on the regressors; a bench whose speed measurement is noisy breaks "
        "the last of those and this does not detect it";
  }

  const double variance = degrees_of_freedom > 0
                              ? residual_sum_of_squares / static_cast<double>(degrees_of_freedom)
                              : 0.0;
  const Eigen::MatrixXd covariance =
      (design.transpose() * design).ldlt().solve(Eigen::MatrixXd::Identity(parameters, parameters))
      * variance;

  for (std::size_t t = 0; t < request.terms.size(); ++t) {
    Coefficient coefficient;
    coefficient.name = request.terms[t].name;
    coefficient.unit = request.terms[t].unit;
    coefficient.value = solution(static_cast<int>(t));
    if (fit.uncertainty_is_estimable) {
      coefficient.standard_error =
          std::sqrt(std::fmax(0.0, covariance(static_cast<int>(t), static_cast<int>(t))));
    }
    fit.coefficients.push_back(std::move(coefficient));
  }
  if (request.intercept) {
    Coefficient coefficient;
    coefficient.name = request.intercept_name;
    coefficient.unit = request.intercept_unit;
    coefficient.value = solution(parameters - 1);
    if (fit.uncertainty_is_estimable) {
      coefficient.standard_error =
          std::sqrt(std::fmax(0.0, covariance(parameters - 1, parameters - 1)));
    }
    fit.coefficients.push_back(std::move(coefficient));
  }
  return fit;
}

}  // namespace galata::identify
