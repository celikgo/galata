// SPDX-License-Identifier: Apache-2.0

#include "galata/analyze/singular_values.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace galata::analyze {

std::size_t SingularValueResponse::channel_count() const {
  return singular_values.empty() ? 0U : singular_values.front().size();
}

std::vector<double> SingularValueResponse::largest() const {
  std::vector<double> values;
  values.reserve(singular_values.size());
  for (const std::vector<double>& entry : singular_values) {
    values.push_back(entry.front());
  }
  return values;
}

std::vector<double> SingularValueResponse::smallest() const {
  std::vector<double> values;
  values.reserve(singular_values.size());
  for (const std::vector<double>& entry : singular_values) {
    values.push_back(entry.back());
  }
  return values;
}

std::vector<double> SingularValueResponse::condition_number() const {
  std::vector<double> values;
  values.reserve(singular_values.size());
  for (const std::vector<double>& entry : singular_values) {
    values.push_back(entry.back() > 0.0 ? entry.front() / entry.back()
                                        : std::numeric_limits<double>::infinity());
  }
  return values;
}

SingularValueResponse singular_values(const FrequencyResponse& response) {
  if (response.response.empty()) {
    throw std::invalid_argument("singular_values: the frequency response is empty");
  }

  SingularValueResponse result;
  result.frequencies_rad_s = response.frequencies_rad_s;
  result.input_names = response.input_names;
  result.output_names = response.output_names;
  result.singular_values.reserve(response.response.size());
  result.searched_from_rad_s = response.frequencies_rad_s.front();
  result.searched_to_rad_s = response.frequencies_rad_s.back();
  result.grid_points = static_cast<int>(response.frequencies_rad_s.size());

  result.peak_gain = -std::numeric_limits<double>::infinity();
  result.peak_frequency_rad_s = std::numeric_limits<double>::quiet_NaN();

  for (std::size_t index = 0; index < response.response.size(); ++index) {
    // Jacobi rather than the divide-and-conquer BDCSVD: the matrices here are
    // the size of a control loop, a handful of channels, where Jacobi is both
    // the more accurate and the faster of the two. Eigen returns the values in
    // descending order already.
    Eigen::JacobiSVD<Eigen::MatrixXcd> svd(response.response[index]);
    const Eigen::VectorXd& values = svd.singularValues();

    std::vector<double> row;
    row.reserve(static_cast<std::size_t>(values.size()));
    for (Eigen::Index k = 0; k < values.size(); ++k) {
      row.push_back(values(k));
    }
    if (row.empty()) {
      throw std::runtime_error("singular_values: the response has no channels");
    }
    if (row.front() > result.peak_gain) {
      result.peak_gain = row.front();
      result.peak_frequency_rad_s = response.frequencies_rad_s[index];
    }
    result.singular_values.push_back(std::move(row));
  }
  return result;
}

SingularValueResponse singular_values(const model::LinearSystem& system,
                                      const std::vector<double>& frequencies_rad_s) {
  return singular_values(frequency_response(system, frequencies_rad_s));
}

}  // namespace galata::analyze
