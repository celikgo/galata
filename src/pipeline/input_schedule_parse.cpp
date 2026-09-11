// SPDX-License-Identifier: Apache-2.0
//
// Moved here from quadrotor_capabilities.cpp when `sim.linear` began to read the
// same schema, so the two capabilities cannot drift apart. The one change on
// moving: keys the schema does not name are now refused at both levels, where
// they were silently ignored — a `interpolation: linear` beside `hold` would
// otherwise read as a request that was honoured.

#include "input_schedule_parse.hpp"

#include <algorithm>
#include <array>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace galata::pipeline {
namespace {

template <std::size_t N>
void require_known_keys(const ValuePtr& value,
                        const std::array<const char*, N>& allowed,
                        const std::string& where) {
  for (const auto& [name, entry] : value->as_map()) {
    (void)entry;
    if (std::none_of(
            allowed.begin(), allowed.end(), [&](const char* key) { return name == key; })) {
      std::string message = "'" + where + "' has an unknown key '" + name + "'; it takes ";
      for (std::size_t i = 0; i < N; ++i) {
        message += (i == 0 ? "`" : ", `") + std::string(allowed[i]) + "`";
      }
      throw std::invalid_argument(message);
    }
  }
}

}  // namespace

// A schedule is a hold policy, an extrapolation policy and a list of samples.
// Both policies are REQUIRED. There is no default that is right for every
// channel — a rotor command that steps is held, a wind that builds is
// interpolated — and inferring one from the data picks a trajectory the study
// does not state.
sim::InputSchedule schedule_at(const StageContext& context,
                               const std::string& key,
                               int expected_width) {
  const ValuePtr value = context.input->get(key);
  if (!value || value->kind() == Value::Kind::Null) {
    return {};
  }
  require_known_keys(value, std::array{"hold", "extrapolation", "samples"}, key);
  const std::string hold_name = value->string_at("hold");
  sim::HoldPolicy hold{};
  if (hold_name == "zero_order") {
    hold = sim::HoldPolicy::ZeroOrder;
  } else if (hold_name == "linear") {
    hold = sim::HoldPolicy::Linear;
  } else {
    throw std::invalid_argument("'" + key + ".hold' must be `zero_order` or `linear`; '" + hold_name
                                + "' is neither, and there is no default");
  }
  const std::string outside_name = value->string_at("extrapolation");
  sim::Extrapolation outside{};
  if (outside_name == "hold") {
    outside = sim::Extrapolation::Hold;
  } else if (outside_name == "refuse") {
    outside = sim::Extrapolation::Refuse;
  } else {
    throw std::invalid_argument("'" + key + ".extrapolation' must be `hold` or `refuse`; '"
                                + outside_name + "' is neither, and there is no default");
  }

  const ValuePtr samples = value->get("samples");
  if (!samples) {
    throw std::invalid_argument("'" + key + "' needs a `samples` list");
  }
  std::vector<double> times;
  std::vector<Eigen::VectorXd> rows;
  for (const ValuePtr& sample : samples->as_list()) {
    require_known_keys(sample, std::array{"time_s", "values"}, key + ".samples[]");
    times.push_back(sample->number_at("time_s"));
    const ValuePtr values_node = sample->get("values");
    if (!values_node) {
      throw std::invalid_argument("'" + key + "' sample needs a `values` list");
    }
    const std::vector<ValuePtr>& items = values_node->as_list();
    if (static_cast<int>(items.size()) != expected_width) {
      std::ostringstream message;
      message << "'" << key << "' sample at t = " << times.back() << " s carries " << items.size()
              << " channel(s); this model needs " << expected_width;
      throw std::invalid_argument(message.str());
    }
    Eigen::VectorXd row(expected_width);
    for (std::size_t i = 0; i < items.size(); ++i) {
      row(static_cast<Eigen::Index>(i)) = items[i]->as_number();
    }
    rows.push_back(std::move(row));
  }
  return sim::InputSchedule(std::move(times), std::move(rows), hold, outside);
}

}  // namespace galata::pipeline
