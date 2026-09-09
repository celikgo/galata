// SPDX-License-Identifier: Apache-2.0
// Typed lowering of x_dot = A x + B u, y = C x + D u, u = command - K x.
// Source: Astrom & Murray, Feedback Systems, 2nd ed., state-space models and
// state feedback; see ADR-0013 for the declared-coordinate and ordering policy.
// This is graph construction, not a second integration or aircraft solver.
#include "galata/modeling/linear_adapter.hpp"

#include <cmath>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace galata::modeling {
namespace {

void check_type(const SignalType& type) {
  for (const int exponent : type.dimension) {
    if (exponent < -16 || exponent > 16) {
      throw Error(ErrorCode::InvalidModel,
                  "linear channel dimension exponents must be in [-16, 16]");
    }
  }
  if (type.frame != Frame::None && type.frame != Frame::Body && type.frame != Frame::Ned) {
    throw Error(ErrorCode::InvalidModel, "unsupported linear channel frame");
  }
}

SignalType derivative_type(const SignalType& state) {
  auto derivative = state;
  --derivative.dimension[2];
  check_type(derivative);
  return derivative;
}

Dimension coefficient_dimension(const SignalType& input, const SignalType& output) {
  Dimension dimension{};
  for (std::size_t i = 0; i < dimension.size(); ++i) {
    dimension[i] = output.dimension[i] - input.dimension[i];
    if (dimension[i] < -16 || dimension[i] > 16) {
      throw Error(ErrorCode::InvalidModel,
                  "linear coefficient dimension exponents must be in [-16, 16]");
    }
  }
  return dimension;
}

void check_matrix_bounds(const Eigen::MatrixXd& matrix) {
  if (matrix.rows() > static_cast<Eigen::Index>(kMaxLinearChannels)
      || matrix.cols() > static_cast<Eigen::Index>(kMaxLinearChannels)) {
    throw Error(ErrorCode::ResourceLimit,
                "linear graph supports at most 32 channels per matrix axis");
  }
}

void check_names(const std::vector<std::string>& names, std::size_t& remaining_bytes) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].size() > remaining_bytes) {
      throw Error(ErrorCode::ResourceLimit, "linear channel names exceed the one-MiB byte limit");
    }
    remaining_bytes -= names[i].size();
    if (names[i].empty()) {
      throw Error(ErrorCode::InvalidModel, "linear channel names must be nonempty");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (names[j] == names[i]) {
        throw Error(ErrorCode::InvalidModel,
                    "linear channel names must be unique within each channel list");
      }
    }
  }
}

std::string channel_id(std::string_view prefix, Eigen::Index index) {
  // Validated channels are within kMaxLinearChannels, so at most two digits;
  // fixed width also preserves source order under the compiler's lexical ID
  // sorting. The pad is computed with max() rather than by subtraction because
  // the subtraction is on std::size_t: a wider index than the field would wrap
  // it and ask for a string of about eighteen quintillion zeroes.
  const auto digits = std::to_string(index);
  const std::size_t width = 3;
  const std::size_t pad = width > digits.size() ? width - digits.size() : 0;
  return std::string(prefix) + std::string(pad, '0') + digits;
}

}  // namespace

LinearGraph lower_linear_system(const model::LinearSystem& system,
                                const LinearChannels& channels,
                                const LinearGraphOptions& options) {
  // Reject oversized shapes/collections before validation can copy names or
  // graph construction can allocate. Even empty matrices retain bounded axes.
  check_matrix_bounds(system.a);
  check_matrix_bounds(system.b);
  check_matrix_bounds(system.c);
  check_matrix_bounds(system.d);
  check_matrix_bounds(options.feedback_gain);
  if (channels.states.size() > kMaxLinearChannels || channels.inputs.size() > kMaxLinearChannels
      || channels.outputs.size() > kMaxLinearChannels
      || system.state_names.size() > kMaxLinearChannels
      || system.input_names.size() > kMaxLinearChannels
      || system.output_names.size() > kMaxLinearChannels
      || options.initial_state.size() > static_cast<Eigen::Index>(kMaxLinearChannels)
      || options.command.size() > static_cast<Eigen::Index>(kMaxLinearChannels)) {
    throw Error(ErrorCode::ResourceLimit,
                "linear graph supports at most 32 states, inputs and outputs");
  }
  const auto n = system.state_count();
  const auto m = system.input_count();
  const auto p = system.output_count();
  if (n < 1 || m < 1 || p < 1) {
    throw Error(ErrorCode::InvalidModel,
                "linear graph requires at least one state, input and output");
  }
  if (system.b.rows() != n || static_cast<Eigen::Index>(system.input_names.size()) != m
      || static_cast<Eigen::Index>(channels.states.size()) != n
      || static_cast<Eigen::Index>(channels.inputs.size()) != m
      || static_cast<Eigen::Index>(channels.outputs.size()) != p
      || options.initial_state.size() != n || options.command.size() != m) {
    throw Error(ErrorCode::InvalidModel,
                "linear graph channel metadata, initial state and command require exact sizes");
  }
  std::size_t remaining_name_bytes = kMaxSourceBytes;
  check_names(system.state_names, remaining_name_bytes);
  check_names(system.input_names, remaining_name_bytes);
  check_names(system.output_names, remaining_name_bytes);
  try {
    system.validate();
  } catch (const std::invalid_argument& error) {
    throw Error(ErrorCode::InvalidModel, error.what());
  }
  const bool feedback = options.feedback_gain.size() != 0;
  if (feedback && (options.feedback_gain.rows() != m || options.feedback_gain.cols() != n)) {
    throw Error(ErrorCode::InvalidModel, "linear feedback gain requires m rows and n columns");
  }
  if (!options.initial_state.allFinite() || !options.command.allFinite()
      || !options.feedback_gain.allFinite()) {
    throw Error(ErrorCode::InvalidModel,
                "linear initial state, command and feedback gain must be finite");
  }
  for (const auto& type : channels.states)
    check_type(type);
  for (const auto& type : channels.inputs)
    check_type(type);
  for (const auto& type : channels.outputs)
    check_type(type);
  const auto c = [&](Eigen::Index row, Eigen::Index column) {
    return system.c.size() == 0 ? (row == column ? 1.0 : 0.0) : system.c(row, column);
  };
  const auto d = [&](Eigen::Index row, Eigen::Index column) {
    return system.d.size() == 0 ? 0.0 : system.d(row, column);
  };
  const auto checked_coefficient =
      [](double value, const SignalType& input, const SignalType& output) {
        if (value != 0.0)
          (void)coefficient_dimension(input, output);
      };
  // Validate every emitted coefficient before constructing or copying graph
  // data. Exact zero coefficients have no dependency or coefficient type.
  for (Eigen::Index row = 0; row < n; ++row) {
    const auto output = derivative_type(channels.states[static_cast<std::size_t>(row)]);
    for (Eigen::Index column = 0; column < n; ++column)
      checked_coefficient(
          system.a(row, column), channels.states[static_cast<std::size_t>(column)], output);
    for (Eigen::Index column = 0; column < m; ++column)
      checked_coefficient(
          system.b(row, column), channels.inputs[static_cast<std::size_t>(column)], output);
  }
  for (Eigen::Index row = 0; row < p; ++row) {
    const auto& output = channels.outputs[static_cast<std::size_t>(row)];
    for (Eigen::Index column = 0; column < n; ++column)
      checked_coefficient(
          c(row, column), channels.states[static_cast<std::size_t>(column)], output);
    for (Eigen::Index column = 0; column < m; ++column)
      checked_coefficient(
          d(row, column), channels.inputs[static_cast<std::size_t>(column)], output);
  }
  if (feedback) {
    for (Eigen::Index row = 0; row < m; ++row)
      for (Eigen::Index column = 0; column < n; ++column)
        checked_coefficient(options.feedback_gain(row, column),
                            channels.states[static_cast<std::size_t>(column)],
                            channels.inputs[static_cast<std::size_t>(row)]);
  }

  LinearGraph result;
  result.model.profile = kLinearProfile;
  const auto populate_ids =
      [](std::vector<std::string>& ids, std::string_view prefix, Eigen::Index count) {
        ids.reserve(static_cast<std::size_t>(count));
        for (Eigen::Index i = 0; i < count; ++i)
          ids.push_back(channel_id(prefix, i));
      };
  populate_ids(result.state_ids, "state_", n);
  populate_ids(result.command_ids, "command_", m);
  populate_ids(result.control_ids, "control_", m);
  populate_ids(result.output_ids, "output_", p);
  populate_ids(result.control_output_ids, "control_output_", m);
  // 2n state/derivative blocks, 3m command/control/scope blocks and 2p
  // measurement/output blocks. Bounds are safely below the graph limits.
  result.model.blocks.reserve(static_cast<std::size_t>(2 * n + 3 * m + 2 * p));
  const auto append_term = [&](LinearCombination& combination,
                               const std::string& target,
                               const SignalType& output,
                               const std::string& source,
                               const SignalType& input,
                               double value) {
    if (value == 0.0)
      return;
    const auto port = combination.terms.size();
    combination.terms.push_back({input, Gain{value, coefficient_dimension(input, output)}});
    result.model.connections.push_back({source, target, port});
  };
  const auto append_row =
      [&](const std::string& id, const SignalType& output, LinearCombination row) {
        if (row.terms.empty())
          result.model.blocks.push_back({id, output, Constant{0.0}});
        else
          result.model.blocks.push_back({id, output, std::move(row)});
      };
  for (Eigen::Index row = 0; row < m; ++row) {
    const auto slot = static_cast<std::size_t>(row);
    const auto& type = channels.inputs[slot];
    result.model.blocks.push_back({result.command_ids[slot], type, Constant{options.command(row)}});
    LinearCombination control;
    append_term(control, result.control_ids[slot], type, result.command_ids[slot], type, 1.0);
    if (feedback) {
      for (Eigen::Index column = 0; column < n; ++column) {
        const auto state_slot = static_cast<std::size_t>(column);
        append_term(control,
                    result.control_ids[slot],
                    type,
                    result.state_ids[state_slot],
                    channels.states[state_slot],
                    -options.feedback_gain(row, column));
      }
    }
    append_row(result.control_ids[slot], type, std::move(control));
    result.model.blocks.push_back({result.control_output_ids[slot], type, Output{}});
    result.model.connections.push_back(
        {result.control_ids[slot], result.control_output_ids[slot], 0});
  }
  for (Eigen::Index row = 0; row < n; ++row) {
    const auto slot = static_cast<std::size_t>(row);
    const auto& state_type = channels.states[slot];
    const auto type = derivative_type(state_type);
    const auto id = channel_id("derivative_", row);
    result.model.blocks.push_back(
        {result.state_ids[slot], state_type, Integrator{options.initial_state(row)}});
    LinearCombination derivative;
    for (Eigen::Index column = 0; column < n; ++column) {
      const auto state_slot = static_cast<std::size_t>(column);
      append_term(derivative,
                  id,
                  type,
                  result.state_ids[state_slot],
                  channels.states[state_slot],
                  system.a(row, column));
    }
    for (Eigen::Index column = 0; column < m; ++column) {
      const auto input_slot = static_cast<std::size_t>(column);
      append_term(derivative,
                  id,
                  type,
                  result.control_ids[input_slot],
                  channels.inputs[input_slot],
                  system.b(row, column));
    }
    append_row(id, type, std::move(derivative));
    result.model.connections.push_back({id, result.state_ids[slot], 0});
  }
  for (Eigen::Index row = 0; row < p; ++row) {
    const auto slot = static_cast<std::size_t>(row);
    const auto& type = channels.outputs[slot];
    const auto id = channel_id("output_value_", row);
    LinearCombination output;
    for (Eigen::Index column = 0; column < n; ++column) {
      const auto state_slot = static_cast<std::size_t>(column);
      append_term(output,
                  id,
                  type,
                  result.state_ids[state_slot],
                  channels.states[state_slot],
                  c(row, column));
    }
    for (Eigen::Index column = 0; column < m; ++column) {
      const auto input_slot = static_cast<std::size_t>(column);
      append_term(output,
                  id,
                  type,
                  result.control_ids[input_slot],
                  channels.inputs[input_slot],
                  d(row, column));
    }
    append_row(id, type, std::move(output));
    result.model.blocks.push_back({result.output_ids[slot], type, Output{}});
    result.model.connections.push_back({id, result.output_ids[slot], 0});
  }
  validate_model(result.model);
  return result;
}

}  // namespace galata::modeling
