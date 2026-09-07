// SPDX-License-Identifier: Apache-2.0
// Continuous scalar compilation and execution, ADR-0010. RK4 is delegated to
// the existing numerics kernel (Hairer/Norsett/Wanner 1993; Butcher 2016).
// WHAT THIS IS NOT: no algebraic solver, discrete events, stiff/error-controlled
// integration, physical validity assessment, or aircraft/controller approval.
#include "galata/modeling/model.hpp"

#include "galata/core/sha256.hpp"
#include "galata/numerics/integrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace galata::modeling {
namespace {
constexpr auto kAbsent = std::numeric_limits<std::size_t>::max();

struct Graph {
  Model source;
  std::vector<std::vector<std::size_t>> inputs;
  std::vector<std::size_t> state_slots;
  std::vector<std::size_t> states;
  std::vector<std::size_t> outputs;
  std::vector<std::size_t> schedule;
  std::vector<std::string> state_ids, output_ids, schedule_ids;
  Eigen::VectorXd initial;
};

bool valid_id(const std::string& id) {
  const auto letter = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); };
  if (id.empty() || id.size() > 64 || (!letter(id[0]) && id[0] != '_'))
    return false;
  return std::all_of(id.begin(), id.end(), [&](char c) {
    return letter(c) || (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}

void dimension_check(const Dimension& dimension, const std::string& id) {
  for (const auto exponent : dimension) {
    if (exponent < -16 || exponent > 16) {
      throw Error(ErrorCode::InvalidModel, "dimension exponents must be in [-16, 16]", id);
    }
  }
}

std::size_t input_count(const Block& block) {
  if (std::holds_alternative<Constant>(block.parameters))
    return 0;
  if (const auto* sum = std::get_if<Sum>(&block.parameters))
    return sum->signs.size();
  return 1;
}

// Iterative DFS identifies an actual cycle, excluding downstream nodes that
// Kahn's unresolved set can also contain. Bounds are the validated graph size.
std::string cycle_path(const Graph& graph, const std::vector<std::vector<std::size_t>>& consumers) {
  std::vector<int> color(consumers.size(), 0);
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  for (std::size_t start = 0; start < consumers.size(); ++start) {
    if (color[start] != 0)
      continue;
    stack.emplace_back(start, 0);
    color[start] = 1;
    while (!stack.empty()) {
      auto& [node, next] = stack.back();
      if (next == consumers[node].size()) {
        color[node] = 2;
        stack.pop_back();
        continue;
      }
      const auto child = consumers[node][next++];
      if (color[child] == 1) {
        std::string path;
        bool in_cycle = false;
        for (const auto& entry : stack) {
          if (entry.first == child)
            in_cycle = true;
          if (in_cycle)
            path += graph.source.blocks[entry.first].id + " -> ";
        }
        return path + graph.source.blocks[child].id;
      }
      if (color[child] == 0) {
        color[child] = 1;
        stack.emplace_back(child, 0);
      }
    }
  }
  return "unresolved instantaneous dependency";
}

Graph prepare(const Model& source) {
  if (source.schema != kSchema || source.profile != kProfile) {
    throw Error(ErrorCode::InvalidModel, "unsupported model schema or execution profile");
  }
  if (source.blocks.size() > kMaxBlocks || source.connections.size() > kMaxConnections) {
    throw Error(ErrorCode::ResourceLimit, "model block or connection limit exceeded");
  }
  if (source.blocks.empty())
    throw Error(ErrorCode::InvalidModel, "model must contain blocks");
  // Validate allocations and scalar metadata before copying a direct C++ model.
  for (const auto& block : source.blocks) {
    if (!valid_id(block.id))
      throw Error(ErrorCode::InvalidModel, "invalid stable block ID");
    dimension_check(block.output.dimension, block.id);
    if (block.output.frame != Frame::None && block.output.frame != Frame::Body
        && block.output.frame != Frame::Ned) {
      throw Error(ErrorCode::InvalidModel, "unsupported signal frame", block.id);
    }
    if (block.parameters.valueless_by_exception()) {
      throw Error(ErrorCode::InvalidModel, "missing block parameters", block.id);
    }
    double parameter = 0.0;
    if (const auto* constant = std::get_if<Constant>(&block.parameters))
      parameter = constant->value;
    if (const auto* gain = std::get_if<Gain>(&block.parameters)) {
      parameter = gain->value;
      dimension_check(gain->dimension, block.id);
    }
    if (const auto* state = std::get_if<Integrator>(&block.parameters))
      parameter = state->initial_value;
    if (!std::isfinite(parameter)) {
      throw Error(ErrorCode::InvalidModel, "parameters must be finite", block.id);
    }
    if (const auto* sum = std::get_if<Sum>(&block.parameters)) {
      if (sum->signs.size() > kMaxSumInputs) {
        throw Error(ErrorCode::ResourceLimit, "sum input limit exceeded", block.id);
      }
      if (sum->signs.empty() || std::any_of(sum->signs.begin(), sum->signs.end(), [](int sign) {
            return sign != 1 && sign != -1;
          })) {
        throw Error(ErrorCode::InvalidModel, "sum requires ordered +1/-1 input signs", block.id);
      }
    }
  }
  for (const auto& connection : source.connections) {
    if (!valid_id(connection.source) || !valid_id(connection.target)) {
      throw Error(ErrorCode::InvalidModel, "invalid connection endpoint ID");
    }
  }
  Graph graph;
  graph.source = source;
  auto& blocks = graph.source.blocks;
  std::sort(
      blocks.begin(), blocks.end(), [](const Block& a, const Block& b) { return a.id < b.id; });
  std::map<std::string, std::size_t> by_id;
  graph.inputs.resize(blocks.size());
  graph.state_slots.resize(blocks.size(), kAbsent);
  for (std::size_t i = 0; i < blocks.size(); ++i) {
    const auto& block = blocks[i];
    if (!by_id.emplace(block.id, i).second) {
      throw Error(ErrorCode::InvalidModel, "duplicate block ID", block.id);
    }
    graph.inputs[i].resize(input_count(block), kAbsent);
    if (std::holds_alternative<Integrator>(block.parameters)) {
      graph.state_slots[i] = graph.states.size();
      graph.states.push_back(i);
      graph.state_ids.push_back(block.id);
    }
    if (std::holds_alternative<Output>(block.parameters)) {
      graph.outputs.push_back(i);
      graph.output_ids.push_back(block.id);
    }
  }
  if (graph.outputs.empty())
    throw Error(ErrorCode::InvalidModel, "model requires an output block");
  for (const auto& connection : source.connections) {
    const auto producer = by_id.find(connection.source), target = by_id.find(connection.target);
    if (producer == by_id.end() || target == by_id.end()) {
      throw Error(ErrorCode::InvalidModel, "unknown connection endpoint", connection.target);
    }
    auto& ports = graph.inputs[target->second];
    if (connection.input >= ports.size()) {
      throw Error(ErrorCode::InvalidModel, "input port does not exist", connection.target);
    }
    if (ports[connection.input] != kAbsent) {
      throw Error(ErrorCode::InvalidModel, "multiple writers to an input port", connection.target);
    }
    ports[connection.input] = producer->second;
  }
  std::vector<std::vector<std::size_t>> consumers(blocks.size());
  std::vector<std::size_t> indegree(blocks.size(), 0);
  for (std::size_t i = 0; i < blocks.size(); ++i) {
    const auto& block = blocks[i];
    const bool integrator = std::holds_alternative<Integrator>(block.parameters);
    for (const auto producer : graph.inputs[i]) {
      if (producer == kAbsent)
        throw Error(ErrorCode::InvalidModel, "unconnected input port", block.id);
      const auto& input_type = blocks[producer].output;
      auto expected = input_type;
      if (const auto* gain = std::get_if<Gain>(&block.parameters)) {
        for (std::size_t d = 0; d < expected.dimension.size(); ++d) {
          expected.dimension[d] += gain->dimension[d];
        }
      } else if (integrator) {
        ++expected.dimension[2];  // integration multiplies by seconds
      }
      if (expected != block.output) {
        throw Error(
            ErrorCode::TypeMismatch, "input dimension/frame incompatible with output", block.id);
      }
      if (!integrator) {
        consumers[producer].push_back(i);
        ++indegree[i];
      }
    }
  }
  std::set<std::size_t> ready;
  for (std::size_t i = 0; i < blocks.size(); ++i)
    if (indegree[i] == 0)
      ready.insert(i);
  while (!ready.empty()) {
    const auto node = *ready.begin();
    ready.erase(ready.begin());
    graph.schedule.push_back(node);
    graph.schedule_ids.push_back(blocks[node].id);
    for (const auto child : consumers[node])
      if (--indegree[child] == 0)
        ready.insert(child);
  }
  if (graph.schedule.size() != blocks.size()) {
    const auto cycle = cycle_path(graph, consumers);
    throw Error(ErrorCode::AlgebraicLoop,
                "instantaneous cycle: " + cycle,
                cycle.substr(0, cycle.find(" -> ")));
  }
  graph.initial.resize(static_cast<Eigen::Index>(graph.states.size()));
  for (std::size_t i = 0; i < graph.states.size(); ++i) {
    graph.initial(static_cast<Eigen::Index>(i)) =
        std::get<Integrator>(blocks[graph.states[i]].parameters).initial_value;
  }
  return graph;
}

void valid_stage_times(double time_s, double step_s) {
  const double middle = time_s + 0.5 * step_s, end = time_s + step_s;
  if (!std::isfinite(end) || !(middle > time_s) || !(end > middle)) {
    throw Error(ErrorCode::InvalidSimulation, "step cannot advance distinct finite RK stage times");
  }
}
}  // namespace

struct CompiledModel::Impl {
  Graph graph;
  std::string hash;
};

Error::Error(ErrorCode code, std::string message, std::string block_id)
    : std::runtime_error(block_id.empty() ? std::move(message) : block_id + ": " + message),
      code_(code), block_id_(std::move(block_id)) {}

void validate_model(const Model& model) {
  (void)prepare(model);
}

CompiledModel::CompiledModel(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

CompiledModel compile_model(const Model& model) {
  // Every executable model must also fit the canonical source evidence
  // contract. Compact input spelling can otherwise conceal an oversized
  // serialized model until after a costly simulation has finished.
  (void)write_model_yaml(model);
  auto impl = std::make_shared<CompiledModel::Impl>();
  impl->graph = prepare(model);
  impl->hash = core::sha256(canonical_model(model));
  return CompiledModel(std::move(impl));
}

const std::vector<std::string>& CompiledModel::state_ids() const {
  return impl_->graph.state_ids;
}

const std::vector<std::string>& CompiledModel::output_ids() const {
  return impl_->graph.output_ids;
}

const std::vector<std::string>& CompiledModel::schedule_ids() const {
  return impl_->graph.schedule_ids;
}

const Eigen::VectorXd& CompiledModel::initial_state() const {
  return impl_->graph.initial;
}

const std::string& CompiledModel::semantic_sha256() const {
  return impl_->hash;
}

const Model& CompiledModel::source_model() const {
  return impl_->graph.source;
}

Evaluation CompiledModel::evaluate(double time_s, const Eigen::VectorXd& state) const {
  const auto& graph = impl_->graph;
  if (!std::isfinite(time_s) || state.size() != graph.initial.size()) {
    throw Error(ErrorCode::InvalidSimulation,
                "evaluation requires finite time and matching state dimension");
  }
  if (!state.allFinite())
    throw Error(ErrorCode::NonFiniteEvaluation, "nonfinite evaluation state");
  std::vector<double> signals(graph.source.blocks.size(), 0.0);
  for (const auto node : graph.schedule) {
    const auto& block = graph.source.blocks[node];
    const auto& ports = graph.inputs[node];
    double value = 0.0;
    if (const auto* constant = std::get_if<Constant>(&block.parameters)) {
      value = constant->value;
    } else if (const auto* gain = std::get_if<Gain>(&block.parameters)) {
      value = gain->value * signals[ports[0]];
    } else if (const auto* sum = std::get_if<Sum>(&block.parameters)) {
      // Explicit left fold, starting at +0; do not reassociate or sort ports.
      for (std::size_t i = 0; i < ports.size(); ++i) {
        value += static_cast<double>(sum->signs[i]) * signals[ports[i]];
        if (!std::isfinite(value)) {
          throw Error(ErrorCode::NonFiniteEvaluation, "sum intermediate overflowed", block.id);
        }
      }
    } else if (std::holds_alternative<Integrator>(block.parameters)) {
      value = state(static_cast<Eigen::Index>(graph.state_slots[node]));
    } else {
      value = signals[ports[0]];
    }
    if (!std::isfinite(value)) {
      throw Error(ErrorCode::NonFiniteEvaluation, "nonfinite block output", block.id);
    }
    signals[node] = value;
  }
  Evaluation result;
  result.derivatives.resize(static_cast<Eigen::Index>(graph.states.size()));
  result.outputs.resize(static_cast<Eigen::Index>(graph.outputs.size()));
  for (std::size_t i = 0; i < graph.states.size(); ++i) {
    result.derivatives(static_cast<Eigen::Index>(i)) = signals[graph.inputs[graph.states[i]][0]];
  }
  for (std::size_t i = 0; i < graph.outputs.size(); ++i) {
    result.outputs(static_cast<Eigen::Index>(i)) = signals[graph.outputs[i]];
  }
  return result;
}

SimulationResult simulate(const CompiledModel& model,
                          const SimulationOptions& options,
                          const std::function<bool()>& cancelled) {
  const auto& [origin, step, count, stride] = options;
  if (!std::isfinite(origin) || !std::isfinite(step) || !(step > 0.0) || !(step / 6.0 > 0.0)
      || count < 0 || stride < 1) {
    throw Error(ErrorCode::InvalidSimulation,
                "invalid finite time/positive step/count/stride contract");
  }
  if (count > kMaxSteps)
    throw Error(ErrorCode::ResourceLimit, "step count limit exceeded");
  const auto time_at = [&](int tick) { return origin + static_cast<double>(tick) * step; };
  if (!std::isfinite(static_cast<double>(count) * step) || !std::isfinite(time_at(count))) {
    throw Error(ErrorCode::InvalidSimulation, "nonfinite simulation time span");
  }
  valid_stage_times(origin, step);
  if (count > 0)
    valid_stage_times(time_at(count - 1), step);
  const auto samples =
      static_cast<std::size_t>(count / stride) + 1U + (count % stride != 0 ? 1U : 0U);
  const auto width = model.state_ids().size() + model.output_ids().size() + 1U;
  if (samples > kMaxRecordedScalars / width) {
    throw Error(ErrorCode::ResourceLimit, "recorded scalar limit exceeded");
  }
  const auto evaluations =
      samples + (model.state_ids().empty() ? 0U : 4U * static_cast<std::size_t>(count));
  if (evaluations > kMaxBlockEvaluations / model.schedule_ids().size()) {
    throw Error(ErrorCode::ResourceLimit, "scheduled block evaluation limit exceeded");
  }
  const auto check_cancelled = [&]() {
    if (cancelled && cancelled())
      throw Error(ErrorCode::Cancelled, "simulation cancelled");
  };
  check_cancelled();
  SimulationResult result;
  result.semantic_sha256 = model.semantic_sha256();
  result.options = options;
  result.state_ids = model.state_ids();
  result.output_ids = model.output_ids();
  result.times_s.reserve(samples);
  result.states.reserve(samples);
  result.outputs.reserve(samples);
  Eigen::VectorXd state = model.initial_state();
  const auto record = [&](int tick) {
    const auto evaluated = model.evaluate(time_at(tick), state);
    result.times_s.push_back(time_at(tick));
    result.states.push_back(state);
    result.outputs.push_back(evaluated.outputs);
  };
  record(0);
  for (int tick = 0; tick < count; ++tick) {
    check_cancelled();
    if (state.size() != 0) {
      try {
        state = numerics::rk4_step(
            [&](double time, const Eigen::VectorXd& temporary) {
              return model.evaluate(time, temporary).derivatives;
            },
            time_at(tick),
            state,
            step);
      } catch (const Error&) {
        throw;
      } catch (const std::invalid_argument& error) {
        throw Error(ErrorCode::InvalidSimulation, error.what());
      } catch (const std::runtime_error& error) {
        throw Error(ErrorCode::NonFiniteEvaluation, error.what());
      }
    }
    const int completed = tick + 1;
    if (completed % stride == 0 || completed == count)
      record(completed);
  }
  check_cancelled();
  return result;
}

}  // namespace galata::modeling
