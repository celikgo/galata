// SPDX-License-Identifier: Apache-2.0
// Versioned continuous scalar graph foundation. See ADR-0010 and
// docs/architecture/MODEL_CONFORMANCE.md for semantics and acceptance bounds.
// This profile is not an aircraft model, sampled controller, algebraic-loop
// solver, error-controlled integrator, or qualified development tool.
#ifndef GALATA_MODELING_MODEL_HPP
#define GALATA_MODELING_MODEL_HPP

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace galata::modeling {

inline constexpr std::string_view kSchema = "galata.model.v1";
inline constexpr std::string_view kProfile = "continuous-scalar.v1";
inline constexpr std::string_view kLinearProfile = "continuous-linear.v1";
inline constexpr std::size_t kMaxSourceBytes = 1024 * 1024;
inline constexpr std::size_t kMaxBlocks = 1024;
inline constexpr std::size_t kMaxConnections = 8192;
inline constexpr std::size_t kMaxSumInputs = 64;
inline constexpr std::size_t kMaxLinearTerms = 64;
inline constexpr int kMaxSteps = 1000000;
inline constexpr std::size_t kMaxRecordedScalars = 1000000;
inline constexpr std::size_t kMaxBlockEvaluations = 100000000;

enum class ErrorCode {
  InvalidDocument,
  InvalidModel,
  TypeMismatch,
  AlgebraicLoop,
  ResourceLimit,
  InvalidSimulation,
  NonFiniteEvaluation,
  Cancelled
};

class Error : public std::runtime_error {
 public:
  Error(ErrorCode code, std::string message, std::string block_id = {});

  [[nodiscard]] ErrorCode code() const noexcept {
    return code_;
  }

  [[nodiscard]] const std::string& block_id() const noexcept {
    return block_id_;
  }

 private:
  ErrorCode code_;
  std::string block_id_;
};

// Canonical SI exponents: length, mass, time, electric current, temperature,
// amount of substance, luminous intensity, angle. Angle is an additional
// semantic dimension to prevent accidental angle/unitless connections.
// Each exponent is in [-16, 16]. No scale or offset conversions are implicit.
using Dimension = std::array<int, 8>;
enum class Frame { None, Body, Ned };

struct SignalType {
  Dimension dimension{};
  Frame frame = Frame::None;
  bool operator==(const SignalType&) const = default;
};

struct Constant {
  double value = 0.0;
};  // in the output's canonical SI units

struct Gain {
  double value = 1.0;  // in the coefficient's canonical SI units
  Dimension dimension{};
};

struct Sum {
  std::vector<int> signs;
};  // ordered input ports, each +1 or -1

struct Integrator {
  double initial_value = 0.0;
};  // output's canonical SI units

struct Output {};

// Ordered coordinate coupling, admitted only by continuous-linear.v1. Each
// incoming signal must exactly match its declared input type. The coefficient
// supplies the dimension difference to the output. Distinct input/output
// frames explicitly declare linear coupling; no coordinate rotation is inferred.
struct LinearTerm {
  SignalType input;
  Gain coefficient;
};

struct LinearCombination {
  std::vector<LinearTerm> terms;
};

using Parameters = std::variant<Constant, Gain, Sum, Integrator, Output, LinearCombination>;

struct Block {
  std::string id;  // [A-Za-z_][A-Za-z0-9_-]{0,63}; stable across source reordering
  SignalType output;
  Parameters parameters;
};

struct Connection {
  std::string source;  // source block's single output
  std::string target;
  std::size_t input = 0;  // zero-based target input port
};

struct Model {
  std::string schema{kSchema};
  std::string profile{kProfile};
  std::vector<Block> blocks;
  std::vector<Connection> connections;
};

// Draft entry points accept incomplete graph connections, but require the same
// closed YAML syntax, schema/profile, bounded collections and valid scalar
// metadata as executable models. Drafts may have no blocks or outputs, unknown
// endpoints, unconnected/invalid ports, type mismatches or instantaneous cycles.
// A draft is never executable and has no semantic digest until full validation
// and compilation succeed. The ordinary parser and serializers remain strict.
void validate_model_draft(const Model& model);
[[nodiscard]] Model parse_model_draft_yaml(std::string_view source);

// These entry points reject invalid executable graphs from C++ or documents.
// YAML has closed keys, finite decimal numbers and no aliases/tags.
// Canonical bytes encode exact binary64 parameter bits and effective semantics;
// block/connection declaration order is excluded; sum and linear term port
// order is retained, including user-authored zero coefficients.
void validate_model(const Model& model);
[[nodiscard]] Model parse_model_yaml(std::string_view source);
[[nodiscard]] std::string write_model_yaml(const Model& model);
[[nodiscard]] std::string canonical_model(const Model& model);

struct Evaluation {
  Eigen::VectorXd derivatives;  // SI state units / s, in state_ids() order
  Eigen::VectorXd outputs;      // declared SI units, in output_ids() order
};

class CompiledModel {
 public:
  [[nodiscard]] const std::vector<std::string>& state_ids() const;
  [[nodiscard]] const std::vector<std::string>& output_ids() const;
  [[nodiscard]] const std::vector<std::string>& schedule_ids() const;
  [[nodiscard]] const Eigen::VectorXd& initial_state() const;
  [[nodiscard]] const std::string& semantic_sha256() const;
  [[nodiscard]] const Model& source_model() const;
  // Pure evaluation from the supplied temporary state; no state commits, I/O,
  // hidden caches, clock reads or callbacks. Valid for stateless models too.
  [[nodiscard]] Evaluation evaluate(double time_s, const Eigen::VectorXd& state) const;

 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
  explicit CompiledModel(std::shared_ptr<const Impl> impl);
  friend CompiledModel compile_model(const Model& model);
};

// Compilation additionally requires canonical YAML to fit kMaxSourceBytes,
// establishing that required source evidence is serializable before execution.
[[nodiscard]] CompiledModel compile_model(const Model& model);

struct SimulationOptions {
  double initial_time_s = 0.0;
  double step_s = 0.01;
  int step_count = 100;
  int sample_stride = 1;
};

struct SimulationResult {
  std::string semantic_sha256;
  SimulationOptions options;
  std::vector<std::string> state_ids;
  std::vector<std::string> output_ids;
  std::vector<double> times_s;
  std::vector<Eigen::VectorXd> states;
  std::vector<Eigen::VectorXd> outputs;
};

// Fixed-step RK4; tick-derived times; records initial and final endpoints,
// including zero steps and nondivisible sample strides. Hard resource limits
// apply before allocation. Cancellation is checked before work and each step;
// it throws Cancelled and returns no partially successful result. A callback
// may throw its own exception. Completion supplies no accuracy/validity claim.
[[nodiscard]] SimulationResult simulate(const CompiledModel& model,
                                        const SimulationOptions& options,
                                        const std::function<bool()>& cancelled = {});

}  // namespace galata::modeling
#endif
