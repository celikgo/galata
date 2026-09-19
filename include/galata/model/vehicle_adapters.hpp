// SPDX-License-Identifier: Apache-2.0
//
// Compatibility adapters for the pre-ADR-0020 aircraft and multirotor
// classes.  They are intentionally small: the model-specific equations stay
// in Aircraft/Quadrotor, while composition, environment bookkeeping,
// quaternion projection and downstream vocabulary are owned by VehicleModel.
#ifndef GALATA_MODEL_VEHICLE_ADAPTERS_HPP
#define GALATA_MODEL_VEHICLE_ADAPTERS_HPP

#include "galata/model/aircraft.hpp"
#include "galata/model/helicopter.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/model/vehicle.hpp"

#include <utility>

namespace galata::model {

class FixedWingVehicleModel final : public VehicleModel {
 public:
  explicit FixedWingVehicleModel(Aircraft aircraft) : aircraft_(std::move(aircraft)) {}

  [[nodiscard]] const Aircraft& source_model() const noexcept { return aircraft_; }
  [[nodiscard]] Aircraft& source_model() noexcept { return aircraft_; }
  [[nodiscard]] std::string description() const override;
  [[nodiscard]] std::vector<std::string> state_names() const override;
  [[nodiscard]] std::vector<std::string> control_names() const override;
  [[nodiscard]] std::vector<std::string> output_names() const override;
  [[nodiscard]] int auxiliary_state_count() const override { return 0; }
  [[nodiscard]] sim::MassProperties mass_properties(const Eigen::VectorXd&) const override;
  [[nodiscard]] sim::Wrench wrench(const core::State& state,
                                   const Eigen::VectorXd& auxiliary,
                                   const Eigen::VectorXd& controls,
                                   const Environment& environment) const override;
  [[nodiscard]] Eigen::VectorXd auxiliary_derivative(const core::State&,
                                                     const Eigen::VectorXd& auxiliary,
                                                     const Eigen::VectorXd&,
                                                     const Environment&) const override;
  [[nodiscard]] EnvelopeStatus envelope(const core::State& state,
                                        const Eigen::VectorXd& auxiliary,
                                        const Eigen::VectorXd& controls,
                                        const Environment& environment) const override;
  [[nodiscard]] std::vector<std::string> supported_operations() const override;

 private:
  Aircraft aircraft_;
};

class MultirotorVehicleModel final : public VehicleModel {
 public:
  explicit MultirotorVehicleModel(Quadrotor quadrotor) : quadrotor_(std::move(quadrotor)) {}

  [[nodiscard]] const Quadrotor& source_model() const noexcept { return quadrotor_; }
  [[nodiscard]] Quadrotor& source_model() noexcept { return quadrotor_; }
  [[nodiscard]] std::string description() const override;
  [[nodiscard]] std::vector<std::string> state_names() const override;
  [[nodiscard]] std::vector<std::string> control_names() const override;
  [[nodiscard]] std::vector<std::string> output_names() const override;
  [[nodiscard]] int auxiliary_state_count() const override;
  [[nodiscard]] sim::MassProperties mass_properties(const Eigen::VectorXd& auxiliary) const override;
  [[nodiscard]] sim::Wrench wrench(const core::State& state,
                                   const Eigen::VectorXd& auxiliary,
                                   const Eigen::VectorXd& controls,
                                   const Environment& environment) const override;
  [[nodiscard]] Eigen::VectorXd auxiliary_derivative(const core::State& state,
                                                     const Eigen::VectorXd& auxiliary,
                                                     const Eigen::VectorXd& controls,
                                                     const Environment& environment) const override;
  [[nodiscard]] EnvelopeStatus envelope(const core::State& state,
                                        const Eigen::VectorXd& auxiliary,
                                        const Eigen::VectorXd& controls,
                                        const Environment& environment) const override;
  [[nodiscard]] std::vector<ChannelMetadata> control_metadata() const override;
  [[nodiscard]] std::vector<std::string> supported_operations() const override;

 private:
  Quadrotor quadrotor_;
};

// The helicopter already is a VehicleModel.  This adapter gives the generic
// pipeline an owning, type-erased object without changing the established
// helicopter class or its public capability names.
class HelicopterVehicleAdapter final : public VehicleModel {
 public:
  explicit HelicopterVehicleAdapter(HelicopterModel helicopter)
      : helicopter_(std::move(helicopter)) {}

  [[nodiscard]] const HelicopterModel& source_model() const noexcept { return helicopter_; }
  [[nodiscard]] HelicopterModel& source_model() noexcept { return helicopter_; }
  [[nodiscard]] std::string description() const override { return helicopter_.description(); }
  [[nodiscard]] std::vector<std::string> state_names() const override {
    return helicopter_.state_names();
  }
  [[nodiscard]] std::vector<std::string> control_names() const override {
    return helicopter_.control_names();
  }
  [[nodiscard]] std::vector<std::string> output_names() const override {
    return helicopter_.output_names();
  }
  [[nodiscard]] int auxiliary_state_count() const override {
    return helicopter_.auxiliary_state_count();
  }
  [[nodiscard]] sim::MassProperties mass_properties(const Eigen::VectorXd& auxiliary) const override {
    return helicopter_.mass_properties(auxiliary);
  }
  [[nodiscard]] sim::Wrench wrench(const core::State& state,
                                   const Eigen::VectorXd& auxiliary,
                                   const Eigen::VectorXd& controls,
                                   const Environment& environment) const override {
    return helicopter_.wrench(state, auxiliary, controls, environment);
  }
  [[nodiscard]] Eigen::VectorXd auxiliary_derivative(const core::State& state,
                                                     const Eigen::VectorXd& auxiliary,
                                                     const Eigen::VectorXd& controls,
                                                     const Environment& environment) const override {
    return helicopter_.auxiliary_derivative(state, auxiliary, controls, environment);
  }
  [[nodiscard]] EnvelopeStatus envelope(const core::State& state,
                                        const Eigen::VectorXd& auxiliary,
                                        const Eigen::VectorXd& controls,
                                        const Environment& environment) const override {
    return helicopter_.envelope(state, auxiliary, controls, environment);
  }
  [[nodiscard]] Eigen::VectorXd outputs(const core::State& state,
                                         const Eigen::VectorXd& auxiliary,
                                         const Eigen::VectorXd& controls,
                                         const Environment& environment) const override {
    return helicopter_.outputs(state, auxiliary, controls, environment);
  }
  [[nodiscard]] std::vector<ChannelMetadata> control_metadata() const override;
  [[nodiscard]] std::vector<std::string> supported_operations() const override;

 private:
  HelicopterModel helicopter_;
};

}  // namespace galata::model

#endif  // GALATA_MODEL_VEHICLE_ADAPTERS_HPP
