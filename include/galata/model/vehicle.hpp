// SPDX-License-Identifier: Apache-2.0
//
// The interface every vehicle implements, and the one thing trim, linearisation,
// simulation, analysis and reporting all dispatch through.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation: Dynamics, Controls Design, and Autonomous Systems", 3rd ed.,
//   Wiley, 2016, chapter 2 — the separation between the six-degree-of-freedom
//   kernel and the force-and-moment model that feeds it, which is the seam this
//   interface formalises.
//
// WHY THIS EXISTS. Before it, the vehicle class was encoded in the capability
// NAME and propagated outwards: sim.nonlinear versus sim.plant, trim.level
// versus trim.hover, linearize.finitediff versus linearize.extended, and a
// chart routine that switched on artifact kind and named elevator, aileron and
// rudder in the plotting code. Adding a third vehicle class along that grain
// means a third parallel family and a third set of hard-coded charts, and the
// cost lands again on every downstream capability. Here the vehicle is DATA and
// the capabilities are generic over it. ADR-0020 records the decision and what
// was rejected.
//
// THE STATE LAYOUT IS ADR-0002's, EXTENDED, AND THE ORDER MATTERS.
//
//   x = [ x_13        the ADR-0002 rigid-body state, unchanged and FIRST
//         aux_0 .. aux_{n-1} ]   whatever else the model carries
//
// Appending after index 12 permutes nothing, so every A and B matrix already
// exported by this project keeps its meaning. `galata/model/quadrotor.hpp`
// established this convention for rotor speeds and a battery; this interface
// generalises it to rotor speed, inflow, flapping and actuator positions
// without a successor to ADR-0002.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not a plugin boundary. Every implementation is compiled in. ADR-0001
//   designs a C ABI for out-of-tree models and this is deliberately NOT it:
//   freezing an ABI around an interface whose shape is still moving would fix
//   the wrong shape. When the vehicle set has stabilised, the ABI wraps this.
//
// * Not a promise that every capability works for every vehicle. A capability
//   that needs something a model does not have — an elevator, four rotors, a
//   scalar thrust — must REFUSE by name, not silently substitute. The interface
//   makes the vehicle generic; it does not make every question answerable.
//
// * Not an algebraic-loop solver. `auxiliary_derivative` is explicit: the
//   returned rate may not depend on the rate being returned. A model needing an
//   implicit solve — an alpha-dot force derivative, an inflow state solved
//   simultaneously with thrust — has to say so and is refused, exactly as
//   `aircraft.hpp` already refuses C_L_alphadot.
//
// * Not variable-mass in the momentum sense. `mass_properties` may vary with
//   the auxiliary state, but the equations do not carry the momentum flux of
//   departing mass. See ADR-0006.

#ifndef GALATA_MODEL_VEHICLE_HPP
#define GALATA_MODEL_VEHICLE_HPP

#include "galata/core/state.hpp"
#include "galata/sim/rigid_body.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::model {

// Everything outside the vehicle that the vehicle can feel.
//
// WIND CARRIES ITS OWN TIME DERIVATIVE, and that is the point of collecting it
// here rather than passing a bare vector. `quadrotor.hpp` documents at length
// that d(v_air)/dt = a - dw/dt, that the second term was not carried, and that
// a caller stepping the wind without re-basing the state "injects the whole
// wind increment as a ground-velocity error, silently and permanently". A field
// that owns its own derivative removes that class of error from the caller.
struct Environment {
  Eigen::Vector3d gravity_ned_m_s2 = Eigen::Vector3d::Zero();    // m/s^2
  Eigen::Vector3d wind_ned_m_s = Eigen::Vector3d::Zero();        // m/s
  Eigen::Vector3d wind_rate_ned_m_s2 = Eigen::Vector3d::Zero();  // m/s^2, dw/dt
  // Geometric altitude used to evaluate the atmospheric properties below.
  // This is an atmospheric altitude, not the vehicle's height above the NED
  // origin. The helicopter pipeline freezes this environment at trim and
  // records the policy in its report; it does not silently re-evaluate the
  // atmosphere as the vehicle moves.
  double atmospheric_altitude_m = 0.0;  // m, geometric altitude
  double delta_isa_k = 0.0;             // K
  double density_kg_m3 = 0.0;           // kg/m^3
  double speed_of_sound_m_s = 0.0;      // m/s
  double pressure_pa = 0.0;             // Pa
  double temperature_k = 0.0;           // K

  // Standard sea-level air with standard gravity and still air. Every field
  // populated, because a zero density silently zeroes every aerodynamic force
  // and the trajectory that results is smooth, plausible and wrong.
  [[nodiscard]] static Environment sea_level_still_air() noexcept;

  // ISA atmosphere at the declared geometric altitude, with standard gravity
  // and still air. Throws the same diagnostic as core::isa() for an altitude
  // outside its envelope or an invalid temperature offset.
  [[nodiscard]] static Environment at_geometric_altitude(double altitude_m,
                                                         double delta_isa_k = 0.0);

  // Throws std::invalid_argument on a non-finite field, a non-positive density,
  // speed of sound, pressure or temperature.
  void validate() const;
};

// How far outside its declared validity a query has strayed.
//
// Every model answers confidently at any input it is given; a first-order lift
// curve goes on rising for ever and a momentum-theory inflow goes on producing
// thrust in a condition where the real rotor is in a vortex ring. This is the
// only defence, and it is advisory: nothing here stops a caller proceeding.
struct EnvelopeStatus {
  bool outside = false;
  double worst_departure = 0.0;  // dimensionless, 0 inside, grows outside
  std::string reason;            // empty when inside; names the quantity
};

class VehicleModel {
 public:
  virtual ~VehicleModel() = default;

  VehicleModel() = default;
  VehicleModel(const VehicleModel&) = default;
  VehicleModel(VehicleModel&&) = default;
  VehicleModel& operator=(const VehicleModel&) = default;
  VehicleModel& operator=(VehicleModel&&) = default;

  // A short identification of the model and where its numbers came from. This
  // reaches the report, so it says what the model IS, not what it does.
  [[nodiscard]] virtual std::string description() const = 0;

  // ---- vocabulary -------------------------------------------------------
  //
  // These names are what every downstream capability reports in: the rows of a
  // linearisation, the columns of a CSV, the axis labels of a chart and the
  // unknowns of a trim problem. They are part of the model's contract.

  // Exactly `13 + auxiliary_state_count()` entries, ADR-0002's thirteen first.
  [[nodiscard]] virtual std::vector<std::string> state_names() const = 0;
  [[nodiscard]] virtual std::vector<std::string> control_names() const = 0;
  [[nodiscard]] virtual std::vector<std::string> output_names() const = 0;

  [[nodiscard]] virtual int auxiliary_state_count() const = 0;

  [[nodiscard]] int control_count() const {
    return static_cast<int>(control_names().size());
  }

  [[nodiscard]] int extended_state_size() const {
    return core::kStateSize + auxiliary_state_count();
  }

  // ---- physics ----------------------------------------------------------

  // Mass properties may depend on the auxiliary state (fuel burn, a slung
  // load). They may NOT depend on the rigid-body state: a CG that moves with
  // attitude is a modelling error, not a feature.
  [[nodiscard]] virtual sim::MassProperties mass_properties(
      const Eigen::VectorXd& auxiliary) const = 0;

  // Forces and moments about the CG, in body axes, EXCLUDING gravity.
  //
  // Gravity is excluded because the kernel resolves it from the attitude, and
  // because keeping it out makes a zero-gravity torque-free validation case a
  // one-argument change rather than a special case in the caller.
  [[nodiscard]] virtual sim::Wrench wrench(const core::State& state,
                                           const Eigen::VectorXd& auxiliary,
                                           const Eigen::VectorXd& controls,
                                           const Environment& environment) const = 0;

  // Derivatives of the states the model carries beyond ADR-0002's thirteen.
  // Returns `auxiliary_state_count()` entries. Empty for a model with none.
  [[nodiscard]] virtual Eigen::VectorXd auxiliary_derivative(
      const core::State& state,
      const Eigen::VectorXd& auxiliary,
      const Eigen::VectorXd& controls,
      const Environment& environment) const = 0;

  [[nodiscard]] virtual EnvelopeStatus envelope(const core::State& state,
                                                const Eigen::VectorXd& auxiliary,
                                                const Eigen::VectorXd& controls,
                                                const Environment& environment) const = 0;

  // Named observations. Defaults to the full extended state, which is what a
  // model with nothing else to say should report.
  [[nodiscard]] virtual Eigen::VectorXd outputs(const core::State& state,
                                                const Eigen::VectorXd& auxiliary,
                                                const Eigen::VectorXd& controls,
                                                const Environment& environment) const;

  // ---- composition ------------------------------------------------------

  // dx/dt for the extended state, in the extended state's order.
  //
  // NOT virtual, and deliberately so: this is the ONE place the ADR-0002
  // ordering, the gravity resolution and the wind treatment happen. A model
  // that overrode it could permute the state or drop the wind term, and the
  // resulting A matrix would still look plausible. The pieces a model is
  // allowed to supply are the wrench and the auxiliary rate, above.
  //
  // Throws std::invalid_argument on a dimension mismatch, and std::runtime_error
  // on a non-finite wrench, mass or auxiliary rate — named, so the caller learns
  // which one.
  [[nodiscard]] Eigen::VectorXd derivative(const Eigen::VectorXd& extended_state,
                                           const Eigen::VectorXd& controls,
                                           const Environment& environment) const;

  // Split and join helpers, so no caller writes segment(13, n) by hand.
  [[nodiscard]] static core::State rigid_body_part(const Eigen::VectorXd& extended_state);
  [[nodiscard]] Eigen::VectorXd auxiliary_part(const Eigen::VectorXd& extended_state) const;
  [[nodiscard]] Eigen::VectorXd join(const core::State& state,
                                     const Eigen::VectorXd& auxiliary) const;

  // Renormalise the attitude quaternion in place. Applied after a completed
  // integration step, never between stages (see numerics/integrator.hpp).
  static void project(Eigen::VectorXd& extended_state);

  // Throws unless the vocabulary is self-consistent: name counts matching the
  // declared sizes, no empty and no duplicate names. Called by every capability
  // that loads a model, because a duplicated state name silently makes a trim
  // unknown ambiguous and a linearisation row unidentifiable.
  void validate_vocabulary() const;
};

}  // namespace galata::model

#endif  // GALATA_MODEL_VEHICLE_HPP
