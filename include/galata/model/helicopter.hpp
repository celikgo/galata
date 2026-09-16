// SPDX-License-Identifier: Apache-2.0
//
// A Level-1 single-main-rotor helicopter: main rotor, tail rotor, fuselage and
// empennage aerodynamics, engine, governor and actuators, on the shared
// six-degree-of-freedom kernel.
//
// Reference:
//   G. D. Padfield, "Helicopter Flight Dynamics: The Theory and Application of
//   Flying Qualities and Simulation Modelling", 2nd ed., Blackwell, 2007,
//   chapter 3 — the component build-up, the Level-1 fidelity definition this
//   model is written to, and the empennage and fuselage treatments.
//   P. D. Talbot, B. E. Tinling, W. A. Decker and R. T. N. Chen, "A
//   Mathematical Model of a Single Main Rotor Helicopter for Piloted
//   Simulation", NASA TM-84281, 1982 — the component decomposition and the
//   main-rotor-on-empennage interference treatment.
//   R. K. Heffley and M. A. Mnich, "Minimum-Complexity Helicopter Simulation
//   Math Model", NASA CR-177476 / USAAVSCOM TR-87-A-7, 1988 — the reduced
//   parameter set a Level-1 model needs and what each parameter does.
//   J. G. Leishman, "Principles of Helicopter Aerodynamics", 2nd ed.,
//   Cambridge, 2006, chapter 5 — helicopter power and the anti-torque balance.
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 2 — the six-degree-of-freedom
//   equations, which this model does not restate: it calls the shared kernel.
//
// Conventions are ADR-0002; the equations are written about the CG per ADR-0006
// and there is no second six-degree-of-freedom implementation here.
//
// THE STATE, in the order it is integrated, fingerprinted and serialised:
//
//   x = [ x_13                     the ADR-0002 rigid-body state, FIRST
//         main_rotor_speed_rad_s   Omega, rad/s
//         main_inflow_ratio        lambda_i of the main rotor, dimensionless
//         tail_inflow_ratio        lambda_i of the tail rotor, dimensionless
//         collective_rad           actuator POSITION, not the command
//         longitudinal_cyclic_rad
//         lateral_cyclic_rad
//         pedal_rad ]
//
// THE CONTROLS ARE COMMANDS, AND THE STATES ARE POSITIONS. The four inputs are
// what the pilot or the control law ASKS for; the four actuator states are what
// the swashplate actually did, after a rate limit, a position limit and a
// first-order lag. Conflating them is how a control law that saturates its
// actuators looks stable in simulation and is not in flight.
//
// THE ANTI-TORQUE BALANCE IS THE THING TO GET RIGHT. The main rotor's shaft
// torque reacts onto the airframe about the yaw axis; the tail rotor's thrust
// at its moment arm opposes it. Both signs follow from `spin_about_shaft` and
// the tail hub's rotation, and `validate()` CHECKS that the pair is consistent
// — that pedal in the positive sense opposes the main rotor's torque rather
// than adding to it. A helicopter with that backwards still trims, at the
// opposite pedal, and departs the moment it is disturbed.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * NOT A VALIDATED MODEL OF ANY AIRCRAFT. The shipped parameter set is derived
//   from the Souxmar preliminary design package, which is a DESIGN STUDY and
//   not a measured aircraft. models/souxmar-heli/PROVENANCE.md records where
//   every number came from, which are measurements (none), which are design
//   inputs, and which are assumptions this model had to add. A completed run is
//   evidence about the equations, never about an aircraft.
//
// * NOT VALID IN AUTOROTATION. The engine and governor model supplies torque to
//   hold rotor speed; with the engine failed the rotor decays under its own
//   drag, which is the right first-order behaviour. What is NOT modelled is the
//   autorotative energy exchange that keeps a real rotor turning — the inboard
//   blade sections driving while the outboard ones absorb — because that needs
//   a spanwise blade-element integration this rotor does not have. An
//   autorotation computed here decays FASTER than the real aircraft and must
//   not be read as a descent-rate prediction.
//
// * NOT VALID IN THE VORTEX-RING STATE, for the reason the rotor's own header
//   gives: momentum theory has no solution there and this model goes on
//   producing a smooth one. `envelope()` reports the condition.
//
// * NO GROUND EFFECT AND NO GROUND CONTACT. Thrust within about one rotor
//   diameter of the ground is UNDER-PREDICTED, so a hover trim near the ground
//   asks for more collective and more power than the aircraft needs. There is
//   no landing gear, no wheel, no contact force: the model will fly through the
//   ground without noticing.
//
// * NO RETREATING-BLADE STALL AND NO COMPRESSIBILITY. These are the two effects
//   that actually limit a helicopter's forward speed. Thrust and control power
//   above the declared C_T/sigma and mu limits are OPTIMISTIC — the real rotor
//   would have stalled and produced less. `envelope()` reports the departure;
//   nothing refuses it.
//
// * NO DYNAMIC FLAPPING, NO LEAD-LAG, NO AIR OR GROUND RESONANCE, NO
//   AEROELASTICITY. The flap regressing mode is assumed fast relative to the
//   body and its transient is not carried, so a control law with bandwidth
//   approaching the flap frequency will look MORE stable here than in flight.
//   Lead-lag is absent entirely, so the rotor-body couplings that produce
//   ground and air resonance cannot appear at all.
//
// * NO MAIN-ROTOR WAKE ON THE EMPENNAGE beyond a declared constant blockage
//   factor on the tail rotor and a declared downwash factor on the horizontal
//   stabiliser. The real interference varies strongly with advance ratio, and
//   through transition it is the dominant source of the pitch-attitude changes
//   pilots actually notice. This model will not show them.
//
// * NOT VARIABLE MASS. Fuel burn does not change mass, CG or inertia.

#ifndef GALATA_MODEL_HELICOPTER_HPP
#define GALATA_MODEL_HELICOPTER_HPP

#include "galata/model/rotor/rotor.hpp"
#include "galata/model/vehicle.hpp"
#include "galata/numerics/integration_method.hpp"
#include "galata/numerics/table.hpp"

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace galata::model {

// Named indices into the helicopter's AUXILIARY state, which begins at
// core::kStateSize in the extended vector.
enum HelicopterAuxIndex : int {
  kMainRotorSpeed = 0,       // rad/s
  kMainInflowRatio = 1,      // dimensionless
  kTailInflowRatio = 2,      // dimensionless
  kCollectivePosition = 3,   // rad
  kLongitudinalCyclicPosition = 4,  // rad
  kLateralCyclicPosition = 5,       // rad
  kPedalPosition = 6,               // rad
  kHelicopterAuxCount = 7,
};

enum HelicopterControlIndex : int {
  kCollectiveCommand = 0,           // rad
  kLongitudinalCyclicCommand = 1,   // rad
  kLateralCyclicCommand = 2,        // rad
  kPedalCommand = 3,                // rad
  kHelicopterControlCount = 4,
};

// One actuator: a position limit, a rate limit and a first-order lag.
//
// All three, because all three bite. A position limit alone lets a control law
// slew infinitely fast to its stop; a rate limit alone lets it sit beyond the
// mechanical travel; a lag alone lets it do both.
struct ActuatorLimits {
  double minimum_rad = 0.0;        // rad
  double maximum_rad = 0.0;        // rad
  double rate_limit_rad_s = 0.0;   // rad/s, positive
  double time_constant_s = 0.0;    // s, positive
};

// Fuselage and empennage, as component aerodynamics.
//
// Every coefficient here is referenced to the DYNAMIC PRESSURE and a declared
// reference area, so a component's force is q * S * C. The fuselage's
// equivalent flat-plate area is the exception and is quoted as an AREA because
// that is how preliminary design states it.
struct HelicopterAirframe {
  // Equivalent flat-plate area, f = D / q, at zero incidence. m^2.
  double flat_plate_area_m2 = 0.0;  // m^2

  // Optional incidence dependence of the fuselage. Both tables take the
  // incidence in RADIANS and return a dimensionless coefficient referenced to
  // `flat_plate_area_m2`. Absent means "no incidence dependence", which is a
  // declared modelling choice and a visible one.
  std::optional<numerics::Table1D> fuselage_lift_vs_alpha;
  std::optional<numerics::Table1D> fuselage_pitching_moment_vs_alpha;

  // Position of the fuselage aerodynamic reference relative to the CG, body axes.
  Eigen::Vector3d cg_to_fuselage_reference_body_m = Eigen::Vector3d::Zero();  // m

  // Horizontal stabiliser: area, lift-curve slope, incidence and arm.
  double horizontal_tail_area_m2 = 0.0;          // m^2
  double horizontal_tail_lift_slope = 0.0;       // 1/rad
  double horizontal_tail_incidence_rad = 0.0;    // rad, positive leading edge up
  Eigen::Vector3d cg_to_horizontal_tail_body_m = Eigen::Vector3d::Zero();  // m
  // Fraction of the main-rotor induced velocity seen at the horizontal tail.
  // Declared, because the real value varies strongly with advance ratio and a
  // constant is an approximation this model states rather than hides.
  double horizontal_tail_downwash_factor = 0.0;  // dimensionless

  // Vertical stabiliser: area, side-force slope, incidence and arm.
  double vertical_tail_area_m2 = 0.0;         // m^2
  double vertical_tail_side_slope = 0.0;      // 1/rad
  double vertical_tail_incidence_rad = 0.0;   // rad, positive nose-left side force
  Eigen::Vector3d cg_to_vertical_tail_body_m = Eigen::Vector3d::Zero();  // m

  // Stall angle beyond which a stabiliser's linear slope is held rather than
  // extrapolated. A linear surface goes on lifting for ever, and through
  // transition a helicopter's horizontal tail really does reach it.
  double surface_stall_angle_rad = 0.0;  // rad, positive
};

// Engine, drivetrain and governor.
//
// The rotor-speed state is real: rotor inertia times its rate equals the
// engine's supplied torque minus everything the rotors demand. That is what
// makes collective-induced droop appear.
struct HelicopterDrivetrain {
  double reference_rotor_speed_rad_s = 0.0;  // Omega_ref, rad/s, the governor's target
  double tail_gear_ratio = 0.0;              // Omega_tail / Omega_main, dimensionless, positive

  // Governor: proportional gain on the speed error, and an integral gain that
  // removes the steady droop a proportional-only governor leaves.
  double governor_proportional_n_m_s = 0.0;  // N m per (rad/s) of error
  double governor_time_constant_s = 0.0;     // s, engine torque response lag

  double maximum_engine_torque_n_m = 0.0;  // N m, at the main-rotor shaft
  double minimum_engine_torque_n_m = 0.0;  // N m, >= 0; a turboshaft cannot motor the rotor
  double transmission_efficiency = 1.0;    // dimensionless, in (0, 1]
  double accessory_torque_n_m = 0.0;       // N m, constant extraction at the main shaft

  // Rotor-speed limits, advisory. Reported by envelope(), not enforced.
  double minimum_rotor_speed_rad_s = 0.0;  // rad/s
  double maximum_rotor_speed_rad_s = 0.0;  // rad/s
};

// Failure hooks. Every field defaults to "no failure", and each is a
// MULTIPLIER or a flag rather than a special case in the physics, so a failed
// run goes down exactly the same code path as a healthy one.
struct HelicopterFailures {
  // Tail-rotor thrust multiplier. 0.0 is complete loss of anti-torque.
  double tail_rotor_effectiveness = 1.0;  // dimensionless, in [0, 1]
  // Engine torque multiplier. 0.0 is a flameout.
  double engine_available_fraction = 1.0;  // dimensionless, in [0, 1]
  // Per-actuator jam. When set, the actuator holds this position regardless of
  // command. Index by HelicopterControlIndex.
  std::array<std::optional<double>, kHelicopterControlCount> jammed_actuator_rad{};

  [[nodiscard]] bool any() const;
};

class HelicopterModel final : public VehicleModel {
 public:
  HelicopterModel() = default;

  std::string description_text;  // reaches the report; says what the model IS
  std::string citation;          // where its numbers came from

  sim::MassProperties mass;
  rotor::RotorGeometry main_rotor;
  rotor::RotorGeometry tail_rotor;
  HelicopterAirframe airframe;
  HelicopterDrivetrain drivetrain;
  HelicopterFailures failures;

  // Tail-rotor blockage: the fraction of its geometric thrust the tail rotor
  // actually delivers, after the fin blocks part of its disc. Declared because
  // it is a real 5-20% effect that every helicopter has and no rotor model
  // produces on its own.
  double tail_rotor_blockage_factor = 1.0;  // dimensionless, in (0, 1]

  // Pedal-to-tail-collective gearing, and the sense. Positive `pedal_rad`
  // produces `pedal_to_tail_collective * pedal_rad` of tail collective.
  double pedal_to_tail_collective = 1.0;  // dimensionless

  std::array<ActuatorLimits, kHelicopterControlCount> actuators{};

  // ---- VehicleModel ----------------------------------------------------
  [[nodiscard]] std::string description() const override;
  [[nodiscard]] std::vector<std::string> state_names() const override;
  [[nodiscard]] std::vector<std::string> control_names() const override;
  [[nodiscard]] std::vector<std::string> output_names() const override;
  [[nodiscard]] int auxiliary_state_count() const override;
  [[nodiscard]] sim::MassProperties mass_properties(
      const Eigen::VectorXd& auxiliary) const override;
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
  [[nodiscard]] Eigen::VectorXd outputs(const core::State& state,
                                        const Eigen::VectorXd& auxiliary,
                                        const Eigen::VectorXd& controls,
                                        const Environment& environment) const override;

  // ---- helicopter-specific --------------------------------------------

  // Everything the component build-up computed at one point, for a report and
  // for the trim's own diagnostics.
  struct Breakdown {
    rotor::RotorSolution main;
    rotor::RotorSolution tail;
    sim::Wrench fuselage;
    sim::Wrench horizontal_tail;
    sim::Wrench vertical_tail;
    sim::Wrench total;  // excluding gravity, as VehicleModel::wrench returns
    double engine_torque_n_m = 0.0;     // N m at the main shaft
    double total_power_w = 0.0;         // W
    double rotor_speed_rad_s = 0.0;     // rad/s
    double anti_torque_residual_n_m = 0.0;  // N m; zero in a yaw-trimmed state
  };
  [[nodiscard]] Breakdown breakdown(const core::State& state,
                                    const Eigen::VectorXd& auxiliary,
                                    const Eigen::VectorXd& controls,
                                    const Environment& environment) const;

  // A starting auxiliary state: rotor at reference speed, inflows at their
  // hover momentum values for the given mass, actuators at the commanded
  // positions. Used as a trim initial guess and as a simulation initial
  // condition, so that neither has to know the layout.
  [[nodiscard]] Eigen::VectorXd initial_auxiliary(const Eigen::VectorXd& controls,
                                                  const Environment& environment) const;

  // Declared magnitude bounds for every state, for the simulation to refuse a
  // divergence against. Generous: these are "this is not a helicopter any more"
  // limits, not envelope limits.
  [[nodiscard]] numerics::StateBounds state_bounds() const;

  // Throws std::invalid_argument on anything a run would otherwise discover
  // late or not at all — including the anti-torque SENSE check described in
  // the file header.
  void validate() const;
};

// Load from the YAML contract documented in docs/MODEL_FILES.md. Unknown keys
// are errors, as everywhere else in this project.
[[nodiscard]] HelicopterModel load_helicopter(const std::string& path);
[[nodiscard]] HelicopterModel parse_helicopter(const std::string& bytes,
                                               const std::string& origin);

}  // namespace galata::model

#endif  // GALATA_MODEL_HELICOPTER_HPP
