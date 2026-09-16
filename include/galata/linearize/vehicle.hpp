// SPDX-License-Identifier: Apache-2.0
//
// Linearise any VehicleModel about a trim point, by central differences, in
// EULER coordinates.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 4 — numerical linearisation of
//   the six-degree-of-freedom equations and the Euler-coordinate reduction.
//   G. D. Padfield, "Helicopter Flight Dynamics", 2nd ed., Blackwell, 2007,
//   section 4.3 — the helicopter stability and control derivatives this
//   produces, and what each one means.
//   W. H. Press et al., "Numerical Recipes", 3rd ed., Cambridge, 2007, §5.7 —
//   the optimal central-difference step and its error balance.
//
// WHY EULER AND NOT THE QUATERNION. A four-component quaternion under a
// three-degree-of-freedom rotation is over-parameterised, so the Jacobian taken
// in it is SINGULAR BY CONSTRUCTION: its eigenvalues include a spurious mode at
// the origin and every modal result has to know to discard it.
// `linearize/finite_difference.hpp` makes the same choice for the same reason
// and says so at length. The alternative — an attitude-error chart, as
// `linearize/extended.hpp` uses for the multirotor — is better for a
// linearisation that will be flown back against the plant, and worse for one a
// human is going to read derivative by derivative. Both are available; this is
// the readable one.
//
// So the linear state is
//
//   [ north, east, down, u, v, w, roll, pitch, yaw, p, q, r, aux... ]
//
// twelve rigid-body coordinates and the model's own auxiliary states, in that
// order. The quaternion never appears.
//
// THE POSITION AND HEADING ROWS ARE KEPT, AND THEY ARE NOT SPURIOUS. A
// helicopter's position states are genuinely uncontrolled integrators and its
// heading is genuinely free in still air, so the A matrix has eigenvalues at
// the origin and that is a fact about the aircraft rather than a numerical
// artefact. `reduced()` drops them for a caller who wants only the dynamics,
// and it drops them BY NAME so the choice is visible.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not exact, and not a claim about the nonlinear model's behaviour anywhere
//   but at the point. A linearisation is a first-order expansion, and the
//   distance over which it is good is a property of the model, not of this
//   routine. `nonlinear_agreement()` measures it rather than assuming it.
//
// * Not valid about a NON-equilibrium. A linearisation about a point where the
//   derivative is not zero carries a constant term that the A matrix cannot
//   represent, so the trajectories it predicts drift from the plant's for a
//   reason no eigenvalue shows. The residual at the point is MEASURED here and
//   a linearisation about a point above the declared budget is REFUSED.
//
// * Not automatic differentiation. Central differences with a Richardson
//   truncation estimate per entry. A model containing a kink — a saturating
//   actuator at its stop, a table breakpoint, a rate limit on its boundary —
//   produces a Jacobian entry that is an average of two different slopes, and
//   the Richardson estimate CANNOT see it, because the error expansion it
//   assumes does not hold across a kink. The actuator-saturation check below
//   exists because that is the kink a helicopter linearisation actually hits.

#ifndef GALATA_LINEARIZE_VEHICLE_HPP
#define GALATA_LINEARIZE_VEHICLE_HPP

#include "galata/model/linear_system.hpp"
#include "galata/model/vehicle.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>

namespace galata::linearize {

struct VehicleLinearisationOptions {
  // Relative perturbation, scaled by each state's own magnitude with a floor.
  // eps^(1/3) for binary64 balances truncation against rounding for a central
  // difference (Numerical Recipes §5.7).
  double relative_step = 6.055454452393343e-06;
  double absolute_step = 6.055454452393343e-06;

  // The equilibrium residual above which the point is not a trim and the
  // linearisation is refused. In the state's own SI units per second.
  double equilibrium_tolerance = 1.0e-6;

  // Richardson truncation estimate per entry, at 2h as well as h. Costs a
  // second full Jacobian and is worth it: without it, an entry corrupted by a
  // kink is indistinguishable from a correct one.
  bool estimate_truncation_error = true;
};

struct VehicleLinearisation {
  Eigen::MatrixXd a;  // d(state rate) / d(state)
  Eigen::MatrixXd b;  // d(state rate) / d(control)

  std::vector<std::string> state_names;
  std::vector<std::string> control_names;

  // The point it was taken about.
  Eigen::VectorXd trim_extended_state;
  Eigen::VectorXd trim_controls;

  // MEASURED at the point, not assumed. This is the number that says whether
  // the linearisation is about an equilibrium at all.
  double equilibrium_residual = 0.0;

  // Worst Richardson truncation estimate over all entries, relative to the
  // entry's own magnitude, and where it was. Zero when not estimated.
  double worst_relative_truncation = 0.0;
  int worst_truncation_row = -1;
  int worst_truncation_column = -1;

  // ACTUATORS THAT WERE AT A STOP WHEN THE JACOBIAN WAS TAKEN, by name. A
  // saturated actuator makes its column a one-sided difference dressed as a
  // central one, and the resulting control derivative is HALF its true value
  // with no indication that anything happened. Reported so a control design
  // built on this matrix knows.
  std::vector<std::string> saturated_controls;

  [[nodiscard]] model::LinearSystem to_linear_system(const std::string& description) const;

  // Drop the three position states and the heading, which are uncontrolled
  // integrators rather than dynamics. Dropped BY NAME, so the choice is visible
  // in the returned system's own state names.
  [[nodiscard]] VehicleLinearisation reduced() const;
};

// Throws std::invalid_argument on a malformed request, and std::runtime_error
// when the point's equilibrium residual exceeds the declared tolerance — with
// the residual, the budget and the worst-offending state NAMED, because a
// linearisation about a non-equilibrium is the error this check exists to
// prevent and "not a trim" is not a diagnosis.
[[nodiscard]] VehicleLinearisation linearize_vehicle(
    const model::VehicleModel& model,
    const Eigen::VectorXd& extended_state,
    const Eigen::VectorXd& controls,
    const model::Environment& environment,
    const VehicleLinearisationOptions& options = {});

// How well the linearisation predicts the nonlinear model's response to a
// perturbation of size `epsilon`.
//
// THE POINT IS THE ORDER, NOT THE MAGNITUDE. A correct linearisation's error
// falls as epsilon SQUARED, so halving the perturbation should quarter the
// discrepancy. A first-order error — a wrong sign, a missing term, a
// linearisation about a non-equilibrium — falls only linearly, and no single
// tolerance on a single perturbation size can tell the two apart. This returns
// the measured order so a test can assert on it.
struct NonlinearAgreement {
  std::vector<double> epsilons;
  std::vector<double> discrepancies;  // ||x_nonlinear(T) - x_linear(T)||
  // log2 of consecutive discrepancy ratios. Near 2 for a correct linearisation.
  std::vector<double> observed_orders;
  double worst_order = 0.0;
  double best_order = 0.0;
};

[[nodiscard]] NonlinearAgreement nonlinear_agreement(const model::VehicleModel& model,
                                                     const VehicleLinearisation& linearisation,
                                                     const model::Environment& environment,
                                                     const Eigen::VectorXd& perturbation_direction,
                                                     const std::vector<double>& epsilons,
                                                     double horizon_s,
                                                     double step_s);

}  // namespace galata::linearize

#endif  // GALATA_LINEARIZE_VEHICLE_HPP
