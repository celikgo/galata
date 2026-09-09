// SPDX-License-Identifier: Apache-2.0
//
// Linearisation of ANY model that exposes f(x_ext, u), on a local attitude-error
// chart, with a declared observation model and named disturbance inputs.
//
// Reference:
//   B. L. Stevens, F. L. Lewis and E. N. Johnson, "Aircraft Control and
//   Simulation", 3rd ed., Wiley, 2016, chapter 3 — numerical linearisation
//   about a trim point.
//   N. Trawny and S. I. Roumeliotis, "Indirect Kalman Filter for 3D Attitude
//   Estimation", University of Minnesota MARS Lab TR-2005-002, 2005 — the
//   multiplicative attitude-error state and its right Jacobian, which is the
//   chart used here.
//   J. Sola, "Quaternion kinematics for the error-state Kalman filter",
//   arXiv:1711.02508, 2017 — the same chart, with the exponential map's
//   Jacobians written out.
//
// THIS LANDS BESIDE `finite_difference.hpp`, IT DOES NOT REPLACE IT.
//
// That routine linearises `model::Aircraft` in twelve EULER coordinates and
// reports a reduced longitudinal or lateral set. It stays exactly as it is, the
// NT-33A export it produces is unchanged, and nothing here touches it. What it
// cannot do is the reason this exists: it is bound to one model type, its chart
// is singular at ninety degrees of pitch — its own header says a vertical-climb
// trim "needs a different chart, not a smaller step" — and it has no way to
// carry a model's appended states.
//
// THE CHART. Twelve rigid-body coordinates plus one per appended state:
//
//   delta = [ p_n p_e p_d    position, NED, m
//             u   v   w      body velocity, air-relative, m/s
//             e_x e_y e_z    attitude ERROR, body axes, rad
//             p   q   r      body rates, rad/s
//             ... ]          the model's appended states, in its own order
//
// The attitude coordinate is a rotation vector `e` relating the perturbed
// attitude to the nominal one by `q = q0 * exp(e/2)`. Three coordinates for
// three degrees of freedom, so unlike a quaternion perturbation it explores no
// direction the dynamics do not have, and unlike an Euler chart it is regular
// at every attitude — the chart is rebuilt about q0 for each linearisation, so
// there is no fixed singular attitude to avoid.
//
// The chart's attitude derivative is NOT simply the body rate. With
// `q = q0 * exp(e/2)` the exact relation is `edot = J_r^-1(e) omega`, and
// `J_r^-1(e) = I + [e]x/2 + O(|e|^2)`. The first-order term is kept:
//
//   edot = omega + (e x omega) / 2
//
// At the nominal, `e = 0` and this is just `omega`. Under a CENTRAL difference
// in an attitude coordinate it is not: dropping the term puts a first-order
// error of size |omega|/2 into the attitude rows of A. That vanishes at hover,
// where omega is zero, and does not vanish in a turn — which is exactly the
// case somebody will linearise next and the one where a silently wrong Jacobian
// is hardest to notice.
//
// THE WIND COLUMNS ARE PERTURBED AT FIXED GROUND VELOCITY, AND THAT IS THE
// WHOLE REASON D HAS A DRAG FEEDTHROUGH.
//
// ADR-0002's velocity is AIR-RELATIVE, and the model's drag acts on it
// directly with no wind subtraction. Perturb the wind while holding that state
// fixed and the air-relative velocity does not move, so the drag does not
// move, so the wind columns of B are the position rows alone and D is exactly
// zero. That is a self-consistent linearisation of a vehicle nothing blows on.
//
// The physical perturbation is the other one. No force acts on the airframe at
// the instant the air mass changes speed, so the GROUND velocity is continuous
// across a wind change and the air-relative velocity jumps by exactly minus the
// wind change — `model/quadrotor.hpp` says this in its own words, and
// `QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory` re-bases
// on it. The wind columns here are taken along that same direction:
//
//   delta w  =>  delta v_air = -R^T delta w,   delta v_ground = 0
//
// so a wind column of B is `-(df/dv_air) R^T` rather than zero, and the
// specific-force rows of D carry the drag the gust puts on the airframe.
//
// THIS DOES NOT MAKE THE MODEL INCONSISTENT, and the reason is worth stating
// because it looks as though it should. Read the chart's velocity coordinate as
// the BODY-AXIS GROUND-VELOCITY perturbation. At fixed wind — which is every
// column of A — the ground and air-relative perturbations are the same vector,
// so A is unchanged by the reading. At fixed velocity coordinate they differ by
// exactly `-R^T delta w`, which is the wind column. The two halves then add up:
// for a drag `-D v_air`, A contributes `-D delta x_v` and the wind column
// contributes `+D R^T delta w`, whose sum is the true `-D delta v_air`. The
// position rate is `R v_air + w = R v_ground_body`, which the wind column
// leaves alone, and does. Nothing is counted twice and nothing is dropped.
//
// A caller that wants the other convention — wind as a pure position-rate
// disturbance at fixed air-relative velocity — declares no wind offset and
// passes the wind through the model instead. It gets a zero D and should.
//
// A FROZEN STATE IS DECLARED, NOT DISCOVERED.
//
// Some appended states have no equilibrium to be at. A powered battery is
// always discharging: `soc_dot` is strictly negative at every hover, so a point
// that is a perfect equilibrium in all six dynamic coordinates still fails an
// equilibrium test that reads the battery row. Excluding it silently would be
// the wrong fix, because the linearisation would then carry a constant term in
// that row that A cannot represent, and the constant would not be visible
// anywhere.
//
// `frozen_appended_states` names those coordinates. A frozen coordinate is
// KEPT in the chart — it is still a row and still a column, so a state that
// reaches the dynamics through some other path keeps that path — and its OWN
// derivative is declared zero rather than measured.
//
// That declaration changes WHICH SYSTEM IS BEING LINEARISED. It does not make
// the linearisation exact, and the earlier wording here that said so was wrong.
// What is linearised is the MODIFIED plant whose frozen rows are identically
// zero — a different dynamical system from the one the model integrates. Two
// consequences follow, and both matter to a reader of the resulting matrices:
//
//   * Every OTHER row is still a finite-difference approximation, with the
//     truncation and cancellation error the rest of this header describes. The
//     freeze removes a constant term from ONE row; it buys no accuracy anywhere.
//   * The result does not reproduce the discharging plant. The real vehicle's
//     state of charge falls, its rotor speed ceiling falls with terminal
//     voltage, and the true trajectory departs from this model's. The matrices
//     are valid over a horizon short against that departure, and no longer.
//
// The rate that was declared away is reported in `frozen_appended_state_rates`,
// so the reader can divide it into the state's range and see the timescale over
// which the freeze is defensible — which is exactly the horizon named above.
// Over a battery's discharge that is minutes; over a rotor lag it is
// milliseconds, which is why only the caller can say which states qualify.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not a trim solver, and not a check that one was used. It recomputes the
//   dynamic residual at the supplied point and REFUSES a point that is not an
//   equilibrium, because a linearisation about a non-equilibrium carries a
//   constant term the A matrix cannot represent. The POSITION rate is excluded
//   from that check on purpose: a relative equilibrium translates, and
//   requiring it to stand still would reject every cruise condition.
//
// * Not exact. Every entry carries a Richardson truncation estimate from the
//   shared `central_difference_jacobian`. What that estimate cannot see is a
//   discontinuity inside the perturbation window — a rate limit, a saturation,
//   a table breakpoint — where the difference quotient returns an average slope
//   the model never has while the estimate looks healthy. Nor can it see
//   cancellation. Both bite here: a plant with quadratic drag has a term that
//   is once differentiable and not twice at zero airspeed, so the hover
//   translational entries carry a FIRST-order error of h / (drag_linear /
//   drag_quadratic) rather than a second-order one, and it is the pole gates
//   that catch that rather than the estimate.
//
//   The step sizing compensates for one thing the chart would otherwise break.
//   Every chart coordinate is zero at the nominal, so the shared routine's
//   relative-step rule sees no magnitude and falls back to one absolute floor
//   for coordinates that span metres, radians and several hundred radians per
//   second. The floors are therefore derived from the state each coordinate
//   perturbs; the implementation says how, and a caller who supplies
//   `absolute_step_per_component` keeps their own.
//
// * Not a claim that the observation model is what a sensor measures. The
//   outputs here are the ideal quantities: no bias, no scale factor, no
//   misalignment, no noise, no lever arm from the CG to a sensor station and no
//   bandwidth. `BodySpecificForce` is what an ideal accelerometer AT THE CG
//   would read.
//
// * Not a model of a discharging battery, or of any other frozen coordinate.
//   A frozen state's row is zero because it was declared zero, so the exported
//   system predicts that the state never moves. It does move. The linearisation
//   is therefore valid only over a horizon short against the timescale in
//   `frozen_state_rates`, and outside that horizon it is not conservative in
//   either direction: a quadrotor whose pack drains loses ceiling, so the real
//   vehicle has LESS control authority than this model says.
//
// * Not valid outside the amplitude where the model is linear. For a plant with
//   quadratic drag that scale is stated by the model itself: the per-axis ratio
//   `drag_linear / drag_quadratic` is the airspeed at which the two terms are
//   equal, and a linearisation about hover cannot see the quadratic term at all
//   because it vanishes at zero airspeed. Above that scale the linear model
//   UNDER-PREDICTS drag, and the error grows with the square of the airspeed.

#ifndef GALATA_LINEARIZE_EXTENDED_HPP
#define GALATA_LINEARIZE_EXTENDED_HPP

#include "galata/core/constants.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/numerics/jacobian.hpp"

#include <Eigen/Core>

#include <functional>
#include <string>
#include <vector>

namespace galata::linearize {

// dx_ext/dt = f(x_ext, u). The first thirteen components of x_ext are the
// ADR-0002 rigid-body state, in its order; everything after them is the model's
// own, in the model's order.
using ExtendedDynamics =
    std::function<Eigen::VectorXd(const Eigen::VectorXd& x_ext, const Eigen::VectorXd& u)>;

// The observation model. `C = I, D = 0` is not this programme's case: the
// sensors do not measure every state, and two of these depend on an input.
enum class OutputKind {
  BodySpecificForce,  // 3, m/s^2 — what an ideal accelerometer at the CG reads
  BodyRates,          // 3, rad/s
  PositionNed,        // 3, m
  Altitude,           // 1, m, positive up, so the negative of the NED down state
  GroundVelocityNed,  // 3, m/s — body velocity rotated out, plus the wind
};

// Chart layout, fixed.
inline constexpr int kRigidChartSize = 12;

enum ChartIndex : int {
  kChartPositionNorth = 0,
  kChartPositionEast = 1,
  kChartPositionDown = 2,
  kChartVelocityU = 3,
  kChartVelocityV = 4,
  kChartVelocityW = 5,
  kChartAttitudeErrorX = 6,
  kChartAttitudeErrorY = 7,
  kChartAttitudeErrorZ = 8,
  kChartRateP = 9,
  kChartRateQ = 10,
  kChartRateR = 11,
};

struct ExtendedLinearisationOptions {
  // Names for the appended states, in the model's order. Its size sets how many
  // appended states there are, so a mismatch with x_ext is an error rather than
  // a silent truncation.
  std::vector<std::string> appended_state_names;

  // Names for every input column, including the wind columns if present.
  std::vector<std::string> input_names;

  // Index in `u` of the first of three consecutive NED wind columns, or -1 for
  // a model with no wind input. Naming them lets `model.channels` select or
  // drop the disturbance, and lets the observation model know which columns
  // carry it — the wind reaches `GroundVelocityNed` directly and
  // `BodySpecificForce` through the drag, which is the D feedthrough.
  int wind_input_offset = -1;

  std::vector<OutputKind> outputs;

  // Indices into the APPENDED block — 0 is the first state after the twelfth
  // chart coordinate — whose own derivative is declared zero rather than
  // measured. See the header. Out of range, or repeated, is an error: a caller
  // that miscounted its own appended states must find out here.
  std::vector<int> frozen_appended_states;

  Eigen::Vector3d gravity_ned_m_s2 = Eigen::Vector3d(0.0, 0.0, core::kStandardGravity);

  numerics::JacobianOptions state_jacobian;
  numerics::JacobianOptions input_jacobian;

  bool report_truncation_error = true;

  // Acceleration-norm budget on the DYNAMIC rates at the supplied point.
  double equilibrium_tolerance = 1e-10;
};

struct ExtendedLinearisation {
  Eigen::MatrixXd a;
  Eigen::MatrixXd b;
  Eigen::MatrixXd c;
  Eigen::MatrixXd d;

  std::vector<std::string> state_names;
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;

  Eigen::VectorXd state_steps;
  Eigen::VectorXd input_steps;
  // Per-entry Richardson estimates for ALL FOUR matrices. C and D are not
  // spared this: the observation model runs the dynamics to recover specific
  // force, so its rows carry the same truncation error the A rows do, and a
  // reader who trusts an accelerometer row without an estimate is trusting a
  // difference quotient nobody measured.
  Eigen::MatrixXd a_truncation;
  Eigen::MatrixXd b_truncation;
  Eigen::MatrixXd c_truncation;
  Eigen::MatrixXd d_truncation;

  // Worst over all four, so one summary number cannot be made to look healthy
  // by an unestimated matrix.
  double worst_relative_truncation = 0.0;
  double worst_relative_truncation_a = 0.0;
  double worst_relative_truncation_b = 0.0;
  double worst_relative_truncation_c = 0.0;
  double worst_relative_truncation_d = 0.0;

  // The dynamic residual actually measured at the supplied point, so the
  // linearisation cannot be separated from the evidence that it was taken about
  // an equilibrium (charter rule 9).
  double equilibrium_residual_norm = 0.0;
  double equilibrium_tolerance = 1e-10;

  // The rate each frozen coordinate actually has at this point, in the order
  // `frozen_appended_states` named them, paired with its state name. Declared
  // to zero in A; reported here so the declaration is auditable rather than
  // invisible.
  std::vector<int> frozen_appended_states;
  std::vector<std::string> frozen_state_names;
  std::vector<double> frozen_state_rates;

  [[nodiscard]] model::LinearSystem to_linear_system(const std::string& description,
                                                     const std::string& citation) const;
};

// Throws std::invalid_argument on a malformed request — a name list that does
// not match its matrix, a wind offset that does not admit three columns, an
// empty output list — and std::runtime_error when the supplied point is not an
// equilibrium within the stated budget.
[[nodiscard]] ExtendedLinearisation linearize_extended(const ExtendedDynamics& dynamics,
                                                       const Eigen::VectorXd& extended_state,
                                                       const Eigen::VectorXd& input,
                                                       const ExtendedLinearisationOptions& options);

}  // namespace galata::linearize

#endif  // GALATA_LINEARIZE_EXTENDED_HPP
