// SPDX-License-Identifier: Apache-2.0
//
// Validation of the multirotor equilibrium and of the linearisation taken about
// it: RFC-0002's WP2 acceptance criteria, one test each.
//
// WHAT ANCHORS THESE CASES. No published quadrotor reference does, for the
// reason RFC-0002's acceptance section records and `test_quadrotor_plant.cpp`
// repeats. Every gate below is a CLOSED FORM derived from the model's own
// parameters — a drag coefficient over a mass, a reciprocal time constant, a
// thrust slope — compared against a number the finite-difference machinery
// produced without being told the answer. That is the same anchor the
// torque-free case uses: mathematics rather than a document. It bounds the
// equations and the differentiation of them, never the parameter set, and a
// completed run of this model remains evidence about equations and never about
// an aircraft.
//
// EVERY BUDGET IS STATED BEFORE THE NUMBER IT GATES, and each is derived from a
// mechanism — the central-difference truncation error at the step actually
// used, floating-point round-off, or the quadratic-drag scale the model itself
// publishes. None is drawn from the value galata happens to produce. Where a
// gate would otherwise be set just above a measured agreement, the measurement
// is reported alongside instead and the gate stays on the mechanism.

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/sha256.hpp"
#include "galata/core/state.hpp"
#include "galata/linearize/extended.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/trim/hover.hpp"
#include "galata/units.hpp"

#include "validation_config.hpp"
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string measured(double value) {
  std::ostringstream text;
  text << std::scientific << std::setprecision(3) << value;
  return text.str();
}

// The fixture is not committed (ADR-0007), so a path is not an identification:
// two runs citing the same path can have read different bytes. The digest is
// the only durable statement about WHICH fixture a retained run consumed, and
// it is recorded as a property so an auditor reads it off the run rather than
// off somebody's shell history.
std::string fixture_digest(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return "unreadable";
  }
  std::ostringstream bytes;
  bytes << in.rdbuf();
  return galata::core::sha256(bytes.str());
}

using galata::core::kStandardGravity;
using galata::linearize::ExtendedLinearisation;
using galata::linearize::ExtendedLinearisationOptions;
using galata::linearize::OutputKind;
using galata::model::Quadrotor;
using galata::trim::HoverTrim;
using galata::trim::HoverTrimRequest;

Quadrotor shipped_model() {
  return galata::model::load_quadrotor(std::string(GALATA_MODELS_DIR)
                                       + "/souxmar-quad/souxmar-quad.yaml");
}

// The four rotor commands followed by three NED wind columns, which is the
// input vector `linearize.extended` builds and the one every gate here reads.
struct HoverSetup {
  Quadrotor model;
  HoverTrim trim;
  ExtendedLinearisation linearisation;
  Eigen::VectorXd input;
};

HoverSetup hover_setup(const Eigen::Vector3d& wind_ned_m_s = Eigen::Vector3d::Zero(),
                       const Eigen::Vector3d& ground_velocity_ned_m_s = Eigen::Vector3d::Zero(),
                       double altitude_m = 0.0) {
  HoverSetup setup{shipped_model(), {}, {}, {}};

  HoverTrimRequest request;
  request.altitude_m = altitude_m;
  request.wind_ned_m_s = wind_ned_m_s;
  request.ground_velocity_ned_m_s = ground_velocity_ned_m_s;
  setup.trim = galata::trim::trim_hover(setup.model, request);

  const int rotors = setup.model.rotor_count();
  setup.input = Eigen::VectorXd::Zero(rotors + 3);
  setup.input.head(rotors) = setup.trim.command_rad_s;
  setup.input.segment<3>(rotors) = wind_ned_m_s;

  ExtendedLinearisationOptions options;
  options.appended_state_names = {
      "rotor_0_rad_s", "rotor_1_rad_s", "rotor_2_rad_s", "rotor_3_rad_s"};
  options.input_names = {"command_0_rad_s",
                         "command_1_rad_s",
                         "command_2_rad_s",
                         "command_3_rad_s",
                         "wind_north_m_s",
                         "wind_east_m_s",
                         "wind_down_m_s"};
  options.wind_input_offset = rotors;
  options.outputs = {OutputKind::BodySpecificForce,
                     OutputKind::BodyRates,
                     OutputKind::PositionNed,
                     OutputKind::Altitude,
                     OutputKind::GroundVelocityNed};

  const Quadrotor& model = setup.model;
  const galata::linearize::ExtendedDynamics dynamics = [&model, rotors](const Eigen::VectorXd& x,
                                                                        const Eigen::VectorXd& u) {
    return model.derivative(x, u.head(rotors), Eigen::Vector3d(u.segment<3>(rotors)));
  };
  setup.linearisation = galata::linearize::linearize_extended(
      dynamics, setup.trim.extended_state, setup.input, options);
  return setup;
}

std::vector<std::complex<double>> eigenvalues_of(const Eigen::MatrixXd& a) {
  const Eigen::EigenSolver<Eigen::MatrixXd> solver(a);
  std::vector<std::complex<double>> values(
      solver.eigenvalues().data(), solver.eigenvalues().data() + solver.eigenvalues().size());
  // Ordered by real part, then imaginary, so the assertions below read against a
  // fixed sequence rather than against whatever order LAPACK returned. ADR-0004:
  // an ordering that depends on the library is an ordering that can change under
  // the test's feet.
  std::sort(values.begin(),
            values.end(),
            [](const std::complex<double>& left, const std::complex<double>& right) {
              if (left.real() != right.real()) {
                return left.real() < right.real();
              }
              return left.imag() < right.imag();
            });
  return values;
}

}  // namespace

// --- The equilibrium itself ------------------------------------------------
//
// BUDGET. `trim_hover` runs a fixed forty Newton iterations and then compares
// the residual once against the requested budget, so the gate here is the
// request's own: 1e-10 on an acceleration norm in m/s^2 and rad/s^2. That is
// far above the round-off floor of a residual assembled from quantities of
// order 10 — about 1e-15 — and far below any modelling error, which is what a
// convergence gate is for. Each condition's actual residual is reported.
//
// The rotor-speed identity is a closed form: four equal rotors carrying the
// weight sit at sqrt(m g / (4 k_T)), evaluated in a handful of operations on
// quantities of order 100, so 1e-9 relative is three orders above round-off.
TEST(QuadrotorHoverTrim, StillAirCrosswindCruiseAndUnequalRotorsSolveToTheirDeclaredBudget) {
  const Quadrotor model = shipped_model();
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);

  // Still air, and a DECLARED mass and altitude. The mass is the model's; the
  // altitude is the request's, and the trim must put it in the state rather
  // than reporting a vehicle at sea level whatever was asked for.
  HoverTrimRequest still;
  still.altitude_m = 120.0;
  const HoverTrim hover = galata::trim::trim_hover(model, still);

  EXPECT_LE(hover.residual_norm, hover.residual_tolerance);
  EXPECT_EQ(hover.newton_iterations, still.iterations)
      << "the iteration count must be fixed, not a count of what convergence needed";
  EXPECT_NEAR(hover.extended_state(galata::core::kPositionDown), -still.altitude_m, 1e-12)
      << "the returned position does not carry the declared altitude";
  EXPECT_NEAR(hover.altitude_m, still.altitude_m, 1e-12);
  EXPECT_NEAR(hover.roll_rad, 0.0, 1e-10);
  EXPECT_NEAR(hover.pitch_rad, 0.0, 1e-10);
  EXPECT_NEAR(hover.airspeed_m_s, 0.0, 1e-10);
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    EXPECT_NEAR(hover.command_rad_s(rotor) / hover_speed, 1.0, 1e-9)
        << "rotor " << rotor << " does not sit at the closed-form hover speed";
  }
  // Every rotor's margin to its own ceiling, and the smallest of them, are
  // reported by the trim rather than recomputed by a caller.
  ASSERT_EQ(static_cast<int>(hover.rotor_margin_fraction.size()), model.rotor_count());
  for (const double margin : hover.rotor_margin_fraction) {
    EXPECT_GE(margin, 0.0);
    EXPECT_NEAR(margin, hover.smallest_rotor_margin_fraction, 1e-12)
        << "four equal rotors must have equal margins";
  }

  // Crosswind hover: the vehicle holds station over the ground, so it must tilt
  // into the wind and its airspeed is the wind speed. Six metres per second is
  // above the 4.8 m/s scale at which this model's quadratic drag matches its
  // linear drag, which is fine for a TRIM — the nonlinear plant is solved, not
  // linearised — and is the reason the linear-versus-nonlinear case below stays
  // far beneath it.
  //
  // THE SIGN, worked out rather than guessed, because it is the half of this
  // case that a transposed rotation would pass. `wind_ned_m_s` is the velocity
  // of the AIR MASS, so (6, 0, 0) is air moving north and the vehicle is being
  // blown north. Its air-relative velocity while holding station is 6 m/s
  // SOUTH, so it is flying backwards through the air and the thrust must have a
  // southward component to balance the drag pushing it north. Thrust acts along
  // body -z, which in NED is (-T sin(pitch), 0, -T cos(pitch)) at zero roll, so
  // a southward thrust component needs sin(pitch) > 0: the vehicle leans away
  // from the wind's destination, nose UP.
  HoverTrimRequest crosswind;
  crosswind.wind_ned_m_s = Eigen::Vector3d(6.0, 0.0, 0.0);
  const HoverTrim blown = galata::trim::trim_hover(model, crosswind);
  EXPECT_LE(blown.residual_norm, blown.residual_tolerance);
  EXPECT_NEAR(blown.airspeed_m_s, 6.0, 1e-9)
      << "holding station in a 6 m/s wind is a 6 m/s airspeed";
  EXPECT_GT(blown.pitch_rad, 1e-3)
      << "air moving north blows the vehicle north, so it must lean south — nose up — to "
         "hold station";
  EXPECT_NEAR(blown.roll_rad, 0.0, 1e-9) << "a purely northerly wind must not roll the vehicle";

  // Cruise as a relative equilibrium: still air, nonzero ground velocity. The
  // position rate is not zero and is not required to be.
  HoverTrimRequest cruise;
  cruise.ground_velocity_ned_m_s = Eigen::Vector3d(6.0, 0.0, 0.0);
  const HoverTrim moving = galata::trim::trim_hover(model, cruise);
  EXPECT_LE(moving.residual_norm, moving.residual_tolerance);
  EXPECT_NEAR(moving.airspeed_m_s, 6.0, 1e-9);
  EXPECT_LT(moving.pitch_rad, -1e-3) << "flying north must pitch the nose down";
  EXPECT_NEAR(moving.pitch_rad, -blown.pitch_rad, 1e-9)
      << "flying north at 6 m/s through still air and holding station while the air moves "
         "north at 6 m/s are the same air-relative condition with the tilt reversed";

  // Unequal rotor speeds. A yawing equilibrium needs no yaw rate: the reaction
  // torques must simply cancel, and they do so at equal speeds only because the
  // shipped rotors share one torque coefficient. Give one pair a stronger
  // coefficient and the trim must split the speeds to cancel the yaw moment.
  Quadrotor asymmetric = shipped_model();
  asymmetric.rotors[0].torque_coefficient_n_m_s2 *= 1.20;
  asymmetric.rotors[2].torque_coefficient_n_m_s2 *= 1.20;
  const HoverTrim split = galata::trim::trim_hover(asymmetric, HoverTrimRequest{});
  EXPECT_LE(split.residual_norm, split.residual_tolerance);
  const double spread = split.command_rad_s.maxCoeff() - split.command_rad_s.minCoeff();
  EXPECT_GT(spread, 1.0) << "a yaw-torque imbalance must be trimmed by unequal rotor speeds";
  // The clockwise pair now makes more reaction torque per unit speed, so it must
  // run slower for the four reactions to cancel.
  EXPECT_LT(split.command_rad_s(0), split.command_rad_s(1));
  EXPECT_LT(split.command_rad_s(2), split.command_rad_s(3));

  RecordProperty("hover_residual", measured(hover.residual_norm));
  RecordProperty("crosswind_residual", measured(blown.residual_norm));
  RecordProperty("cruise_residual", measured(moving.residual_norm));
  RecordProperty("unequal_rotor_residual", measured(split.residual_norm));
  RecordProperty("hover_rotor_margin_fraction", measured(hover.smallest_rotor_margin_fraction));
  RecordProperty("crosswind_pitch_rad", measured(blown.pitch_rad));
  RecordProperty("unequal_rotor_spread_rad_s", measured(spread));
}

// --- The pole structure ----------------------------------------------------
//
// WHAT IS BEING CHECKED, and why each part of it is a real gate.
//
// Six eigenvalues at the origin: three because nothing in the dynamics reads
// position, and three more because nothing reads attitude at a level hover in
// still air. They are not a numerical accident — a chart that mishandled the
// attitude coordinate produces three of them, not six.
//
// Three translational decay rates at -c_k/m and three rotational ones at
// -k_k/I_kk, from the LINEAR drag alone. The quadratic term contributes exactly
// nothing to them: its derivative 2 c |v| vanishes at zero airspeed, which is
// precisely why the linear-versus-nonlinear case below has to name the airspeed
// above which the linear model stops meaning anything.
//
// Four rotor poles at -1/tau, exact rather than approximate because the rotor
// rows depend on no other state: the lag's target is the command, and the
// command is an input.
//
// THE BUDGET IS DERIVED FROM THE FINITE-DIFFERENCE MECHANISM, and the three
// groups do not get the same one, because they do not have the same error.
//
// The translational entries are the interesting case, and they are WORSE than
// second order. `numerics/jacobian.hpp` warns that a central difference is not
// valid across a kink inside the perturbation window and that the Richardson
// estimate cannot see one. The quadratic drag term c_q v|v| is exactly such a
// kink: it is continuous and once differentiable at v = 0 and not twice, and at
// hover the perturbation window straddles it. Working the difference quotient
// out by hand on -(c_l v + c_q v|v|) gives -c_l - c_q h exactly, so the relative
// error is h / (c_l / c_q) — the step divided by the very quadratic-drag
// airspeed this model publishes — and it is FIRST order in h, not second. The
// step is read from the linearisation's own `state_steps` rather than assumed,
// and halved because the reported Jacobian is the half-step one.
//
// That predicts about 6.3e-7 relative on the body x and y rates. The gate is
// set at four times the prediction: the prediction is exact for the drag entry
// in isolation, while what is compared here is an eigenvalue of the assembled
// sixteen-by-sixteen matrix, and the factor covers the eigen solve without
// leaving room for a structural error, which moves these by tens of percent.
//
// The rotational and rotor entries have no such term — both are exactly linear
// in their own coordinate — so their only error is round-off in the difference
// quotient, and they are held to 1e-9 relative, which is far tighter than the
// translational gate and would fail immediately if a quadratic term appeared
// there.
//
// The zeros are gated at 1e-9 absolute, round-off for a matrix of this size.
TEST(QuadrotorHoverLinearisation, PoleStructureIsSixIntegratorsThreeDragPairsAndFourRotorLags) {
  const HoverSetup setup = hover_setup();
  const Quadrotor& model = setup.model;

  ASSERT_EQ(setup.linearisation.a.rows(), 16) << "twelve chart coordinates and four rotor states";
  ASSERT_EQ(setup.linearisation.a.cols(), 16);

  // The closed forms, computed from the model's parameters and not from the
  // matrix, so the comparison has something independent to be a comparison with.
  const double mass_kg = model.mass.mass_kg;
  const std::vector<double> translational = {-model.drag_linear_n_s_m.x() / mass_kg,
                                             -model.drag_linear_n_s_m.y() / mass_kg,
                                             -model.drag_linear_n_s_m.z() / mass_kg};
  const std::vector<double> rotational = {
      -model.angular_drag_n_m_s.x() / model.mass.inertia_cg_body_kg_m2(0, 0),
      -model.angular_drag_n_m_s.y() / model.mass.inertia_cg_body_kg_m2(1, 1),
      -model.angular_drag_n_m_s.z() / model.mass.inertia_cg_body_kg_m2(2, 2)};
  const double rotor_pole = -1.0 / model.rotors.front().speed_time_constant_s;

  // The kink bound, declared before the comparison. The reported Jacobian is
  // the half-step one, so the step that sets the error is half of what the
  // options asked for.
  const double half_step_m_s =
      0.5 * setup.linearisation.state_steps(galata::linearize::kChartVelocityU);
  ASSERT_GT(half_step_m_s, 0.0);
  std::vector<double> translational_budget;
  for (int axis = 0; axis < 3; ++axis) {
    const double quadratic_limit_m_s =
        model.drag_linear_n_s_m(axis) / model.drag_quadratic_n_s2_m2(axis);
    translational_budget.push_back(4.0 * half_step_m_s / quadratic_limit_m_s);
  }
  const double linear_coordinate_budget = 1e-9;
  const double integrator_budget = 1e-9;

  std::vector<double> expected = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  expected.insert(expected.end(), translational.begin(), translational.end());
  expected.insert(expected.end(), rotational.begin(), rotational.end());
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    expected.push_back(rotor_pole);
  }
  std::sort(expected.begin(), expected.end());

  // The budget for each expected pole, in the same order, so a sorted
  // comparison keeps each gate with the pole it was derived for.
  std::vector<double> budget;
  for (const double pole : expected) {
    if (pole == 0.0) {
      budget.push_back(integrator_budget);
      continue;
    }
    double worst_for_pole = linear_coordinate_budget;
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(pole - translational[static_cast<std::size_t>(axis)]) < 1e-12) {
        worst_for_pole = translational_budget[static_cast<std::size_t>(axis)];
      }
    }
    budget.push_back(worst_for_pole);
  }

  const std::vector<std::complex<double>> poles = eigenvalues_of(setup.linearisation.a);
  ASSERT_EQ(static_cast<int>(poles.size()), 16);

  double worst_zero = 0.0;
  double worst_translational = 0.0;
  double worst_linear_coordinate = 0.0;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(poles[i].imag(), 0.0, 1e-9)
        << "pole " << i << " is complex; every mode of a level hover of this plant is real";
    if (expected[i] == 0.0) {
      worst_zero = std::fmax(worst_zero, std::abs(poles[i].real()));
      EXPECT_LT(std::abs(poles[i].real()), budget[i])
          << "pole " << i << " should be an integrator from position or attitude";
      continue;
    }
    const double relative = std::abs(poles[i].real() - expected[i]) / std::abs(expected[i]);
    if (budget[i] == linear_coordinate_budget) {
      worst_linear_coordinate = std::fmax(worst_linear_coordinate, relative);
    } else {
      worst_translational = std::fmax(worst_translational, relative);
    }
    EXPECT_LT(relative, budget[i])
        << "pole " << i << " is " << poles[i].real() << " where the closed form is " << expected[i];
  }

  RecordProperty("integrator_poles", "6");
  RecordProperty("worst_integrator_magnitude", measured(worst_zero));
  RecordProperty("worst_translational_relative_error", measured(worst_translational));
  RecordProperty("translational_kink_budget", measured(translational_budget.front()));
  RecordProperty("worst_rotational_and_rotor_relative_error", measured(worst_linear_coordinate));
  RecordProperty("worst_relative_truncation",
                 measured(setup.linearisation.worst_relative_truncation));
}

// --- The collective vertical gain ------------------------------------------
//
// THE CLOSED FORM, and where it lives. Total thrust is n k_T omega^2, so raising
// every rotor together has slope n * 2 k_T omega_h = 8 k_T omega_h at the hover
// speed, and dividing by the mass gives an acceleration slope of
// 8 k_T omega_h / m. RFC-0002 writes it as 4 * 2 * k_T * omega_h / m.
//
// THE SIGN IS PART OF THE GATE, not a detail. Thrust acts along body -z; body z
// is down at a level hover and so is the NED down axis. A faster rotor
// therefore accelerates the vehicle in NEGATIVE NED down, which is upward. A
// model that had this backwards would hover, trim and produce a plausible pole
// structure — every case above would pass — and would climb when commanded to
// descend.
//
// WHERE IT IS NOT. It is not in B. The rotor wrench reads the rotor SPEED
// state, and the command reaches that state only through the lag, so the
// command columns of B touch the velocity rows not at all; that is asserted
// here too, because finding this entry in the wrong matrix is the likely
// mistake.
//
// BUDGET. 1e-6 relative, the same central-difference argument as the case
// above, against a closed form evaluated in three operations.
TEST(QuadrotorHoverLinearisation, CollectiveVerticalGainMatchesTheClosedFormAndActsUpward) {
  const HoverSetup setup = hover_setup();
  const Quadrotor& model = setup.model;
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);

  const double expected = -static_cast<double>(model.rotor_count()) * 2.0
                          * model.rotors.front().thrust_coefficient_n_s2 * hover_speed
                          / model.mass.mass_kg;
  ASSERT_LT(expected, 0.0) << "the closed form itself must be negative in NED down";

  // Sum over the four rotor-speed columns of the body-w row: raising every
  // rotor together by one rad/s.
  double collective_gain = 0.0;
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    collective_gain += setup.linearisation.a(galata::linearize::kChartVelocityW,
                                             galata::linearize::kRigidChartSize + rotor);
  }
  EXPECT_LT(collective_gain, 0.0)
      << "more rotor speed must accelerate the vehicle UP, which is negative NED down";
  const double relative = std::abs(collective_gain - expected) / std::abs(expected);
  EXPECT_LT(relative, 1e-6) << "collective vertical gain " << collective_gain
                            << " against the closed form " << expected;

  // The same slope must appear in the specific-force z output, because at hover
  // the only thing separating the two is gravity, which no rotor speed moves.
  const std::vector<std::string>& outputs = setup.linearisation.output_names;
  const auto found = std::find(outputs.begin(), outputs.end(), "specific_force_z_m_s2");
  ASSERT_NE(found, outputs.end());
  const Eigen::Index row = std::distance(outputs.begin(), found);
  double observed_gain = 0.0;
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    observed_gain += setup.linearisation.c(row, galata::linearize::kRigidChartSize + rotor);
  }
  EXPECT_LT(std::abs(observed_gain - expected) / std::abs(expected), 1e-6)
      << "the accelerometer row disagrees with the state row about the same slope";

  // Not in B: the command reaches the airframe only through the rotor lag.
  for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
    EXPECT_NEAR(setup.linearisation.b(galata::linearize::kChartVelocityW, rotor), 0.0, 1e-9)
        << "a rotor COMMAND has no instantaneous authority over acceleration";
    EXPECT_NEAR(setup.linearisation.b(galata::linearize::kRigidChartSize + rotor, rotor),
                1.0 / model.rotors.front().speed_time_constant_s,
                1e-6)
        << "the command's authority is over the rotor state, at 1/tau";
  }

  RecordProperty("collective_vertical_gain_m_s2_per_rad_s", measured(collective_gain));
  RecordProperty("collective_vertical_gain_closed_form", measured(expected));
  RecordProperty("collective_vertical_gain_relative_error", measured(relative));
}

// --- The wind columns ------------------------------------------------------
//
// WHAT THIS GATES. RFC-0002 requires the wind to appear as separately named
// columns of B and D, and requires D to carry "the wind-to-specific-force drag
// feedthrough". Both halves are checked, and so is the thing that makes them
// possible: the wind perturbation is taken at fixed GROUND velocity, which the
// linearisation's header derives and which is the only reading under which a
// gust does anything at all to a plant whose velocity state is air-relative.
//
// THE CLOSED FORM. At a level hover the body axes are the NED axes, so a wind
// of one metre per second north is an air-relative velocity of one metre per
// second aft. The linear drag then puts c_x / m of acceleration on the body x
// axis, in the same direction as the wind. The quadratic term contributes
// nothing, for the same reason it contributed nothing to the poles: its
// derivative vanishes at zero airspeed.
//
// THE ZEROS ARE HALF THE CASE. Holding the ground velocity fixed is a claim
// with a consequence: the ground-velocity observation must not move, and
// neither must the position rate. If those columns were nonzero the wind would
// be counted twice, once through the disturbance and once through the state it
// displaced. They are gated at 1e-9, which is round-off for a difference
// quotient of quantities of order one.
//
// BUDGET. 1e-6 relative on the feedthrough, the central-difference argument
// again.
TEST(QuadrotorHoverLinearisation, WindColumnsCarryTheDragFeedthroughAtFixedGroundVelocity) {
  const HoverSetup setup = hover_setup();
  const Quadrotor& model = setup.model;
  const int wind = model.rotor_count();

  const std::vector<std::string>& inputs = setup.linearisation.input_names;
  ASSERT_EQ(inputs[static_cast<std::size_t>(wind)], "wind_north_m_s");
  ASSERT_EQ(inputs[static_cast<std::size_t>(wind) + 1], "wind_east_m_s");
  ASSERT_EQ(inputs[static_cast<std::size_t>(wind) + 2], "wind_down_m_s");

  const std::vector<std::string>& outputs = setup.linearisation.output_names;
  const auto row_of = [&](const std::string& name) {
    const auto found = std::find(outputs.begin(), outputs.end(), name);
    EXPECT_NE(found, outputs.end()) << name << " is not among the declared outputs";
    return std::distance(outputs.begin(), found);
  };

  const double mass_kg = model.mass.mass_kg;
  const std::vector<double> expected = {model.drag_linear_n_s_m.x() / mass_kg,
                                        model.drag_linear_n_s_m.y() / mass_kg,
                                        model.drag_linear_n_s_m.z() / mass_kg};
  const std::vector<std::string> force_rows = {
      "specific_force_x_m_s2", "specific_force_y_m_s2", "specific_force_z_m_s2"};
  const std::vector<int> velocity_rows = {galata::linearize::kChartVelocityU,
                                          galata::linearize::kChartVelocityV,
                                          galata::linearize::kChartVelocityW};

  double worst_relative = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double b_entry =
        setup.linearisation.b(velocity_rows[static_cast<std::size_t>(axis)], wind + axis);
    const double d_entry =
        setup.linearisation.d(row_of(force_rows[static_cast<std::size_t>(axis)]), wind + axis);
    const double target = expected[static_cast<std::size_t>(axis)];

    ASSERT_GT(std::abs(d_entry), 0.0)
        << "axis " << axis
        << ": D is exactly zero, which is what perturbing the wind at fixed AIR-RELATIVE "
           "velocity produces. The gust is doing nothing to the airframe.";
    for (const double entry : {b_entry, d_entry}) {
      const double relative = std::abs(entry - target) / std::abs(target);
      worst_relative = std::fmax(worst_relative, relative);
      EXPECT_LT(relative, 1e-6) << "axis " << axis << ": wind feedthrough " << entry
                                << " against the closed form " << target;
    }
  }

  // The two things that must NOT move, because the ground velocity was held.
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(setup.linearisation.b(galata::linearize::kChartPositionNorth + axis, wind + axis),
                0.0,
                1e-9)
        << "a wind change must not move the position rate; the ground velocity is continuous";
  }
  const Eigen::Index ground_north = row_of("ground_velocity_north_m_s");
  for (int axis = 0; axis < 3; ++axis) {
    for (int column = 0; column < 3; ++column) {
      EXPECT_NEAR(setup.linearisation.d(ground_north + axis, wind + column), 0.0, 1e-9)
          << "the ground-velocity observation must be blind to a perturbation that holds it "
             "fixed";
    }
  }

  // Truncation estimates exist for all four matrices, which is what lets a
  // reader qualify a D entry rather than take it on faith.
  EXPECT_EQ(setup.linearisation.a_truncation.rows(), setup.linearisation.a.rows());
  EXPECT_EQ(setup.linearisation.b_truncation.rows(), setup.linearisation.b.rows());
  EXPECT_EQ(setup.linearisation.c_truncation.rows(), setup.linearisation.c.rows());
  EXPECT_EQ(setup.linearisation.d_truncation.rows(), setup.linearisation.d.rows());

  RecordProperty("worst_wind_feedthrough_relative_error", measured(worst_relative));
  RecordProperty("worst_relative_truncation_c",
                 measured(setup.linearisation.worst_relative_truncation_c));
  RecordProperty("worst_relative_truncation_d",
                 measured(setup.linearisation.worst_relative_truncation_d));
}

// --- Linear against nonlinear ----------------------------------------------
//
// EVERYTHING BELOW IS DECLARED BEFORE ANY COMPARISON IS MADE, which is the
// point of the case: a bound chosen after seeing the disagreement is a
// regression lock wearing a budget's name.
//
// THE PERTURBATIONS AND THE HORIZON. One metre per second on each body velocity
// axis, one degree on each attitude axis, two degrees per second on each body
// rate, over one second. The horizon is chosen SHORT against the slowest mode
// rather than long: the slowest translational decay has a 1/e time of
// m / c_lin = 1.6 / 0.12 = 13 s, so over one second the states are still moving
// and the comparison is made mid-response. A horizon long enough for everything
// to settle would pass for any model whose eigenvalues are in the left half
// plane, which is not a test of the Jacobian.
//
// THE QUADRATIC-DRAG LIMIT, named as RFC-0002 requires. The model publishes a
// per-axis linear and a quadratic drag coefficient, and their ratio is the
// airspeed at which the two contribute equally: 0.12/0.025 = 4.8 m/s on body x
// and y, 0.18/0.035 = 5.14 m/s on body z. A linearisation taken at hover cannot
// see the quadratic term at all — its derivative 2 c |v| vanishes at zero
// airspeed — so the linear model UNDER-PREDICTS drag, by a fraction that grows
// linearly with the airspeed reached. That is why the perturbation sits at
// 1 m/s, a fifth of the limit, and the case asserts that ordering before it
// compares anything.
//
// THE BUDGET, DERIVED FROM THE THREE NEGLECTED SECOND-ORDER TERMS, computed
// below from the model's own parameters rather than written here as a figure.
// The first attempt at this case named only the quadratic drag and gated on it,
// and the vertical channel failed at 42 percent — correctly, because quadratic
// drag is NOT the largest thing a hover linearisation drops. The three are:
//
//   1. Quadratic drag. Neglected force c_q v|v| against retained c_l v, so a
//      fraction v/(c_l/c_q) of a drag term that itself moves the velocity by
//      (c_l/m) T v over the horizon.
//   2. Gravity and thrust projection. Both are exact in the attitude, and the
//      linearisation keeps only the first-order tilt: the neglected part is
//      g (1 - cos theta) ~= g theta^2 / 2, acting over the horizon, where the
//      attitude excursion is theta_0 + omega_0 T.
//   3. The Coriolis term omega x v, which is a product of two perturbed
//      quantities and therefore entirely second order.
//
// Their sum is the bound, doubled once and for a stated reason: each is an
// order-of-magnitude estimate taken at its worst point over the horizon rather
// than integrated along the response, and cross terms between the three are not
// counted. A factor of two covers that. It does not cover a structural error —
// a chart that mishandled the attitude derivative, or a rotation applied
// transposed, moves these channels by tens of percent, not by single figures.
//
// The velocity bound is the largest of the three coordinate groups and is
// applied to all of them. The rate channels drop the gyroscopic omega x I omega,
// about |omega|^2 (dI/I) T, and the attitude channels drop the chart's own
// second-order term, about theta omega T / 2; both are near two percent at these
// amplitudes, comfortably inside the velocity bound.
//
// EACH CHANNEL IS NORMALISED BY ITS COORDINATE GROUP'S PERTURBATION SCALE, not
// by its own peak excursion. A channel whose peak response is small — the
// vertical velocity at hover is driven only by drag decay — would otherwise be
// held to a bound thousands of times tighter than the mechanism that limits it,
// which is how the first attempt at this case failed for the wrong reason.
//
// The measured agreement is reported and is far tighter than the gate. That
// difference is deliberately NOT folded in: a budget drawn from an observed
// value is a regression lock, and charter rule 8 requires a lock to be
// labelled as one.
TEST(QuadrotorHoverLinearisation, LinearAndNonlinearAgreeWithinTheSecondOrderBoundOverOneSecond) {
  const HoverSetup setup = hover_setup();
  const Quadrotor& model = setup.model;

  // --- declared before the comparison ---
  const double horizon_s = 1.0;
  const double step_s = 0.001;
  const double velocity_perturbation_m_s = 1.0;
  const double attitude_perturbation_rad = galata::units::degrees_to_radians(1.0);
  const double rate_perturbation_rad_s = galata::units::degrees_to_radians(2.0);

  const double quadratic_limit_x_m_s =
      model.drag_linear_n_s_m.x() / model.drag_quadratic_n_s2_m2.x();
  const double quadratic_limit_z_m_s =
      model.drag_linear_n_s_m.z() / model.drag_quadratic_n_s2_m2.z();
  ASSERT_LT(velocity_perturbation_m_s, quadratic_limit_x_m_s)
      << "the perturbation must sit below the airspeed at which the neglected quadratic drag "
         "matches the retained linear drag";
  ASSERT_LT(velocity_perturbation_m_s, quadratic_limit_z_m_s);

  // The attitude the vehicle reaches over the horizon, which is what the
  // second-order gravity term is quadratic in.
  const double attitude_excursion_rad =
      attitude_perturbation_rad + rate_perturbation_rad_s * horizon_s;

  const double drag_rate_per_s = model.drag_linear_n_s_m.maxCoeff() / model.mass.mass_kg;
  const double quadratic_term_m_s = drag_rate_per_s * horizon_s * velocity_perturbation_m_s
                                    * velocity_perturbation_m_s
                                    / std::fmin(quadratic_limit_x_m_s, quadratic_limit_z_m_s);
  const double gravity_term_m_s =
      0.5 * kStandardGravity * attitude_excursion_rad * attitude_excursion_rad * horizon_s;
  const double coriolis_term_m_s = rate_perturbation_rad_s * velocity_perturbation_m_s * horizon_s;
  const double budget_fraction =
      2.0 * (quadratic_term_m_s + gravity_term_m_s + coriolis_term_m_s) / velocity_perturbation_m_s;

  ASSERT_LT(budget_fraction, 0.5)
      << "a bound above half the perturbation is not a test of anything; shrink the "
         "perturbation or the horizon";

  const int chart = static_cast<int>(setup.linearisation.a.rows());

  // The chart perturbation, and the nonlinear state it corresponds to. Attitude
  // is applied through the chart's own exponential map so the two runs start at
  // the same point rather than at nearby ones.
  Eigen::VectorXd delta = Eigen::VectorXd::Zero(chart);
  delta.segment<3>(galata::linearize::kChartVelocityU).setConstant(velocity_perturbation_m_s);
  delta.segment<3>(galata::linearize::kChartAttitudeErrorX).setConstant(attitude_perturbation_rad);
  delta.segment<3>(galata::linearize::kChartRateP).setConstant(rate_perturbation_rad_s);

  const galata::core::Quaternion nominal_attitude(
      setup.trim.extended_state(galata::core::kQuaternionW),
      setup.trim.extended_state(galata::core::kQuaternionX),
      setup.trim.extended_state(galata::core::kQuaternionY),
      setup.trim.extended_state(galata::core::kQuaternionZ));

  Eigen::VectorXd nonlinear_start = setup.trim.extended_state;
  nonlinear_start.segment<3>(galata::core::kVelocityU) +=
      delta.segment<3>(galata::linearize::kChartVelocityU);
  nonlinear_start.segment<3>(galata::core::kRateP) +=
      delta.segment<3>(galata::linearize::kChartRateP);
  {
    const Eigen::Vector3d error = delta.segment<3>(galata::linearize::kChartAttitudeErrorX);
    const double angle = error.norm();
    const Eigen::Vector3d axis = error / angle;
    const double half = 0.5 * angle;
    const galata::core::Quaternion increment(std::cos(half),
                                             axis.x() * std::sin(half),
                                             axis.y() * std::sin(half),
                                             axis.z() * std::sin(half));
    const galata::core::Quaternion perturbed = (nominal_attitude * increment).normalized();
    nonlinear_start(galata::core::kQuaternionW) = perturbed.w();
    nonlinear_start(galata::core::kQuaternionX) = perturbed.x();
    nonlinear_start(galata::core::kQuaternionY) = perturbed.y();
    nonlinear_start(galata::core::kQuaternionZ) = perturbed.z();
  }

  const Eigen::VectorXd command = setup.trim.command_rad_s;
  const auto nonlinear_derivative = [&](double, const Eigen::VectorXd& x) {
    return model.derivative(x, command, setup.trim.wind_ned_m_s);
  };
  const auto projection = [&](Eigen::VectorXd& x) { model.project(x); };

  const int steps = static_cast<int>(std::llround(horizon_s / step_s));
  const auto nonlinear = galata::numerics::integrate_fixed_step(
      nonlinear_derivative, nonlinear_start, 0.0, step_s, steps, steps, projection);

  // The linear response, integrated with the same scheme at the same step, so
  // the comparison is between two models rather than between two integrators.
  const Eigen::MatrixXd& a = setup.linearisation.a;
  const auto linear_derivative = [&](double, const Eigen::VectorXd& d) {
    return Eigen::VectorXd(a * d);
  };
  const auto linear =
      galata::numerics::integrate_fixed_step(linear_derivative, delta, 0.0, step_s, steps, steps);

  ASSERT_EQ(nonlinear.states.size(), linear.states.size());

  // The nonlinear state read back into the chart the linear model is written in.
  const auto nonlinear_chart = [&](const Eigen::VectorXd& state, int channel) {
    if (channel >= galata::linearize::kChartVelocityU
        && channel <= galata::linearize::kChartVelocityW) {
      const int axis = channel - galata::linearize::kChartVelocityU;
      return state(galata::core::kVelocityU + axis)
             - setup.trim.extended_state(galata::core::kVelocityU + axis);
    }
    if (channel >= galata::linearize::kChartRateP) {
      return state(galata::core::kRateP + channel - galata::linearize::kChartRateP);
    }
    const galata::core::Quaternion actual(state(galata::core::kQuaternionW),
                                          state(galata::core::kQuaternionX),
                                          state(galata::core::kQuaternionY),
                                          state(galata::core::kQuaternionZ));
    const galata::core::Quaternion error = nominal_attitude.conjugate() * actual;
    const Eigen::Vector3d vector(error.x(), error.y(), error.z());
    const double norm = vector.norm();
    const Eigen::Vector3d rotation =
        norm > 0.0 ? Eigen::Vector3d(2.0 * std::atan2(norm, error.w()) * vector / norm)
                   : Eigen::Vector3d::Zero();
    return rotation(channel - galata::linearize::kChartAttitudeErrorX);
  };

  const std::vector<int> compared = {galata::linearize::kChartVelocityU,
                                     galata::linearize::kChartVelocityV,
                                     galata::linearize::kChartVelocityW,
                                     galata::linearize::kChartAttitudeErrorX,
                                     galata::linearize::kChartAttitudeErrorY,
                                     galata::linearize::kChartAttitudeErrorZ,
                                     galata::linearize::kChartRateP,
                                     galata::linearize::kChartRateQ,
                                     galata::linearize::kChartRateR};

  double worst_fraction = 0.0;
  for (const int channel : compared) {
    // The scale is the coordinate group's own perturbation, not the channel's
    // peak. See the header comment on why the peak is the wrong denominator.
    double scale = velocity_perturbation_m_s;
    if (channel >= galata::linearize::kChartAttitudeErrorX
        && channel <= galata::linearize::kChartAttitudeErrorZ) {
      scale = attitude_excursion_rad;
    } else if (channel >= galata::linearize::kChartRateP) {
      scale = rate_perturbation_rad_s;
    }

    double worst = 0.0;
    double moved = 0.0;
    for (std::size_t sample = 0; sample < linear.states.size(); ++sample) {
      const double linear_value = linear.states[sample](channel);
      const double nonlinear_value = nonlinear_chart(nonlinear.states[sample], channel);
      worst = std::fmax(worst, std::abs(linear_value - nonlinear_value));
      moved = std::fmax(moved, std::abs(linear_value));
    }
    ASSERT_GT(moved, 0.0) << "channel " << channel << " never moves, so nothing is compared";

    const double fraction = worst / scale;
    worst_fraction = std::fmax(worst_fraction, fraction);
    EXPECT_LT(fraction, budget_fraction)
        << "channel " << channel << " disagrees by " << (100.0 * fraction)
        << " percent of its perturbation scale, above the " << (100.0 * budget_fraction)
        << " percent the three neglected second-order terms can account for";
  }

  RecordProperty("horizon_s", measured(horizon_s));
  RecordProperty("velocity_perturbation_m_s", measured(velocity_perturbation_m_s));
  RecordProperty("attitude_excursion_rad", measured(attitude_excursion_rad));
  RecordProperty("quadratic_drag_limit_x_m_s", measured(quadratic_limit_x_m_s));
  RecordProperty("quadratic_drag_limit_z_m_s", measured(quadratic_limit_z_m_s));
  RecordProperty("budget_fraction", measured(budget_fraction));
  RecordProperty("worst_disagreement_fraction", measured(worst_fraction));
}

// --- Cross-implementation against the Souxmar export -----------------------
//
// THIS IS A CROSS-CHECK, NOT A VALIDATION, for the reason
// `QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory` states:
// agreement between two implementations of the same equations says nothing about
// an aircraft. The case registry records it as self-consistent. What makes it
// worth having anyway is that the other implementation is INDEPENDENT — a
// separate Python plant, trimmed and linearised by its own code in ENU/FLU and
// transformed into this repository's NED/FRD conventions by an explicit
// similarity — so it is the only check here that could catch a shared mistake in
// galata's own reasoning about the chart.
//
// IT IS ALSO THE CHECK ON THE COORDINATE CONTRACT. RFC-0002's WP2 acceptance
// section requires D to carry a wind-to-specific-force drag feedthrough, which
// is only possible if the wind perturbation holds the GROUND velocity fixed.
// The other implementation names its velocity states `ground_v_*` and reports
// exactly that feedthrough, with a zero D block against its own ground-velocity
// outputs and zero wind columns in its position rows. If galata had taken the
// wind at fixed air-relative velocity instead, every one of those blocks would
// disagree — three of them by being zero where this is not, and one by being
// nonzero where this is zero.
//
// The fixture is not committed: its rights position is unestablished, so ADR-0007
// routes it to a path plus regeneration instructions and this case states why it
// did not run when the path is absent.
//
// THE BUDGET, derived before the comparison and from both implementations'
// finite-difference error rather than from their agreement. Each takes its own
// central differences at its own step, and at hover each therefore carries the
// first-order kink error the case above derives — the quadratic drag term is not
// twice differentiable at zero airspeed. The other implementation's own
// translational entry is 0.075000015625 against an exact 0.075, so its relative
// error is 2.1e-7; galata's is 6.3e-7. Their sum bounds a disagreement that is
// entirely method, and 1e-5 relative is an order above it: tight enough that a
// transposed rotation, a sign error or a wrong wind convention cannot pass, loose
// enough that two independent step choices need not match.
//
// Entries whose reference magnitude is below 1e-6 are compared ABSOLUTELY at
// 1e-6, because a relative test on a structural zero measures nothing. The
// smallest genuinely nonzero entry in the reference is 5.1e-3, so that floor sits
// three orders below any real coupling and cannot hide one.
TEST(QuadrotorHoverLinearisation, MatricesAgreeWithTheIndependentSouxmarExport) {
  const std::string directory = GALATA_SOUXMAR_FIXTURE_DIR;
  if (directory.empty()) {
    GTEST_SKIP() << "no cross-implementation fixture configured: this case compares against an "
                    "external programme's exported state-space model, which is not committed "
                    "because its rights position is unestablished (ADR-0007, RFC-0002). "
                    "Configure with -DGALATA_SOUXMAR_FIXTURE_DIR=<dir containing "
                    "quad_hover_ned_frd.yaml>.";
  }
  const std::string path = directory + "/quad_hover_ned_frd.yaml";
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << "cross-implementation model unavailable at " << path
                 << ". Regenerate it with `python -m apps.galata_bridge --output "
                    "outputs/galata_bridge --galata <galata-cli>`.";
  }

  // Read through the SHIPPED loader, unchanged. That the other programme's file
  // is valid input to `model.linear.statespace` is half of what this case
  // checks; RFC-0002 requires that contract to hold.
  const galata::model::LinearSystem reference = galata::model::load_linear_system(path);

  const HoverSetup setup = hover_setup();
  const galata::model::LinearSystem computed =
      setup.linearisation.to_linear_system("galata", "galata");

  ASSERT_EQ(computed.state_count(), reference.state_count())
      << "the two models do not even agree on how many states a hovering quadrotor has";
  ASSERT_EQ(computed.input_count(), reference.input_count());
  ASSERT_EQ(computed.output_count(), reference.output_count());

  // The orders coincide, which is why an entry-by-entry comparison is legal:
  // position, then velocity, then attitude error, then body rates, then rotors
  // in the same rotor order; four commands then three NED wind columns; specific
  // force, body rates, position, altitude, ground velocity. The NAMES differ —
  // the other programme writes `ground_v_north_m_s` where galata writes
  // `velocity_u_m_s`, which at a level hover is the same axis — and that
  // difference is exactly the coordinate reading this case exists to confirm.
  ASSERT_EQ(computed.state_count(), 16);

  const double relative_budget = 1e-5;
  const double absolute_floor = 1e-6;

  // ONE OUTPUT ROW IS EXCLUDED FROM THE BULK COMPARISON AND HELD SEPARATELY
  // BELOW. It is not a tolerance problem and it is not absorbed into one.
  //
  // The other implementation's tenth output is named `altitude_down_m` and its C
  // row is +1 on the NED down state, so the quantity it reports is the DOWN
  // COORDINATE. galata's `OutputKind::Altitude` is documented as positive up —
  // the negative of the NED down state — which is -1. Both are internally
  // consistent; they are different quantities under similar names, and the other
  // programme's name has `down` in it.
  //
  // galata does not change. Altitude positive up is the ordinary meaning of the
  // word, the observation model's header states it, and ADR-0002's down axis
  // points down. Absorbing a factor of -1 into a numerical budget would be
  // absorbing a SIGN ERROR, which is the one thing a budget must never hide, so
  // charter rule 3 applies: the deviation is localised, published, and held by a
  // two-sided check that fails if the disagreement disappears as well as if it
  // grows. A change on either side is then loud rather than silent.
  const Eigen::Index excluded_output_row = 9;

  double worst_relative = 0.0;
  double worst_absolute = 0.0;
  int compared_entries = 0;

  const auto compare =
      [&](const char* name, const Eigen::MatrixXd& mine, const Eigen::MatrixXd& theirs) {
        const bool is_output_matrix = name[0] == 'C' || name[0] == 'D';
        ASSERT_EQ(mine.rows(), theirs.rows()) << name;
        ASSERT_EQ(mine.cols(), theirs.cols()) << name;
        for (Eigen::Index row = 0; row < mine.rows(); ++row) {
          if (is_output_matrix && row == excluded_output_row) {
            continue;
          }
          for (Eigen::Index column = 0; column < mine.cols(); ++column) {
            const double target = theirs(row, column);
            const double actual = mine(row, column);
            ++compared_entries;
            if (std::abs(target) <= absolute_floor) {
              worst_absolute = std::fmax(worst_absolute, std::abs(actual - target));
              EXPECT_LT(std::abs(actual - target), absolute_floor)
                  << name << "(" << row << ", " << column << "): galata has " << actual
                  << " where the independent implementation has a structural zero";
              continue;
            }
            const double relative = std::abs(actual - target) / std::abs(target);
            worst_relative = std::fmax(worst_relative, relative);
            EXPECT_LT(relative, relative_budget)
                << name << "(" << row << ", " << column << "): galata has " << actual
                << " where the independent implementation has " << target;
          }
        }
      };

  compare("A", computed.a, reference.a);
  compare("B", computed.b, reference.b);
  compare("C", computed.output_matrix(), reference.output_matrix());
  compare("D", computed.feedthrough_matrix(), reference.feedthrough_matrix());

  // The localised deviation, held two-sidedly. Both rows are structural
  // selectors rather than difference quotients of anything, so the relationship
  // is exact and is asserted exactly.
  ASSERT_EQ(computed.output_names[static_cast<std::size_t>(excluded_output_row)], "altitude_m");
  ASSERT_EQ(reference.output_labels()[static_cast<std::size_t>(excluded_output_row)],
            "altitude_down_m")
      << "the excluded row is identified by position; if the reference's output order changed, "
         "this exclusion is now hiding a different channel and must be re-derived";
  const Eigen::MatrixXd mine_c = computed.output_matrix();
  const Eigen::MatrixXd their_c = reference.output_matrix();
  for (Eigen::Index column = 0; column < mine_c.cols(); ++column) {
    EXPECT_DOUBLE_EQ(mine_c(excluded_output_row, column), -their_c(excluded_output_row, column))
        << "altitude row, column " << column
        << ": galata reports altitude positive UP and the reference reports the down "
           "coordinate, so the two rows must be exact negatives. They are not, so the "
           "disagreement is no longer only a sign and this exclusion is no longer justified.";
  }
  // The other side of the lock: if the reference is ever corrected to report
  // altitude positive up, its entry becomes -1, the negation above stops holding
  // and this fails. A future fix is loud.
  EXPECT_DOUBLE_EQ(their_c(excluded_output_row, 2), 1.0)
      << "the reference's altitude channel no longer selects +1 on the down state. If it now "
         "reports altitude positive up, delete this exclusion and compare the row in bulk.";

  RecordProperty("fixture", path);
  RecordProperty("fixture_sha256", fixture_digest(path));
  RecordProperty("altitude_row_disagreement", "sign only; localised, not absorbed");
  RecordProperty("compared_entries", std::to_string(compared_entries));
  RecordProperty("worst_relative_disagreement", measured(worst_relative));
  RecordProperty("worst_absolute_disagreement_on_structural_zeros", measured(worst_absolute));
  RecordProperty("relative_budget", measured(relative_budget));
}
