// SPDX-License-Identifier: Apache-2.0
//
// Validation of the nonlinear multirotor plant, model::Quadrotor.
//
// WHAT ANCHORS THESE CASES. No published quadrotor reference does. Cases 1 to 5
// are exact invariants of the model's own equations — the same kind of anchor
// as TorqueFreeConservation, whose reference is mathematics rather than a
// document — and case 6 is agreement with an independent implementation, which
// is a cross-check and not a validation. RFC-0002's acceptance section records
// that decision and why no published source anchors the parameter set. The
// capability is registered implemented-unvalidated for exactly this reason.
//
// Every budget below is stated before the number it gates, and each is derived
// from a mechanism — floating-point round-off, RK4's convergence order, or the
// integration-scheme difference the fixture's own author quantified — never
// from the value galata happens to produce.

#include "galata/core/constants.hpp"
#include "galata/core/frames.hpp"
#include "galata/core/state.hpp"
#include "galata/model/quadrotor.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/sim/rigid_body.hpp"

#include "validation_config.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

// std::to_string fixes six decimals, which reports every figure below a
// micrometre as zero. These numbers are the point of the case.
std::string measured(double value) {
  std::ostringstream text;
  text << std::scientific << std::setprecision(3) << value;
  return text.str();
}

using galata::core::kStandardGravity;
using galata::core::kStateSize;
using galata::model::Quadrotor;

Quadrotor shipped_model() {
  return galata::model::load_quadrotor(std::string(GALATA_MODELS_DIR)
                                       + "/souxmar-quad/souxmar-quad.yaml");
}

Eigen::VectorXd equal_speeds(const Quadrotor& model, double speed_rad_s) {
  return Eigen::VectorXd::Constant(model.rotor_count(), speed_rad_s);
}

// A level, stationary extended state with every rotor at `speed_rad_s`.
Eigen::VectorXd level_state(const Quadrotor& model, double speed_rad_s) {
  Eigen::VectorXd extended = Eigen::VectorXd::Zero(model.extended_state_size());
  extended(galata::core::kQuaternionW) = 1.0;
  extended.segment(model.rotor_state_offset(), model.rotor_count()).setConstant(speed_rad_s);
  return extended;
}

}  // namespace

// --- Case 1: hover balance -------------------------------------------------
//
// BUDGET. This is a closed-form identity evaluated in about ten floating-point
// operations on quantities of order 10, so the only error present is round-off,
// bounded by a few units in the last place — of order 1e-15 absolute. The gates
// are set at 1e-12, three orders looser, which leaves room for a platform's
// sqrt and multiplication to differ in the last bits without leaving room for a
// modelling error to hide.
TEST(QuadrotorHover, EqualSpeedsCarryTheWeightAndProduceNoMoment) {
  const Quadrotor model = shipped_model();
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);

  // The defining identity: n k_T omega^2 = m g.
  const double thrust_coefficient = model.rotors.front().thrust_coefficient_n_s2;
  const double total_thrust_n =
      static_cast<double>(model.rotor_count()) * thrust_coefficient * hover_speed * hover_speed;
  const double weight_n = model.mass.mass_kg * kStandardGravity;
  EXPECT_NEAR(total_thrust_n, weight_n, 1e-12) << "hover thrust does not equal the weight";

  const galata::sim::Wrench wrench = model.rotor_wrench(equal_speeds(model, hover_speed));

  // Thrust acts along body -z, so the force is the weight pointing up.
  EXPECT_NEAR(wrench.force_body_n.x(), 0.0, 1e-12);
  EXPECT_NEAR(wrench.force_body_n.y(), 0.0, 1e-12);
  EXPECT_NEAR(wrench.force_body_n.z(), -weight_n, 1e-12);

  // Equal speeds on a symmetric airframe: every moment cancels, including yaw,
  // which cancels only because the spin signs are two and two.
  EXPECT_NEAR(wrench.moment_cg_body_n_m.x(), 0.0, 1e-12) << "roll moment at hover";
  EXPECT_NEAR(wrench.moment_cg_body_n_m.y(), 0.0, 1e-12) << "pitch moment at hover";
  EXPECT_NEAR(wrench.moment_cg_body_n_m.z(), 0.0, 1e-12) << "yaw moment at hover";
}

// --- Case 2: free fall -----------------------------------------------------
//
// BUDGET. As case 1: an exact identity, gated at 1e-12 against a round-off
// floor near 1e-15.
TEST(QuadrotorFreeFall, ZeroRotorSpeedGivesZeroSpecificForceAndOneGeeDown) {
  const Quadrotor model = shipped_model();
  const Eigen::VectorXd extended = level_state(model, 0.0);
  const galata::core::State state = galata::core::State::from_vector(extended.head<kStateSize>());

  // At rest with the rotors stopped there is no thrust and no drag, so the
  // accelerometer reads zero. This is the property that separates specific
  // force from acceleration, and it is the one a sign error in gravity breaks.
  const galata::sim::Wrench wrench =
      model.wrench(state, Eigen::VectorXd::Zero(model.rotor_count()));
  EXPECT_NEAR(wrench.force_body_n.norm(), 0.0, 1e-12) << "specific force in free fall";
  EXPECT_NEAR(wrench.moment_cg_body_n_m.norm(), 0.0, 1e-12);

  const Eigen::VectorXd derivative =
      model.derivative(extended, Eigen::VectorXd::Zero(model.rotor_count()));

  // Level attitude, so body axes and NED axes coincide and the acceleration is
  // g downward — positive on the NED down axis.
  EXPECT_NEAR(derivative(galata::core::kVelocityU), 0.0, 1e-12);
  EXPECT_NEAR(derivative(galata::core::kVelocityV), 0.0, 1e-12);
  EXPECT_NEAR(derivative(galata::core::kVelocityW), kStandardGravity, 1e-12)
      << "free-fall acceleration is not one g down";
}

// --- Case 3: torque signs --------------------------------------------------
//
// BUDGET. Two-sided. The axis being driven must exceed 1e-3 of its own natural
// scale, which is a presence check rather than a tolerance; the two axes that
// must NOT move are gated at 1e-12, the round-off budget of cases 1 and 2. It
// is the second half that has the teeth: a transposed cross product or a
// swapped hub coordinate leaves the driven axis looking perfectly healthy and
// shows up only as a moment on an axis that should be silent.
TEST(QuadrotorTorqueSigns, DifferentialThrustDrivesTheExpectedAxisAndOnlyThatAxis) {
  const Quadrotor model = shipped_model();
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);
  const double delta = 0.05 * hover_speed;

  // Rotor order is front-left, rear-left, rear-right, front-right.
  const int front_left = 0;
  const int rear_left = 1;
  const int rear_right = 2;
  const int front_right = 3;

  // Roll: raise both LEFT rotors, lower both right. More thrust on the left
  // rolls the vehicle to the RIGHT, which is a positive roll moment in FRD.
  {
    Eigen::VectorXd speeds = equal_speeds(model, hover_speed);
    speeds(front_left) += delta;
    speeds(rear_left) += delta;
    speeds(rear_right) -= delta;
    speeds(front_right) -= delta;
    const galata::sim::Wrench wrench = model.rotor_wrench(speeds);
    EXPECT_GT(wrench.moment_cg_body_n_m.x(), 1e-3) << "raising the left rotors must roll right";
    EXPECT_NEAR(wrench.moment_cg_body_n_m.y(), 0.0, 1e-12) << "roll input produced a pitch moment";
    EXPECT_NEAR(wrench.moment_cg_body_n_m.z(), 0.0, 1e-12) << "roll input produced a yaw moment";
  }

  // Pitch: raise both FRONT rotors. Upward force ahead of the CG raises the
  // nose, which is a positive pitch moment.
  {
    Eigen::VectorXd speeds = equal_speeds(model, hover_speed);
    speeds(front_left) += delta;
    speeds(front_right) += delta;
    speeds(rear_left) -= delta;
    speeds(rear_right) -= delta;
    const galata::sim::Wrench wrench = model.rotor_wrench(speeds);
    EXPECT_GT(wrench.moment_cg_body_n_m.y(), 1e-3) << "raising the front rotors must pitch nose-up";
    EXPECT_NEAR(wrench.moment_cg_body_n_m.x(), 0.0, 1e-12) << "pitch input produced a roll moment";
    EXPECT_NEAR(wrench.moment_cg_body_n_m.z(), 0.0, 1e-12) << "pitch input produced a yaw moment";
  }

  // Yaw: raise the diagonal pair that shares a spin direction and lower the
  // other diagonal. Thrust and both tilting moments cancel; only the reaction
  // torques survive, and they yaw AGAINST the raised pair's spin.
  {
    Eigen::VectorXd speeds = equal_speeds(model, hover_speed);
    speeds(front_left) += delta;
    speeds(rear_right) += delta;
    speeds(rear_left) -= delta;
    speeds(front_right) -= delta;
    ASSERT_EQ(model.rotors[front_left].spin_about_body_z,
              model.rotors[rear_right].spin_about_body_z)
        << "this case assumes front-left and rear-right share a spin direction";
    const galata::sim::Wrench wrench = model.rotor_wrench(speeds);
    const double expected_sign = -static_cast<double>(model.rotors[front_left].spin_about_body_z);
    EXPECT_GT(expected_sign * wrench.moment_cg_body_n_m.z(), 1e-5)
        << "raising a same-spin diagonal must yaw against that spin";
    EXPECT_NEAR(wrench.moment_cg_body_n_m.x(), 0.0, 1e-12) << "yaw input produced a roll moment";
    EXPECT_NEAR(wrench.moment_cg_body_n_m.y(), 0.0, 1e-12) << "yaw input produced a pitch moment";
  }
}

// --- Case 4: torque-free conservation --------------------------------------
//
// BUDGET. The kernel's existing one, unchanged: TorqueFreeConservation in
// test_rigid_body_dynamics.cpp bounds relative energy and angular-momentum
// drift at 1e-11 over the same kind of run, and this case asserts that routing
// the same physics through the quadrotor's derivative does not degrade it. A
// looser bound here would prove nothing; the point is that the quadrotor adds
// no dissipation of its own when its rotors and drag are switched off.
TEST(QuadrotorTorqueFree, RotorsOffAndDragOffConservesEnergyAndAngularMomentum) {
  Quadrotor model = shipped_model();
  model.drag_linear_n_s_m.setZero();
  model.drag_quadratic_n_s2_m2.setZero();
  model.angular_drag_n_m_s.setZero();
  model.validate();

  Eigen::VectorXd extended = level_state(model, 0.0);
  // A general tumble: three unequal rates about a tensor with two equal
  // principal moments, so the motion is a real precession and not a rotation
  // about one axis, which conserves everything trivially.
  extended(galata::core::kRateP) = 0.9;
  extended(galata::core::kRateQ) = -0.4;
  extended(galata::core::kRateR) = 0.6;

  const Eigen::VectorXd command = Eigen::VectorXd::Zero(model.rotor_count());
  const auto derivative = [&](double, const Eigen::VectorXd& x) {
    return model.derivative(x, command);
  };
  const auto projection = [&](Eigen::VectorXd& x) { model.project(x); };

  const galata::core::State initial = galata::core::State::from_vector(extended.head<kStateSize>());
  const double energy_0 = galata::sim::rotational_kinetic_energy(initial, model.mass);
  const double momentum_0 = galata::sim::angular_momentum_ned(initial, model.mass).norm();
  ASSERT_GT(energy_0, 0.0);
  ASSERT_GT(momentum_0, 0.0);

  const galata::numerics::Trajectory trajectory = galata::numerics::integrate_fixed_step(
      derivative, extended, 0.0, 0.002, 5000, 10, projection);

  double worst_energy_drift = 0.0;
  double worst_momentum_drift = 0.0;
  for (const Eigen::VectorXd& sample : trajectory.states) {
    const galata::core::State state = galata::core::State::from_vector(sample.head<kStateSize>());
    worst_energy_drift = std::fmax(
        worst_energy_drift,
        std::fabs(galata::sim::rotational_kinetic_energy(state, model.mass) - energy_0) / energy_0);
    worst_momentum_drift = std::fmax(
        worst_momentum_drift,
        std::fabs(galata::sim::angular_momentum_ned(state, model.mass).norm() - momentum_0)
            / momentum_0);
  }

  EXPECT_LT(worst_energy_drift, 1e-11) << "relative energy drift";
  EXPECT_LT(worst_momentum_drift, 1e-11) << "relative angular-momentum drift";
}

// --- Case 5: step refinement -----------------------------------------------
//
// BUDGET, stated as an ORDER rather than as a magnitude. A magnitude budget for
// an integration step is a number somebody chose; the convergence order is a
// property of the scheme, and it is what actually distinguishes a correct
// fixed-step RK4 from one whose stage weights are subtly wrong. Halving the
// step must reduce the step-to-step difference by a factor near 2^4 = 16. The
// gate admits 10 to 22, which is wide enough for the finite-precision floor and
// the manoeuvre's nonlinearity and far too narrow for a scheme that is
// accidentally second or third order.
//
// The absolute agreement at the finest pair is reported alongside, so a reader
// sees the size of the error and not only its rate.
TEST(QuadrotorStepRefinement, HalvingTheStepConvergesAtFourthOrderOverAManoeuvre) {
  const Quadrotor model = shipped_model();
  const double hover_speed = model.hover_speed_rad_s(kStandardGravity);

  // A manoeuvre, not a hover: a steady roll-and-yaw command so the attitude
  // travels far enough for the integrator to be exercised.
  Eigen::VectorXd command = equal_speeds(model, hover_speed);
  command(0) += 0.03 * hover_speed;
  command(1) += 0.01 * hover_speed;
  command(2) -= 0.01 * hover_speed;
  command(3) -= 0.03 * hover_speed;

  const auto derivative = [&](double, const Eigen::VectorXd& x) {
    return model.derivative(x, command);
  };
  const auto projection = [&](Eigen::VectorXd& x) { model.project(x); };
  const Eigen::VectorXd start = level_state(model, hover_speed);

  // 30 s, as the case calls for, at three steps in a factor-of-two ladder.
  const auto final_state = [&](double step_s, int steps) {
    return galata::numerics::integrate_fixed_step(
               derivative, start, 0.0, step_s, steps, steps, projection)
        .states.back();
  };
  const Eigen::VectorXd coarse = final_state(0.004, 7500);
  const Eigen::VectorXd medium = final_state(0.002, 15000);
  const Eigen::VectorXd fine = final_state(0.001, 30000);

  const auto position_difference = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    return (a.head<3>() - b.head<3>()).norm();
  };
  const auto attitude_difference = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    return (a.segment(galata::core::kQuaternionW, 4) - b.segment(galata::core::kQuaternionW, 4))
        .norm();
  };

  const double position_coarse = position_difference(coarse, medium);
  const double position_fine = position_difference(medium, fine);
  const double attitude_coarse = attitude_difference(coarse, medium);
  const double attitude_fine = attitude_difference(medium, fine);

  ASSERT_GT(position_fine, 0.0)
      << "the two finest runs agree exactly, so the order is unmeasurable";
  ASSERT_GT(attitude_fine, 0.0);

  const double position_ratio = position_coarse / position_fine;
  const double attitude_ratio = attitude_coarse / attitude_fine;

  EXPECT_GT(position_ratio, 10.0) << "position convergence ratio " << position_ratio
                                  << " is below fourth order";
  EXPECT_LT(position_ratio, 22.0) << "position convergence ratio " << position_ratio
                                  << " is implausible for RK4";
  EXPECT_GT(attitude_ratio, 10.0) << "attitude convergence ratio " << attitude_ratio;
  EXPECT_LT(attitude_ratio, 22.0) << "attitude convergence ratio " << attitude_ratio;

  RecordProperty("position_difference_m_at_1ms", measured(position_fine));
  RecordProperty("attitude_difference_at_1ms", measured(attitude_fine));
}

// --- Case 6: cross-implementation against the Souxmar fixture --------------
//
// THIS IS A CROSS-CHECK, NOT A VALIDATION. Agreement here says two independent
// implementations of the same equations agree. It says nothing about any
// aircraft, and the case registry records it as self-consistent for that
// reason.
//
// WHY THIS CASE CAN SKIP, WHEN NO OTHER VALIDATION-TIER CASE DOES. Every other
// reference in this tier is committed, so its absence is a broken checkout and
// the right response is to fail. This fixture is deliberately NOT committed:
// it is a 2001-sample dataset whose rights position is unestablished, and
// ADR-0007 routes a dataset to a loader plus fetch instructions rather than
// into the tree. RFC-0002's acceptance section records that decision. The
// fixture is regenerated by the requesting programme with
//
//   python -m apps.galata_bridge --output outputs/galata_bridge --galata <galata-cli>
//
// and this case is pointed at the result with -DGALATA_SOUXMAR_FIXTURE_DIR.
// A skip here is therefore a statement about the checkout, not about galata,
// and it says so.
//
// BUDGET, and where it comes from. The two implementations differ in
// integration scheme as well as in code: the fixture applies an EXACT
// first-order rotor lag sampled at the RK stages, galata carries the rotor
// speed as an ODE state through classical RK4. At the fixture's dt of 0.004 s
// and tau of 0.035 s the two amplification factors differ by 1.79e-7 relative
// per step, and the requesting programme measured the resulting envelope
// difference over one three-per-cent rotor transient at no more than 5.8e-7 of
// the commanded fraction — about 3.6e-4 rad/s on a 626 rad/s rotor. That
// scheme term is the floor for the rotor channel and it is attributed, not
// absorbed. The remaining budgets are set an order above the motion that term
// can induce over 8 s. A disagreement beyond them is a finding to be reported,
// never a tolerance to widen.
namespace {

struct FixtureTable {
  std::vector<std::string> columns;
  std::vector<std::vector<double>> rows;

  [[nodiscard]] int index_of(const std::string& name) const {
    for (std::size_t i = 0; i < columns.size(); ++i) {
      if (columns[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }
};

// The fixture is written with CRLF endings, so the trailing carriage return
// would otherwise become part of the last column's NAME and the lookup would
// fail on a column that is plainly present.
void strip_carriage_return(std::string& line) {
  while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
    line.pop_back();
  }
}

bool read_fixture(const std::string& path, FixtureTable& table, std::string& why_not) {
  std::ifstream file(path);
  if (!file) {
    why_not = "cannot open " + path;
    return false;
  }
  std::string line;
  if (!std::getline(file, line)) {
    why_not = "empty file " + path;
    return false;
  }
  strip_carriage_return(line);
  std::stringstream header(line);
  std::string field;
  while (std::getline(header, field, ',')) {
    table.columns.push_back(field);
  }
  while (std::getline(file, line)) {
    strip_carriage_return(line);
    if (line.empty()) {
      continue;
    }
    std::stringstream values(line);
    std::vector<double> row;
    row.reserve(table.columns.size());
    while (std::getline(values, field, ',')) {
      row.push_back(std::stod(field));
    }
    if (row.size() != table.columns.size()) {
      why_not = "ragged row in " + path;
      return false;
    }
    table.rows.push_back(std::move(row));
  }
  return !table.rows.empty();
}

}  // namespace

TEST(QuadrotorCrossImplementation, ReproducesTheSouxmarOpenLoopTrajectory) {
  const std::string directory = GALATA_SOUXMAR_FIXTURE_DIR;
  if (directory.empty()) {
    GTEST_SKIP() << "no cross-implementation fixture configured: this case compares against an "
                    "external programme's trajectory, which is not committed because its rights "
                    "position is unestablished (ADR-0007, RFC-0002). Configure with "
                    "-DGALATA_SOUXMAR_FIXTURE_DIR=<dir containing reference_trajectory.csv>.";
  }
  const std::string path = directory + "/reference_trajectory.csv";
  FixtureTable fixture;
  std::string why_not;
  if (!read_fixture(path, fixture, why_not)) {
    GTEST_SKIP() << "cross-implementation fixture unavailable: " << why_not
                 << ". Regenerate it with `python -m apps.galata_bridge --output "
                    "outputs/galata_bridge --galata <galata-cli>`.";
  }

  const Quadrotor model = shipped_model();
  ASSERT_EQ(model.rotor_count(), 4);

  // Column names, in the fixture's own rotor order and in this repository's
  // frame convention. The fixture publishes both conventions; the NED/FRD
  // columns are its own transform of the ENU/FLU ones and no component is
  // reinterpreted here.
  const std::vector<std::string> command_columns = {
      "cmd_fl_rad_s", "cmd_rl_rad_s", "cmd_rr_rad_s", "cmd_fr_rad_s"};
  const std::vector<std::string> rotor_columns = {
      "rotor_fl_rad_s", "rotor_rl_rad_s", "rotor_rr_rad_s", "rotor_fr_rad_s"};
  const std::vector<std::string> needed = {"time_s",
                                           "north_m_ned",
                                           "east_m_ned",
                                           "down_m_ned",
                                           "ground_v_north_m_s_ned",
                                           "ground_v_east_m_s_ned",
                                           "ground_v_down_m_s_ned",
                                           "q_w_frd_to_ned",
                                           "q_x_frd_to_ned",
                                           "q_y_frd_to_ned",
                                           "q_z_frd_to_ned",
                                           "p_frd_rad_s",
                                           "q_frd_rad_s",
                                           "r_frd_rad_s",
                                           "wind_east_m_s",
                                           "wind_north_m_s",
                                           "wind_up_m_s"};
  for (const std::string& column : needed) {
    ASSERT_GE(fixture.index_of(column), 0) << "fixture is missing column " << column;
  }

  const auto column = [&](const std::vector<double>& row, const std::string& name) {
    return row[static_cast<std::size_t>(fixture.index_of(name))];
  };

  // Wind is published in ENU and enters this repository as NED by
  // [n, e, d] = [y, x, -z]. It is constant over the fixture's schedule, but it
  // is read per row rather than assumed.
  const auto wind_ned = [&](const std::vector<double>& row) {
    return Eigen::Vector3d(
        column(row, "wind_north_m_s"), column(row, "wind_east_m_s"), -column(row, "wind_up_m_s"));
  };

  // The fixture's state is GROUND velocity; this repository's state velocity is
  // AIR-RELATIVE. The conversion is v_body = R_body<-ned (v_ground - wind),
  // which the requesting programme confirmed is exactly its own drag argument.
  const auto extended_from_row = [&](const std::vector<double>& row) {
    Eigen::VectorXd extended = Eigen::VectorXd::Zero(model.extended_state_size());
    extended(galata::core::kPositionNorth) = column(row, "north_m_ned");
    extended(galata::core::kPositionEast) = column(row, "east_m_ned");
    extended(galata::core::kPositionDown) = column(row, "down_m_ned");
    const galata::core::Quaternion attitude(column(row, "q_w_frd_to_ned"),
                                            column(row, "q_x_frd_to_ned"),
                                            column(row, "q_y_frd_to_ned"),
                                            column(row, "q_z_frd_to_ned"));
    const Eigen::Vector3d ground_ned(column(row, "ground_v_north_m_s_ned"),
                                     column(row, "ground_v_east_m_s_ned"),
                                     column(row, "ground_v_down_m_s_ned"));
    const Eigen::Vector3d air_body =
        galata::core::dcm_ned_from_body(attitude).transpose() * (ground_ned - wind_ned(row));
    extended(galata::core::kVelocityU) = air_body.x();
    extended(galata::core::kVelocityV) = air_body.y();
    extended(galata::core::kVelocityW) = air_body.z();
    extended(galata::core::kQuaternionW) = attitude.w();
    extended(galata::core::kQuaternionX) = attitude.x();
    extended(galata::core::kQuaternionY) = attitude.y();
    extended(galata::core::kQuaternionZ) = attitude.z();
    extended(galata::core::kRateP) = column(row, "p_frd_rad_s");
    extended(galata::core::kRateQ) = column(row, "q_frd_rad_s");
    extended(galata::core::kRateR) = column(row, "r_frd_rad_s");
    for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
      extended(model.rotor_state_offset() + rotor) =
          column(row, rotor_columns[static_cast<std::size_t>(rotor)]);
    }
    return extended;
  };

  Eigen::VectorXd extended = extended_from_row(fixture.rows.front());
  const double step_s = column(fixture.rows[1], "time_s") - column(fixture.rows[0], "time_s");
  ASSERT_NEAR(step_s, 0.004, 1e-12) << "the fixture's step is not the declared 0.004 s";

  double worst_position_m = 0.0;
  double worst_attitude = 0.0;
  double worst_rate_rad_s = 0.0;
  double worst_rotor_rad_s = 0.0;

  // BOTH trajectories are retained, not just the disagreement between them.
  // A scalar worst-case says a run agreed; it does not let a reader see WHERE
  // the two diverged, and that is the question anyone asks next.
  std::ostringstream retained;
  retained << "time_s";
  for (const char* quantity : {"north_m",
                               "east_m",
                               "down_m",
                               "q_w",
                               "q_x",
                               "q_y",
                               "q_z",
                               "p_rad_s",
                               "q_rad_s",
                               "r_rad_s",
                               "rotor_0_rad_s",
                               "rotor_1_rad_s",
                               "rotor_2_rad_s",
                               "rotor_3_rad_s"}) {
    retained << ",galata_" << quantity << ",souxmar_" << quantity;
  }
  retained << "\n" << std::scientific << std::setprecision(12);
  const auto retain =
      [&](double time_s, const Eigen::VectorXd& mine_row, const Eigen::VectorXd& theirs_row) {
        retained << time_s;
        for (int i : {0, 1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}) {
          retained << ',' << mine_row(i) << ',' << theirs_row(i);
        }
        retained << '\n';
      };
  retain(column(fixture.rows.front(), "time_s"), extended, extended);

  Eigen::Vector3d previous_wind = wind_ned(fixture.rows.front());
  for (std::size_t step = 0; step + 1 < fixture.rows.size(); ++step) {
    // Commands are held over each step, which is the fixture's own statement of
    // its schedule.
    Eigen::VectorXd command(model.rotor_count());
    for (int rotor = 0; rotor < model.rotor_count(); ++rotor) {
      command(rotor) = column(fixture.rows[step], command_columns[static_cast<std::size_t>(rotor)]);
    }
    const Eigen::Vector3d wind = wind_ned(fixture.rows[step]);

    // THE WIND STEP IS A DISCONTINUITY IN THIS STATE, AND NOT IN THE FIXTURE'S.
    //
    // ADR-0002's velocity is air-relative; the fixture carries ground velocity.
    // When the wind changes, the GROUND velocity is continuous — no force acts
    // at the instant the air mass changes speed — so the air-relative velocity
    // must jump by exactly minus the wind change. The model's derivative takes
    // the wind as steady and carries no -R^T dw/dt term, so the caller owns this
    // re-basing, exactly as state.hpp makes the caller own adding the wind back
    // to the position rate. Omitting it injects the whole wind vector as a
    // ground-velocity error at the step.
    if (wind != previous_wind) {
      const galata::core::State current =
          galata::core::State::from_vector(extended.head<kStateSize>());
      const Eigen::Vector3d jump =
          galata::core::dcm_ned_from_body(current.attitude_body_to_ned).transpose()
          * (previous_wind - wind);
      extended.segment(galata::core::kVelocityU, 3) += jump;
      previous_wind = wind;
    }
    const auto derivative = [&](double, const Eigen::VectorXd& x) {
      return model.derivative(x, command, wind);
    };
    extended = galata::numerics::rk4_step(derivative, 0.0, extended, step_s);
    model.project(extended);

    const Eigen::VectorXd expected = extended_from_row(fixture.rows[step + 1]);
    worst_position_m =
        std::fmax(worst_position_m, (extended.head<3>() - expected.head<3>()).norm());
    // Quaternion difference taken on the shorter of q and -q: the double cover
    // is not collapsed in the state, so a sign flip is the same attitude.
    const Eigen::Vector4d mine = extended.segment(galata::core::kQuaternionW, 4);
    const Eigen::Vector4d theirs = expected.segment(galata::core::kQuaternionW, 4);
    worst_attitude =
        std::fmax(worst_attitude, std::fmin((mine - theirs).norm(), (mine + theirs).norm()));
    worst_rate_rad_s = std::fmax(
        worst_rate_rad_s,
        (extended.segment(galata::core::kRateP, 3) - expected.segment(galata::core::kRateP, 3))
            .norm());
    worst_rotor_rad_s =
        std::fmax(worst_rotor_rad_s,
                  (extended.segment(model.rotor_state_offset(), model.rotor_count())
                   - expected.segment(model.rotor_state_offset(), model.rotor_count()))
                      .norm());
    retain(column(fixture.rows[step + 1], "time_s"), extended, expected);
  }

  const std::string retained_path =
      std::string(GALATA_VALIDATION_OUTPUT_DIR) + "/quadrotor_cross_implementation.csv";
  {
    std::ofstream out(retained_path);
    ASSERT_TRUE(out) << "cannot retain the compared trajectories at " << retained_path;
    out << retained.str();
  }
  RecordProperty("retained_trajectories", retained_path);

  RecordProperty("worst_position_m", measured(worst_position_m));
  RecordProperty("worst_attitude", measured(worst_attitude));
  RecordProperty("worst_body_rate_rad_s", measured(worst_rate_rad_s));
  RecordProperty("worst_rotor_rad_s", measured(worst_rotor_rad_s));

  // The rotor channel carries the scheme term derived in this case's banner.
  EXPECT_LT(worst_rotor_rad_s, 1e-2) << "rotor speeds disagree beyond the integration-scheme term";
  EXPECT_LT(worst_attitude, 1e-6) << "attitude disagrees beyond the stated budget";
  EXPECT_LT(worst_rate_rad_s, 1e-5) << "body rates disagree beyond the stated budget";
  EXPECT_LT(worst_position_m, 1e-3) << "position disagrees beyond the stated budget";
}
