// SPDX-License-Identifier: Apache-2.0
//
// The Level-1 helicopter model and the declared trim problem.
//
// The centre of this file is the hover trim: six unknowns, six residuals, and
// the anti-torque balance that decides whether the model is a helicopter or a
// plausible-looking arrangement of forces. Before the rotor module and the
// declared trim problem, neither could be written — the only rotary-wing trim
// refused anything but exactly four rotors, and rotor force had no direction.

#include "galata/model/helicopter.hpp"
#include "galata/trim/problem.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

using galata::core::State;
using galata::core::identity_attitude;
using galata::model::Environment;
using galata::model::HelicopterModel;
using galata::model::VehicleModel;
using galata::model::kCollectiveCommand;
using galata::model::kCollectivePosition;
using galata::model::kMainRotorSpeed;
using galata::model::kPedalCommand;
using galata::model::kPedalPosition;
using galata::model::load_helicopter;
using galata::trim::TrimCondition;
using galata::trim::helicopter_trim_problem;
using galata::trim::solve_trim;

namespace {

std::string model_path() {
  return std::string(GALATA_SOURCE_DIR) + "/models/souxmar-heli/souxmar-heli.yaml";
}

HelicopterModel souxmar() {
  return load_helicopter(model_path());
}

TrimCondition hover_condition(const HelicopterModel& heli, const Environment& environment) {
  TrimCondition condition;
  condition.environment = environment;
  condition.controls = Eigen::VectorXd::Zero(heli.control_count());
  State state;
  state.attitude_body_to_ned = identity_attitude();
  condition.extended_state =
      heli.join(state, heli.initial_auxiliary(condition.controls, environment));
  return condition;
}

}  // namespace

// ---------------------------------------------------------------------------
// The model loads, validates and speaks a vocabulary.
// ---------------------------------------------------------------------------

TEST(Helicopter, LoadsFromYamlAndValidates) {
  const auto heli = souxmar();
  EXPECT_NO_THROW(heli.validate());
  EXPECT_EQ(heli.extended_state_size(), 21);
  EXPECT_EQ(heli.auxiliary_state_count(), 8);
  EXPECT_EQ(heli.control_count(), 4);
  EXPECT_FALSE(heli.citation.empty());
}

TEST(Helicopter, TheControlVocabularyIsCollectiveCyclicAndPedal) {
  const auto heli = souxmar();
  const auto controls = heli.control_names();
  ASSERT_EQ(controls.size(), 4u);
  EXPECT_EQ(controls[0], "collective_command_rad");
  EXPECT_EQ(controls[1], "longitudinal_cyclic_command_rad");
  EXPECT_EQ(controls[2], "lateral_cyclic_command_rad");
  EXPECT_EQ(controls[3], "pedal_command_rad");

  // And the actuator POSITIONS are states, distinct from the commands.
  const auto states = heli.state_names();
  EXPECT_NE(std::find(states.begin(), states.end(), "collective_rad"), states.end());
  EXPECT_NE(std::find(states.begin(), states.end(), "pedal_rad"), states.end());
  EXPECT_NE(std::find(states.begin(), states.end(), "main_rotor_speed_rad_s"), states.end());
}

TEST(Helicopter, SolidityRoundTripsThroughTheDeclaredChord) {
  const auto heli = souxmar();
  // The YAML carries a chord derived from the design's declared solidity, so
  // the round trip must be exact or the two have drifted apart.
  EXPECT_NEAR(heli.main_rotor.solidity(), 0.072, 1e-12);
  EXPECT_NEAR(heli.tail_rotor.solidity(), 0.180, 1e-12);
}

TEST(Helicopter, ProducesAFiniteWrenchAcrossItsDeclaredEnvelope) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const Eigen::VectorXd controls = Eigen::VectorXd::Zero(heli.control_count());
  Eigen::VectorXd auxiliary = heli.initial_auxiliary(controls, environment);

  for (double collective = 0.0; collective <= 0.34; collective += 0.02) {
    for (double speed : {0.0, 10.0, 30.0, 50.0, 60.0}) {
      State state;
      state.attitude_body_to_ned = identity_attitude();
      state.velocity_body_m_s = Eigen::Vector3d(speed, 0.0, 0.0);
      auxiliary(kCollectivePosition) = collective;
      const auto wrench = heli.wrench(state, auxiliary, controls, environment);
      ASSERT_TRUE(wrench.force_body_n.allFinite())
          << "collective " << collective << " speed " << speed;
      ASSERT_TRUE(wrench.moment_cg_body_n_m.allFinite())
          << "collective " << collective << " speed " << speed;
    }
  }
}

// ---------------------------------------------------------------------------
// The anti-torque sense check. This is the one that catches a sign error that
// nothing else in the model notices.
// ---------------------------------------------------------------------------

TEST(Helicopter, RefusesAModelWhoseTailRotorReinforcesTheTorqueItOpposes) {
  auto heli = souxmar();
  ASSERT_NO_THROW(heli.validate());

  // Flip the tail rotor to push the other way. The aircraft now yaws itself
  // further in the direction the main rotor already yaws it.
  heli.tail_rotor.hub_to_body = galata::model::rotor::tail_rotor_hub(true);
  EXPECT_THROW(heli.validate(), std::invalid_argument);
  try {
    heli.validate();
  } catch (const std::invalid_argument& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("anti-torque sense"), std::string::npos) << what;
    EXPECT_NE(what.find("spin_about_shaft"), std::string::npos) << what;
  }

  // Reversing the pedal gearing instead reaches the same wrong configuration.
  auto reversed = souxmar();
  reversed.pedal_to_tail_collective = -1.0;
  EXPECT_THROW(reversed.validate(), std::invalid_argument);
}

TEST(Helicopter, RefusesAnActuatorWithNoRateLimit) {
  auto heli = souxmar();
  heli.actuators[kCollectiveCommand].rate_limit_rad_s = 0.0;
  EXPECT_THROW(heli.validate(), std::invalid_argument);
}

TEST(Helicopter, RefusesAnEngineThatCouldMotorTheRotor) {
  auto heli = souxmar();
  heli.drivetrain.minimum_engine_torque_n_m = -100.0;
  EXPECT_THROW(heli.validate(), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// HOVER TRIM — six declared unknowns, six residuals.
// ---------------------------------------------------------------------------

TEST(HelicopterTrim, SolvesHoverForItsDeclaredUnknowns) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto problem = helicopter_trim_problem(heli);

  // SIX PILOT UNKNOWNS PLUS THE TWO INFLOW STATES. The six are roll, pitch,
  // collective, both cyclics and pedal, against the six force-and-moment
  // residuals. The two inflow states are there because a rotor with a declared
  // dynamic-inflow lag carries its inflow as a STATE with its own equilibrium,
  // and solving the six alone leaves it wherever the guess put it — an
  // equilibrium of the airframe and not of the rotor. That omission was
  // invisible in the forces, which trimmed to 1e-17, and was caught only when
  // linearize_vehicle asked for the inflow's own rate and refused the point
  // with `tail_inflow_ratio` named.
  // NINE: the six pilot unknowns, the two rotor inflow states, and the engine
  // torque. The last three are there because each is a STATE with its own
  // equilibrium — a rotor's inflow settles, and the governed engine torque is
  // the value at which the rotor neither accelerates nor decelerates. Solving
  // the six force-and-moment equations alone leaves all three wherever the guess
  // put them, which is an equilibrium of the airframe and not of the aircraft.
  ASSERT_EQ(problem.unknowns.size(), 9u);
  ASSERT_EQ(problem.residuals.size(), 9u);
  const auto has_unknown = [&](const std::string& name) {
    return std::any_of(problem.unknowns.begin(), problem.unknowns.end(),
                       [&](const auto& u) { return u.name == name; });
  };
  EXPECT_TRUE(has_unknown("collective_rad"));
  EXPECT_TRUE(has_unknown("pedal_rad"));
  EXPECT_TRUE(has_unknown("main_inflow_ratio"));
  EXPECT_TRUE(has_unknown("tail_inflow_ratio"));
  EXPECT_TRUE(has_unknown("engine_torque_n_m"));

  const auto result = solve_trim(heli, problem, hover_condition(heli, environment));

  EXPECT_TRUE(result.converged);
  // The residual is body accelerations, so this is metres per second squared
  // and radians per second squared: 1e-8 is a very tight equilibrium.
  EXPECT_LT(result.residual_norm, 1.0e-8);
  EXPECT_EQ(result.jacobian_rank, 9);
  EXPECT_EQ(result.unknown_count, 9);
  // A WELL-CONDITIONED PROBLEM, and the gate is tight on purpose. The unknowns
  // are non-dimensionalised by their declared scales before the Jacobian is
  // taken, so a large condition number here means the PHYSICS is
  // ill-conditioned rather than the units. Before that scaling was applied
  // correctly this measured 7.4e6, all of it units; it now measures about 1.9e2.
  EXPECT_LT(result.jacobian_condition_number, 1.0e4);
  EXPECT_TRUE(result.unconstrained_unknowns.empty());
  EXPECT_TRUE(result.out_of_bounds_unknowns.empty());
}

TEST(HelicopterTrim, TheHoverTrimIsPhysicallySensible) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto result =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  ASSERT_TRUE(result.converged);

  const auto value = [&](const std::string& name) {
    const auto it = std::find(result.unknown_names.begin(), result.unknown_names.end(), name);
    EXPECT_NE(it, result.unknown_names.end()) << name;
    return result.unknown_values(it - result.unknown_names.begin());
  };

  // Collective in the band a helicopter of this disc loading actually hovers at.
  const double collective = value("collective_rad");
  EXPECT_GT(collective, 0.15);
  EXPECT_LT(collective, 0.30);

  // THE AIRCRAFT BANKS. The tail rotor pushes sideways, so a hovering
  // helicopter must hold a small bank angle for the lateral forces to balance.
  // This is real and every helicopter does it; a model that trims at zero roll
  // has lost the tail-rotor side force somewhere.
  const double roll = value("roll_rad");
  EXPECT_GT(std::fabs(roll), 0.005) << "a hovering helicopter banks against its tail-rotor thrust";
  EXPECT_LT(std::fabs(roll), 0.15);

  // Pedal is displaced from neutral: the tail rotor is working.
  EXPECT_GT(std::fabs(value("pedal_rad")), 0.005);
}

TEST(HelicopterTrim, TheAntiTorqueBalancesInHover) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto result =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  ASSERT_TRUE(result.converged);

  const auto parts = heli.breakdown(VehicleModel::rigid_body_part(result.extended_state),
                                    heli.auxiliary_part(result.extended_state), result.controls,
                                    environment);

  // The yaw moment about the CG vanishes: that IS the anti-torque balance, and
  // it is what the sixth residual asked for.
  EXPECT_NEAR(parts.anti_torque_residual_n_m, 0.0, 1.0e-6);

  // And it balances the way it should: tail thrust at its arm against the main
  // rotor's shaft torque. The two agree to a few percent, the remainder being
  // the tail rotor's own shaft torque and the fin's side force.
  const double tail_moment = 6.7 * std::fabs(parts.tail.wrench.force_body_n.y());
  EXPECT_NEAR(tail_moment, parts.main.torque_n_m, 0.05 * parts.main.torque_n_m);

  // Main thrust carries the weight, to within the tilt of the thrust vector.
  EXPECT_NEAR(parts.main.thrust_n, heli.mass.mass_kg * 9.80665,
              0.03 * heli.mass.mass_kg * 9.80665);
}

TEST(HelicopterTrim, TheRotorSpeedIsStationaryAtTheTrim) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto result =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  ASSERT_TRUE(result.converged);

  // The governor holds rotor speed, so the rate is near zero even though it is
  // NOT one of the six residuals. This is the check that the drivetrain's
  // torque balance is consistent with the trim the six equations found.
  const double rate = galata::trim::rotor_speed_residual(heli, result, environment,
                                                         "main_rotor_speed_rad_s");
  EXPECT_LT(std::fabs(rate), 1.0e-6) << "rotor speed drifts at the trim point";
}

TEST(HelicopterTrim, ForwardFlightTrimsAndNeedsForwardCyclic) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();

  double previous_cyclic = 0.0;
  for (double speed : {0.0, 20.0, 40.0}) {
    auto condition = hover_condition(heli, environment);
    condition.extended_state(galata::core::kVelocityU) = speed;
    const auto result = solve_trim(heli, helicopter_trim_problem(heli), condition);
    ASSERT_TRUE(result.converged) << "at " << speed << " m/s";

    const auto it = std::find(result.unknown_names.begin(), result.unknown_names.end(),
                              "longitudinal_cyclic_rad");
    const double cyclic = result.unknown_values(it - result.unknown_names.begin());
    if (speed > 0.0) {
      // Forward flight needs progressively more forward cyclic to overcome
      // fuselage drag and the rotor's own flap-back. Monotone in speed.
      EXPECT_GT(cyclic, previous_cyclic)
          << "forward cyclic must increase with airspeed; at " << speed << " m/s";
    }
    previous_cyclic = cyclic;
  }
}

// ---------------------------------------------------------------------------
// The refusals. A declared problem can be malformed in ways a hard-coded one
// could not, so each is checked.
// ---------------------------------------------------------------------------

TEST(HelicopterTrim, RefusesANonSquareProblem) {
  const auto heli = souxmar();
  auto problem = helicopter_trim_problem(heli);
  problem.unknowns.pop_back();
  EXPECT_THROW(problem.validate(heli), std::invalid_argument);
  try {
    problem.validate(heli);
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("SQUARE"), std::string::npos) << error.what();
  }
}

TEST(HelicopterTrim, RefusesAnUnknownTheModelDoesNotHave) {
  const auto heli = souxmar();
  auto problem = helicopter_trim_problem(heli);
  problem.unknowns[0].name = "elevator_rad";
  EXPECT_THROW(problem.validate(heli), std::invalid_argument);
  try {
    problem.validate(heli);
  } catch (const std::invalid_argument& error) {
    // The diagnostic lists what the model DOES have, so the caller can fix it
    // without reading the source.
    const std::string what = error.what();
    EXPECT_NE(what.find("collective_rad"), std::string::npos) << what;
  }
}

TEST(HelicopterTrim, RefusesADuplicatedUnknownOrResidual) {
  const auto heli = souxmar();
  auto duplicate_unknown = helicopter_trim_problem(heli);
  duplicate_unknown.unknowns[1].name = duplicate_unknown.unknowns[0].name;
  EXPECT_THROW(duplicate_unknown.validate(heli), std::invalid_argument);

  auto duplicate_residual = helicopter_trim_problem(heli);
  duplicate_residual.residuals[1].kind = duplicate_residual.residuals[0].kind;
  EXPECT_THROW(duplicate_residual.validate(heli), std::invalid_argument);
}

TEST(HelicopterTrim, RefusesAnUnconstrainedUnknownAndNamesIt) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  auto problem = helicopter_trim_problem(heli);
  // Yaw is not constrained by any of the six residuals: a helicopter in still
  // air is in equilibrium at any heading. Declaring it as an unknown makes the
  // system singular, and the solver must say WHICH unknown rather than
  // returning whichever point the iteration reached.
  problem.unknowns[0].name = "yaw_rad";

  try {
    const auto result = solve_trim(heli, problem, hover_condition(heli, environment));
    FAIL() << "a singular trim must be refused, not answered; got residual "
           << result.residual_norm;
  } catch (const std::runtime_error& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("yaw_rad"), std::string::npos)
        << "the refusal must name the unconstrained unknown: " << what;
    EXPECT_NE(what.find("Jacobian rank"), std::string::npos) << what;
  }
}

TEST(HelicopterTrim, RefusesATrimOutsideTheDeclaredActuatorTravel) {
  auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  // The collective travel is cut to a quarter of what the hover needs. The
  // AIRCRAFT can still lift itself — the engine and rotor are untouched — so
  // this isolates the bounds refusal from the saturation refusal below.
  auto problem = helicopter_trim_problem(heli);
  for (auto& unknown : problem.unknowns) {
    if (unknown.name == "collective_rad") {
      unknown.maximum = 0.06;  // about 3.4 degrees; hover needs 15
    }
  }
  try {
    (void)solve_trim(heli, problem, hover_condition(heli, environment));
    FAIL() << "a trim needing more collective than the declared travel must be refused";
  } catch (const std::runtime_error& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("Outside declared bounds"), std::string::npos) << what;
    EXPECT_NE(what.find("collective_rad"), std::string::npos) << what;
  }
}

TEST(HelicopterTrim, RefusesAConditionBeyondTheDrivetrainAndExplainsSaturation) {
  auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  // Four times the mass needs far more torque than the drivetrain has. The
  // engine torque saturates, its Jacobian column goes flat, and the solve
  // becomes rank deficient. That is a real signal and the message has to make it
  // actionable rather than leaving the reader with "rank 8 of 9".
  heli.mass.mass_kg *= 4.0;
  try {
    (void)solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
    FAIL() << "a condition beyond the drivetrain must be refused";
  } catch (const std::runtime_error& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("engine_torque_n_m"), std::string::npos) << what;
    EXPECT_NE(what.find("SATURATED"), std::string::npos)
        << "the refusal must explain that a flat column is usually a saturation: " << what;
    EXPECT_NE(what.find("beyond the aircraft"), std::string::npos) << what;
  }
}

TEST(HelicopterTrim, TheRefusalCarriesTheFiveNumbersThatDiagnoseIt) {
  auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  heli.mass.mass_kg *= 4.0;
  try {
    (void)solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
    FAIL() << "expected a refusal";
  } catch (const std::runtime_error& error) {
    const std::string what = error.what();
    for (const char* expected : {"Residual norm", "budget", "iterations", "Jacobian rank",
                                 "condition number"}) {
      EXPECT_NE(what.find(expected), std::string::npos)
          << "the refusal must carry '" << expected << "': " << what;
    }
  }
}

// ---------------------------------------------------------------------------
// Failures, which are multipliers rather than special cases in the physics.
// ---------------------------------------------------------------------------

TEST(Helicopter, TailRotorFailureRemovesTheAntiTorqueAndLeavesAYawMoment) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto trimmed =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  ASSERT_TRUE(trimmed.converged);

  auto failed = souxmar();
  failed.failures.tail_rotor_effectiveness = 0.0;
  const auto parts = failed.breakdown(VehicleModel::rigid_body_part(trimmed.extended_state),
                                      failed.auxiliary_part(trimmed.extended_state),
                                      trimmed.controls, environment);

  // With the tail rotor gone the main rotor's torque is unopposed, so a large
  // yaw moment remains where the trimmed aircraft had none.
  EXPECT_GT(std::fabs(parts.anti_torque_residual_n_m), 1000.0);
  EXPECT_NEAR(parts.tail.thrust_n, 0.0, 1e-12);
  // And it acts in the sense the main rotor's torque does.
  EXPECT_LT(parts.anti_torque_residual_n_m, 0.0);
}

TEST(Helicopter, EngineFailureRemovesTheSuppliedTorqueSoTheRotorDecays) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto trimmed =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  ASSERT_TRUE(trimmed.converged);

  auto failed = souxmar();
  failed.failures.engine_available_fraction = 0.0;
  const Eigen::VectorXd rate =
      failed.derivative(trimmed.extended_state, trimmed.controls, environment);
  const int speed_index = galata::core::kStateSize + kMainRotorSpeed;
  EXPECT_LT(rate(speed_index), 0.0) << "with no engine torque the rotor must decelerate";
}

TEST(Helicopter, AJammedActuatorDoesNotMove) {
  auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  heli.failures.jammed_actuator_rad[kPedalCommand] = 0.05;

  Eigen::VectorXd controls = Eigen::VectorXd::Zero(heli.control_count());
  controls(kPedalCommand) = 0.3;  // commanded far from the jam
  Eigen::VectorXd auxiliary = heli.initial_auxiliary(controls, environment);
  auxiliary(kPedalPosition) = 0.05;

  State state;
  state.attitude_body_to_ned = identity_attitude();
  const Eigen::VectorXd rate =
      heli.auxiliary_derivative(state, auxiliary, controls, environment);
  EXPECT_EQ(rate(kPedalPosition), 0.0) << "a jammed actuator's rate is zero, not merely slow";
}

// ---------------------------------------------------------------------------
// Determinism.
// ---------------------------------------------------------------------------

TEST(HelicopterTrim, RepeatedTrimsAreBitIdentical) {
  const auto heli = souxmar();
  const auto environment = Environment::sea_level_still_air();
  const auto a =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  const auto b =
      solve_trim(heli, helicopter_trim_problem(heli), hover_condition(heli, environment));
  ASSERT_TRUE(a.converged);
  ASSERT_TRUE(b.converged);
  for (Eigen::Index i = 0; i < a.unknown_values.size(); ++i) {
    EXPECT_EQ(a.unknown_values(i), b.unknown_values(i)) << "unknown " << a.unknown_names[static_cast<std::size_t>(i)];
  }
  EXPECT_EQ(a.residual_norm, b.residual_norm);
  EXPECT_EQ(a.jacobian_condition_number, b.jacobian_condition_number);
}
