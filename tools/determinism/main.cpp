// SPDX-License-Identifier: Apache-2.0
//
// Emits a determinism fingerprint: a fixed battery of computations, each
// printed at full round-trippable precision.
//
// The fingerprint is used two ways, and ADR-0004 explains why they are
// different claims:
//
//   TIER 1  same binary, same platform, run twice: byte-identical output.
//           Gated with `diff`.
//
//   TIER 2  same source, different platform: agreement to a published bound.
//           Gated with scripts/compare-determinism.sh, which reports the
//           observed deviation rather than merely asserting one.
//
// It is not byte-identical across platforms and does not claim to be. `pow`,
// `exp` and the trigonometric functions come from the platform's math library,
// and those do not agree in their final bits. Every line in this battery passes
// through at least one of them.
//
// Adding a line here is cheap and is the right response to any new numerical
// path: what is not fingerprinted is not gated.
//
// ONE CONSTRAINT ON WHAT BELONGS HERE. A cross-platform bound is only
// meaningful on a computation that does not amplify small differences. Two
// platforms' math libraries disagree by around 1e-16 relative; a chaotic
// trajectory would turn that into anything at all, and the tier 2 gate would
// then be measuring chaos rather than agreement. The rigid-body case below was
// measured: it amplifies a perturbation by a factor between 0.06 and 1.0 over
// 60 s — a bounded tumble under a constant wrench — which leaves the 1e-9 gate
// about seven orders of headroom.
// Determinism.TheFingerprintTrajectoryIsNotChaotic asserts this, so a future
// change that makes the battery chaotic fails there rather than as an
// intermittently red workflow.

#include "galata/analyze/modes.hpp"
#include "galata/core/atmosphere.hpp"
#include "galata/core/quaternion.hpp"
#include "galata/core/state.hpp"
#include "galata/numerics/integrator.hpp"
#include "galata/sim/rigid_body.hpp"
#include "galata/version.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

// %.17g round-trips a double exactly, so a byte-identical fingerprint means
// bit-identical values and not merely values that print the same.
void emit(const std::string& key, double value) {
  std::printf("%s\t%.17g\n", key.c_str(), value);
}

void fingerprint_atmosphere() {
  for (const double altitude :
       {-1000.0, 0.0, 1500.0, 3048.0, 11000.0, 20000.0, 47000.0, 71000.0, 86000.0}) {
    const galata::core::AtmosphereState state = galata::core::isa(altitude);
    const std::string prefix = "atmosphere." + std::to_string(static_cast<long>(altitude)) + ".";
    emit(prefix + "temperature_k", state.temperature_k);
    emit(prefix + "pressure_pa", state.pressure_pa);
    emit(prefix + "density_kg_m3", state.density_kg_m3);
    emit(prefix + "speed_of_sound_m_s", state.speed_of_sound_m_s);
    emit(prefix + "viscosity_pa_s", state.dynamic_viscosity_pa_s);
  }
  // A non-standard day exercises the offset path.
  const galata::core::AtmosphereState hot = galata::core::isa(3048.0, 15.0);
  emit("atmosphere.isa_plus_15.density_kg_m3", hot.density_kg_m3);
  emit("atmosphere.isa_plus_15.speed_of_sound_m_s", hot.speed_of_sound_m_s);
}

void fingerprint_rigid_body() {
  galata::sim::MassProperties mass;
  mass.mass_kg = 4000.0;
  mass.inertia_cg_body_kg_m2 << 12000.0, 0.0, -1500.0,  //
      0.0, 40000.0, 0.0,                                //
      -1500.0, 0.0, 48000.0;

  galata::core::State initial;
  initial.position_ned_m = Eigen::Vector3d(0.0, 0.0, -3048.0);
  initial.velocity_body_m_s = Eigen::Vector3d(120.0, 3.0, 8.0);
  initial.attitude_body_to_ned = galata::core::quaternion_from_euler({0.3, -0.2, 1.1});
  initial.angular_rate_body_rad_s = Eigen::Vector3d(0.9, -0.4, 0.6);

  // With gravity and a constant applied wrench, so the translational and
  // rotational paths and the attitude kinematics are all exercised.
  galata::sim::Wrench wrench;
  wrench.force_body_n = Eigen::Vector3d(2500.0, -400.0, -1200.0);
  wrench.moment_cg_body_n_m = Eigen::Vector3d(800.0, -1500.0, 300.0);

  const galata::numerics::DerivativeFunction derivative =
      [&mass, &wrench](double, const Eigen::VectorXd& x) -> Eigen::VectorXd {
    return galata::sim::rigid_body_derivative(
        galata::core::State::from_vector(galata::core::StateVector(x)),
        mass,
        wrench,
        Eigen::Vector3d(0.0, 0.0, 9.80665));
  };
  const galata::numerics::ProjectionFunction project = [](Eigen::VectorXd& x) {
    galata::core::State state = galata::core::State::from_vector(galata::core::StateVector(x));
    state.renormalise_attitude();
    x = state.to_vector();
  };

  const auto trajectory = galata::numerics::integrate_fixed_step(
      derivative, initial.to_vector(), 0.0, 0.002, 15000, 15000, project);

  static const char* kNames[] = {
      "p_n", "p_e", "p_d", "u", "v", "w", "q_w", "q_x", "q_y", "q_z", "p", "q", "r"};
  const Eigen::VectorXd& final_state = trajectory.states.back();
  for (int i = 0; i < galata::core::kStateSize; ++i) {
    emit(std::string("rigid_body.30s.") + kNames[i], final_state(i));
  }

  const galata::core::State final_object =
      galata::core::State::from_vector(galata::core::StateVector(final_state));
  emit("rigid_body.30s.kinetic_energy_j",
       galata::sim::rotational_kinetic_energy(final_object, mass));
  emit("rigid_body.30s.airspeed_m_s", galata::core::airspeed(final_object.velocity_body_m_s));
  emit("rigid_body.30s.alpha_rad", galata::core::angle_of_attack(final_object.velocity_body_m_s));
  emit("rigid_body.30s.beta_rad", galata::core::sideslip_angle(final_object.velocity_body_m_s));
}

void fingerprint_modes() {
  // The NT-33A lateral matrix, as in the shipped example.
  Eigen::MatrixXd a(4, 4);
  a << -0.125, 0.0383878091, -0.9992629164, 0.1410102351,  //
      -5.49, -2.03, 0.641, 0.0,                            //
      0.667, -0.116, -0.207, 0.0,                          //
      0.0, 1.0, 0.0384161250, 0.0;
  const std::vector<std::string> names = {"beta", "p", "r", "phi"};
  const auto result =
      galata::analyze::analyze_modes(a, names, galata::analyze::StateRoles::from_names(names));

  emit("modes.lateral.condition_number", result.eigenvector_condition_number);
  for (const auto& mode : result.modes) {
    const std::string prefix = "modes.lateral." + galata::analyze::to_string(mode.label) + ".";
    emit(prefix + "real", mode.eigenvalue.real());
    emit(prefix + "imag", mode.eigenvalue.imag());
    emit(prefix + "omega_n", mode.natural_frequency_rad_s);
    emit(prefix + "zeta", mode.damping_ratio);
    emit(prefix + "label_score", mode.label_score);
    for (std::size_t i = 0; i < mode.participation.size(); ++i) {
      emit(prefix + "participation." + names[i], mode.participation[i]);
    }
  }
}

}  // namespace

int main() {
  // The build identification is deliberately NOT in the fingerprint: it names
  // the compiler, and this file is compared across compilers.
  std::printf("# galata determinism fingerprint, version %s\n",
              std::string(galata::version_string()).c_str());
  fingerprint_atmosphere();
  fingerprint_rigid_body();
  fingerprint_modes();
  return 0;
}
