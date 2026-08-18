// SPDX-License-Identifier: Apache-2.0
//
// U.S. Standard Atmosphere, 1976 — the seven-layer system, surface to 86 km.
//
// Reference:
//   U.S. Committee on Extension to the Standard Atmosphere (COESA),
//   "U.S. Standard Atmosphere, 1976", NOAA-S/T 76-1562 / NASA-TM-X-74335,
//   U.S. Government Printing Office, October 1976.
//   NTRS 19770009539, https://ntrs.nasa.gov/citations/19770009539
//
// Equations (18), (19), (23), (33a), (33b), (42), (50) and (51) are that
// document's numbering. The validity envelope and known error directions are in
// the header's "WHAT THIS IS NOT" block.

#include "galata/core/atmosphere.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace galata::core {
namespace {

using namespace galata::core::ussa1976;

// Table 4 of the standard: eight geopotential breakpoints, SEVEN gradients.
// The b = 7 entry at 84.8520 km' is a terminator, not a layer — the standard
// says so explicitly ("the set of eight values H_b", "the set of seven values
// L_M,b"). Getting that off-by-one wrong invents an eighth layer.
struct Layer {
  double base_geopotential_m;  // m',  H_b
  double lapse_rate_k_per_m;   // K/m', L_M,b (the standard prints K/km')
};

constexpr int kLayerCount = 7;

constexpr Layer kLayers[kLayerCount] = {
    {0.0, -6.5e-3},      // b=0  troposphere
    {11000.0, 0.0},      // b=1  tropopause
    {20000.0, 1.0e-3},   // b=2  stratosphere, lower
    {32000.0, 2.8e-3},   // b=3  stratosphere, upper
    {47000.0, 0.0},      // b=4  stratopause
    {51000.0, -2.8e-3},  // b=5  mesosphere, lower
    {71000.0, -2.0e-3},  // b=6  mesosphere, upper
};

// Base temperature of layer b.
//
// Derived by successive application of equation (23) from T_M,0, NOT read from
// a table: the standard's Table 4 has no base-temperature column, and its own
// note says the gradients "plus T_0 ... completely specify the geopotential-
// height profile". Deriving them is therefore faithful to the standard rather
// than merely convenient, and the validation suite checks the derived values
// against the standard's own tabulation.
double base_temperature(int layer) noexcept {
  double temperature = kSeaLevelTemperature;
  for (int b = 0; b < layer; ++b) {
    temperature += kLayers[b].lapse_rate_k_per_m
                   * (kLayers[b + 1].base_geopotential_m - kLayers[b].base_geopotential_m);
  }
  return temperature;
}

// Pressure at the base of layer b.
//
// Walked from P_0 on every call rather than cached in a static table. Seven
// iterations is nothing, and it keeps the operation sequence identical for
// every query, which is what ADR-0004's bit-identity tier needs — a lazily
// initialised static would make the first call's result depend on which thread
// got there first.
double base_pressure(int layer) noexcept {
  double pressure = kSeaLevelPressure;
  for (int b = 0; b < layer; ++b) {
    const double base_t = base_temperature(b);
    const double delta_h = kLayers[b + 1].base_geopotential_m - kLayers[b].base_geopotential_m;
    const double lapse = kLayers[b].lapse_rate_k_per_m;

    if (lapse == 0.0) {
      // Equation (33b), the isothermal case.
      pressure *= std::exp(-kGeopotentialGravity * kMeanMolecularWeight * delta_h
                           / (kUniversalGasConstant * base_t));
    } else {
      // Equation (33a). Note the exponent's sign: the standard writes the ratio
      // as T_M,b / (T_M,b + L(H - H_b)), i.e. base over local, with a POSITIVE
      // exponent. Writing it as (local/base) with a negated exponent is
      // algebraically identical and is how most implementations spell it; this
      // follows the source.
      const double top_t = base_t + lapse * delta_h;
      pressure *=
          std::pow(base_t / top_t,
                   kGeopotentialGravity * kMeanMolecularWeight / (kUniversalGasConstant * lapse));
    }
  }
  return pressure;
}

// Index of the layer containing a geopotential altitude.
int layer_index(double geopotential_m) noexcept {
  int index = 0;
  for (int b = 1; b < kLayerCount; ++b) {
    if (geopotential_m >= kLayers[b].base_geopotential_m) {
      index = b;
    }
  }
  return index;
}

}  // namespace

double geopotential_from_geometric(double geometric_altitude_m) noexcept {
  // Equation (18). Gamma = g_0/g_0' = 1 m'/m numerically, retained explicitly
  // so the dimensional bookkeeping is visible rather than folded away.
  const double gamma = kStandardGravity / kGeopotentialGravity;
  return gamma * kEarthRadius * geometric_altitude_m / (kEarthRadius + geometric_altitude_m);
}

double geometric_from_geopotential(double geopotential_altitude_m) noexcept {
  // Equation (19).
  const double gamma = kStandardGravity / kGeopotentialGravity;
  return kEarthRadius * geopotential_altitude_m / (gamma * kEarthRadius - geopotential_altitude_m);
}

bool is_within_envelope(double geometric_altitude_m) noexcept {
  // Written as a negated comparison so that NaN, which compares false against
  // everything, is rejected rather than sailing through.
  if (!(geometric_altitude_m >= kMinGeometricAltitude)) {
    return false;
  }
  return geometric_altitude_m <= kMaxGeometricAltitude;
}

AtmosphereState isa(double geometric_altitude_m, double delta_isa_k) {
  if (!is_within_envelope(geometric_altitude_m)) {
    throw std::out_of_range(
        "galata::core::isa: geometric altitude " + std::to_string(geometric_altitude_m) +
        " m is outside the U.S. Standard Atmosphere 1976 envelope of " +
        std::to_string(kMinGeometricAltitude) + " m to 86000 m. The seven-layer system is "
        "defined only to 84852 m geopotential; galata refuses to extrapolate it because a "
        "silently extrapolated density produces a confidently wrong trim.");
  }

  const double geopotential_m = geopotential_from_geometric(geometric_altitude_m);
  const int layer = layer_index(geopotential_m);

  // Equation (23): the molecular-scale temperature profile.
  const double standard_temperature =
      base_temperature(layer)
      + kLayers[layer].lapse_rate_k_per_m * (geopotential_m - kLayers[layer].base_geopotential_m);

  // Equations (33a) / (33b): the pressure profile. Pressure is the STANDARD
  // profile regardless of delta_isa — see the header for why.
  const double base_t = base_temperature(layer);
  const double delta_h = geopotential_m - kLayers[layer].base_geopotential_m;
  const double lapse = kLayers[layer].lapse_rate_k_per_m;

  double pressure = base_pressure(layer);
  if (lapse == 0.0) {
    pressure *= std::exp(-kGeopotentialGravity * kMeanMolecularWeight * delta_h
                         / (kUniversalGasConstant * base_t));
  } else {
    pressure *=
        std::pow(base_t / standard_temperature,
                 kGeopotentialGravity * kMeanMolecularWeight / (kUniversalGasConstant * lapse));
  }

  const double temperature = standard_temperature + delta_isa_k;

  AtmosphereState state;
  state.geometric_altitude_m = geometric_altitude_m;
  state.geopotential_altitude_m = geopotential_m;
  state.delta_isa_k = delta_isa_k;
  state.temperature_k = temperature;
  state.pressure_pa = pressure;
  // Equation (42), rearranged: rho = P M_0 / (R* T).
  state.density_kg_m3 = pressure / (kSpecificGasConstant * temperature);
  // Equation (50).
  state.speed_of_sound_m_s = std::sqrt(kRatioOfSpecificHeats * kSpecificGasConstant * temperature);
  // Equation (51), Sutherland's law.
  state.dynamic_viscosity_pa_s =
      kSutherlandBeta * temperature * std::sqrt(temperature) / (temperature + kSutherlandConstant);
  return state;
}

}  // namespace galata::core
