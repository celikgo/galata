// SPDX-License-Identifier: Apache-2.0
//
// U.S. Standard Atmosphere, 1976.
//
// Reference (primary, transcribed from the scanned original — see
// tests/validation/reference/ussa1976.csv for the transcription and its audit
// trail):
//   U.S. Committee on Extension to the Standard Atmosphere (COESA),
//   "U.S. Standard Atmosphere, 1976", NOAA / NASA / USAF,
//   NOAA-S/T 76-1562, NASA-TM-X-74335, U.S. Government Printing Office,
//   Washington, D.C., October 1976.
//   NTRS document 19770009539, https://ntrs.nasa.gov/citations/19770009539
//   Rights: "Work of the US Gov. Public Use Permitted."
//
// Equation numbers cited in the implementation are that document's.
//
// ===========================================================================
// WHAT THIS IS NOT
// ===========================================================================
// * Not a weather model, and not a climatology. It is a single idealised
//   vertical profile intended to represent mid-latitude annual-mean conditions
//   at moderate solar activity. Real atmospheres depart from it constantly. A
//   tropical day at sea level runs 15-20 K warm; a polar winter stratosphere
//   runs 30 K cold. Performance computed against ISA is nominal performance,
//   not the performance you will measure.
//
// * Not valid above 86 km geometric. The seven-layer system this implements is
//   defined only to 84.8520 km' geopotential, which is 86 km geometric.
//   Queries above that are rejected rather than extrapolated.
//
// * Not exactly the tabulated KINETIC temperature above 80 km. The seven-layer
//   system is defined in terms of MOLECULAR-SCALE temperature T_M, which
//   equals kinetic temperature T only where the mean molecular weight equals
//   its sea-level value. Above about 80 km M/M_0 falls away from 1 — the
//   document's Table 8 gives 0.9995788 at 86 km — so the T this returns is
//   high relative to the tabulated kinetic T by up to about 0.08 K in the top
//   6 km of the range. That is a property of the model, not an implementation
//   error, and docs/VERIFICATION.md records it as such.
//
// * Not a source of humidity, wind, or any composition information. The
//   standard treats the air below 86 km as a dry, homogeneously mixed ideal
//   gas of fixed composition.
//
// * The dynamic viscosity carries a documented ambiguity in the source itself:
//   the standard prints Sutherland's constant S as 110 K in Table 2B and on
//   printed page 4, and as 110.4 K in the text accompanying equation (51).
//   galata uses 110.4 K, the value stated with the equation it appears in.
//   See kSutherlandConstant.

#ifndef GALATA_CORE_ATMOSPHERE_HPP
#define GALATA_CORE_ATMOSPHERE_HPP

namespace galata::core {

// Defining constants, Table 2 of the standard unless noted. Every one is a
// transcription from the primary document.
namespace ussa1976 {

// Category II defining constants.
inline constexpr double kSeaLevelPressure = 101325.0;   // Pa, P_0
inline constexpr double kSeaLevelTemperature = 288.15;  // K, T_0 = T_M,0
inline constexpr double kStandardGravity = 9.80665;     // m/s^2, g_0
// g_0' relates the standard geopotential metre to geometric height. It is
// numerically equal to g_0 but dimensionally distinct, m^2/(s^2 m').
inline constexpr double kGeopotentialGravity = 9.80665;  // m^2/(s^2 m')

// R*, the universal gas constant as the standard defines it. Printed as
// 8.31432e3 N m/(kmol K) in the body text on printed page 3. Table 2A prints
// the exponent with a spurious minus sign; that is a typographical error in the
// source, corroborated by the fact that only 10^3 makes equations (33a) and
// (33b) dimensionally consistent.
inline constexpr double kUniversalGasConstant = 8314.32;  // N m/(kmol K)

inline constexpr double kMeanMolecularWeight = 28.9644;  // kg/kmol, M_0

// Effective radius of the Earth used for the geopotential/geometric relation,
// equations (18) and (19). Taken from the two legible body-text statements
// (6,356,766 m on printed page 8; 6356.766 km on printed page 4) rather than
// from Table 2B, whose exponent glyph is degraded in the available scans.
inline constexpr double kEarthRadius = 6356766.0;  // m

inline constexpr double kRatioOfSpecificHeats = 1.40;  // dimensionless, exact

// Sutherland's law constants, equation (51).
inline constexpr double kSutherlandBeta = 1.458e-6;  // kg/(s m K^1/2)
// The source is internally inconsistent here: 110 K in Table 2B and on printed
// page 4, 110.4 K in the text at equation (51). 110.4 is used because it is the
// value stated alongside the equation it parameterises. The difference moves
// viscosity by about 0.1% at sea level, which matters for no galata result
// today and is recorded so that it is not mistaken for a transcription error.
inline constexpr double kSutherlandConstant = 110.4;  // K

// Specific gas constant of sea-level air, R* / M_0. Derived, not defined.
inline constexpr double kSpecificGasConstant =
    kUniversalGasConstant / kMeanMolecularWeight;  // J/(kg K)

// Upper bound of the seven-layer system.
//
// The standard states this bound twice, as 84.8520 km' geopotential and as
// 86 km geometric, and treats them as the same altitude. They are not exactly
// the same number: equation (18) maps 86000 m to 84852.0458 m', because
// 84.8520 km' is the standard's own rounding of that quantity to four decimal
// places of a kilometre.
//
// galata bounds the envelope in GEOMETRIC altitude. Bounding it in geopotential
// instead would reject a query at exactly 86000 m — the standard's own stated
// ceiling — by 4.6 centimetres of geopotential, which is a floating-point cliff
// standing where no physical boundary is.
inline constexpr double kMaxGeometricAltitude = 86000.0;  // m

// The same bound as the standard prints it in geopotential. Retained for
// documentation and for the validation suite; the envelope check uses the
// geometric bound above.
inline constexpr double kMaxGeopotentialAltitude = 84852.0;  // m'

// Lower bound. The standard tabulates down to -5000 m; below that the layer
// system is an extrapolation of the troposphere and galata does not offer it.
inline constexpr double kMinGeometricAltitude = -5000.0;  // m

}  // namespace ussa1976

// The atmospheric state at a point. Every field is SI (ADR-0003).
struct AtmosphereState {
  double temperature_k = 0.0;            // K   — molecular-scale; see header
  double pressure_pa = 0.0;              // Pa
  double density_kg_m3 = 0.0;            // kg/m^3
  double speed_of_sound_m_s = 0.0;       // m/s
  double dynamic_viscosity_pa_s = 0.0;   // Pa s
  double geometric_altitude_m = 0.0;     // m   — the query altitude
  double geopotential_altitude_m = 0.0;  // m'  — the derived geopotential
  // The temperature offset that was applied, so a result carries the
  // non-standard condition it was computed under rather than looking standard.
  double delta_isa_k = 0.0;  // K
};

// Equation (18). H = (g_0/g_0') r_0 Z / (r_0 + Z).
[[nodiscard]] double geopotential_from_geometric(double geometric_altitude_m) noexcept;

// Equation (19). Z = r_0 H / (Gamma r_0 - H). Exact inverse of the above.
[[nodiscard]] double geometric_from_geopotential(double geopotential_altitude_m) noexcept;

// True when the altitude lies within the model's validity envelope,
// -5000 m to 86000 m geometric. Callers that cannot handle an exception check
// this first; isa() rejects out-of-range altitudes rather than extrapolating.
[[nodiscard]] bool is_within_envelope(double geometric_altitude_m) noexcept;

// The standard atmosphere at a geometric altitude.
//
// delta_isa_k offsets the TEMPERATURE profile while leaving the pressure
// profile standard, which is the aviation convention: an ISA+15 day at a given
// pressure altitude is hotter and less dense, but the pressure that defines
// that altitude is unchanged. Density and speed of sound therefore both respond
// to the offset, through the ideal gas law and through a = sqrt(gamma R T).
//
// Throws std::out_of_range if the altitude is outside the envelope. It throws
// rather than clamping because a silently clamped atmosphere produces a
// confidently wrong density, and a confidently wrong density produces a
// confidently wrong trim.
[[nodiscard]] AtmosphereState isa(double geometric_altitude_m, double delta_isa_k = 0.0);

}  // namespace galata::core

#endif  // GALATA_CORE_ATMOSPHERE_HPP
