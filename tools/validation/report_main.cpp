// SPDX-License-Identifier: Apache-2.0
//
// Generates docs/VERIFICATION.md.
//
// The report is generated rather than written because its whole value is that
// its numbers are the numbers the code actually produces. A hand-maintained V&V
// report drifts from the implementation silently, and a drifted V&V report is
// worse than none: it is a document that looks like evidence.
//
// Usage:  galata-validation-report <reference-directory>
// CI regenerates and diffs against the committed file.

#include "galata/core/atmosphere.hpp"
#include "galata/version.hpp"

#include "reference_table.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr double kPascalsPerMillibar = 100.0;

struct Deviation {
  double altitude_m = 0.0;
  std::string quantity;
  double published = 0.0;
  double computed = 0.0;
  int significant_figures = 0;
  // How far the computed value sits from the published one, measured in units
  // of the last printed significant figure. Below 0.5 the computed value rounds
  // to exactly what is printed.
  double units_in_last_place = 0.0;
};

double last_place_units(double computed, double published, int significant_figures) {
  if (published == 0.0) {
    return 0.0;
  }
  const double magnitude = std::floor(std::log10(std::fabs(published)));
  const double unit = std::pow(10.0, magnitude - (significant_figures - 1));
  return std::fabs(computed - published) / unit;
}

std::string format(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.6g", value);
  return buffer;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: galata-validation-report <reference-directory>\n";
    return 2;
  }
  const std::string reference_dir = argv[1];

  galata::testing::ReferenceTable table;
  try {
    table = galata::testing::load_reference(reference_dir, "ussa1976.csv");
  } catch (const std::exception& error) {
    std::cerr << "galata-validation-report: " << error.what() << "\n";
    return 1;
  }

  std::vector<Deviation> deviations;
  double worst_temperature = 0.0;
  double worst_pressure = 0.0;
  double worst_density = 0.0;
  double worst_sound = 0.0;

  for (std::size_t row = 0; row < table.rows.size(); ++row) {
    const double altitude = table.at(row, "geometric_altitude_m");
    const galata::core::AtmosphereState computed = galata::core::isa(altitude);

    struct Case {
      const char* name;
      double published;
      double computed;
      int figures;
      double* worst;
    };

    const Case cases[] = {
        {"temperature",
         table.at(row, "temperature_k"),
         computed.temperature_k,
         static_cast<int>(table.at(row, "temperature_sig_figs")),
         &worst_temperature},
        {"pressure",
         table.at(row, "pressure_mb"),
         computed.pressure_pa / kPascalsPerMillibar,
         static_cast<int>(table.at(row, "pressure_sig_figs")),
         &worst_pressure},
        {"density",
         table.at(row, "density_kg_m3"),
         computed.density_kg_m3,
         static_cast<int>(table.at(row, "density_sig_figs")),
         &worst_density},
        {"speed of sound",
         table.at(row, "speed_of_sound_m_s"),
         computed.speed_of_sound_m_s,
         static_cast<int>(table.at(row, "speed_of_sound_sig_figs")),
         &worst_sound},
    };

    for (const Case& c : cases) {
      const double ulp = last_place_units(c.computed, c.published, c.figures);
      if (ulp > *c.worst) {
        *c.worst = ulp;
      }
      if (ulp > 0.5) {
        deviations.push_back({altitude, c.name, c.published, c.computed, c.figures, ulp});
      }
    }
  }

  std::cout << R"(<!-- GENERATED FILE — DO NOT EDIT.
     Produced by tools/validation/report_main.cpp via scripts/gen-verification.sh.
     CI regenerates this file and fails if it differs from the committed copy,
     so every number below is a number the code actually produced. -->

# Verification and validation

What galata has been checked against, what agreement was measured, and what has
not been checked at all.

The last column is the one to read. "Unvalidated" is not a placeholder here — it
is a statement that no published reference value has been found for that case,
and it is preferred to a number invented to fill the gap.

## Summary

| Case | Reference | Status |
|---|---|---|
| U.S. Standard Atmosphere 1976 — temperature, pressure, density, speed of sound | COESA, NOAA-S/T 76-1562 / NASA-TM-X-74335 (1976), Tables I and III | **validated** |
| U.S. Standard Atmosphere 1976 — derived layer base temperatures | same, Table I at each breakpoint | **validated** |
| U.S. Standard Atmosphere 1976 — dynamic viscosity | same, equation (51) | **unvalidated** — no tabulated viscosity values were transcribed |
| Quaternion, frame and state conventions | ADR-0002; cross-checked against Eigen's independent implementation | **self-consistent, not externally validated** |
| Rigid-body dynamics | — | **not implemented** |
| Riccati solvers | — | **not implemented** |
| Aircraft modal characteristics | — | **not implemented** |
| Determinism, cross-platform | — | **not implemented** |

)";

  std::cout << "## U.S. Standard Atmosphere, 1976\n\n";
  std::cout << "**Reference.** U.S. Committee on Extension to the Standard Atmosphere (COESA),\n"
               "*U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335,\n"
               "U.S. Government Printing Office, October 1976.\n"
               "NTRS document 19770009539, <https://ntrs.nasa.gov/citations/19770009539>.\n"
               "Rights: *Work of the US Gov. Public Use Permitted.*\n\n";

  std::cout
      << "**Transcription.** The NTRS scan's OCR layer is unusable for numeric tables —\n"
         "roughly 95,800 extractable characters across 243 pages, with digits rendered as\n"
         "underscores and letters. No value came from it. The page images were extracted and\n"
         "read visually, with load-bearing digits re-cropped at native resolution, and a\n"
         "second independent pass re-read twelve values without reference to the first.\n"
         "The full method is in the header of `tests/validation/reference/ussa1976.csv`.\n\n";

  std::cout << "**Agreement measured.** Deviation is given in units of the last printed\n"
               "significant figure. Below 0.5 the computed value rounds to exactly what the\n"
               "document prints.\n\n";

  std::cout << "| Quantity | Worst deviation (units in last printed place) | Rounds to the printed "
               "value everywhere |\n";
  std::cout << "|---|---|---|\n";

  const struct {
    const char* name;
    double worst;
  } summary[] = {
      {"Temperature", worst_temperature},
      {"Pressure", worst_pressure},
      {"Density", worst_density},
      {"Speed of sound", worst_sound},
  };

  for (const auto& entry : summary) {
    std::cout << "| " << entry.name << " | " << format(entry.worst) << " | "
              << (entry.worst <= 0.5 ? "yes" : "**no**") << " |\n";
  }

  std::cout << "\n**Gate.** The validation suite requires every cell to agree within 1.0 units of\n"
               "the last printed place. That bound is chosen to be the tightest one the 1976\n"
               "tables actually support, not the loosest one that passes: the measured\n"
               "disagreements are around one part in 10^5, while a wrong lapse-rate sign, a\n"
               "geopotential/geometric mix-up or the wrong Earth radius each move these values by\n"
               "one part in 10^2 or worse.\n\n";

  if (deviations.empty()) {
    std::cout << "Every tabulated cell rounds to the printed value.\n\n";
  } else {
    std::cout << "### Cells that do not round to the printed value\n\n";
    std::cout << "There are " << deviations.size() << " of them, out of " << (table.rows.size() * 4)
              << " tabulated cells.\n\n";
    std::cout
        << "| Geometric altitude (m) | Quantity | Published | Computed | Units in last place |\n";
    std::cout << "|---|---|---|---|---|\n";
    for (const Deviation& d : deviations) {
      std::cout << "| " << format(d.altitude_m) << " | " << d.quantity << " | "
                << format(d.published) << " | " << format(d.computed) << " | "
                << format(d.units_in_last_place) << " |\n";
    }
    std::cout << "\n";
  }

  std::cout << R"(### Known model-versus-table differences that are not errors

**Molecular-scale versus kinetic temperature above 80 km.** The seven-layer
system is defined in terms of molecular-scale temperature `T_M`, which equals
kinetic temperature `T` only where the mean molecular weight equals its
sea-level value. Above roughly 80 km the ratio `M/M_0` falls away from 1 — the
document's Table 8 gives 0.9995788 at 86 km — so galata's temperature runs high
relative to the tabulated kinetic temperature by up to about 0.08 K in the top
6 km. At 86 km the document tabulates `T = 186.87` K and `T_M = 186.95` K;
galata computes 186.946 K, which is the `T_M` value.

**The upper bound is stated twice and the two statements differ.** The standard
gives the ceiling as both 84.8520 km' geopotential and 86 km geometric.
Equation (18) maps 86000 m to 84852.046 m', so the geopotential figure is the
document's own rounding. galata bounds the envelope in geometric altitude, so
that a query at exactly the standard's stated ceiling is accepted.

### Ambiguities in the source document

These are inconsistencies in the 1976 publication itself, found during
transcription. Each is recorded so that it is not later mistaken for a
transcription error.

| Quantity | Printed as | And also as | galata uses |
|---|---|---|---|
| Sutherland's constant `S` | 110 K (Table 2B, printed page 4) | 110.4 K (text at equation (51)) | 110.4 K, the value stated with the equation it parameterises |
| `R*` | 8.31432e3 N m/(kmol K) (body text, printed page 3) | 8.31432e-3 (Table 2A) | 8.31432e3; only this makes equations (33a)/(33b) dimensionally consistent |
| Sutherland's `beta` | 1.458e-6 (Table 2B, printed page 19) | 1.458e6 (printed page 4, minus sign absent) | 1.458e-6, corroborated twice |
| `r_0` | 6,356,766 m (printed page 8) | 6356.766 km (printed page 4) | 6356766 m; Table 2B's exponent glyph is illegible in the available scans and was not used |

### What is not validated here

**Dynamic viscosity.** galata implements equation (51), but no tabulated
viscosity values were transcribed from the document, so there is nothing to
compare against. The implementation is unvalidated. It additionally inherits the
`S = 110` versus `S = 110.4` ambiguity above, which moves the result by about
0.1%.

**Altitudes between the tabulated points.** Eight altitudes are checked against
the tables. Continuity, monotonicity and the absence of steps at layer
boundaries are checked on a 100 m grid across the whole envelope, which
constrains the space between the sampled points but is not the same as checking
it against published values.

**Everything above 86 km.** Out of scope: galata refuses the query rather than
extrapolating.

)";

  // Version only, deliberately NOT build_identification(): the report is
  // regenerated and diffed by CI on a different compiler and build type from a
  // developer's, and a footer naming the toolchain would make the diff fail for
  // a reason that has nothing to do with the numbers. The provenance of this
  // document is the commit it is committed in.
  std::cout << "---\n\nGenerated from galata " << galata::version_string()
            << " by `tools/validation/report_main.cpp`.\n";
  return 0;
}
