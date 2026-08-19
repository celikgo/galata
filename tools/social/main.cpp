// SPDX-License-Identifier: Apache-2.0
//
// Emits the NT-33A flight-condition-1 pole map as JSON on stdout.
//
//   galata-modal-map > poles.json
//
// This exists so that docs/assets/social-preview.png — the card GitHub renders
// wherever this repository is linked — is DRAWN FROM A RUN rather than from
// numbers typed into a drawing script. The picture claims that galata computes
// these poles and labels them by eigenvector participation; a picture whose
// numbers were transcribed by hand would be making that claim without evidence,
// and would go stale the first time the model changed.
//
// It is the same chain as the shipped study and the same chain the validation
// tier gates: load the aircraft, trim it, linearise longitudinally and
// laterally by central differences, and classify the modes by participation.
// The labels below are NOT assigned here — they come back from
// analyze::analyze_modes, which is the whole point of the picture.
//
// Values are emitted to SIX significant figures, not to full precision, and
// that is a determinism decision rather than a formatting one. These numbers
// are downstream of a central difference, which ADR-0004 excludes from the
// cross-platform bounded tier precisely because dividing by h amplifies a
// libm disagreement by 1/h. Six figures is three orders inside the observed
// spread and about two more than the picture can draw, so the committed file
// is stable across platforms — and scripts/gen-modal-map.sh --check compares
// it NUMERICALLY, with a stated tolerance, rather than as text.
//
// Consumed by scripts/gen-social-preview.py.

#include "galata/analyze/modes.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"
#include "galata/version.hpp"

#include <cstdio>
#include <exception>
#include <string>
#include <vector>

namespace {

// The published true airspeed for flight condition 1, in feet per second, from
// NASA CR-2144 Table II-2. The one input this tool needs that the model file
// does not carry, because the model is the aircraft and this is the condition.
constexpr double kTrueAirspeedFtS = 228.0;

std::string escape(const std::string& text) {
  std::string out;
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

void emit_modes(const galata::analyze::ModalDecomposition& decomposition,
                const char* axis,
                bool& first) {
  for (const galata::analyze::Mode& mode : decomposition.modes) {
    if (!first) {
      std::printf(",\n");
    }
    first = false;
    std::printf(
        "    {\"label\": \"%s\", \"axis\": \"%s\", \"re\": %.6g, \"im\": %.6g, "
        "\"omega_n\": %.6g, \"zeta\": %.6g, \"oscillatory\": %s, "
        "\"label_score\": %.6g, \"label_reason\": \"%s\"}",
        escape(galata::analyze::to_string(mode.label)).c_str(),
        axis,
        mode.eigenvalue.real(),
        mode.eigenvalue.imag(),
        mode.natural_frequency_rad_s,
        mode.damping_ratio,
        mode.is_oscillatory ? "true" : "false",
        mode.label_score,
        escape(mode.label_reason).c_str());
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string model_path = (argc > 1) ? argv[1] : std::string(GALATA_SOCIAL_MODEL);

  try {
    const galata::model::Aircraft aircraft = galata::model::load_aircraft(model_path);

    galata::trim::LevelTrimRequest request;
    request.altitude_m = 0.0;
    request.airspeed_m_s = galata::units::feet_to_metres(kTrueAirspeedFtS);
    const galata::trim::TrimPoint trim = galata::trim::trim_level(aircraft, request);

    galata::linearize::LinearisationOptions longitudinal_options;
    longitudinal_options.state_subset = galata::linearize::longitudinal_states();
    const auto longitudinal =
        galata::linearize::linearize_finite_difference(aircraft, trim, longitudinal_options);

    galata::linearize::LinearisationOptions lateral_options;
    lateral_options.state_subset = galata::linearize::lateral_states();
    const auto lateral =
        galata::linearize::linearize_finite_difference(aircraft, trim, lateral_options);

    using galata::analyze::analyze_modes;
    using galata::analyze::StateRoles;
    const auto longitudinal_modes = analyze_modes(
        longitudinal.a, longitudinal.state_names, StateRoles::from_names(longitudinal.state_names));
    const auto lateral_modes =
        analyze_modes(lateral.a, lateral.state_names, StateRoles::from_names(lateral.state_names));

    std::printf("{\n");
    std::printf("  \"generator\": \"galata-modal-map\",\n");
    std::printf("  \"version\": \"%s\",\n", std::string(galata::version_string()).c_str());
    std::printf("  \"model\": \"%s\",\n", escape(aircraft.description).c_str());
    std::printf(
        "  \"condition\": {\"altitude_m\": %.6g, \"airspeed_m_s\": %.6g, "
        "\"mach\": %.6g, \"alpha_deg\": %.6g, \"residual_norm\": %.6g},\n",
        request.altitude_m,
        trim.airspeed_m_s,
        trim.mach,
        galata::units::radians_to_degrees(trim.alpha_rad),
        trim.residual_norm);
    std::printf("  \"modes\": [\n");
    bool first = true;
    emit_modes(longitudinal_modes, "longitudinal", first);
    emit_modes(lateral_modes, "lateral", first);
    std::printf("\n  ]\n}\n");
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "galata-modal-map: %s\n", error.what());
    return 1;
  }
}
