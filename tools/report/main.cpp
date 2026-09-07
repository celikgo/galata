// SPDX-License-Identifier: Apache-2.0
//
// Emits one whole flight-condition study as JSON on stdout.
//
//   galata-report-data [model.yaml] [airspeed_m_s] [altitude_m] > run.json
//
// This is tools/social/ generalised. That tool emits the poles the social
// preview card is drawn from, for one reason: a picture whose numbers were
// typed by hand asserts that galata computes them without ever having computed
// them. The same argument applies with more force to a REPORT, which is the
// artefact a flight-dynamics engineer would actually read — so the report page
// is drawn from this file, and this file is a run.
//
// It is the same chain the validation tier gates and the shipped study runs:
// load the nonlinear aircraft, trim it, linearise longitudinally and laterally
// by central differences, classify the modes by eigenvector participation, and
// then close ONE loop and measure it. Nothing here re-implements any of that;
// every number below comes back from the library routine named beside it in the
// output, so the page can say which routine produced each figure rather than
// asking the reader to take the number on trust (charter rule 9).
//
// THE LOOP IS NOT THE AIRCRAFT. Margins are a property of a loop, and an
// aircraft on its own is not one. The bank-angle loop measured here is the
// smallest control law that makes the question meaningful — measure bank angle,
// multiply by a gain, drive the aileron — and that GAIN IS A CHOICE, not a
// published value. It is 0.5 rad/rad, the same gain as
// examples/nt33a-bank-loop-margins/, chosen there because at that gain the
// magnitude crosses unity more than once and the loop therefore has more than
// one phase margin. The output records the gain and where the loop was broken
// so the page can say so too.
//
// WHY THE BUILD IDENTIFICATION IS NOT IN THIS FILE. It names the compiler, the
// build type and the platform, and this file is generated on one machine and
// checked on another (scripts/gen-report.sh --check). A field that differs by
// construction across platforms would make that gate fail for a reason having
// nothing to do with the numbers. The version is emitted instead, and the build
// identification stays where it can be trusted — in the CLI's own output.
//
// SIX SIGNIFICANT FIGURES, for the reason tools/social/main.cpp gives at
// length: these values are downstream of a central difference, ADR-0004
// excludes such values from the cross-platform bounded tier because dividing by
// h amplifies a libm disagreement by 1/h, and six figures is far inside the
// observed spread. scripts/gen-report.sh --check compares this file
// NUMERICALLY, with a stated tolerance, rather than as text.
//
// THE PUBLISHED VALUES ARE READ, NOT TYPED. The comparison strip on the page
// comes from tests/validation/reference/nt33a_fc1.csv, through the same loader
// the validation tier uses (tools/validation/reference_table.hpp), so the page
// and the gate are reading one file through one parser. A page that carried its
// own copy of the published numbers would be a second answer to the question
// the test already answers.
//
// It emits the deviation and DOES NOT emit a verdict. The budget for these
// quantities is not a constant: it is the published value's own rounding plus
// the rounding of the ten inputs it was computed from, derived per quantity in
// tests/validation/test_nt33a_modes.cpp and published in docs/VERIFICATION.md.
// Re-deriving it here would be a second gate, and two gates are two answers.
//
// Consumed by scripts/gen-report-page.py.

#include "galata/analyze/disk_margin.hpp"
#include "galata/analyze/frequency_response.hpp"
#include "galata/analyze/margins.hpp"
#include "galata/analyze/modes.hpp"
#include "galata/analyze/sensitivity.hpp"
#include "galata/linearize/finite_difference.hpp"
#include "galata/model/aircraft.hpp"
#include "galata/model/linear_system.hpp"
#include "galata/trim/level.hpp"
#include "galata/units.hpp"
#include "galata/version.hpp"

#include "reference_table.hpp"
#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

// The published true airspeed for flight condition 1, in feet per second, from
// NASA CR-2144 Table II-2. The one input the model file does not carry, because
// the model is the aircraft and this is the condition.
constexpr double kTrueAirspeedFtS = 228.0;

// Bank-angle feedback gain, rad of aileron per rad of bank error. NOT from any
// document — see the header. It matches examples/nt33a-bank-loop-margins/.
constexpr double kBankGain = 0.5;

// The band the loop is measured over, and the point count. Four decades either
// side of the aircraft's own modes, which run from the spiral at 0.032 rad/s to
// the roll subsidence at 2.2 rad/s.
constexpr double kLoopFromRadS = 0.01;
constexpr double kLoopToRadS = 100.0;
constexpr int kLoopPoints = 400;

std::string escape(const std::string& text) {
  std::string out;
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(c);
    } else if (c == '\n') {
      out += "\\n";
    } else if (static_cast<unsigned char>(c) < 0x20) {
      out.push_back(' ');
    } else {
      out.push_back(c);
    }
  }
  return out;
}

// A quantity that is not defined is emitted as JSON null, never as 0 and never
// as NaN. A real root has no period; a converging mode has no time to double.
// Printing zero there would invite a plot to draw it, and NaN is not JSON.
void number(double value) {
  if (std::isnan(value)) {
    std::printf("null");
  } else if (std::isinf(value)) {
    // Infinite is a real answer here — an unbounded gain range, an absent phase
    // crossover — and it is emitted as a string so the page can say "infinite"
    // rather than inventing a large finite number.
    std::printf("\"%sinfinite\"", value < 0.0 ? "-" : "");
  } else {
    std::printf("%.6g", value);
  }
}

void number_array(const std::vector<double>& values) {
  std::printf("[");
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      std::printf(", ");
    }
    number(values[i]);
  }
  std::printf("]");
}

void string_array(const std::vector<std::string>& values) {
  std::printf("[");
  for (std::size_t i = 0; i < values.size(); ++i) {
    std::printf("%s\"%s\"", i == 0 ? "" : ", ", escape(values[i]).c_str());
  }
  std::printf("]");
}

void matrix(const Eigen::MatrixXd& m, const char* indent) {
  std::printf("[\n");
  for (Eigen::Index i = 0; i < m.rows(); ++i) {
    std::printf("%s  [", indent);
    for (Eigen::Index j = 0; j < m.cols(); ++j) {
      if (j != 0) {
        std::printf(", ");
      }
      number(m(i, j));
    }
    std::printf("]%s\n", i + 1 == m.rows() ? "" : ",");
  }
  std::printf("%s]", indent);
}

void emit_modes(const galata::analyze::ModalDecomposition& decomposition, const char* indent) {
  std::printf("[\n");
  for (std::size_t i = 0; i < decomposition.modes.size(); ++i) {
    const galata::analyze::Mode& mode = decomposition.modes[i];
    std::printf("%s  {\"label\": \"%s\", \"re\": ",
                indent,
                escape(galata::analyze::to_string(mode.label)).c_str());
    number(mode.eigenvalue.real());
    std::printf(", \"im\": ");
    number(mode.eigenvalue.imag());
    std::printf(", \"oscillatory\": %s, \"omega_n\": ", mode.is_oscillatory ? "true" : "false");
    number(mode.natural_frequency_rad_s);
    std::printf(", \"zeta\": ");
    number(mode.damping_ratio);
    std::printf(", \"period_s\": ");
    number(mode.period_s);
    std::printf(", \"time_to_half_s\": ");
    number(mode.time_to_half_amplitude_s);
    std::printf(", \"time_to_double_s\": ");
    number(mode.time_to_double_amplitude_s);
    std::printf(", \"time_constant_s\": ");
    number(mode.time_constant_s);
    std::printf(", \"label_score\": ");
    number(mode.label_score);
    std::printf(", \"label_reason\": \"%s\", \"participation\": ",
                escape(mode.label_reason).c_str());
    number_array(mode.participation);
    std::printf("}%s\n", i + 1 == decomposition.modes.size() ? "" : ",");
  }
  std::printf("%s]", indent);
}

// The published modal characteristics, matched to the computed modes by label.
//
// `printed_precision` is half a unit in the last of the source's three printed
// significant figures — the width of the band the published number itself
// carries. It is emitted so the page can show the reader what "0.35%" is being
// measured against, and it comes from the validation tier's own function rather
// than from a formula retyped here.
struct PublishedMode {
  const char* csv_key;
  const char* quantity;
  galata::analyze::ModeLabel label;
  bool is_damping_ratio;  // false means the natural frequency, or a real root
};

constexpr PublishedMode kPublished[] = {
    {"phugoid_damping_ratio", "zeta", galata::analyze::ModeLabel::Phugoid, true},
    {"phugoid_natural_frequency", "omega_n", galata::analyze::ModeLabel::Phugoid, false},
    {"short_period_damping_ratio", "zeta", galata::analyze::ModeLabel::ShortPeriod, true},
    {"short_period_natural_frequency", "omega_n", galata::analyze::ModeLabel::ShortPeriod, false},
    {"dutch_roll_damping_ratio", "zeta", galata::analyze::ModeLabel::DutchRoll, true},
    {"dutch_roll_natural_frequency", "omega_n", galata::analyze::ModeLabel::DutchRoll, false},
    {"roll_subsidence_root", "1/T", galata::analyze::ModeLabel::RollSubsidence, false},
    {"spiral_root", "1/T", galata::analyze::ModeLabel::Spiral, false},
};

// The source prints every quantity in this reference set to three significant
// figures; tests/validation/test_nt33a_modes.cpp says the same in one line.
constexpr int kPrintedFigures = 3;

void emit_published_comparison(const std::map<std::string, double>& published,
                               const galata::analyze::ModalDecomposition& longitudinal,
                               const galata::analyze::ModalDecomposition& lateral) {
  std::printf("  \"published\": [\n");
  bool first = true;
  for (const PublishedMode& entry : kPublished) {
    const galata::analyze::Mode* mode = longitudinal.find(entry.label);
    if (mode == nullptr) {
      mode = lateral.find(entry.label);
    }
    if (mode == nullptr) {
      std::fprintf(stderr,
                   "galata-report-data: the chain produced no '%s' mode, so the published "
                   "comparison for %s cannot be made\n",
                   galata::analyze::to_string(entry.label).c_str(),
                   entry.csv_key);
      std::exit(1);
    }
    const auto found = published.find(entry.csv_key);
    if (found == published.end()) {
      std::fprintf(stderr, "galata-report-data: no '%s' in the reference table\n", entry.csv_key);
      std::exit(1);
    }
    const double reference = found->second;
    // The report prints a real root as the FACTOR (s + 1/T), so a printed
    // +0.0318 is a convergent root at -0.0318. galata reports the eigenvalue.
    // Comparing |1/T| against |Re(lambda)| compares like with like, and the
    // sign is checked separately by
    // Nt33aHandAssembled.TheSpiralIsConvergentAsThePublishedSignImplies.
    const double computed = entry.is_damping_ratio
                                ? mode->damping_ratio
                                : (mode->is_oscillatory ? mode->natural_frequency_rad_s
                                                        : std::fabs(mode->eigenvalue.real()));

    std::printf("%s    {\"mode\": \"%s\", \"quantity\": \"%s\", \"source\": \"%s\", ",
                first ? "" : ",\n",
                escape(galata::analyze::to_string(entry.label)).c_str(),
                entry.quantity,
                entry.csv_key);
    first = false;
    std::printf("\"published\": ");
    number(reference);
    std::printf(", \"computed\": ");
    number(computed);
    std::printf(", \"deviation_percent\": ");
    number(100.0 * std::fabs(computed - reference) / std::fabs(reference));
    std::printf(", \"printed_precision\": ");
    number(galata::testing::printed_precision_tolerance(reference, kPrintedFigures));
    std::printf("}");
  }
  std::printf("\n  ],\n");
}

void emit_axis(const char* axis,
               const char* selection_note,
               const galata::linearize::Linearisation& linearisation,
               const galata::analyze::ModalDecomposition& decomposition) {
  std::printf("    {\n");
  std::printf("      \"axis\": \"%s\",\n", axis);
  std::printf("      \"selection\": \"%s\",\n", escape(selection_note).c_str());
  std::printf("      \"routine\": \"galata::linearize::linearize_finite_difference\",\n");
  std::printf("      \"modes_routine\": \"galata::analyze::analyze_modes\",\n");
  std::printf("      \"states\": ");
  string_array(linearisation.state_names);
  std::printf(",\n      \"inputs\": ");
  string_array(linearisation.input_names);
  std::printf(",\n      \"a\": ");
  matrix(linearisation.a, "      ");
  std::printf(",\n      \"b\": ");
  matrix(linearisation.b, "      ");
  std::printf(",\n      \"a_truncation\": ");
  matrix(linearisation.a_truncation, "      ");
  std::printf(",\n      \"worst_relative_truncation\": ");
  number(linearisation.worst_relative_truncation);
  std::printf(",\n      \"neglected_coupling\": ");
  number(linearisation.neglected_coupling);
  std::printf(",\n      \"chart_conditioning\": ");
  number(linearisation.chart_conditioning);
  std::printf(",\n      \"eigenvector_condition_number\": ");
  number(decomposition.eigenvector_condition_number);
  std::printf(",\n      \"participation_is_meaningful\": %s",
              decomposition.participation_is_meaningful ? "true" : "false");
  std::printf(",\n      \"modes\": ");
  emit_modes(decomposition, "      ");
  std::printf("\n    }");
}

// The bank-angle loop, assembled from the chain's OWN lateral linearisation.
//
// Not read from a file: if it were, the page would be showing margins for a
// matrix somebody typed rather than for the aircraft the pole map above it
// draws. A and B come from the linearisation; C is the control law and D is
// zero. The loop is broken at the PLANT INPUT, between the gain and the
// aileron — breaking it at the output gives a different transfer function and
// different margins for the same closed-loop system, and that is a modelling
// decision the tool cannot make (see include/galata/analyze/margins.hpp).
galata::model::LinearSystem bank_angle_loop(const galata::linearize::Linearisation& lateral,
                                            int aileron_index,
                                            int bank_index) {
  galata::model::LinearSystem loop;
  loop.a = lateral.a;
  loop.b = lateral.b.col(aileron_index);
  loop.c = Eigen::MatrixXd::Zero(1, lateral.a.cols());
  loop.c(0, bank_index) = kBankGain;
  loop.d = Eigen::MatrixXd::Zero(1, 1);
  loop.state_names = lateral.state_names;
  loop.input_names = {lateral.input_names.at(static_cast<std::size_t>(aileron_index))};
  loop.output_names = {"bank_feedback"};
  loop.description = "proportional bank-angle loop, broken at the aileron";
  loop.units = "aileron rad in, bank-angle feedback rad out";
  loop.validate();
  return loop;
}

int index_of(const std::vector<std::string>& names, const std::string& wanted) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == wanted) {
      return static_cast<int>(i);
    }
  }
  std::fprintf(stderr, "galata-report-data: no '%s' among the model's names\n", wanted.c_str());
  std::exit(1);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string model_path = (argc > 1) ? argv[1] : std::string(GALATA_CASE_MODEL);
  const double airspeed_m_s =
      (argc > 2) ? std::atof(argv[2]) : galata::units::feet_to_metres(kTrueAirspeedFtS);
  const double altitude_m = (argc > 3) ? std::atof(argv[3]) : 0.0;
  const std::string reference_dir = (argc > 4) ? argv[4] : std::string(GALATA_CASE_REFERENCE_DIR);

  try {
    const galata::model::Aircraft aircraft = galata::model::load_aircraft(model_path);
    const galata::testing::ReferenceTable reference =
        galata::testing::load_reference(reference_dir, "nt33a_fc1.csv");
    const std::map<std::string, double> published = reference.as_lookup("quantity", "value");

    galata::trim::LevelTrimRequest request;
    request.altitude_m = altitude_m;
    request.airspeed_m_s = airspeed_m_s;
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

    const galata::model::LinearSystem loop = bank_angle_loop(
        lateral, index_of(lateral.input_names, "aileron"), index_of(lateral.state_names, "phi"));

    galata::analyze::MarginOptions margin_options;
    margin_options.start_rad_s = kLoopFromRadS;
    margin_options.stop_rad_s = kLoopToRadS;
    margin_options.grid_points = kLoopPoints;

    const std::vector<double> grid =
        galata::analyze::grid_refined_for_modes(loop.a, kLoopFromRadS, kLoopToRadS, kLoopPoints);
    const galata::analyze::FrequencyResponse response =
        galata::analyze::single_loop_response(loop, 0, 0, grid);
    const galata::analyze::StabilityMargins margins =
        galata::analyze::stability_margins(loop, 0, 0, margin_options);
    const galata::analyze::DiskMargin disk =
        galata::analyze::disk_margin(loop, 0, 0, 0.0, margin_options);
    const galata::analyze::SensitivityPeaks peaks =
        galata::analyze::sensitivity_peaks(loop, margin_options);
    // Sampled peaks cannot establish upper-norm evidence. Keep absent bounds
    // explicit for report consumers; numerical bounds use a separate API.
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    const galata::analyze::GuaranteedMargins guaranteed{
        false, false, unavailable, unavailable, unavailable, unavailable};

    // ---------------------------------------------------------------------
    std::printf("{\n");
    std::printf("  \"generator\": \"galata-report-data\",\n");
    std::printf("  \"version\": \"%s\",\n", std::string(galata::version_string()).c_str());
    std::printf("  \"model\": {\"description\": \"%s\", \"citation\": \"%s\"},\n",
                escape(aircraft.description).c_str(),
                escape(aircraft.citation).c_str());

    std::printf("  \"trim\": {\"routine\": \"galata::trim::trim_level\", \"altitude_m\": ");
    number(altitude_m);
    std::printf(", \"airspeed_m_s\": ");
    number(trim.airspeed_m_s);
    std::printf(", \"mach\": ");
    number(trim.mach);
    std::printf(", \"dynamic_pressure_pa\": ");
    number(trim.dynamic_pressure_pa);
    std::printf(",\n            \"alpha_deg\": ");
    number(galata::units::radians_to_degrees(trim.alpha_rad));
    std::printf(", \"flight_path_angle_deg\": ");
    number(galata::units::radians_to_degrees(trim.flight_path_angle_rad));
    std::printf(", \"pitch_attitude_deg\": ");
    number(galata::units::radians_to_degrees(trim.pitch_attitude_rad));
    std::printf(",\n            \"elevator_deg\": ");
    number(galata::units::radians_to_degrees(trim.controls.elevator_rad));
    std::printf(", \"thrust_n\": ");
    number(trim.controls.thrust_n);
    std::printf(", \"lift_coefficient\": ");
    number(trim.lift_coefficient);
    std::printf(",\n            \"residual_norm\": ");
    number(trim.residual_norm);
    std::printf(", \"jacobian_condition_number\": ");
    number(trim.jacobian_condition_number);
    std::printf(", \"outside_advisory_envelope\": %s",
                trim.envelope.outside_advisory_envelope ? "true" : "false");
    std::printf("},\n");

    std::printf("  \"axes\": [\n");
    emit_axis("longitudinal", "u, w, q, theta", longitudinal, longitudinal_modes);
    std::printf(",\n");
    emit_axis("lateral", "v, p, r, phi", lateral, lateral_modes);
    std::printf("\n  ],\n");

    emit_published_comparison(published, longitudinal_modes, lateral_modes);

    // ---------------------------------------------------------------------
    std::printf("  \"loop\": {\n");
    std::printf("    \"description\": \"%s\",\n", escape(loop.description).c_str());
    std::printf("    \"gain_rad_per_rad\": ");
    number(kBankGain);
    std::printf(",\n    \"broken_at\": \"plant input, between the gain and the aileron\",\n");
    std::printf("    \"gain_provenance\": \"chosen for this example, not from a document\",\n");
    std::printf("    \"input\": \"%s\", \"output\": \"%s\",\n",
                escape(loop.input_names.front()).c_str(),
                escape(loop.output_names.front()).c_str());
    std::printf("    \"freqresp_routine\": \"galata::analyze::single_loop_response\",\n");
    std::printf("    \"margins_routine\": \"galata::analyze::stability_margins\",\n");
    std::printf("    \"disk_routine\": \"galata::analyze::disk_margin\",\n");
    std::printf("    \"sensitivity_routine\": \"galata::analyze::sensitivity_peaks\",\n");

    std::printf("    \"frequencies_rad_s\": ");
    number_array(response.frequencies_rad_s);
    std::printf(",\n    \"magnitude_db\": ");
    number_array(response.magnitude_db());

    const std::vector<double> phase_rad = response.phase_rad();
    std::vector<double> phase_deg(phase_rad.size());
    std::vector<double> real_part(phase_rad.size());
    std::vector<double> imag_part(phase_rad.size());
    for (std::size_t i = 0; i < phase_rad.size(); ++i) {
      phase_deg[i] = galata::units::radians_to_degrees(phase_rad[i]);
      const std::complex<double> value = response.response[i](0, 0);
      real_part[i] = value.real();
      imag_part[i] = value.imag();
    }
    std::printf(",\n    \"phase_deg\": ");
    number_array(phase_deg);
    std::printf(",\n    \"nyquist_re\": ");
    number_array(real_part);
    std::printf(",\n    \"nyquist_im\": ");
    number_array(imag_part);
    std::printf(",\n    \"pivot_ratio_min\": ");
    number(*std::min_element(response.pivot_ratio.begin(), response.pivot_ratio.end()));

    // --- margins ----------------------------------------------------------
    std::printf(
        ",\n    \"margins\": {\"nominal_stability_checked\": %s, "
        "\"nominal_closed_loop_stable\": %s, ",
        margins.nominal_stability_checked ? "true" : "false",
        margins.nominal_closed_loop_stable ? "true" : "false");
    std::printf("\"has_gain_margin\": %s, \"gain_margin\": ",
                margins.has_gain_margin ? "true" : "false");
    number(margins.gain_margin);
    std::printf(", \"gain_margin_db\": ");
    number(margins.gain_margin_db);
    std::printf(", \"gain_margin_frequency_rad_s\": ");
    number(margins.has_gain_margin ? margins.gain_margin_frequency_rad_s
                                   : std::numeric_limits<double>::quiet_NaN());
    std::printf(",\n                 \"has_phase_margin\": %s, \"phase_margin_deg\": ",
                margins.has_phase_margin ? "true" : "false");
    number(galata::units::radians_to_degrees(margins.phase_margin_rad));
    std::printf(", \"phase_margin_frequency_rad_s\": ");
    number(margins.has_phase_margin ? margins.phase_margin_frequency_rad_s
                                    : std::numeric_limits<double>::quiet_NaN());
    std::printf(",\n                 \"has_delay_margin\": %s, \"delay_margin_s\": ",
                margins.has_delay_margin ? "true" : "false");
    number(margins.has_delay_margin ? margins.delay_margin_s
                                    : std::numeric_limits<double>::quiet_NaN());
    std::printf(", \"delay_margin_frequency_rad_s\": ");
    number(margins.has_delay_margin ? margins.delay_margin_frequency_rad_s
                                    : std::numeric_limits<double>::quiet_NaN());
    std::printf(",\n                 \"searched_from_rad_s\": ");
    number(margins.searched_from_rad_s);
    std::printf(", \"searched_to_rad_s\": ");
    number(margins.searched_to_rad_s);
    std::printf(", \"grid_points\": %d", margins.grid_points);

    std::printf(",\n                 \"gain_crossings\": [");
    for (std::size_t i = 0; i < margins.gain_crossings.size(); ++i) {
      const galata::analyze::GainCrossing& crossing = margins.gain_crossings[i];
      std::printf("%s{\"frequency_rad_s\": ", i == 0 ? "" : ", ");
      number(crossing.frequency_rad_s);
      std::printf(", \"phase_margin_deg\": ");
      number(galata::units::radians_to_degrees(crossing.phase_margin_rad));
      std::printf(", \"delay_margin_s\": ");
      number(crossing.delay_margin_s);
      std::printf("}");
    }
    std::printf("],\n                 \"phase_crossings\": [");
    for (std::size_t i = 0; i < margins.phase_crossings.size(); ++i) {
      const galata::analyze::PhaseCrossing& crossing = margins.phase_crossings[i];
      std::printf("%s{\"frequency_rad_s\": ", i == 0 ? "" : ", ");
      number(crossing.frequency_rad_s);
      std::printf(", \"gain_margin\": ");
      number(crossing.gain_margin);
      std::printf(", \"gain_margin_db\": ");
      number(crossing.gain_margin_db);
      std::printf("}");
    }
    std::printf("]},\n");

    // --- disk margin ------------------------------------------------------
    std::printf("    \"disk\": {\"skew\": ");
    number(disk.skew);
    std::printf(", \"alpha\": ");
    number(disk.alpha);
    std::printf(", \"peak_gain\": ");
    number(disk.peak_gain);
    std::printf(", \"critical_frequency_rad_s\": ");
    number(disk.critical_frequency_rad_s);
    std::printf(",\n             \"gain_variation_min\": ");
    number(disk.gain_variation_min);
    std::printf(", \"gain_variation_max\": ");
    number(disk.gain_variation_is_bounded ? disk.gain_variation_max
                                          : std::numeric_limits<double>::infinity());
    std::printf(", \"gain_variation_is_bounded\": %s",
                disk.gain_variation_is_bounded ? "true" : "false");
    std::printf(",\n             \"gain_variation_min_db\": ");
    number(disk.gain_variation_min_db);
    std::printf(", \"gain_variation_max_db\": ");
    number(disk.gain_variation_is_bounded ? disk.gain_variation_max_db
                                          : std::numeric_limits<double>::infinity());
    std::printf(",\n             \"phase_variation_deg\": ");
    number(disk.phase_variation_is_bounded
               ? galata::units::radians_to_degrees(disk.phase_variation_rad)
               : std::numeric_limits<double>::infinity());
    std::printf(", \"phase_variation_is_bounded\": %s",
                disk.phase_variation_is_bounded ? "true" : "false");
    std::printf(",\n             \"destabilising_perturbation_re\": ");
    number(disk.destabilising_perturbation.real());
    std::printf(", \"destabilising_perturbation_im\": ");
    number(disk.destabilising_perturbation.imag());
    std::printf(",\n             \"searched_from_rad_s\": ");
    number(disk.searched_from_rad_s);
    std::printf(", \"searched_to_rad_s\": ");
    number(disk.searched_to_rad_s);
    std::printf(", \"grid_points\": %d},\n", disk.grid_points);

    // --- sensitivity ------------------------------------------------------
    std::printf("    \"sensitivity\": {\"m_s\": ");
    number(peaks.sensitivity_peak);
    std::printf(", \"m_s_frequency_rad_s\": ");
    number(peaks.sensitivity_peak_frequency_rad_s);
    std::printf(", \"m_t\": ");
    number(peaks.complementary_peak);
    std::printf(", \"m_t_frequency_rad_s\": ");
    number(peaks.complementary_peak_frequency_rad_s);
    std::printf(",\n                    \"guaranteed_applies\": %s, \"guaranteed_valid\": %s",
                guaranteed.applies ? "true" : "false",
                guaranteed.valid ? "true" : "false");
    std::printf(",\n                    \"gain_margin_from_m_s\": ");
    number(guaranteed.gain_margin_from_sensitivity);
    std::printf(", \"gain_margin_from_m_t\": ");
    number(guaranteed.gain_margin_from_complementary);
    std::printf(",\n                    \"phase_margin_from_m_s_deg\": ");
    number(galata::units::radians_to_degrees(guaranteed.phase_margin_from_sensitivity_rad));
    std::printf(", \"phase_margin_from_m_t_deg\": ");
    number(galata::units::radians_to_degrees(guaranteed.phase_margin_from_complementary_rad));
    std::printf("}\n");

    std::printf("  }\n}\n");
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "galata-report-data: %s\n", error.what());
    return 1;
  }
}
