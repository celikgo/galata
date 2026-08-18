// SPDX-License-Identifier: Apache-2.0

#include "case_registry.hpp"

namespace galata::validation {
namespace {

using E = Evidence;
constexpr Binary kUnit = Binary::Unit;
constexpr Binary kProperty = Binary::Property;
constexpr Binary kValidation = Binary::Validation;
constexpr Binary kDeterminism = Binary::Determinism;

const char* const kUssa =
    "COESA, *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335";
const char* const kCr2144 =
    "Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972)";
const char* const kSeiler =
    "Seiler, Packard & Gahinet, *An Introduction to Disk Margins*, IEEE CSM 40(5) (2020)";
const char* const kMathWorks =
    "MathWorks, *Stability Analysis Using Disk Margins*, Robust Control Toolbox documentation";
const char* const kEuler =
    "Closed-form solutions of Euler's equations (Goldstein; Landau & Lifshitz)";

}  // namespace

std::string to_string(Status status) {
  switch (status) {
    case Status::Validated:
      return "**validated**";
    case Status::ValidatedWithCaveat:
      return "**validated**, with a caveat";
    case Status::KnownDiscrepancy:
      return "**known discrepancy**";
    case Status::Unvalidated:
      return "unvalidated";
    case Status::SelfConsistent:
      return "self-consistent, not externally validated";
    case Status::NotImplemented:
      return "not implemented";
  }
  return "unknown";
}

bool claims_a_comparison(Status status) {
  return status == Status::Validated || status == Status::ValidatedWithCaveat
         || status == Status::KnownDiscrepancy;
}

std::string to_string(Binary binary) {
  switch (binary) {
    case Binary::Unit:
      return "unit";
    case Binary::Property:
      return "property";
    case Binary::Integration:
      return "integration";
    case Binary::Validation:
      return "validation";
    case Binary::Determinism:
      return "determinism";
  }
  return "unknown";
}

const std::vector<Case>& validation_cases() {
  static const std::vector<Case> cases = {
      // --- Atmosphere ------------------------------------------------------
      {"atmosphere.tables",
       "U.S. Standard Atmosphere 1976 — temperature, pressure, density, speed of sound",
       {},
       std::string(kUssa) + ", Tables I and III",
       Status::Validated,
       {E{kValidation, "Ussa1976.TemperatureMatchesThePublishedTable"},
        E{kValidation, "Ussa1976.PressureMatchesThePublishedTable"},
        E{kValidation, "Ussa1976.DensityMatchesThePublishedTable"},
        E{kValidation, "Ussa1976.SpeedOfSoundMatchesThePublishedTable"}},
       "Temperature and speed of sound round to the printed value everywhere; pressure and "
       "density do not, at {atm.deviating} of {atm.total} cells, by at most {atm.worst_ulp} units "
       "in the last printed place. "
       "Every deviation is listed below."},

      {"atmosphere.layer_recurrence",
       "U.S. Standard Atmosphere 1976 — derived layer base temperatures and the pressure "
       "recurrence",
       {},
       std::string(kUssa) + ", Table I at each breakpoint",
       Status::Validated,
       {E{kValidation, "Ussa1976.DerivedBaseTemperaturesMatchTheStandardsOwnTabulation"},
        E{kValidation, "Ussa1976.PressureAtTheTopOfTheModelMatchesAfterSevenLayers"}},
       "Table 4 has no base-temperature column, so these are derived rather than transcribed. "
       "The pressure recurrence is checked at the top of the seven-layer chain, where any "
       "per-layer error would have accumulated."},

      {"atmosphere.viscosity",
       "U.S. Standard Atmosphere 1976 — dynamic viscosity",
       {},
       std::string(kUssa) + ", equation (51)",
       Status::Unvalidated,
       {},
       "Implemented, but no tabulated viscosity values were transcribed, so there is nothing "
       "to compare against. It also inherits the source's own S = 110 K versus 110.4 K "
       "ambiguity, worth {atm.viscosity_s_percent}."},

      // --- Core conventions -------------------------------------------------
      {"core.conventions",
       "Quaternion, frame and state conventions",
       {},
       "ADR-0002, cross-checked against Eigen's independent implementation",
       Status::SelfConsistent,
       {E{kUnit, "Quaternion.DcmMatchesEigensOwnRotationMatrix"},
        E{kProperty, "QuaternionProperties.HandWrittenDcmAlwaysAgreesWithEigen"}},
       "The rotation matrix is written out by hand from ADR-0002 and compared against Eigen "
       "over the whole rotation group. That checks the documented convention against the "
       "implemented one; it is not a comparison against a document."},

      // --- Numerics ---------------------------------------------------------
      {"numerics.jacobian",
       "Numerical Jacobians against analytically known Jacobians",
       {},
       "Charter validation case 4; analytic derivatives of closed-form functions",
       Status::Validated,
       {E{kUnit, "Jacobian.QuadraticFunctionMatchesItsAnalyticJacobian"},
        E{kUnit, "Jacobian.LinearFunctionIsDifferentiatedToTheCancellationLimit"},
        E{kUnit, "Jacobian.TruncationEstimateBoundsTheActualError"}},
       "Agreement to the cancellation limit, eps |f| / h, which is the floor a central "
       "difference has even on a linear function. The Richardson estimate is checked to bound "
       "the actual error rather than understate it."},

      {"numerics.rk4_order",
       "Fixed-step RK4 — method order",
       {},
       "Hairer, Norsett & Wanner (1993); exact solutions of closed-form problems",
       Status::Validated,
       {E{kUnit, "Rk4.IntegratesCubicsInTimeExactly"},
        E{kUnit, "Rk4.IsFourthOrderOnTheExponential"},
        E{kUnit, "Rk4.StepSizeStudyRecoversTheMethodOrder"}},
       "Exact on cubics, as Simpson's rule must be; error falls by 16 per halving on the "
       "exponential."},

      {"numerics.newton",
       "Newton's method — convergence and failure reporting",
       {},
       "Nocedal & Wright (2006); systems with closed-form roots",
       Status::Validated,
       {E{kUnit, "Newton.SolvesALinearSystemInOneStep"},
        E{kUnit, "Newton.ConvergesQuadraticallyOnASmoothNonlinearSystem"},
        E{kUnit, "Newton.ReportsFailureRatherThanReturningAWrongRoot"}},
       "Quadratic convergence on a smooth system; a system with no real root is reported as "
       "unconverged rather than returned as a least-bad point."},

      // --- Rigid body -------------------------------------------------------
      {"rigid_body.precession",
       "Torque-free precession of a symmetric top",
       {},
       kEuler,
       Status::Validated,
       {E{kValidation,
          "Shapes/SymmetricTop.ConvergesToTheClosedFormPrecessionAtFourthOrder/oblate"},
        E{kValidation,
          "Shapes/SymmetricTop.ConvergesToTheClosedFormPrecessionAtFourthOrder/prolate"}},
       "Checked for fourth-order CONVERGENCE to the closed form, not proximity to it. A "
       "solution converging to the wrong closed form sits at a small constant error and passes "
       "an absolute check."},

      {"rigid_body.intermediate_axis",
       "Intermediate-axis instability (the Dzhanibekov effect)",
       {},
       kEuler,
       Status::Validated,
       {E{kValidation, "IntermediateAxis.PerturbationFollowsTheClosedFormHyperbolicGrowth"},
        E{kValidation, "IntermediateAxis.RotationAboutTheMajorAndMinorAxesIsStable"}},
       "Asserted against the cosh/sinh closed form pointwise, including the sign the "
       "(I2 - I3) < 0 factor forces. Fitting a log-slope instead measures {cosh.slope_factor} "
       "sigma "
       "and looks "
       "like a defect in the dynamics."},

      {"rigid_body.conservation",
       "Energy and angular-momentum conservation, general inertia tensor",
       {},
       "Exact invariants of torque-free motion",
       Status::Validated,
       {E{kValidation, "TorqueFreeConservation.EnergyAndAngularMomentumDriftIsBounded"},
        E{kValidation, "TorqueFreeConservation.AngularMomentumRotatesInBodyAxesButNotInNed"}},
       "The angular-momentum figure is the VECTOR resolved in NED, not its body-axis "
       "magnitude. A transposed direction-cosine matrix conserves the magnitude and fails "
       "this. Drift measured below."},

      {"rigid_body.aerodynamic_forces",
       "Six-degree-of-freedom equations with aerodynamic forces",
       {},
       std::string(kCr2144) + ", Tables II-1 and II-7",
       Status::ValidatedWithCaveat,
       {E{kValidation, "Nt33aChain.LateralDimensionalDerivativesMatchThePublishedTable"}},
       "Validated INDIRECTLY: the linearised derivatives that match Table II-7 to "
       "{deriv.worst_percent} run "
       "through these equations, the coefficient buildup and the wind-to-body rotation. There "
       "is no case comparing the equations in isolation."},

      {"sim.nonlinear_loop",
       "Nonlinear simulation with aerodynamic forces, over time",
       {},
       "",
       Status::NotImplemented,
       {},
       "There is a state derivative, not a loop flying an aircraft through time."},

      // --- Aircraft, from a hand-assembled matrix ---------------------------
      {"nt33a.lateral_modes_hand",
       "Aircraft lateral modes from a hand-assembled matrix — spiral, roll subsidence, Dutch "
       "roll",
       {"analyze.modes"},
       std::string(kCr2144) + ", Table II-8",
       Status::Validated,
       {E{kValidation,
          "Nt33aHandAssembled.LateralModesMatchThePublishedValuesWithinTheSourcesOwnPrecision"},
        E{kValidation, "Nt33aHandAssembled.TheDutchRollPeriodAgreesWithThePublishedPeriod"}},
       "Tolerance measured, not chosen: each input is perturbed by half a unit in its own last "
       "printed digit and the published value's own rounding is added."},

      {"nt33a.longitudinal_modes_hand",
       "Aircraft longitudinal modes from a hand-assembled matrix — phugoid frequency, "
       "short-period frequency and damping",
       {"analyze.modes"},
       std::string(kCr2144) + ", Table II-4",
       Status::Validated,
       {E{kValidation,
          "Nt33aHandAssembled."
          "LongitudinalModesMatchThePublishedValuesWithinTheSourcesOwnPrecision"}},
       "Three of the four longitudinal quantities. The fourth is the row below."},

      {"nt33a.phugoid_damping_hand",
       "Aircraft longitudinal modes — phugoid DAMPING RATIO, from a hand-assembled matrix",
       {"analyze.modes"},
       std::string(kCr2144) + ", Table II-4",
       Status::KnownDiscrepancy,
       {E{kValidation, "Nt33aHandAssembled.PhugoidDampingDiscrepancyDoesNotGrow"}},
       "{hand.phugoid_zeta} against a published 0.0948, out by {hand.phugoid_percent}, about three "
       "times what the inputs' rounding allows. "
       "Localised to the hand assembly — the full chain reproduces it. Held by a labelled "
       "regression lock; see below."},

      {"analyze.classification",
       "Modal classification into the five classical modes",
       {"analyze.modes"},
       std::string(kCr2144) + ", labels checked against the report's own identification",
       Status::Validated,
       {E{kValidation, "Nt33aHandAssembled.AllThreeLateralModesAreFoundAndCorrectlyLabelled"},
        E{kValidation, "Nt33aHandAssembled.BothLongitudinalModesAreFoundAndCorrectlyLabelled"},
        E{kValidation, "Nt33aChain.ModesAreLabelledCorrectlyFromParticipationAlone"},
        E{kUnit, "Modes.ClassifiesLongitudinalModesByParticipationNotByFrequency"}},
       "By eigenvector participation, not by frequency. A unit test builds a system whose "
       "phugoid block is deliberately faster than its short-period block; a frequency-based "
       "classifier gets both labels backwards on it."},

      // --- Aircraft, the full chain ----------------------------------------
      {"nt33a.trim",
       "Trim of a nonlinear model against the published flight condition",
       {"model.aircraft.derivatives", "trim.level"},
       std::string(kCr2144) + ", Table II-2",
       Status::Validated,
       {E{kValidation, "Nt33aChain.TrimConvergesToMachinePrecision"},
        E{kValidation, "Nt33aChain.TrimSatisfiesTheClosedFormForceBalanceExactly"},
        E{kValidation, "Nt33aChain.DynamicPressureAndMachMatchThePublishedFlightCondition"},
        E{kValidation,
          "Nt33aChain.TrimAlphaDiffersFromThePublishedValueByExactlyTheDragInclinationTerm"}},
       "Dynamic pressure {trim.psf} psf against a published {trim.published_psf}; Mach {trim.mach} "
       "against {trim.published_mach}. The trimmed alpha is {trim.alpha_shift_deg} deg below the "
       "published {trim.published_alpha_deg}, and a test asserts that difference "
       "is exactly the drag-inclination term the conventional C_L = W/(qS) relation neglects."},

      {"nt33a.linearised_derivatives",
       "Linearised dimensional derivatives from a nonlinear model",
       {"model.aircraft.derivatives", "trim.level", "linearize.finitediff"},
       std::string(kCr2144) + ", Table II-7",
       Status::Validated,
       {E{kValidation, "Nt33aChain.LateralDimensionalDerivativesMatchThePublishedTable"},
        E{kValidation, "Nt33aChain.TruncationErrorIsNegligible"},
        E{kValidation, "Nt33aChain.TheLongitudinalAndLateralAxesDecoupleAtThisTrim"}},
       "Seven numbers the report computed from the same non-dimensional set by a different "
       "route, reproduced to {deriv.worst_percent}. The sharpest comparison in the suite."},

      {"nt33a.chain_modes",
       "All five classical modes from trim and linearisation of a nonlinear model",
       {"model.aircraft.derivatives", "trim.level", "linearize.finitediff", "analyze.modes"},
       std::string(kCr2144) + ", Tables II-4 and II-8",
       Status::Validated,
       {E{kValidation, "Nt33aChain.AllFiveClassicalModesMatchThePublishedValues"},
        E{kValidation, "Nt33aChain.ThePhugoidDampingThatTheHandAssembledMatrixMissedIsRecovered"}},
       "To {modes.worst_percent}, worst case {modes.worst_name}. The input is a non-dimensional "
       "derivative "
       "set and some geometry; there is no matrix anywhere in it."},

      // --- Determinism ------------------------------------------------------
      {"determinism.tier1",
       "Determinism tier 1 — same platform, byte-identical",
       {},
       "ADR-0004",
       Status::Validated,
       {E{kDeterminism, "Determinism.LongIntegrationIsBitIdenticalAcrossRuns"},
        E{kDeterminism, "Determinism.SplittingAnIntegrationInTwoGivesTheSameResult"},
        E{kDeterminism, "Determinism.ModalDecompositionIsBitIdenticalAndOrderStable"},
        E{kDeterminism, "Determinism.AtmosphereDoesNotDependOnQueryOrder"}},
       "Gated on Linux, macOS and Windows over {det.total} fingerprinted values. The strongest of "
       "these is splitting: 4000 steps must equal 1500 then 2500, bit for bit."},

      {"determinism.tier2",
       "Determinism tier 2 — cross-platform, bounded",
       {},
       "ADR-0004",
       Status::ValidatedWithCaveat,
       {E{kDeterminism, "Determinism.TheFingerprintTrajectoryIsNotChaotic"}},
       "Bounded at 1e-9 relative between every pair of platforms, not bit-identical, because "
       "platform math libraries disagree on sin in the last bits. Values downstream of a "
       "finite difference are excluded from this tier and held byte-identical in tier 1 "
       "instead — {det.tier1_only} of the {det.total} values — because dividing by h amplifies a "
       "libm disagreement by 1/h."},

      // --- Frequency response and margins -----------------------------------
      {"analyze.freqresp",
       "Frequency response G(jw) against closed-form transfer functions",
       {"analyze.freqresp"},
       "Closed-form evaluation of rational transfer functions at s = jw",
       Status::Validated,
       {E{kUnit, "FrequencyResponse.FirstOrderLagMatchesItsClosedForm"},
        E{kUnit, "FrequencyResponse.SecondOrderResonantPeakMatchesItsClosedForm"},
        E{kUnit, "FrequencyResponse.RationalTransferFunctionWithZerosMatchesItsRatio"},
        E{kUnit, "FrequencyResponse.PhaseIsUnwrappedAcrossTheHalfTurnBoundary"}},
       "The reference is arithmetic, not a document: for a system whose transfer function can be "
       "written down, G(jw) is a ratio of polynomials and the comparison is exact to rounding."},

      {"analyze.freqresp.hessenberg",
       "The hand-written Hessenberg solver against a general LU on the unreduced matrix",
       {"analyze.freqresp"},
       "Laub, *Efficient multivariable frequency response computations*, IEEE TAC 26(2) (1981)",
       Status::Validated,
       {E{kUnit, "FrequencyResponse.HessenbergSolveAgreesWithADirectlyFormedSolve"}},
       "Two different eliminations of the same system over a grid reaching a condition number "
       "above 1e6. The gate is kappa * eps — the conditioning of the problem — not a chosen "
       "tolerance."},

      {"analyze.margins",
       "Gain, phase and delay margins against loops whose margins are exact",
       {"analyze.margins"},
       "Franklin, Powell & Emami-Naeini, *Feedback Control of Dynamic Systems*; "
       "Astrom & Murray, *Feedback Systems*, ch. 10",
       Status::Validated,
       {E{kUnit, "Margins.ThirdOrderIntegratorChainMatchesItsClosedFormMargins"},
        E{kUnit, "Margins.RepeatedPoleChainMatchesItsClosedFormMargins"},
        E{kUnit, "Margins.PureDelayLoopIsMeasuredThroughTheEvaluator"},
        E{kUnit, "Margins.GainMarginBelowUnityMeansTheGainMustComeDown"},
        E{kValidation,
          "DiskMarginSeiler2020.ClassicalMarginsMatchThePublishedValuesAndTheClosedForm"}},
       "1/(s(s+1)(s+2)) has gain margin exactly 6 at exactly sqrt(2) rad/s, and 1/(s(s+1)^2) "
       "exactly 2 at exactly 1 rad/s. The delay margin is checked by PROPERTY as well as by "
       "formula: applying the reported delay must land the loop on the critical point."},

      {"analyze.diskmargin",
       "Disk margin — robustness to simultaneous gain and phase variation",
       {"analyze.diskmargin"},
       std::string(kSeiler) + ", worked example `ex:edm`",
       Status::Validated,
       {E{kValidation, "DiskMarginSeiler2020.SymmetricDiskMarginMatchesThePublishedValues"},
        E{kValidation,
          "DiskMarginSeiler2020.TheConstructedPerturbationActuallyDestabilisesTheLoop"},
        E{kValidation, "DiskMarginSeiler2020.NamedSkewsHaveThePublishedInterceptClosedForms"},
        E{kValidation,
          "DiskMarginSeiler2020.GainInterceptsFollowTheDiskParameterisationAtEverySkew"},
        E{kUnit, "DiskMargin.ConstructedPerturbationDestabilisesWhateverTheSkew"}},
       "Eight published values reproduced. The strongest evidence is not a value at all: the "
       "perturbation the theorem constructs must actually destabilise the loop, placing a "
       "closed-loop pole on the imaginary axis at the critical frequency. See ADR-0007 for why "
       "values from a copyrighted paper may be committed."},

      {"analyze.diskmargin.phase",
       "Disk margin — the guaranteed PHASE variation phi_m",
       {"analyze.diskmargin"},
       kMathWorks,
       Status::ValidatedWithCaveat,
       {E{kValidation,
          "DiskMarginSeiler2020.AgreesWithASecondImplementationIncludingThePhaseMargin"}},
       "Against VENDOR DOCUMENTATION, not a peer-reviewed source, and marked as such. The paper "
       "derives phi_m but prints no number for it, so without a second source this output would "
       "be gated against nothing. galata computes {disk.phi_m} degrees and MathWorks' published "
       "diskmargin output for the same loop agrees to every figure it prints — see "
       "tests/validation/reference/seiler2020_disk_margin.csv, which carries that value and its "
       "location. A second implementation agreeing is real evidence; it is not a published "
       "derivation."},

      {"analyze.diskmargin.critical_frequency",
       "Disk margin — the critical frequency, against the paper's printed value",
       {"analyze.diskmargin"},
       std::string(kSeiler) + ", worked example `ex:edm`",
       Status::KnownDiscrepancy,
       {E{kValidation,
          "DiskMarginSeiler2020."
          "PublishedCriticalFrequencyDisagreesWithItsOwnPublishedPerturbation"}},
       "The discrepancy is in the SOURCE, not in galata. The paper prints omega_0 = 1.94 rad/s, "
       "but its own printed delta_0 and f_0 are evaluated at omega_0 and are reproduced only near "
       "{disk.omega0} rad/s, which is where |S - 1/2| actually peaks. At the printed 1.94 the "
       "construction gives Re delta_0 = {disk.delta_real_at_printed} against a printed 0.212, out "
       "by {disk.omega0_units_off} units in its last printed figure. galata reports the "
       "self-consistent value. A test asserts the inconsistency, so that a future resolution of "
       "it fails loudly rather than passing quietly."},

      // --- Not implemented --------------------------------------------------
      {"synth.riccati",
       "Riccati solvers against the CAREX and DAREX benchmark collections",
       {},
       "",
       Status::NotImplemented,
       {},
       "Named in the v0.2 milestone."},

  };
  return cases;
}

}  // namespace galata::validation
