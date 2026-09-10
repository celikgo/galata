// SPDX-License-Identifier: Apache-2.0

#include "case_registry.hpp"

namespace galata::validation {
namespace {

using E = Evidence;
constexpr Binary kUnit = Binary::Unit;
constexpr Binary kProperty = Binary::Property;
constexpr Binary kValidation = Binary::Validation;
constexpr Binary kIntegration = Binary::Integration;
constexpr Binary kDeterminism = Binary::Determinism;

const char* const kUssa =
    "COESA, *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335";
const char* const kCr2144 =
    "Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972)";
const char* const kSeiler =
    "Seiler, Packard & Gahinet, *An Introduction to Disk Margins*, IEEE CSM 40(5) (2020)";
const char* const kMathWorks =
    "MathWorks, *Stability Analysis Using Disk Margins*, Robust Control Toolbox documentation";
const char* const kSkogestad =
    "Skogestad & Postlethwaite, *Multivariable Feedback Control*, 2nd ed. (2005)";
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

      {"quadrotor.invariants",
       "Multirotor plant against the exact invariants of its own equations",
       {},
       "Exact invariants of the multirotor equations of motion",
       Status::Validated,
       {E{kValidation, "QuadrotorHover.EqualSpeedsCarryTheWeightAndProduceNoMoment"},
        E{kValidation, "QuadrotorFreeFall.ZeroRotorSpeedGivesZeroSpecificForceAndOneGeeDown"},
        E{kValidation,
          "QuadrotorTorqueSigns.DifferentialThrustDrivesTheExpectedAxisAndOnlyThatAxis"},
        E{kValidation, "QuadrotorTorqueFree.RotorsOffAndDragOffConservesEnergyAndAngularMomentum"},
        E{kValidation,
          "QuadrotorStepRefinement.HalvingTheStepConvergesAtFourthOrderOverAManoeuvre"}},
       "Validated against mathematics rather than a document, as the torque-free case above "
       "is. These bound the equations, not the parameter set: no published source anchors "
       "the coefficients, so `model.quadrotor` is registered implemented-unvalidated and a "
       "completed run of it is evidence about the equations and never about an aircraft. The "
       "torque-signs case earns its keep on the axes that must stay SILENT — a transposed "
       "cross product leaves the driven axis looking healthy."},

      {
          "identify.recovery_under_noise",
          "Grey-box identification against a record carrying declared Gaussian noise",
          {"identify.greybox", "identify.validate"},
          "Exact invariants of the identification arithmetic, on records this repository "
          "generated; no published source and no measured data",
          Status::SelfConsistent,
          {E{kValidation,
             "IdentificationRecovery."
             "RecoversDeclaredParametersFromANoisyRecordWithinItsOwnIntervals"},
           E{kValidation,
             "IdentificationRecovery.TheReportedIntervalWidensWithTheNoiseTheRecordCarries"},
           E{kValidation,
             "IdentificationRecovery.TheSameParameterSetIsRefusedOnARecordThatCannotSeparateIt"},
           E{kValidation,
             "IdentificationRecovery."
             "AWeaklyExcitedParameterIsReportedAsRestingOnItsBoundNotRefused"},
           E{kValidation,
             "IdentificationRecovery."
             "AWrongModelIsCaughtByTheShapeOfItsHeldOutResidualNotOnlyItsSize"},
           E{kValidation,
             "IdentificationRecovery."
             "AFitFromOneWindowIsScoredOnAnotherAndTheLabelIsNotIndependence"}},
          "RFC-0002's WP4 verification, and self-consistent rather than validated because the "
          "records are galata's own output with a declared pseudo-random sequence added — the "
          "reference is the arithmetic of the estimator, not a document. THE ACCEPTANCE CRITERION "
          "IS THE FIT'S OWN REPORTED INTERVAL rather than a tolerance chosen by the test: three "
          "standard errors, fixed before any number was read, with a ceiling on the interval's "
          "width so a fit cannot widen its way to passing. A fit can be accurate and overconfident "
          "at once, and only the first case catches that. The second case is what makes the "
          "interval worth quoting at all: an estimator reporting a constant would pass the first "
          "whenever it happened to be accurate, so the interval is required to track a four-fold "
          "noise increase to within a factor of two either way. The third and fourth separate the "
          "two ways a parameter can be poorly determined, on ONE parameter set: refused outright "
          "on a purely vertical record whose body rates are identically zero, so the sensitivity "
          "column is exactly zero rather than merely small; and reported as resting on a declared "
          "bound when a rotor-lag transient makes the same coefficient barely visible. "
          "Identifiability is a property of the record and not of the model, and showing both "
          "halves is what makes the refusal informative. The fifth is wrong-model detection by the "
          "SHAPE of the held-out residual and not its size — a large residual could be a noisy "
          "sensor; a residual correlated with itself is a missing dynamic effect, and the correct "
          "model on the same window is required to leave a residual that is not. What none of this "
          "establishes: the noise is independent and Gaussian because that is what the reported "
          "covariance assumes, and real sensor error is coloured, quantised, occasionally missing "
          "and correlated with the manoeuvre. There is no aircraft, no bench run and no flight "
          "log.",
      },

      {"quadrotor.cross_implementation",
       "Multirotor plant against an independent implementation, open loop with wind",
       {"model.quadrotor"},
       "Souxmar forest-ISR programme, independent nonlinear quadrotor plant; not a published "
       "source",
       Status::SelfConsistent,
       {E{kValidation, "QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory"}},
       "Agreement between two implementations of the same equations, which is not validation "
       "and is recorded as self-consistent for that reason. The fixture is not committed: it "
       "is a dataset whose rights position is unestablished, so ADR-0007 routes it to a "
       "loader plus fetch instructions and the case states why it did not run when the path "
       "is absent. Its rotor channel carries an integration-scheme floor — the fixture "
       "samples an exact first-order lag at the RK stages where galata carries the lag as an "
       "ODE state — and that floor is attributed in the case rather than absorbed into the "
       "budget. Finding, recorded: the wind step is a discontinuity in an air-relative "
       "velocity state and not in the fixture's ground-velocity one, so the case re-bases "
       "across it; a caller that does not is silently wrong by the whole wind increment."},

      {"quadrotor.hover_trim",
       "Multirotor equilibrium — still air, crosswind, cruise, and unequal rotor speeds",
       {"trim.hover"},
       "Exact conditions of equilibrium for the multirotor equations",
       Status::Validated,
       {E{kValidation,
          "QuadrotorHoverTrim.StillAirCrosswindCruiseAndUnequalRotorsSolveToTheirDeclaredBudget"},
        E{kUnit, "HoverTrim.ATrimNeedingMoreRotorThanTheVehicleHasIsRefusedNotReturned"},
        E{kUnit, "HoverTrim.AnOverActuatedVehicleIsRefusedRatherThanAllocatedArbitrarily"}},
       "Validated against mathematics rather than a document, as the invariants case above "
       "is. The four conditions solve to their declared residual budget and the rotor speeds "
       "sit on the closed-form hover speed; the crosswind and cruise cases are the same "
       "air-relative condition with the tilt reversed, which is the sign a transposed "
       "rotation would pass every other check and fail here. Two REFUSALS carry as much "
       "weight as the answers: an over-actuated vehicle is refused because six equations and "
       "eight unknowns leave a null space a Newton solve would resolve arbitrarily, and an "
       "infeasible trim is refused rather than returned with a note, because a best effort "
       "reported as a trim gets linearised. State of charge is frozen and excluded from the "
       "residual: a powered battery has no zero-energy-derivative equilibrium, so requiring "
       "one would make every trim infeasible for a reason that has nothing to do with "
       "flight."},

      {"quadrotor.hover_linearisation",
       "Hover linearisation on a local attitude-error chart — pole structure, control gain, "
       "disturbance feedthrough, and agreement with the nonlinear plant",
       {"linearize.extended"},
       "Closed forms derived from the model's own parameters",
       Status::Validated,
       {E{kValidation,
          "QuadrotorHoverLinearisation."
          "PoleStructureIsSixIntegratorsThreeDragPairsAndFourRotorLags"},
        E{kValidation,
          "QuadrotorHoverLinearisation.CollectiveVerticalGainMatchesTheClosedFormAndActsUpward"},
        E{kValidation,
          "QuadrotorHoverLinearisation.WindColumnsCarryTheDragFeedthroughAtFixedGroundVelocity"},
        E{kValidation,
          "QuadrotorHoverLinearisation."
          "LinearAndNonlinearAgreeWithinTheSecondOrderBoundOverOneSecond"},
        E{kUnit,
          "ExtendedLinearize.TheChartIsRegularAtNinetyDegreesOfPitchWhereTheEulerChartIsNot"}},
       "Every gate is a closed form in the model's parameters — a drag coefficient over a "
       "mass, a reciprocal time constant, a thrust slope — compared against a "
       "finite-difference Jacobian that was not told the answer. The pole structure is six "
       "integrators from position and attitude, three translational and three rotational "
       "drag rates, and four rotor lags at -1/tau; the collective vertical gain is "
       "8 k_T omega_h / m and is gated on its SIGN as well as its magnitude, because a model "
       "with it inverted hovers, trims and produces a plausible pole map while climbing when "
       "commanded to descend. Two findings, recorded rather than absorbed. The translational "
       "entries carry a FIRST-order finite-difference error, not a second-order one, because "
       "the quadratic drag term is once differentiable and not twice at zero airspeed; the "
       "budget is derived from that kink and the Richardson estimate cannot see it. And the "
       "chart's coordinates are all zero at the nominal, which defeats the shared Jacobian's "
       "relative-step rule and cost the rotor-lag entries eight digits to cancellation until "
       "the step floors were derived from the state each coordinate perturbs."},

      {"quadrotor.hover_linearisation_cross_implementation",
       "Hover linearisation against an independent implementation's exported A, B, C and D",
       {"linearize.extended", "model.linear.statespace"},
       "Souxmar forest-ISR programme, independent quadrotor linearisation exported as a named "
       "state-space file; not a published source",
       Status::SelfConsistent,
       {E{kValidation, "QuadrotorHoverLinearisation.MatricesAgreeWithTheIndependentSouxmarExport"}},
       "Agreement between two implementations, which is not validation and is recorded as "
       "self-consistent for that reason. It is the only case here that could catch a shared "
       "mistake in galata's own reasoning about the chart, because the other implementation "
       "trims and linearises in ENU/FLU with its own code and reaches these conventions by an "
       "explicit similarity. It is also the check on the COORDINATE CONTRACT: the other "
       "programme names its velocity states ground-relative and reports the wind-to-specific-force "
       "drag feedthrough in D with a zero D block against its own ground-velocity outputs and "
       "zero wind columns in its position rows. Had galata taken the wind at fixed air-relative "
       "velocity, three of those blocks would be zero where this one is not and one would be "
       "nonzero where this one is zero. The budget is derived from both implementations' "
       "finite-difference error before comparing, never from their agreement, and the fixture is "
       "not committed: ADR-0007 routes it to a path plus regeneration instructions and the case "
       "states why it did not run when the path is absent. Finding, localised and published "
       "rather than absorbed: the reference's tenth output is named `altitude_down_m` and selects "
       "+1 on the NED down state, so it reports the down coordinate where galata's `Altitude` "
       "reports altitude positive up. galata does not change — absorbing a factor of -1 into a "
       "numerical budget would be absorbing a sign error — and the row is held by a two-sided "
       "check that fails if the disagreement disappears as well as if it grows."},

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
       {"sim.nonlinear"},
       "",
       Status::Unvalidated,
       {E{kIntegration,
          "DesignWorkflow.AircraftStudyProducesAStabilisingLawAndCompletedTimeHistories"}},
       "The fixed-step rigid-body loop includes continuous full-state feedback, explicit "
       "actuator limits and envelope termination. Aircraft time histories have not been "
       "compared with independently published flight or simulator data."},

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
       "{hand.phugoid_zeta} against a published 0.0948, out by {hand.phugoid_percent} — outside "
       "the "
       "envelope of the inputs' own rounding, which reaches only -1.67%. "
       "Localised to the hand assembly, and now to ONE entry of it: the M_wdot (-g sin theta) "
       "coupling that closing Appendix C's descriptor form with the whole w_dot equation "
       "manufactures, worth 98.8% of the gap. Held by a labelled regression lock; "
       "the investigation is in [the note on this discrepancy](notes/phugoid-damping.md)."},

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
       "Gated on Linux and macOS over {det.total} fingerprinted values. The strongest of "
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
        E{kUnit, "Margins.EveryGainCrossoverIsReportedAndTheWorstOneGoverns"},
        E{kUnit, "Margins.ACrossingExactlyOnAGridPointIsCountedOnce"},
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

      // --- Singular values and the sensitivity peaks -------------------------
      {"analyze.sigma",
       "Singular values of a transfer matrix against closed-form decompositions",
       {"analyze.sigma"},
       "Closed-form singular value decompositions; Skogestad & Postlethwaite, *Multivariable "
       "Feedback Control*, 2nd ed. (2005)",
       Status::Validated,
       {E{kUnit, "SingularValues.TriangularGainHasTheGoldenRatioSingularValues"},
        E{kUnit, "SingularValues.DiagonalSystemHasTheElementMagnitudesAsItsSingularValues"},
        E{kUnit, "SingularValues.SisoSingularValueIsTheMagnitude"},
        E{kUnit, "SingularValues.RankDeficiencyIsReportedAsAnInfiniteConditionNumber"}},
       "The reference is algebra rather than a document. For [[1,1],[0,1]] the singular values "
       "are the golden ratio and its reciprocal, and every element of that matrix has magnitude "
       "at most 1 while its largest gain is 1.618 — which is the reason a MIMO system needs "
       "singular values and not a grid of element-by-element Bode plots."},

      {"analyze.sensitivity",
       "Sensitivity and complementary sensitivity peaks M_S and M_T",
       {"analyze.sensitivity"},
       std::string(kSeiler) + ", Theorem `thm:edm` and its named skews",
       Status::Validated,
       {E{kUnit, "Sensitivity.PeaksAgreeWithTheDiskMarginAtTheNamedSkews"},
        E{kUnit, "Sensitivity.SisoTracesMatchTheirClosedForms"},
        E{kUnit, "Sensitivity.DiagonalMimoLoopMatchesItsClosedForm"},
        E{kUnit, "Sensitivity.SplusTIsTheIdentityAsMatrices"}},
       "The strongest evidence is a published IDENTITY between two of galata's own "
       "computations: the disk margin at skew +1 is 1/M_S and at skew -1 is 1/M_T. One route "
       "takes the peak of a scalar sensitivity, the other inverts the smallest singular value "
       "of I + L; they share nothing below the frequency response, and they agree to "
       "{sigma.st_ulps} "
       "relative."},

      {"analyze.sigma.grid_bound",
       "The reported peak gain is a lower bound on the H-infinity norm, not equal to it",
       {"analyze.sigma", "analyze.sensitivity", "analyze.diskmargin"},
       "Boyd & Balakrishnan (1990); Bruinsma & Steinbuch (1990) — the exact computation galata "
       "does NOT use",
       Status::ValidatedWithCaveat,
       {E{kUnit, "SingularValues.ThePeakIsALowerBoundOnTheTrueNorm"},
        E{kUnit, "Sensitivity.TheReportedPeakIsALowerBoundOnTheTrueOne"}},
       "Every peak in this library is found on a refined frequency grid rather than by the "
       "exact Hamiltonian-eigenvalue method, so it UNDERSTATES the true supremum. For a "
       "robustness margin that error is optimistic. The tests demonstrate the shortfall rather "
       "than hiding it: 1/(s+1) has an H-infinity norm of exactly 1 attained at zero frequency, "
       "which no logarithmic grid contains, and the reported peak approaches it from below as "
       "the sweep widens."},

      {"analyze.sensitivity.bounds",
       "Classical margins guaranteed by M_S and M_T",
       {"analyze.sensitivity"},
       std::string(kSkogestad) + ", equations (2.47), (2.48) and (2.50), pp. 35-37",
       Status::Validated,
       {E{kValidation, "SkogestadSensitivityBounds.WorkedValuesMatchTheBook"},
        E{kValidation, "SkogestadSensitivityBounds.TheBoundsActuallyBoundRealLoops"},
        E{kValidation,
          "SkogestadSensitivityBounds.SensitivityAndComplementaryAgreeAtTheGainCrossover"},
        E{kValidation,
          "SkogestadSensitivityBounds.MsIsTheReciprocalOfTheDistanceToTheCriticalPoint"},
        E{kValidation, "SkogestadSensitivityBounds.TheBoundsAreRefusedForAMimoLoop"}},
       "The book's own worked values reproduced — M_S = 2 guarantees GM >= 2 and PM >= 29.0 "
       "degrees — but the stronger check is that the inequalities BOUND real loops: across four "
       "loop gains, the measured margins are at least what the peaks promise. Equation (2.50), "
       "an exact identity, ties three separate parts of galata together at the gain crossover. "
       "The bounds are SISO only, which is the source's own scope and not a hedge, and galata "
       "refuses them for a MIMO loop."},

      {"synth.care.worked",
       "Continuous-time Riccati solutions against SLICOT worked examples",
       {"synth.care"},
       "SLICOT BB01AD (CAREX 2.3) and SB02MD Program Data and Program Results",
       Status::ValidatedWithCaveat,
       {E{kValidation, "CareSlicot.MatchesPublishedWorkedSolutionsWithinPrintedPrecision"}},
       "The dense stabilising solver agrees with the printed solution matrices within half "
       "a unit of their last decimal place. These are small worked examples, including one "
       "CAREX parameter value; this is not validation against the full benchmark collection."},
      {"synth.lqr.aircraft",
       "Aircraft full-state-feedback design against published controller gains",
       {"synth.lqr"},
       "",
       Status::Unvalidated,
       {E{kIntegration,
          "DesignWorkflow.AircraftStudyProducesAStabilisingLawAndCompletedTimeHistories"}},
       "The aircraft design workflow executes and checks the stabilising CARE solution. "
       "No published aircraft example containing plant, costs and resulting controller gains "
       "has been transcribed; the example's weights and actuator limits are illustrative."},
      {"synth.pid.realisation",
       "Filtered PID and state-space interconnections",
       {"synth.pid", "model.series", "model.feedback"},
       "Astrom & Murray, Feedback Systems, 2nd ed., chapter 11; state-space block algebra",
       Status::SelfConsistent,
       {E{kUnit, "FilteredPid.FrequencyResponseMatchesTheDefinedTransferFunction"},
        E{kUnit,
          "Interconnection."
          "CascadeAndFeedbackMatchIndependentScalarTransferFunctionsWithFeedthrough"},
        E{kUnit, "Interconnection.MatrixChannelOrderAndAlgebraicFeedthroughArePreserved"}},
       "Realizations agree with independently evaluated transfer functions, including direct "
       "feedthrough and matrix channel order. This does not validate controller tuning."},
      {"hinfinity.analytic_bounds",
       "Hamiltonian H-infinity and sensitivity/disk bounds",
       {"analyze.hinfnorm", "analyze.robust_bounds"},
       "Benner & Mitchell, arXiv:1707.02497, Theorem 2.1; Seiler, Packard & Gahinet (2020)",
       Status::SelfConsistent,
       {E{kUnit, "Hinfinity.NarrowResonanceMissedByAFrequencyGridIsBracketed"},
        E{kUnit, "Hinfinity.DiagonalAndRectangularMimoMatchAnalyticSingularValues"},
        E{kUnit, "RobustBounds.DiskEndpointsInvertTheNormInTheConservativeDirection"}},
       "Analytic scalar/MIMO cases check DC, feedthrough, a narrow resonance and conservative "
       "reciprocal direction. The eigensystem is checked numerically; this is not a "
       "directed-rounding enclosure or an independent-package benchmark collection."},
      {"simulation.convergence",
       "Linear/nonlinear time histories and local convergence",
       {"sim.linear", "sim.nonlinear"},
       "Hairer, Norsett & Wanner, Solving Ordinary Differential Equations I (1993); "
       "smooth-ODE RK4 order and small-disturbance linearization",
       Status::SelfConsistent,
       {E{kUnit,
          "NonlinearSimulation.UnsaturatedActuatorStepConvergesToTheExponentialAtFourthOrder"},
        E{kIntegration, "SimulationConvergence.NonlinearPipelineConvergesUnderStepHalving"},
        E{kIntegration,
          "SimulationConvergence.SmallDisturbancePipelineApproachesTheAugmentedLinearClosedLoop"}},
       "The actuator is checked against its exact exponential; smooth aircraft trajectories "
       "are checked under step halving and shrinking perturbations against a full linearization "
       "with actuator lags. Flight-data validation remains absent."},

      {"model.continuous_scalar",
       "Experimental continuous scalar graph compilation and simulation",
       {"model.compile", "sim.model"},
       "MODEL_CONFORMANCE.md MC01-MC26/B01-B04; analytic linear ODEs and independent RK4 "
       "polynomial",
       Status::Unvalidated,
       {E{kUnit, "Modeling.ExponentialFeedbackMatchesIndependentRk4PolynomialAndContinuousBounds"},
        E{kUnit, "Modeling.CoupledOscillatorUsesOneTemporaryStateForAllDerivatives"},
        E{kUnit, "ModelIo.CanonicalVersionHasAHandSpecifiedByteContract"},
        E{kIntegration,
          "ModelWorkflow.CompiledFeedbackProducesLabeledSamplesAndExplicitEvidenceLimits"}},
       "Synthetic analytic, structural/refusal, parser and source-to-run contracts cover the "
       "bounded continuous scalar feasibility profile. Broader platform/reviewer acceptance, "
       "aircraft blocks, sampled/hybrid execution and aircraft-model validity remain open; "
       "successful execution does not assess numerical accuracy for an arbitrary run."},

      // --- Not implemented --------------------------------------------------
      {"synth.riccati",
       "Full CAREX and DAREX benchmark collections and generalised-pencil solvers",
       {},
       "",
       Status::NotImplemented,
       {},
       "Only the bounded continuous-time Schur solver and small worked comparisons exist. "
       "Singular or indefinite costs and discrete-time Riccati equations remain unsupported."},

  };
  return cases;
}

}  // namespace galata::validation
