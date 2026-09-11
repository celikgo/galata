#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""A-priori budgets for the Souxmar cross-implementation case, and the checks behind them.

THE CASE. QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory, in
tests/validation/test_quadrotor_plant.cpp, steps galata's quadrotor plant through the rotor
commands and wind recorded in the Souxmar fixture reference_trajectory.csv. It gates four
worst-case disagreements with the fixture, every row: the four-rotor speed norm, the attitude
metric min(|qG - qS|, |qG + qS|), the body-rate norm and the position norm. The gate values
are read from the four EXPECT_LT statements that close that test, and the tool prints their
file:line when it runs, so the citation cannot go stale when the comment above them changes.

WHAT THIS DERIVES. How far two CORRECT implementations of the same equations can drift apart
on this fixture because they integrate differently, and each gate's headroom over that figure.
The rotor figure is exact, the body-rate and attitude figures are first-order upper bounds, and
the position figure is an estimate; WHAT THIS IS NOT says what each rests on.
Both steppers take classical RK4 at the fixture's step, hold the command over the step and
renormalise the quaternion once the step is complete. They are related by a constant change
of frame (ENU/FLU to NED/FRD), which RK4 and that renormalisation both commute with. In exact
arithmetic four things separate them:
  1. End-of-step rotor speed. Souxmar applies the exact first-order lag, so it decays by e^-a
     per step, a = h/tau. galata carries rotor speed as a state through RK4, which decays by
     the quartic R(-a) = 1 - a + a^2/2 - a^3/6 + a^4/24.
  2. In-step rotor samples. Souxmar samples the exact lag at the stage times (t, t+h/2, t+h/2,
     t+h). galata's stages see RK4's own stage values. Both reach the rigid body through
     thrust and rotor moment.
  3. Velocity coordinate. Souxmar integrates ENU ground velocity. galata integrates body-axis
     air-relative velocity. RK4 does not commute with that attitude-dependent change of variable.
  4. Stage attitude matrix. Souxmar normalises the quaternion before it forms the matrix at every
     stage. galata's dcm_ned_from_body does not.
Moments depend only on rotor speed and body rate, so items 3 and 4 can reach position but not
attitude or body rate.

PART 1, A PRIORI, EXCEPT ITEMS 3 AND 4. The lag is linear and decoupled, so its disagreement
is exact. For a linear ODE RK4 obeys the identity
h * (weighted stage mean of e) = tau * (e_k - e_{k+1}). So the rate forcing that item 2
drives TELESCOPES over each transient. What survives is the rotor difference itself, which
decays, and a plateau that is set by Souxmar's Simpson rule on the exact exponential. The
Simpson excess is printed as sigma - 1. Telescoping ignores what acts between steps, so
summation by parts charges what it leaves out: tau (e^(h|A|) - 1) sum_{n<N} (R^n - E^n) per
unit change, with |A|_pq for a pitch or roll change and D/K for yaw, to the end of the run
with no cutoff. That charge assumes the between-step propagators have norm at most 1; the
tool checks them on the fixture's rates. Each command change is carried by its own closed form
through the model's moment sensitivities. Transients are summed in absolute value; no
cancellation is assumed between a pulse and the change that undoes it. N counts steps from
the fixture row whose command first differs. The rigid-body Jacobian is charged at the
fixture's own largest rates: item 2's in-step stage coupling, the gyroscopic feed from yaw
into pitch and roll (each step at the larger endpoint), and the Jacobian's action inside each
step on the angle. Attitude is the sum of each step's angle increment. Velocity is driven by
the attitude figure through the fixture's thrust-plus-drag slope, the drag Jacobian dropped as
dissipative in body axes. Items 3 and 4 are NOT a priori: they are one-step defects MEASURED
against the fixture, the galata-scheme reimplementation stepped from each fixture row and
differenced with the fixture's next row. Summed without the dissipation, they supply the part
of the position estimate that items 3 and 4 share.

PART 2, CHECKS, run after the bounds.
  - A reimplementation of Souxmar's stepper, driven by the fixture's commands and wind, must
    reproduce every recorded state column BIT-EXACTLY. That is what makes the parameters read
    from the model file, and the Souxmar scheme assumed above, the fixture's own.
  - A reimplementation of the test's loop around galata's stepper, run against the fixture,
    must stay under the Part 1 figure at EVERY row on every compared quantity. For position
    this check is the only support the figure has. A round-off
    allowance applies to that comparison only: 4 ulp of the quantity's largest magnitude per
    step, never added to a bound. The reimplementation is a model of galata's scheme, not
    galata. It is not bit-exact to the C++, and the case records its own measured values as
    test properties.
  - The decomposition: items 1-2 alone (galata's RK4 lag in Souxmar's coordinates) and
    items 3-4 alone (Souxmar's exact rotor samples fed to galata's rigid body).
  - COUNTERFACTUALS, labelled as such, on schedules that are NOT the fixture. The first holds
    every pulse instead of undoing it; it shows where the validity envelope ends. The second
    runs galata's scheme with a wrong lag time constant; it shows what the case can see.

SOURCES REIMPLEMENTED. galata, line numbers as of this file's commit:
  src/numerics/integrator.cpp   24-63  rk4_step: stages 49-52, combination 58
  src/model/quadrotor.cpp       40-49  drag_force_body_n (per-axis v and v|v| on air velocity)
                                344-365 rotor_wrench: thrust along body -z, r x F, reaction
                                        -spin * k_Q * w^2 about body z
                                367-374 wrench: plus drag and angular drag
                                376-423 derivative: gravity 389, wind added to the position rate
                                        400-402, lag toward the clamped target 418-422
                                484-496 project: renormalise 486, clamp the rotor speeds 490-496
  src/sim/rigid_body.cpp        109-112 gravity_body; 114-169 rigid_body_derivative: position
                                        rate 122-123, -omega x v 128-131, quaternion 134-135,
                                        Euler's equations 150-152
  src/core/quaternion.cpp       35-59  dcm_ned_from_body, written out, NO normalisation (item 4)
                                130-148 quaternion_derivative
  src/core/state.cpp            49-51  renormalise_attitude
  include/galata/core/constants.hpp 30 kStandardGravity (read by this tool at run time)
  and the test's loop: the air-relative state built from each row, the wind re-basing at a
  wind change, rk4_step then project, and the four worst-case metrics.
Souxmar, revision 6ebe5f010ef3dd460ade18e6989bb0c262ecdded (read, never imported):
  fcs/plant/quadrotor.py        11-12  gravity; 56-62 rotor_wrench_matrix (X order FL, RL, RR,
                                        FR; reaction signs); 65-68 drag_force_body;
                                139-150 _derivative (ENU ground velocity, drag on the air
                                        velocity in body axes); 152-176 step: exact lag
                                        159-162, stages sample it 164-168, combination 169,
                                        normalisation after the step 174
  fcs/math3d.py                 18-24  quaternion_normalize; 27-36 quaternion_multiply;
                                39-46  rotation_matrix (normalises at every call);
                                49-67  quaternion_from_matrix
  fcs/analysis/reference.py     21-27  schedule and wind; 50-61 command_at, wind_at;
                                64-67  NED/FRD quaternion by matrix basis change;
                                81-92  the loop: record the row, then step with its command
  interfaces/frames.py          5-6    ENU_TO_NED, FLU_TO_FRD

INPUTS. Every physical parameter comes from the model file: mass, principal inertia, hub
positions, spin senses, k_T, k_Q, the lag time constant, speed limits, and the linear,
quadratic and angular drag. The file is read with the standard library; its subset of YAML
is parsed strictly. A key the derivation does not model is an error, not ignored. Gravity is
read from galata's constants header. The step, the commands, the wind and the motion
envelope come from the fixture. The hover speed and the lag decay the fixture implies are
checked against the model. If the fixture's sidecar reference_trajectory.json is present, its
digest and every parameter this derivation uses are checked against it. Its
max_rotor_speed_rad_s differs from the model's by design (models/souxmar-quad/PROVENANCE.md),
so the tool checks only that neither ceiling clamps the fixture. Its battery,
motor-efficiency, avionics and reserve keys are not used and not checked. If the programme's
bridge_check.json is present, its measured linear-check rotor error is read and printed: an
earlier banner quoted it.

WHAT THIS IS NOT.
  - Not a validation. It bounds the disagreement between two implementations of the same
    equations. It says nothing about any aircraft, and nothing about whether the equations
    are right.
  - First order. The bounds are linear in a per-step relative defect of order 1e-7. Part 1
    prints that defect as (R-E)/E. The neglected second order is below double-precision
    round-off on these states. It is not zero in general.
  - Valid only inside the fixture's motion envelope. Part 1 prints that envelope: the largest
    |omega|, the largest |v_b| and the largest thrust-plus-drag slope. Items 3 and 4 grow like
    (h|omega|)^5 |v|. A schedule that spins the body an order of magnitude faster needs a new
    derivation, not these numbers; the first counterfactual shows one that breaks the
    position gate.
  - The Souxmar reimplementation is checked BIT-EXACT against the fixture. The galata
    reimplementation is not checked against galata. It is a model of the scheme, and it
    stands or falls by staying under the bounds at every row.
  - Not every figure is a proven bound. The rotor term is exact. The body-rate and attitude
    figures are upper bounds to first order: telescoped closed forms plus the between-step
    remainder, with the propagator norms they assume checked on the fixture, and stage values
    between rows taken from the rows and their mean. The velocity and position figures are
    ESTIMATES: sampled at rows with the larger endpoint, and built on the measured items 3-4
    defects. They are not proven to bound. Their only support is the row-by-row check in
    Part 2, which reports its margin.

Usage:
  tools/validation/souxmar_cross_check_budgets.py --fixture-dir <dir with reference_trajectory.csv>
      [--model models/souxmar-quad/souxmar-quad.yaml]
It writes nothing and prints to stdout. The output is deterministic for a given fixture, model
and platform.

Exit status: 0 when every check holds. 1 when a gate sits at or below its Part 1 figure, when
the galata-scheme reimplementation exceeds a Part 1 figure at any row, when the Souxmar
reimplementation is not bit-exact, when a checked propagator is not contracting, or when the
model and the fixture disagree. 2 on unusable input: a file that cannot be read, a
non-numeric or missing cell, a missing key, or a fixture whose commands never change.

Dependencies: Python 3 and numpy, the only third-party package. It has been run under Python
3.14 with numpy 2.5; no other version has been tried.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = REPO_ROOT / "models" / "souxmar-quad" / "souxmar-quad.yaml"
TEST_SOURCE = REPO_ROOT / "tests" / "validation" / "test_quadrotor_plant.cpp"
CONSTANTS_HEADER = REPO_ROOT / "include" / "galata" / "core" / "constants.hpp"
TEST_SUITE, TEST_NAME = "QuadrotorCrossImplementation", "ReproducesTheSouxmarOpenLoopTrajectory"

# The fixture's frame statement (reference_trajectory.json "frames"; interfaces/frames.py:5-6).
ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])

# Rotor sign patterns in the fixture's order FL, RL, RR, FR, with the channel each excites.
PATTERNS = {"pitch": (1, -1, -1, 1), "roll": (1, 1, -1, -1), "yaw": (1, -1, 1, -1),
            "collective": (1, 1, 1, 1)}

GATE_METRICS = {"worst_rotor_rad_s": "rotor", "worst_attitude": "attitude",
                "worst_rate_rad_s": "body rate", "worst_position_m": "position"}
GATE_ORDER = ("rotor", "attitude", "body rate", "position")

STATE_COLUMNS_ENU = ("east_m", "north_m", "up_m", "ground_v_east_m_s", "ground_v_north_m_s",
                     "ground_v_up_m_s", "q_w", "q_x_flu", "q_y_flu", "q_z_flu",
                     "rate_x_flu_rad_s", "rate_y_flu_rad_s", "rate_z_flu_rad_s")
STATE_COLUMNS_NED = ("north_m_ned", "east_m_ned", "down_m_ned", "ground_v_north_m_s_ned",
                     "ground_v_east_m_s_ned", "ground_v_down_m_s_ned", "q_w_frd_to_ned",
                     "q_x_frd_to_ned", "q_y_frd_to_ned", "q_z_frd_to_ned", "p_frd_rad_s",
                     "q_frd_rad_s", "r_frd_rad_s")
COMMAND_COLUMNS = ("cmd_fl_rad_s", "cmd_rl_rad_s", "cmd_rr_rad_s", "cmd_fr_rad_s")
ROTOR_COLUMNS = ("rotor_fl_rad_s", "rotor_rl_rad_s", "rotor_rr_rad_s", "rotor_fr_rad_s")
WIND_COLUMNS_ENU = ("wind_east_m_s", "wind_north_m_s", "wind_up_m_s")


class InputError(Exception):
    """An input the derivation cannot use: exit status 2."""


class Checks:
    """Collects failed checks so every one is reported before the exit status is decided."""

    def __init__(self) -> None:
        self.failures: list[str] = []

    def require(self, condition: bool, message: str) -> None:
        """Record ``message`` as a failure unless ``condition`` holds."""
        if not condition:
            self.failures.append(message)
            print(f"  FAILED: {message}")


# =============================================================================================
# INPUTS
# =============================================================================================

def _strip_comment(line: str, where: str) -> str:
    """Drop a '#' comment that is outside a double-quoted string."""
    inside = False
    for index, character in enumerate(line):
        if character == '"':
            inside = not inside
        elif character == "#" and not inside and (index == 0 or line[index - 1] == " "):
            return line[:index].rstrip()
    if inside:
        raise InputError(f"{where}: unterminated string")
    return line.rstrip()


def _parse_number(text: str, where: str) -> float:
    """A finite number, or an error naming the line."""
    try:
        value = float(text)
    except ValueError:
        raise InputError(f"{where}: expected a number, found {text!r}") from None
    if not math.isfinite(value):
        raise InputError(f"{where}: {text!r} is not finite")
    return value


def _parse_scalar(text: str, where: str):
    """A double-quoted string, a flow sequence of numbers, or a number. Nothing else."""
    if text.startswith('"'):
        body = text[1:-1]
        if len(text) < 2 or not text.endswith('"') or '"' in body or "\\" in body:
            raise InputError(f"{where}: unsupported string {text}")
        return body
    if text.startswith("["):
        if not text.endswith("]"):
            raise InputError(f"{where}: unterminated sequence {text}")
        return [_parse_number(item.strip(), where) for item in text[1:-1].split(",")]
    return _parse_number(text, where)


def _parse_block(lines, start: int, indent: int, name: str):
    """Parse the mapping or the sequence of mappings that begins at ``lines[start]``."""
    if lines[start][2].startswith("- "):
        items = []
        index = start
        while index < len(lines) and lines[index][1] == indent and lines[index][2].startswith("- "):
            number, _, text = lines[index]
            item_lines = [(number, indent + 2, text[2:].strip())]
            index += 1
            while index < len(lines) and lines[index][1] > indent:
                item_lines.append(lines[index])
                index += 1
            value, used = _parse_block(item_lines, 0, indent + 2, name)
            if used != len(item_lines):
                raise InputError(f"{name}:{item_lines[used][0]}: unexpected indentation")
            items.append(value)
        return items, index
    mapping = {}
    index = start
    while index < len(lines) and lines[index][1] == indent:
        number, _, text = lines[index]
        where = f"{name}:{number}"
        key, colon, rest = text.partition(":")
        if not colon or not re.fullmatch(r"[a-z_][a-z0-9_]*", key):
            raise InputError(f"{where}: expected 'key: value', found {text!r}")
        if key in mapping:
            raise InputError(f"{where}: duplicate key {key!r}")
        rest = rest.strip()
        index += 1
        if rest:
            mapping[key] = _parse_scalar(rest, where)
        elif index < len(lines) and lines[index][1] > indent:
            mapping[key], index = _parse_block(lines, index, lines[index][1], name)
        else:
            raise InputError(f"{where}: {key!r} has no value")
    return mapping, index


def parse_model_yaml(text: str, name: str) -> dict:
    """Parse the strict YAML subset the model files use: block mappings, block sequences of
    mappings, flow sequences of numbers, numbers and double-quoted strings."""
    lines = []
    for number, raw in enumerate(text.splitlines(), start=1):
        where = f"{name}:{number}"
        if "\t" in raw:
            raise InputError(f"{where}: tab character")
        content = _strip_comment(raw, where)
        if content.strip():
            lines.append((number, len(content) - len(content.lstrip(" ")), content.strip()))
    if not lines or lines[0][1] != 0:
        raise InputError(f"{name}: empty or indented document")
    value, used = _parse_block(lines, 0, 0, name)
    if used != len(lines):
        raise InputError(f"{name}:{lines[used][0]}: unexpected indentation")
    return value


def _keys(mapping, required: set, optional: set, where: str) -> None:
    """Require exactly the keys this derivation models; an unknown key is an error."""
    if not isinstance(mapping, dict):
        raise InputError(f"{where}: expected a mapping")
    missing = sorted(required - mapping.keys())
    unknown = sorted(mapping.keys() - required - optional)
    if missing:
        raise InputError(f"{where}: missing {', '.join(missing)}")
    if unknown:
        raise InputError(f"{where}: {', '.join(unknown)} is not modelled by this derivation")


def _vector3(value, where: str) -> np.ndarray:
    """Three numbers."""
    if not isinstance(value, list) or len(value) != 3:
        raise InputError(f"{where}: expected three numbers")
    return np.array(value, dtype=float)


@dataclass(frozen=True)
class Model:
    """The physical parameters the derivation uses, all from the model file, SI."""

    label: str
    sha256: str
    mass_kg: float
    inertia_kg_m2: np.ndarray            # principal moments, body axes
    hub_frd_m: np.ndarray                # (4, 3) CG to hub, body FRD, fixture rotor order
    spin: np.ndarray                     # (4,) +1/-1 about body z
    thrust_coefficient_n_s2: float
    torque_coefficient_n_m_s2: float
    time_constant_s: float
    minimum_speed_rad_s: float
    maximum_speed_rad_s: float
    drag_linear_n_s_m: np.ndarray
    drag_quadratic_n_s2_m2: np.ndarray
    drag_angular_n_m_s: np.ndarray

    @property
    def hub_offset_m(self) -> float:
        """The common |x| = |y| of an X-configuration hub."""
        return float(abs(self.hub_frd_m[0, 0]))


def load_model(path: Path) -> Model:
    """Read the model file and require the assumptions the algebra rests on."""
    try:
        raw = path.read_bytes()
        text = raw.decode("utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise InputError(f"cannot read the model {path}: {error}") from None
    try:
        label = str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        label = str(path)
    root = parse_model_yaml(text, label)
    _keys(root, {"mass", "rotors", "drag"}, {"description", "citation"}, label)
    mass = root["mass"]
    products = {"product_of_inertia_xy_kg_m2", "product_of_inertia_xz_kg_m2",
                "product_of_inertia_yz_kg_m2"}
    _keys(mass, {"mass_kg", "inertia_xx_kg_m2", "inertia_yy_kg_m2", "inertia_zz_kg_m2"},
          products, f"{label}.mass")
    for key in sorted(products & mass.keys()):
        if mass[key] != 0.0:
            raise InputError(f"{label}.mass.{key}: a product of inertia is not modelled")
    rotors = root["rotors"]
    if not isinstance(rotors, list) or len(rotors) != 4:
        raise InputError(f"{label}.rotors: the fixture has four rotors")
    rotor_keys = {"position_cg_to_hub_body_m", "spin_about_body_z", "thrust_coefficient_n_s2",
                  "torque_coefficient_n_m_s2", "speed_time_constant_s", "maximum_speed_rad_s"}
    for index, rotor in enumerate(rotors):
        _keys(rotor, rotor_keys, {"minimum_speed_rad_s"}, f"{label}.rotors[{index}]")
        rotor.setdefault("minimum_speed_rad_s", 0.0)   # the loader's default, quadrotor.cpp:575
    for key in sorted(rotor_keys - {"position_cg_to_hub_body_m", "spin_about_body_z"}
                      | {"minimum_speed_rad_s"}):
        if len({rotor[key] for rotor in rotors}) != 1:
            raise InputError(f"{label}.rotors: {key} differs between rotors; the lag algebra "
                             "and the hover speed assume one value")
    hub = np.array([_vector3(r["position_cg_to_hub_body_m"], f"{label}.rotors") for r in rotors])
    spin = np.array([float(r["spin_about_body_z"]) for r in rotors])
    if not np.all(np.isin(spin, (1.0, -1.0))):
        raise InputError(f"{label}.rotors: spin_about_body_z must be +1 or -1")
    offset = abs(hub[0, 0])
    if not (np.all(np.abs(hub[:, :2]) == offset) and np.all(hub[:, 2] == 0.0)):
        raise InputError(f"{label}.rotors: the sensitivities assume an X configuration with "
                         "equal |x| = |y| and the rotor plane through the CG")
    drag = root["drag"]
    _keys(drag, {"linear_n_s_m", "quadratic_n_s2_m2", "angular_n_m_s"}, set(), f"{label}.drag")
    inertia = np.array([mass["inertia_xx_kg_m2"], mass["inertia_yy_kg_m2"],
                        mass["inertia_zz_kg_m2"]], dtype=float)
    if inertia[0] != inertia[1]:
        raise InputError(f"{label}.mass: the pitch/roll coupling bound assumes J_xx = J_yy")
    if drag["angular_n_m_s"][0] != drag["angular_n_m_s"][1]:
        raise InputError(f"{label}.drag: the pitch/roll propagator assumes equal x and y "
                         "angular drag")
    first = rotors[0]
    return Model(label=label, sha256=hashlib.sha256(raw).hexdigest(),
                 mass_kg=float(mass["mass_kg"]),
                 inertia_kg_m2=inertia, hub_frd_m=hub, spin=spin,
                 thrust_coefficient_n_s2=float(first["thrust_coefficient_n_s2"]),
                 torque_coefficient_n_m_s2=float(first["torque_coefficient_n_m_s2"]),
                 time_constant_s=float(first["speed_time_constant_s"]),
                 minimum_speed_rad_s=float(first["minimum_speed_rad_s"]),
                 maximum_speed_rad_s=float(first["maximum_speed_rad_s"]),
                 drag_linear_n_s_m=_vector3(drag["linear_n_s_m"], f"{label}.drag"),
                 drag_quadratic_n_s2_m2=_vector3(drag["quadratic_n_s2_m2"], f"{label}.drag"),
                 drag_angular_n_m_s=_vector3(drag["angular_n_m_s"], f"{label}.drag"))


def read_standard_gravity(path: Path) -> float:
    """kStandardGravity from galata's constants header, so the tool carries no copy of it."""
    try:
        text = path.read_text("utf-8")
    except (OSError, UnicodeDecodeError) as error:
        raise InputError(f"cannot read {path}: {error}") from None
    match = re.search(r"kStandardGravity\s*=\s*([0-9.eE+-]+)\s*;", text)
    if not match:
        raise InputError(f"{path}: kStandardGravity not found")
    return float(match.group(1))


@dataclass(frozen=True)
class Gate:
    """One EXPECT_LT that closes the test: what it bounds, the value, and where it is."""

    quantity: str
    value: float
    literal: str      # as the test writes it, so the output quotes the gate verbatim
    line: int


def read_gates(path: Path) -> dict[str, Gate]:
    """The four gates, as the test states them. Reading them keeps the tool and the test honest
    with each other: a changed gate changes what this tool compares against."""
    text = path.read_text("utf-8")
    header = f"TEST({TEST_SUITE}, {TEST_NAME})"
    start = text.find(header)
    if start < 0:
        raise InputError(f"{path}: {header} not found")
    following = text.find("\nTEST", start + len(header))
    body = text[start:] if following < 0 else text[start:following]
    gates: dict[str, Gate] = {}
    for match in re.finditer(r"EXPECT_LT\((worst_[a-z_]+),\s*([0-9.eE+-]+)\)", body):
        quantity = GATE_METRICS.get(match.group(1))
        if quantity is None or quantity in gates:
            raise InputError(f"{path}: unexpected or repeated gate on {match.group(1)}")
        line = text.count("\n", 0, start + match.start()) + 1
        gates[quantity] = Gate(quantity, float(match.group(2)), match.group(2), line)
    if set(gates) != set(GATE_ORDER):
        raise InputError(f"{path}: expected gates on {', '.join(GATE_ORDER)}")
    return gates


@dataclass(frozen=True)
class Fixture:
    """The recorded trajectory: every column, by name."""

    path: Path
    sha256: str
    columns: dict[str, np.ndarray]
    sidecar: dict | None
    bridge: dict | None   # bridge_check.json linear_check rotor: case -> (relative error, step)

    def stack(self, names) -> np.ndarray:
        """Columns side by side, one row per sample."""
        return np.stack([self.columns[name] for name in names], axis=1)


def load_fixture(directory: Path) -> Fixture:
    """reference_trajectory.csv and, when present, its sidecar reference_trajectory.json."""
    path = directory / "reference_trajectory.csv"
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise InputError(f"cannot read {path}: {error}") from None
    try:
        lines = raw.decode("utf-8").replace("\r", "").strip().split("\n")
    except UnicodeDecodeError as error:
        raise InputError(f"{path}: not UTF-8: {error}") from None
    names = lines[0].split(",")
    if len(lines) < 3:
        raise InputError(f"{path}: fewer than two rows")
    parsed = []
    for number, line in enumerate(lines[1:], start=2):
        values = line.split(",")
        if len(values) != len(names):
            raise InputError(f"{path}:{number}: {len(values)} cells, header has {len(names)}")
        parsed.append([_parse_number(value, f"{path}:{number}") for value in values])
    rows = np.array(parsed)
    columns = {name: rows[:, index] for index, name in enumerate(names)}
    needed = (("time_s",) + COMMAND_COLUMNS + WIND_COLUMNS_ENU + ROTOR_COLUMNS
              + STATE_COLUMNS_ENU + STATE_COLUMNS_NED)
    missing = [name for name in needed if name not in columns]
    if missing:
        raise InputError(f"{path}: missing columns {', '.join(missing)}")
    sidecar = _read_json(directory / "reference_trajectory.json")
    bridge = None
    check = _read_json(directory / "bridge_check.json")
    if check is not None:
        try:
            cases = check["linear_check"]["cases"]
            bridge = {name: (float(cases[name]["relative_error"]["rotor"]),
                             float(cases[name]["max_excursion"]["rotor"]))
                      for name in ("climb", "pitch")}
        except (KeyError, TypeError, ValueError) as error:
            raise InputError(f"{directory / 'bridge_check.json'}: no linear_check rotor figure "
                             f"for climb and pitch ({error!r})") from None
    return Fixture(path, hashlib.sha256(raw).hexdigest(), columns, sidecar, bridge)


def _read_json(path: Path) -> dict | None:
    """A JSON object, None when the file is absent, an InputError when it is unusable."""
    if not path.is_file():
        return None
    try:
        value = json.loads(path.read_text("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise InputError(f"cannot read {path}: {error}") from None
    if not isinstance(value, dict):
        raise InputError(f"{path}: expected a JSON object")
    return value


# =============================================================================================
# THE TWO STEPPERS, REIMPLEMENTED
# =============================================================================================

def dcm(quaternion: np.ndarray) -> np.ndarray:
    """Body-to-world matrix of a wxyz quaternion, NOT normalised first (quaternion.cpp:35-59)."""
    w, x, y, z = quaternion
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def dcm_normalised(quaternion: np.ndarray) -> np.ndarray:
    """Souxmar's rotation_matrix: normalise, then form the matrix (math3d.py:39-46)."""
    return dcm(quaternion / np.linalg.norm(quaternion))


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Hamilton product, wxyz (math3d.py:27-36)."""
    aw, ax, ay, az = left
    bw, bx, by, bz = right
    return np.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Souxmar's quaternion_from_matrix, same arithmetic (math3d.py:49-67)."""
    trace = float(np.trace(matrix))
    if trace > 0:
        s = 2 * np.sqrt(trace + 1)
        q = np.array([s / 4, (matrix[2, 1] - matrix[1, 2]) / s, (matrix[0, 2] - matrix[2, 0]) / s,
                      (matrix[1, 0] - matrix[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(matrix)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(max(0.0, 1 + matrix[i, i] - matrix[j, j] - matrix[k, k]))
        q = np.zeros(4)
        q[0] = (matrix[k, j] - matrix[j, k]) / s
        q[i + 1] = s / 4
        q[j + 1] = (matrix[j, i] + matrix[i, j]) / s
        q[k + 1] = (matrix[k, i] + matrix[i, k]) / s
    return q / np.linalg.norm(q)


def quaternion_distance(first: np.ndarray, second: np.ndarray) -> float:
    """The test's attitude metric: the shorter of q and -q."""
    return min(np.linalg.norm(first - second), np.linalg.norm(first + second))


class Plant:
    """The model's parameters arranged the way each implementation consumes them."""

    def __init__(self, model: Model, gravity_m_s2: float, step_s: float):
        self.model = model
        self.gravity = gravity_m_s2
        self.step_s = step_s
        self.mass = model.mass_kg
        self.inertia = model.inertia_kg_m2
        self.k_thrust = model.thrust_coefficient_n_s2
        self.k_torque = model.torque_coefficient_n_m_s2
        self.tau = model.time_constant_s
        self.drag_linear = model.drag_linear_n_s_m
        self.drag_quadratic = model.drag_quadratic_n_s2_m2
        self.drag_angular = model.drag_angular_n_m_s
        # Souxmar's wrench matrix (quadrotor.py:56-62): rows collective, FLU x torque from the
        # hub FLU y, FLU y torque from minus the hub FLU x, and the reaction row. Its reaction
        # sign about FLU z is +spin, because galata's reaction is -spin about FRD z = -FLU z.
        hub_flu = model.hub_frd_m @ FLU_TO_FRD   # FLU_TO_FRD is its own inverse
        self.wrench_matrix = np.vstack((np.ones(4), hub_flu[:, 1], -hub_flu[:, 0],
                                        model.spin * self.k_torque / self.k_thrust))
        self.gravity_enu = np.array([0.0, 0.0, -gravity_m_s2])

    # ---- Souxmar --------------------------------------------------------------------------
    def souxmar_derivative(self, state, rotor_speed, wind_enu):
        """quadrotor.py:139-150, same operation order."""
        velocity, quaternion, rate = state[3:6], state[6:10], state[10:13]
        rotation = dcm_normalised(quaternion)
        wrench = self.wrench_matrix @ (self.k_thrust * rotor_speed ** 2)
        air = rotation.T @ (velocity - wind_enu)
        body_force = (-self.drag_linear * air - self.drag_quadratic * air * np.abs(air)
                      + np.array([0.0, 0.0, wrench[0]]))
        acceleration = self.gravity_enu + rotation @ body_force / self.mass
        angular = (wrench[1:] - self.drag_angular * rate
                   - np.cross(rate, self.inertia * rate)) / self.inertia
        quaternion_rate = 0.5 * quaternion_multiply(quaternion, np.r_[0.0, rate])
        return np.r_[velocity, acceleration, quaternion_rate, angular]

    def lag_stages(self, old, target, scheme, tau=None):
        """Stage rotor speeds and the end-of-step speed. 'exact' is Souxmar's (quadrotor.py:
        159-162); 'rk4' is the lag carried through RK4 on its own, as galata's is."""
        tau = self.tau if tau is None else tau
        h = self.step_s
        if scheme == "exact":
            decay = np.exp(-h / tau)
            half = target + (old - target) * np.sqrt(decay)
            new = target + (old - target) * decay
            return [old, half, half, new], new
        s1 = old
        g1 = (target - s1) / tau
        s2 = old + 0.5 * h * g1
        g2 = (target - s2) / tau
        s3 = old + 0.5 * h * g2
        g3 = (target - s3) / tau
        s4 = old + h * g3
        g4 = (target - s4) / tau
        return [s1, s2, s3, s4], old + (h / 6.0) * (g1 + 2.0 * (g2 + g3) + g4)

    def souxmar_step(self, state, rotor, command, wind_enu, lag="exact"):
        """quadrotor.py:152-176. lag='rk4' replaces only the rotor lag (items 1-2 alone)."""
        h = self.step_s
        stages, new_rotor = self.lag_stages(rotor, command, lag)
        k1 = self.souxmar_derivative(state, stages[0], wind_enu)
        k2 = self.souxmar_derivative(state + 0.5 * h * k1, stages[1], wind_enu)
        k3 = self.souxmar_derivative(state + 0.5 * h * k2, stages[2], wind_enu)
        k4 = self.souxmar_derivative(state + h * k3, stages[3], wind_enu)
        new = state + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        new[6:10] = new[6:10] / np.linalg.norm(new[6:10])
        return new, new_rotor

    # ---- galata ---------------------------------------------------------------------------
    def galata_rigid_derivative(self, state, rotor_speed, wind_ned):
        """quadrotor.cpp:344-402 with rigid_body.cpp:114-169: air-relative body velocity,
        unnormalised stage matrix."""
        velocity, quaternion, rate = state[3:6], state[6:10], state[10:13]
        rotation = dcm(quaternion)
        thrust = self.k_thrust * rotor_speed * rotor_speed
        drag = self.drag_linear * velocity + self.drag_quadratic * velocity * np.abs(velocity)
        force = np.array([0.0, 0.0, -thrust.sum()]) - drag
        moment = np.zeros(3)
        for index in range(4):
            moment += np.cross(self.model.hub_frd_m[index], np.array([0.0, 0.0, -thrust[index]]))
        moment[2] += np.sum(-self.model.spin * self.k_torque * rotor_speed * rotor_speed)
        moment -= self.drag_angular * rate
        return np.r_[rotation @ velocity + wind_ned,
                     force / self.mass + rotation.T @ np.array([0.0, 0.0, self.gravity])
                     - np.cross(rate, velocity),
                     0.5 * quaternion_multiply(quaternion, np.r_[0.0, rate]),
                     (moment - np.cross(rate, self.inertia * rate)) / self.inertia]

    def galata_step(self, state, rotor, command, wind_ned, lag="rk4", tau=None):
        """rk4_step on the 17-state, then project (integrator.cpp:24-63, quadrotor.cpp:484-496).
        lag='exact' feeds Souxmar's exact stage samples to galata's rigid body instead, a
        hybrid that isolates items 3-4."""
        h = self.step_s
        tau = self.tau if tau is None else tau
        low, high = self.model.minimum_speed_rad_s, self.model.maximum_speed_rad_s
        if lag == "rk4":
            target = np.clip(command, low, high)               # quadrotor.cpp:418-420

            def rate_of(extended):
                return np.r_[self.galata_rigid_derivative(extended[:13], extended[13:], wind_ned),
                             (target - extended[13:]) / tau]

            extended = np.r_[state, rotor]
            k1 = rate_of(extended)
            k2 = rate_of(extended + 0.5 * h * k1)
            k3 = rate_of(extended + 0.5 * h * k2)
            k4 = rate_of(extended + h * k3)
            extended = extended + (h / 6.0) * (k1 + 2.0 * (k2 + k3) + k4)
            new, new_rotor = extended[:13].copy(), extended[13:].copy()
        else:
            stages, new_rotor = self.lag_stages(rotor, command, "exact")
            k1 = self.galata_rigid_derivative(state, stages[0], wind_ned)
            k2 = self.galata_rigid_derivative(state + 0.5 * h * k1, stages[1], wind_ned)
            k3 = self.galata_rigid_derivative(state + 0.5 * h * k2, stages[2], wind_ned)
            k4 = self.galata_rigid_derivative(state + h * k3, stages[3], wind_ned)
            new = state + (h / 6.0) * (k1 + 2.0 * (k2 + k3) + k4)
        new[6:10] = new[6:10] / np.linalg.norm(new[6:10])
        return new, np.clip(new_rotor, low, high)


@dataclass(frozen=True)
class Reference:
    """A trajectory in the test's convention (NED/FRD, ground velocity), one row per sample."""

    command: np.ndarray        # (n, 4) held over the step from each row
    wind_ned: np.ndarray       # (n, 3)
    position: np.ndarray       # (n, 3)
    ground_velocity: np.ndarray
    quaternion: np.ndarray     # (n, 4) FRD to NED
    rate: np.ndarray           # (n, 3) FRD
    rotor: np.ndarray          # (n, 4)


def enu_wind_to_ned(wind_enu: np.ndarray) -> np.ndarray:
    """[n, e, d] = [y, x, -z], the test's wind conversion."""
    return wind_enu @ ENU_TO_NED.T


def reference_from_fixture(fixture: Fixture) -> Reference:
    """The fixture's own NED/FRD columns, as the test reads them."""
    return Reference(command=fixture.stack(COMMAND_COLUMNS),
                     wind_ned=enu_wind_to_ned(fixture.stack(WIND_COLUMNS_ENU)),
                     position=fixture.stack(STATE_COLUMNS_NED[0:3]),
                     ground_velocity=fixture.stack(STATE_COLUMNS_NED[3:6]),
                     quaternion=fixture.stack(STATE_COLUMNS_NED[6:10]),
                     rate=fixture.stack(STATE_COLUMNS_NED[10:13]),
                     rotor=fixture.stack(ROTOR_COLUMNS))


def souxmar_run(plant: Plant, fixture: Fixture, commands: np.ndarray, lag: str = "exact"):
    """reference.py:81-92 from the fixture's first row, with the given commands and the
    fixture's wind. Returns the raw ENU/FLU state and rotor rows, and the NED Reference."""
    wind_enu = fixture.stack(WIND_COLUMNS_ENU)
    state = fixture.stack(STATE_COLUMNS_ENU)[0].copy()
    rotor = fixture.stack(ROTOR_COLUMNS)[0].copy()
    samples = commands.shape[0]
    states, rotors = np.empty((samples, 13)), np.empty((samples, 4))
    for row in range(samples):
        states[row], rotors[row] = state, rotor
        if row + 1 < samples:
            state, rotor = plant.souxmar_step(state, rotor, commands[row], wind_enu[row], lag)
    quaternion = np.array([quaternion_from_matrix(ENU_TO_NED @ dcm_normalised(q) @ FLU_TO_FRD)
                           for q in states[:, 6:10]])
    reference = Reference(command=commands, wind_ned=enu_wind_to_ned(wind_enu),
                          position=states[:, 0:3] @ ENU_TO_NED.T,
                          ground_velocity=states[:, 3:6] @ ENU_TO_NED.T,
                          quaternion=quaternion, rate=states[:, 10:13] @ FLU_TO_FRD.T,
                          rotor=rotors)
    return states, rotors, reference


@dataclass(frozen=True)
class Comparison:
    """The test's four metrics, worst over the run and per compared row (rows 1..n-1)."""

    rotor: np.ndarray
    attitude: np.ndarray
    rate: np.ndarray
    position: np.ndarray

    def worst(self) -> dict[str, float]:
        """Worst case of each quantity, keyed as the gates are."""
        return {"rotor": float(self.rotor.max()), "attitude": float(self.attitude.max()),
                "body rate": float(self.rate.max()), "position": float(self.position.max())}


def describe(worst: dict[str, float]) -> str:
    """One line, fixed format."""
    return (f"rotor {worst['rotor']:.4e}  attitude {worst['attitude']:.4e}  "
            f"body_rate {worst['body rate']:.4e}  position {worst['position']:.4e}")


def air_body_state(reference: Reference, row: int) -> np.ndarray:
    """The test's extended_from_row: air-relative body velocity from ground velocity and wind."""
    quaternion = reference.quaternion[row]
    air = dcm(quaternion).T @ (reference.ground_velocity[row] - reference.wind_ned[row])
    return np.r_[reference.position[row], air, quaternion, reference.rate[row]]


def galata_versus(plant: Plant, reference: Reference, lag: str = "rk4", tau=None) -> Comparison:
    """The test's loop: start from the reference's first row, step with each row's command and
    wind, re-base the air velocity when the wind changes, compare with the next row."""
    samples = reference.command.shape[0]
    state, rotor = air_body_state(reference, 0), reference.rotor[0].copy()
    previous_wind = reference.wind_ned[0].copy()
    metrics = np.zeros((4, samples - 1))
    for row in range(samples - 1):
        wind = reference.wind_ned[row]
        if np.any(wind != previous_wind):
            state[3:6] += dcm(state[6:10]).T @ (previous_wind - wind)
            previous_wind = wind.copy()
        state, rotor = plant.galata_step(state, rotor, reference.command[row], wind, lag, tau)
        expected = air_body_state(reference, row + 1)
        metrics[0, row] = np.linalg.norm(rotor - reference.rotor[row + 1])
        metrics[1, row] = quaternion_distance(state[6:10], expected[6:10])
        metrics[2, row] = np.linalg.norm(state[10:13] - expected[10:13])
        metrics[3, row] = np.linalg.norm(state[:3] - expected[:3])
    return Comparison(*metrics)


# =============================================================================================
# MODEL AGAINST FIXTURE
# =============================================================================================

def check_model_against_fixture(model: Model, fixture: Fixture, gravity: float, step_s: float,
                                checks: Checks) -> float:
    """Every parameter the fixture lets this tool check; returns the hover speed."""
    print("\n=== model file against the fixture ===")
    command = fixture.stack(COMMAND_COLUMNS)
    rotor = fixture.stack(ROTOR_COLUMNS)
    hover = float(command[0, 0])
    implied = math.sqrt(model.mass_kg * gravity / (4 * model.thrust_coefficient_n_s2))
    print(f"hover: fixture row 0 {hover!r} rad/s; sqrt(m g / (4 k_T)) from the model {implied!r}")
    checks.require(abs(implied - hover) <= 4 * np.spacing(hover) and np.all(command[0] == hover)
                   and np.all(rotor[0] == hover), "hover speed disagrees with m g / (4 k_T)")
    first = next((row for row in range(1, command.shape[0] - 1)
                  if np.any(command[row] != command[row - 1])), None)
    if first is None:
        raise InputError(f"{fixture.path}: no command change before the last row")
    target = command[first]
    decay_implied = (rotor[first + 1] - target) / (rotor[first] - target)
    decay_model = math.exp(-step_s / model.time_constant_s)
    worst = float(np.abs(decay_implied / decay_model - 1).max())
    print(f"lag: the step after row {first} decays by {float(decay_implied[0])!r}; "
          f"exp(-h/tau) from the model {decay_model!r}; worst relative difference {worst:.1e}")
    checks.require(worst < 1e-12, "the lag time constant disagrees with the fixture's rotor decay")
    speed_max = float(max(command.max(), rotor.max()))
    speed_min = float(min(command.min(), rotor.min()))
    print(f"speed limits: fixture spans [{speed_min:.2f}, {speed_max:.2f}] rad/s inside the "
          f"model's [{model.minimum_speed_rad_s:.2f}, {model.maximum_speed_rad_s:.2f}], so "
          "galata's clamps are inactive; Souxmar's are shown inactive by the bit-exact "
          "reproduction, which has none")
    checks.require(model.minimum_speed_rad_s < speed_min and speed_max < model.maximum_speed_rad_s,
                   "a rotor clamp would be active")
    sidecar = fixture.sidecar
    if sidecar is None:
        print("sidecar reference_trajectory.json absent: parameters are checked only through the "
              "bit-exact reproduction in Part 2")
        return hover
    try:
        vehicle = sidecar["vehicle_parameters"]
        pairs = _sidecar_pairs(sidecar, vehicle, model, fixture, step_s, command, hover)
        ceiling = float(vehicle["max_rotor_speed_rad_s"])
    except (KeyError, TypeError, ValueError) as error:
        raise InputError(f"sidecar reference_trajectory.json: missing or malformed "
                         f"{error!r}") from None
    for name, published, used in pairs:
        same = published == used
        print(f"sidecar {name}: {'identical' if same else f'{published!r} != {used!r}'}")
        checks.require(same, f"sidecar {name} disagrees with the model or the fixture")
    # The sidecar's ceiling is the speed at nominal voltage; the model carries the hover-load
    # ceiling (models/souxmar-quad/PROVENANCE.md). They differ by design. What the derivation
    # needs is that neither clamps on this fixture.
    print(f"sidecar max_rotor_speed_rad_s: {ceiling!r} against the model's "
          f"{model.maximum_speed_rad_s!r}, different by design (the nominal-voltage ceiling "
          f"against the hover-load one, models/souxmar-quad/PROVENANCE.md); both lie above the "
          f"fixture's {speed_max:.2f}, so neither clamps")
    checks.require(speed_max < ceiling, "the sidecar's speed ceiling would clamp the fixture")
    unchecked = sorted(set(vehicle) - {"mass_kg", "inertia_kg_m2", "thrust_coefficient_n_s2",
                                       "torque_coefficient_nm_s2", "motor_time_constant_s",
                                       "drag_linear_n_s_m", "drag_quadratic_n_s2_m2",
                                       "angular_drag_nm_s", "arm_length_m",
                                       "max_rotor_speed_rad_s"})
    print(f"sidecar keys this derivation does not use, not checked: {', '.join(unchecked)}")
    return hover


def _sidecar_pairs(sidecar, vehicle, model, fixture, step_s, command, hover):
    """Each sidecar figure the derivation uses, beside the value the tool uses."""
    return [("csv_sha256", sidecar["csv_sha256"], fixture.sha256),
             ("dt_s", sidecar["dt_s"], step_s),
             ("samples", sidecar["samples"], command.shape[0]),
             ("hover_rotor_speed_rad_s", sidecar["hover_rotor_speed_rad_s"], hover),
             ("mass_kg", vehicle["mass_kg"], model.mass_kg),
             ("inertia_kg_m2", list(vehicle["inertia_kg_m2"]), list(model.inertia_kg_m2)),
             ("thrust_coefficient_n_s2", vehicle["thrust_coefficient_n_s2"],
              model.thrust_coefficient_n_s2),
             ("torque_coefficient_nm_s2", vehicle["torque_coefficient_nm_s2"],
              model.torque_coefficient_n_m_s2),
             ("motor_time_constant_s", vehicle["motor_time_constant_s"], model.time_constant_s),
             ("drag_linear_n_s_m", list(vehicle["drag_linear_n_s_m"]),
              list(model.drag_linear_n_s_m)),
             ("drag_quadratic_n_s2_m2", list(vehicle["drag_quadratic_n_s2_m2"]),
              list(model.drag_quadratic_n_s2_m2)),
             ("angular_drag_nm_s", list(vehicle["angular_drag_nm_s"]),
              list(model.drag_angular_n_m_s)),
             ("arm_length_m / sqrt(2)", vehicle["arm_length_m"] / np.sqrt(2), model.hub_offset_m)]


# =============================================================================================
# PART 1. A PRIORI
# =============================================================================================

@dataclass(frozen=True)
class Transient:
    """One command change: the fixture row whose command first differs, and its pattern."""

    row: int
    change: np.ndarray
    kind: str
    magnitude: float


def command_transients(fixture: Fixture) -> list[Transient]:
    """Every command change, classified by its rotor sign pattern."""
    command = fixture.stack(COMMAND_COLUMNS)
    transients = []
    for row in range(1, command.shape[0]):
        change = command[row] - command[row - 1]
        if not np.any(change != 0.0):
            continue
        magnitude = float(np.abs(change).max())
        if np.abs(np.abs(change) / magnitude - 1).max() > 1e-12:
            raise InputError(f"row {row}: the four rotors change by different amounts")
        signs = tuple(int(s) for s in np.sign(change))
        kind = next((name for name, pattern in PATTERNS.items()
                     if signs == pattern or signs == tuple(-p for p in pattern)), None)
        if kind is None:
            raise InputError(f"row {row}: pattern {signs} is none of pitch, roll, yaw, collective")
        transients.append(Transient(row, change, kind, magnitude))
    if not transients:
        raise InputError(f"{fixture.path}: the commands never change")
    return transients


def rk4_propagator(stage_matrices, h: float) -> np.ndarray:
    """RK4's one-step map of a linear system whose matrix is sampled at the four stages."""
    identity = np.eye(stage_matrices[0].shape[0])
    k1 = stage_matrices[0]
    k2 = stage_matrices[1] @ (identity + 0.5 * h * k1)
    k3 = stage_matrices[2] @ (identity + 0.5 * h * k2)
    k4 = stage_matrices[3] @ (identity + h * k3)
    return identity + (h / 6.0) * (k1 + 2.0 * (k2 + k3) + k4)


def quaternion_rate_matrix(omega: np.ndarray) -> np.ndarray:
    """M with M q = q (x) [0, omega] / 2, the kinematics both steppers integrate."""
    p, q, r = omega
    return 0.5 * np.array([[0.0, -p, -q, -r], [p, 0.0, r, -q], [q, -r, 0.0, p],
                           [r, q, -p, 0.0]])


def part1(model: Model, fixture: Fixture, plant: Plant, hover: float, gates: dict[str, Gate],
          checks: Checks) -> dict:
    """The a-priori bounds. Returns them per row for the pointwise checks in Part 2."""
    print("\n=== PART 1: a-priori derivation, and the items 3-4 one-step defects measured against "
          "the fixture ===")
    h, tau = plant.step_s, model.time_constant_s
    ratio = h / tau                                   # a
    exact = math.exp(-ratio)                          # E, Souxmar's per-step decay
    half = math.exp(-ratio / 2)                       # E^(1/2), its half-step sample
    rk4 = 1 - ratio + ratio * ratio / 2 - ratio ** 3 / 6 + ratio ** 4 / 24   # R, RK4's decay
    print(f"a=h/tau={ratio:.10f}  E=e^-a={exact:.12f}  R_RK4={rk4:.12f}  "
          f"(R-E)/E={(rk4 - exact) / exact:.6e}")

    # Stage values of the lag error, as multiples of e_k = x_k - c (c the target).
    galata_stage = [1.0, 1 - ratio / 2, 1 - ratio / 2 + ratio * ratio / 4,
                    1 - ratio + ratio * ratio / 2 - ratio ** 3 / 4]
    souxmar_stage = [1.0, half, half, exact]
    stage_gap = [galata_stage[i] - souxmar_stage[i] for i in range(4)]
    print("stage deviation galata-minus-Souxmar per unit e_k:", ["%.4e" % x for x in stage_gap])
    rk4_mean = (galata_stage[0] + 2 * galata_stage[1] + 2 * galata_stage[2] + galata_stage[3]) / 6
    simpson_mean = (1 + 4 * half + exact) / 6
    print(f"W=(1-R)/a check {rk4_mean:.12f} vs {(1 - rk4) / ratio:.12f};  Simpson mean "
          f"{simpson_mean:.12f};  V=W-Simp {rk4_mean - simpson_mean:.4e}")

    # TELESCOPING. galata: h * W * e_k = tau * (e_k - e_{k+1}) exactly, since e_{k+1} = R e_k and
    # W = (1-R)/a. Souxmar: h * Simpson(e) = sigma * tau * (e_k - e_{k+1}). Summed over a
    # transient, galata's rate increments return to zero with the rotor difference; Souxmar's
    # leave (sigma - 1) * tau * (change).
    sigma = ratio * simpson_mean / (1 - exact)
    print(f"sigma-1 (Souxmar's Simpson excess on the exact exponential) = {sigma - 1:.6e}"
          f"   [a^4/2880 = {ratio ** 4 / 2880:.6e}]")
    print(f"identity check h*W == tau*(1-R): {h * rk4_mean:.15e} {tau * (1 - rk4):.15e}")

    # Every closed form is tabulated to the end of the run, so no transient is cut off.
    table_steps = fixture.columns["time_s"].shape[0]
    steps = np.arange(table_steps + 1)
    rk4_powers, exact_powers = rk4 ** steps, exact ** steps
    gap = rk4_powers - exact_powers                  # rotor difference per unit change
    peak = int(np.argmax(gap))
    print(f"max_N |R^N-E^N| = {gap.max():.6e} at N={peak};  sum_N (R^N-E^N) = {gap.sum():.6e}"
          f"  closed form (R-E)/((1-R)(1-E)) = {(rk4 - exact) / ((1 - rk4) * (1 - exact)):.6e}")
    print(f"  the old banner's 1.79e-7 is (R-E)/E at 3 significant figures, "
          f"{(rk4 - exact) / exact:.2e}; no output of the programme states it")
    if fixture.bridge is None:
        print("  bridge_check.json absent: the old banner's 5.8e-7 is not traced here")
    else:
        climb, pitch = fixture.bridge["climb"], fixture.bridge["pitch"]
        print(f"  the old banner's 5.8e-7 is the programme's bridge_check.json, linear_check, "
              f"relative rotor error: climb {climb[0]!r}, pitch {pitch[0]!r}. It is a MEASURED "
              f"error of its linear model against its nonlinear plant after rotor steps of "
              f"{climb[1]:.3g} and {pitch[1]:.3g} rad/s, and agrees with max|R^N-E^N| to "
              f"{abs(climb[0] / gap.max() - 1):.1e} relative. It is not a bound on a transient")

    # ---- rotor: exact, linear and decoupled ----
    transients = command_transients(fixture)
    time = fixture.columns["time_s"]
    samples = time.shape[0]
    print("command changes (row, per-rotor change, pattern):")
    for transient in transients:
        print(f"   row {transient.row:4d} t={time[transient.row]:.3f}  "
              f"d={np.round(transient.change, 6)}  {transient.kind}")
    rotor_gap = np.zeros((samples, 4))
    for transient in transients:
        after = np.arange(samples) - transient.row          # N at each row
        live = after >= 0
        rotor_gap[live] += np.outer(gap[after[live]], transient.change)
    rotor_bound = np.linalg.norm(rotor_gap, axis=1)
    largest = max(np.linalg.norm(t.change) for t in transients)
    print(f"rotor, exact: max over rows of |sum over changes (R^N-E^N) d| = {rotor_bound.max():.6e}"
          f" rad/s  (= max|R^N-E^N| x largest 4-rotor change {largest:.6f} = "
          f"{gap.max() * largest:.6e});  gate {gates['rotor'].literal} is "
          f"{gates['rotor'].value / rotor_bound.max():.1f}x")
    print(f"  the old banner took 5.8e-7 as a bound and multiplied it by the hover speed: 5.8e-7 x "
          f"{hover:.2f} = {5.8e-7 * hover:.3e}. The rotor term scales with the change, not the "
          "hover speed, and that product bounds nothing the case compares")

    # ---- sensitivities, first order; verified against the model's geometry per change ----
    J, K = model.inertia_kg_m2[0], model.inertia_kg_m2[2]
    a_hub = model.hub_offset_m
    kT, kQ = model.thrust_coefficient_n_s2, model.torque_coefficient_n_m_s2
    sensitivity_pq = 4 * a_hub * 2 * kT * hover / J       # rad/s^2 per rad/s of per-rotor change
    sensitivity_r = 4 * 2 * kQ * hover / K
    sensitivity_thrust = 4 * 2 * kT * hover / model.mass_kg
    print(f"s_pq = {sensitivity_pq:.6f} 1/s,  s_r = {sensitivity_r:.6f} 1/s,  s_T(collective) = "
          f"{sensitivity_thrust:.6f} m/s^2 per rad/s")
    for transient in transients:
        thrust_change = 2 * kT * hover * transient.change
        moment = sum(np.cross(model.hub_frd_m[i], [0.0, 0.0, -thrust_change[i]]) for i in range(4))
        moment[2] += np.sum(-model.spin * 2 * kQ * hover * transient.change)
        channels = {"pq": np.hypot(moment[0], moment[1]) / J, "r": abs(moment[2]) / K,
                    "thrust": abs(thrust_change.sum()) / model.mass_kg}
        own = {"pitch": "pq", "roll": "pq", "yaw": "r", "collective": "thrust"}[transient.kind]
        scale = {name: value * transient.magnitude for name, value in
                 (("pq", sensitivity_pq), ("r", sensitivity_r), ("thrust", sensitivity_thrust))}
        leak = max(channels[name] / scale[name] for name in channels if name != own)
        checks.require(abs(channels[own] / scale[own] - 1) < 1e-12 and leak < 1e-9,
                       f"row {transient.row}: the {transient.kind} change does not excite only "
                       "its own channel with the closed-form sensitivity")

    # Per-unit-change sequences for one transient, N = steps since the change took effect
    # (the rotor at row + N; the rate error at row + N; the in-step angle of the step from it).
    rate_unit = tau * (gap + (sigma - 1) * (1 - exact_powers))
    recursion = np.zeros(table_steps + 1)
    for n in range(table_steps):
        recursion[n + 1] = recursion[n] + h * (-rk4_mean * rk4_powers[n]
                                               + simpson_mean * exact_powers[n])
    print(f"telescoped rate closed form vs direct recursion: max|diff| = "
          f"{np.abs(recursion - rate_unit).max():.3e}  (peak {rate_unit.max():.6e} s at "
          f"N={int(np.argmax(rate_unit))}, plateau {tau * (sigma - 1):.6e} s)")
    # SUMMATION BY PARTS. The telescoping above sums one transient's rate forcing f_n as if no
    # dynamics acted between steps. They do: the error is carried from step to step by the
    # step propagator Phi_n. Split f_n into a part whose partial sums are tau (R^n - E^n) and
    # sum to zero, and the plateau's part tau |sigma-1| (1-E) E^n, which is one-signed. Then
    #   e_N = F_N + sum_{n=1}^{N-1} Phi(N,n+1) (Phi_n - I) F_n     (the zero-sum part)
    # and the plateau part is at most the sum of its absolute values. With |Phi| <= 1 (checked
    # on the fixture below) and |Phi_n - I| <= e^(h|A|) - 1 (RK4's tableau is non-negative, so
    # its polynomial in stage Jacobians of norm <= |A| is at most R(h|A|) - 1), the closed form
    # is short by at most tau (e^(h|A|) - 1) sum_{n<N} (R^n - E^n) per unit change. That
    # between-step remainder is charged at every N to the end of the run, with no cutoff. Its
    # limit is the closed form tau (e^(h|A|) - 1) (R-E)/((1-R)(1-E)).
    rate_bound_unit = tau * (gap + abs(sigma - 1) * (1 - exact_powers))
    gap_partial = np.concatenate([[0.0], np.cumsum(gap)[:-1]])   # sum_{n<N} (R^n - E^n)
    weight_gap = galata_stage[0] + galata_stage[1] + galata_stage[2]   # of Delta_k in dU1+dU2+dU3
    weight_error = stage_gap[0] + stage_gap[1] + stage_gap[2]         # of e_k
    instep_unit = (h * h / 6) * (weight_gap * gap + weight_error * exact_powers)
    print(f"in-step angle weights: Delta x {weight_gap:.6f}, e_k x {weight_error:.4e};  "
          f"sum over transient {instep_unit.sum():.6e} s^2")
    print(f"per-unit rate-path angle (transient part) {h * tau * gap.sum():.6e} s^2;  signed "
          f"combined {h * tau * gap.sum() - instep_unit.sum():.6e}  (rate path and in-step oppose)")

    # Stage coupling through the rigid-body Jacobian A, charged at the fixture's own rates.
    rates = fixture.stack(STATE_COLUMNS_NED[10:13])
    rate_norm = np.sqrt(rates[:, 0] ** 2 + rates[:, 1] ** 2 + rates[:, 2] ** 2)
    rate_pq = np.sqrt(rates[:, 0] ** 2 + rates[:, 1] ** 2)
    kappa = (K - J) / J
    angular = model.drag_angular_n_m_s
    jacobian_pq = kappa * np.abs(rates[:, 2]).max() + angular[0] / J
    jacobian_r = kappa * rate_pq.max() + angular[2] / K
    jacobian_norm = kappa * rate_norm.max() + max(angular[0] / J, angular[2] / K)
    coupling_unit = (h * h / 6) * (weight_gap * gap + np.abs(weight_error) * exact_powers)
    print(f"fixture envelope: max|omega| {rate_norm.max():.4f}, max|omega_pq| {rate_pq.max():.4f}, "
          f"max|r| {np.abs(rates[:, 2]).max():.4f}")
    print(f"|A|_pq <= kappa*max|r| + D/J = {jacobian_pq:.4f} 1/s;  "
          f"|A|_r <= kappa*max|w_pq| + D/K = {jacobian_r:.4f};  (full-norm {jacobian_norm:.3f})")
    pq_change = max((t.magnitude for t in transients if t.kind in ("pitch", "roll")), default=0.0)
    yaw_change = max((t.magnitude for t in transients if t.kind == "yaw"), default=0.0)
    print(f"stage-coupling RATE allowance per transient: pq "
          f"{coupling_unit.sum() * jacobian_pq * sensitivity_pq * pq_change:.3e} rad/s, yaw "
          f"{coupling_unit.sum() * jacobian_r * sensitivity_r * yaw_change:.3e} rad/s")

    # The between-step propagator of the pitch/roll rate error. With J_xx = J_yy and equal x and
    # y angular drag, Euler's equations linearise to A = [[-D/J, -kappa r], [kappa r, -D/J]]:
    # a rotation at kappa r, damped. RK4 applies it with r at the stage times, taken here from
    # the rows and their mean. The yaw error's propagator is the scalar R(-h D/K). The angle
    # error is carried by RK4's map of the quaternion kinematics, checked the same way.
    damping_pq, damping_r = angular[0] / J, angular[2] / K
    worst_norm, worst_change, worst_attitude = 0.0, 0.0, 0.0
    for row in range(samples - 1):
        middle = 0.5 * (rates[row] + rates[row + 1])
        at_stages = (rates[row], middle, middle, rates[row + 1])
        phi = rk4_propagator([np.array([[-damping_pq, -kappa * w[2]], [kappa * w[2], -damping_pq]])
                              for w in at_stages], h)
        worst_norm = max(worst_norm, float(np.linalg.norm(phi, 2)))
        worst_change = max(worst_change, float(np.linalg.norm(phi - np.eye(2), 2)))
        attitude_phi = rk4_propagator([quaternion_rate_matrix(w) for w in at_stages], h)
        worst_attitude = max(worst_attitude, float(np.linalg.norm(attitude_phi, 2)))
    print(f"attitude error propagator over the fixture: max|Phi| - 1 = {worst_attitude - 1:.1e} "
          "(round-off: RK4 on a rotation does not amplify)")
    checks.require(worst_attitude <= 1 + 8 * np.finfo(float).eps,
                   "the attitude error propagator amplifies on the fixture's rates")
    yaw_phi = 1 - damping_r * h + (damping_r * h) ** 2 / 2 - (damping_r * h) ** 3 / 6 \
        + (damping_r * h) ** 4 / 24
    step_pq, step_r = math.expm1(h * jacobian_pq), math.expm1(h * damping_r)
    print(f"between-step propagator over the fixture: pitch/roll max|Phi| {worst_norm:.10f}, "
          f"max|Phi - I| {worst_change:.4e} <= e^(h|A|_pq)-1 = {step_pq:.4e};  yaw Phi = "
          f"{yaw_phi:.10f}, |Phi - 1| {1 - yaw_phi:.4e} <= e^(h D/K)-1 = {step_r:.4e}")
    checks.require(worst_norm <= 1.0 and worst_change <= step_pq and 0.0 < yaw_phi <= 1.0
                   and 1 - yaw_phi <= step_r,
                   "a between-step propagator is not contracting, or exceeds its charged change")
    limit = tau * (rk4 - exact) / ((1 - rk4) * (1 - exact))
    print(f"between-step remainder per transient, closed-form limit: pq "
          f"{sensitivity_pq * pq_change * step_pq * limit:.3e} rad/s, yaw "
          f"{sensitivity_r * yaw_change * step_r * limit:.3e} rad/s")

    # |dU1| + |dU2| per unit change: the stage rotor differences the Jacobian acts on in-step.
    stage_unit = (galata_stage[0] + galata_stage[1]) * gap \
        + (abs(stage_gap[0]) + abs(stage_gap[1])) * exact_powers

    # ---- schedule bounds: each transient by its own closed form, summed in absolute value.
    #      N counts steps from the row whose command first differs: the rotor difference first
    #      appears one row later as (R-E) d, and so does the rate difference. ----
    bound_pq, bound_r = np.zeros(samples), np.zeros(samples)
    bound_instep, bound_thrust = np.zeros(samples), np.zeros(samples)
    stage_pq, stage_r = np.zeros(samples), np.zeros(samples)
    for transient in transients:
        after = np.arange(samples) - transient.row
        started = after >= 0
        n = np.clip(after, 0, None)
        d = transient.magnitude
        rate = np.where(started, rate_bound_unit[n], 0.0) * d
        remainder = np.where(started, tau * gap_partial[n], 0.0) * d
        instep = np.where(started, instep_unit[n], 0.0) * d
        coupling = np.concatenate(
            [[0.0], np.cumsum(np.where(started, coupling_unit[n], 0.0))[:-1]]) * d
        stage = np.where(started, stage_unit[n], 0.0) * d
        if transient.kind in ("pitch", "roll"):
            bound_pq += sensitivity_pq * (rate + step_pq * remainder + jacobian_pq * coupling)
            bound_instep += sensitivity_pq * instep
            stage_pq += sensitivity_pq * stage
        elif transient.kind == "yaw":
            bound_r += sensitivity_r * (rate + step_r * remainder + jacobian_r * coupling)
            bound_instep += sensitivity_r * instep
            stage_r += sensitivity_r * stage
        else:
            bound_thrust += sensitivity_thrust * rate
    # A yaw-rate error feeds pitch and roll through kappa * (-q, p) * dr. That forcing is NOT
    # telescoped: it is summed in absolute value and carried by the pitch/roll propagator, whose
    # norm is checked above to stay at or below 1. Each step charges the larger endpoint of
    # |omega_pq| and of the yaw bound, never the left one alone.
    feed = np.concatenate([[0.0], np.cumsum(h * kappa * np.maximum(rate_pq[:-1], rate_pq[1:])
                                            * np.maximum(bound_r[:-1], bound_r[1:]))])
    bound_rate = np.sqrt((bound_pq + feed) ** 2 + bound_r ** 2)
    # The ANGLE a step adds. With stage rates w_i = w + c_i h a_i and a_i = S U_i + A w_i (A the
    # Euler Jacobian), one step's angle difference is, to first order in the defect,
    #   h dw + (h^2/6) S (dU1+dU2+dU3) + (h^2/2) A dw + (h^3/12) A (da1 + da2),
    # |da1| + |da2| <= (1 + h|A|/2) (S (|dU1| + |dU2|) + 2 |A dw|).
    # The first two terms are the rate path and the in-step term. The last two are charged
    # here. Without them the bound falls below the reimplementation on the first row of each
    # transient, where the stage cancellation leaves the in-step term smallest.
    jacobian_on_rate = jacobian_pq * (bound_pq + feed) + jacobian_r * bound_r
    coupling_angle = ((h * h / 2) * jacobian_on_rate
                      + (h ** 3 / 12) * (1 + h * jacobian_norm / 2)
                      * (jacobian_pq * stage_pq + jacobian_r * stage_r
                         + 2 * jacobian_norm * jacobian_on_rate))
    bound_angle = np.concatenate([[0.0], np.cumsum(h * bound_rate[:-1] + bound_instep[:-1]
                                                   + coupling_angle[:-1])])
    print(f"in-step Jacobian terms on the angle: {coupling_angle[:-1].sum():.3e} rad over the run "
          f"(rate path {h * bound_rate[:-1].sum():.3e}, in-step {bound_instep[:-1].sum():.3e})")
    print("\nNO-PAIR-CANCELLATION bounds, first order (each transient by its own telescoped closed "
          "form plus its between-step remainder, summed in absolute value across transients):")
    print(f"  body rate  <= {bound_rate[1:].max():.4e} rad/s   "
          f"(gate {gates['body rate'].literal}: "
          f"{gates['body rate'].value / bound_rate[1:].max():.1f}x)")
    print(f"  attitude   <= {bound_angle[1:].max() / 2:.4e} (metric = half the angle; angle "
          f"{bound_angle[1:].max():.4e} rad)   (gate {gates['attitude'].literal}: "
          f"{gates['attitude'].value / (bound_angle[1:].max() / 2):.2f}x)")

    # ---- position: dV' <= a_th |dtheta| + |dT|/m, the drag Jacobian dropped as dissipative ----
    reference = reference_from_fixture(fixture)
    rotor = fixture.stack(ROTOR_COLUMNS)
    slope = np.zeros(samples)
    air_speed = np.zeros(samples)
    for row in range(samples):
        air = air_body_state(reference, row)[3:6]
        force = (np.array([0.0, 0.0, -kT * (rotor[row] ** 2).sum()]) - model.drag_linear_n_s_m * air
                 - model.drag_quadratic_n_s2_m2 * air * np.abs(air))
        steepest = (model.drag_linear_n_s_m + 2 * model.drag_quadratic_n_s2_m2 * np.abs(air)).max()
        slope[row] = (np.linalg.norm(force) + steepest * np.linalg.norm(air)) / model.mass_kg
        air_speed[row] = np.linalg.norm(air)
    print(f"a_th along fixture: min {slope.min():.3f} max {slope.max():.3f} m/s^2 per rad")

    # An ESTIMATE, not a bound: each step takes the larger endpoint of the slope, the angle and
    # the velocity, but the integrand is sampled at rows, not bounded over the step. The thrust
    # term is the telescoped velocity increment of each collective change, a velocity already.
    def propagate(angle, thrust_velocity, extra_velocity=None, extra_position=None):
        steps_in = h * np.maximum(slope[:-1], slope[1:]) * np.maximum(angle[:-1], angle[1:])
        velocity = np.concatenate([[0.0], np.cumsum(steps_in)]) + thrust_velocity
        if extra_velocity is not None:
            velocity = velocity + extra_velocity
        position = np.concatenate([[0.0], np.cumsum(h * np.maximum(velocity[:-1], velocity[1:]))])
        if extra_position is not None:
            position = position + extra_position
        return velocity, position

    velocity_scheme, position_scheme = propagate(bound_angle, bound_thrust)
    print(f"  position (rotor-scheme part), estimate {position_scheme.max():.4e} m   ground "
          f"velocity, estimate {velocity_scheme.max():.4e} m/s")

    # ---- items 3-4: per-step defects of galata's rigid-body RK4 from each FIXTURE state, with
    #      Souxmar's exact rotor samples on both sides ----
    ground = reference.ground_velocity
    defect_velocity, defect_position, defect_angle = (np.zeros(samples) for _ in range(3))
    for row in range(samples - 1):
        stepped, _ = plant.galata_step(air_body_state(reference, row), rotor[row],
                                       reference.command[row], reference.wind_ned[row], "exact")
        stepped_ground = dcm(stepped[6:10]) @ stepped[3:6] + reference.wind_ned[row]
        defect_velocity[row + 1] = np.linalg.norm(stepped_ground - ground[row + 1])
        defect_position[row + 1] = np.linalg.norm(stepped[:3] - reference.position[row + 1])
        defect_angle[row + 1] = 2 * quaternion_distance(stepped[6:10],
                                                        reference.quaternion[row + 1])
    print("\nitems 3-4: one-step defects MEASURED against the fixture (the galata-scheme "
          "reimplementation stepped from each fixture row with exact rotor inputs, differenced "
          "with the fixture's next row):")
    print(f"  max |dV| {defect_velocity.max():.3e} m/s, sum {defect_velocity.sum():.3e};  max |dp| "
          f"{defect_position.max():.3e} m, sum {defect_position.sum():.3e};  max angle "
          f"{defect_angle.max():.3e} (round-off: rotation is autonomous)")
    print(f"  order estimate (h*max|w|)^5/120*max|v_b| = "
          f"{(h * rate_norm.max()) ** 5 / 120 * air_speed.max():.3e} m/s per step; max|v_b| "
          f"{air_speed.max():.3f}")
    extra_velocity, extra_position = np.cumsum(defect_velocity), np.cumsum(defect_position)
    _, position_items34 = propagate(np.zeros(samples), np.zeros(samples), extra_velocity,
                                    extra_position)
    print(f"  items 3-4 alone, propagated without the dissipative drag term: position estimate "
          f"{position_items34.max():.3e} m")
    _, bound_position = propagate(bound_angle, bound_thrust, extra_velocity, extra_position)
    print(f"  POSITION total ESTIMATE (no pair cancellation; not proven, checked against the "
          f"reimplementation at every row in Part 2) {bound_position.max():.4e} m  "
          f"(gate {gates['position'].literal}: "
          f"{gates['position'].value / bound_position.max():.1f}x)")

    bounds = {"rotor": rotor_bound, "attitude": bound_angle / 2, "body rate": bound_rate,
              "position": bound_position}
    kinds = {"rotor": "exact", "attitude": "bound", "body rate": "bound",
             "position": "estimate, checked at every row"}
    print("\nGATES against Part 1 (gate / figure; a gate at or below its figure fails):")
    for quantity in GATE_ORDER:
        gate = gates[quantity]
        bound = float(bounds[quantity][1:].max())
        print(f"  {quantity:9s} gate {gate.literal} "
              f"({TEST_SOURCE.relative_to(REPO_ROOT)}:{gate.line})  {kinds[quantity]} "
              f"{bound:.4e}  headroom {gate.value / bound:.2f}x")
        checks.require(gate.value > bound,
                       f"the {quantity} gate is at or below its Part 1 figure ({kinds[quantity]})")
    print(f"validity envelope used: max|omega| {rate_norm.max():.4f} rad/s, max|v_b| "
          f"{air_speed.max():.3f} m/s, a_th <= {slope.max():.3f} m/s^2 per rad")
    return {"bounds": bounds, "transients": transients, "peak": peak}


# =============================================================================================
# PART 2. CHECKS
# =============================================================================================

def part2(plant: Plant, fixture: Fixture, part1_result: dict, gates: dict[str, Gate],
          checks: Checks) -> None:
    """Reimplementations against the fixture, the decomposition, and the counterfactuals."""
    print("\n=== PART 2: checks (run after the bounds above) ===")
    commands = fixture.stack(COMMAND_COLUMNS)
    states, rotors, _ = souxmar_run(plant, fixture, commands)
    ned_rows = []
    for state in states:
        ned_rows.append(np.r_[ENU_TO_NED @ state[0:3], ENU_TO_NED @ state[3:6],
                              quaternion_from_matrix(ENU_TO_NED @ dcm_normalised(state[6:10])
                                                     @ FLU_TO_FRD),
                              FLU_TO_FRD @ state[10:13]])
    mine = np.hstack([states, np.array(ned_rows), rotors])
    theirs = np.hstack([fixture.stack(STATE_COLUMNS_ENU), fixture.stack(STATE_COLUMNS_NED),
                        fixture.stack(ROTOR_COLUMNS)])
    difference = float(np.abs(mine - theirs).max())
    print(f"Souxmar reimplementation vs fixture, max |diff| over all {theirs.shape[0]} rows and "
          f"{theirs.shape[1]} recorded state columns (ENU/FLU, NED/FRD, rotor): {difference:.3e}")
    checks.require(difference == 0.0, "the Souxmar reimplementation is not bit-exact")

    reference = reference_from_fixture(fixture)
    full = galata_versus(plant, reference, "rk4")
    print("galata-scheme reimpl vs fixture:             " + describe(full.worst()))
    items34 = galata_versus(plant, reference, "exact")
    print("items 3-4 only (exact rotor into galata RB):  " + describe(items34.worst()))
    _, _, souxmar_rk4 = souxmar_run(plant, fixture, commands, lag="rk4")
    items12 = {"rotor": float(np.linalg.norm(souxmar_rk4.rotor - reference.rotor, axis=1).max()),
               "attitude": max(quaternion_distance(souxmar_rk4.quaternion[r],
                                                   reference.quaternion[r])
                               for r in range(1, commands.shape[0])),
               "body rate": float(np.linalg.norm(souxmar_rk4.rate - reference.rate, axis=1).max()),
               "position": float(np.linalg.norm(souxmar_rk4.position - reference.position,
                                                axis=1).max())}
    print("items 1-2 only (RK4 rotor in Souxmar coords): " + describe(items12))

    time = fixture.columns["time_s"][1:]
    print(f"where: attitude max at t={time[np.argmax(full.attitude)]:.3f}, rate max at "
          f"t={time[np.argmax(full.rate)]:.3f}, position max at "
          f"t={time[np.argmax(full.position)]:.3f}")
    first = part1_result["transients"][0].row
    bounds = part1_result["bounds"]
    for row in (first + part1_result["peak"], first + 62, first + 122, first + 150, first + 250):
        print(f"   t={time[row - 1]:.3f}: rate err {full.rate[row - 1]:.3e}   attitude "
              f"{full.attitude[row - 1]:.3e}   bound(no-pair) rate {bounds['body rate'][row]:.3e}")

    # Round-off allowance for the row-by-row comparison only; it is never added to a bound. One
    # rounding of the state per step in each implementation, doubled, for every step of the run,
    # at the largest magnitude that quantity takes in the fixture.
    steps = reference.command.shape[0] - 1
    magnitude = {"rotor": np.abs(reference.rotor).max(), "attitude": 1.0,
                 "body rate": np.abs(reference.rate).max(),
                 "position": np.abs(reference.position).max()}
    print("pointwise: the galata-scheme reimplementation against the Part 1 figure at every row "
          "(smallest (bound + round-off allowance) / measured, and where):")
    measured = {"rotor": full.rotor, "attitude": full.attitude, "body rate": full.rate,
                "position": full.position}
    for quantity in GATE_ORDER:
        allowance = 4 * steps * float(np.spacing(magnitude[quantity]))
        ratio = (bounds[quantity][1:] + allowance) / np.maximum(measured[quantity],
                                                                np.finfo(float).tiny)
        worst_row = int(np.argmin(ratio))
        print(f"  {quantity:9s} {ratio[worst_row]:.4f} at t={time[worst_row]:.3f}  "
              f"(allowance {allowance:.1e})")
        checks.require(ratio[worst_row] >= 1.0,
                       f"the {quantity} bound is exceeded by the galata-scheme reimplementation")

    # ---- COUNTERFACTUAL 1: every pulse held instead of undone ----
    transients = part1_result["transients"]
    held_commands = np.repeat(commands[:1], commands.shape[0], axis=0)
    undone = set()   # a change that exactly reverses an earlier one is dropped
    for index, transient in enumerate(transients):
        if index in undone:
            continue
        held_commands[transient.row:] += transient.change
        partner = next((j for j in range(index + 1, len(transients)) if j not in undone
                        and np.array_equal(transients[j].change, -transient.change)), None)
        if partner is not None:
            undone.add(partner)
    _, _, held = souxmar_run(plant, fixture, held_commands)
    held_rows = range(held.rate.shape[0])
    held_rate = np.linalg.norm(held.rate, axis=1).max()
    held_ground = np.linalg.norm(held.ground_velocity, axis=1).max()
    held_air = max(np.linalg.norm(air_body_state(held, r)[3:6]) for r in held_rows)
    print(f"\nCOUNTERFACTUAL (not the fixture): every pulse held to the end instead of undone; "
          f"max|omega| {held_rate:.1f} rad/s, max|V| {held_ground:.1f} m/s, "
          f"max|v_b| {held_air:.1f} m/s")
    held_full = galata_versus(plant, held, "rk4").worst()
    held_items34 = galata_versus(plant, held, "exact").worst()
    print("  galata-scheme vs Souxmar-scheme:   " + describe(held_full))
    print("  items 3-4 only:                    " + describe(held_items34))
    _, _, held_rk4 = souxmar_run(plant, fixture, held_commands, lag="rk4")
    held_attitude = max(quaternion_distance(held_rk4.quaternion[r], held.quaternion[r])
                        for r in held_rows)
    held_position = np.linalg.norm(held_rk4.position - held.position, axis=1).max()
    print(f"  items 1-2 only: attitude {held_attitude:.4e} position {held_position:.4e}")
    position_gate = gates["position"]
    verdict = "FAILS" if held_full["position"] >= position_gate.value else "holds"
    print(f"  the position gate {position_gate.literal} {verdict} on this schedule, outside the "
          "fixture's envelope; the Part 1 bounds do not apply to it")

    # ---- COUNTERFACTUAL 2: galata's lag time constant wrong by a fraction of itself ----
    print("\nCOUNTERFACTUAL (not galata): the galata scheme with tau*(1+rel), against the fixture")
    cache: dict[float, dict[str, float]] = {0.0: full.worst()}

    def worst_at(relative: float) -> dict[str, float]:
        if relative not in cache:
            cache[relative] = galata_versus(plant, reference, "rk4",
                                            plant.tau * (1 + relative)).worst()
        return cache[relative]

    for relative in (2.9e-4, 4.0e-5, 1.5e-5):
        print(f"  galata with tau*(1+{relative:g}): " + describe(worst_at(relative)))
    # The smallest |rel| that takes each metric to its gate: two secant steps through the rel=0
    # value from a starting probe, a fixed count so the output does not depend on a tolerance.
    probes = {"rotor": 2.9e-4, "attitude": 1.5e-5, "body rate": 4.0e-5, "position": 2.9e-4}
    print("  |rel| at which each gate first fails (both signs; metric there):")
    for quantity in GATE_ORDER:
        found = []
        for sign in (1.0, -1.0):
            relative = sign * probes[quantity]
            for _ in range(2):
                baseline, metric = cache[0.0][quantity], worst_at(relative)[quantity]
                relative = relative * (gates[quantity].value - baseline) / (metric - baseline)
            found.append(f"{relative:+.2e} ({worst_at(relative)[quantity]:.4e})")
        print(f"    {quantity:9s} gate {gates[quantity].literal}: {'  '.join(found)}")


def main(argv=None) -> int:
    """Parse the arguments, derive, check, and return the exit status."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--fixture-dir", required=True, type=Path,
                        help="directory holding reference_trajectory.csv")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL,
                        help="galata model file (default: the repository's souxmar-quad.yaml)")
    arguments = parser.parse_args(argv)
    try:
        model = load_model(arguments.model)
        gravity = read_standard_gravity(CONSTANTS_HEADER)
        gates = read_gates(TEST_SOURCE)
        fixture = load_fixture(arguments.fixture_dir)
        time = fixture.columns["time_s"]
        step_s = float(time[1] - time[0])
        if not np.all(np.diff(time) > 0) or np.abs(np.diff(time) - step_s).max() > 1e-12:
            raise InputError("the fixture's time column is not uniformly stepped")
        print(f"fixture {fixture.path} sha256 {fixture.sha256}")
        print(f"model {model.label} sha256 {model.sha256}")
        print(f"gravity {gravity!r} m/s^2 ({CONSTANTS_HEADER.relative_to(REPO_ROOT)}); "
              f"numpy {np.__version__}, Python {platform.python_version()}")
        print(f"steps {time.shape[0] - 1} h {step_s}")
        checks = Checks()
        plant = Plant(model, gravity, step_s)
        hover = check_model_against_fixture(model, fixture, gravity, step_s, checks)
        result = part1(model, fixture, plant, hover, gates, checks)
        part2(plant, fixture, result, gates, checks)
    except InputError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(f"\n{len(checks.failures)} check(s) failed" if checks.failures else "\nall checks hold")
    return 1 if checks.failures else 0


if __name__ == "__main__":
    sys.exit(main())
