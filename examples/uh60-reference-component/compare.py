#!/usr/bin/env python3
"""Executable analytical component comparison from NASA TM-85890 Table 1."""

import json
import math
from pathlib import Path


config_path = Path(__file__).parent / "reference-aircraft.json"
config = json.loads(config_path.read_text())
p = config["transcribed_inputs"]
radius_m = p["main_rotor_radius_ft"] * 0.3048
weight_n = p["aircraft_weight_lbf"] * 4.4482216152605
disk_area_m2 = math.pi * radius_m * radius_m
solidity = p["main_rotor_blades"] * p["main_rotor_chord_ft"] / (
    math.pi * p["main_rotor_radius_ft"]
)
disk_loading_n_m2 = weight_n / disk_area_m2
tip_speed_m_s = p["main_rotor_omega_rad_s"] * radius_m
solidity_error = abs(solidity - p["main_rotor_solidity"])
passed = solidity_error <= config["acceptance"]["solidity_absolute_tolerance"]
result = {
    "evidence_class": "analytical_verification_of_transcription",
    "reference_aircraft": config["aircraft"],
    "source": config["source"],
    "derived": {
        "radius_m": radius_m,
        "weight_n": weight_n,
        "disk_area_m2": disk_area_m2,
        "disk_loading_n_m2": disk_loading_n_m2,
        "tip_speed_m_s": tip_speed_m_s,
        "recomputed_solidity": solidity,
    },
    "solidity_absolute_error": solidity_error,
    "criteria_passed": passed,
    "souxmar_validation": False,
}
print(json.dumps(result, indent=2, sort_keys=True))
if not passed:
    raise SystemExit("published-reference transcription check failed")
