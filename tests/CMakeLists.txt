# SPDX-License-Identifier: Apache-2.0
#
# Test tiers, per docs/TESTING.md:
#   unit/        one algorithm at a time, against closed-form answers
#   integration/ one pipeline stage at a time, against a frozen contract
#   validation/  against published reference data (docs/VERIFICATION.md)
#   determinism/ bit-identical repeat runs, and cross-platform agreement
#
# Only the tiers that exist are added. A directory appears here in the same
# commit that adds its first test, never before.

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_subdirectory(unit)
