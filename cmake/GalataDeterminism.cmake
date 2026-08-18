# SPDX-License-Identifier: Apache-2.0
#
# Determinism build policy. Read docs/adr/0004-determinism-policy.md before
# changing anything in this file.
#
# Non-negotiable rule 6 of the project charter says determinism is a tested
# property, not an aspiration. Flags are only one half of that; the other half
# is in the numerics (fixed-step integrators, fixed iteration counts, seeded
# PRNGs, ordered containers). This file covers the flags.
#
# These options are applied GLOBALLY rather than through an interface target
# that a numerical target links. That is deliberate: an interface target can be
# forgotten by whoever adds the next library, and a target compiled without
# these flags silently breaks the guarantee for the whole binary it links into.
# There is no opt-out.

include_guard(GLOBAL)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  # -ffp-contract=off forbids the compiler from fusing a*b+c into a single
  # FMA instruction. Contraction changes results because the intermediate
  # product is not rounded, so a contracted expression is *more* accurate than
  # the source says, and whether it happens depends on the target's ISA
  # extensions. GCC defaults to contraction being allowed for C++; Clang has
  # defaulted to allowing it within a statement since Clang 14. Both are
  # overridden here.
  add_compile_options(-ffp-contract=off)

  # -ffast-math and its components (-funsafe-math-optimizations,
  # -ffinite-math-only, -fassociative-math, ...) license the compiler to
  # reassociate floating-point expressions. Reassociation is not
  # value-preserving in IEEE 754, so it destroys reproducibility outright.
  # Passed explicitly rather than merely "not passed" so that a toolchain file
  # or a dependency's usage requirement cannot turn it on behind our back.
  add_compile_options(-fno-fast-math)

elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # /fp:precise is MSVC's value-safe mode: it preserves source-level ordering
  # and does not reassociate. MSVC does not contract into FMA under /fp:precise
  # unless /fp:contract is passed, and /fp:contract is not passed here.
  #
  # /fp:strict would additionally preserve exception and rounding-mode
  # semantics. galata does not manipulate the FP environment, so /fp:strict
  # buys no reproducibility over /fp:precise and costs measurable performance.
  add_compile_options(/fp:precise)
endif()

# ---------------------------------------------------------------------------
# What these flags do NOT buy, stated plainly so nobody reads the determinism
# badge as a stronger claim than it is:
#
#   * They do not make transcendental functions agree across platforms. sin,
#     cos, tan, asin, atan2, exp, log and pow are supplied by the platform's
#     libm — glibc on Linux, Apple's libm on macOS, the UCRT on Windows — and
#     those implementations are not correctly rounded and do not agree with
#     each other in the last bits. sqrt is the exception: IEEE 754 requires it
#     to be correctly rounded, so it is bit-identical everywhere.
#
#   * They therefore do not make cross-platform output bit-identical for any
#     result whose derivation passes through a transcendental. The
#     determinism suite splits on exactly this line: same-platform repeat runs
#     are gated bit-identical, and cross-platform runs are gated against a
#     published ULP bound with the observed spread recorded in
#     docs/VERIFICATION.md.
#
#   * They do not survive a consumer compiling galata into their own build
#     with -ffast-math. Nothing can prevent that; it is documented instead.
#
# long double is banned in the numerical core for a related reason: it is
# 80-bit extended on x86-64 System V, 64-bit on MSVC, and 128-bit quad on
# AArch64 Linux. A result that touches it is not portable by construction.
# ---------------------------------------------------------------------------
