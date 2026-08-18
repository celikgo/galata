// SPDX-License-Identifier: Apache-2.0
//
// The half of the case registry that needs GoogleTest.
//
// Separated so the report generator — which renders the table and has no
// business linking a test framework — can depend on the case data alone.

#ifndef GALATA_TOOLS_VALIDATION_CASE_REGISTRY_CHECK_HPP
#define GALATA_TOOLS_VALIDATION_CASE_REGISTRY_CHECK_HPP

#include "case_registry.hpp"

#include <string>
#include <vector>

namespace galata::validation {

// Every piece of evidence declared to live in `binary`, that does NOT name a
// test registered in the calling process. Empty means all of it is real.
//
// Must be called from inside the binary in question: a process can only
// enumerate the tests linked into it. Each test binary therefore carries one
// test that calls this with its own name, and between them they cover the
// whole registry.
[[nodiscard]] std::vector<std::string> unregistered_evidence(Binary binary);

}  // namespace galata::validation

#endif  // GALATA_TOOLS_VALIDATION_CASE_REGISTRY_CHECK_HPP
