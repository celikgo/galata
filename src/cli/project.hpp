// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_CLI_PROJECT_HPP
#define GALATA_CLI_PROJECT_HPP
#include <string>
#include <vector>
// Experimental local project/worker protocol; see ADR-0012.
int project_command(const std::vector<std::string>& arguments);
#endif
