// SPDX-License-Identifier: Apache-2.0
//
// The galata command-line interface.
//
// Everything the desktop application will do, this does first, through the same
// pipeline document. That ordering is deliberate: a capability reachable only
// through a graphical surface is a capability that cannot be run in CI, cannot
// be scripted, and cannot be reproduced by a reader of a paper.

#include "galata/pipeline/pipeline.hpp"
#include "galata/pipeline/registry.hpp"
#include "galata/version.hpp"

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

int print_usage(std::ostream& out) {
  out << "galata — flight dynamics, control-law design and simulation\n"
         "\n"
         "usage:\n"
         "  galata run <pipeline.yaml> [--output-dir <dir>]\n"
         "  galata capabilities [--markdown]\n"
         "  galata --version\n"
         "  galata --help\n"
         "\n"
         "  run           execute a pipeline, streaming each stage as it completes\n"
         "  capabilities  list what this build can do, and how far each has been checked\n"
         "\n"
         "Relative paths inside a pipeline resolve against the pipeline file's own\n"
         "directory, so a study is runnable from anywhere. Outputs go to the pipeline's\n"
         "directory unless --output-dir says otherwise.\n";
  return 0;
}

int print_version() {
  // Single source of version truth: this string comes from the VERSION file by
  // way of the generated build_config.hpp. See ADR-0005.
  std::cout << galata::build_identification() << "\n";
  return 0;
}

// Markdown, for the README's status table.
//
// Generated rather than written by hand, and checked by CI, because charter
// rule 2 says nothing is documented before it works — and a hand-maintained
// capability table is exactly the thing that quietly starts claiming more than
// the code does.
int list_capabilities_markdown() {
  std::cout << "| Capability | What it does | Produces | State |\n";
  std::cout << "|---|---|---|---|\n";
  for (const galata::pipeline::Capability* capability :
       galata::pipeline::builtin_registry().all()) {
    std::cout << "| `" << capability->id << "` | " << capability->summary << " | `"
              << capability->produces << "` | " << galata::pipeline::to_string(capability->state)
              << " |\n";
  }
  return 0;
}

int list_capabilities() {
  const galata::pipeline::Registry& registry = galata::pipeline::builtin_registry();
  const std::vector<const galata::pipeline::Capability*> all = registry.all();

  std::cout << "galata " << galata::version_string() << " provides " << all.size()
            << " capabilities.\n\n";
  for (const galata::pipeline::Capability* capability : all) {
    std::cout << "  " << capability->id << "\n";
    std::cout << "      " << capability->summary << "\n";
    std::cout << "      produces: " << capability->produces
              << "   state: " << galata::pipeline::to_string(capability->state) << "\n\n";
  }
  std::cout << "\"implemented, unvalidated\" means the capability works and is tested, but\n"
               "its output has not been compared against a published reference. See\n"
               "docs/VERIFICATION.md for what has.\n";
  return 0;
}

int run_pipeline_command(const std::vector<std::string>& arguments) {
  if (arguments.empty()) {
    std::cerr << "galata run: no pipeline file given\n";
    return 2;
  }

  std::string pipeline_path = arguments[0];
  std::string output_directory;

  for (std::size_t i = 1; i < arguments.size(); ++i) {
    if (arguments[i] == "--output-dir") {
      if (i + 1 >= arguments.size()) {
        std::cerr << "galata run: --output-dir needs a directory\n";
        return 2;
      }
      output_directory = arguments[++i];
    } else {
      std::cerr << "galata run: unrecognised argument '" << arguments[i] << "'\n";
      return 2;
    }
  }

  try {
    const std::filesystem::path path(pipeline_path);
    if (!std::filesystem::exists(path)) {
      std::cerr << "galata run: no such file: " << pipeline_path << "\n";
      return 1;
    }
    const std::string base_directory =
        path.has_parent_path() ? path.parent_path().string() : std::string(".");
    if (output_directory.empty()) {
      output_directory = base_directory;
    }

    const galata::pipeline::Pipeline pipeline = galata::pipeline::load_pipeline(pipeline_path);
    std::cout << "galata " << galata::version_string() << " — running " << pipeline_path << " ("
              << pipeline.stages.size() << " stages)\n\n";

    // One line per stage, printed when the stage finishes.
    //
    // Not a carriage-return spinner: this output is redirected to a file or a
    // CI log at least as often as it is watched in a terminal, and a \r that
    // nothing consumes leaves both halves of every line in the transcript.
    const auto progress = [](const std::string& stage_id,
                             const std::string& capability,
                             bool finished,
                             const std::string& summary) {
      if (!finished) {
        return;
      }
      std::cout << "  " << stage_id << "  [" << capability << "]  " << summary << "\n"
                << std::flush;
    };

    const galata::pipeline::RunResult result = galata::pipeline::run_pipeline(
        pipeline, galata::pipeline::builtin_registry(), base_directory, output_directory, progress);

    std::cout << "\n" << result.stages.size() << " stages completed.\n";
    return 0;
  } catch (const std::exception& error) {
    // The message already names the stage and capability; adding a prefix here
    // would only push the useful part further right.
    std::cerr << "\ngalata: " << error.what() << "\n";
    return 1;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> arguments(argv + 1, argv + argc);
  if (arguments.empty()) {
    return print_usage(std::cout);
  }

  const std::string& command = arguments[0];
  if (command == "--help" || command == "-h" || command == "help") {
    return print_usage(std::cout);
  }
  if (command == "--version" || command == "-v" || command == "version") {
    return print_version();
  }
  if (command == "capabilities") {
    if (arguments.size() > 1 && arguments[1] == "--markdown") {
      return list_capabilities_markdown();
    }
    return list_capabilities();
  }
  if (command == "run") {
    return run_pipeline_command(std::vector<std::string>(arguments.begin() + 1, arguments.end()));
  }

  std::cerr << "galata: unrecognised command '" << command << "'\n\n";
  (void)print_usage(std::cerr);
  return 2;
}
