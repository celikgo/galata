#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Install, relocate, compile and run a consumer without galata's source includes."""
import os
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def run(command, **kwargs):
    print("+ " + " ".join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, **kwargs)


def check(build):
    cache = {}
    for line in (build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if line and not line.startswith(("#", "//")) and "=" in line:
            key, value = line.split("=", 1)
            cache[key.split(":", 1)[0]] = value
    configuration = cache.get("CMAKE_BUILD_TYPE") or "Release"
    dependency_prefix = Path(cache["VCPKG_INSTALLED_DIR"]) / cache["VCPKG_TARGET_TRIPLET"]
    with tempfile.TemporaryDirectory(prefix="galata-install-") as directory:
        scratch = Path(directory)
        original, relocated = scratch / "original", scratch / "relocated"
        run(["cmake", "--install", build, "--prefix", original, "--config", configuration])
        original.rename(relocated)
        for export in relocated.rglob("GalataTargets*.cmake"):
            content = export.read_text(encoding="utf-8")
            for forbidden in (str(build), str(original)):
                if forbidden in content:
                    raise ValueError(f"nonrelocatable path in {export}: {forbidden}")
        consumer = scratch / "consumer"
        consumer.mkdir()
        (consumer / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.24)
project(InstalledConsumer LANGUAGES CXX)
find_package(Galata CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE galata::pipeline)
add_executable(model_consumer model.cpp)
target_link_libraries(model_consumer PRIVATE galata::modeling)
''', encoding="utf-8")
        (consumer / "main.cpp").write_text('''#include <galata/version.hpp>
#include <galata/pipeline/registry.hpp>
#include <galata/modeling/model.hpp>
int main() {
  const auto* model = galata::pipeline::builtin_registry().find("model.compile");
  return model == nullptr || galata::modeling::kProfile.empty()
      || galata::version_string().empty()
      || galata::pipeline::builtin_registry().all().empty();
}
''', encoding="utf-8")
        (consumer / "model.cpp").write_text('''#include <galata/modeling/model.hpp>
#include <galata/modeling/linear_adapter.hpp>
// Every invariant is named and every extent is checked before it is indexed.
// A smoke test that faults instead of reporting which expectation failed cannot
// be diagnosed from a hosted log, where the only evidence is an exit status.
#include <cstdio>
#include <exception>
int failed(const char* invariant) {
  std::fprintf(stderr, "installed-consumer check failed: %s\\n", invariant);
  return 1;
}

int run() {
  namespace m = galata::modeling;
  m::Model source;
  source.blocks = {{"constant", {}, m::Constant{2.0}}, {"output", {}, m::Output{}}};
  source.connections = {{"constant", "output", 0}};
  const auto compiled = m::compile_model(m::parse_model_yaml(m::write_model_yaml(source)));
  const auto result = m::simulate(compiled, {.step_count = 0});
  if (!result.state_ids.empty())
    return failed("a constant-to-output model reported states");
  if (result.outputs.size() != 1)
    return failed("a zero-step run did not record exactly one output sample");
  if (result.outputs.front().size() != 1)
    return failed("the recorded output sample is not one-dimensional");
  if (result.outputs.front()(0) != 2.0)
    return failed("the constant did not reach the output");
  if (result.semantic_sha256.size() != 64)
    return failed("the semantic digest is not a 64-character hex string");

  galata::model::LinearSystem plant;
  plant.a = Eigen::MatrixXd::Zero(1, 1);
  plant.b = Eigen::MatrixXd::Ones(1, 1);
  plant.state_names = {"x"};
  plant.input_names = {"u"};
  m::LinearChannels channels{{m::SignalType{}}, {m::SignalType{}}, {m::SignalType{}}};
  const auto graph = m::lower_linear_system(plant, channels,
      {Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1), {}});
  const auto linear = m::compile_model(graph.model);
  const auto& initial = linear.initial_state();
  std::fprintf(stderr, "lowered graph: %zu states, initial extent %lld\\n",
               linear.state_ids().size(), static_cast<long long>(initial.size()));
  if (linear.state_ids().size() != 1 || initial.size() != 1)
    return failed("lowering xdot = u did not produce exactly one state");
  const auto evaluated = linear.evaluate(0.0, initial);
  if (evaluated.derivatives.size() != 1)
    return failed("the evaluated derivative vector is not one-dimensional");
  if (evaluated.derivatives(0) != 1.0)
    return failed("xdot = u with a unit command did not evaluate to 1");
  return 0;
}

int main() {
  try {
    return run();
  } catch (const std::exception& error) {
    std::fprintf(stderr, "installed-consumer check threw: %s\\n", error.what());
    return 1;
  }
}
''', encoding="utf-8")
        consumer_build = scratch / "consumer-build"
        run(["cmake", "-S", consumer, "-B", consumer_build, "-G", "Ninja",
             f"-DCMAKE_BUILD_TYPE={configuration}",
             f"-DCMAKE_CXX_COMPILER={cache['CMAKE_CXX_COMPILER']}",
             f"-DCMAKE_PREFIX_PATH={relocated};{dependency_prefix}"])
        run(["cmake", "--build", consumer_build, "--config", configuration])
        environment = os.environ.copy()
        environment["PATH"] = os.pathsep.join([str(relocated / "bin"),
                                               str(dependency_prefix / "debug/bin"),
                                               str(dependency_prefix / "bin"),
                                               environment.get("PATH", "")])
        run([consumer_build / "consumer"], env=environment)
        run([consumer_build / "model_consumer"], env=environment)
        data = relocated / "share/galata"
        for notice in ("eigen3.txt", "yaml-cpp.txt"):
            if not (data / "third_party/licenses" / notice).read_bytes().strip():
                raise ValueError(f"missing installed dependency notice: {notice}")
        studies = {"continuous-feedback": ("response.csv", "evidence.json"),
                   "nt33a-graph-design": ("model.yaml", "adapter.json", "graph-response.csv",
                                          "graph-evidence.json", "linear-response.csv"),
                   "nt33a-trim-and-linearise": ("trim-and-modes.md",),
                   "nt33a-control-design": ("control-design.md", "linear-response.csv",
                                            "nonlinear-response.csv")}
        for study, reports in studies.items():
            output = scratch / study
            output.mkdir()
            run([relocated / "bin" / "galata", "run",
                 data / "examples" / study / "study.yaml", "--output-dir", output],
                env=environment)
            for report in reports:
                if not (output / report).is_file() or not (output / report).stat().st_size:
                    raise ValueError(f"installed CLI did not produce {study}/{report}")
        if os.name == "posix":
            project = scratch / "installed-project.galata"
            engine = relocated / "bin/galata"
            run([engine, "project", "create", project], env=environment)
            result = subprocess.run([str(engine), "project", "run", str(project)],
                                    check=True, capture_output=True, text=True, env=environment)
            if json.loads(result.stdout)["status"] != "completed":
                raise ValueError("installed project worker did not complete")
            if cache.get("GALATA_BUILD_DESKTOP") == "ON":
                bundle = relocated / "Galata Preview.app/Contents"
                bundled_engine = bundle / "MacOS/galata"
                if hashlib.sha256(bundled_engine.read_bytes()).digest() != hashlib.sha256(engine.read_bytes()).digest():
                    raise ValueError("desktop bundle contains a stale numerical worker")
                for notice in ("LICENSE", "NOTICE", "THIRD_PARTY_LICENSES.md",
                               "licenses/eigen3.txt", "licenses/yaml-cpp.txt"):
                    if not (bundle / "Resources" / notice).read_bytes().strip():
                        raise ValueError(f"missing desktop dependency notice: {notice}")
                result = subprocess.run([str(bundled_engine), "project", "run", str(project)],
                                        check=True, capture_output=True, text=True, env=environment)
                if json.loads(result.stdout)["status"] != "completed":
                    raise ValueError("relocated bundled worker did not complete")
        print("Relocated C++ package and installed CLI passed.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: check-install.py <configured-and-built-vcpkg-directory>")
    try:
        check(Path(sys.argv[1]).resolve())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        sys.exit(f"install smoke failed: {error}")
