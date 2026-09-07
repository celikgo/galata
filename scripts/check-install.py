#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Install, relocate, compile and run a consumer without galata's source includes."""
import os
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
int main() {
  namespace m = galata::modeling;
  m::Model source;
  source.blocks = {{"constant", {}, m::Constant{2.0}}, {"output", {}, m::Output{}}};
  source.connections = {{"constant", "output", 0}};
  const auto compiled = m::compile_model(m::parse_model_yaml(m::write_model_yaml(source)));
  const auto result = m::simulate(compiled, {.step_count = 0});
  return !result.state_ids.empty() || result.outputs.size() != 1
      || result.outputs.front()(0) != 2.0 || result.semantic_sha256.size() != 64;
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
        suffix = ".exe" if os.name == "nt" else ""
        run([consumer_build / ("consumer" + suffix)], env=environment)
        run([consumer_build / ("model_consumer" + suffix)], env=environment)
        data = relocated / "share/galata"
        for notice in ("eigen3.txt", "yaml-cpp.txt"):
            if not (data / "third_party/licenses" / notice).read_bytes().strip():
                raise ValueError(f"missing installed dependency notice: {notice}")
        studies = {"continuous-feedback": ("response.csv", "evidence.json"),
                   "nt33a-trim-and-linearise": ("trim-and-modes.md",),
                   "nt33a-control-design": ("control-design.md", "linear-response.csv",
                                            "nonlinear-response.csv")}
        for study, reports in studies.items():
            output = scratch / study
            output.mkdir()
            run([relocated / "bin" / ("galata" + suffix), "run",
                 data / "examples" / study / "study.yaml", "--output-dir", output],
                env=environment)
            for report in reports:
                if not (output / report).is_file() or not (output / report).stat().st_size:
                    raise ValueError(f"installed CLI did not produce {study}/{report}")
        print("Relocated C++ package and installed CLI passed.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: check-install.py <configured-and-built-vcpkg-directory>")
    try:
        check(Path(sys.argv[1]).resolve())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        sys.exit(f"install smoke failed: {error}")
