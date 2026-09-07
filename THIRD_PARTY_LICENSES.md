# Third-party software and reference data

galata's own source is Apache-2.0. Dependencies keep their own terms. The
inventory below was checked against the installed vcpkg package metadata and
copyright files used for the development build. Version floors in the manifest
are not assertions about the resolved version. The registry baseline and Eigen
override are recorded in [vcpkg.json](vcpkg.json).

| Dependency | Observed package version | Use | Supplied notice |
|---|---|---|---|
| Eigen | 3.4.1, port 1 | Public C++ types and numerical implementation, header-only | [Eigen notice bundle](third_party/licenses/eigen3.txt) |
| yaml-cpp | 0.9.0, port 1 | YAML parsing, linked by model and pipeline libraries | [MIT notice](third_party/licenses/yaml-cpp.txt) |
| GoogleTest | 1.17.0, port 2 | Test executables only; not linked by the CLI | [BSD 3-clause notice](third_party/licenses/gtest.txt) |
| vcpkg-cmake | 2024-04-23 | Dependency build helper | [MIT notice](third_party/licenses/vcpkg-cmake.txt) |
| vcpkg-cmake-config | 2024-05-23 | Dependency package configuration helper | [MIT notice](third_party/licenses/vcpkg-cmake-config.txt) |

Eigen's supplied notice describes its primary MPL-2.0 licensing and includes
additional component notices. The complete bundle is retained; listing Eigen
here does not relicense its files under Apache-2.0. These are copies of the
packages' supplied notices, not a claim that every optional Eigen component is
used. No modified third-party implementation is vendored in this repository.
The actual headers and libraries are obtained through the dependency manager.

Release archives collect the copyright files from the packages actually
installed for that build, including dependency build helpers and test packages.
Their dependency inventory records versions, architecture and notice hashes.
The extra notices describe build inputs and do not imply all those packages are
linked into the CLI. CMake installation also installs the notices for the Eigen
and yaml-cpp packages it resolved; non-vcpkg builds may provide their notice
paths through the documented CMake cache variables in
[GalataInstall.cmake](cmake/GalataInstall.cmake).

Compiler runtimes and operating-system libraries are platform dependencies.
This inventory is not an assertion that a build on an arbitrary toolchain has
the same dependency contents. Rebuilds must retain the notices supplied by the
actual dependency versions they use.

The published validation data are numerical transcriptions, with source and
transcription information in each file:

- [NT-33A flight condition 1](tests/validation/reference/nt33a_fc1.csv).
- [U.S. Standard Atmosphere 1976](tests/validation/reference/ussa1976.csv).
- [Seiler, Packard and Gahinet disk-margin example](tests/validation/reference/seiler2020_disk_margin.csv).
- [Skogestad and Postlethwaite sensitivity bounds](tests/validation/reference/skogestad2005_sensitivity_bounds.csv).

These citations do not relicense the source publications. The policy for
reference values from copyrighted sources is recorded in
[ADR-0007](docs/adr/0007-reference-values-from-copyrighted-sources.md).
