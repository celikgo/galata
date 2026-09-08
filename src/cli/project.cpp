// SPDX-License-Identifier: Apache-2.0
// Experimental local project store and one-job CLI worker. ADR-0012 and
// ADR-0014 define identity, publication and recovery. Numerical execution
// stays in the pipeline.
// WHAT THIS IS NOT: a shared-drive database, native-code sandbox, migration
// system, or proof of model validity. Locks cover cooperating local processes.
#include "project.hpp"

#include "galata/modeling/model.hpp"
#include "galata/pipeline/files.hpp"
#include "galata/pipeline/pipeline.hpp"
#include "galata/pipeline/registry.hpp"

#include "../io/strict_yaml.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#ifndef _WIN32
#include <sys/file.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <unistd.h>
#endif

namespace {
#ifndef _WIN32
namespace fs = std::filesystem;
namespace p = galata::pipeline;
namespace m = galata::modeling;
namespace io = galata::io;
using Node = YAML::Node;
constexpr std::size_t kDocumentBytes = 2 * 1024 * 1024;
constexpr std::size_t kOriginBytes = 8 * 1024 * 1024;
constexpr std::size_t kArtifactBytes = 128 * 1024 * 1024;
constexpr std::size_t kHistoryEntries = 1024;
constexpr std::size_t kPresentationRoutes = 256;
constexpr std::size_t kRoutePoints = 64;

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error("project: " + message);
}

volatile std::sig_atomic_t cancellation = 0;

void cancel_handler(int) {
  cancellation = 1;
}

class Signals {
 public:
  Signals() {
    cancellation = 0;
    struct sigaction action{};
    action.sa_handler = cancel_handler;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGTERM, &action, &term_) != 0)
      fail("cannot install termination handler");
    if (sigaction(SIGINT, &action, &interrupt_) != 0) {
      sigaction(SIGTERM, &term_, nullptr);
      fail("cannot install interrupt handler");
    }
  }

  ~Signals() {
    sigaction(SIGTERM, &term_, nullptr);
    sigaction(SIGINT, &interrupt_, nullptr);
  }

  Signals(const Signals&) = delete;
  Signals& operator=(const Signals&) = delete;

 private:
  struct sigaction term_{}, interrupt_{};
};

class Lock {
 public:
  explicit Lock(const std::string& path, bool required = true) {
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK, 0600);
    struct stat st{};
    if (fd_ < 0 || fstat(fd_, &st) != 0 || !S_ISREG(st.st_mode) || st.st_nlink != 1) {
      if (fd_ >= 0)
        ::close(fd_);
      fail("lock must be a private regular file");
    }
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      const int error = errno;
      ::close(fd_);
      fd_ = -1;
      if (error != EWOULDBLOCK
#if EAGAIN != EWOULDBLOCK
          && error != EAGAIN
#endif
      )
        fail("cannot acquire project lock");
      if (required)
        fail("project is busy; retry after the current edit or snapshot");
    }
  }

  ~Lock() {
    if (fd_ >= 0)
      ::close(fd_);
  }

  bool acquired() const {
    return fd_ >= 0;
  }

  Lock(const Lock&) = delete;
  Lock& operator=(const Lock&) = delete;

 private:
  int fd_ = -1;
};

std::string read(const p::RunFiles& files,
                 const std::string& relative,
                 std::size_t limit = kDocumentBytes) {
  const auto path = files.output_path(relative);
  const int fd = ::open(path.c_str(), O_RDONLY | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK);
  struct stat st{};
  if (fd < 0 || fstat(fd, &st) != 0 || !S_ISREG(st.st_mode)) {
    if (fd >= 0)
      ::close(fd);
    fail("cannot read a regular project file: " + relative);
  }
  std::string result;
  std::array<char, 8192> buffer{};
  try {
    while (true) {
      const auto count =
          ::read(fd, buffer.data(), std::min(buffer.size(), limit - result.size() + 1));
      if (count < 0 && errno == EINTR)
        continue;
      if (count < 0)
        fail("cannot read " + relative);
      if (count == 0)
        break;
      result.append(buffer.data(), static_cast<std::size_t>(count));
      if (result.size() > limit)
        fail("file exceeds byte limit: " + relative);
    }
    ::close(fd);
    return result;
  } catch (...) {
    ::close(fd);
    throw;
  }
}

Node load(const p::RunFiles& files,
          const std::string& relative,
          std::size_t limit = kDocumentBytes) {
  return io::load_yaml(read(files, relative, limit), relative);
}

bool digest(const std::string& text) {
  return text.size() == 64 && std::all_of(text.begin(), text.end(), [](char c) {
           return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
         });
}

std::string string(const Node& node, const std::string& key) {
  const auto value = node[key];
  if (!value || !value.IsScalar())
    fail("missing scalar '" + key + "'");
  return value.Scalar();
}

void schema(const Node& node, const std::string& expected) {
  if (string(node, "schema") != expected)
    fail("unsupported schema; expected " + expected);
}

// Preserve decimal lexemes, including subnormals, signed zero and integer
// indices wider than binary64. The model parser owns finite-range validation.
bool decimal(const std::string& text) {
  std::size_t offset = 0;
  if (!text.empty() && (text[0] == '+' || text[0] == '-'))
    ++offset;
  const auto digits = [&] {
    const auto begin = offset;
    while (offset < text.size() && text[offset] >= '0' && text[offset] <= '9')
      ++offset;
    return begin != offset;
  };
  const bool whole = digits();
  bool fraction = false;
  if (offset < text.size() && text[offset] == '.') {
    ++offset;
    fraction = digits();
  }
  if (!whole && !fraction)
    return false;
  if (offset < text.size() && (text[offset] == 'e' || text[offset] == 'E')) {
    ++offset;
    if (offset < text.size() && (text[offset] == '+' || text[offset] == '-'))
      ++offset;
    if (!digits())
      return false;
  }
  return offset == text.size();
}

std::string json_decimal(std::string text) {
  std::string sign;
  if (text.front() == '+' || text.front() == '-') {
    if (text.front() == '-')
      sign = "-";
    text.erase(0, 1);
  }
  const auto exponent_at = text.find_first_of("eE");
  const auto exponent = exponent_at == std::string::npos ? "" : text.substr(exponent_at);
  auto significand = text.substr(0, exponent_at);
  if (significand.front() == '.')
    significand.insert(0, "0");
  const auto first_nonzero = significand.find_first_not_of('0');
  if (first_nonzero == std::string::npos)
    significand = "0";
  else if (first_nonzero > 0)
    significand.erase(0, first_nonzero - (significand[first_nonzero] == '.' ? 1 : 0));
  if (significand.back() == '.')
    significand += '0';
  return sign + significand + exponent;
}

std::string json(const Node& node, bool force_string = false) {
  if (!node || node.IsNull())
    return "null";
  if (node.IsSequence()) {
    std::string result = "[";
    for (std::size_t i = 0; i < node.size(); ++i)
      result += (i ? "," : "") + json(node[i]);
    return result + "]";
  }
  if (node.IsMap()) {
    std::map<std::string, Node> entries;
    for (const auto& item : node)
      entries.emplace(item.first.Scalar(), item.second);
    static const std::set<std::string> string_fields = {"schema",
                                                        "profile",
                                                        "id",
                                                        "kind",
                                                        "frame",
                                                        "source",
                                                        "target",
                                                        "revision",
                                                        "status",
                                                        "diagnostic",
                                                        "path",
                                                        "sha256"};
    std::string result = "{";
    for (const auto& [key, value] : entries) {
      if (result.size() > 1)
        result += ',';
      result += p::json_quote(key) + ':' + json(value, string_fields.contains(key));
    }
    return result + '}';
  }
  const auto text = node.Scalar();
  if (!force_string && node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str") {
    if (decimal(text))
      return json_decimal(text);
    if (text == "true" || text == "false")
      return text;
  }
  return p::json_quote(text);
}

Node text_node(const std::string& value) {
  Node result(value);
  result.SetTag("tag:yaml.org,2002:str");
  return result;
}

void write(p::RunFiles& files, const std::string& path, const Node& value, bool immutable = true) {
  files.write_output(path, json(value) + '\n', immutable);
}

double number(const Node& node, const std::string& key, double low, double high) {
  const Node value = node[key];
  double result = 0;
  if (!value || !value.IsScalar() || value.Tag() == "!" || !decimal(value.Scalar())
      || !YAML::convert<double>::decode(value, result) || !std::isfinite(result) || result < low
      || result > high)
    fail("invalid numeric field '" + key + "'");
  return result;
}

std::size_t route_input(const Node& route) {
  const Node value = route["input"];
  if (!value || !value.IsScalar() || value.Tag() == "!" || value.Tag() == "tag:yaml.org,2002:str")
    fail("route input must be an unquoted nonnegative decimal integer");
  const auto text = value.Scalar();
  std::size_t result = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result, 10);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
    fail("route input must be a representable nonnegative decimal integer");
  return result;
}

void validate_routes(const Node& routes, const m::Model& model) {
  if (!routes.IsSequence() || routes.size() > kPresentationRoutes)
    fail("presentation.routes must be an array of at most 256 routes");
  using Wire = std::tuple<std::string, std::string, std::size_t>;
  std::set<Wire> connections;
  for (const auto& connection : model.connections)
    connections.emplace(connection.source, connection.target, connection.input);
  std::set<Wire> seen;
  for (const auto& route : routes) {
    io::yaml_keys(route, "route", {"source", "target", "input", "points"});
    const Wire wire{string(route, "source"), string(route, "target"), route_input(route)};
    if (!connections.contains(wire))
      fail("presentation route must name an existing connection");
    if (!seen.insert(wire).second)
      fail("presentation routes contain a duplicate connection");
    const Node points = route["points"];
    if (!points.IsSequence() || points.size() < 2 || points.size() > kRoutePoints)
      fail("route points must be an array of 2 through 64 canvas points");
    double previous_x = 0, previous_y = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      io::yaml_keys(points[i], "route point", {"x", "y"});
      const double x = number(points[i], "x", -100000, 100000);
      const double y = number(points[i], "y", -100000, 100000);
      if (i != 0 && ((x == previous_x) == (y == previous_y)))
        fail("route segments must be nonzero and horizontal or vertical");
      previous_x = x;
      previous_y = y;
    }
  }
}

void validate_draft(const Node& draft) {
  if (string(draft, "schema") == "galata.project-draft.v2") {
    io::yaml_keys(
        draft, "draft", {"schema", "model", "presentation", "simulation", "origin_sha256"});
    if (!digest(string(draft, "origin_sha256")))
      fail("invalid origin identity");
  } else {
    io::yaml_keys(draft, "draft", {"schema", "model", "presentation", "simulation"});
    schema(draft, "galata.project-draft.v1");
  }
  // Draft validation deliberately admits incomplete wiring, but never opens
  // the source vocabulary or weakens the executable compiler contract.
  const auto model = m::parse_model_draft_yaml(json(draft["model"]));
  const Node presentation = draft["presentation"];
  const bool manual_routes = string(presentation, "schema") == "galata.presentation.v2";
  if (manual_routes)
    io::yaml_keys(presentation, "presentation", {"schema", "positions", "routes"});
  else {
    io::yaml_keys(presentation, "presentation", {"schema", "positions"});
    schema(presentation, "galata.presentation.v1");
  }
  const Node positions = presentation["positions"];
  if (!positions.IsMap() || positions.size() > m::kMaxBlocks)
    fail("presentation.positions must be a bounded map");
  std::set<std::string> ids;
  for (const auto& block : model.blocks)
    ids.insert(block.id);
  for (const auto& entry : positions) {
    if (!ids.contains(entry.first.Scalar()))
      fail("presentation position names an unknown block");
    io::yaml_keys(entry.second, "position", {"x", "y"});
    (void)number(entry.second, "x", -100000, 100000);
    (void)number(entry.second, "y", -100000, 100000);
  }
  if (manual_routes)
    validate_routes(presentation["routes"], model);
  const Node simulation = draft["simulation"];
  io::yaml_keys(simulation, "simulation", {"initial_time_s", "step_s", "steps", "sample_stride"});
  (void)number(simulation, "initial_time_s", -1e12, 1e12);
  (void)number(simulation, "step_s", std::numeric_limits<double>::min(), 1e12);
  for (const auto& key : {"steps", "sample_stride"}) {
    const double value = number(simulation, key, key == std::string("steps") ? 0 : 1, m::kMaxSteps);
    if (value != std::floor(value))
      fail(std::string(key) + " must be integral");
  }
}

Node origin_at(const p::RunFiles& files, const Node& draft);

std::string head(const p::RunFiles& files) {
  const auto node = load(files, "project.json");
  io::yaml_keys(node, "project", {"schema", "revision"});
  schema(node, "galata.project.v1");
  const auto revision = string(node, "revision");
  if (!digest(revision))
    fail("invalid revision identity");
  return revision;
}

Node draft_at(const p::RunFiles& files, const std::string& revision) {
  if (!digest(revision))
    fail("invalid revision identity");
  const auto bytes = read(files, "revisions/" + revision + ".json");
  if (p::sha256(bytes) != revision)
    fail("revision content digest mismatch");
  const auto draft = io::load_yaml(bytes, "revision");
  validate_draft(draft);
  return draft;
}

Node revision_at(const p::RunFiles& files, const std::string& revision) {
  const auto draft = draft_at(files, revision);
  if (draft["origin_sha256"])
    (void)origin_at(files, draft);
  return draft;
}

std::vector<std::string> entries(const p::RunFiles& files, const std::string& directory) {
  const auto path = files.output_path(directory + "/entry");
  const fs::path parent = fs::path(path).parent_path();
  std::vector<std::string> result;
  if (!fs::exists(parent))
    return result;
  for (const auto& entry : fs::directory_iterator(parent)) {
    if (result.size() >= kHistoryEntries)
      fail(directory + " exceeds the preview history limit");
    result.push_back(entry.path().filename().string());
  }
  std::sort(result.begin(), result.end());
  return result;
}

void publish_head(p::RunFiles& files, const std::string& identity) {
  Node pointer;
  pointer["schema"] = text_node("galata.project.v1");
  pointer["revision"] = text_node(identity);
  write(files, "project.json", pointer, false);
}

void publish_revision(p::RunFiles& files, const std::string& bytes) {
  const auto identity = p::sha256(bytes);
  const auto path = "revisions/" + identity + ".json";
  if (fs::exists(files.output_path(path))) {
    if (read(files, path) != bytes)
      fail("existing revision is corrupted");
  } else {
    if (entries(files, "revisions").size() >= kHistoryEntries)
      fail("revision history limit exceeded");
    files.write_output(path, bytes, true);
  }
  publish_head(files, identity);
}

Node default_draft() {
  // Independently defined synthetic system: x' = 1 - x, x(0)=0;
  // SI units: x in m, the constant/rate in m/s, feedback coefficient in 1/s.
  const m::Dimension length{1, 0, 0, 0, 0, 0, 0, 0}, rate{1, 0, -1, 0, 0, 0, 0, 0};
  m::Model model;
  model.blocks = {{"command", {rate}, m::Constant{1}},
                  {"feedback", {rate}, m::Gain{1, {0, 0, -1, 0, 0, 0, 0, 0}}},
                  {"rate", {rate}, m::Sum{{1, -1}}},
                  {"x", {length}, m::Integrator{0}},
                  {"y", {length}, m::Output{}}};
  model.connections = {{"command", "rate", 0},
                       {"feedback", "rate", 1},
                       {"rate", "x", 0},
                       {"x", "feedback", 0},
                       {"x", "y", 0}};
  Node draft;
  draft["schema"] = text_node("galata.project-draft.v1");
  draft["model"] = io::load_yaml(m::write_model_yaml(model), "default model");
  draft["presentation"]["schema"] = text_node("galata.presentation.v1");
  auto positions = draft["presentation"]["positions"];
  const std::map<std::string, std::pair<int, int>> layout = {{"command", {40, 55}},
                                                             {"rate", {260, 55}},
                                                             {"x", {480, 55}},
                                                             {"y", {690, 55}},
                                                             {"feedback", {360, 205}}};
  for (const auto& [id, xy] : layout) {
    positions[id]["x"] = xy.first;
    positions[id]["y"] = xy.second;
  }
  draft["simulation"]["initial_time_s"] = 0;
  draft["simulation"]["step_s"] = 0.01;
  draft["simulation"]["steps"] = 500;
  draft["simulation"]["sample_stride"] = 1;
  return draft;
}

std::string make_study(const Node& draft) {
  const std::string context = draft["origin_sha256"] ? ", context_path: origin.json" : "";
  return "version: 1\nstages:\n"
         "  - id: model\n    capability: model.compile\n    input: {path: model.yaml" + context + "}\n"
         "  - id: response\n    capability: sim.model\n    input:\n"
         "      model: {from: model}\n"
         "      csv_path: response.csv\n      evidence_path: evidence.json\n"
         "      initial_time_s: "
         + json(draft["simulation"]["initial_time_s"])
         + "\n      step_s: " + json(draft["simulation"]["step_s"])
         + "\n      steps: " + json(draft["simulation"]["steps"])
         + "\n      sample_stride: " + json(draft["simulation"]["sample_stride"]) + "\n";
}

Node artifact(const p::RunFiles& files, const std::string& path) {
  Node result;
  result["path"] = text_node(path);
  result["sha256"] = text_node(p::sha256(read(files, path, kArtifactBytes)));
  return result;
}

std::string check_artifact(const p::RunFiles& files,
                           const Node& record,
                           const std::string& required_prefix) {
  io::yaml_keys(record, "artifact", {"path", "sha256"});
  const auto path = string(record, "path");
  if (!path.starts_with(required_prefix))
    fail("artifact is outside its run");
  const auto bytes = read(files, path, kArtifactBytes);
  if (p::sha256(bytes) != string(record, "sha256"))
    fail("artifact digest mismatch: " + path);
  return bytes;
}

void check_input(const Node& record, const std::string& bytes) {
  const auto hex = string(record, "bytes_hex");
  constexpr char digits[] = "0123456789abcdef";
  if (hex.size() != bytes.size() * 2 || string(record, "sha256") != p::sha256(bytes)
      || record["size_bytes"].as<std::size_t>() != bytes.size())
    fail("manifest consumed-input identity mismatch");
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    const auto byte = static_cast<unsigned char>(bytes[i]);
    if (hex[2 * i] != digits[byte >> 4U] || hex[2 * i + 1] != digits[byte & 15U])
      fail("manifest consumed-input bytes mismatch");
  }
}

void check_snapshot(const Node& record) {
  const auto hex = string(record, "bytes_hex");
  if (hex.size() % 2 != 0)
    fail("invalid input snapshot encoding");
  const auto digit = [](char c) -> unsigned char {
    if (c >= '0' && c <= '9')
      return static_cast<unsigned char>(c - '0');
    if (c >= 'a' && c <= 'f')
      return static_cast<unsigned char>(c - 'a' + 10);
    fail("invalid input snapshot encoding");
  };
  std::string bytes;
  bytes.reserve(hex.size() / 2);
  for (std::size_t i = 0; i < hex.size(); i += 2)
    bytes += static_cast<char>((digit(hex[i]) << 4U) | digit(hex[i + 1]));
  check_input(record, bytes);
}

// Source context is immutable origin evidence, never a claim about an edited
// graph. All active lookups use project-relative paths; old absolute source
// paths in the imported manifest remain provenance after project relocation.
Node origin_at(const p::RunFiles& files, const Node& draft) {
  const auto identity = string(draft, "origin_sha256");
  if (!digest(identity))
    fail("invalid origin identity");
  const auto bytes = read(files, "origins/" + identity + ".json", kOriginBytes);
  if (p::sha256(bytes) != identity)
    fail("origin content digest mismatch");
  const auto origin = io::load_yaml(bytes, "origin");
  schema(origin, "galata.project-origin.v1");
  io::yaml_keys(origin,
                "origin",
                {"schema",
                 "model_semantic_sha256",
                 "model_yaml",
                 "adapter_json",
                 "manifest_json",
                 "manifest_path",
                 "artifacts"});
  const auto manifest_bytes = string(origin, "manifest_json");
  const auto manifest = io::load_yaml(manifest_bytes, "original manifest");
  schema(manifest, "galata.run.v1");
  if (string(manifest, "status") != "completed")
    fail("import manifest is not completed");
  const fs::path manifest_path(string(origin, "manifest_path"));
  const auto prefix = manifest_path.parent_path().generic_string() + '/';
  if (!prefix.starts_with("imports/") || manifest_path.parent_path().parent_path() != "imports"
      || !digest(manifest_path.parent_path().filename().string())
      || manifest_path.filename() != "run-" + p::sha256(manifest_bytes) + ".json")
    fail("invalid import manifest location");
  const auto records = origin["artifacts"];
  if (!records.IsSequence() || records.size() < 3 || records.size() > 128)
    fail("invalid import artifact set");
  std::map<std::string, std::string> retained;
  for (const auto& record : records) {
    const auto path = string(record, "path");
    if (!retained.emplace(path, string(record, "sha256")).second)
      fail("duplicate import artifact");
    (void)check_artifact(files, record, prefix);
  }
  if (retained[manifest_path.generic_string()] != p::sha256(manifest_bytes)
      || read(files, manifest_path.generic_string(), kOriginBytes) != manifest_bytes)
    fail("original manifest binding mismatch");
  check_snapshot(manifest["study"]);
  if (!manifest["inputs"].IsSequence() || manifest["inputs"].size() > 128)
    fail("invalid original input set");
  for (const auto& input : manifest["inputs"])
    check_snapshot(input);
  const auto outputs = manifest["outputs"];
  if (!outputs.IsSequence() || outputs.size() + 1 != retained.size())
    fail("import manifest output set mismatch");
  std::set<std::string> seen;
  for (const auto& output : outputs) {
    const auto relative = string(output, "path");
    const auto path = prefix + relative;
    if (!seen.insert(path).second || !retained.contains(path)
        || retained.at(path) != string(output, "sha256"))
      fail("import manifest output identity mismatch");
  }
  const auto model_bytes = string(origin, "model_yaml");
  const auto adapter_bytes = string(origin, "adapter_json");
  if (model_bytes.size() > m::kMaxSourceBytes || adapter_bytes.size() > kDocumentBytes)
    fail("import model or adapter exceeds byte limit");
  const auto adapter = io::load_yaml(adapter_bytes, "adapter");
  schema(adapter, "galata.linear-adapter.v1");
  const auto semantic = m::compile_model(m::parse_model_yaml(model_bytes)).semantic_sha256();
  if (string(origin, "model_semantic_sha256") != semantic
      || string(adapter, "model_semantic_sha256") != semantic)
    fail("original graph semantic identity mismatch");
  // Both exact exports must be outputs of the retained original pipeline.
  for (const auto& exported : {model_bytes, adapter_bytes}) {
    const auto hash = p::sha256(exported);
    if (std::none_of(
            seen.begin(), seen.end(), [&](const auto& path) { return retained.at(path) == hash; }))
      fail("original export is not bound by its manifest");
  }
  return origin;
}

std::string origin_relation(const Node& draft, const Node& origin) {
  try {
    const auto compiled = m::compile_model(m::parse_model_yaml(json(draft["model"])));
    return compiled.semantic_sha256() == string(origin, "model_semantic_sha256")
               ? "matches_imported_model"
               : "modified_from_import";
  } catch (const std::exception&) {
    return "uncompiled_draft";
  }
}

void check_completion(const p::RunFiles& files,
                      const Node& result,
                      const Node& draft,
                      const std::string& prefix) {
  const Node artifacts = result["artifacts"];
  io::yaml_keys(artifacts, "artifacts", {"trajectory_csv", "evidence_json", "manifest_path"});
  const auto trajectory = check_artifact(files, artifacts["trajectory_csv"], prefix + "output/");
  const auto evidence = io::load_yaml(
      check_artifact(files, artifacts["evidence_json"], prefix + "output/"), "evidence");
  const auto manifest_bytes = check_artifact(files, artifacts["manifest_path"], prefix + "output/");
  const auto manifest = io::load_yaml(manifest_bytes, "manifest");
  schema(manifest, "galata.run.v1");
  schema(evidence, "galata.model-run.v1");
  if (string(manifest, "status") != "completed" || string(evidence, "execution") != "completed")
    fail("completion evidence is not completed");
  const auto manifest_name =
      fs::path(string(artifacts["manifest_path"], "path")).filename().string();
  if (manifest_name != "run-" + p::sha256(manifest_bytes) + ".json")
    fail("manifest name does not bind its content");
  if (string(evidence, "trajectory_sha256") != p::sha256(trajectory))
    fail("trajectory does not match numerical evidence");
  const auto source = read(files, prefix + "model.yaml", m::kMaxSourceBytes);
  if (source != json(draft["model"]) + '\n'
      || read(files, prefix + "study.yaml") != make_study(draft))
    fail("run source differs from its immutable revision");
  check_input(manifest["study"], make_study(draft));
  const auto inputs = manifest["inputs"];
  std::map<std::string, std::string> expected_inputs = {{"model.yaml", source},
                                                        {"study.yaml", make_study(draft)}};
  if (draft["origin_sha256"]) {
    const auto original =
        read(files, "origins/" + string(draft, "origin_sha256") + ".json", kOriginBytes);
    if (read(files, prefix + "origin.json", kOriginBytes) != original)
      fail("run origin differs from immutable revision");
    expected_inputs.emplace("origin.json", original);
  }
  if (!inputs.IsSequence() || inputs.size() != expected_inputs.size())
    fail("unexpected manifest input set");
  std::set<std::string> consumed;
  for (const auto& input : inputs) {
    const auto name = fs::path(string(input, "path")).filename().string();
    if (!consumed.insert(name).second || !expected_inputs.contains(name))
      fail("unexpected consumed source");
    check_input(input, expected_inputs.at(name));
  }
  const auto compiled = m::compile_model(m::parse_model_yaml(source));
  if (string(evidence, "model_semantic_sha256") != compiled.semantic_sha256())
    fail("model semantic identity mismatch");
  std::map<std::string, std::string> output_digests;
  const Node outputs = manifest["outputs"];
  if (!outputs.IsSequence() || outputs.size() != 2)
    fail("unexpected manifest output set");
  for (const auto& output : outputs)
    output_digests.emplace(fs::path(string(output, "path")).filename().string(),
                           string(output, "sha256"));
  if (output_digests["response.csv"] != p::sha256(trajectory)
      || output_digests["evidence.json"] != string(artifacts["evidence_json"], "sha256"))
    fail("pipeline output digests do not match project artifacts");
  const Node solver = evidence["solver"];
  for (const auto& key : {"initial_time_s", "step_s", "steps", "sample_stride"}) {
    if (solver[key].as<double>() != draft["simulation"][key].as<double>())
      fail("solver options differ from immutable request");
  }
}

Node run_view(const p::RunFiles& files, const std::string& id) {
  Node view;
  view["id"] = text_node(id);
  try {
    if (!digest(id))
      fail("invalid run directory identity");
    const auto prefix = "runs/" + id + '/';
    Lock active(files.output_path(prefix + "worker.lock"), false);
    const Node request = load(files, prefix + "request.json");
    io::yaml_keys(request, "request", {"schema", "revision", "id"});
    schema(request, "galata.project-request.v1");
    if (string(request, "id") != id)
      fail("request identity mismatch");
    const auto revision = string(request, "revision");
    const auto draft = revision_at(files, revision);
    view["revision"] = text_node(revision);
    if (draft["origin_sha256"])
      view["origin_relation"] = text_node(origin_relation(draft, origin_at(files, draft)));
    if (!fs::exists(files.output_path(prefix + "result.json"))) {
      view["status"] = text_node(active.acquired() ? "interrupted" : "running");
      view["diagnostic"] = text_node(
          active.acquired() ? "Worker exited without a terminal record; no accepted result."
                            : "Worker is executing its immutable request.");
      return view;
    }
    const auto result = load(files, prefix + "result.json");
    io::yaml_keys(
        result, "result", {"schema", "id", "revision", "status", "diagnostic", "artifacts"});
    schema(result, "galata.project-result.v1");
    if (string(result, "id") != id || string(result, "revision") != revision)
      fail("terminal record request identity mismatch");
    const auto status = string(result, "status");
    if (status != "completed" && status != "failed" && status != "cancelled")
      fail("unsupported terminal state");
    if (status == "completed") {
      check_completion(files, result, draft, prefix);
      for (const auto& key : {"trajectory_csv", "evidence_json", "manifest_path"})
        view[key] = text_node(files.output_path(string(result["artifacts"][key], "path")));
    }
    view["status"] = text_node(status);
    view["diagnostic"] = text_node(string(result, "diagnostic"));
  } catch (const std::exception& error) {
    view["status"] = text_node("invalid");
    view["diagnostic"] = text_node(error.what());
  }
  return view;
}

Node inspect_revision(const p::RunFiles& files, const std::string& revision) {
  const auto draft = revision_at(files, revision);
  Node result;
  result["schema"] = text_node("galata.project-view.v1");
  result["revision"] = text_node(revision);
  result["draft_schema"] = draft["schema"];
  if (draft["origin_sha256"]) {
    result["origin_sha256"] = draft["origin_sha256"];
    const auto origin = origin_at(files, draft);
    result["origin"]["schema"] = text_node("galata.project-origin-view.v1");
    result["origin"]["relation"] = text_node(origin_relation(draft, origin));
    result["origin"]["model_semantic_sha256"] = origin["model_semantic_sha256"];
    result["origin"]["adapter"] = io::load_yaml(string(origin, "adapter_json"), "adapter");
    result["origin"]["manifest"] =
        io::load_yaml(string(origin, "manifest_json"), "original manifest");
    result["origin"]["manifest_path"] =
        text_node(files.output_path(string(origin, "manifest_path")));
  }
  for (const auto& key : {"model", "presentation", "simulation"})
    result[key] = draft[key];
  result["runs"] = Node(YAML::NodeType::Sequence);
  for (const auto& id : entries(files, "runs"))
    result["runs"].push_back(run_view(files, id));
  return result;
}

Node inspect(const p::RunFiles& files) {
  return inspect_revision(files, head(files));
}

Node revision_view(const p::RunFiles& files, const std::string& revision) {
  const auto draft = revision_at(files, revision);
  Node result;
  result["schema"] = text_node("galata.project-revision.v1");
  result["revision"] = text_node(revision);
  result["draft"] = draft;
  if (draft["origin_sha256"])
    result["origin_relation"] = text_node(origin_relation(draft, origin_at(files, draft)));
  return result;
}

Node revisions(const p::RunFiles& files) {
  const auto current = head(files);
  Node result;
  result["schema"] = text_node("galata.project-history.v1");
  result["current_revision"] = text_node(current);
  result["current_status"] = text_node("invalid");
  result["current_diagnostic"] = text_node("current revision file is missing");
  result["ordering"] = text_node("revision_filename_ascending");
  result["revisions"] = Node(YAML::NodeType::Sequence);
  // Every draft is checked independently. Most imported revisions share one
  // immutable origin: verify its artifacts once during this locked listing,
  // retaining only the semantic digest or diagnostic rather than full files.
  std::map<std::string, Node> verified_origins;
  std::map<std::string, std::string> invalid_origins;
  for (const auto& filename : entries(files, "revisions")) {
    const auto identity =
        filename.ends_with(".json") ? filename.substr(0, filename.size() - 5) : filename;
    const bool is_current = filename == current + ".json";
    Node entry;
    entry["filename"] = text_node(filename);
    entry["revision"] = text_node(identity);
    entry["is_current"] = is_current;
    try {
      if (filename != identity + ".json" || !digest(identity))
        fail("invalid revision filename");
      const auto draft = draft_at(files, identity);
      if (draft["origin_sha256"]) {
        const auto origin_identity = string(draft, "origin_sha256");
        if (invalid_origins.contains(origin_identity))
          throw std::runtime_error(invalid_origins.at(origin_identity));
        if (!verified_origins.contains(origin_identity)) {
          try {
            const auto origin = origin_at(files, draft);
            Node summary;
            summary["model_semantic_sha256"] = text_node(string(origin, "model_semantic_sha256"));
            verified_origins.emplace(origin_identity, summary);
          } catch (const std::exception& error) {
            invalid_origins.emplace(origin_identity, error.what());
            throw;
          }
        }
        entry["origin_relation"] =
            text_node(origin_relation(draft, verified_origins.at(origin_identity)));
      }
      entry["status"] = text_node("valid");
      entry["model_profile"] = text_node(string(draft["model"], "profile"));
      entry["block_count"] = draft["model"]["blocks"].size();
      entry["simulation"] = YAML::Clone(draft["simulation"]);
    } catch (const std::exception& error) {
      entry["status"] = text_node("invalid");
      entry["diagnostic"] = text_node(error.what());
    }
    if (is_current) {
      result["current_status"] = entry["status"];
      if (entry["diagnostic"])
        result["current_diagnostic"] = entry["diagnostic"];
      else
        result.remove("current_diagnostic");
    }
    result["revisions"].push_back(entry);
  }
  return result;
}

void preserve_origin(const Node& current, const Node& selected) {
  if (string(current, "schema") != string(selected, "schema")
      || (current["origin_sha256"]
          && string(current, "origin_sha256") != string(selected, "origin_sha256")))
    fail(
        "save and restore must preserve the project's original source attachment and draft schema");
}

void import_linear(p::RunFiles& files, const std::string& study_path) {
  Signals signals;
  Lock edit(files.output_path("project.lock"));
  if (fs::exists(files.output_path("project.json")))
    fail("project already exists");
  const auto absolute_study = fs::canonical(study_path);
  auto pipeline = p::parse_pipeline(p::read_file_bytes(absolute_study.string(), kDocumentBytes));
  pipeline.source_name = absolute_study.string();
  if (pipeline.stages.size() > 64)
    fail("import study exceeds 64 stages");
  const p::Stage* graph = nullptr;
  const p::Stage* simulation = nullptr;
  for (const auto& stage : pipeline.stages) {
    if (stage.capability == "model.linear_graph") {
      if (graph)
        fail("import requires exactly one model.linear_graph stage");
      graph = &stage;
    }
    if (stage.capability == "sim.model") {
      if (simulation)
        fail("import requires exactly one sim.model stage");
      simulation = &stage;
    }
  }
  if (!graph || !simulation || !simulation->input->get("model")
      || simulation->input->get("model")->as_stage_reference() != graph->id)
    fail("import requires one model.linear_graph with a directly connected sim.model stage");
  Node draft;
  draft["schema"] = text_node("galata.project-draft.v2");
  draft["simulation"]["initial_time_s"] = simulation->input->number_at("initial_time_s", 0.0);
  draft["simulation"]["step_s"] = simulation->input->number_at("step_s");
  draft["simulation"]["steps"] = simulation->input->integer_at("steps", -1);
  draft["simulation"]["sample_stride"] = simulation->input->integer_at("sample_stride", 1);
  fs::create_directories(fs::path(files.output_path("imports/entry")).parent_path());
  auto temporary = files.output_path("imports/.new-XXXXXX");
  if (!mkdtemp(temporary.data()))
    fail("cannot reserve an import directory");
  const auto id = p::sha256(temporary + pipeline.source_bytes);
  const auto prefix = "imports/" + id + '/';
  const auto import_root = fs::path(files.output_path(prefix + "entry")).parent_path();
  fs::rename(temporary, import_root);
  p::RunOptions options;
  options.cancelled = [] { return cancellation != 0; };
  const auto result = p::run_pipeline(pipeline,
                                      p::builtin_registry(),
                                      absolute_study.parent_path().string(),
                                      import_root.string(),
                                      nullptr,
                                      options);
  if (cancellation)
    fail("import cancelled before publication");
  const auto model_bytes =
      read(files, prefix + graph->input->string_at("model_path"), m::kMaxSourceBytes);
  const auto adapter_bytes = read(files, prefix + graph->input->string_at("adapter_path"));
  const auto compiled = m::compile_model(m::parse_model_yaml(model_bytes));
  draft["model"] = io::load_yaml(model_bytes, "imported model");
  draft["presentation"]["schema"] = text_node("galata.presentation.v1");
  draft["presentation"]["positions"] = Node(YAML::NodeType::Map);
  std::map<std::string, int> rows;
  for (const auto& block : compiled.source_model().blocks) {
    // Columns follow signal flow, while channel suffixes preserve source order.
    const auto group = block.id.substr(0, block.id.find_last_of('_'));
    const std::map<std::string, int> columns = {{"command", 40},
                                                {"control", 270},
                                                {"derivative", 500},
                                                {"state", 730},
                                                {"output_value", 960},
                                                {"output", 1190},
                                                {"control_output", 500}};
    draft["presentation"]["positions"][block.id]["x"] = columns.at(group);
    draft["presentation"]["positions"][block.id]["y"] =
        55 + 95 * rows[group]++ + (group == "control_output" ? 450 : 0);
  }
  const auto manifest_path = prefix + fs::path(result.manifest_path).filename().string();
  const auto manifest_bytes = read(files, manifest_path, kOriginBytes);
  const auto manifest = io::load_yaml(manifest_bytes, "import manifest");
  Node origin;
  origin["schema"] = text_node("galata.project-origin.v1");
  origin["model_semantic_sha256"] = text_node(compiled.semantic_sha256());
  origin["model_yaml"] = text_node(model_bytes);
  origin["adapter_json"] = text_node(adapter_bytes);
  origin["manifest_json"] = text_node(manifest_bytes);
  origin["manifest_path"] = text_node(manifest_path);
  origin["artifacts"] = Node(YAML::NodeType::Sequence);
  for (const auto& output : manifest["outputs"]) {
    origin["artifacts"].push_back(artifact(files, prefix + string(output, "path")));
  }
  origin["artifacts"].push_back(artifact(files, manifest_path));
  const auto origin_bytes = json(origin) + '\n';
  if (origin_bytes.size() > kOriginBytes)
    fail("import origin exceeds 8 MiB");
  const auto identity = p::sha256(origin_bytes);
  files.write_output("origins/" + identity + ".json", origin_bytes, true);
  draft["origin_sha256"] = text_node(identity);
  validate_draft(draft);
  (void)origin_at(files, draft);
  const auto draft_bytes = json(draft) + '\n';
  if (draft_bytes.size() > kDocumentBytes)
    fail("imported draft exceeds byte limit");
  if (cancellation)
    fail("import cancelled before publication");
  publish_revision(files, draft_bytes);
}

Node run(p::RunFiles& files) {
  Signals signals;
  std::string id, revision;
  Node draft;
  std::unique_ptr<Lock> worker;
  {
    Lock edit(files.output_path("project.lock"));
    revision = head(files);
    draft = revision_at(files, revision);
    if (entries(files, "runs").size() >= kHistoryEntries)
      fail("run history limit exceeded");
    // mkdtemp supplies collision-free job allocation, not numerical identity.
    // Request/revision/artifact hashes supply content identity independently.
    fs::create_directories(fs::path(files.output_path("runs/entry")).parent_path());
    std::string temporary = files.output_path("runs/.new-XXXXXX");
    if (!mkdtemp(temporary.data()))
      fail("cannot reserve a run directory");
    id = p::sha256(temporary + revision);
    const auto final = files.output_path("runs/" + id);
    fs::rename(temporary, final);
    const auto prefix = "runs/" + id + '/';
    worker = std::make_unique<Lock>(files.output_path(prefix + "worker.lock"));
    Node request;
    request["schema"] = text_node("galata.project-request.v1");
    request["revision"] = text_node(revision);
    request["id"] = text_node(id);
    write(files, prefix + "request.json", request);
  }
  const auto prefix = "runs/" + id + '/';
  Node terminal;
  terminal["schema"] = text_node("galata.project-result.v1");
  terminal["id"] = text_node(id);
  terminal["revision"] = text_node(revision);
  try {
    files.write_output(prefix + "model.yaml", json(draft["model"]) + '\n', true);
    files.write_output(prefix + "study.yaml", make_study(draft), true);
    if (draft["origin_sha256"])
      files.write_output(
          prefix + "origin.json",
          read(files, "origins/" + string(draft, "origin_sha256") + ".json", kOriginBytes),
          true);
    p::RunOptions options;
    options.cancelled = [] { return cancellation != 0; };
    const auto output = files.output_path(prefix + "output/entry");
    const auto pipeline = p::load_pipeline(files.output_path(prefix + "study.yaml"));
    const auto result =
        p::run_pipeline(pipeline,
                        p::builtin_registry(),
                        fs::path(files.output_path(prefix + "model.yaml")).parent_path().string(),
                        fs::path(output).parent_path().string(),
                        nullptr,
                        options);
    if (cancellation)
      fail("cancelled before publication");
    terminal["status"] = text_node("completed");
    terminal["diagnostic"] = text_node(
        "Execution completed. Numerical accuracy, model validity and engineering acceptance are "
        "not assessed.");
    terminal["artifacts"]["trajectory_csv"] = artifact(files, prefix + "output/response.csv");
    terminal["artifacts"]["evidence_json"] = artifact(files, prefix + "output/evidence.json");
    terminal["artifacts"]["manifest_path"] =
        artifact(files, prefix + "output/" + fs::path(result.manifest_path).filename().string());
    check_completion(files, terminal, draft, prefix);
    if (cancellation)
      fail("cancelled before publication");
  } catch (const std::exception& error) {
    terminal["status"] = text_node(cancellation ? "cancelled" : "failed");
    terminal["diagnostic"] = text_node(error.what());
    terminal.remove("artifacts");
  }
  write(files, prefix + "result.json", terminal);
  worker.reset();
  return run_view(files, id);
}
#endif
}  // namespace

int project_command(const std::vector<std::string>& arguments) {
#ifdef _WIN32
  (void)arguments;
  std::cerr << "galata project: the experimental local project worker supports macOS/Linux only\n";
  return 2;
#else
  try {
    if (arguments.size() < 2)
      fail(
          "usage: galata project <create|inspect|revisions|run> <directory>, revision <directory> "
          "<revision>, import-linear <directory> <study.yaml>, or <save|restore> <directory> "
          "<draft.json|revision> --expected-revision <revision>");
    const auto& operation = arguments[0];
    const bool expected = operation == "save" || operation == "restore";
    const bool selected = operation == "import-linear" || operation == "revision";
    if ((expected && (arguments.size() != 5 || arguments[3] != "--expected-revision"))
        || (selected && arguments.size() != 3) || (!expected && !selected && arguments.size() != 2))
      fail("invalid command arguments");
    if (operation != "create" && operation != "save" && operation != "run" && operation != "inspect"
        && operation != "import-linear" && operation != "revisions" && operation != "revision"
        && operation != "restore")
      fail("unknown project operation");
    const fs::path root(arguments[1]);
    if (fs::is_symlink(fs::symlink_status(root)))
      fail("project root must not be a symlink");
    if (operation == "create" || operation == "import-linear") {
      if (fs::exists(root) && (!fs::is_directory(root) || !fs::is_empty(root)))
        fail("create/import requires a new or empty directory");
    } else if (!fs::is_directory(root)) {
      fail("project directory does not exist");
    }
    p::RunFiles files(root.string(), true);
    if (operation == "create") {
      Lock edit(files.output_path("project.lock"));
      if (fs::exists(files.output_path("project.json")))
        fail("project already exists");
      const auto draft = default_draft();
      validate_draft(draft);
      publish_revision(files, json(draft) + '\n');
    } else if (operation == "import-linear") {
      import_linear(files, arguments[2]);
    } else if (operation == "save") {
      const auto bytes = p::read_file_bytes(arguments[2], kDocumentBytes);
      const auto draft = io::load_yaml(bytes, "draft");
      validate_draft(draft);
      Lock edit(files.output_path("project.lock"));
      if (head(files) != arguments[4])
        fail("stale revision; reopen before saving to preserve the other edit");
      const auto current = revision_at(files, arguments[4]);
      preserve_origin(current, draft);
      publish_revision(files, bytes);
    } else if (operation == "revisions" || operation == "revision") {
      Lock edit(files.output_path("project.lock"));
      const auto view =
          operation == "revisions" ? revisions(files) : revision_view(files, arguments[2]);
      std::cout << json(view) << '\n';
      return 0;
    } else if (operation == "restore") {
      Lock edit(files.output_path("project.lock"));
      if (head(files) != arguments[4])
        fail("stale revision; reopen before restoring to preserve the other edit");
      const auto current = revision_at(files, arguments[4]);
      const auto selected_draft = revision_at(files, arguments[2]);
      preserve_origin(current, selected_draft);
      // Complete review before changing the head, so a refused history scan or
      // serialization cannot report failure after a successful restoration.
      const auto response = json(inspect_revision(files, arguments[2]));
      publish_head(files, arguments[2]);
      std::cout << response << '\n';
      return 0;
    } else if (operation == "run") {
      const auto view = run(files);
      std::cout << json(view) << '\n';
      return string(view, "status") == "completed" ? 0 : 1;
    }
    std::cout << json(inspect(files)) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
#endif
}
