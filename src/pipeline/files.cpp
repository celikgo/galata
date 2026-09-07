// SPDX-License-Identifier: Apache-2.0
#include "galata/pipeline/files.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>

#include <fcntl.h>
#include <unistd.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
#endif

namespace galata::pipeline {
namespace {
namespace fs = std::filesystem;
std::atomic<unsigned long> temporary_counter{0};

[[noreturn]] void fail(const std::string& action, const std::string& path) {
  throw std::runtime_error(action + ": '" + path + "'");
}

fs::path relative_output(const std::string& text) {
  const fs::path path(text);
  if (text.empty() || path.is_absolute() || path.has_root_name()
      || text.find('\0') != std::string::npos || text.find(':') != std::string::npos
      || text.find('\\') != std::string::npos) {
    fail("output path must be a relative portable path inside the output directory", text);
  }
  for (const auto& part : path) {
    if (part == "..") {
      fail("output path must not contain '..'", text);
    }
    if (part == ".") {
      continue;
    }
    const auto name = part.string();
    if (name.empty() || name.back() == '.' || name.back() == ' '
        || name.find_first_of("<>\"|?*") != std::string::npos
        || std::any_of(name.begin(), name.end(), [](unsigned char c) { return c < 32; })) {
      fail("output path contains a non-portable filename", text);
    }
    auto stem = name.substr(0, name.find('.'));
    std::transform(stem.begin(), stem.end(), stem.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
    if (stem == "CON" || stem == "PRN" || stem == "AUX" || stem == "NUL"
        || (stem.size() == 4 && (stem.substr(0, 3) == "COM" || stem.substr(0, 3) == "LPT")
            && stem[3] >= '1' && stem[3] <= '9')) {
      fail("output path uses a reserved device name", text);
    }
  }
  const auto normalized = path.lexically_normal();
  if (normalized.filename().empty() || normalized == ".") {
    fail("output path must name a file", text);
  }
  return normalized;
}

std::string portable_identity(const std::string& relative) {
  auto identity = relative_output(relative).generic_string();
  // A study must not acquire two writers for one file when moved from a
  // case-sensitive filesystem to the default Windows/macOS filesystem.
  std::transform(identity.begin(), identity.end(), identity.begin(), [](unsigned char c) {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + ('a' - 'A')) : static_cast<char>(c);
  });
  return identity;
}

#ifdef _WIN32
struct Handle {
  HANDLE value = INVALID_HANDLE_VALUE;

  explicit Handle(HANDLE handle) : value(handle) {}

  ~Handle() {
    if (value != INVALID_HANDLE_VALUE) {
      CloseHandle(value);
    }
  }

  Handle(const Handle&) = delete;
  Handle& operator=(const Handle&) = delete;
};

void publish(const fs::path& root,
             const fs::path& relative,
             const std::string& bytes,
             bool overwrite) {
  // Hold each parent without delete sharing, so a directory cannot be replaced
  // with a junction while the file is written. Never follow reparse points.
  std::vector<std::unique_ptr<Handle>> parents;
  auto directory = root;
  auto hold = [&parents](const fs::path& path) {
    auto handle = std::make_unique<Handle>(
        CreateFileW(path.c_str(),
                    FILE_LIST_DIRECTORY,
                    FILE_SHARE_READ | FILE_SHARE_WRITE,
                    nullptr,
                    OPEN_EXISTING,
                    FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
                    nullptr));
    BY_HANDLE_FILE_INFORMATION info{};
    if (handle->value == INVALID_HANDLE_VALUE || !GetFileInformationByHandle(handle->value, &info)
        || (info.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0
        || (info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0) {
      fail("output parent is not a plain directory", path.string());
    }
    parents.push_back(std::move(handle));
  };
  hold(directory);
  for (const auto& part : relative.parent_path()) {
    if (part == ".") {
      continue;
    }
    directory /= part;
    if (!CreateDirectoryW(directory.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
      fail("cannot create output parent", directory.string());
    }
    hold(directory);
  }
  const auto destination = directory / relative.filename();
  const DWORD attrs = GetFileAttributesW(destination.c_str());
  if (attrs != INVALID_FILE_ATTRIBUTES) {
    if ((attrs & (FILE_ATTRIBUTE_REPARSE_POINT | FILE_ATTRIBUTE_DIRECTORY)) != 0) {
      fail("output target must be a regular file, never a symlink or directory",
           destination.string());
    }
    if (!overwrite) {
      fail("output exists; use --overwrite to replace it", destination.string());
    }
  }
  fs::path temporary;
  HANDLE raw = INVALID_HANDLE_VALUE;
  for (int attempt = 0; attempt < 100 && raw == INVALID_HANDLE_VALUE; ++attempt) {
    temporary = directory
                / (".galata-write-" + std::to_string(GetCurrentProcessId()) + "-"
                   + std::to_string(temporary_counter.fetch_add(1)));
    raw = CreateFileW(
        temporary.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (raw == INVALID_HANDLE_VALUE && GetLastError() != ERROR_FILE_EXISTS) {
      fail("cannot create temporary output", temporary.string());
    }
  }
  if (raw == INVALID_HANDLE_VALUE) {
    fail("cannot reserve temporary output", destination.string());
  }
  try {
    {
      Handle file(raw);
      std::size_t offset = 0;
      while (offset < bytes.size()) {
        const auto count =
            static_cast<DWORD>(std::min<std::size_t>(bytes.size() - offset, 1U << 30));
        DWORD written = 0;
        if (!WriteFile(file.value, bytes.data() + offset, count, &written, nullptr)
            || written == 0) {
          fail("failed writing output", destination.string());
        }
        offset += written;
      }
      if (!FlushFileBuffers(file.value)) {
        fail("failed flushing output", destination.string());
      }
    }
    const DWORD flags = MOVEFILE_WRITE_THROUGH | (overwrite ? MOVEFILE_REPLACE_EXISTING : 0U);
    if (!MoveFileExW(temporary.c_str(), destination.c_str(), flags)) {
      fail("cannot publish output (existing files require --overwrite)", destination.string());
    }
  } catch (...) {
    DeleteFileW(temporary.c_str());
    throw;
  }
}
#else
struct Descriptor {
  int value;

  explicit Descriptor(int fd) : value(fd) {}

  ~Descriptor() {
    if (value >= 0) {
      (void)::close(value);
    }
  }

  Descriptor(const Descriptor&) = delete;
  Descriptor& operator=(const Descriptor&) = delete;
};

void publish(const fs::path& root,
             const fs::path& relative,
             const std::string& bytes,
             bool overwrite) {
  Descriptor parent(::open(root.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  if (parent.value < 0) {
    fail("cannot open output directory", root.string());
  }
  for (const auto& part : relative.parent_path()) {
    if (part == ".") {
      continue;
    }
    if (::mkdirat(parent.value, part.c_str(), 0755) != 0 && errno != EEXIST) {
      fail("cannot create output parent", (root / relative).string());
    }
    const int next =
        ::openat(parent.value, part.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
    if (next < 0) {
      fail("output parent is not a plain directory", (root / relative).string());
    }
    (void)::close(parent.value);
    parent.value = next;
  }
  const auto name = relative.filename().string();
  struct stat info{};
  if (::fstatat(parent.value, name.c_str(), &info, AT_SYMLINK_NOFOLLOW) == 0) {
    if (!S_ISREG(info.st_mode)) {
      fail("output target must be a regular file, never a symlink or directory", name);
    }
    if (!overwrite) {
      fail("output exists; use --overwrite to replace it", name);
    }
  } else if (errno != ENOENT) {
    fail("cannot inspect output", name);
  }
  std::string temporary;
  int raw = -1;
  for (int attempt = 0; attempt < 100 && raw < 0; ++attempt) {
    temporary = ".galata-write-" + std::to_string(::getpid()) + "-"
                + std::to_string(temporary_counter.fetch_add(1));
    raw = ::openat(parent.value,
                   temporary.c_str(),
                   O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                   0600);
    if (raw < 0 && errno != EEXIST) {
      fail("cannot create temporary output", name);
    }
  }
  if (raw < 0) {
    fail("cannot reserve temporary output", name);
  }
  try {
    {
      Descriptor file(raw);
      std::size_t offset = 0;
      while (offset < bytes.size()) {
        const std::size_t count = std::min<std::size_t>(bytes.size() - offset, 1U << 30);
        const auto written = ::write(file.value, bytes.data() + offset, count);
        if (written < 0 && errno == EINTR) {
          continue;
        }
        if (written <= 0) {
          fail("failed writing output", name);
        }
        offset += static_cast<std::size_t>(written);
      }
      if (::fsync(file.value) != 0) {
        fail("failed flushing output", name);
      }
    }
    // linkat publishes a complete file only if the name is still absent. A
    // exists()+rename() check would race and could overwrite without consent.
    const int result =
        overwrite ? ::renameat(parent.value, temporary.c_str(), parent.value, name.c_str())
                  : ::linkat(parent.value, temporary.c_str(), parent.value, name.c_str(), 0);
    if (result != 0) {
      fail("cannot publish output (existing files require --overwrite)", name);
    }
    if (!overwrite) {
      (void)::unlinkat(parent.value, temporary.c_str(), 0);
    }
  } catch (...) {
    (void)::unlinkat(parent.value, temporary.c_str(), 0);
    throw;
  }
}
#endif
}  // namespace

std::string read_file_bytes(const std::string& path) {
  return read_file_bytes(path, std::numeric_limits<std::size_t>::max());
}

std::string read_file_bytes(const std::string& path, std::size_t max_bytes) {
  if (path.empty() || path.find('\0') != std::string::npos) {
    fail("input path must be non-empty and contain no NUL byte", path);
  }
#ifdef _WIN32
  // Resolve ordinary input symlinks as before, then refuse a replacement
  // reparse point/device on the opened handle before attempting to read it.
  const auto resolved = fs::canonical(path);
  Handle file(CreateFileW(resolved.c_str(),
                          GENERIC_READ,
                          FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                          nullptr,
                          OPEN_EXISTING,
                          FILE_FLAG_OPEN_REPARSE_POINT,
                          nullptr));
  if (file.value == INVALID_HANDLE_VALUE) {
    fail("cannot open input", path);
  }
  BY_HANDLE_FILE_INFORMATION info{};
  if (GetFileType(file.value) != FILE_TYPE_DISK || !GetFileInformationByHandle(file.value, &info)
      || (info.dwFileAttributes & (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT)) != 0) {
    fail("input must be a regular file", path);
  }
#else
  // O_NONBLOCK prevents a substituted FIFO from blocking during open. The
  // descriptor check, rather than a prior path stat, establishes regularity.
  Descriptor file(::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK));
  if (file.value < 0)
    fail("cannot open input", path);
  struct stat info{};
  if (::fstat(file.value, &info) != 0 || !S_ISREG(info.st_mode)) {
    fail("input must be a regular file", path);
  }
#endif
  std::string bytes;
  std::array<char, 8192> buffer{};
  for (;;) {
    const auto remaining = max_bytes - bytes.size();
    const auto request = remaining < buffer.size() ? remaining + 1U : buffer.size();
    std::size_t received = 0;
#ifdef _WIN32
    DWORD count = 0;
    if (!ReadFile(file.value, buffer.data(), static_cast<DWORD>(request), &count, nullptr)) {
      fail("failed reading input", path);
    }
    received = count;
#else
    ssize_t count;
    do {
      count = ::read(file.value, buffer.data(), request);
    } while (count < 0 && errno == EINTR);
    if (count < 0)
      fail("failed reading input", path);
    received = static_cast<std::size_t>(count);
#endif
    if (received > remaining) {
      fail("input exceeds byte limit " + std::to_string(max_bytes), path);
    }
    if (received == 0)
      break;
    bytes.append(buffer.data(), received);
  }
  return bytes;
}

std::string current_executable() {
#ifdef _WIN32
  std::vector<wchar_t> path(32768);
  const DWORD size = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
  if (size == 0 || size >= path.size()) {
    throw std::runtime_error("cannot identify executable");
  }
  return fs::path(std::wstring(path.data(), size)).string();
#elif defined(__APPLE__)
  std::uint32_t size = 0;
  (void)_NSGetExecutablePath(nullptr, &size);
  std::vector<char> path(size);
  if (_NSGetExecutablePath(path.data(), &size) != 0) {
    throw std::runtime_error("cannot identify executable");
  }
  return fs::canonical(path.data()).string();
#else
  return fs::canonical("/proc/self/exe").string();
#endif
}

FileRecord snapshot_file(const std::string& path) {
  const auto canonical = fs::canonical(path).string();
  auto bytes = read_file_bytes(canonical);
  const auto digest = sha256(bytes);
  return FileRecord{canonical, digest, std::move(bytes)};
}

void verify_file_unchanged(const FileRecord& snapshot) {
  if (read_file_bytes(snapshot.path) != snapshot.bytes) {
    fail("runtime file changed during the run; no completed manifest can be written",
         snapshot.path);
  }
}

std::string json_quote(std::string_view text) {
  constexpr char hex[] = "0123456789abcdef";
  std::string out = "\"";
  for (const char raw : text) {
    const auto c = static_cast<unsigned char>(raw);
    if (c == '"' || c == '\\') {
      out += '\\';
      out += raw;
    } else if (c < 0x20) {
      out += "\\u00";
      out += hex[c >> 4U];
      out += hex[c & 15U];
    } else {
      out += raw;
    }
  }
  return out + '"';
}

RunFiles::RunFiles(std::string directory, bool overwrite) : overwrite_(overwrite) {
  if (directory.empty()) {
    directory = ".";
  }
  fs::create_directories(directory);
  output_directory_ = fs::canonical(directory).string();
}

std::string RunFiles::output_path(const std::string& relative) const {
  const auto candidate = relative_output(relative);
  fs::path path(output_directory_);
  for (const auto& part : candidate) {
    path /= part;
    const auto status = fs::symlink_status(path);
    if (fs::is_symlink(status)) {
      fail("output paths must not traverse symlinks", relative);
    }
  }
  return path.string();
}

void RunFiles::reserve_output(const std::string& relative) {
  const auto path = output_path(relative);
  const auto identity = portable_identity(relative);
  if (std::find(reserved_outputs_.begin(), reserved_outputs_.end(), identity)
      != reserved_outputs_.end()) {
    fail("two stages may not write the same output", relative);
  }
  check_input_collision(path, relative);
  if (fs::exists(path)) {
    if (!fs::is_regular_file(path)) {
      fail("output target must be a regular file", relative);
    }
    if (!overwrite_) {
      fail("output exists; use --overwrite to replace it", relative);
    }
  }
  reserved_outputs_.push_back(identity);
}

void RunFiles::protect_input_path(const std::string& path) {
  if (path.empty() || path.find('\0') != std::string::npos) {
    fail("protected input path must be non-empty and contain no NUL byte", path);
  }
  const auto canonical = fs::weakly_canonical(path).string();
  if (std::find(protected_inputs_.begin(), protected_inputs_.end(), canonical)
      == protected_inputs_.end()) {
    protected_inputs_.push_back(canonical);
  }
}

void RunFiles::check_input_collision(const std::string& path, const std::string& relative) const {
  const auto check = [&](const std::string& input) {
    std::error_code error;
    if (input == path || fs::equivalent(input, path, error)) {
      fail("an output may not overwrite a run input or executable", relative);
    }
  };
  for (const auto& [input, record] : inputs_) {
    (void)record;
    check(input);
  }
  for (const auto& input : protected_inputs_) {
    check(input);
  }
}

std::string RunFiles::read_input(const std::string& path) {
  return read_input(path, std::numeric_limits<std::size_t>::max());
}

std::string RunFiles::read_input(const std::string& path, std::size_t max_bytes) {
  if (path.empty() || path.find('\0') != std::string::npos) {
    fail("input path must be non-empty and contain no NUL byte", path);
  }
  const auto canonical = fs::canonical(path).string();
  const auto found = inputs_.find(canonical);
  if (found != inputs_.end()) {
    if (found->second.bytes.size() > max_bytes) {
      fail("cached input exceeds byte limit " + std::to_string(max_bytes), canonical);
    }
    return found->second.bytes;
  }
  auto bytes = read_file_bytes(canonical, max_bytes);
  inputs_.emplace(canonical, FileRecord{canonical, sha256(bytes), bytes});
  return bytes;
}

void RunFiles::record_input(const std::string& path, const std::string& bytes) {
  const auto canonical = fs::weakly_canonical(path).string();
  const auto found = inputs_.find(canonical);
  if (found != inputs_.end() && found->second.bytes != bytes) {
    fail("input changed within a run", canonical);
  }
  inputs_[canonical] = FileRecord{canonical, sha256(bytes), bytes};
}

void RunFiles::write_output(const std::string& relative, const std::string& bytes, bool immutable) {
  const auto path = output_path(relative);
  const auto normalized = relative_output(relative).generic_string();
  for (const auto& [prior, record] : outputs_) {
    (void)record;
    if (portable_identity(prior) == portable_identity(normalized)
        || (fs::exists(path) && fs::equivalent(fs::path(output_directory_) / prior, path))) {
      fail("two stages may not write the same output", relative);
    }
  }
  check_input_collision(path, relative);
  // Immutable manifests are content-addressed and may be reused only if every
  // byte is identical. --overwrite never changes a manifest in place.
  if (immutable && fs::exists(path)) {
    if (fs::is_regular_file(path) && read_file_bytes(path) == bytes) {
      return;
    }
    fail("immutable run manifest already exists with different content", relative);
  }
  publish(output_directory_, relative_output(relative), bytes, overwrite_ && !immutable);
  outputs_.emplace(normalized, FileRecord{normalized, sha256(bytes), bytes});
}
}  // namespace galata::pipeline
