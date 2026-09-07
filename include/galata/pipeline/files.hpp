// SPDX-License-Identifier: Apache-2.0
// Run-scoped file access for offline engineering studies. Outputs are relative
// to an operator-chosen directory, atomically published, and never replaced
// unless explicitly requested. Input snapshots hash the exact parsed bytes.
#ifndef GALATA_PIPELINE_FILES_HPP
#define GALATA_PIPELINE_FILES_HPP

#include <cstddef>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace galata::pipeline {

struct FileRecord {
  std::string path;
  std::string sha256;
  std::string bytes;
};

// SHA-256 as defined by NIST FIPS 180-4, section 6.2. Used for content identity,
// not signatures, authentication, or a claim of tamper-resistant storage.
[[nodiscard]] std::string sha256(std::string_view bytes);
[[nodiscard]] std::string read_file_bytes(const std::string& path);
// Reads regular files only. The limit is enforced while reading, not just
// against an earlier file size; at most one excess byte is inspected.
[[nodiscard]] std::string read_file_bytes(const std::string& path, std::size_t max_bytes);
[[nodiscard]] std::string current_executable();
[[nodiscard]] std::string json_quote(std::string_view text);
[[nodiscard]] FileRecord snapshot_file(const std::string& path);
void verify_file_unchanged(const FileRecord& snapshot);

class RunFiles {
 public:
  explicit RunFiles(std::string output_directory, bool overwrite = false);

  [[nodiscard]] const std::string& output_directory() const {
    return output_directory_;
  }

  [[nodiscard]] std::string output_path(const std::string& relative) const;
  void reserve_output(const std::string& relative);
  // Protect runtime files (for example the executable) without embedding their
  // contents in the study's model-input snapshots.
  void protect_input_path(const std::string& path);
  [[nodiscard]] std::string read_input(const std::string& path);
  // A stricter later limit also applies to an already cached input snapshot.
  [[nodiscard]] std::string read_input(const std::string& path, std::size_t max_bytes);
  void record_input(const std::string& path, const std::string& bytes);
  void write_output(const std::string& relative, const std::string& bytes, bool immutable = false);

  [[nodiscard]] const std::map<std::string, FileRecord>& inputs() const {
    return inputs_;
  }

  [[nodiscard]] const std::map<std::string, FileRecord>& outputs() const {
    return outputs_;
  }

 private:
  void check_input_collision(const std::string& path, const std::string& relative) const;
  std::string output_directory_;
  bool overwrite_;
  std::map<std::string, FileRecord> inputs_;
  std::map<std::string, FileRecord> outputs_;
  std::vector<std::string> reserved_outputs_;
  std::vector<std::string> protected_inputs_;
};

}  // namespace galata::pipeline
#endif
