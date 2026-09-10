// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the PX4 ULog reader declared in include/galata/data/ulog.hpp.
//
// The container is a header followed by a stream of messages, each a uint16
// length, a uint8 type and that many payload bytes. Everything below is a
// bounds-checked walk over that stream: every read goes through `take`, which
// refuses to run off the end rather than returning whatever follows.

#include "galata/data/ulog.hpp"

#include "galata/core/sha256.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace galata::data {
namespace {

constexpr char kMagic[] = {'U', 'L', 'o', 'g', '\x01', '\x12', '\x35'};
constexpr std::size_t kHeaderSize = 16;

struct Field {
  std::string type;
  std::string name;
  int count = 1;  // 1 for a scalar, N for "type[N] name"
  std::size_t offset = 0;
  std::size_t element_size = 0;
};

struct Format {
  std::vector<Field> fields;
  std::size_t size = 0;
};

std::size_t size_of(const std::string& type) {
  if (type == "int8_t" || type == "uint8_t" || type == "bool" || type == "char") {
    return 1;
  }
  if (type == "int16_t" || type == "uint16_t") {
    return 2;
  }
  if (type == "int32_t" || type == "uint32_t" || type == "float") {
    return 4;
  }
  if (type == "int64_t" || type == "uint64_t" || type == "double") {
    return 8;
  }
  return 0;
}

// Reads one element as a double. The log's own type decides how; a reader that
// guessed would turn an integer counter into a float-shaped nonsense value.
double element_as_double(const std::string& type, const char* at) {
  const auto read = [&](auto sample) {
    std::memcpy(&sample, at, sizeof(sample));
    return static_cast<double>(sample);
  };
  if (type == "float") {
    return read(float{});
  }
  if (type == "double") {
    return read(double{});
  }
  if (type == "uint8_t" || type == "bool" || type == "char") {
    return read(std::uint8_t{});
  }
  if (type == "int8_t") {
    return read(std::int8_t{});
  }
  if (type == "uint16_t") {
    return read(std::uint16_t{});
  }
  if (type == "int16_t") {
    return read(std::int16_t{});
  }
  if (type == "uint32_t") {
    return read(std::uint32_t{});
  }
  if (type == "int32_t") {
    return read(std::int32_t{});
  }
  if (type == "uint64_t") {
    return read(std::uint64_t{});
  }
  if (type == "int64_t") {
    return read(std::int64_t{});
  }
  throw std::invalid_argument("data.import.ulog: unsupported field type '" + type + "'");
}

Format parse_format(const std::string& body, std::string& name) {
  const auto colon = body.find(':');
  if (colon == std::string::npos) {
    throw std::invalid_argument("data.import.ulog: a format message has no ':' separator");
  }
  name = body.substr(0, colon);
  Format format;
  std::size_t start = colon + 1;
  while (start < body.size()) {
    const auto semicolon = body.find(';', start);
    if (semicolon == std::string::npos) {
      break;
    }
    const std::string entry = body.substr(start, semicolon - start);
    start = semicolon + 1;
    const auto space = entry.find(' ');
    if (space == std::string::npos) {
      continue;
    }
    Field field;
    field.type = entry.substr(0, space);
    field.name = entry.substr(space + 1);
    const auto bracket = field.type.find('[');
    if (bracket != std::string::npos) {
      field.count = std::atoi(field.type.c_str() + bracket + 1);
      field.type = field.type.substr(0, bracket);
    }
    field.element_size = size_of(field.type);
    if (field.element_size == 0) {
      // Refused rather than skipped: an unknown type has an unknown width, so
      // every field after it would be read from the wrong offset. Silence here
      // is how a record acquires plausible wrong numbers.
      throw std::invalid_argument("data.import.ulog: field '" + field.name + "' of topic '"
                                  + name + "' has type '" + field.type
                                  + "', whose width this reader does not know. Every field "
                                    "after it would be read from the wrong offset");
    }
    field.offset = format.size;
    format.size += field.element_size * static_cast<std::size_t>(field.count);
    format.fields.push_back(std::move(field));
  }
  return format;
}

// Splits "xyz[2]" into ("xyz", 2); a bare name is index 0 of a scalar.
void split_field(const std::string& text, std::string& base, int& index) {
  const auto bracket = text.find('[');
  if (bracket == std::string::npos) {
    base = text;
    index = 0;
    return;
  }
  base = text.substr(0, bracket);
  index = std::atoi(text.c_str() + bracket + 1);
}

class Cursor {
 public:
  Cursor(const char* begin, std::size_t size) : at_(begin), end_(begin + size) {}

  [[nodiscard]] bool has(std::size_t count) const {
    return static_cast<std::size_t>(end_ - at_) >= count;
  }

  const char* take(std::size_t count) {
    if (!has(count)) {
      throw std::invalid_argument(
          "data.import.ulog: the file ends inside a message. A truncated log is refused rather "
          "than read up to the cut, because a partial record is not a shorter flight");
    }
    const char* result = at_;
    at_ += count;
    return result;
  }

  template <typename T>
  T read() {
    T value{};
    std::memcpy(&value, take(sizeof(T)), sizeof(T));
    return value;
  }

  [[nodiscard]] bool done() const {
    return at_ >= end_;
  }

 private:
  const char* at_;
  const char* end_;
};

}  // namespace

Record read_ulog(const std::string& bytes,
                 const std::string& source_path,
                 const UlogImportRequest& request) {
  if (!(request.resample_hz > 0.0) || !std::isfinite(request.resample_hz)) {
    throw std::invalid_argument(
        "data.import.ulog: resample_hz is required and must be positive. Topics arrive on their "
        "own timestamps at their own rates, and a record whose channels do not share a timebase "
        "is not one a fit can read");
  }
  if (request.channels.empty()) {
    throw std::invalid_argument("data.import.ulog: at least one channel is required");
  }
  if (bytes.size() < kHeaderSize || std::memcmp(bytes.data(), kMagic, sizeof(kMagic)) != 0) {
    throw std::invalid_argument(
        "data.import.ulog: this is not a ULog file — the magic bytes do not match");
  }
  const auto version = static_cast<unsigned char>(bytes[7]);
  if (version != 1) {
    std::ostringstream message;
    message << "data.import.ulog: format version " << static_cast<int>(version)
            << " is not supported; this reader implements version 1 and refuses rather than "
               "guessing at a layout it does not know";
    throw std::invalid_argument(message.str());
  }

  Cursor cursor(bytes.data() + kHeaderSize, bytes.size() - kHeaderSize);
  std::map<std::string, Format> formats;
  // (msg_id) -> (topic, multi_id)
  std::map<std::uint16_t, std::pair<std::string, int>> subscriptions;
  // (topic, multi_id) -> samples, each (timestamp_us, payload)
  std::map<std::pair<std::string, int>, std::vector<std::pair<std::uint64_t, std::string>>> series;
  std::int64_t dropouts = 0;

  while (!cursor.done()) {
    if (!cursor.has(3)) {
      break;
    }
    const auto size = cursor.read<std::uint16_t>();
    const auto type = cursor.read<std::uint8_t>();
    const char* payload = cursor.take(size);

    switch (type) {
      case 'B': {
        if (size < 40) {
          throw std::invalid_argument("data.import.ulog: the flag-bits message is too short");
        }
        std::uint64_t incompatible = 0;
        std::memcpy(&incompatible, payload + 8, sizeof(incompatible));
        if (incompatible != 0) {
          throw std::invalid_argument(
              "data.import.ulog: the log declares an incompatible flag this reader does not "
              "implement — appended data, which is what a log truncated by a crash carries. "
              "Refused rather than read as though the flag were absent");
        }
        break;
      }
      case 'F': {
        std::string name;
        Format format = parse_format(std::string(payload, size), name);
        formats[name] = std::move(format);
        break;
      }
      case 'A': {
        if (size < 3) {
          throw std::invalid_argument("data.import.ulog: a subscription message is too short");
        }
        const auto multi_id = static_cast<int>(static_cast<std::uint8_t>(payload[0]));
        std::uint16_t msg_id = 0;
        std::memcpy(&msg_id, payload + 1, sizeof(msg_id));
        subscriptions[msg_id] = {std::string(payload + 3, size - 3), multi_id};
        break;
      }
      case 'D': {
        if (size < 2) {
          throw std::invalid_argument("data.import.ulog: a data message is too short");
        }
        std::uint16_t msg_id = 0;
        std::memcpy(&msg_id, payload, sizeof(msg_id));
        const auto found = subscriptions.find(msg_id);
        if (found == subscriptions.end()) {
          break;  // data for a subscription we never saw; nothing to attribute it to
        }
        const auto format = formats.find(found->second.first);
        if (format == formats.end()) {
          break;
        }
        const char* body = payload + 2;
        const std::size_t body_size = size - 2;
        if (body_size < format->second.size) {
          break;
        }
        std::uint64_t timestamp_us = 0;
        std::memcpy(&timestamp_us, body, sizeof(timestamp_us));
        series[found->second].emplace_back(timestamp_us, std::string(body, body_size));
        break;
      }
      case 'O':
        ++dropouts;
        break;
      default:
        // Every other message type is skipped by its own length. The format
        // permits that, and it is what lets a reader survive a log written by a
        // newer PX4 — but it applies only to messages, never to a FIELD, whose
        // width has to be known for anything after it to be read at all.
        break;
    }
  }

  // Resolve every requested channel before touching the grid, so a study that
  // asks for something the log does not have is refused before it gets a
  // partial answer.
  struct Resolved {
    const std::vector<std::pair<std::uint64_t, std::string>>* samples = nullptr;
    Field field;
    int index = 0;
  };

  std::vector<Resolved> resolved;
  resolved.reserve(request.channels.size());
  for (const UlogChannel& channel : request.channels) {
    if (channel.unit.empty() || channel.frame.empty()) {
      throw std::invalid_argument("data.import.ulog: channel '" + channel.name
                                  + "' must declare a unit and a frame; use frame `none` for a "
                                    "scalar that has none");
    }
    const auto key = std::make_pair(channel.topic, channel.multi_id);
    const auto found = series.find(key);
    if (found == series.end() || found->second.empty()) {
      throw std::invalid_argument(
          "data.import.ulog: the log carries no samples for topic '" + channel.topic
          + "'. Refused rather than returning a record without it: a topic quietly absent is a "
            "fit performed on less data than the study thinks it used");
    }
    const auto format = formats.find(channel.topic);
    if (format == formats.end()) {
      throw std::invalid_argument("data.import.ulog: no format for topic '" + channel.topic + "'");
    }
    std::string base;
    int index = 0;
    split_field(channel.field, base, index);
    const auto field = std::find_if(format->second.fields.begin(),
                                    format->second.fields.end(),
                                    [&](const Field& f) { return f.name == base; });
    if (field == format->second.fields.end()) {
      throw std::invalid_argument("data.import.ulog: topic '" + channel.topic + "' has no field '"
                                  + base + "'");
    }
    if (index < 0 || index >= field->count) {
      std::ostringstream message;
      message << "data.import.ulog: '" << channel.field << "' is out of range; '" << base
              << "' has " << field->count << " element(s)";
      throw std::invalid_argument(message.str());
    }
    resolved.push_back({&found->second, *field, index});
  }

  // The grid is clipped to the span EVERY requested channel covers, so no
  // channel is ever asked for a value it does not have. No extrapolation, in
  // either direction, ever.
  std::uint64_t begin_us = 0;
  std::uint64_t end_us = std::numeric_limits<std::uint64_t>::max();
  for (const Resolved& entry : resolved) {
    begin_us = std::max(begin_us, entry.samples->front().first);
    end_us = std::min(end_us, entry.samples->back().first);
  }
  if (end_us <= begin_us) {
    throw std::invalid_argument(
        "data.import.ulog: the requested topics do not overlap in time, so there is no interval "
        "on which they share a timebase");
  }

  Record record;
  record.source_path = source_path;
  record.source_sha256 = core::sha256(bytes);
  record.description = request.description;
  record.timebase = Timebase::UniformResampled;
  record.sample_rate_hz = request.resample_hz;
  record.channels.resize(request.channels.size());

  const double period_s = 1.0 / request.resample_hz;
  const double begin_s = static_cast<double>(begin_us) * 1e-6;
  const double span_s = static_cast<double>(end_us - begin_us) * 1e-6;
  const auto steps = static_cast<std::int64_t>(std::floor(span_s / period_s));

  for (std::size_t c = 0; c < request.channels.size(); ++c) {
    const UlogChannel& channel = request.channels[c];
    Channel& out = record.channels[c];
    out.name = channel.name;
    out.source_name = channel.topic + "." + channel.field;
    out.unit = channel.unit;
    out.frame = channel.frame;
    out.scale_applied = channel.scale;
    out.offset_applied = channel.offset;
    out.samples.reserve(static_cast<std::size_t>(steps) + 1);
  }

  for (std::int64_t k = 0; k <= steps; ++k) {
    const double time_s = begin_s + static_cast<double>(k) * period_s;
    const auto target_us = static_cast<std::uint64_t>(std::llround(time_s * 1e6));
    record.times_s.push_back(time_s);
    for (std::size_t c = 0; c < resolved.size(); ++c) {
      const Resolved& entry = resolved[c];
      // Zero-order hold: the last sample at or before the grid time. Declared,
      // and the only rule applied — nothing here interpolates.
      const auto upper = std::upper_bound(
          entry.samples->begin(),
          entry.samples->end(),
          target_us,
          [](std::uint64_t value, const auto& sample) { return value < sample.first; });
      const auto at = upper == entry.samples->begin() ? upper : std::prev(upper);
      const char* element = at->second.data() + entry.field.offset
                            + entry.field.element_size * static_cast<std::size_t>(entry.index);
      const double raw = element_as_double(entry.field.type, element);
      const double si = request.channels[c].scale * raw + request.channels[c].offset;
      record.channels[c].samples.push_back(si);
      if (request.channels[c].scale != 1.0 || request.channels[c].offset != 0.0) {
        ++record.samples_altered;
      }
    }
  }
  record.rows_read = static_cast<std::int64_t>(record.times_s.size());
  record.rows_dropped_duplicate_time = dropouts;
  return record;
}

}  // namespace galata::data
