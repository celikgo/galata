// SPDX-License-Identifier: Apache-2.0
//
// data.import.* — measured records entering galata through a declared contract.

#include "galata/data/csv.hpp"
#include "galata/data/ulog.hpp"
#include "galata/data/window.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"

#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace galata::pipeline {
namespace {

data::CsvImportRequest request_from(const StageContext& context) {
  data::CsvImportRequest request;
  request.time_column = context.input->string_at("time_column");
  request.time_scale_to_seconds = context.input->number_at("time_scale_to_seconds", 1.0);
  request.description = context.input->string_at("description", "");

  const std::string missing = context.input->string_at("missing", "refuse");
  if (missing == "refuse") {
    request.missing = data::MissingPolicy::Refuse;
  } else if (missing == "drop_row") {
    request.missing = data::MissingPolicy::DropRow;
  } else {
    throw std::invalid_argument("data.import.csv: `missing` must be `refuse` or `drop_row`; '"
                                + missing + "' is neither");
  }

  const ValuePtr channels = context.input->get("channels");
  if (!channels) {
    throw std::invalid_argument("data.import.csv: `channels` is required");
  }
  for (const ValuePtr& entry : channels->as_list()) {
    data::ColumnMapping mapping;
    mapping.column = entry->string_at("column");
    mapping.name = entry->string_at("name");
    // No fallback for either. A default unit is a guess wearing a declaration's
    // clothes, and it is the guess that survives all the way into a fitted
    // coefficient without anything downstream able to see it.
    mapping.unit = entry->string_at("unit");
    mapping.frame = entry->string_at("frame");
    mapping.scale = entry->number_at("scale", 1.0);
    mapping.offset = entry->number_at("offset", 0.0);
    request.channels.push_back(std::move(mapping));
  }
  if (const ValuePtr ignored = context.input->get("ignore_columns")) {
    for (const ValuePtr& entry : ignored->as_list()) {
      request.ignore_columns.push_back(entry->as_string());
    }
  }
  return request;
}

Artifact import_csv(const StageContext& context) {
  const std::string path = context.input->string_at("path");
  const data::Record record = data::read_csv(context.read_input(path), path, request_from(context));

  std::ostringstream summary;
  summary << record.sample_count() << " sample(s), " << record.channels.size() << " channel(s), "
          << std::fixed << std::setprecision(3) << record.duration_s() << " s";
  if (record.rows_dropped_nonfinite > 0 || record.rows_dropped_duplicate_time > 0) {
    summary << "; dropped " << record.rows_dropped_nonfinite << " non-finite and "
            << record.rows_dropped_duplicate_time << " duplicate-time row(s)";
  }

  Artifact artifact;
  artifact.kind = "measured_record";
  artifact.summary = summary.str();
  artifact.payload = record;
  return artifact;
}

Artifact import_ulog(const StageContext& context) {
  const std::string path = context.input->string_at("path");
  data::UlogImportRequest request;
  request.resample_hz = context.input->number_at("resample_hz");
  request.description = context.input->string_at("description", "");
  const ValuePtr channels = context.input->get("channels");
  if (!channels) {
    throw std::invalid_argument("data.import.ulog: `channels` is required");
  }
  for (const ValuePtr& entry : channels->as_list()) {
    data::UlogChannel channel;
    channel.topic = entry->string_at("topic");
    channel.field = entry->string_at("field");
    channel.name = entry->string_at("name");
    // Required, as for CSV and for the same reason: a default here becomes a
    // factor or a sign inside a fitted coefficient, with nothing downstream
    // able to see it.
    channel.unit = entry->string_at("unit");
    channel.frame = entry->string_at("frame");
    channel.scale = entry->number_at("scale", 1.0);
    channel.offset = entry->number_at("offset", 0.0);
    channel.multi_id = static_cast<int>(entry->number_at("multi_id", 0.0));
    request.channels.push_back(std::move(channel));
  }

  const data::Record record = data::read_ulog(context.read_input(path), path, request);
  std::ostringstream summary;
  summary << record.sample_count() << " sample(s), " << record.channels.size() << " channel(s) at "
          << std::fixed << std::setprecision(1) << record.sample_rate_hz << " Hz, "
          << std::setprecision(3) << record.duration_s() << " s";
  if (record.rows_dropped_duplicate_time > 0) {
    summary << "; " << record.rows_dropped_duplicate_time << " logged dropout(s)";
  }

  Artifact artifact;
  artifact.kind = "measured_record";
  artifact.summary = summary.str();
  artifact.payload = record;
  return artifact;
}

// --- data.window -----------------------------------------------------------
//
// The estimation/validation split, done where it can be CHECKED. See
// include/galata/data/window.hpp for why this is a capability rather than an
// option on the readers: two windows of one import keep one source identity and
// gain an interval, and that is the only shape in which `identify.validate` can
// prove a split rather than accept a claim about one.
Artifact window_capability(const StageContext& context) {
  const auto& record = context.upstream_at("record").payload_as<data::Record>("measured_record");
  const double start_s = context.input->number_at("start_time_s");
  const double end_s = context.input->number_at("end_time_s");
  const data::Record cut = data::window_record(record, start_s, end_s);

  std::ostringstream summary;
  summary << cut.sample_count() << " of " << record.sample_count() << " sample(s) in ["
          << std::fixed << std::setprecision(3) << start_s << ", " << end_s << ") s; "
          << cut.samples_outside_window << " outside";

  Artifact artifact;
  artifact.kind = "measured_record";
  artifact.summary = summary.str();
  artifact.payload = cut;
  return artifact;
}

// --- report.record ---------------------------------------------------------
//
// A measured record, written out. The missing counterpart to `data.import.*`:
// until this existed, a record could be imported and consumed and there was no
// way to see what galata had actually decoded — which made the reader's
// agreement with an independent parser unverifiable from outside the test
// suite, and ADR-0016 asks for exactly that verification.
//
// TWO FILES, BOTH REQUIRED. The CSV carries the samples; the evidence file
// carries each channel's unit, frame, source name and the scale and offset
// applied on the way in, plus what the import had to drop. A record's numbers
// without their units and frames are not measurements — the same argument
// `model.quadrotor.export` makes about a model — so neither path is optional.
Artifact write_record(const StageContext& context) {
  const auto& record = context.upstream_at("record").payload_as<data::Record>("measured_record");
  const std::string path = context.input->string_at("path");
  const std::string evidence_path = context.input->string_at("evidence_path");
  if (path == evidence_path) {
    throw std::invalid_argument(
        "report.record: `path` and `evidence_path` name the same file. The samples and the "
        "account of what they are are two documents and one would overwrite the other");
  }

  std::ostringstream csv;
  csv.imbue(std::locale::classic());
  csv << std::setprecision(std::numeric_limits<double>::max_digits10);
  csv << "time_s";
  for (const data::Channel& channel : record.channels) {
    csv << ',' << channel.name;
  }
  csv << '\n';
  for (std::size_t k = 0; k < record.times_s.size(); ++k) {
    csv << record.times_s[k];
    for (const data::Channel& channel : record.channels) {
      csv << ',' << channel.samples[k];
    }
    csv << '\n';
  }
  context.write_output(path, csv.str());

  std::ostringstream evidence;
  evidence.imbue(std::locale::classic());
  evidence << std::setprecision(std::numeric_limits<double>::max_digits10);
  evidence << "# SPDX-License-Identifier: Apache-2.0\n";
  evidence << "#\n";
  evidence << "# Written by `report.record`. What the samples in the CSV beside this ARE:\n";
  evidence << "# the unit and frame a human declared for each, what the source held before\n";
  evidence << "# conversion, and what the import had to drop. Not a study input.\n\n";
  evidence << "samples_csv: \"" << path << "\"\n";
  evidence << "source_path: \"" << record.source_path << "\"\n";
  evidence << "source_sha256: \"" << record.source_sha256 << "\"\n";
  evidence << "sample_count: " << record.sample_count() << "\n";
  evidence << "first_time_s: " << record.first_time_s() << "\n";
  evidence << "last_time_s: " << record.last_time_s() << "\n";
  evidence << "timebase: \""
           << (record.timebase == data::Timebase::UniformResampled ? "uniform_resampled"
                                                                   : "source_timestamps")
           << "\"\n";
  if (record.timebase == data::Timebase::UniformResampled) {
    evidence << "sample_rate_hz: " << record.sample_rate_hz << "\n";
  }
  evidence << "is_window: " << (record.is_window ? "true" : "false") << "\n";
  if (record.is_window) {
    evidence << "window_start_s: " << record.window_start_s << "\n";
    evidence << "window_end_s: " << record.window_end_s << "\n";
    evidence << "samples_outside_window: " << record.samples_outside_window << "\n";
  }
  evidence << "\nimport:\n";
  evidence << "  rows_read: " << record.rows_read << "\n";
  evidence << "  rows_dropped_nonfinite: " << record.rows_dropped_nonfinite << "\n";
  evidence << "  rows_dropped_duplicate_time: " << record.rows_dropped_duplicate_time << "\n";
  evidence << "  samples_altered: " << record.samples_altered << "\n";
  evidence << "\nchannels:\n";
  for (const data::Channel& channel : record.channels) {
    evidence << "  - name: \"" << channel.name << "\"\n";
    evidence << "    source_name: \"" << channel.source_name << "\"\n";
    evidence << "    unit: \"" << channel.unit << "\"\n";
    evidence << "    frame: \"" << channel.frame << "\"\n";
    if (!channel.source_unit.empty()) {
      evidence << "    source_unit: \"" << channel.source_unit << "\"\n";
    }
    evidence << "    scale_applied: " << channel.scale_applied << "\n";
    evidence << "    offset_applied: " << channel.offset_applied << "\n";
  }
  context.write_output(evidence_path, evidence.str());

  std::ostringstream summary;
  summary << "wrote " << record.sample_count() << " sample(s) of " << record.channels.size()
          << " channel(s) to " << path << ", with their units and frames in " << evidence_path;

  Artifact artifact;
  artifact.kind = "report";
  artifact.summary = summary.str();
  artifact.payload = context.resolve_output_path(path);
  return artifact;
}

}  // namespace

void register_data_capabilities(Registry& registry) {
  registry.add(Capability{
      "data.import.csv",
      "Read a measured record from delimited text under a declared unit, frame and timebase "
      "mapping, refusing anything the study has not accounted for",
      "measured_record",
      Capability::State::ImplementedUnvalidated,
      import_csv,
      {"path",
       "time_column",
       "time_scale_to_seconds",
       "channels",
       "ignore_columns",
       "missing",
       "description"},
      {"path"}});

  registry.add(Capability{
      "data.import.ulog",
      "Read a measured record from a PX4 ULog under a declared channel, unit and frame mapping, "
      "resampled onto one timebase by zero-order hold",
      "measured_record",
      Capability::State::ImplementedUnvalidated,
      import_ulog,
      {"path", "channels", "resample_hz", "description"},
      {"path"}});

  registry.add(Capability{
      "report.record",
      "Write an imported record as CSV, with each channel's unit, frame and applied conversion "
      "in a required evidence file beside it",
      "report",
      Capability::State::ImplementedUnvalidated,
      write_record,
      {"record", "path", "evidence_path"},
      {},
      {"path", "evidence_path"}});

  registry.add(
      Capability{"data.window",
                 "Cut a measured record to a half-open time window of itself, keeping the source "
                 "identity so that two windows of one import are provably disjoint",
                 "measured_record",
                 Capability::State::ImplementedUnvalidated,
                 window_capability,
                 {"record", "start_time_s", "end_time_s"}});
}

}  // namespace galata::pipeline
