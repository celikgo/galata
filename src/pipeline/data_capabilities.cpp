// SPDX-License-Identifier: Apache-2.0
//
// data.import.* — measured records entering galata through a declared contract.

#include "galata/data/csv.hpp"
#include "galata/pipeline/artifacts.hpp"
#include "galata/pipeline/files.hpp"

#include <iomanip>
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
}

}  // namespace galata::pipeline
