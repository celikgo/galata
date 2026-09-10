// SPDX-License-Identifier: Apache-2.0
//
// data.import.* — measured records entering galata through a declared contract.

#include "galata/data/csv.hpp"
#include "galata/identify/static_fit.hpp"
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

// --- identify.static_fit ----------------------------------------------------
//
// The library API does the work; this is the study-facing shape of it. Every
// term, its power, its name and its unit are declared, because a coefficient
// whose meaning was chosen by software is not a measurement of anything.
Artifact static_fit_capability(const StageContext& context) {
  const auto& record = context.upstream_at("record").payload_as<data::Record>("measured_record");

  identify::StaticFitRequest request;
  request.response_channel = context.input->string_at("response");
  request.intercept = context.input->bool_at("intercept", false);
  request.intercept_name = context.input->string_at("intercept_name", "intercept");
  request.intercept_unit = context.input->string_at("intercept_unit", "");
  request.maximum_condition_number = context.input->number_at("maximum_condition_number", 1e8);

  const ValuePtr terms = context.input->get("terms");
  if (!terms) {
    throw std::invalid_argument("identify.static_fit: `terms` is required");
  }
  for (const ValuePtr& entry : terms->as_list()) {
    identify::Term term;
    term.channel = entry->string_at("channel");
    term.power = entry->number_at("power");
    term.name = entry->string_at("name");
    term.unit = entry->string_at("unit");
    request.terms.push_back(std::move(term));
  }

  const identify::StaticFit fit = identify::fit_static(record, request);

  std::ostringstream summary;
  summary << fit.coefficients.size() << " coefficient(s) from " << fit.sample_count
          << " sample(s); residual RMS " << std::scientific << std::setprecision(3)
          << fit.residual_rms;
  // Whether an uncertainty exists is part of the headline, not a detail: a
  // reader who skims the summary must not come away thinking one was reported
  // when none could be.
  summary << (fit.uncertainty_is_estimable ? "; standard errors reported"
                                           : "; NO uncertainty (" + fit.uncertainty_note + ")");

  Artifact artifact;
  artifact.kind = "static_fit";
  artifact.summary = summary.str();
  artifact.payload = fit;
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
      "identify.static_fit",
      "Fit a response that is linear in declared terms — a bench map — reporting the range it "
      "was measured over and an uncertainty only where the data supports one",
      "static_fit",
      Capability::State::ImplementedUnvalidated,
      static_fit_capability,
      {"record",
       "response",
       "terms",
       "intercept",
       "intercept_name",
       "intercept_unit",
       "maximum_condition_number"}});
}

}  // namespace galata::pipeline
