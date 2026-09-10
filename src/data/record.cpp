// SPDX-License-Identifier: Apache-2.0

#include "galata/data/record.hpp"

#include <stdexcept>

namespace galata::data {

const Channel* Record::find(const std::string& name) const {
  for (const Channel& channel : channels) {
    if (channel.name == name) {
      return &channel;
    }
  }
  return nullptr;
}

double Record::first_time_s() const {
  return times_s.empty() ? 0.0 : times_s.front();
}

double Record::last_time_s() const {
  return times_s.empty() ? 0.0 : times_s.back();
}

double Record::duration_s() const {
  if (times_s.size() < 2) {
    return 0.0;
  }
  return times_s.back() - times_s.front();
}

}  // namespace galata::data
