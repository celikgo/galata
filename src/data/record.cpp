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

double Record::duration_s() const {
  if (times_s.size() < 2) {
    return 0.0;
  }
  return times_s.back() - times_s.front();
}

}  // namespace galata::data
