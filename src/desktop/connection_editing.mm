// SPDX-License-Identifier: Apache-2.0
#import "connection_editing.hpp"

#include <cmath>

namespace {

NSUInteger inputCount(NSDictionary* block) {
  if ([block[@"kind"] isEqual:@"constant"])
    return 0;
  if ([block[@"kind"] isEqual:@"sum"])
    return [block[@"signs"] isKindOfClass:[NSArray class]] ? [block[@"signs"] count] : 0;
  if ([block[@"kind"] isEqual:@"linear_combination"])
    return [block[@"terms"] isKindOfClass:[NSArray class]] ? [block[@"terms"] count] : 0;
  return 1;
}

}  // namespace

NSArray<NSDictionary*>* GalataConnectionEdit(NSArray<NSDictionary*>* blocks,
                                             NSArray<NSDictionary*>* connections,
                                             NSDictionary* oldWire,
                                             NSDictionary* proposedWire,
                                             NSString** error) {
  if (error)
    *error = nil;
  const auto refuse = [&](NSString* message) -> NSArray<NSDictionary*>* {
    if (error)
      *error = message;
    return nil;
  };
  if (![blocks isKindOfClass:[NSArray class]] || ![connections isKindOfClass:[NSArray class]]
      || ![proposedWire isKindOfClass:[NSDictionary class]]
      || (oldWire && ![oldWire isKindOfClass:[NSDictionary class]]))
    return refuse(@"The connection draft is unavailable. Select the connection again.");

  NSString* source = proposedWire[@"source"];
  NSString* target = proposedWire[@"target"];
  if (![source isKindOfClass:[NSString class]] || ![target isKindOfClass:[NSString class]])
    return refuse(@"Choose a source and target block.");
  BOOL knownSource = NO;
  NSDictionary* targetBlock = nil;
  for (NSDictionary* block in blocks) {
    if (![block isKindOfClass:[NSDictionary class]])
      return refuse(@"The block draft is unavailable. Reopen the project before editing.");
    if ([block[@"id"] isEqual:source])
      knownSource = YES;
    if ([block[@"id"] isEqual:target])
      targetBlock = block;
  }
  if (!knownSource || !targetBlock)
    return refuse(@"The source or target block no longer exists. Choose the endpoints again.");

  id input = proposedWire[@"input"];
  if (![input isKindOfClass:[NSNumber class]]
      || CFGetTypeID((__bridge CFTypeRef)input) == CFBooleanGetTypeID())
    return refuse(@"Input index must be a nonnegative integer (the first input is 0).");
  const double inputValue = [input doubleValue];
  if (!std::isfinite(inputValue) || inputValue < 0 || std::floor(inputValue) != inputValue)
    return refuse(@"Input index must be a nonnegative integer (the first input is 0).");
  const NSUInteger count = inputCount(targetBlock);
  if (inputValue >= static_cast<double>(count)) {
    if (count == 0)
      return refuse(@"The selected target has no input ports. Choose a different target block.");
    return refuse([NSString stringWithFormat:@"%@ has %lu inputs. Choose an index from 0 to %lu.",
                                             target,
                                             (unsigned long)count,
                                             (unsigned long)count - 1]);
  }

  NSUInteger selectedIndex = NSNotFound;
  for (NSUInteger i = 0; i < connections.count; ++i) {
    NSDictionary* wire = connections[i];
    if (![wire isKindOfClass:[NSDictionary class]])
      return refuse(@"The connection draft is unavailable. Reopen the project before editing.");
    if (oldWire && [wire isEqualToDictionary:oldWire]) {
      if (selectedIndex != NSNotFound)
        return refuse(
            @"The selected connection occurs more than once. Disconnect its duplicate first.");
      selectedIndex = i;
    }
  }
  if (oldWire && selectedIndex == NSNotFound)
    return refuse(
        @"The selected connection no longer exists. Select a current connection to edit.");
  for (NSUInteger i = 0; i < connections.count; ++i) {
    NSDictionary* wire = connections[i];
    if (i != selectedIndex && [wire[@"target"] isEqual:target] && [wire[@"input"] isEqual:input])
      return refuse(@"That input already has a source. Select its wire to reconnect it, or choose "
                    @"a free input.");
  }

  NSMutableArray<NSDictionary*>* result = [NSMutableArray arrayWithCapacity:connections.count + 1];
  for (NSUInteger i = 0; i < connections.count; ++i)
    [result addObject:[(i == selectedIndex ? proposedWire : connections[i]) mutableCopy]];
  if (!oldWire)
    [result addObject:[proposedWire mutableCopy]];
  return result;
}
