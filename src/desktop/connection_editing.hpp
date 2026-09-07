// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_DESKTOP_CONNECTION_EDITING_HPP
#define GALATA_DESKTOP_CONNECTION_EDITING_HPP

#import <Foundation/Foundation.h>

// Draft editing only; signal types and algebraic loops remain the shared
// compiler's responsibility. Pass nil for oldWire to append a connection, or an
// existing wire to replace it at the same index. The selected wire must occur
// exactly once. Unknown endpoints, invalid input indices and an occupied target
// input are refused without modifying any argument.
//
// On success, returns a new array with new mutable wire dictionaries, preserving
// unrelated wire values and order, and clears error. An unchanged replacement
// is successful and equivalent to the input. On refusal, returns nil and writes
// a user-facing message to error when supplied.
NSArray<NSDictionary*>* GalataConnectionEdit(NSArray<NSDictionary*>* blocks,
                                             NSArray<NSDictionary*>* connections,
                                             NSDictionary* oldWire,
                                             NSDictionary* proposedWire,
                                             NSString** error);

#endif
