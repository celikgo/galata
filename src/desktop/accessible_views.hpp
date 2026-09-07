// SPDX-License-Identifier: Apache-2.0
#pragma once

#import <AppKit/AppKit.h>

NS_ASSUME_NONNULL_BEGIN

// These native tables present the same draft and retained samples as the canvas
// and plot. They do not own edits, parse artifacts, or recompute results.
@interface GalataBlockList : NSView
@property(nonatomic, readonly) NSTableView* tableView;
@property(nonatomic, copy, nullable) NSString* selectedID;
@property(nonatomic, copy, nullable) void (^selectionHandler)(NSString* _Nullable identifier);
@property(nonatomic, getter=isEnabled) BOOL enabled;
- (void)setBlocks:(NSArray<NSDictionary*>*)blocks
    originalNames:(NSDictionary<NSString*, NSString*>*)originalNames;
@end

@interface GalataTrajectoryTable : NSView
@property(nonatomic, readonly) NSTableView* tableView;
// Keep every retained sample, in artifact order. Virtualized cells format each
// binary64 value with enough significant digits to recover its parsed value.
- (void)setHeaders:(NSArray<NSString*>*)headers rows:(NSArray<NSArray<NSNumber*>*>*)rows;
@end

NS_ASSUME_NONNULL_END
