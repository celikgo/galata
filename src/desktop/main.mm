// SPDX-License-Identifier: Apache-2.0
// Native macOS feasibility preview. Numerical work and durable project writes
// belong to the shared CLI; this process only edits drafts and presents evidence.
#include "galata/build_config.hpp"

#include "accessible_views.hpp"
#include "connection_editing.hpp"
#include "diagram_editing.hpp"
#include "diagram_routing.hpp"
#include "dim_theme.hpp"
#import <AppKit/AppKit.h>
#import <CommonCrypto/CommonDigest.h>

#include <cmath>
#include <limits>
#include <string>

static NSString* JSONText(id value) {
  NSData* data = [NSJSONSerialization dataWithJSONObject:value
                                                 options:NSJSONWritingPrettyPrinted
                                                   error:nil];
  return data ? [[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding] : @"";
}

static NSMutableDictionary* JSONCopy(NSDictionary* value) {
  NSData* data = [NSJSONSerialization dataWithJSONObject:value options:0 error:nil];
  return data ? [NSJSONSerialization JSONObjectWithData:data
                                                options:NSJSONReadingMutableContainers
                                                  error:nil]
              : nil;
}

static NSString* SHA256Hex(NSString* value) {
  NSData* data = [value dataUsingEncoding:NSUTF8StringEncoding];
  unsigned char digest[CC_SHA256_DIGEST_LENGTH];
  CC_SHA256(data.bytes, (CC_LONG)data.length, digest);
  NSMutableString* hex = [NSMutableString stringWithCapacity:CC_SHA256_DIGEST_LENGTH * 2];
  for (NSUInteger index = 0; index < CC_SHA256_DIGEST_LENGTH; ++index)
    [hex appendFormat:@"%02x", digest[index]];
  return hex;
}

static NSUInteger BlockInputCount(NSDictionary* block) {
  if ([block[@"kind"] isEqual:@"constant"])
    return 0;
  if ([block[@"kind"] isEqual:@"sum"])
    return [block[@"signs"] isKindOfClass:[NSArray class]] ? [block[@"signs"] count] : 0;
  if ([block[@"kind"] isEqual:@"linear_combination"])
    return [block[@"terms"] isKindOfClass:[NSArray class]] ? [block[@"terms"] count] : 0;
  return 1;
}

static NSSize BlockCardSize(NSDictionary* block) {
  CGFloat titleWidth = [block[@"id"] sizeWithAttributes:@{
                         NSFontAttributeName: [NSFont systemFontOfSize:13
                                                                weight:NSFontWeightSemibold]
                       }]
                           .width;
  NSUInteger ports = MIN((NSUInteger)64, BlockInputCount(block));
  return NSMakeSize(fmax(180, fmin(320, titleWidth + 24)),
                    fmax(72, ports ? 24 + 16 * (ports - 1) : 72));
}

static NSPoint BlockInputPoint(NSRect rect, NSUInteger count, NSUInteger slot) {
  return NSMakePoint(NSMinX(rect), count <= 1 ? NSMidY(rect) : NSMinY(rect) + 12 + 16 * slot);
}

static NSColor* WireColor(NSUInteger index) {
  const unsigned int colors[] = {
      0x58C7FF, 0xF5B95F, 0xC99AFF, 0x5AD7B1, 0xFF889A, 0xD2DC75, 0x8BAFFF, 0xEB9CDA};
  return DimRGB(colors[index % 8]);
}

static NSUInteger WireStyle(NSDictionary* wire) {
  NSString* key = [NSString stringWithFormat:@"%@:%@", wire[@"target"], wire[@"input"]];
  NSUInteger hash = 2166136261U;
  for (unsigned char byte : std::string(key.UTF8String))
    hash = (hash ^ byte) * 16777619U;
  return hash;
}

static NSString* WireName(NSDictionary* wire, NSUInteger index) {
  return [NSString stringWithFormat:@"W%02lu · %@ → %@ / input %@",
                                    (unsigned long)index + 1,
                                    wire[@"source"],
                                    wire[@"target"],
                                    wire[@"input"]];
}

static CGFloat RouteDistance(NSPoint point, NSArray<NSValue*>* route) {
  CGFloat nearest = CGFLOAT_MAX;
  for (NSUInteger i = 1; i < route.count; ++i) {
    NSPoint a = route[i - 1].pointValue, b = route[i].pointValue;
    CGFloat dx = b.x - a.x, dy = b.y - a.y;
    CGFloat length = dx * dx + dy * dy;
    CGFloat t =
        length ? fmax(0, fmin(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / length)) : 0;
    nearest = fmin(nearest, hypot(point.x - a.x - t * dx, point.y - a.y - t * dy));
  }
  return nearest;
}

static BOOL SameWire(NSDictionary* a, NSDictionary* b) {
  return a && b && [a[@"source"] isEqual:b[@"source"]] && [a[@"target"] isEqual:b[@"target"]] &&
         [a[@"input"] isEqual:b[@"input"]];
}

static NSArray<NSValue*>* OffsetRoute(NSArray<NSValue*>* points, NSPoint offset) {
  NSMutableArray* result = [NSMutableArray array];
  for (NSValue* value in points) {
    NSPoint point = value.pointValue;
    [result addObject:[NSValue valueWithPoint:NSMakePoint(point.x + offset.x, point.y + offset.y)]];
  }
  return result;
}

static NSArray<NSValue*>* StoredRoutePoints(NSDictionary* route) {
  NSMutableArray* result = [NSMutableArray array];
  for (NSDictionary* point in route[@"points"])
    [result addObject:[NSValue valueWithPoint:NSMakePoint([point[@"x"] doubleValue],
                                                          [point[@"y"] doubleValue])]];
  return result;
}

static constexpr CGFloat PresentationCoordinateLimit = 100000;

static BOOL CanvasPositionSupported(NSPoint point) {
  return std::isfinite(point.x) && std::isfinite(point.y)
         && fabs(point.x) <= PresentationCoordinateLimit
         && fabs(point.y) <= PresentationCoordinateLimit;
}

static NSString* OriginRelation(NSString* relation) {
  if ([relation isEqual:@"matches_imported_model"])
    return @"matches imported model";
  if ([relation isEqual:@"modified_from_import"])
    return @"modified from import";
  if ([relation isEqual:@"uncompiled_draft"])
    return @"uncompiled draft";
  return @"relation unavailable";
}

static NSTextField* Label(NSString* text, NSRect frame) {
  NSTextField* label = [NSTextField labelWithString:text];
  label.frame = frame;
  label.font = [NSFont systemFontOfSize:12];
  label.textColor = DimText();
  return label;
}

static NSButton* Button(NSString* title, NSRect frame, id target, SEL action) {
  NSButton* button = [NSButton buttonWithTitle:title target:target action:action];
  button.frame = frame;
  button.bezelStyle = NSBezelStyleRounded;
  button.bezelColor = DimRaised();
  return button;
}

// Each JSON editor owns text history. Sharing the window's undo manager would
// let replacing a block inspector also undo text from the simulation editor.
@interface GalataJSONTextView : NSTextView
@property(nonatomic, strong) NSUndoManager* textEdits;
@end

@implementation GalataJSONTextView

- (NSUndoManager*)undoManager {
  if (!self.textEdits) {
    self.textEdits = [[NSUndoManager alloc] init];
    self.textEdits.levelsOfUndo = 50;
  }
  return self.textEdits;
}

@end

static NSTextView* TextEditor(NSView* parent, NSRect frame, BOOL editable) {
  NSScrollView* scroll = [[NSScrollView alloc] initWithFrame:frame];
  scroll.borderType = NSBezelBorder;
  scroll.hasVerticalScroller = YES;
  scroll.backgroundColor = DimBackground();
  DimPanel(scroll, DimBackground(), true);
  NSTextView* text = [[GalataJSONTextView alloc]
      initWithFrame:NSMakeRect(0, 0, frame.size.width, frame.size.height)];
  text.font = [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
  text.richText = NO;
  text.backgroundColor = DimBackground();
  text.textColor = DimText();
  text.insertionPointColor = DimText();
  text.selectedTextAttributes =
      @{NSBackgroundColorAttributeName: DimRaised(), NSForegroundColorAttributeName: DimText()};
  text.editable = editable;
  text.allowsUndo = editable;
  text.automaticQuoteSubstitutionEnabled = NO;
  text.automaticDashSubstitutionEnabled = NO;
  text.automaticTextReplacementEnabled = NO;
  text.textContainerInset = NSMakeSize(7, 7);
  text.autoresizingMask = NSViewWidthSizable;
  text.textContainer.widthTracksTextView = YES;
  scroll.documentView = text;
  [parent addSubview:scroll];
  return text;
}

@class GalataController;

@interface DiagramView : NSView
@property(nonatomic, weak) GalataController* controller;
@property(nonatomic) NSPoint dragOffset;
@property(nonatomic) BOOL dragging;
@property(nonatomic, copy) NSString* draggedID;
@property(nonatomic, copy) NSArray* routeSignature;
@property(nonatomic, copy) NSArray<NSArray<NSValue*>*>* routes;
@property(nonatomic) BOOL wiring;
@property(nonatomic) BOOL wiringMoved;
@property(nonatomic) BOOL movingSource;
@property(nonatomic) NSPoint wirePointer;
@property(nonatomic) NSPoint wireAnchor;
@property(nonatomic, copy) NSDictionary* editingWire;
@property(nonatomic, copy) NSDictionary* fixedPort;
@property(nonatomic, copy) NSDictionary* dropPort;
@property(nonatomic) BOOL validDrop;
@property(nonatomic, copy) NSDictionary* shapingWire;
@property(nonatomic, copy) NSArray<NSValue*>* shapeStart;
@property(nonatomic, copy) NSArray<NSValue*>* shapePreview;
@property(nonatomic) NSUInteger shapeSegment;
@property(nonatomic) BOOL shapeMoved;
@property(nonatomic) NSPoint shapePointerStart;
@property(nonatomic) NSPoint blockPreview;
@property(nonatomic) NSPoint blockPointerStart;
@property(nonatomic) BOOL verticalGuide;
@property(nonatomic) BOOL horizontalGuide;
@property(nonatomic) CGFloat guideX;
@property(nonatomic) CGFloat guideY;
- (void)prepareRoutes;
- (NSRect)contentBounds;
- (void)cancelWiring;
@end

@interface TrajectoryView : NSView
@property(nonatomic, copy) NSArray<NSArray<NSNumber*>*>* samples;
@property(nonatomic, copy) NSArray<NSString*>* columns;
@property(nonatomic) NSUInteger series;
@end

@interface GalataController : NSObject <NSApplicationDelegate, NSWindowDelegate, NSTextViewDelegate>
@property(nonatomic, strong) NSWindow* window;
@property(nonatomic, strong) NSView* workspace;
@property(nonatomic, strong) NSView* connectionBar;
@property(nonatomic, strong) NSView* resultPane;
@property(nonatomic, strong) NSButton* focusDiagramButton;
@property(nonatomic) BOOL diagramFocused;
@property(nonatomic, strong) DiagramView* diagram;
@property(nonatomic, strong) TrajectoryView* plot;
@property(nonatomic, strong) NSScrollView* canvasScroll;
@property(nonatomic, strong) NSButton* zoomButton;
@property(nonatomic, strong) NSButton* snapButton;
@property(nonatomic, strong) NSButton* resetRouteButton;
@property(nonatomic) BOOL snapEnabled;
@property(nonatomic, strong) NSTextField* diagramHint;
@property(nonatomic) BOOL diagramFitMode;
@property(nonatomic) NSPoint diagramOffset;
@property(nonatomic, copy) NSDictionary* highlightedWire;
@property(nonatomic, strong) GalataBlockList* blockList;
@property(nonatomic, strong) GalataTrajectoryTable* samplesTable;
@property(nonatomic, strong) NSSegmentedControl* modelMode;
@property(nonatomic, strong) NSSegmentedControl* resultMode;
@property(nonatomic, strong) NSTextField* sampleSummary;
@property(nonatomic, strong) NSWindow* revisionsWindow;
@property(nonatomic, strong) NSPopUpButton* revisionChoices;
@property(nonatomic, strong) NSTextView* revisionPreview;
@property(nonatomic, strong) NSButton* restoreButton;
@property(nonatomic, copy) NSString* recoveryProject;
@property(nonatomic, copy) NSString* recoveryHead;
@property(nonatomic, copy) NSString* reviewedRevision;
@property(nonatomic, strong) NSDictionary* revisionHistory;
@property(nonatomic, strong) NSTextView* properties;
@property(nonatomic, strong) NSTextView* simulationEditor;
@property(nonatomic, strong) NSTextView* evidence;
@property(nonatomic, strong) NSWindow* originWindow;
@property(nonatomic, strong) NSTextView* originEvidence;
@property(nonatomic, strong) NSTextField* originContext;
@property(nonatomic, strong) NSTextField* status;
@property(nonatomic, strong) NSTextField* selectionLabel;
@property(nonatomic, strong) NSTextField* input;
@property(nonatomic, strong) NSPopUpButton* palette;
@property(nonatomic, strong) NSPopUpButton* source;
@property(nonatomic, strong) NSPopUpButton* target;
@property(nonatomic, strong) NSPopUpButton* wires;
@property(nonatomic, strong) NSButton* connectionButton;
@property(nonatomic, strong) NSButton* removeWireButton;
@property(nonatomic, strong) NSButton* createWireModeButton;
@property(nonatomic, strong) NSPopUpButton* history;
@property(nonatomic, strong) NSPopUpButton* series;
@property(nonatomic, strong) NSButton* saveButton;
@property(nonatomic, strong) NSButton* importButton;
@property(nonatomic, strong) NSButton* originButton;
@property(nonatomic, strong) NSButton* runButton;
@property(nonatomic, strong) NSButton* cancelButton;
@property(nonatomic, strong) NSMutableDictionary* draft;
@property(nonatomic, strong) NSDictionary* projectView;
@property(nonatomic, strong) NSUndoManager* edits;
@property(nonatomic, strong) NSTask* task;
@property(nonatomic, copy) NSString* engine;
@property(nonatomic, copy) NSString* project;
@property(nonatomic, copy) NSString* selectedID;
@property(nonatomic, copy) NSString* revision;
@property(nonatomic, copy) NSString* selectedRunID;
@property(nonatomic, copy) NSString* pendingOpenProject;
@property(nonatomic) BOOL finishedLaunching;
@property(nonatomic) BOOL dirty;
@property(nonatomic) BOOL propertyDirty;
@property(nonatomic) BOOL simulationDirty;
@property(nonatomic) BOOL populating;
@property(nonatomic) BOOL cancelling;
@property(nonatomic, strong) NSTimer* recoveryTimer;
- (NSArray*)blocks;
- (NSArray*)connections;
- (NSMutableDictionary*)selectedBlock;
- (NSRect)rectForBlock:(NSDictionary*)block;
- (NSRect)modelRectForBlock:(NSDictionary*)block;
- (NSString*)originalChannelName:(NSString*)identifier;
- (void)fitDiagram:(id)sender;
- (void)updateDiagramExtent;
- (void)updateDiagramZoom;
- (void)selectBlock:(NSString*)identifier;
- (void)checkpoint:(NSString*)name;
- (NSString*)recoverySnapshotPathForProject:(NSString*)project;
- (void)scheduleRecoverySnapshot;
- (void)persistRecoverySnapshot;
- (void)discardRecoverySnapshotForProject:(NSString*)project;
- (void)offerRecoverySnapshotForProject:(NSString*)project;
- (void)refresh;
- (void)deleteBlock:(id)sender;
- (void)showRevisions:(id)sender;
- (void)selectRevision:(id)sender;
- (void)restoreRevision:(id)sender;
- (void)layoutWorkspace;
- (void)focusDiagram:(id)sender;
- (void)chooseWire:(NSDictionary*)wire;
- (void)newWire:(id)sender;
- (void)disconnect:(id)sender;
- (BOOL)commitProperties;
- (BOOL)applyConnection:(NSDictionary*)wire replacing:(NSDictionary*)oldWire;
- (void)updateConnectionControls;
- (NSArray<NSValue*>*)logicalBlockRects;
- (NSDictionary*)manualRouteForWire:(NSDictionary*)wire;
- (NSArray<NSValue*>*)validManualPoints:(NSDictionary*)route;
- (void)saveRoute:(NSArray<NSValue*>*)points forWire:(NSDictionary*)wire;
- (void)moveBlock:(NSString*)identifier to:(NSPoint)point;
- (void)removeManualRouteForWire:(NSDictionary*)wire;
- (NSString*)promptText:(NSString*)title
                message:(NSString*)message
            placeholder:(NSString*)placeholder;
- (void)verifyQualification:(id)sender;
- (void)verifyOnboardDeployment:(id)sender;
- (void)createOnboardTargetEvidence:(id)sender;
- (void)verifyOnboardTargetEvidence:(id)sender;
- (void)onboardSelfTest:(id)sender;
- (void)deployOnboard:(id)sender;
- (void)createFlightTest:(id)sender;
- (void)validateFlightTest:(id)sender;
- (void)createQualification:(id)sender;
- (void)verifyQualificationChain:(id)sender;
- (void)runStudy:(id)sender;
@end

@implementation DiagramView

- (BOOL)isFlipped {
  return YES;
}

- (BOOL)acceptsFirstResponder {
  return YES;
}

- (void)prepareRoutes {
  NSMutableArray* rects = [NSMutableArray array];
  NSMutableArray* ports = [NSMutableArray array];
  NSMutableDictionary* byID = [NSMutableDictionary dictionary];
  for (NSDictionary* block in [self.controller blocks]) {
    NSValue* rect = [NSValue valueWithRect:[self.controller rectForBlock:block]];
    [rects addObject:rect];
    [ports addObject:@[block[@"id"], @(BlockInputCount(block))]];
    byID[block[@"id"]] = block;
  }
  NSArray* wires = [self.controller connections] ? [self.controller connections] : @[];
  NSArray* signature = @[
    rects,
    ports,
    JSONText(wires),
    JSONText(self.controller.draft[@"presentation"][@"routes"]
                 ? self.controller.draft[@"presentation"][@"routes"]
                 : @[])
  ];
  if ([signature isEqual:self.routeSignature])
    return;
  self.routeSignature = signature;
  NSMutableArray* starts = [NSMutableArray array];
  NSMutableArray* ends = [NSMutableArray array];
  for (NSDictionary* wire in wires) {
    NSDictionary* from = byID[wire[@"source"]];
    NSDictionary* to = byID[wire[@"target"]];
    NSInteger slot = [wire[@"input"] integerValue];
    NSPoint start = NSMakePoint(NAN, NAN), end = start;
    if (from && to && slot >= 0 && (NSUInteger)slot < MIN((NSUInteger)64, BlockInputCount(to))) {
      NSRect a = [self.controller rectForBlock:from];
      NSRect b = [self.controller rectForBlock:to];
      start = NSMakePoint(NSMaxX(a), NSMidY(a));
      end = BlockInputPoint(b, BlockInputCount(to), (NSUInteger)slot);
    }
    [starts addObject:[NSValue valueWithPoint:start]];
    [ends addObject:[NSValue valueWithPoint:end]];
  }
  NSArray* routed = GalataWireRoutes(starts, ends, rects);
  NSMutableArray* routes = [NSMutableArray array];
  NSUInteger unresolved = 0;
  NSUInteger blockedManual = 0;
  for (NSUInteger i = 0; i < wires.count; ++i) {
    NSArray* route = i < routed.count ? routed[i] : @[];
    NSDictionary* saved = [self.controller manualRouteForWire:wires[i]];
    if (saved && rects.count <= 128 && wires.count <= 256) {
      NSArray<NSValue*>* logical = [self.controller validManualPoints:saved];
      NSArray<NSValue*>* displayed = OffsetRoute(logical, self.controller.diagramOffset);
      if (displayed.count >= 2)
        route = displayed;
      else
        ++blockedManual;
    }
    [routes addObject:route];
    if (!route.count)
      ++unresolved;
  }
  self.routes = routes;
  BOOL exceedsBlockLimit = unresolved && rects.count > 128;
  BOOL exceedsWireLimit = unresolved && wires.count > 256;
  self.controller.diagramHint.stringValue =
      exceedsBlockLimit
          ? @"Wire rendering is limited to 128 blocks"
          : (exceedsWireLimit
                 ? @"Wire rendering is limited to 256 connections"
                 : (unresolved
                        ? [NSString
                              stringWithFormat:@"%lu wire(s) not drawn · Check layout or ports",
                                               (unsigned long)unresolved]
                        : (blockedManual ? @"Blocked custom route · Showing automatic path"
                                         : @"Drag wire segments · Snap aligns blocks")));
  self.controller.diagramHint.toolTip =
      exceedsBlockLimit
          ? @"This project exceeds the diagram's wire-rendering block limit. Review blocks and "
            @"connections in the block list and wiring selector. Arranging does not remove this "
            @"limit."
          : (unresolved
                 ? @"Connections may be hidden by overlapping blocks, invalid ports or a diagram "
                   @"rendering limit. Move or arrange overlapping blocks, and review inputs in the "
                   @"wiring selector. Dense diagrams can exceed the rendering limit."
                 : @"Custom routes blocked by moved blocks use an automatic path; select the wire "
                   @"and Reset Route or reshape it. Click a wire to edit its endpoints below. "
                   @"Drag a round middle handle to shape the line. Snap uses a 16-point grid and "
                   @"block alignment guides; hold Option to move freely. Drag an output port to an "
                   @"input to "
                   @"create a wire; drag the square handles of a selected wire to reconnect it. "
                   @"Click again at a crossing to cycle wires. Colors, line patterns and W labels "
                   @"distinguish connections. Escape cancels a drag.");
}

- (NSRect)contentBounds {
  [self prepareRoutes];
  NSRect bounds = NSZeroRect;
  for (NSDictionary* block in [self.controller blocks]) {
    NSRect rect = [self.controller rectForBlock:block];
    bounds = NSIsEmptyRect(bounds) ? rect : NSUnionRect(bounds, rect);
  }
  for (NSArray<NSValue*>* route in self.routes) {
    for (NSValue* value in route) {
      NSPoint point = value.pointValue;
      bounds = NSUnionRect(bounds, NSMakeRect(point.x, point.y, 1, 1));
    }
  }
  return NSIsEmptyRect(bounds) ? NSMakeRect(0, 0, 500, 280) : NSInsetRect(bounds, -36, -28);
}

- (void)drawRect:(NSRect)dirtyRect {
  [DimBackground() setFill];
  NSRectFill(dirtyRect);
  if (self.controller.snapEnabled) {
    CGFloat step = 16;
    while (step * self.controller.canvasScroll.magnification < 12)
      step *= 2;
    NSPoint offset = self.controller.diagramOffset;
    CGFloat startX = ceil((NSMinX(dirtyRect) - offset.x) / step) * step + offset.x;
    CGFloat startY = ceil((NSMinY(dirtyRect) - offset.y) / step) * step + offset.y;
    [[DimBorder() colorWithAlphaComponent:0.65] setFill];
    CGFloat dot = 1.2 / self.controller.canvasScroll.magnification;
    for (CGFloat x = startX; x < NSMaxX(dirtyRect); x += step)
      for (CGFloat y = startY; y < NSMaxY(dirtyRect); y += step)
        NSRectFill(NSMakeRect(x, y, dot, dot));
  }
  NSDictionary* textStyle = @{
    NSFontAttributeName: [NSFont systemFontOfSize:12],
    NSForegroundColorAttributeName: DimMuted()
  };
  if (!self.controller.draft) {
    [@"Create or open a .galata project to begin." drawAtPoint:NSMakePoint(28, 34)
                                                withAttributes:textStyle];
    return;
  }
  [self prepareRoutes];
  NSArray* wires = [self.controller connections];
  // Draw highlighted routes last so each chosen connection can be followed.
  for (NSUInteger pass = 0; pass < 2; ++pass) {
    for (NSUInteger index = 0; index < wires.count; ++index) {
      NSDictionary* wire = wires[index];
      NSUInteger style = WireStyle(wire);
      BOOL highlighted = self.controller.highlightedWire
                             ? [wire isEqual:self.controller.highlightedWire]
                             : ([wire[@"source"] isEqual:self.controller.selectedID] ||
                                [wire[@"target"] isEqual:self.controller.selectedID]);
      if (highlighted != (pass == 1))
        continue;
      NSArray<NSValue*>* route = self.routes[index];
      if (SameWire(wire, self.shapingWire) && self.shapePreview)
        route = OffsetRoute(self.shapePreview, self.controller.diagramOffset);
      if (route.count < 2)
        continue;
      NSBezierPath* path = [NSBezierPath bezierPath];
      [path moveToPoint:route.firstObject.pointValue];
      for (NSUInteger point = 1; point < route.count; ++point)
        [path lineToPoint:route[point].pointValue];
      path.lineJoinStyle = NSLineJoinStyleRound;
      // A background gap makes crossings distinct from junctions. Each wire
      // also retains a color, pattern and W label when selection changes.
      path.lineWidth = highlighted ? 7 : 4.5;
      [DimBackground() setStroke];
      [path stroke];
      path.lineWidth = highlighted ? 3 : 1.8;
      if ((style / 8) % 3 == 1) {
        const CGFloat dash[] = {9, 4};
        [path setLineDash:dash count:2 phase:0];
      } else if ((style / 8) % 3 == 2) {
        const CGFloat dash[] = {2, 3};
        [path setLineDash:dash count:2 phase:0];
      }
      CGFloat alpha = self.controller.highlightedWire && !highlighted ? 0.30 : 0.95;
      [[WireColor(style) colorWithAlphaComponent:alpha] setStroke];
      [path stroke];
      NSPoint end = route.lastObject.pointValue;
      NSBezierPath* arrow = [NSBezierPath bezierPath];
      [arrow moveToPoint:NSMakePoint(end.x - 6, end.y - 3)];
      [arrow lineToPoint:end];
      [arrow lineToPoint:NSMakePoint(end.x - 6, end.y + 3)];
      arrow.lineWidth = highlighted ? 2.5 : 1.6;
      [arrow stroke];
    }
  }
  NSMutableParagraphStyle* titleStyle = [[NSMutableParagraphStyle alloc] init];
  titleStyle.lineBreakMode = NSLineBreakByTruncatingTail;
  for (NSDictionary* block in [self.controller blocks]) {
    NSRect rect = [self.controller rectForBlock:block];
    if (!NSIntersectsRect(NSInsetRect(rect, -30, -6), dirtyRect))
      continue;
    BOOL selected = [block[@"id"] isEqual:self.controller.selectedID];
    NSBezierPath* card = [NSBezierPath bezierPathWithRoundedRect:rect xRadius:7 yRadius:7];
    [DimSurface() setFill];
    [card fill];
    [(selected ? DimAccent() : DimBorder()) setStroke];
    card.lineWidth = selected ? 2.5 : 1;
    [card stroke];
    [block[@"id"]
            drawInRect:NSMakeRect(rect.origin.x + 10, rect.origin.y + 10, rect.size.width - 20, 19)
        withAttributes:@{
          NSFontAttributeName: [NSFont systemFontOfSize:13 weight:NSFontWeightSemibold],
          NSForegroundColorAttributeName: DimText(),
          NSParagraphStyleAttributeName: titleStyle
        }];
    NSString* kind = [block[@"kind"] isEqual:@"linear_combination"]
                         ? [NSString stringWithFormat:@"linear row · %lu inputs",
                                                      (unsigned long)BlockInputCount(block)]
                         : block[@"kind"];
    [kind drawAtPoint:NSMakePoint(rect.origin.x + 10, rect.origin.y + 33) withAttributes:textStyle];
    NSString* original = [self.controller originalChannelName:block[@"id"]];
    if (original) {
      [[NSString stringWithFormat:@"original: %@", original]
              drawInRect:NSMakeRect(
                             rect.origin.x + 10, rect.origin.y + 51, rect.size.width - 20, 15)
          withAttributes:@{
            NSFontAttributeName: [NSFont systemFontOfSize:10],
            NSForegroundColorAttributeName: DimMuted(),
            NSParagraphStyleAttributeName: titleStyle
          }];
    }
    NSUInteger count = MIN((NSUInteger)64, BlockInputCount(block));
    for (NSUInteger port = 0; port < count; ++port) {
      NSPoint point = BlockInputPoint(rect, count, port);
      NSBezierPath* dot =
          [NSBezierPath bezierPathWithOvalInRect:NSMakeRect(point.x - 4, point.y - 4, 8, 8)];
      [DimMuted() setFill];
      [dot fill];
      NSDictionary* portStyle = @{
        NSFontAttributeName: [NSFont monospacedDigitSystemFontOfSize:10 weight:NSFontWeightMedium],
        NSForegroundColorAttributeName: DimText()
      };
      [[NSString stringWithFormat:@"%lu", (unsigned long)port]
             drawAtPoint:NSMakePoint(point.x - 21, point.y - 12)
          withAttributes:portStyle];
    }
    NSPoint out = NSMakePoint(NSMaxX(rect), NSMidY(rect));
    [DimMuted() setFill];
    [[NSBezierPath bezierPathWithOvalInRect:NSMakeRect(out.x - 4, out.y - 4, 8, 8)] fill];
  }
  for (NSUInteger index = 0; index < wires.count; ++index) {
    NSArray<NSValue*>* route = self.routes[index];
    if (SameWire(wires[index], self.shapingWire) && self.shapePreview)
      route = OffsetRoute(self.shapePreview, self.controller.diagramOffset);
    if (route.count < 2)
      continue;
    NSPoint end = route.lastObject.pointValue;
    NSString* label = [NSString stringWithFormat:@"W%02lu", (unsigned long)index + 1];
    NSDictionary* attributes = @{
      NSFontAttributeName: [NSFont monospacedSystemFontOfSize:9 weight:NSFontWeightSemibold],
      NSForegroundColorAttributeName: WireColor(WireStyle(wires[index])),
      NSBackgroundColorAttributeName: DimBackground()
    };
    [label drawAtPoint:NSMakePoint(end.x - 52, end.y - 12) withAttributes:attributes];
    if ([wires[index] isEqual:self.controller.highlightedWire]) {
      for (NSUInteger segment = 0; segment + 1 < route.count; ++segment) {
        NSPoint a = route[segment].pointValue, b = route[segment + 1].pointValue;
        if (hypot(b.x - a.x, b.y - a.y) * self.controller.canvasScroll.magnification < 28)
          continue;
        NSPoint middle = NSMakePoint((a.x + b.x) / 2, (a.y + b.y) / 2);
        CGFloat radius = 4 / self.controller.canvasScroll.magnification;
        NSBezierPath* handle = [NSBezierPath bezierPathWithOvalInRect:NSMakeRect(middle.x - radius,
                                                                                 middle.y - radius,
                                                                                 radius * 2,
                                                                                 radius * 2)];
        [DimBackground() setFill];
        [handle fill];
        [DimText() setStroke];
        handle.lineWidth = 1.4 / self.controller.canvasScroll.magnification;
        [handle stroke];
      }
      for (NSValue* value in @[route.firstObject, route.lastObject]) {
        NSPoint point = value.pointValue;
        CGFloat radius = fmax(5, 5 / self.controller.canvasScroll.magnification);
        NSBezierPath* handle = [NSBezierPath
            bezierPathWithRect:NSMakeRect(
                                   point.x - radius, point.y - radius, radius * 2, radius * 2)];
        [WireColor(WireStyle(wires[index])) setFill];
        [handle fill];
        [DimText() setStroke];
        handle.lineWidth = 1.5 / self.controller.canvasScroll.magnification;
        [handle stroke];
      }
    }
  }
  if (self.verticalGuide || self.horizontalGuide) {
    NSBezierPath* guide = [NSBezierPath bezierPath];
    NSRect visible = self.visibleRect;
    if (self.verticalGuide) {
      CGFloat x = self.guideX + self.controller.diagramOffset.x;
      [guide moveToPoint:NSMakePoint(x, NSMinY(visible))];
      [guide lineToPoint:NSMakePoint(x, NSMaxY(visible))];
    }
    if (self.horizontalGuide) {
      CGFloat y = self.guideY + self.controller.diagramOffset.y;
      [guide moveToPoint:NSMakePoint(NSMinX(visible), y)];
      [guide lineToPoint:NSMakePoint(NSMaxX(visible), y)];
    }
    CGFloat dash[] = {5 / self.controller.canvasScroll.magnification,
                      4 / self.controller.canvasScroll.magnification};
    [guide setLineDash:dash count:2 phase:0];
    guide.lineWidth = 1 / self.controller.canvasScroll.magnification;
    [(self.shapingWire && !self.shapePreview ? DimRGB(0xFF889A) : DimRGB(0x58C7FF)) setStroke];
    [guide stroke];
  }
  if (self.wiring && self.wiringMoved) {
    NSPoint from = self.movingSource ? self.wirePointer : self.wireAnchor;
    NSPoint to = self.movingSource ? self.wireAnchor : self.wirePointer;
    NSBezierPath* preview = [NSBezierPath bezierPath];
    [preview moveToPoint:from];
    CGFloat middle = (from.x + to.x) / 2;
    [preview lineToPoint:NSMakePoint(middle, from.y)];
    [preview lineToPoint:NSMakePoint(middle, to.y)];
    [preview lineToPoint:to];
    const CGFloat dash[] = {7, 5};
    [preview setLineDash:dash count:2 phase:0];
    preview.lineWidth = 2.5 / self.controller.canvasScroll.magnification;
    [(self.dropPort ? (self.validDrop ? DimRGB(0x5AD7B1) : DimRGB(0xFF889A))
                    : DimText()) setStroke];
    [preview stroke];
    [[NSBezierPath
        bezierPathWithOvalInRect:NSMakeRect(self.wirePointer.x - 7, self.wirePointer.y - 7, 14, 14)]
        stroke];
  }
}

- (NSDictionary*)portAtPoint:(NSPoint)point side:(NSInteger)side {
  CGFloat nearest = 9 / self.controller.canvasScroll.magnification;
  NSDictionary* result = nil;
  for (NSDictionary* block in [self.controller blocks]) {
    NSRect rect = [self.controller rectForBlock:block];
    NSUInteger count = MIN((NSUInteger)64, BlockInputCount(block));
    for (NSInteger slot = -1; slot < (NSInteger)count; ++slot) {
      if ((side == 1 && slot >= 0) || (side == 0 && slot < 0))
        continue;
      NSPoint port = slot < 0 ? NSMakePoint(NSMaxX(rect), NSMidY(rect))
                              : BlockInputPoint(rect, BlockInputCount(block), (NSUInteger)slot);
      CGFloat distance = hypot(point.x - port.x, point.y - port.y);
      if (distance <= nearest) {
        nearest = distance;
        result =
            @{@"block": block[@"id"], @"input": @(slot), @"point": [NSValue valueWithPoint:port]};
      }
    }
  }
  return result;
}

- (NSDictionary*)proposedWire {
  if (!self.dropPort || !self.fixedPort)
    return nil;
  NSDictionary* source = self.movingSource ? self.dropPort : self.fixedPort;
  NSDictionary* target = self.movingSource ? self.fixedPort : self.dropPort;
  return @{@"source": source[@"block"], @"target": target[@"block"], @"input": target[@"input"]};
}

- (void)cancelWiring {
  self.wiring = NO;
  self.wiringMoved = NO;
  self.editingWire = nil;
  self.fixedPort = nil;
  self.dropPort = nil;
  self.shapingWire = nil;
  self.shapeStart = nil;
  self.shapePreview = nil;
  self.shapeMoved = NO;
  self.dragging = NO;
  self.draggedID = nil;
  self.verticalGuide = NO;
  self.horizontalGuide = NO;
  self.needsDisplay = YES;
}

- (void)beginShape:(NSDictionary*)wire segment:(NSUInteger)segment point:(NSPoint)point {
  NSUInteger index = [[self.controller connections] indexOfObject:wire];
  if (index >= self.routes.count || segment + 1 >= [self.routes[index] count])
    return;
  self.shapingWire = wire;
  self.shapeSegment = segment;
  NSPoint offset = self.controller.diagramOffset;
  self.shapeStart = OffsetRoute(self.routes[index], NSMakePoint(-offset.x, -offset.y));
  self.shapePreview = self.shapeStart;
  self.shapePointerStart = point;
  self.shapeMoved = NO;
}

- (void)updateShapeDrag:(NSEvent*)event {
  NSPoint point = [self convertPoint:event.locationInWindow fromView:nil];
  if (!self.shapeMoved
      && hypot(point.x - self.shapePointerStart.x, point.y - self.shapePointerStart.y)
                 * self.controller.canvasScroll.magnification
             < 3)
    return;
  self.shapeMoved = YES;
  self.controller.diagramFitMode = NO;
  [self autoscroll:event];
  point = [self convertPoint:event.locationInWindow fromView:nil];
  NSPoint a = self.shapeStart[self.shapeSegment].pointValue;
  NSPoint b = self.shapeStart[self.shapeSegment + 1].pointValue;
  BOOL horizontal = a.y == b.y;
  CGFloat coordinate = horizontal ? a.y + point.y - self.shapePointerStart.y
                                  : a.x + point.x - self.shapePointerStart.x;
  BOOL snap = self.controller.snapEnabled && !(event.modifierFlags & NSEventModifierFlagOption);
  if (snap)
    coordinate = GalataSnapToGrid(coordinate);
  self.horizontalGuide = horizontal && snap;
  self.verticalGuide = !horizontal && snap;
  self.guideX = coordinate;
  self.guideY = coordinate;
  self.shapePreview = GalataMoveWireSegment(
      self.shapeStart, self.shapeSegment, coordinate, [self.controller logicalBlockRects]);
  self.controller.status.stringValue =
      self.shapePreview
          ? @"Release to keep this wire shape. Endpoints stay connected. Option bypasses snap; "
            @"Escape cancels."
          : @"That wire shape crosses a block or exceeds the canvas bounds. Choose a clear lane.";
  if (self.shapePreview) {
    for (NSValue* value in OffsetRoute(self.shapePreview, self.controller.diagramOffset)) {
      NSPoint p = value.pointValue;
      [self setFrameSize:NSMakeSize(fmax(self.frame.size.width, p.x + 100),
                                    fmax(self.frame.size.height, p.y + 100))];
    }
  }
  self.needsDisplay = YES;
}

- (void)updateBlockDrag:(NSEvent*)event {
  NSPoint point = [self convertPoint:event.locationInWindow fromView:nil];
  if (!self.dragging
      && hypot(point.x - self.blockPointerStart.x, point.y - self.blockPointerStart.y)
                 * self.controller.canvasScroll.magnification
             < 3)
    return;
  self.dragging = YES;
  self.controller.diagramFitMode = NO;
  [self autoscroll:event];
  point = [self convertPoint:event.locationInWindow fromView:nil];
  NSRect proposed = [self.controller modelRectForBlock:[self.controller selectedBlock]];
  proposed.origin =
      NSMakePoint(fmax(32, point.x - self.dragOffset.x) - self.controller.diagramOffset.x,
                  fmax(32, point.y - self.dragOffset.y) - self.controller.diagramOffset.y);
  self.verticalGuide = NO;
  self.horizontalGuide = NO;
  if (self.controller.snapEnabled && !(event.modifierFlags & NSEventModifierFlagOption)) {
    NSMutableArray* peers = [NSMutableArray array];
    for (NSDictionary* block in [self.controller blocks]) {
      if (![block[@"id"] isEqual:self.draggedID])
        [peers addObject:[NSValue valueWithRect:[self.controller modelRectForBlock:block]]];
    }
    GalataBlockSnapResult snap =
        GalataSnapBlock(proposed, peers, self.controller.canvasScroll.magnification, YES);
    proposed.origin = snap.origin;
    self.verticalGuide = snap.hasVerticalGuide;
    self.horizontalGuide = snap.hasHorizontalGuide;
    self.guideX = snap.verticalGuide;
    self.guideY = snap.horizontalGuide;
  }
  self.blockPreview = NSMakePoint(
      fmax(-PresentationCoordinateLimit, fmin(PresentationCoordinateLimit, proposed.origin.x)),
      fmax(-PresentationCoordinateLimit, fmin(PresentationCoordinateLimit, proposed.origin.y)));
  NSRect moved = [self.controller rectForBlock:[self.controller selectedBlock]];
  [self setFrameSize:NSMakeSize(fmax(self.frame.size.width, NSMaxX(moved) + 100),
                                fmax(self.frame.size.height, NSMaxY(moved) + 100))];
  self.controller.status.stringValue =
      @"Release to place the block. Snap aligns grid, edges and centers; Option moves freely.";
  self.needsDisplay = YES;
}

- (BOOL)beginWireAtPort:(NSDictionary*)port editing:(NSDictionary*)wire movingSource:(BOOL)source {
  if (![self.controller commitProperties])
    return NO;
  if (wire) {
    [self.controller chooseWire:wire];
    for (NSDictionary* block in [self.controller blocks]) {
      NSString* identifier = source ? wire[@"target"] : wire[@"source"];
      if (![block[@"id"] isEqual:identifier])
        continue;
      NSRect rect = [self.controller rectForBlock:block];
      NSInteger slot = source ? [wire[@"input"] integerValue] : -1;
      self.wireAnchor = source ? BlockInputPoint(rect, BlockInputCount(block), (NSUInteger)slot)
                               : NSMakePoint(NSMaxX(rect), NSMidY(rect));
      self.fixedPort = @{
        @"block": identifier,
        @"input": @(slot),
        @"point": [NSValue valueWithPoint:self.wireAnchor]
      };
    }
  } else {
    [self.controller newWire:self];
    self.fixedPort = port;
    self.wireAnchor = [port[@"point"] pointValue];
  }
  self.wiring = YES;
  self.wiringMoved = NO;
  self.movingSource = source;
  self.editingWire = wire;
  self.wirePointer = [port[@"point"] pointValue];
  self.controller.status.stringValue = source ? @"Drag to an output port. Escape cancels."
                                              : @"Drag to an input port. Escape cancels.";
  return YES;
}

- (void)mouseDown:(NSEvent*)event {
  [self cancelWiring];
  self.draggedID = nil;
  self.dragging = NO;
  if (self.controller.task || ![self.controller commitProperties])
    return;
  [self.window makeFirstResponder:self];
  NSPoint point = [self convertPoint:event.locationInWindow fromView:nil];
  [self prepareRoutes];
  NSDictionary* chosen = self.controller.highlightedWire;
  NSUInteger chosenIndex = [[self.controller connections] indexOfObject:chosen];
  if (chosen && chosenIndex < self.routes.count && [self.routes[chosenIndex] count] >= 2) {
    NSArray<NSValue*>* route = self.routes[chosenIndex];
    for (NSUInteger endpoint = 0; endpoint < 2; ++endpoint) {
      NSPoint handle = endpoint ? route.lastObject.pointValue : route.firstObject.pointValue;
      if (hypot(point.x - handle.x, point.y - handle.y)
          <= 9 / self.controller.canvasScroll.magnification) {
        NSDictionary* port = @{@"point": [NSValue valueWithPoint:handle]};
        [self beginWireAtPort:port editing:chosen movingSource:endpoint == 0];
        return;
      }
    }
    for (NSUInteger segment = 0; segment + 1 < route.count; ++segment) {
      NSPoint a = route[segment].pointValue, b = route[segment + 1].pointValue;
      NSPoint middle = NSMakePoint((a.x + b.x) / 2, (a.y + b.y) / 2);
      if (hypot(b.x - a.x, b.y - a.y) * self.controller.canvasScroll.magnification >= 28
          && hypot(point.x - middle.x, point.y - middle.y)
                 <= 7 / self.controller.canvasScroll.magnification) {
        [self beginShape:chosen segment:segment point:point];
        return;
      }
    }
  }
  NSDictionary* port = [self portAtPoint:point side:-1];
  if (port) {
    NSDictionary* occupied = nil;
    if ([port[@"input"] integerValue] >= 0) {
      for (NSDictionary* wire in [self.controller connections])
        if ([wire[@"target"] isEqual:port[@"block"]] && [wire[@"input"] isEqual:port[@"input"]])
          occupied = wire;
    }
    [self beginWireAtPort:port
                  editing:occupied
             movingSource:!occupied && [port[@"input"] integerValue] >= 0];
    return;
  }
  for (NSDictionary* block in [[self.controller blocks] reverseObjectEnumerator]) {
    NSRect rect = [self.controller rectForBlock:block];
    if (NSPointInRect(point, rect)) {
      [self.controller selectBlock:block[@"id"]];
      if (![self.controller.selectedID isEqual:block[@"id"]])
        return;
      self.dragOffset = NSMakePoint(point.x - rect.origin.x, point.y - rect.origin.y);
      self.draggedID = block[@"id"];
      self.blockPreview = [self.controller modelRectForBlock:block].origin;
      self.blockPointerStart = point;
      return;
    }
  }
  NSMutableArray* hits = [NSMutableArray array];
  for (NSUInteger index = 0; index < self.routes.count; ++index) {
    if (RouteDistance(point, self.routes[index]) <= 7 / self.controller.canvasScroll.magnification)
      [hits addObject:[self.controller connections][index]];
  }
  if (hits.count) {
    NSUInteger current = [hits indexOfObject:self.controller.highlightedWire];
    [self.controller chooseWire:hits[current == NSNotFound ? 0 : (current + 1) % hits.count]];
    point = [self convertPoint:event.locationInWindow fromView:nil];
    [self prepareRoutes];
    NSDictionary* wire = self.controller.highlightedWire;
    NSUInteger index = [[self.controller connections] indexOfObject:wire];
    if (index < self.routes.count) {
      NSArray<NSValue*>* route = self.routes[index];
      CGFloat nearest = CGFLOAT_MAX;
      NSUInteger segment = 0;
      for (NSUInteger i = 0; i + 1 < route.count; ++i) {
        CGFloat distance = RouteDistance(point, @[route[i], route[i + 1]]);
        if (distance < nearest) {
          nearest = distance;
          segment = i;
        }
      }
      [self beginShape:wire segment:segment point:point];
    }
    return;
  }
  [self.controller selectBlock:nil];
}

- (void)mouseDragged:(NSEvent*)event {
  if (self.shapingWire && !self.controller.task) {
    [self updateShapeDrag:event];
    return;
  }
  if (self.wiring && !self.controller.task) {
    self.wiringMoved = YES;
    [self autoscroll:event];
    self.wirePointer = [self convertPoint:event.locationInWindow fromView:nil];
    self.dropPort = [self portAtPoint:self.wirePointer side:self.movingSource ? 1 : 0];
    if (self.dropPort)
      self.wirePointer = [self.dropPort[@"point"] pointValue];
    NSString* error = nil;
    self.validDrop = self.dropPort
                     && GalataConnectionEdit([self.controller blocks],
                                             [self.controller connections],
                                             self.editingWire,
                                             [self proposedWire],
                                             &error)
                            != nil;
    self.controller.status.stringValue =
        self.validDrop ? @"Release to connect. Escape cancels."
                       : (error ? error
                                : (self.movingSource ? @"Drop on an output port. Escape cancels."
                                                     : @"Drop on an input port. Escape cancels."));
    self.needsDisplay = YES;
    return;
  }
  if (!self.draggedID || ![self.draggedID isEqual:self.controller.selectedID]
      || self.controller.task)
    return;
  [self updateBlockDrag:event];
}

- (void)mouseUp:(NSEvent*)event {
  if (self.shapingWire) {
    [self updateShapeDrag:event];
    NSDictionary* wire = self.shapingWire;
    NSArray* points = self.shapePreview;
    BOOL changed = self.shapeMoved && points && ![points isEqual:self.shapeStart];
    BOOL invalid = self.shapeMoved && !points;
    [self cancelWiring];
    if (changed && !self.controller.task)
      [self.controller saveRoute:points forWire:wire];
    else {
      [self.controller refresh];
      if (invalid)
        self.controller.status.stringValue =
            @"Wire shape unchanged. The proposed path was blocked or out of bounds.";
    }
    return;
  }
  if (self.wiring) {
    self.wirePointer = [self convertPoint:event.locationInWindow fromView:nil];
    self.dropPort = [self portAtPoint:self.wirePointer side:self.movingSource ? 1 : 0];
    NSDictionary* proposed = [self proposedWire];
    NSDictionary* oldWire = self.editingWire;
    BOOL moved = self.wiringMoved;
    [self cancelWiring];
    if (moved && proposed && !self.controller.task)
      [self.controller applyConnection:proposed replacing:oldWire];
    else if (moved)
      self.controller.status.stringValue = @"Connection drag cancelled. No wiring changed.";
    return;
  }
  if (self.draggedID && !self.controller.task) {
    [self updateBlockDrag:event];
    NSString* identifier = self.draggedID;
    NSPoint point = self.blockPreview;
    BOOL moved = self.dragging;
    [self cancelWiring];
    if (moved)
      [self.controller moveBlock:identifier to:point];
  }
  [self cancelWiring];
  [self.controller refresh];
}

- (void)magnifyWithEvent:(NSEvent*)event {
  self.controller.diagramFitMode = NO;
  [super magnifyWithEvent:event];
  [self.controller updateDiagramZoom];
}

- (void)keyDown:(NSEvent*)event {
  if (event.keyCode == 53) {
    if (self.wiring || self.shapingWire || self.draggedID) {
      [self cancelWiring];
      [self.controller refresh];
      self.controller.status.stringValue = @"Drag cancelled. No changes were applied.";
    }
  } else if (event.keyCode == 51 || event.keyCode == 117) {
    if (self.controller.highlightedWire)
      [self.controller disconnect:self];
    else
      [self.controller deleteBlock:self];
  } else if (event.keyCode >= 123 && event.keyCode <= 126 && !self.controller.task) {
    NSArray* blocks = [self.controller blocks];
    if (!blocks.count)
      return;
    NSUInteger index = self.controller.selectedID
                           ? [[blocks valueForKey:@"id"] indexOfObject:self.controller.selectedID]
                           : NSNotFound;
    BOOL previous = event.keyCode == 123 || event.keyCode == 126;
    if (index == NSNotFound)
      index = 0;
    else
      index = previous ? (index + blocks.count - 1) % blocks.count : (index + 1) % blocks.count;
    [self.controller selectBlock:blocks[index][@"id"]];
    [self scrollRectToVisible:[self.controller rectForBlock:blocks[index]]];
  } else
    [super keyDown:event];
}

@end

@implementation TrajectoryView

- (void)drawRect:(NSRect)dirtyRect {
  (void)dirtyRect;
  [DimBackground() setFill];
  NSRectFill(self.bounds);
  NSDictionary* style = @{
    NSFontAttributeName: [NSFont monospacedDigitSystemFontOfSize:10 weight:NSFontWeightRegular],
    NSForegroundColorAttributeName: DimMuted()
  };
  if (self.samples.count == 0 || self.series >= self.columns.count) {
    [@"Select a completed run to inspect its trajectory." drawAtPoint:NSMakePoint(15, 20)
                                                       withAttributes:style];
    return;
  }
  double low = std::numeric_limits<double>::infinity();
  double high = -low;
  double start = [self.samples.firstObject.firstObject doubleValue];
  double end = [self.samples.lastObject.firstObject doubleValue];
  for (NSArray* sample in self.samples) {
    double value = [sample[self.series] doubleValue];
    low = fmin(low, value);
    high = fmax(high, value);
  }
  // Scale before subtraction: finite binary64 endpoints may have an infinite
  // difference, and adding 0.5 does not separate large equal endpoints.
  double scale = fmax(1.0, fmax(fabs(low), fabs(high)));
  double scaledLow = low / scale;
  double scaledHigh = high / scale;
  BOOL flat = low == high || scaledLow == scaledHigh;
  if (start == end)
    end += 1;
  NSRect graph = NSInsetRect(self.bounds, 55, 30);
  [DimBorder() setStroke];
  [NSBezierPath strokeRect:graph];
  NSBezierPath* line = [NSBezierPath bezierPath];
  BOOL first = YES;
  for (NSArray* sample in self.samples) {
    double time = [sample[0] doubleValue];
    double value = [sample[self.series] doubleValue];
    double height = flat ? 0.5 : (value / scale - scaledLow) / (scaledHigh - scaledLow);
    NSPoint p = NSMakePoint(graph.origin.x + (time - start) / (end - start) * graph.size.width,
                            graph.origin.y + height * graph.size.height);
    if (first)
      [line moveToPoint:p];
    else
      [line lineToPoint:p];
    first = NO;
  }
  [DimAccent() setStroke];
  line.lineWidth = 1.6;
  [line stroke];
  [[NSString stringWithFormat:@"%.4g", low] drawAtPoint:NSMakePoint(4, graph.origin.y)
                                         withAttributes:style];
  [[NSString stringWithFormat:@"%.4g", high] drawAtPoint:NSMakePoint(4, NSMaxY(graph) - 10)
                                          withAttributes:style];
  [[NSString stringWithFormat:@"%.4g s", start] drawAtPoint:NSMakePoint(graph.origin.x, 10)
                                             withAttributes:style];
  [[NSString stringWithFormat:@"%.4g s", end] drawAtPoint:NSMakePoint(NSMaxX(graph) - 55, 10)
                                           withAttributes:style];
}

@end

@implementation GalataController

- (BOOL)application:(NSApplication*)application openFile:(NSString*)filename {
  NSString* path = filename.stringByStandardizingPath;
  BOOL directory = NO;
  NSString* extension = path.pathExtension.lowercaseString;
  if ((![extension isEqual:@"galata"] && ![extension isEqual:@"galata-review"])
      || ![[NSFileManager defaultManager] fileExistsAtPath:path isDirectory:&directory]
      || !directory) {
    if (self.finishedLaunching)
      [self problem:@"Choose a Galata project package ending in .galata or .galata-review."];
    return NO;
  }
  // Launch Services can deliver the initial document before the editor and
  // engine path exist. Retain one request until launch setup is complete.
  if (!self.finishedLaunching) {
    if (self.pendingOpenProject && ![self.pendingOpenProject isEqual:path])
      return NO;
    self.pendingOpenProject = path;
    return YES;
  }
  [self.window makeKeyAndOrderFront:nil];
  [application activateIgnoringOtherApps:YES];
  // Opening the displayed package again focuses its working draft. It must not
  // reload over unsaved edits or interrupt that project's current command.
  if ([self.project.stringByStandardizingPath.stringByResolvingSymlinksInPath
          isEqual:path.stringByResolvingSymlinksInPath])
    return YES;
  if (![self canReplaceProject])
    return NO;
  [self loadProject:path];
  return YES;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
  (void)notification;
  NSApp.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
  self.edits = [[NSUndoManager alloc] init];
  self.edits.levelsOfUndo = 50;
  self.edits.groupsByEvent = NO;
  NSArray* args = [NSProcessInfo processInfo].arguments;
  NSUInteger engineArg = [args indexOfObject:@"--engine"];
  self.engine = engineArg != NSNotFound && engineArg + 1 < args.count
                    ? args[engineArg + 1]
                    : [[[NSBundle mainBundle] executablePath].stringByDeletingLastPathComponent
                          stringByAppendingPathComponent:@"galata"];
  [self buildMenu];
  self.window = [[NSWindow alloc]
      initWithContentRect:NSMakeRect(0, 0, 1220, 820)
                styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                          | NSWindowStyleMaskMiniaturizable | NSWindowStyleMaskResizable
                  backing:NSBackingStoreBuffered
                    defer:NO];
  DimWindow(self.window);
  self.window.contentMinSize = NSMakeSize(1120, 820);
  self.window.title = @"Galata Preview";
  self.window.delegate = self;
  NSView* content = self.window.contentView;
  NSView* toolbar = [[NSView alloc] initWithFrame:NSMakeRect(0, 770, 1220, 50)];
  toolbar.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
  DimPanel(toolbar, DimSurface());
  [content addSubview:toolbar];
  [toolbar addSubview:Button(@"New… ⌘N", NSMakeRect(12, 10, 90, 30), self, @selector(newProject:))];
  [toolbar
      addSubview:Button(@"Open… ⌘O", NSMakeRect(108, 10, 90, 30), self, @selector(openProject:))];
  self.importButton =
      Button(@"Import Study…", NSMakeRect(204, 10, 140, 30), self, @selector(importStudy:));
  [toolbar addSubview:self.importButton];
  self.saveButton = Button(@"Save ⌘S", NSMakeRect(350, 10, 100, 30), self, @selector(saveProject:));
  [toolbar addSubview:self.saveButton];
  self.runButton =
      Button(@"Run Saved ⌘R", NSMakeRect(456, 10, 138, 30), self, @selector(runProject:));
  DimPrimaryButton(self.runButton);
  [toolbar addSubview:self.runButton];
  self.cancelButton = Button(@"Cancel", NSMakeRect(600, 10, 100, 30), self, @selector(cancelRun:));
  [toolbar addSubview:self.cancelButton];
  self.originButton =
      Button(@"Original Study…", NSMakeRect(706, 10, 150, 30), self, @selector(showOriginalStudy:));
  [toolbar addSubview:self.originButton];
  [toolbar addSubview:Button(@"Saved Revisions…",
                             NSMakeRect(862, 10, 156, 30),
                             self,
                             @selector(showRevisions:))];
  NSTextField* themeLabel = Label(@"DIM · PREVIEW", NSMakeRect(1030, 15, 170, 20));
  themeLabel.textColor = DimMuted();
  [toolbar addSubview:themeLabel];
  NSView* inspector = [[NSView alloc] initWithFrame:NSMakeRect(870, 42, 350, 726)];
  inspector.autoresizingMask = NSViewMinXMargin | NSViewHeightSizable;
  DimPanel(inspector, DimSurface(), true);
  [content addSubview:inspector];
  NSTextField* propertiesTitle = Label(@"BLOCK PROPERTIES — JSON", NSMakeRect(12, 695, 330, 20));
  propertiesTitle.autoresizingMask = NSViewMinYMargin;
  [inspector addSubview:propertiesTitle];
  self.selectionLabel = Label(@"Select a block on the canvas", NSMakeRect(12, 670, 330, 20));
  self.selectionLabel.autoresizingMask = NSViewMinYMargin;
  [inspector addSubview:self.selectionLabel];
  self.properties = TextEditor(inspector, NSMakeRect(12, 390, 325, 275), YES);
  self.properties.enclosingScrollView.autoresizingMask = NSViewHeightSizable;
  self.properties.delegate = self;
  self.properties.accessibilityLabel = @"Block properties JSON";
  [inspector addSubview:Button(@"Apply Properties",
                               NSMakeRect(10, 351, 158, 30),
                               self,
                               @selector(applyProperties:))];
  [inspector addSubview:Button(@"Delete Block",
                               NSMakeRect(183, 351, 155, 30),
                               self,
                               @selector(deleteBlock:))];
  [inspector addSubview:Label(@"SI dimensions: m, kg, s, A, K, mol, cd, rad",
                              NSMakeRect(12, 326, 330, 20))];
  [inspector addSubview:Label(@"SIMULATION — JSON", NSMakeRect(12, 292, 325, 20))];
  self.simulationEditor = TextEditor(inspector, NSMakeRect(12, 120, 325, 167), YES);
  self.simulationEditor.delegate = self;
  self.simulationEditor.accessibilityLabel = @"Simulation settings JSON";
  [inspector addSubview:Button(@"Apply Simulation",
                               NSMakeRect(10, 82, 180, 30),
                               self,
                               @selector(applySimulation:))];
  NSTextField* note = [NSTextField
      wrappingLabelWithString:
          @"Save validates the draft through the shared engine. Run compiles the saved model; "
          @"connect every required input and specify explicit dimensions and frames."];
  note.frame = NSMakeRect(12, 4, 320, 70);
  note.font = [NSFont systemFontOfSize:11];
  note.textColor = DimMuted();
  [inspector addSubview:note];
  NSView* work = [[NSView alloc] initWithFrame:NSMakeRect(12, 42, 846, 726)];
  self.workspace = work;
  work.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
  [content addSubview:work];
  NSView* paletteBar = [[NSView alloc] initWithFrame:NSMakeRect(0, 680, 846, 46)];
  paletteBar.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
  [work addSubview:paletteBar];
  NSView* navigation = [[NSView alloc] initWithFrame:NSMakeRect(0, 646, 846, 32)];
  navigation.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
  [work addSubview:navigation];
  [navigation
      addSubview:Button(@"Fit All", NSMakeRect(0, 1, 78, 28), self, @selector(fitDiagram:))];
  [navigation addSubview:Button(@"−", NSMakeRect(84, 1, 34, 28), self, @selector(zoomOut:))];
  self.zoomButton =
      Button(@"100%", NSMakeRect(121, 1, 66, 28), self, @selector(actualDiagramSize:));
  self.zoomButton.toolTip = @"Current zoom. Click to restore 100% detail.";
  self.zoomButton.accessibilityLabel = @"Diagram zoom; reset to 100 percent";
  [navigation addSubview:self.zoomButton];
  [navigation addSubview:Button(@"+", NSMakeRect(190, 1, 34, 28), self, @selector(zoomIn:))];
  [navigation
      addSubview:Button(@"Arrange", NSMakeRect(235, 1, 85, 28), self, @selector(arrangeDiagram:))];
  self.focusDiagramButton =
      Button(@"Focus Diagram", NSMakeRect(328, 1, 140, 28), self, @selector(focusDiagram:));
  [navigation addSubview:self.focusDiagramButton];
  self.snapEnabled = YES;
  self.snapButton = Button(@"Snap On", NSMakeRect(480, 1, 88, 28), self, @selector(toggleSnap:));
  self.snapButton.toolTip =
      @"Snap to the 16-point grid and block alignment guides. Hold Option to move freely.";
  [navigation addSubview:self.snapButton];
  self.resetRouteButton =
      Button(@"Reset Route", NSMakeRect(574, 1, 106, 28), self, @selector(resetRoute:));
  self.resetRouteButton.toolTip =
      @"Return the selected wire to automatic routing. Command-Z restores its custom shape.";
  [navigation addSubview:self.resetRouteButton];
  self.diagramHint = Label(@"Drag segments to shape wires", NSMakeRect(688, 6, 152, 20));
  self.diagramHint.font = [NSFont systemFontOfSize:11];
  self.diagramHint.textColor = DimMuted();
  self.diagramHint.lineBreakMode = NSLineBreakByTruncatingTail;
  self.diagramHint.autoresizingMask = NSViewWidthSizable;
  [navigation addSubview:self.diagramHint];
  self.palette = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(0, 10, 150, 28) pullsDown:NO];
  [self.palette addItemsWithTitles:@[@"constant", @"gain", @"sum", @"integrator", @"output"]];
  self.palette.accessibilityLabel = @"Block kind";
  [paletteBar addSubview:self.palette];
  [paletteBar
      addSubview:Button(@"Add Block", NSMakeRect(158, 9, 115, 30), self, @selector(addBlock:))];
  self.modelMode = [NSSegmentedControl segmentedControlWithLabels:@[@"Diagram", @"Block List"]
                                                     trackingMode:NSSegmentSwitchTrackingSelectOne
                                                           target:self
                                                           action:@selector(changeModelMode:)];
  self.modelMode.frame = NSMakeRect(285, 10, 185, 28);
  self.modelMode.selectedSegment = 0;
  self.modelMode.selectedSegmentBezelColor = DimRaised();
  self.modelMode.accessibilityLabel = @"Model view";
  [paletteBar addSubview:self.modelMode];
  self.originContext = Label(@"Arrows to select · ⌘Z undo", NSMakeRect(482, 15, 350, 20));
  self.originContext.lineBreakMode = NSLineBreakByTruncatingTail;
  self.originContext.autoresizingMask = NSViewWidthSizable;
  [paletteBar addSubview:self.originContext];
  NSScrollView* canvasScroll = [[NSScrollView alloc] initWithFrame:NSMakeRect(0, 382, 846, 260)];
  self.canvasScroll = canvasScroll;
  canvasScroll.backgroundColor = DimBackground();
  DimPanel(canvasScroll, DimBackground(), true);
  canvasScroll.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
  canvasScroll.hasVerticalScroller = YES;
  canvasScroll.hasHorizontalScroller = YES;
  canvasScroll.allowsMagnification = YES;
  // Fit the complete supported coordinate range, even in the smallest workspace.
  canvasScroll.minMagnification = 0.0001;
  canvasScroll.maxMagnification = 2.5;
  self.diagramFitMode = YES;
  [[NSNotificationCenter defaultCenter] addObserver:self
                                           selector:@selector(diagramMagnified:)
                                               name:NSScrollViewDidEndLiveMagnifyNotification
                                             object:canvasScroll];
  canvasScroll.borderType = NSBezelBorder;
  self.diagram = [[DiagramView alloc] initWithFrame:NSMakeRect(0, 0, 1600, 900)];
  self.diagram.controller = self;
  self.diagram.accessibilityLabel =
      @"Block diagram; click wires to edit, drag ports to connect, "
      @"drag blocks to arrange. Delete removes the selected wire or block.";
  canvasScroll.documentView = self.diagram;
  [work addSubview:canvasScroll];
  self.blockList = [[GalataBlockList alloc] initWithFrame:canvasScroll.frame];
  self.blockList.autoresizingMask = canvasScroll.autoresizingMask;
  self.blockList.hidden = YES;
  __weak GalataController* weakController = self;
  self.blockList.selectionHandler = ^(NSString* identifier) {
    GalataController* controller = weakController;
    if (!controller || controller.task)
      return;
    [controller selectBlock:identifier];
    controller.blockList.selectedID = controller.selectedID;
    if ([controller selectedBlock])
      [controller.diagram scrollRectToVisible:[controller rectForBlock:[controller selectedBlock]]];
  };
  [work addSubview:self.blockList];
  NSView* connectionBar = [[NSView alloc] initWithFrame:NSMakeRect(0, 304, 846, 74)];
  self.connectionBar = connectionBar;
  connectionBar.autoresizingMask = NSViewWidthSizable;
  [work addSubview:connectionBar];
  self.source = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(0, 42, 175, 28) pullsDown:NO];
  self.source.accessibilityLabel = @"Connection source block";
  [connectionBar addSubview:self.source];
  [connectionBar addSubview:Label(@"→", NSMakeRect(181, 47, 20, 20))];
  self.target = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(203, 42, 175, 28) pullsDown:NO];
  self.target.target = self;
  self.target.action = @selector(selectTarget:);
  self.target.accessibilityLabel = @"Connection target block";
  [connectionBar addSubview:self.target];
  [connectionBar addSubview:Label(@"input", NSMakeRect(384, 47, 40, 20))];
  self.input = [[NSTextField alloc] initWithFrame:NSMakeRect(426, 45, 45, 24)];
  self.input.stringValue = @"0";
  self.input.backgroundColor = DimBackground();
  self.input.textColor = DimText();
  self.input.accessibilityLabel = @"Zero-based target input index";
  [connectionBar addSubview:self.input];
  self.connectionButton =
      Button(@"Create Wire", NSMakeRect(480, 41, 116, 30), self, @selector(connect:));
  self.connectionButton.toolTip =
      @"Create a new wire or update the selected wire using these endpoints.";
  [connectionBar addSubview:self.connectionButton];
  self.createWireModeButton =
      Button(@"New Wire", NSMakeRect(602, 41, 112, 30), self, @selector(newWire:));
  self.createWireModeButton.toolTip =
      @"Leave wire editing and create another connection. You can also drag ports.";
  [connectionBar addSubview:self.createWireModeButton];
  self.wires = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(0, 6, 469, 28) pullsDown:NO];
  self.wires.accessibilityLabel = @"Existing connections";
  self.wires.target = self;
  self.wires.action = @selector(selectWire:);
  [connectionBar addSubview:self.wires];
  self.removeWireButton =
      Button(@"Remove Wire", NSMakeRect(480, 4, 116, 30), self, @selector(disconnect:));
  self.removeWireButton.toolTip = @"Remove only the selected wire. Command-Z restores it.";
  [connectionBar addSubview:self.removeWireButton];
  NSTextField* wireHelp = Label(@"Click wire · Drag ports", NSMakeRect(606, 10, 180, 18));
  wireHelp.font = [NSFont systemFontOfSize:10];
  wireHelp.textColor = DimMuted();
  wireHelp.autoresizingMask = NSViewWidthSizable;
  wireHelp.lineBreakMode = NSLineBreakByTruncatingTail;
  [connectionBar addSubview:wireHelp];
  self.history = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(0, 271, 570, 28) pullsDown:NO];
  self.history.target = self;
  self.history.action = @selector(selectRun:);
  self.history.accessibilityLabel = @"Immutable project run history";
  [work addSubview:self.history];
  self.series = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(588, 271, 252, 28) pullsDown:NO];
  self.series.target = self;
  self.series.action = @selector(selectSeries:);
  self.series.accessibilityLabel = @"Trajectory column";
  [work addSubview:self.series];
  NSView* resultPane = [[NSView alloc] initWithFrame:NSMakeRect(0, 4, 417, 259)];
  self.resultPane = resultPane;
  resultPane.autoresizingMask = NSViewWidthSizable;
  DimPanel(resultPane, DimSurface(), true);
  [work addSubview:resultPane];
  self.resultMode = [NSSegmentedControl segmentedControlWithLabels:@[@"Plot", @"Samples"]
                                                      trackingMode:NSSegmentSwitchTrackingSelectOne
                                                            target:self
                                                            action:@selector(changeResultMode:)];
  self.resultMode.frame = NSMakeRect(8, 226, 172, 28);
  self.resultMode.selectedSegment = 0;
  self.resultMode.selectedSegmentBezelColor = DimRaised();
  self.resultMode.accessibilityLabel = @"Result view";
  [resultPane addSubview:self.resultMode];
  self.sampleSummary = Label(@"No samples", NSMakeRect(190, 232, 218, 18));
  self.sampleSummary.textColor = DimMuted();
  self.sampleSummary.autoresizingMask = NSViewWidthSizable;
  self.sampleSummary.lineBreakMode = NSLineBreakByTruncatingTail;
  [resultPane addSubview:self.sampleSummary];
  self.plot = [[TrajectoryView alloc] initWithFrame:NSMakeRect(0, 0, 417, 222)];
  self.plot.autoresizingMask = NSViewWidthSizable;
  [resultPane addSubview:self.plot];
  self.samplesTable = [[GalataTrajectoryTable alloc] initWithFrame:self.plot.frame];
  self.samplesTable.autoresizingMask = NSViewWidthSizable;
  self.samplesTable.hidden = YES;
  [resultPane addSubview:self.samplesTable];
  self.evidence = TextEditor(work, NSMakeRect(425, 4, 418, 259), NO);
  self.evidence.enclosingScrollView.autoresizingMask = NSViewMinXMargin;
  self.evidence.accessibilityLabel = @"Run evidence and diagnostics";
  self.status = Label(@"Create or open a project. The engine validates drafts and compiles models "
                      @"before execution.",
                      NSMakeRect(14, 10, 1190, 22));
  self.status.autoresizingMask = NSViewWidthSizable;
  self.status.lineBreakMode = NSLineBreakByTruncatingMiddle;
  [content addSubview:self.status];
  [self layoutWorkspace];
  [self refresh];
  [self.window center];
  [self.window makeKeyAndOrderFront:nil];
  [NSApp activateIgnoringOtherApps:YES];
  self.finishedLaunching = YES;
  NSUInteger projectArg = [args indexOfObject:@"--project"];
  if (self.pendingOpenProject) {
    NSString* path = self.pendingOpenProject;
    self.pendingOpenProject = nil;
    [self loadProject:path];
  } else if (projectArg != NSNotFound && projectArg + 1 < args.count)
    [self loadProject:args[projectArg + 1]];
}

- (void)layoutWorkspace {
  if (!self.workspace || !self.resultPane)
    return;
  const CGFloat width = self.workspace.bounds.size.width;
  const CGFloat height = self.workspace.bounds.size.height;
  // The palette occupies the top 46 points; navigation below it occupies 34.
  // Keep the wiring controls available when the result section is collapsed.
  const CGFloat canvasBottom = self.diagramFocused ? 82 : 382;
  const CGFloat canvasTop = height - 84;
  self.canvasScroll.frame = NSMakeRect(0, canvasBottom, width, fmax(1, canvasTop - canvasBottom));
  self.blockList.frame = self.canvasScroll.frame;
  self.connectionBar.frame = NSMakeRect(0, self.diagramFocused ? 4 : 304, width, 74);

  const CGFloat seriesWidth = fmin(300, fmax(220, floor(width * 0.30)));
  self.history.frame = NSMakeRect(0, 271, width - seriesWidth - 12, 28);
  self.series.frame = NSMakeRect(width - seriesWidth, 271, seriesWidth, 28);
  const CGFloat plotWidth = floor((width - 8) / 2);
  self.resultPane.frame = NSMakeRect(0, 4, plotWidth, 259);
  self.evidence.enclosingScrollView.frame =
      NSMakeRect(plotWidth + 8, 4, width - plotWidth - 8, 259);
  self.plot.frame = NSMakeRect(0, 0, plotWidth, 222);
  self.samplesTable.frame = self.plot.frame;
  self.sampleSummary.frame = NSMakeRect(190, 232, fmax(1, plotWidth - 198), 18);
  self.history.hidden = self.diagramFocused;
  self.series.hidden = self.diagramFocused;
  self.resultPane.hidden = self.diagramFocused;
  self.evidence.enclosingScrollView.hidden = self.diagramFocused;
  self.focusDiagramButton.title = self.diagramFocused ? @"Show Results" : @"Focus Diagram";
  self.focusDiagramButton.toolTip =
      self.diagramFocused ? @"Show the run history, trajectory and evidence again."
                          : @"Expand the diagram while keeping wiring controls available.";
}

- (void)windowDidResize:(NSNotification*)notification {
  if (notification.object == self.window) {
    [self layoutWorkspace];
    if (self.diagramFitMode)
      [self fitDiagram:nil];
  }
}

- (void)focusDiagram:(id)sender {
  (void)sender;
  self.diagramFocused = !self.diagramFocused;
  [self layoutWorkspace];
  if (self.diagramFocused) {
    self.modelMode.selectedSegment = 0;
    [self changeModelMode:self];
  }
  if (self.diagramFitMode)
    [self fitDiagram:nil];
}

- (void)updateDiagramZoom {
  CGFloat percent = self.canvasScroll.magnification * 100;
  self.zoomButton.title = [NSString stringWithFormat:percent < 1 ? @"%.2f%%" : @"%.0f%%", percent];
  // AppKit may reuse the prior drawing when magnifying. Redraw screen-sized
  // handles and grid dots so their visible sizes match the gesture hit regions.
  self.diagram.needsDisplay = YES;
}

- (void)diagramMagnified:(NSNotification*)notification {
  (void)notification;
  self.diagramFitMode = NO;
  [self updateDiagramZoom];
}

- (void)fitDiagram:(id)sender {
  (void)sender;
  if (!self.diagram || !self.canvasScroll)
    return;
  self.diagramFitMode = YES;
  [self.canvasScroll magnifyToFitRect:[self.diagram contentBounds]];
  [self updateDiagramZoom];
}

- (void)setDiagramZoom:(CGFloat)scale {
  self.diagramFitMode = NO;
  NSRect visible = self.diagram.visibleRect;
  NSPoint center = [self selectedBlock]
                       ? NSMakePoint(NSMidX([self rectForBlock:[self selectedBlock]]),
                                     NSMidY([self rectForBlock:[self selectedBlock]]))
                       : NSMakePoint(NSMidX(visible), NSMidY(visible));
  [self.canvasScroll setMagnification:fmax(self.canvasScroll.minMagnification,
                                           fmin(self.canvasScroll.maxMagnification, scale))
                      centeredAtPoint:center];
  if ([self selectedBlock])
    [self.diagram
        scrollRectToVisible:NSInsetRect([self rectForBlock:[self selectedBlock]], -40, -30)];
  [self updateDiagramZoom];
}

- (void)zoomIn:(id)sender {
  (void)sender;
  [self setDiagramZoom:self.canvasScroll.magnification * 1.25];
}

- (void)zoomOut:(id)sender {
  (void)sender;
  [self setDiagramZoom:self.canvasScroll.magnification / 1.25];
}

- (void)actualDiagramSize:(id)sender {
  (void)sender;
  [self setDiagramZoom:1];
}

- (void)selectWire:(id)sender {
  (void)sender;
  NSDictionary* wire = self.wires.selectedItem.representedObject;
  if (!wire) {
    [self newWire:self];
    return;
  }
  [self chooseWire:wire];
}

- (void)chooseWire:(NSDictionary*)wire {
  if (!wire || self.task || ![self commitProperties] || ![[self connections] containsObject:wire])
    return;
  [self selectBlock:wire[@"target"]];
  if (![self.selectedID isEqual:wire[@"target"]])
    return;
  self.highlightedWire = wire;
  [self refresh];
  [self.source selectItemWithTitle:wire[@"source"]];
  [self.target selectItemWithTitle:wire[@"target"]];
  [self selectTarget:self];
  self.input.stringValue = [wire[@"input"] description];
  [self showDiagram:nil];
  self.diagram.needsDisplay = YES;
  self.status.stringValue =
      [NSString stringWithFormat:@"Editing %@ · Round handles shape the line; square handles "
                                 @"reconnect endpoints. Delete removes this wire.",
                                 WireName(wire, [[self connections] indexOfObject:wire])];
}

- (void)newWire:(id)sender {
  (void)sender;
  if (self.task || ![self commitProperties])
    return;
  [self selectBlock:nil];
  [self.wires selectItemAtIndex:0];
  [self updateConnectionControls];
  self.status.stringValue =
      @"New wire · Drag an output port to an input, or choose endpoints and Create Wire.";
}

- (void)updateConnectionControls {
  BOOL selected = self.highlightedWire && [[self connections] containsObject:self.highlightedWire];
  self.connectionButton.title = selected ? @"Update Wire" : @"Create Wire";
  self.connectionButton.enabled = self.draft && !self.task;
  self.createWireModeButton.enabled = self.draft && !self.task;
  self.removeWireButton.enabled = selected && !self.task;
  self.resetRouteButton.enabled =
      selected && !self.task && [self manualRouteForWire:self.highlightedWire];
  self.source.enabled = self.draft && !self.task;
  self.target.enabled = self.draft && !self.task;
  self.input.enabled = self.draft && !self.task;
  self.wires.enabled = self.draft && !self.task;
  if (!selected && self.wires.numberOfItems)
    [self.wires selectItemAtIndex:0];
}

- (void)toggleSnap:(id)sender {
  (void)sender;
  self.snapEnabled = !self.snapEnabled;
  self.snapButton.title = self.snapEnabled ? @"Snap On" : @"Snap Off";
  self.diagram.needsDisplay = YES;
  self.status.stringValue =
      self.snapEnabled
          ? @"Snap enabled: 16-point grid and block alignment. Hold Option to move freely."
          : @"Snap disabled. Blocks and wire segments move freely.";
}

- (NSArray<NSValue*>*)logicalBlockRects {
  NSMutableArray* rects = [NSMutableArray array];
  for (NSDictionary* block in [self blocks])
    [rects addObject:[NSValue valueWithRect:[self modelRectForBlock:block]]];
  return rects;
}

- (NSDictionary*)manualRouteForWire:(NSDictionary*)wire {
  for (NSDictionary* route in self.draft[@"presentation"][@"routes"])
    if (SameWire(route, wire))
      return route;
  return nil;
}

- (NSArray<NSValue*>*)validManualPoints:(NSDictionary*)route {
  NSArray<NSValue*>* points = StoredRoutePoints(route);
  if (points.count < 2 || !GalataWirePathClear(points, [self logicalBlockRects]))
    return nil;
  NSDictionary* source = nil;
  NSDictionary* target = nil;
  for (NSDictionary* block in [self blocks]) {
    if ([block[@"id"] isEqual:route[@"source"]])
      source = block;
    if ([block[@"id"] isEqual:route[@"target"]])
      target = block;
  }
  NSUInteger slot = [route[@"input"] unsignedIntegerValue];
  if (!source || !target || slot >= MIN((NSUInteger)64, BlockInputCount(target)))
    return nil;
  NSRect from = [self modelRectForBlock:source], to = [self modelRectForBlock:target];
  NSPoint start = NSMakePoint(NSMaxX(from), NSMidY(from));
  NSPoint end = BlockInputPoint(to, BlockInputCount(target), slot);
  // Adding/removing a display offset may round fractional font-derived widths.
  // This tolerance is far below a screen pixel and only checks UI attachment.
  if (hypot(points.firstObject.pointValue.x - start.x, points.firstObject.pointValue.y - start.y)
          > 1e-7
      || hypot(points.lastObject.pointValue.x - end.x, points.lastObject.pointValue.y - end.y)
             > 1e-7)
    return nil;
  return points;
}

- (void)removeManualRouteForWire:(NSDictionary*)wire {
  NSMutableArray* kept = [NSMutableArray array];
  for (NSDictionary* route in self.draft[@"presentation"][@"routes"])
    if (!SameWire(route, wire))
      [kept addObject:route];
  if (kept.count)
    self.draft[@"presentation"][@"routes"] = kept;
  else if (self.draft[@"presentation"][@"routes"]) {
    [self.draft[@"presentation"] removeObjectForKey:@"routes"];
    self.draft[@"presentation"][@"schema"] = @"galata.presentation.v1";
  }
}

- (void)saveRoute:(NSArray<NSValue*>*)points forWire:(NSDictionary*)wire {
  if (self.task || !self.draft || ![[self connections] containsObject:wire]
      || !GalataWirePathClear(points, [self logicalBlockRects]))
    return;
  if ([points isEqual:StoredRoutePoints([self manualRouteForWire:wire])])
    return;
  if (![self manualRouteForWire:wire] && [self.draft[@"presentation"][@"routes"] count] >= 256) {
    [self problem:@"At most 256 wires can have custom routes. Reset another route first."];
    return;
  }
  NSMutableArray* encoded = [NSMutableArray array];
  for (NSValue* value in points) {
    NSPoint point = value.pointValue;
    [encoded addObject:[@{@"x": @(point.x), @"y": @(point.y)} mutableCopy]];
  }
  [self checkpoint:@"Shape wire"];
  [self removeManualRouteForWire:wire];
  self.draft[@"presentation"][@"schema"] = @"galata.presentation.v2";
  if (!self.draft[@"presentation"][@"routes"])
    self.draft[@"presentation"][@"routes"] = [NSMutableArray array];
  [self.draft[@"presentation"][@"routes"] addObject:[@{
                                            @"source": wire[@"source"],
                                            @"target": wire[@"target"],
                                            @"input": wire[@"input"],
                                            @"points": encoded
                                          } mutableCopy]];
  [self refresh];
  self.status.stringValue =
      @"Wire shape changed. Endpoints are unchanged. Command-Z undoes it; Save keeps the route.";
}

- (void)resetRoute:(id)sender {
  (void)sender;
  if (self.task || ![self commitProperties] || ![self manualRouteForWire:self.highlightedWire])
    return;
  [self.diagram cancelWiring];
  [self checkpoint:@"Reset wire route"];
  [self removeManualRouteForWire:self.highlightedWire];
  [self refresh];
  self.status.stringValue = @"Automatic routing restored. Command-Z brings back the custom shape.";
}

- (void)moveBlock:(NSString*)identifier to:(NSPoint)point {
  if (self.task || !CanvasPositionSupported(point))
    return;
  NSDictionary* block = nil;
  for (NSDictionary* candidate in [self blocks])
    if ([candidate[@"id"] isEqual:identifier])
      block = candidate;
  if (!block || NSEqualPoints([self modelRectForBlock:block].origin, point))
    return;
  [self checkpoint:@"Move block"];
  self.draft[@"presentation"][@"positions"][identifier] =
      [@{@"x": @(point.x), @"y": @(point.y)} mutableCopy];
  for (NSDictionary* wire in [self connections])
    if ([wire[@"source"] isEqual:identifier] || [wire[@"target"] isEqual:identifier])
      [self removeManualRouteForWire:wire];
  [self refresh];
  self.status.stringValue = @"Block placed. Attached wires rerouted. Command-Z undoes the move and "
                            @"restores custom routes.";
}

- (void)arrangeDiagram:(id)sender {
  (void)sender;
  if (self.task || !self.draft || ![self commitProperties])
    return;
  // Keep existing column and row order, but expand it for actual card sizes.
  // This is an explicit, undoable presentation edit, independent of model semantics.
  NSArray* ordered = [[self blocks]
      sortedArrayUsingComparator:^NSComparisonResult(NSDictionary* a, NSDictionary* b) {
        NSRect ar = [self modelRectForBlock:a], br = [self modelRectForBlock:b];
        if (ar.origin.x != br.origin.x)
          return ar.origin.x < br.origin.x ? NSOrderedAscending : NSOrderedDescending;
        if (ar.origin.y != br.origin.y)
          return ar.origin.y < br.origin.y ? NSOrderedAscending : NSOrderedDescending;
        return [a[@"id"] compare:b[@"id"]];
      }];
  NSMutableArray<NSMutableArray*>* columns = [NSMutableArray array];
  CGFloat columnStart = -std::numeric_limits<CGFloat>::infinity();
  for (NSDictionary* block in ordered) {
    CGFloat x = [self modelRectForBlock:block].origin.x;
    if (!columns.count || x - columnStart > 90) {
      [columns addObject:[NSMutableArray array]];
      columnStart = x;
    }
    [columns.lastObject addObject:block];
  }
  NSMutableDictionary* plannedPositions = [NSMutableDictionary dictionary];
  CGFloat x = 80;
  for (NSArray* column in columns) {
    CGFloat width = 180, y = 80;
    for (NSDictionary* block in column) {
      NSSize size = BlockCardSize(block);
      if (!CanvasPositionSupported(NSMakePoint(x, y))) {
        [self problem:@"This arrangement exceeds the supported canvas size. Move some blocks "
                      @"closer together before arranging."];
        return;
      }
      plannedPositions[block[@"id"]] = [@{@"x": @(x), @"y": @(y)} mutableCopy];
      y += size.height + 56;
      width = fmax(width, size.width);
    }
    x += width + 110;
  }
  [self checkpoint:@"Arrange diagram"];
  if (self.draft[@"presentation"][@"routes"]) {
    [self.draft[@"presentation"] removeObjectForKey:@"routes"];
    self.draft[@"presentation"][@"schema"] = @"galata.presentation.v1";
  }
  [self.draft[@"presentation"][@"positions"] addEntriesFromDictionary:plannedPositions];
  [self refresh];
  [self fitDiagram:nil];
  self.status.stringValue = @"Diagram arranged with automatic wire routes. ⌘Z restores the prior "
                            @"layout and custom routes; Save keeps it.";
}

- (void)buildMenu {
  NSMenu* menu = [[NSMenu alloc] init];
  NSMenuItem* appItem = [[NSMenuItem alloc] init];
  [menu addItem:appItem];
  NSMenu* appMenu = [[NSMenu alloc] initWithTitle:@"Galata Preview"];
  [appMenu addItemWithTitle:@"About Galata Preview"
                     action:@selector(orderFrontStandardAboutPanel:)
              keyEquivalent:@""];
  [appMenu addItem:[NSMenuItem separatorItem]];
  [appMenu addItemWithTitle:@"Quit Galata Preview" action:@selector(terminate:) keyEquivalent:@"q"];
  appItem.submenu = appMenu;
  NSMenuItem* fileItem = [[NSMenuItem alloc] init];
  [menu addItem:fileItem];
  NSMenu* file = [[NSMenu alloc] initWithTitle:@"File"];
  NSArray* titles = @[
    @"New Project…",
    @"Open Project…",
    @"Import Study…",
    @"Run Study…",
    @"Save Project",
    @"Run Saved Project",
    @"Export Review Package…",
    @"Verify Review Package…",
    @"Verify Onboard Manifest…",
    @"Stage Onboard Package…",
    @"Deploy Onboard Runtime…",
    @"Verify Onboard Deployment…",
    @"Create Target Evidence Package…",
    @"Verify Target Evidence…",
    @"Onboard SIL Self-Test…",
    @"Create Flight-Test Package…",
    @"Verify Flight-Test Evidence…",
    @"Run Flight-Test Validation…",
    @"Create Qualification Dossier…",
    @"Verify Qualification Dossier…",
    @"Verify Qualification Chain…",
    @"Saved Revisions…",
    @"Original Study…"
  ];
  NSArray* keys = @[
    @"n", @"o", @"i", @"t", @"s", @"r", @"", @"", @"", @"",  @"", @"",
    @"",  @"",  @"",  @"",  @"",  @"",  @"", @"", @"", @"h", @""
  ];
  NSArray* actions = @[
    @"newProject:",
    @"openProject:",
    @"importStudy:",
    @"runStudy:",
    @"saveProject:",
    @"runProject:",
    @"exportReview:",
    @"verifyReview:",
    @"verifyOnboard:",
    @"stageOnboard:",
    @"deployOnboard:",
    @"verifyOnboardDeployment:",
    @"createOnboardTargetEvidence:",
    @"verifyOnboardTargetEvidence:",
    @"onboardSelfTest:",
    @"createFlightTest:",
    @"verifyFlightTest:",
    @"validateFlightTest:",
    @"createQualification:",
    @"verifyQualification:",
    @"verifyQualificationChain:",
    @"showRevisions:",
    @"showOriginalStudy:"
  ];
  for (NSUInteger i = 0; i < titles.count; ++i) {
    NSMenuItem* item = [file addItemWithTitle:titles[i]
                                       action:NSSelectorFromString(actions[i])
                                keyEquivalent:keys[i]];
    item.target = self;
  }
  fileItem.submenu = file;
  NSMenuItem* editItem = [[NSMenuItem alloc] init];
  [menu addItem:editItem];
  NSMenu* edit = [[NSMenu alloc] initWithTitle:@"Edit"];
  NSMenuItem* undo = [edit addItemWithTitle:@"Undo Model Edit"
                                     action:@selector(undoEdit:)
                              keyEquivalent:@"z"];
  undo.target = self;
  NSMenuItem* redo = [edit addItemWithTitle:@"Redo Model Edit"
                                     action:@selector(redoEdit:)
                              keyEquivalent:@"Z"];
  redo.target = self;
  [edit addItem:[NSMenuItem separatorItem]];
  [edit addItemWithTitle:@"Cut" action:@selector(cut:) keyEquivalent:@"x"];
  [edit addItemWithTitle:@"Copy" action:@selector(copy:) keyEquivalent:@"c"];
  [edit addItemWithTitle:@"Paste" action:@selector(paste:) keyEquivalent:@"v"];
  [edit addItemWithTitle:@"Select All" action:@selector(selectAll:) keyEquivalent:@"a"];
  editItem.submenu = edit;
  NSMenuItem* viewItem = [[NSMenuItem alloc] init];
  [menu addItem:viewItem];
  NSMenu* view = [[NSMenu alloc] initWithTitle:@"View"];
  NSArray* viewTitles = @[@"Diagram", @"Block List", @"Plot", @"Samples", @"Block Properties"];
  NSArray* viewActions =
      @[@"showDiagram:", @"showBlockList:", @"showPlot:", @"showSamples:", @"focusProperties:"];
  for (NSUInteger i = 0; i < viewTitles.count; ++i) {
    NSMenuItem* item =
        [view addItemWithTitle:viewTitles[i]
                        action:NSSelectorFromString(viewActions[i])
                 keyEquivalent:[NSString stringWithFormat:@"%lu", (unsigned long)i + 1]];
    item.target = self;
  }
  viewItem.submenu = view;
  NSApp.mainMenu = menu;
}

- (void)changeModelMode:(id)sender {
  (void)sender;
  BOOL list = self.modelMode.selectedSegment == 1;
  self.blockList.hidden = !list;
  self.canvasScroll.hidden = list;
  [self.window makeFirstResponder:list ? self.blockList.tableView : self.diagram];
}

- (void)changeResultMode:(id)sender {
  (void)sender;
  if (self.diagramFocused) {
    self.diagramFocused = NO;
    [self layoutWorkspace];
    if (self.diagramFitMode)
      [self fitDiagram:nil];
  }
  BOOL table = self.resultMode.selectedSegment == 1;
  self.samplesTable.hidden = !table;
  self.plot.hidden = table;
  if (table)
    [self.window makeFirstResponder:self.samplesTable.tableView];
}

- (void)showDiagram:(id)sender {
  self.modelMode.selectedSegment = 0;
  [self changeModelMode:sender];
}

- (void)showBlockList:(id)sender {
  self.modelMode.selectedSegment = 1;
  [self changeModelMode:sender];
}

- (void)showPlot:(id)sender {
  self.resultMode.selectedSegment = 0;
  [self changeResultMode:sender];
}

- (void)showSamples:(id)sender {
  self.resultMode.selectedSegment = 1;
  [self changeResultMode:sender];
}

- (void)focusProperties:(id)sender {
  (void)sender;
  [self.window makeFirstResponder:self.properties];
}

- (NSArray*)blocks {
  return self.draft[@"model"][@"blocks"];
}

- (NSArray*)connections {
  return self.draft[@"model"][@"connections"];
}

- (NSMutableDictionary*)selectedBlock {
  for (NSMutableDictionary* block in [self blocks])
    if ([block[@"id"] isEqual:self.selectedID])
      return block;
  return nil;
}

- (NSString*)originalChannelName:(NSString*)identifier {
  NSDictionary* adapter = self.projectView[@"origin"][@"adapter"];
  NSDictionary* mapping = adapter[@"mapping"];
  NSDictionary* system = adapter[@"source_system"];
  if (![mapping isKindOfClass:[NSDictionary class]] || ![system isKindOfClass:[NSDictionary class]])
    return nil;
  NSArray* idKeys =
      @[@"state_ids", @"command_ids", @"control_ids", @"control_output_ids", @"output_ids"];
  NSArray* nameKeys =
      @[@"state_names", @"input_names", @"input_names", @"input_names", @"output_names"];
  for (NSUInteger i = 0; i < idKeys.count; ++i) {
    NSArray* ids = mapping[idKeys[i]];
    NSArray* names = system[nameKeys[i]];
    if (![ids isKindOfClass:[NSArray class]] || ![names isKindOfClass:[NSArray class]])
      continue;
    NSUInteger slot = [ids indexOfObject:identifier];
    if (slot != NSNotFound && slot < names.count && [names[slot] isKindOfClass:[NSString class]])
      return names[slot];
  }
  return nil;
}

- (NSRect)modelRectForBlock:(NSDictionary*)block {
  NSDictionary* p = self.draft[@"presentation"][@"positions"][block[@"id"]];
  NSUInteger index = [[self blocks] indexOfObject:block];
  double x = p ? [p[@"x"] doubleValue] : 35 + (index % 4) * 195;
  double y = p ? [p[@"y"] doubleValue] : 45 + (index / 4) * 110;
  NSSize size = BlockCardSize(block);
  if (self.diagram.dragging && [block[@"id"] isEqual:self.diagram.draggedID])
    return NSMakeRect(
        self.diagram.blockPreview.x, self.diagram.blockPreview.y, size.width, size.height);
  return NSMakeRect(x, y, size.width, size.height);
}

- (NSRect)rectForBlock:(NSDictionary*)block {
  return NSOffsetRect([self modelRectForBlock:block], self.diagramOffset.x, self.diagramOffset.y);
}

- (void)updateDiagramExtent {
  CGFloat lowX = 80, lowY = 80;
  for (NSDictionary* block in [self blocks]) {
    NSRect rect = [self modelRectForBlock:block];
    lowX = fmin(lowX, NSMinX(rect));
    lowY = fmin(lowY, NSMinY(rect));
  }
  for (NSDictionary* route in self.draft[@"presentation"][@"routes"]) {
    for (NSValue* value in [self validManualPoints:route]) {
      lowX = fmin(lowX, value.pointValue.x);
      lowY = fmin(lowY, value.pointValue.y);
    }
  }
  NSPoint previous = self.diagramOffset;
  self.diagramOffset = NSMakePoint(80 - lowX, 80 - lowY);
  NSRect content = [self.diagram contentBounds];
  NSSize viewport = self.canvasScroll.contentView.bounds.size;
  [self.diagram setFrameSize:NSMakeSize(fmax(viewport.width, NSMaxX(content) + 40),
                                        fmax(viewport.height, NSMaxY(content) + 40))];
  if (!self.diagramFitMode && !NSEqualPoints(previous, self.diagramOffset)) {
    NSPoint origin = self.canvasScroll.contentView.bounds.origin;
    origin.x += self.diagramOffset.x - previous.x;
    origin.y += self.diagramOffset.y - previous.y;
    [self.canvasScroll.contentView scrollToPoint:origin];
    [self.canvasScroll reflectScrolledClipView:self.canvasScroll.contentView];
  }
}

- (void)selectBlock:(NSString*)identifier {
  if (self.propertyDirty && ![self commitProperties])
    return;
  [self.diagram cancelWiring];
  self.selectedID = identifier;
  self.highlightedWire = nil;
  [self updateConnectionControls];
  self.blockList.selectedID = identifier;
  [self populateEditors];
  self.diagram.needsDisplay = YES;
}

- (void)populateEditors {
  self.populating = YES;
  self.properties.string = [self selectedBlock] ? JSONText([self selectedBlock]) : @"";
  [self.properties.undoManager removeAllActions];
  self.selectionLabel.stringValue =
      self.selectedID ? self.selectedID : @"Select a block on the canvas";
  NSString* originalName = self.selectedID ? [self originalChannelName:self.selectedID] : nil;
  if (originalName)
    self.selectionLabel.stringValue =
        [NSString stringWithFormat:@"%@ · ORIGINAL %@", self.selectedID, originalName];
  self.selectionLabel.toolTip = self.selectionLabel.stringValue;
  self.selectionLabel.lineBreakMode = NSLineBreakByTruncatingTail;
  self.propertyDirty = NO;
  if (!self.simulationDirty) {
    self.simulationEditor.string = self.draft ? JSONText(self.draft[@"simulation"]) : @"";
    [self.simulationEditor.undoManager removeAllActions];
  }
  self.populating = NO;
}

- (void)refresh {
  self.saveButton.enabled = self.draft && !self.task;
  self.importButton.enabled = !self.task;
  self.originButton.enabled = [self.projectView[@"origin"] isKindOfClass:[NSDictionary class]];
  self.runButton.enabled =
      self.draft && !self.task && !self.dirty && !self.propertyDirty && !self.simulationDirty;
  self.cancelButton.enabled = self.task
                              && ([self.task.arguments containsObject:@"run"] ||
                                  [self.task.arguments containsObject:@"import-linear"])
                              && !self.cancelling;
  self.properties.editable = self.draft && !self.task;
  self.simulationEditor.editable = self.draft && !self.task;
  self.blockList.enabled = self.draft && !self.task;
  self.revisionChoices.enabled = !self.task;
  self.restoreButton.enabled =
      self.reviewedRevision && !self.task && !self.dirty && !self.propertyDirty
      && !self.simulationDirty && [self.revisionHistory[@"current_status"] isEqual:@"valid"] &&
      [self.project isEqual:self.recoveryProject] && [self.revision isEqual:self.recoveryHead]
      && ![self.reviewedRevision isEqual:self.recoveryHead];
  self.window.documentEdited = self.dirty || self.propertyDirty || self.simulationDirty;
  self.window.title = self.project ? [NSString stringWithFormat:@"%@ — Galata Preview",
                                                                self.project.lastPathComponent]
                                   : @"Galata Preview";
  NSString* selectedKind = self.palette.titleOfSelectedItem;
  BOOL linear = [self.draft[@"model"][@"profile"] isEqual:@"continuous-linear.v1"];
  BOOL hasLinear = [self.palette itemWithTitle:@"linear_combination"] != nil;
  if (linear && !hasLinear)
    [self.palette addItemWithTitle:@"linear_combination"];
  else if (!linear && hasLinear)
    [self.palette removeItemWithTitle:@"linear_combination"];
  if (selectedKind && [self.palette itemWithTitle:selectedKind])
    [self.palette selectItemWithTitle:selectedKind];
  NSDictionary* origin = self.projectView[@"origin"];
  if (origin) {
    self.originContext.stringValue =
        [NSString stringWithFormat:@"ORIGINAL STUDY · Saved graph %@%@",
                                   OriginRelation(origin[@"relation"]),
                                   self.window.documentEdited ? @" · unsaved edits" : @""];
  } else {
    self.originContext.stringValue = @"Drag to arrange · Arrows to select · Delete · ⌘Z undo";
  }
  self.originContext.toolTip = self.originContext.stringValue;
  NSMutableDictionary* names = [NSMutableDictionary dictionary];
  for (NSDictionary* block in [self blocks]) {
    NSString* original = [self originalChannelName:block[@"id"]];
    if (original)
      names[block[@"id"]] = original;
  }
  [self.blockList setBlocks:([self blocks] ? [self blocks] : @[]) originalNames:names];
  self.blockList.selectedID = self.selectedID;
  NSString* oldSource = self.source.titleOfSelectedItem;
  NSString* oldTarget = self.target.titleOfSelectedItem;
  [self.source removeAllItems];
  [self.target removeAllItems];
  for (NSDictionary* block in [self blocks]) {
    [self.source addItemWithTitle:block[@"id"]];
    [self.target addItemWithTitle:block[@"id"]];
  }
  if (oldSource && [self.source itemWithTitle:oldSource])
    [self.source selectItemWithTitle:oldSource];
  if (oldTarget && [self.target itemWithTitle:oldTarget])
    [self.target selectItemWithTitle:oldTarget];
  [self selectTarget:self];
  [self.wires removeAllItems];
  [self.wires addItemWithTitle:@"Select a wire to edit…"];
  NSUInteger wireIndex = 0;
  for (NSDictionary* wire in [self connections]) {
    [self.wires addItemWithTitle:WireName(wire, wireIndex++)];
    self.wires.lastItem.representedObject = wire;
    if ([wire isEqual:self.highlightedWire])
      [self.wires selectItem:self.wires.lastItem];
  }
  if (self.highlightedWire && ![[self connections] containsObject:self.highlightedWire])
    self.highlightedWire = nil;
  [self updateConnectionControls];
  [self updateDiagramExtent];
  if (self.diagramFitMode)
    [self fitDiagram:nil];
  self.diagram.needsDisplay = YES;
}

- (void)selectTarget:(id)sender {
  (void)sender;
  for (NSDictionary* block in [self blocks]) {
    if (![block[@"id"] isEqual:self.target.titleOfSelectedItem])
      continue;
    NSUInteger count = BlockInputCount(block);
    self.input.toolTip =
        count ? [NSString stringWithFormat:@"%@ has %lu input ports, indexed 0 to %lu.",
                                           block[@"id"],
                                           (unsigned long)count,
                                           (unsigned long)(count - 1)]
              : @"This block has no input ports.";
    return;
  }
  self.input.toolTip = @"Select a target block to inspect its input ports.";
}

- (void)problem:(NSString*)message {
  self.status.stringValue = message;
  NSAlert* alert = [[NSAlert alloc] init];
  alert.messageText = @"Galata Preview";
  alert.informativeText = message;
  [alert runModal];
}

- (NSString*)promptText:(NSString*)title
                message:(NSString*)message
            placeholder:(NSString*)placeholder {
  NSAlert* alert = [[NSAlert alloc] init];
  alert.messageText = title;
  alert.informativeText = message;
  NSTextField* field = [[NSTextField alloc] initWithFrame:NSMakeRect(0, 0, 360, 24)];
  field.placeholderString = placeholder;
  field.accessibilityLabel = title;
  alert.accessoryView = field;
  [alert addButtonWithTitle:@"Continue"];
  [alert addButtonWithTitle:@"Cancel"];
  if ([alert runModal] != NSAlertFirstButtonReturn)
    return nil;
  NSString* value = [field.stringValue
      stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
  if (!value.length) {
    [self problem:[NSString stringWithFormat:@"%@ is required.", title]];
    return nil;
  }
  return value;
}

- (BOOL)canReplaceProject {
  if (self.task) {
    [self problem:@"Wait for the active command or cancel the run or study import first."];
    return NO;
  }
  if (!self.dirty && !self.propertyDirty && !self.simulationDirty)
    return YES;
  NSAlert* alert = [[NSAlert alloc] init];
  alert.messageText = @"Discard unsaved changes?";
  alert.informativeText =
      @"The current draft has unsaved edits. Save the project first to keep them.";
  [alert addButtonWithTitle:@"Keep Editing"];
  [alert addButtonWithTitle:@"Discard Changes"];
  return [alert runModal] == NSAlertSecondButtonReturn;
}

- (NSString*)recoverySnapshotPathForProject:(NSString*)project {
  if (!project.length)
    return nil;
  NSArray* directories =
      NSSearchPathForDirectoriesInDomains(NSApplicationSupportDirectory, NSUserDomainMask, YES);
  if (!directories.count)
    return nil;
  NSString* directory = [[directories.firstObject stringByAppendingPathComponent:@"Galata Preview"]
      stringByAppendingPathComponent:@"Recovery"];
  [[NSFileManager defaultManager] createDirectoryAtPath:directory
                            withIntermediateDirectories:YES
                                             attributes:nil
                                                  error:nil];
  NSString* normalized = project.stringByStandardizingPath;
  return [directory
      stringByAppendingPathComponent:[NSString stringWithFormat:@"%@.json", SHA256Hex(normalized)]];
}

- (void)scheduleRecoverySnapshot {
  [self.recoveryTimer invalidate];
  self.recoveryTimer = [NSTimer scheduledTimerWithTimeInterval:0.35
                                                        target:self
                                                      selector:@selector(persistRecoverySnapshot)
                                                      userInfo:nil
                                                       repeats:NO];
}

- (void)persistRecoverySnapshot {
  [self.recoveryTimer invalidate];
  self.recoveryTimer = nil;
  if (!self.project || !self.draft)
    return;
  if (!self.dirty && !self.propertyDirty && !self.simulationDirty) {
    [self discardRecoverySnapshotForProject:self.project];
    return;
  }
  NSString* path = [self recoverySnapshotPathForProject:self.project];
  if (!path)
    return;
  NSMutableDictionary* snapshot = [@{
    @"schema": @"galata.desktop-recovery.v1",
    @"project": self.project.stringByStandardizingPath,
    @"base_revision": self.revision ? self.revision : @"",
    @"updated_at": @([[NSDate date] timeIntervalSince1970]),
    @"draft": JSONCopy(self.draft)
  } mutableCopy];
  if (self.propertyDirty)
    snapshot[@"pending_properties"] = self.properties.string ? self.properties.string : @"";
  if (self.simulationDirty)
    snapshot[@"pending_simulation"] =
        self.simulationEditor.string ? self.simulationEditor.string : @"";
  NSData* data = [NSJSONSerialization dataWithJSONObject:snapshot
                                                 options:NSJSONWritingPrettyPrinted
                                                   error:nil];
  // Keep recovery bounded to one small local draft and never let a malformed or
  // unexpectedly large editor buffer turn recovery into an unbounded cache.
  if (!data || data.length > 8 * 1024 * 1024)
    return;
  [data writeToFile:path options:NSDataWritingAtomic error:nil];
}

- (void)discardRecoverySnapshotForProject:(NSString*)project {
  NSString* path = [self recoverySnapshotPathForProject:project];
  if (path)
    [[NSFileManager defaultManager] removeItemAtPath:path error:nil];
}

- (void)offerRecoverySnapshotForProject:(NSString*)project {
  NSString* path = [self recoverySnapshotPathForProject:project];
  if (!path)
    return;
  NSData* data = [NSData dataWithContentsOfFile:path options:0 error:nil];
  NSDictionary* snapshot =
      data ? [NSJSONSerialization JSONObjectWithData:data
                                             options:NSJSONReadingMutableContainers
                                               error:nil]
           : nil;
  NSString* normalized = project.stringByStandardizingPath;
  NSDictionary* recoveredDraft =
      [snapshot isKindOfClass:[NSDictionary class]] ? snapshot[@"draft"] : nil;
  BOOL valid = [snapshot isKindOfClass:[NSDictionary class]] &&
               [snapshot[@"schema"] isEqual:@"galata.desktop-recovery.v1"] &&
               [snapshot[@"project"] isEqual:normalized] &&
               [snapshot[@"base_revision"] isEqual:self.revision] &&
               [recoveredDraft isKindOfClass:[NSDictionary class]];
  if (!valid) {
    // A snapshot based on another saved revision cannot be merged safely. The
    // saved project remains authoritative, so retire the stale local journal.
    [self discardRecoverySnapshotForProject:project];
    return;
  }
  if ([recoveredDraft isEqual:self.draft] && !snapshot[@"pending_properties"]
      && !snapshot[@"pending_simulation"]) {
    [self discardRecoverySnapshotForProject:project];
    return;
  }
  NSAlert* alert = [[NSAlert alloc] init];
  alert.messageText = @"Recover unsaved draft?";
  alert.informativeText =
      @"Galata found a local recovery snapshot based on the currently saved revision. Recovering "
      @"it restores an unsaved draft; Save will validate it and create the next project revision.";
  [alert addButtonWithTitle:@"Recover Draft"];
  [alert addButtonWithTitle:@"Discard Snapshot"];
  if ([alert runModal] != NSAlertFirstButtonReturn) {
    [self discardRecoverySnapshotForProject:project];
    self.status.stringValue = @"Recovery snapshot discarded. The saved draft is active.";
    return;
  }
  self.draft = JSONCopy(recoveredDraft);
  self.dirty = YES;
  self.propertyDirty = NO;
  self.simulationDirty = NO;
  [self.edits removeAllActions];
  [self populateEditors];
  if ([snapshot[@"pending_properties"] isKindOfClass:[NSString class]]) {
    self.properties.string = snapshot[@"pending_properties"];
    self.propertyDirty = YES;
  }
  if ([snapshot[@"pending_simulation"] isKindOfClass:[NSString class]]) {
    self.simulationEditor.string = snapshot[@"pending_simulation"];
    self.simulationDirty = YES;
  }
  [self refresh];
  self.status.stringValue = @"Recovered unsaved draft. Save to validate and retain it.";
}

- (void)checkpoint:(NSString*)name {
  NSDictionary* previous = JSONCopy(self.draft);
  // Applying pending properties and then deleting/moving a block are separate
  // edits even when one event triggers both. Undo must first recover the block
  // as the user most recently edited it, before undoing its property change.
  [self.edits beginUndoGrouping];
  [self.edits registerUndoWithTarget:self
                             handler:^(GalataController* controller) {
                               [controller restoreDraft:previous];
                             }];
  [self.edits setActionName:name];
  [self.edits endUndoGrouping];
  // Stabilize legacy drafts that omitted positions before array membership changes.
  for (NSDictionary* block in [self blocks]) {
    if (!self.draft[@"presentation"][@"positions"][block[@"id"]]) {
      NSRect rect = [self modelRectForBlock:block];
      self.draft[@"presentation"][@"positions"][block[@"id"]] =
          [@{@"x": @(rect.origin.x), @"y": @(rect.origin.y)} mutableCopy];
    }
  }
  self.dirty = YES;
  [self scheduleRecoverySnapshot];
}

- (void)restoreDraft:(NSDictionary*)previous {
  [self checkpoint:@"Model edit"];
  self.draft = JSONCopy(previous);
  NSString* savedSchema = self.projectView[@"draft_schema"] ? self.projectView[@"draft_schema"]
                                                            : @"galata.project-draft.v1";
  id draftOrigin = self.draft[@"origin_sha256"];
  id savedOrigin = self.projectView[@"origin_sha256"];
  self.dirty = !(self.projectView && [self.draft[@"schema"] isEqual:savedSchema] &&
                 [self.draft[@"model"] isEqual:self.projectView[@"model"]] &&
                 [self.draft[@"presentation"] isEqual:self.projectView[@"presentation"]] &&
                 [self.draft[@"simulation"] isEqual:self.projectView[@"simulation"]]
                 && (draftOrigin == savedOrigin || [draftOrigin isEqual:savedOrigin]));
  if (![self selectedBlock])
    self.selectedID = nil;
  self.simulationDirty = NO;
  [self populateEditors];
  [self refresh];
  self.status.stringValue = self.dirty ? @"Edit restored. Save to validate the draft."
                                       : @"Returned to the saved draft. Run Saved is available.";
  [self scheduleRecoverySnapshot];
}

- (void)undoEdit:(id)sender {
  (void)sender;
  if (!self.task) {
    [self.diagram cancelWiring];
    NSTextView* editor = [self focusedTextEditor];
    if (editor) {
      [editor.undoManager undo];
      return;
    }
    // Pending valid text is the newest edit. Capture it before undoing so an
    // unrelated diagram action cannot silently discard inspector contents.
    if (![self commitProperties] || ![self commitSimulation])
      return;
    [self.edits undo];
    [self refresh];
  }
}

- (void)redoEdit:(id)sender {
  (void)sender;
  if (!self.task) {
    [self.diagram cancelWiring];
    NSTextView* editor = [self focusedTextEditor];
    if (editor) {
      [editor.undoManager redo];
      return;
    }
    if (self.propertyDirty || self.simulationDirty) {
      self.status.stringValue = @"Apply or undo pending JSON text before redoing a model edit.";
      return;
    }
    [self.edits redo];
    [self refresh];
  }
}

- (NSTextView*)focusedTextEditor {
  NSResponder* responder = NSApp.keyWindow.firstResponder;
  if ([responder isKindOfClass:[NSTextView class]] && [(NSTextView*)responder isEditable])
    return (NSTextView*)responder;
  return nil;
}

- (BOOL)validateMenuItem:(NSMenuItem*)item {
  if (item.action == @selector(runStudy:) || item.action == @selector(verifyOnboard:)
      || item.action == @selector(stageOnboard:) || item.action == @selector(deployOnboard:)
      || item.action == @selector(verifyOnboardDeployment:)
      || item.action == @selector(createOnboardTargetEvidence:)
      || item.action == @selector(verifyOnboardTargetEvidence:)
      || item.action == @selector(onboardSelfTest:) || item.action == @selector(createFlightTest:)
      || item.action == @selector(verifyFlightTest:)
      || item.action == @selector(validateFlightTest:)
      || item.action == @selector(createQualification:)
      || item.action == @selector(verifyQualification:)
      || item.action == @selector(verifyQualificationChain:))
    return !self.task;
  if (item.action == @selector(undoEdit:) || item.action == @selector(redoEdit:)) {
    BOOL undo = item.action == @selector(undoEdit:);
    NSTextView* editor = [self focusedTextEditor];
    NSUndoManager* history = editor ? editor.undoManager : self.edits;
    item.title = editor ? (undo ? @"Undo Text Edit" : @"Redo Text Edit")
                        : (undo ? @"Undo Model Edit" : @"Redo Model Edit");
    BOOL pending = self.propertyDirty || self.simulationDirty;
    return !self.task
           && (undo ? (history.canUndo || (!editor && pending))
                    : (history.canRedo && (editor || !pending)));
  }
  return YES;
}

- (void)textDidChange:(NSNotification*)notification {
  if (self.populating)
    return;
  if (notification.object == self.properties)
    self.propertyDirty = ![self.properties.string isEqual:JSONText([self selectedBlock])];
  if (notification.object == self.simulationEditor)
    self.simulationDirty =
        ![self.simulationEditor.string isEqual:JSONText(self.draft[@"simulation"])];
  if (self.project && (self.propertyDirty || self.simulationDirty))
    [self scheduleRecoverySnapshot];
  [self refresh];
}

- (NSMutableDictionary*)parseEditor:(NSTextView*)editor {
  NSError* error = nil;
  id value =
      [NSJSONSerialization JSONObjectWithData:[editor.string dataUsingEncoding:NSUTF8StringEncoding]
                                      options:NSJSONReadingMutableContainers
                                        error:&error];
  if (![value isKindOfClass:[NSDictionary class]]) {
    [self
        problem:error.localizedDescription ? error.localizedDescription : @"Enter a JSON object."];
    return nil;
  }
  return value;
}

- (BOOL)commitProperties {
  if (!self.propertyDirty)
    return YES;
  NSMutableDictionary* value = [self parseEditor:self.properties];
  if (!value)
    return NO;
  NSMutableDictionary* old = [self selectedBlock];
  if (!old || ![value[@"id"] isEqual:old[@"id"]] || ![value[@"kind"] isEqual:old[@"kind"]]) {
    [self problem:@"Keep the block id and kind unchanged. Add a new block to use another kind."];
    return NO;
  }
  if (![value[@"output"] isKindOfClass:[NSDictionary class]]
      || ([value[@"kind"] isEqual:@"sum"] && ![value[@"signs"] isKindOfClass:[NSArray class]])
      || ([value[@"kind"] isEqual:@"linear_combination"]
          && ![value[@"terms"] isKindOfClass:[NSArray class]])) {
    [self problem:@"A block needs an output object. A sum needs signs; a linear row needs an "
                  @"ordered terms array with input types and coefficients."];
    return NO;
  }
  if ([value isEqual:old]) {
    self.propertyDirty = NO;
    [self refresh];
    return YES;
  }
  [self checkpoint:@"Edit block properties"];
  NSMutableArray* blocks = self.draft[@"model"][@"blocks"];
  blocks[[blocks indexOfObject:old]] = value;
  self.propertyDirty = NO;
  [self refresh];
  return YES;
}

- (BOOL)commitSimulation {
  if (!self.simulationDirty)
    return YES;
  NSMutableDictionary* value = [self parseEditor:self.simulationEditor];
  if (!value)
    return NO;
  if ([value isEqual:self.draft[@"simulation"]]) {
    self.simulationDirty = NO;
    [self refresh];
    return YES;
  }
  [self checkpoint:@"Edit simulation"];
  self.draft[@"simulation"] = value;
  self.simulationDirty = NO;
  [self refresh];
  return YES;
}

- (void)applyProperties:(id)sender {
  (void)sender;
  if (self.task || !self.draft)
    return;
  if ([self commitProperties])
    self.status.stringValue = @"Properties applied to draft. Save to validate the draft.";
}

- (void)applySimulation:(id)sender {
  (void)sender;
  if (self.task || !self.draft)
    return;
  if ([self commitSimulation])
    self.status.stringValue = @"Simulation applied to draft. Save to validate.";
}

- (void)addBlock:(id)sender {
  (void)sender;
  if (self.task || !self.draft || ![self commitProperties])
    return;
  NSString* kind = self.palette.titleOfSelectedItem;
  if ([kind isEqual:@"linear_combination"]
      && ![self.draft[@"model"][@"profile"] isEqual:@"continuous-linear.v1"]) {
    [self
        problem:
            @"Linear rows require a continuous-linear.v1 project. Import a linear study to begin."];
    return;
  }
  NSUInteger suffix = 1;
  NSString* identifier;
  NSArray* ids = [[self blocks] valueForKey:@"id"];
  do {
    identifier = [NSString stringWithFormat:@"%@_%lu", kind, (unsigned long)suffix++];
  } while ([ids containsObject:identifier]);
  NSMutableDictionary* block = JSONCopy(@{
    @"id": identifier,
    @"kind": kind,
    @"output": @{@"dimension": @[@0, @0, @0, @0, @0, @0, @0, @0], @"frame": @"none"}
  });
  if ([kind isEqual:@"constant"])
    block[@"value"] = @1;
  if ([kind isEqual:@"gain"])
    block[@"coefficient"] =
        [@{@"value": @1, @"dimension": @[@0, @0, @0, @0, @0, @0, @0, @0]} mutableCopy];
  if ([kind isEqual:@"sum"])
    block[@"signs"] = [@[@1, @1] mutableCopy];
  if ([kind isEqual:@"integrator"])
    block[@"initial_value"] = @0;
  if ([kind isEqual:@"linear_combination"]) {
    block[@"terms"] = [@[JSONCopy(@{
      @"input": block[@"output"],
      @"coefficient": @{@"value": @1, @"dimension": @[@0, @0, @0, @0, @0, @0, @0, @0]}
    })] mutableCopy];
  }
  NSSize size = BlockCardSize(block);
  NSRect visible = self.diagram.visibleRect;
  CGFloat startX = fmax(40, NSMinX(visible) + 40) - self.diagramOffset.x;
  CGFloat startY = fmax(40, NSMinY(visible) + 40) - self.diagramOffset.y;
  NSUInteger columns = (NSUInteger)fmax(1, fmin(12, floor(visible.size.width / (size.width + 40))));
  NSRect placement = NSMakeRect(startX, startY, size.width, size.height);
  BOOL placed = NO;
  for (NSUInteger attempt = 0; attempt < 4096; ++attempt) {
    placement.origin = NSMakePoint(startX + (attempt % columns) * (size.width + 40),
                                   startY + (attempt / columns) * (size.height + 40));
    placed = YES;
    for (NSDictionary* existing in [self blocks]) {
      if (NSIntersectsRect(placement, NSInsetRect([self modelRectForBlock:existing], -24, -24))) {
        placed = NO;
        break;
      }
    }
    if (placed)
      break;
  }
  if (!placed) {
    placement.origin = NSMakePoint(startX, startY);
    for (NSDictionary* existing in [self blocks])
      placement.origin.y = fmax(placement.origin.y, NSMaxY([self modelRectForBlock:existing]) + 40);
  }
  if (!CanvasPositionSupported(placement.origin)) {
    [self problem:@"There is no room for this block within the supported canvas size. Move some "
                  @"blocks closer together or return to another part of the diagram, then add it."];
    return;
  }
  [self checkpoint:@"Add block"];
  self.draft[@"presentation"][@"positions"][identifier] =
      [@{@"x": @(placement.origin.x), @"y": @(placement.origin.y)} mutableCopy];
  [self.draft[@"model"][@"blocks"] addObject:block];
  [self selectBlock:identifier];
  [self refresh];
  [self showDiagram:nil];
  [self setDiagramZoom:1];
  self.status.stringValue =
      @"Block added. Set its dimensions and connect required inputs, then Save.";
}

- (void)deleteBlock:(id)sender {
  (void)sender;
  if (self.task || ![self selectedBlock] || ![self commitProperties])
    return;
  [self.diagram cancelWiring];
  [self checkpoint:@"Delete block"];
  for (NSDictionary* wire in [self connections])
    if ([wire[@"source"] isEqual:self.selectedID] || [wire[@"target"] isEqual:self.selectedID])
      [self removeManualRouteForWire:wire];
  [self.draft[@"model"][@"blocks"] removeObject:[self selectedBlock]];
  NSMutableArray* wires = self.draft[@"model"][@"connections"];
  NSIndexSet* indices =
      [wires indexesOfObjectsPassingTest:^BOOL(NSDictionary* wire, NSUInteger index, BOOL* stop) {
        (void)index;
        (void)stop;
        return
            [wire[@"source"] isEqual:self.selectedID] || [wire[@"target"] isEqual:self.selectedID];
      }];
  [wires removeObjectsAtIndexes:indices];
  [self.draft[@"presentation"][@"positions"] removeObjectForKey:self.selectedID];
  self.selectedID = nil;
  [self populateEditors];
  [self refresh];
}

- (void)connect:(id)sender {
  (void)sender;
  if (self.task || !self.draft || !self.source.titleOfSelectedItem
      || !self.target.titleOfSelectedItem)
    return;
  NSScanner* scanner = [NSScanner scannerWithString:self.input.stringValue];
  NSInteger slot = -1;
  if (![scanner scanInteger:&slot] || !scanner.isAtEnd || slot < 0) {
    [self problem:@"Input index must be a nonnegative integer (the first input is 0)."];
    return;
  }
  NSDictionary* wire = @{
    @"source": self.source.titleOfSelectedItem,
    @"target": self.target.titleOfSelectedItem,
    @"input": @(slot)
  };
  [self applyConnection:wire replacing:self.highlightedWire];
}

- (BOOL)applyConnection:(NSDictionary*)wire replacing:(NSDictionary*)oldWire {
  if (self.task || !self.draft || ![self commitProperties])
    return NO;
  NSString* error = nil;
  NSArray* updated = GalataConnectionEdit([self blocks], [self connections], oldWire, wire, &error);
  if (!updated) {
    [self problem:error];
    return NO;
  }
  if ([updated isEqual:[self connections]]) {
    [self chooseWire:wire];
    self.status.stringValue = @"Wire endpoints are unchanged.";
    return YES;
  }
  [self checkpoint:oldWire ? @"Reconnect wire" : @"Create wire"];
  if (oldWire)
    [self removeManualRouteForWire:oldWire];
  self.draft[@"model"][@"connections"] = [updated mutableCopy];
  [self chooseWire:wire];
  self.status.stringValue =
      [NSString stringWithFormat:@"%@ · %@. Command-Z undoes this edit; Save keeps it.",
                                 oldWire ? @"Wire updated" : @"Wire created",
                                 WireName(wire, [[self connections] indexOfObject:wire])];
  return YES;
}

- (void)disconnect:(id)sender {
  (void)sender;
  if (self.task || !self.highlightedWire || ![self commitProperties])
    return;
  [self.diagram cancelWiring];
  NSUInteger index = [[self connections] indexOfObject:self.highlightedWire];
  if (index == NSNotFound)
    return;
  [self checkpoint:@"Remove wire"];
  [self removeManualRouteForWire:self.highlightedWire];
  [self.draft[@"model"][@"connections"] removeObjectAtIndex:index];
  [self selectBlock:nil];
  [self refresh];
  self.status.stringValue = @"Wire removed. Blocks are unchanged. Command-Z restores the wire.";
}

// Launch directly, without a shell. Drain the combined pipe off the UI thread
// so both large diagnostics and long simulations remain cancellable.
- (void)command:(NSArray<NSString*>*)arguments
     completion:(void (^)(NSDictionary*, NSString*))completion {
  if (self.task)
    return;
  [self.diagram cancelWiring];
  if (![[NSFileManager defaultManager] isExecutableFileAtPath:self.engine]) {
    completion(nil,
               [NSString stringWithFormat:@"Cannot launch engine at %@. Build galata_cli or launch "
                                          @"with --engine /path/to/galata.",
                                          self.engine]);
    return;
  }
  NSTask* task = [[NSTask alloc] init];
  task.executableURL = [NSURL fileURLWithPath:self.engine];
  task.arguments = arguments;
  NSPipe* pipe = [NSPipe pipe];
  task.standardOutput = pipe;
  task.standardError = pipe;
  NSError* error = nil;
  if (![task launchAndReturnError:&error]) {
    completion(nil, error.localizedDescription);
    return;
  }
  self.task = task;
  self.cancelling = NO;
  self.status.stringValue =
      [NSString stringWithFormat:@"Engine: %@…", [arguments componentsJoinedByString:@" "]];
  [self refresh];
  dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
    NSMutableData* output = [NSMutableData data];
    BOOL truncated = NO;
    while (YES) {
      NSData* part = [pipe.fileHandleForReading availableData];
      if (part.length == 0)
        break;
      if (output.length + part.length <= 8 * 1024 * 1024)
        [output appendData:part];
      else
        truncated = YES;
    }
    [task waitUntilExit];
    NSString* message = [[NSString alloc] initWithData:output encoding:NSUTF8StringEncoding];
    if (!message)
      message = @"Engine returned non-UTF-8 output.";
    id decoded = truncated ? nil
                           : [NSJSONSerialization JSONObjectWithData:output
                                                             options:NSJSONReadingMutableContainers
                                                               error:nil];
    NSDictionary* result = [decoded isKindOfClass:[NSDictionary class]] ? decoded : nil;
    if (truncated)
      message = @"Engine output exceeded the preview's 8 MiB limit.";
    if (!result && message.length == 0)
      message =
          [NSString stringWithFormat:@"Engine exited with status %d.", task.terminationStatus];
    dispatch_async(dispatch_get_main_queue(), ^{
      self.task = nil;
      self.cancelling = NO;
      [self refresh];
      completion(result, result ? nil : message);
    });
  });
}

- (void)adoptProjectView:(NSDictionary*)view {
  [self.diagram cancelWiring];
  // Reloading the same head can still change retained-file validity and runs.
  // An adopted editor view invalidates the complete previous review snapshot.
  if (self.recoveryProject) {
    [self.revisionsWindow orderOut:nil];
    self.recoveryProject = nil;
    self.recoveryHead = nil;
    self.revisionHistory = nil;
    self.reviewedRevision = nil;
    [self.revisionChoices removeAllItems];
    self.revisionPreview.string = @"";
  }
  self.projectView = view;
  self.revision = [view[@"revision"] description];
  self.draft = JSONCopy(@{
    @"schema": view[@"draft_schema"] ? view[@"draft_schema"] : @"galata.project-draft.v1",
    @"model": view[@"model"],
    @"presentation": view[@"presentation"],
    @"simulation": view[@"simulation"]
  });
  if (view[@"origin_sha256"])
    self.draft[@"origin_sha256"] = view[@"origin_sha256"];
  if (!self.draft[@"presentation"][@"positions"])
    self.draft[@"presentation"][@"positions"] = [NSMutableDictionary dictionary];
  self.dirty = NO;
  self.propertyDirty = NO;
  self.simulationDirty = NO;
  if (![self selectedBlock])
    self.selectedID = nil;
  [self.edits removeAllActions];
  [self populateEditors];
  [self.history removeAllItems];
  for (NSDictionary* run in view[@"runs"]) {
    NSString* identifier = run[@"id"];
    NSString* shortID = [identifier substringToIndex:MIN((NSUInteger)12, identifier.length)];
    [self.history addItemWithTitle:[NSString stringWithFormat:@"%@ · %@", shortID, run[@"status"]]];
    self.history.lastItem.representedObject = run;
  }
  if (self.history.numberOfItems) {
    [self.history selectItemAtIndex:self.history.numberOfItems - 1];
    for (NSMenuItem* item in self.history.itemArray) {
      if ([item.representedObject[@"id"] isEqual:self.selectedRunID])
        [self.history selectItem:item];
    }
    [self selectRun:self];
  } else {
    self.evidence.string = @"Run evidence will appear here. Completed execution does not establish "
                           @"numerical acceptance.";
    self.plot.samples = @[];
    [self.samplesTable setHeaders:@[] rows:@[]];
    self.sampleSummary.stringValue = @"No samples";
    self.plot.needsDisplay = YES;
    [self.series removeAllItems];
  }
  if (self.originWindow.isVisible) {
    if (view[@"origin"])
      [self showOriginalStudy:self];
    else
      [self.originWindow orderOut:nil];
  }
  [self refresh];
  if (self.diagramFitMode)
    [self fitDiagram:nil];
}

- (BOOL)isProjectView:(NSDictionary*)view {
  if (!([view[@"schema"] isEqual:@"galata.project-view.v1"] &&
        [view[@"model"] isKindOfClass:[NSDictionary class]] &&
        [view[@"model"][@"blocks"] isKindOfClass:[NSArray class]] &&
        [view[@"model"][@"connections"] isKindOfClass:[NSArray class]] &&
        [view[@"presentation"] isKindOfClass:[NSDictionary class]] &&
        [view[@"simulation"] isKindOfClass:[NSDictionary class]] && view[@"revision"]))
    return NO;
  NSString* schema = view[@"draft_schema"] ? view[@"draft_schema"] : @"galata.project-draft.v1";
  if (![schema isEqual:@"galata.project-draft.v1"] && ![schema isEqual:@"galata.project-draft.v2"])
    return NO;
  id originHash = view[@"origin_sha256"];
  if ([schema isEqual:@"galata.project-draft.v2"] && !originHash)
    return NO;
  if (originHash
      && (![originHash isKindOfClass:[NSString class]] || [originHash length] != 64 ||
          [originHash
              rangeOfCharacterFromSet:[[NSCharacterSet
                                          characterSetWithCharactersInString:@"0123456789abcdef"]
                                          invertedSet]]
                  .location
              != NSNotFound))
    return NO;
  id origin = view[@"origin"];
  if (origin
      && (![origin isKindOfClass:[NSDictionary class]]
          || ![origin[@"schema"] isEqual:@"galata.project-origin-view.v1"]
          || ![origin[@"adapter"] isKindOfClass:[NSDictionary class]]
          || ![origin[@"manifest"] isKindOfClass:[NSDictionary class]]))
    return NO;
  return YES;
}

- (void)showOriginalStudy:(id)sender {
  (void)sender;
  NSDictionary* origin = self.projectView[@"origin"];
  if (![origin isKindOfClass:[NSDictionary class]])
    return;
  if (!self.originWindow) {
    self.originWindow =
        [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 850, 680)
                                    styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                              | NSWindowStyleMaskResizable
                                      backing:NSBackingStoreBuffered
                                        defer:NO];
    DimWindow(self.originWindow);
    self.originWindow.releasedWhenClosed = NO;
    self.originWindow.contentMinSize = NSMakeSize(640, 400);
    NSView* content = self.originWindow.contentView;
    self.originEvidence = TextEditor(content, NSMakeRect(12, 52, 826, 616), NO);
    self.originEvidence.enclosingScrollView.autoresizingMask =
        NSViewWidthSizable | NSViewHeightSizable;
    self.originEvidence.accessibilityLabel = @"ORIGINAL study context and manifest, read only";
    [content addSubview:Button(@"Reveal Original Manifest",
                               NSMakeRect(12, 12, 220, 30),
                               self,
                               @selector(revealOriginalManifest:))];
    NSTextField* note =
        Label(@"Original source context · selected-run evidence stays in the project window",
              NSMakeRect(246, 17, 590, 20));
    note.autoresizingMask = NSViewWidthSizable;
    note.lineBreakMode = NSLineBreakByTruncatingTail;
    [content addSubview:note];
    [self.originWindow center];
  }
  self.originWindow.title =
      [NSString stringWithFormat:@"ORIGINAL Study — %@", self.project.lastPathComponent];
  NSString* path = [origin[@"manifest_path"] isKindOfClass:[NSString class]]
                       ? origin[@"manifest_path"]
                       : @"Unavailable";
  self.originEvidence.string = [NSString
      stringWithFormat:
          @"ORIGINAL IMPORTED STUDY — READ-ONLY SOURCE CONTEXT\n\n"
          @"Saved graph relation: %@\n%@"
          @"Original model semantic SHA-256: %@\nOriginal manifest: %@\n\n"
          @"This context records the imported study. Its source assumptions and controller design "
          @"describe the original model. Review the selected run in the project window for its "
          @"own graph identity, relation, and execution evidence.\n\n"
          @"ORIGINAL ADAPTER\n%@\n\nORIGINAL MANIFEST\n%@",
          OriginRelation(origin[@"relation"]),
          (self.dirty || self.propertyDirty || self.simulationDirty)
              ? @"Current draft has unsaved edits; the saved relation does not describe those "
                @"edits.\n"
              : @"",
          origin[@"model_semantic_sha256"] ? origin[@"model_semantic_sha256"] : @"Unavailable",
          path,
          JSONText(origin[@"adapter"]),
          JSONText(origin[@"manifest"])];
  [self.originWindow makeKeyAndOrderFront:nil];
}

- (void)showRevisions:(id)sender {
  (void)sender;
  if (self.task || !self.project)
    return;
  NSString* project = self.project;
  [self command:@[@"project", @"revisions", project]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.project-history.v1"]
            || ![result[@"revisions"] isKindOfClass:[NSArray class]]) {
          [self problem:error ? error : @"Cannot read saved revisions."];
          return;
        }
        self.recoveryProject = project;
        self.recoveryHead = result[@"current_revision"];
        self.revisionHistory = result;
        self.reviewedRevision = nil;
        if (!self.revisionsWindow) {
          self.revisionsWindow = [[NSWindow alloc]
              initWithContentRect:NSMakeRect(0, 0, 850, 650)
                        styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable
                                  | NSWindowStyleMaskResizable
                          backing:NSBackingStoreBuffered
                            defer:NO];
          self.revisionsWindow.releasedWhenClosed = NO;
          self.revisionsWindow.contentMinSize = NSMakeSize(650, 400);
          DimWindow(self.revisionsWindow);
          NSView* content = self.revisionsWindow.contentView;
          NSTextField* description = Label(@"Review a saved revision before restoring. Newer "
                                           @"revisions and all runs stay available.",
                                           NSMakeRect(14, 612, 820, 22));
          description.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
          [content addSubview:description];
          self.revisionChoices = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(12, 572, 824, 30)
                                                            pullsDown:NO];
          self.revisionChoices.autoresizingMask = NSViewWidthSizable | NSViewMinYMargin;
          self.revisionChoices.accessibilityLabel = @"Saved revision to review";
          self.revisionChoices.target = self;
          self.revisionChoices.action = @selector(selectRevision:);
          [content addSubview:self.revisionChoices];
          self.revisionPreview = TextEditor(content, NSMakeRect(12, 54, 824, 508), NO);
          self.revisionPreview.accessibilityLabel = @"Saved revision contents, read only";
          self.revisionPreview.enclosingScrollView.autoresizingMask =
              NSViewWidthSizable | NSViewHeightSizable;
          self.restoreButton = Button(@"Restore Reviewed Revision",
                                      NSMakeRect(12, 12, 240, 30),
                                      self,
                                      @selector(restoreRevision:));
          DimPrimaryButton(self.restoreButton);
          [content addSubview:self.restoreButton];
          NSTextField* note = Label(@"Sorted by content ID · Save working edits before restoring",
                                    NSMakeRect(266, 17, 566, 20));
          note.textColor = DimMuted();
          note.autoresizingMask = NSViewWidthSizable;
          [content addSubview:note];
          [self.revisionsWindow center];
        }
        self.revisionsWindow.title =
            [NSString stringWithFormat:@"Saved Revisions — %@", project.lastPathComponent];
        [self.revisionChoices removeAllItems];
        for (NSDictionary* entry in result[@"revisions"]) {
          NSString* revision = [entry[@"revision"] description];
          NSString* shortID = [revision substringToIndex:MIN((NSUInteger)16, revision.length)];
          NSString* detail = [entry[@"status"] isEqual:@"valid"]
                                 ? [NSString stringWithFormat:@"%@ · %@ blocks",
                                                              entry[@"model_profile"],
                                                              entry[@"block_count"]]
                                 : @"invalid retained revision";
          [self.revisionChoices
              addItemWithTitle:[NSString stringWithFormat:@"%@ · %@%@",
                                                          shortID,
                                                          detail,
                                                          [entry[@"is_current"] boolValue]
                                                              ? @" · current"
                                                              : @""]];
          self.revisionChoices.lastItem.representedObject = entry;
        }
        for (NSMenuItem* item in self.revisionChoices.itemArray)
          if ([item.representedObject[@"is_current"] boolValue])
            [self.revisionChoices selectItem:item];
        [self.revisionsWindow makeKeyAndOrderFront:nil];
        [self selectRevision:self];
      }];
}

- (void)selectRevision:(id)sender {
  (void)sender;
  if (self.task)
    return;
  self.reviewedRevision = nil;
  [self refresh];
  NSString* notice = @"";
  if (![self.revisionHistory[@"current_status"] isEqual:@"valid"]) {
    notice = [NSString
        stringWithFormat:@"The current saved revision is invalid. Restore is unavailable.\n%@\n\n",
                         self.revisionHistory[@"current_diagnostic"]
                             ? self.revisionHistory[@"current_diagnostic"]
                             : @"The current revision could not be verified."];
  } else if (![self.revision isEqual:self.recoveryHead]) {
    notice = @"The open editor is stale: another command changed the saved project head. "
             @"Reopen this project before restoring. Preserve any unsaved work before "
             @"reopening.\n\n";
  }
  NSDictionary* entry = self.revisionChoices.selectedItem.representedObject;
  if (![entry[@"status"] isEqual:@"valid"]) {
    self.revisionPreview.string =
        [notice stringByAppendingString:entry ? JSONText(entry) : @"No retained revisions."];
    return;
  }
  NSString* selected = entry[@"revision"];
  NSString* project = self.recoveryProject;
  self.revisionPreview.string =
      [notice stringByAppendingString:@"Reading and verifying the selected revision…"];
  [self command:@[@"project", @"revision", project, selected]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.project-revision.v1"]
            || ![result[@"revision"] isEqual:selected]
            || ![result[@"draft"] isKindOfClass:[NSDictionary class]]) {
          self.revisionPreview.string =
              [notice stringByAppendingString:error ? error : @"Cannot verify this revision."];
          return;
        }
        NSDictionary* draft = result[@"draft"];
        BOOL compatible = [draft[@"schema"] isEqual:self.draft[@"schema"]]
                          && ((!draft[@"origin_sha256"] && !self.draft[@"origin_sha256"]) ||
                              [draft[@"origin_sha256"] isEqual:self.draft[@"origin_sha256"]]);
        self.revisionPreview.string = [NSString
            stringWithFormat:@"SAVED REVISION — READ ONLY\n%@%@\n\n%@",
                             notice,
                             compatible ? @"Restoring changes the saved project head. All retained "
                                          @"evidence remains available."
                                        : @"This revision has a different source attachment or "
                                          @"schema and cannot be restored here.",
                             JSONText(result)];
        if (compatible && [self.recoveryProject isEqual:project])
          self.reviewedRevision = selected;
        [self refresh];
      }];
}

- (void)restoreRevision:(id)sender {
  (void)sender;
  if (!self.restoreButton.enabled || self.task || !self.reviewedRevision)
    return;
  NSString* selected = self.reviewedRevision;
  NSString* project = self.recoveryProject;
  [self command:@[
    @"project",
    @"restore",
    project,
    selected,
    @"--expected-revision",
    self.recoveryHead
  ]
      completion:^(NSDictionary* result, NSString* error) {
        self.reviewedRevision = nil;
        if (![self isProjectView:result]) {
          self.revisionPreview.string =
              error ? error : @"Restore was refused. Reopen saved revisions to retry.";
          [self refresh];
          return;
        }
        [self adoptProjectView:result];
        [self.revisionsWindow orderOut:nil];
        [self.window makeKeyAndOrderFront:nil];
        self.status.stringValue = [NSString
            stringWithFormat:
                @"Restored revision %@. Newer revisions and all runs remain available.", selected];
      }];
}

- (void)revealOriginalManifest:(id)sender {
  (void)sender;
  NSString* path = self.projectView[@"origin"][@"manifest_path"];
  if ([path isKindOfClass:[NSString class]] && path.isAbsolutePath &&
      [[NSFileManager defaultManager] fileExistsAtPath:path])
    [[NSWorkspace sharedWorkspace] activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:path]]];
  else
    [self
        problem:
            @"The original manifest is unavailable. Its verified view remains in Original Study."];
}

- (void)loadProject:(NSString*)path {
  void (^inspectProject)(void) = ^{
    [self command:@[@"project", @"inspect", path]
        completion:^(NSDictionary* result, NSString* error) {
          if (![self isProjectView:result]) {
            [self problem:error ? error : @"Engine returned an invalid project view."];
            return;
          }
          self.project = path;
          self.diagramFitMode = YES;
          [self adoptProjectView:result];
          self.status.stringValue =
              [NSString stringWithFormat:@"Opened %@ · revision %@", path, self.revision];
          [self offerRecoverySnapshotForProject:path];
        }];
  };
  NSString* manifest = [path stringByAppendingPathComponent:@"review.json"];
  if ([[NSFileManager defaultManager] fileExistsAtPath:manifest]) {
    [self command:@[@"project", @"verify", path]
        completion:^(NSDictionary* result, NSString* error) {
          if (![result[@"schema"] isEqual:@"galata.project-review-verification.v1"]
              || ![result[@"status"] isEqual:@"verified"]) {
            [self problem:error ? error : @"The review package failed integrity verification."];
            return;
          }
          self.status.stringValue = @"Review package verified. Loading its saved project view…";
          inspectProject();
        }];
  } else {
    inspectProject();
  }
}

- (void)newProject:(id)sender {
  (void)sender;
  if (![self canReplaceProject])
    return;
  NSSavePanel* chooser = [NSSavePanel savePanel];
  chooser.title = @"Create Galata Project Directory";
  chooser.nameFieldStringValue = @"Untitled.galata";
  chooser.canCreateDirectories = YES;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* path = chooser.URL.path;
  if (![path.pathExtension isEqual:@"galata"])
    path = [path stringByAppendingPathExtension:@"galata"];
  [self command:@[@"project", @"create", path]
      completion:^(NSDictionary* result, NSString* error) {
        if (error) {
          [self problem:error];
          return;
        }
        if ([self isProjectView:result]) {
          self.project = path;
          self.diagramFitMode = YES;
          [self discardRecoverySnapshotForProject:path];
          [self adoptProjectView:result];
          self.status.stringValue = @"Project created. Edit the diagram or run the saved example.";
        } else
          [self loadProject:path];
      }];
}

- (void)openProject:(id)sender {
  (void)sender;
  if (![self canReplaceProject])
    return;
  NSOpenPanel* chooser = [NSOpenPanel openPanel];
  chooser.title = @"Open Galata Project";
  // Registered .galata packages are selectable as documents; ordinary project
  // directories remain selectable for existing extensionless workspaces.
  chooser.canChooseFiles = YES;
  chooser.canChooseDirectories = YES;
  chooser.allowsMultipleSelection = NO;
  chooser.treatsFilePackagesAsDirectories = NO;
  if ([chooser runModal] == NSModalResponseOK)
    [self loadProject:chooser.URL.path];
}

- (void)importStudy:(id)sender {
  (void)sender;
  if (![self canReplaceProject])
    return;
  NSOpenPanel* source = [NSOpenPanel openPanel];
  source.title = @"Import Linear Study";
  source.message = @"Choose a study YAML that exports a typed linear model. The original study and "
                   @"manifest will remain available in the new project.";
  source.canChooseFiles = YES;
  source.canChooseDirectories = NO;
  source.allowsMultipleSelection = NO;
  if ([source runModal] != NSModalResponseOK)
    return;
  NSString* study = source.URL.path;
  NSSavePanel* destination = [NSSavePanel savePanel];
  destination.title = @"Create Imported Galata Project Directory";
  destination.message =
      @"Choose a new project directory for the imported model and original study evidence.";
  destination.nameFieldStringValue = [study.lastPathComponent.stringByDeletingPathExtension
      stringByAppendingPathExtension:@"galata"];
  destination.canCreateDirectories = YES;
  if ([destination runModal] != NSModalResponseOK)
    return;
  NSString* path = destination.URL.path;
  if (![path.pathExtension isEqual:@"galata"])
    path = [path stringByAppendingPathExtension:@"galata"];
  [self command:@[@"project", @"import-linear", path, study]
      completion:^(NSDictionary* result, NSString* error) {
        if (![self isProjectView:result]) {
          [self problem:error ? error : @"Engine refused the study import."];
          return;
        }
        self.project = path;
        self.diagramFitMode = YES;
        self.selectedID = nil;
        self.selectedRunID = nil;
        [self discardRecoverySnapshotForProject:path];
        [self adoptProjectView:result];
        self.status.stringValue = @"Study imported. Original Study retains source context; Run "
                                  @"Saved records new graph execution evidence.";
      }];
}

- (void)saveProject:(id)sender {
  (void)sender;
  if (self.task || !self.project || ![self commitProperties] || ![self commitSimulation])
    return;
  NSString* temp = [NSTemporaryDirectory()
      stringByAppendingPathComponent:[NSString stringWithFormat:@"galata-draft-%@.json",
                                                                [NSUUID UUID].UUIDString]];
  NSError* writeError = nil;
  if (![JSONText(self.draft) writeToFile:temp
                              atomically:YES
                                encoding:NSUTF8StringEncoding
                                   error:&writeError]) {
    [self problem:writeError.localizedDescription];
    return;
  }
  [self command:@[@"project", @"save", self.project, temp, @"--expected-revision", self.revision]
      completion:^(NSDictionary* result, NSString* error) {
        [[NSFileManager defaultManager] removeItemAtPath:temp error:nil];
        if (![self isProjectView:result]) {
          [self problem:error ? error : @"Engine refused to save the draft."];
          return;
        }
        [self discardRecoverySnapshotForProject:self.project];
        [self adoptProjectView:result];
        self.status.stringValue = [NSString
            stringWithFormat:@"Draft saved · revision %@. Run Saved compiles and executes it.",
                             self.revision];
      }];
}

- (void)runProject:(id)sender {
  (void)sender;
  if (self.task || !self.project)
    return;
  if (self.dirty || self.propertyDirty || self.simulationDirty) {
    [self problem:@"Save the draft before running. Each run uses an immutable saved snapshot."];
    return;
  }
  NSString* path = self.project;
  NSArray* existingIDs = [self.projectView[@"runs"] valueForKey:@"id"];
  [self command:@[@"project", @"run", path]
      completion:^(NSDictionary* result, NSString* error) {
        NSString* finalStatus =
            result
                ? [NSString
                      stringWithFormat:
                          @"Run %@ · %@. Review evidence; execution is not numerical acceptance.",
                          result[@"id"],
                          result[@"status"]]
                : (error ? error : @"Run interrupted.");
        [self command:@[@"project", @"inspect", path]
            completion:^(NSDictionary* view, NSString* inspectError) {
              if ([self isProjectView:view]) {
                self.selectedRunID = result[@"id"];
                if (!self.selectedRunID) {
                  for (NSDictionary* run in view[@"runs"]) {
                    if (![existingIDs containsObject:run[@"id"]])
                      self.selectedRunID = run[@"id"];
                  }
                }
                [self adoptProjectView:view];
              }
              self.status.stringValue = finalStatus;
              if (inspectError)
                self.evidence.string =
                    [NSString stringWithFormat:@"%@\n\n%@", finalStatus, inspectError];
            }];
      }];
}

- (void)runStudy:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* studyChooser = [NSOpenPanel openPanel];
  studyChooser.title = @"Run Galata Study";
  studyChooser.message =
      @"Choose any Galata study YAML, including model validation and flight-test workflows.";
  studyChooser.canChooseFiles = YES;
  studyChooser.canChooseDirectories = NO;
  studyChooser.allowsMultipleSelection = NO;
  if ([studyChooser runModal] != NSModalResponseOK)
    return;

  NSString* study = studyChooser.URL.path;
  NSString* basename = study.lastPathComponent.stringByDeletingPathExtension;
  NSSavePanel* outputChooser = [NSSavePanel savePanel];
  outputChooser.title = @"Choose Study Output Directory";
  outputChooser.message = @"Choose a new or empty directory. The shared CLI writes the run "
                          @"manifest and artifacts there.";
  outputChooser.nameFieldStringValue = [NSString stringWithFormat:@"%@-run", basename];
  outputChooser.canCreateDirectories = YES;
  if ([outputChooser runModal] != NSModalResponseOK)
    return;
  NSString* output = outputChooser.URL.path;

  if (![[NSFileManager defaultManager] isExecutableFileAtPath:self.engine]) {
    [self problem:[NSString stringWithFormat:@"Cannot launch engine at %@. Build galata_cli or "
                                             @"launch with --engine /path/to/galata.",
                                             self.engine]];
    return;
  }

  NSTask* task = [[NSTask alloc] init];
  task.executableURL = [NSURL fileURLWithPath:self.engine];
  task.arguments = @[@"run", study, @"--output-dir", output];
  task.currentDirectoryURL = studyChooser.URL.URLByDeletingLastPathComponent;
  NSPipe* pipe = [NSPipe pipe];
  task.standardOutput = pipe;
  task.standardError = pipe;
  NSError* launchError = nil;
  if (![task launchAndReturnError:&launchError]) {
    [self problem:launchError.localizedDescription];
    return;
  }
  self.task = task;
  self.cancelling = NO;
  self.status.stringValue =
      [NSString stringWithFormat:@"Running study: %@…", study.lastPathComponent];
  [self refresh];

  dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
    NSMutableData* outputData = [NSMutableData data];
    BOOL truncated = NO;
    while (YES) {
      NSData* part = [pipe.fileHandleForReading availableData];
      if (part.length == 0)
        break;
      if (outputData.length + part.length <= 8 * 1024 * 1024)
        [outputData appendData:part];
      else
        truncated = YES;
    }
    [task waitUntilExit];
    NSString* log = [[NSString alloc] initWithData:outputData encoding:NSUTF8StringEncoding];
    if (!log)
      log = @"Engine returned non-UTF-8 output.";
    if (truncated)
      log = [log stringByAppendingString:@"\n\n[Engine log truncated at 8 MiB.]\n"];
    const int terminationStatus = task.terminationStatus;
    dispatch_async(dispatch_get_main_queue(), ^{
      self.task = nil;
      self.cancelling = NO;
      self.evidence.string = log;
      [self refresh];
      if (terminationStatus == 0) {
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Study completed";
        alert.informativeText = [NSString
            stringWithFormat:
                @"Output directory:\n%@\n\nThe full engine log is available in the Evidence pane. "
                @"Execution completion is not numerical, flight-test, airworthiness or "
                @"certification acceptance.",
                output];
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"Open Output"];
        [alert addButtonWithTitle:@"Close"];
        if ([alert runModal] == NSAlertFirstButtonReturn &&
            [[NSFileManager defaultManager] fileExistsAtPath:output]) {
          [[NSWorkspace sharedWorkspace]
              activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:output]]];
        }
        self.status.stringValue = [NSString stringWithFormat:@"Study completed · %@", output];
      } else {
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Study failed or was cancelled";
        alert.informativeText = [NSString
            stringWithFormat:@"Exit status: %d\nOutput directory: %@\n\nSee the Evidence pane "
                             @"for the complete engine log.",
                             terminationStatus,
                             output];
        alert.alertStyle = NSAlertStyleWarning;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Study exited with status %d", terminationStatus];
      }
    });
  });
}

- (void)exportReview:(id)sender {
  (void)sender;
  if (self.task || !self.project)
    return;
  if (self.dirty || self.propertyDirty || self.simulationDirty) {
    [self problem:@"Save the draft before exporting. The review package contains only immutable "
                  @"saved content."];
    return;
  }
  NSSavePanel* chooser = [NSSavePanel savePanel];
  chooser.title = @"Export Galata Review Package";
  chooser.nameFieldStringValue = [self.project.lastPathComponent.stringByDeletingPathExtension
      stringByAppendingPathExtension:@"galata-review"];
  chooser.canCreateDirectories = YES;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* project = self.project;
  NSString* destination = chooser.URL.path;
  [self command:@[@"project", @"export", project, destination]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.project-review-export.v1"]) {
          [self problem:error ? error : @"Engine refused to export the review package."];
          return;
        }
        self.status.stringValue =
            [NSString stringWithFormat:@"Review package exported · %@ · verify before acceptance.",
                                       result[@"destination"]];
        [[NSWorkspace sharedWorkspace]
            activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:destination]]];
      }];
}

- (void)verifyReview:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* chooser = [NSOpenPanel openPanel];
  chooser.title = @"Verify Galata Review Package";
  chooser.canChooseFiles = NO;
  chooser.canChooseDirectories = YES;
  chooser.allowsMultipleSelection = NO;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* package = chooser.URL.path;
  [self command:@[@"project", @"verify", package]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.project-review-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]) {
          [self problem:error ? error : @"Engine could not verify the review package."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Directory: %@\nRevision: %@\nFiles: %@\nRetained runs: %@\nManifest SHA-256: "
                @"%@\n\nContent identity is verified. This does not establish numerical accuracy, "
                @"flight-test validity, airworthiness, certification or qualification.",
                package,
                result[@"revision"],
                result[@"file_count"],
                result[@"run_count"],
                result[@"manifest_sha256"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Review package verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Review package verified · %@", package];
      }];
}

- (void)verifyOnboard:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* chooser = [NSOpenPanel openPanel];
  chooser.title = @"Verify Onboard Manifest";
  chooser.message =
      @"Choose the manifest-only handoff received from the reviewed model/controller build.";
  chooser.canChooseFiles = YES;
  chooser.canChooseDirectories = NO;
  chooser.allowsMultipleSelection = NO;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* manifest = chooser.URL.path;
  [self command:@[@"onboard", @"verify", manifest]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]) {
          [self problem:error ? error : @"Engine refused to verify the onboard manifest."];
          return;
        }
        NSArray* roles = [result[@"artifact_roles"] isKindOfClass:[NSArray class]]
                             ? result[@"artifact_roles"]
                             : @[];
        NSDictionary* profile = [result[@"transport_profile"] isKindOfClass:[NSDictionary class]]
                                    ? result[@"transport_profile"]
                                    : @{};
        NSString* details = [NSString
            stringWithFormat:
                @"Manifest: %@\nSHA-256: %@\nArtifact roles: %@\nController budget: %@ s\n"
                @"Transport: %@ · endpoint %@ · watchdog %@ s\n\nThis is a manifest-only, "
                @"not_qualified handoff. Verification checks structure and declared identities; "
                @"it does not install executable code or establish flight approval.",
                manifest,
                result[@"manifest_sha256"],
                roles.count ? [roles componentsJoinedByString:@", "] : @"none",
                result[@"max_controller_time_s"],
                profile[@"transport"],
                profile[@"endpoint"],
                profile[@"watchdog_timeout_s"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Onboard manifest verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Onboard manifest verified · %@", manifest];
      }];
}

- (void)verifyFlightTest:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* chooser = [NSOpenPanel openPanel];
  chooser.title = @"Verify Flight-Test Evidence";
  chooser.message =
      @"Choose a campaign manifest. Galata checks the controlled files and their SHA-256 "
       "identities; it does not qualify the aircraft or accept the flight.";
  chooser.canChooseFiles = YES;
  chooser.canChooseDirectories = NO;
  chooser.allowsMultipleSelection = NO;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* manifest = chooser.URL.path;
  [self command:@[@"flighttest", @"verify", manifest]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.flight-test-evidence-verification.v2"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"file_roles"] isKindOfClass:[NSArray class]]) {
          [self problem:error ? error
                              : @"Engine refused to verify the flight-test evidence package."];
          return;
        }
        NSArray* roles = result[@"file_roles"];
        NSString* details =
            [NSString stringWithFormat:
                          @"Manifest: %@\nSHA-256: %@\nAircraft: %@\nConfiguration: %@\nTest plan: "
                          @"%@\nEvidence class: %@\nReviewer: %@\nFiles: %@ (%@ bytes)\nRoles: "
                          @"%@\n\nControlled "
                          @"file identities and package completeness are verified. This remains "
                          @"not_qualified and makes no airworthiness or certification claim.",
                          manifest,
                          result[@"manifest_sha256"],
                          result[@"aircraft_id"],
                          result[@"aircraft_configuration"],
                          result[@"test_plan_id"],
                          result[@"evidence_class"],
                          result[@"reviewer_id"],
                          result[@"file_count"],
                          result[@"total_bytes"],
                          roles.count ? [roles componentsJoinedByString:@", "] : @"none"];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Flight-test evidence package verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Flight-test evidence verified · %@", manifest];
      }];
}

- (void)validateFlightTest:(id)sender {
  (void)sender;
  if (self.task)
    return;

  NSOpenPanel* campaignChooser = [NSOpenPanel openPanel];
  campaignChooser.title = @"Choose Flight-Test Campaign Manifest";
  campaignChooser.message =
      @"Choose the verified campaign.manifest that the study's acceptance block references.";
  campaignChooser.canChooseFiles = YES;
  campaignChooser.canChooseDirectories = NO;
  campaignChooser.allowsMultipleSelection = NO;
  if ([campaignChooser runModal] != NSModalResponseOK)
    return;

  NSOpenPanel* studyChooser = [NSOpenPanel openPanel];
  studyChooser.title = @"Choose Flight-Test Validation Study";
  studyChooser.message = @"Choose a Galata study containing identify.validate.vehicle and a "
                         @"campaign_manifest binding.";
  studyChooser.canChooseFiles = YES;
  studyChooser.canChooseDirectories = NO;
  studyChooser.allowsMultipleSelection = NO;
  if ([studyChooser runModal] != NSModalResponseOK)
    return;

  NSSavePanel* outputChooser = [NSSavePanel savePanel];
  outputChooser.title = @"Choose Validation Output Directory";
  outputChooser.message = @"Choose a new or empty directory. The run manifest, report and "
                          @"validation receipt are written here.";
  outputChooser.nameFieldStringValue =
      [studyChooser.URL.path.lastPathComponent.stringByDeletingPathExtension
          stringByAppendingString:@"-flight-validation"];
  outputChooser.canCreateDirectories = YES;
  if ([outputChooser runModal] != NSModalResponseOK)
    return;

  NSString* campaign = campaignChooser.URL.path;
  NSString* study = studyChooser.URL.path;
  NSString* output = outputChooser.URL.path;
  [self command:@[@"flighttest", @"validate", campaign, study, @"--output-dir", output]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.flight-test-validation-receipt.v1"]
            || ![result[@"receipt"] isKindOfClass:[NSString class]]) {
          [self problem:error ? error : @"Engine refused to execute the flight-test validation."];
          return;
        }
        NSString* status = result[@"status"];
        NSString* details = [NSString
            stringWithFormat:
                @"Receipt: %@\nCampaign manifest SHA-256: %@\nNumerical gate: %@\n"
                 "Flight-test evidence gate: %@\n\n%@\n\nThe receipt is an execution and evidence-"
                 "traceability result. It remains not_qualified and makes no airworthiness, "
                 "certification or authority-acceptance claim.",
                result[@"receipt"],
                result[@"campaign_manifest_sha256"],
                result[@"numerical_acceptance_gate"],
                result[@"flight_test_evidence_gate"],
                [status isEqual:@"gate_passed"]
                    ? @"The package-backed validation gate passed. Independent review and external "
                       "authority acceptance remain required."
                    : @"The study executed, but the production evidence gate is not ready. Review "
                       "the receipt's reasons and controlled evidence package."];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = [status isEqual:@"gate_passed"]
                                ? @"Flight-test validation receipt created"
                                : @"Flight-test validation is not ready";
        alert.informativeText = details;
        alert.alertStyle =
            [status isEqual:@"gate_passed"] ? NSAlertStyleInformational : NSAlertStyleWarning;
        [alert addButtonWithTitle:@"Open Receipt Folder"];
        [alert addButtonWithTitle:@"Close"];
        if ([alert runModal] == NSAlertFirstButtonReturn)
          [[NSWorkspace sharedWorkspace]
              activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:output]]];
        self.status.stringValue = [NSString
            stringWithFormat:@"Flight-test validation %@ · %@", status, result[@"receipt"]];
      }];
}

- (void)verifyQualification:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* chooser = [NSOpenPanel openPanel];
  chooser.title = @"Verify Qualification Dossier";
  chooser.message =
      @"Choose a dossier manifest. Galata checks the controlled evidence files and their "
       "SHA-256 identities; it does not grant qualification or certification.";
  chooser.canChooseFiles = YES;
  chooser.canChooseDirectories = NO;
  chooser.allowsMultipleSelection = NO;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* manifest = chooser.URL.path;
  [self command:@[@"qualification", @"verify", manifest]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.qualification-evidence-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"file_roles"] isKindOfClass:[NSArray class]]) {
          [self problem:error ? error : @"Engine refused to verify the qualification dossier."];
          return;
        }
        NSArray* roles = result[@"file_roles"];
        NSString* details = [NSString
            stringWithFormat:
                @"Manifest: %@\nSHA-256: %@\nProduct: %@ %@\nIntended use: %@\nAircraft: %@\n"
                 "Configuration: %@\nBasis: %@\nAuthority: %@\nFiles: %@ (%@ bytes)\nRoles: %@\n\n"
                 "The dossier is byte-complete and reviewable, but remains not_qualified. "
                 "External independent review and authorized authority acceptance are still "
                 "required; no airworthiness or certification claim is made.",
                manifest,
                result[@"manifest_sha256"],
                result[@"product_id"],
                result[@"product_version"],
                result[@"intended_use"],
                result[@"aircraft_id"],
                result[@"aircraft_configuration"],
                result[@"qualification_basis"],
                result[@"authority_id"],
                result[@"file_count"],
                result[@"total_bytes"],
                roles.count ? [roles componentsJoinedByString:@", "] : @"none"];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Qualification dossier verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Qualification dossier verified · %@", manifest];
      }];
}

- (void)verifyQualificationChain:(id)sender {
  (void)sender;
  if (self.task)
    return;

  NSOpenPanel* dossierChooser = [NSOpenPanel openPanel];
  dossierChooser.title = @"Choose Qualification Dossier Manifest";
  dossierChooser.message =
      @"Choose the dossier whose flight-test and target-evidence links will be checked.";
  dossierChooser.canChooseFiles = YES;
  dossierChooser.canChooseDirectories = NO;
  dossierChooser.allowsMultipleSelection = NO;
  if ([dossierChooser runModal] != NSModalResponseOK)
    return;

  NSOpenPanel* flightChooser = [NSOpenPanel openPanel];
  flightChooser.title = @"Choose Flight-Test Campaign Manifest";
  flightChooser.message =
      @"Choose the complete campaign package manifest. Its bytes must be linked by the dossier.";
  flightChooser.canChooseFiles = YES;
  flightChooser.canChooseDirectories = NO;
  flightChooser.allowsMultipleSelection = NO;
  if ([flightChooser runModal] != NSModalResponseOK)
    return;

  NSOpenPanel* receiptChooser = [NSOpenPanel openPanel];
  receiptChooser.title = @"Choose Flight-Validation Receipt";
  receiptChooser.message =
      @"Choose the gate-passed receipt produced by Run Flight-Test Validation. Its campaign "
       "digest must match the campaign manifest above.";
  receiptChooser.canChooseFiles = YES;
  receiptChooser.canChooseDirectories = NO;
  receiptChooser.allowsMultipleSelection = NO;
  if ([receiptChooser runModal] != NSModalResponseOK)
    return;

  NSOpenPanel* deploymentChooser = [NSOpenPanel openPanel];
  deploymentChooser.title = @"Choose Onboard Deployment Manifest";
  deploymentChooser.message = @"Choose the exact deployment manifest bound to the target evidence.";
  deploymentChooser.canChooseFiles = YES;
  deploymentChooser.canChooseDirectories = NO;
  deploymentChooser.allowsMultipleSelection = NO;
  if ([deploymentChooser runModal] != NSModalResponseOK)
    return;

  NSOpenPanel* deploymentDirectoryChooser = [NSOpenPanel openPanel];
  deploymentDirectoryChooser.title = @"Choose Verified Onboard Runtime Package";
  deploymentDirectoryChooser.message =
      @"Choose the complete staged runtime directory containing deployment.receipt and bin/galata.";
  deploymentDirectoryChooser.canChooseFiles = NO;
  deploymentDirectoryChooser.canChooseDirectories = YES;
  deploymentDirectoryChooser.allowsMultipleSelection = NO;
  if ([deploymentDirectoryChooser runModal] != NSModalResponseOK)
    return;

  NSOpenPanel* targetChooser = [NSOpenPanel openPanel];
  targetChooser.title = @"Choose Target-Evidence Manifest";
  targetChooser.message =
      @"Choose the target evidence package manifest. Its bytes must be linked by the dossier.";
  targetChooser.canChooseFiles = YES;
  targetChooser.canChooseDirectories = NO;
  targetChooser.allowsMultipleSelection = NO;
  if ([targetChooser runModal] != NSModalResponseOK)
    return;

  NSArray* arguments = @[
    @"qualification",
    @"verify-chain",
    dossierChooser.URL.path,
    @"--flighttest",
    flightChooser.URL.path,
    @"--flight-validation-receipt",
    receiptChooser.URL.path,
    @"--deployment",
    deploymentChooser.URL.path,
    @"--deployment-dir",
    deploymentDirectoryChooser.URL.path,
    @"--target-evidence",
    targetChooser.URL.path
  ];
  [self command:arguments
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.qualification-chain-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"dossier_evidence_verified"] boolValue]
            || ![result[@"flight_test_evidence_verified"] boolValue]
            || ![result[@"target_evidence_verified"] boolValue]
            || ![result[@"deployment_runtime_verified"] boolValue]
            || ![result[@"traceability_links_verified"] boolValue]) {
          [self problem:error ? error : @"Engine refused to verify the qualification chain."];
          return;
        }
        NSArray* eligibilityReasons =
            [result[@"qualification_eligibility_reasons"] isKindOfClass:[NSArray class]]
                ? result[@"qualification_eligibility_reasons"]
                : @[];
        NSString* eligibility =
            [result[@"qualification_eligibility"] isKindOfClass:[NSString class]]
                ? result[@"qualification_eligibility"]
                : @"not_ready";
        NSString* reasons = eligibilityReasons.count
                                ? [eligibilityReasons componentsJoinedByString:@"; "]
                                : @"none reported";
        NSString* details = [NSString
            stringWithFormat:
                @"Dossier: %@\nFlight-test manifest SHA-256: %@\nValidation receipt SHA-256: %@\n"
                 "Target-evidence manifest SHA-256: %@\nDeployment manifest SHA-256: %@\nRuntime "
                 "SHA-256: %@\nAircraft: %@\nConfiguration: "
                @"%@\nQualification eligibility: %@\nEligibility reasons: %@\n\nThe dossier, "
                @"complete flight campaign, target evidence and deployment binding are "
                @"traceability-verified. The chain remains not_qualified and makes no "
                @"airworthiness or certification claim.",
                dossierChooser.URL.path,
                result[@"flight_test_manifest_sha256"],
                result[@"flight_validation_receipt_sha256"],
                result[@"target_evidence_manifest_sha256"],
                result[@"deployment_manifest_sha256"],
                result[@"deployment_runtime_sha256"],
                result[@"aircraft_id"],
                result[@"aircraft_configuration"],
                eligibility,
                reasons];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Qualification chain verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Qualification chain verified · %@ · %@",
                                       eligibility,
                                       dossierChooser.URL.path];
      }];
}

- (void)createQualification:(id)sender {
  (void)sender;
  if (self.task)
    return;

  NSString* productID =
      [self promptText:@"Product Identifier"
               message:@"Use the controlled product identifier from the release system."
           placeholder:@"galata"];
  if (!productID)
    return;
  NSString* productVersion =
      [self promptText:@"Product Version"
               message:@"Enter the exact software release version under review."
           placeholder:[NSString stringWithUTF8String:GALATA_VERSION_STRING]];
  if (!productVersion)
    return;
  NSString* intendedUse = [self promptText:@"Intended Use"
                                   message:@"Describe the bounded intended use for this dossier."
                               placeholder:@"controlled engineering review"];
  if (!intendedUse)
    return;
  NSString* aircraftID = [self promptText:@"Aircraft Identifier"
                                  message:@"Use the controlled airframe identifier."
                              placeholder:@"airframe-01"];
  if (!aircraftID)
    return;
  NSString* configuration =
      [self promptText:@"Aircraft Configuration"
               message:@"Use the exact aircraft configuration or baseline identifier."
           placeholder:@"configuration-2026-09"];
  if (!configuration)
    return;
  NSString* basis = [self promptText:@"Qualification Basis"
                             message:@"Enter the governing application-specific review basis."
                         placeholder:@"application-specific review basis"];
  if (!basis)
    return;
  NSString* authorityID =
      [self promptText:@"Authority Identifier"
               message:@"Identify the external authority that owns the decision."
           placeholder:@"external-authority-pending"];
  if (!authorityID)
    return;

  const NSArray<NSString*>* roles = @[
    @"requirements_matrix",
    @"software_release",
    @"verification_report",
    @"flight_test_campaign",
    @"hardware_hil_report",
    @"safety_case",
    @"independent_review",
    @"authority_decision",
    @"maintenance_plan"
  ];
  NSMutableArray* arguments = [NSMutableArray arrayWithObjects:@"qualification", @"create", nil];
  [arguments addObjectsFromArray:@[
    @"--product-id",
    productID,
    @"--product-version",
    productVersion,
    @"--intended-use",
    intendedUse,
    @"--aircraft-id",
    aircraftID,
    @"--configuration",
    configuration,
    @"--qualification-basis",
    basis,
    @"--authority-id",
    authorityID
  ]];
  for (NSString* role in roles) {
    NSOpenPanel* chooser = [NSOpenPanel openPanel];
    chooser.title = [NSString stringWithFormat:@"Choose %@ Evidence", role];
    chooser.message =
        @"Choose the controlled regular file. The engine will copy it and hash the copied bytes.";
    chooser.canChooseFiles = YES;
    chooser.canChooseDirectories = NO;
    chooser.allowsMultipleSelection = NO;
    if ([chooser runModal] != NSModalResponseOK)
      return;
    [arguments addObject:@"--file"];
    [arguments addObject:[NSString stringWithFormat:@"%@=%@", role, chooser.URL.path]];
  }

  NSSavePanel* destinationChooser = [NSSavePanel savePanel];
  destinationChooser.title = @"Choose New Qualification Dossier Directory";
  destinationChooser.message =
      @"Choose a new directory. Existing destinations are refused by the engine.";
  destinationChooser.nameFieldStringValue = @"qualification-dossier";
  destinationChooser.canCreateDirectories = YES;
  if ([destinationChooser runModal] != NSModalResponseOK)
    return;
  [arguments insertObject:destinationChooser.URL.path atIndex:2];

  [self command:arguments
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.qualification-evidence-package.v1"]
            || ![result[@"status"] isEqual:@"staged"]
            || ![result[@"evidence_references_verified"] boolValue]) {
          [self problem:error ? error : @"Engine refused to create the qualification dossier."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Directory: %@\nManifest: %@\nManifest SHA-256: %@\nFiles: %@\n\n"
                @"The evidence files were copied, hashed and verified atomically. This proves "
                @"package completeness only; it does not establish independent acceptance, "
                @"airworthiness, certification or qualification.",
                result[@"destination"],
                result[@"manifest"],
                result[@"manifest_sha256"],
                result[@"file_count"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Qualification dossier created";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue = [NSString
            stringWithFormat:@"Qualification dossier created · %@", result[@"destination"]];
        [[NSWorkspace sharedWorkspace]
            activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:result[@"destination"]]]];
      }];
}

- (void)stageOnboard:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* manifestChooser = [NSOpenPanel openPanel];
  manifestChooser.title = @"Choose Onboard Manifest";
  manifestChooser.message =
      @"The manifest is verified first. You will then choose one regular file for each declared "
      @"artifact role.";
  manifestChooser.canChooseFiles = YES;
  manifestChooser.canChooseDirectories = NO;
  manifestChooser.allowsMultipleSelection = NO;
  if ([manifestChooser runModal] != NSModalResponseOK)
    return;
  NSString* manifest = manifestChooser.URL.path;
  [self command:@[@"onboard", @"verify", manifest]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"artifact_roles"] isKindOfClass:[NSArray class]]) {
          [self problem:error ? error : @"Engine refused to verify the onboard manifest."];
          return;
        }
        NSArray* roles = result[@"artifact_roles"];
        NSDictionary* profile = [result[@"transport_profile"] isKindOfClass:[NSDictionary class]]
                                    ? result[@"transport_profile"]
                                    : @{};
        NSSavePanel* destinationChooser = [NSSavePanel savePanel];
        destinationChooser.title = @"Choose New Onboard Staging Directory";
        destinationChooser.message =
            @"Choose a new directory. Existing destinations are refused by the engine.";
        destinationChooser.nameFieldStringValue = @"onboard-staged";
        destinationChooser.canCreateDirectories = YES;
        if ([destinationChooser runModal] != NSModalResponseOK)
          return;
        NSString* destination = destinationChooser.URL.path;
        NSMutableArray* arguments =
            [NSMutableArray arrayWithObjects:@"onboard", @"stage", manifest, destination, nil];
        for (id roleValue in roles) {
          if (![roleValue isKindOfClass:[NSString class]] || [roleValue length] == 0) {
            [self problem:@"The verified manifest returned an invalid artifact role."];
            return;
          }
          NSString* role = roleValue;
          NSOpenPanel* artifactChooser = [NSOpenPanel openPanel];
          artifactChooser.title = [NSString stringWithFormat:@"Choose %@ Artifact", role];
          artifactChooser.message =
              @"Choose the exact regular file whose SHA-256 is declared by the manifest.";
          artifactChooser.canChooseFiles = YES;
          artifactChooser.canChooseDirectories = NO;
          artifactChooser.allowsMultipleSelection = NO;
          if ([artifactChooser runModal] != NSModalResponseOK)
            return;
          [arguments addObject:@"--artifact"];
          [arguments
              addObject:[NSString stringWithFormat:@"%@=%@", role, artifactChooser.URL.path]];
        }
        [self command:arguments
            completion:^(NSDictionary* staged, NSString* stageError) {
              if (![staged[@"schema"] isEqual:@"galata.onboard-stage.v1"]
                  || ![staged[@"status"] isEqual:@"staged"]) {
                [self problem:stageError ? stageError
                                         : @"Engine refused to stage the onboard package."];
                return;
              }
              NSString* details = [NSString
                  stringWithFormat:
                      @"Destination: %@\nManifest SHA-256: %@\nArtifacts staged: %@\nController "
                      @"budget: %@ s\nTransport: %@ · endpoint %@ · watchdog %@ s\n\nThe "
                      @"directory is atomically published, manifest-only and not qualified. It "
                      @"is not an installer or a flight release.",
                      staged[@"destination"],
                      staged[@"manifest_sha256"],
                      staged[@"artifact_count"],
                      staged[@"max_controller_time_s"],
                      profile[@"transport"],
                      profile[@"endpoint"],
                      profile[@"watchdog_timeout_s"]];
              NSAlert* alert = [[NSAlert alloc] init];
              alert.messageText = @"Onboard package staged";
              alert.informativeText = details;
              alert.alertStyle = NSAlertStyleInformational;
              [alert addButtonWithTitle:@"OK"];
              [alert runModal];
              self.status.stringValue = [NSString
                  stringWithFormat:@"Onboard package staged · %@", staged[@"destination"]];
              [[NSWorkspace sharedWorkspace]
                  activateFileViewerSelectingURLs:@[[NSURL
                                                      fileURLWithPath:staged[@"destination"]]]];
            }];
      }];
}

- (void)verifyOnboardDeployment:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* chooser = [NSOpenPanel openPanel];
  chooser.title = @"Verify Onboard Deployment";
  chooser.message =
      @"Choose an atomically published runtime directory. Galata checks its manifest, receipt, "
      @"executable and model/controller hashes.";
  chooser.canChooseFiles = NO;
  chooser.canChooseDirectories = YES;
  chooser.allowsMultipleSelection = NO;
  if ([chooser runModal] != NSModalResponseOK)
    return;
  NSString* directory = chooser.URL.path;
  [self command:@[@"onboard", @"verify-deployment", directory]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-deployment-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"contains_executable"] boolValue]) {
          [self problem:error ? error : @"Engine refused to verify the onboard deployment."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Directory: %@\nManifest SHA-256: %@\nRuntime SHA-256: %@\nArtifacts: %@\n\n"
                @"The POSIX runtime bundle is structurally complete and hash-consistent. It "
                @"remains not_qualified: target timing, watchdog behavior, signing, HIL, "
                @"airworthiness and authorized deployment approval are separate gates.",
                result[@"destination"],
                result[@"manifest_sha256"],
                result[@"runtime_sha256"],
                result[@"artifact_count"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Onboard deployment verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Onboard deployment verified · %@", directory];
      }];
}

- (void)onboardSelfTest:(id)sender {
  (void)sender;
  if (self.task)
    return;
  [self command:@[@"onboard", @"self-test"]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-self-test.v1"]
            || ![result[@"status"] isEqual:@"passed"]) {
          [self problem:error ? error : @"Engine refused to complete the onboard SIL self-test."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Transport: %@\nCycles: %@\nQualification state: %@\n\nThe guarded replay "
                @"supervisor passed on this host. This is not target timing, hardware-in-the-"
                @"loop, flight-controller compatibility or qualification evidence.",
                result[@"transport"],
                result[@"completed_cycles"],
                result[@"qualification_state"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Onboard SIL self-test passed";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue = @"Onboard SIL self-test passed · not qualified";
      }];
}

- (void)createOnboardTargetEvidence:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* deploymentChooser = [NSOpenPanel openPanel];
  deploymentChooser.title = @"Choose Onboard Deployment Manifest";
  deploymentChooser.message =
      @"The target evidence package will be bound to this exact deployment manifest.";
  deploymentChooser.canChooseFiles = YES;
  deploymentChooser.canChooseDirectories = NO;
  deploymentChooser.allowsMultipleSelection = NO;
  if ([deploymentChooser runModal] != NSModalResponseOK)
    return;

  NSString* controllerWorstCase =
      [self promptText:@"Measured Controller Worst Case"
               message:@"Enter the externally measured worst-case controller time in seconds."
           placeholder:@"0.001"];
  if (!controllerWorstCase)
    return;
  NSString* cycleWorstCase =
      [self promptText:@"Measured Cycle Worst Case"
               message:@"Enter the externally measured complete-cycle time in seconds."
           placeholder:@"0.004"];
  if (!cycleWorstCase)
    return;
  NSString* watchdogResponse =
      [self promptText:@"Measured Watchdog Response"
               message:@"Enter the externally measured watchdog response in seconds."
           placeholder:@"0.008"];
  if (!watchdogResponse)
    return;
  NSString* evidenceClass = [self promptText:@"Evidence Class"
                                     message:@"Enter host_sil for host replay/SIL, target_hil for "
                                             @"the intended flight computer in HIL, or "
                                             @"flight_target for the installed target."
                                 placeholder:@"target_hil"];
  if (!evidenceClass)
    return;
  const BOOL physicalTestsApplicable =
      [evidenceClass isEqual:@"target_hil"] || [evidenceClass isEqual:@"flight_target"];
  if (![evidenceClass isEqual:@"host_sil"] && !physicalTestsApplicable) {
    [self problem:@"Evidence class must be host_sil, target_hil or flight_target."];
    return;
  }

  if (physicalTestsApplicable) {
    NSAlert* confirmation = [[NSAlert alloc] init];
    confirmation.messageText = @"Confirm external target results";
    confirmation.informativeText =
        @"Confirm only if the responsible integration programme has already produced passing "
         "emergency-stop, loss-of-link, HIL and signing records. Galata will copy and hash those "
         "records; it will not perform or certify the tests.";
    [confirmation addButtonWithTitle:@"Continue"];
    [confirmation addButtonWithTitle:@"Cancel"];
    if ([confirmation runModal] != NSAlertFirstButtonReturn)
      return;
  }

  const NSArray<NSString*>* roles = @[
    @"timing_report",
    @"hardware_hil_report",
    @"failsafe_report",
    @"signing_record",
    @"target_configuration"
  ];
  NSMutableArray* arguments = [NSMutableArray arrayWithObjects:@"onboard",
                                                               @"target",
                                                               @"create",
                                                               @"",
                                                               deploymentChooser.URL.path,
                                                               @"--evidence-class",
                                                               evidenceClass,
                                                               @"--controller-worst-case-s",
                                                               controllerWorstCase,
                                                               @"--cycle-worst-case-s",
                                                               cycleWorstCase,
                                                               @"--watchdog-response-s",
                                                               watchdogResponse,
                                                               nil];
  if (physicalTestsApplicable) {
    [arguments addObjectsFromArray:@[
      @"--emergency-stop-passed",
      @"true",
      @"--loss-of-link-passed",
      @"true",
      @"--hil-passed",
      @"true",
      @"--signing-verified",
      @"true"
    ]];
  }
  for (NSString* role in roles) {
    NSOpenPanel* evidenceChooser = [NSOpenPanel openPanel];
    evidenceChooser.title = [NSString stringWithFormat:@"Choose %@", role];
    evidenceChooser.message =
        @"Choose the externally produced regular evidence file. It will be copied and hashed.";
    evidenceChooser.canChooseFiles = YES;
    evidenceChooser.canChooseDirectories = NO;
    evidenceChooser.allowsMultipleSelection = NO;
    if ([evidenceChooser runModal] != NSModalResponseOK)
      return;
    [arguments addObject:@"--file"];
    [arguments addObject:[NSString stringWithFormat:@"%@=%@", role, evidenceChooser.URL.path]];
  }

  NSSavePanel* destinationChooser = [NSSavePanel savePanel];
  destinationChooser.title = @"Choose New Target Evidence Directory";
  destinationChooser.message =
      @"Choose a new directory. Existing destinations are refused atomically.";
  destinationChooser.nameFieldStringValue = @"target-evidence";
  destinationChooser.canCreateDirectories = YES;
  if ([destinationChooser runModal] != NSModalResponseOK)
    return;
  arguments[3] = destinationChooser.URL.path;

  [self command:arguments
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-target-evidence-package.v2"]
            || ![result[@"status"] isEqual:@"staged"]
            || ![result[@"evidence_references_verified"] boolValue]) {
          [self problem:error ? error : @"Engine refused to create target evidence package."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Directory: %@\nManifest: %@\nManifest SHA-256: %@\nEvidence files: %@\n"
                @"Evidence class: %@\nBytes: %@\n\nThe records were copied, hashed and "
                @"contract-checked. This "
                @"package remains not_qualified; it does not create or certify the underlying "
                @"hardware results.",
                result[@"destination"],
                result[@"manifest"],
                result[@"manifest_sha256"],
                result[@"file_count"],
                result[@"evidence_class"],
                result[@"total_bytes"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Target evidence package created";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue = @"Target evidence package created · not qualified";
        [[NSWorkspace sharedWorkspace]
            activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:result[@"destination"]]]];
      }];
}

- (void)verifyOnboardTargetEvidence:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* deploymentChooser = [NSOpenPanel openPanel];
  deploymentChooser.title = @"Choose Onboard Deployment Manifest";
  deploymentChooser.message =
      @"Choose the exact onboard.manifest that the target-integration evidence tested.";
  deploymentChooser.canChooseFiles = YES;
  deploymentChooser.canChooseDirectories = NO;
  deploymentChooser.allowsMultipleSelection = NO;
  if ([deploymentChooser runModal] != NSModalResponseOK)
    return;
  NSOpenPanel* evidenceChooser = [NSOpenPanel openPanel];
  evidenceChooser.title = @"Choose Target Evidence Manifest";
  evidenceChooser.message =
      @"Choose target-evidence.manifest and keep its evidence directory beside it.";
  evidenceChooser.canChooseFiles = YES;
  evidenceChooser.canChooseDirectories = NO;
  evidenceChooser.allowsMultipleSelection = NO;
  if ([evidenceChooser runModal] != NSModalResponseOK)
    return;
  NSString* deployment = deploymentChooser.URL.path;
  NSString* evidence = evidenceChooser.URL.path;
  [self command:@[@"onboard", @"target", @"verify", deployment, evidence]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-target-evidence-verification.v2"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"target_acceptance_state"] isEqual:@"passed"]) {
          [self problem:error ? error : @"Engine refused to verify target-integration evidence."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Target: %@\nFlight computer: %@\nFirmware: %@\nController worst case: %@ s\n"
                @"Cycle worst case: %@ s\nWatchdog response: %@ s\nEvidence class: %@\nEvidence "
                @"files: %@\n\n"
                @"The supplied records are byte-verified and bound to the deployment contract. "
                @"The package remains not_qualified and does not establish certification or "
                @"airworthiness.",
                result[@"target_hardware_id"],
                result[@"flight_computer_id"],
                result[@"firmware_id"],
                result[@"controller_worst_case_s"],
                result[@"cycle_worst_case_s"],
                result[@"watchdog_response_s"],
                result[@"evidence_class"],
                result[@"file_count"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Target evidence verified";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue = @"Target evidence verified · not qualified";
      }];
}

- (void)deployOnboard:(id)sender {
  (void)sender;
  if (self.task)
    return;
  NSOpenPanel* manifestChooser = [NSOpenPanel openPanel];
  manifestChooser.title = @"Choose Onboard Manifest";
  manifestChooser.message =
      @"The manifest is verified first. You will then select the executable runtime and one "
      @"regular source file for every declared artifact role.";
  manifestChooser.canChooseFiles = YES;
  manifestChooser.canChooseDirectories = NO;
  manifestChooser.allowsMultipleSelection = NO;
  if ([manifestChooser runModal] != NSModalResponseOK)
    return;
  NSString* manifest = manifestChooser.URL.path;
  [self command:@[@"onboard", @"verify", manifest]
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.onboard-verification.v1"]
            || ![result[@"status"] isEqual:@"verified"]
            || ![result[@"artifact_roles"] isKindOfClass:[NSArray class]]) {
          [self problem:error ? error : @"Engine refused to verify the onboard manifest."];
          return;
        }

        NSOpenPanel* runtimeChooser = [NSOpenPanel openPanel];
        runtimeChooser.title = @"Choose Onboard Runtime Executable";
        runtimeChooser.message = [NSString
            stringWithFormat:
                @"Choose an executable POSIX runner. The packaged desktop engine is %@; use a "
                @"target-specific build only after its own review.",
                self.engine];
        runtimeChooser.canChooseFiles = YES;
        runtimeChooser.canChooseDirectories = NO;
        runtimeChooser.allowsMultipleSelection = NO;
        if ([runtimeChooser runModal] != NSModalResponseOK)
          return;
        NSString* runtime = runtimeChooser.URL.path;
        if (![[NSFileManager defaultManager] isExecutableFileAtPath:runtime]) {
          [self problem:@"The selected runtime is not executable."];
          return;
        }

        NSSavePanel* destinationChooser = [NSSavePanel savePanel];
        destinationChooser.title = @"Choose New Onboard Runtime Directory";
        destinationChooser.message =
            @"Choose a new directory. Existing destinations are refused by the engine.";
        destinationChooser.nameFieldStringValue = @"onboard-runtime";
        destinationChooser.canCreateDirectories = YES;
        if ([destinationChooser runModal] != NSModalResponseOK)
          return;
        NSString* destination = destinationChooser.URL.path;

        NSArray* roles = result[@"artifact_roles"];
        NSMutableArray* arguments = [NSMutableArray arrayWithObjects:@"onboard",
                                                                     @"deploy",
                                                                     manifest,
                                                                     destination,
                                                                     @"--runtime",
                                                                     runtime,
                                                                     nil];
        for (id roleValue in roles) {
          if (![roleValue isKindOfClass:[NSString class]] || [roleValue length] == 0) {
            [self problem:@"The verified manifest returned an invalid artifact role."];
            return;
          }
          NSString* role = roleValue;
          NSOpenPanel* artifactChooser = [NSOpenPanel openPanel];
          artifactChooser.title = [NSString stringWithFormat:@"Choose %@ Artifact", role];
          artifactChooser.message =
              @"Choose the exact regular file whose SHA-256 is declared by the manifest.";
          artifactChooser.canChooseFiles = YES;
          artifactChooser.canChooseDirectories = NO;
          artifactChooser.allowsMultipleSelection = NO;
          if ([artifactChooser runModal] != NSModalResponseOK)
            return;
          [arguments addObject:@"--artifact"];
          [arguments
              addObject:[NSString stringWithFormat:@"%@=%@", role, artifactChooser.URL.path]];
        }

        [self command:arguments
            completion:^(NSDictionary* deployed, NSString* deployError) {
              if (![deployed[@"schema"] isEqual:@"galata.onboard-deployment.v1"]
                  || ![deployed[@"status"] isEqual:@"staged"]
                  || ![deployed[@"contains_executable"] boolValue]) {
                [self problem:deployError ? deployError
                                          : @"Engine refused to deploy the onboard runtime."];
                return;
              }
              NSString* details = [NSString
                  stringWithFormat:
                      @"Directory: %@\nManifest SHA-256: %@\nRuntime SHA-256: %@\nArtifacts: %@\n\n"
                      @"The executable POSIX runtime bundle was published atomically and can be "
                      @"verified or transferred for bench integration. It remains "
                      @"not_qualified and is not a flight release.",
                      deployed[@"destination"],
                      deployed[@"manifest_sha256"],
                      deployed[@"runtime_sha256"],
                      deployed[@"artifact_count"]];
              NSAlert* alert = [[NSAlert alloc] init];
              alert.messageText = @"Onboard runtime deployed";
              alert.informativeText = details;
              alert.alertStyle = NSAlertStyleInformational;
              [alert addButtonWithTitle:@"OK"];
              [alert runModal];
              self.status.stringValue = [NSString
                  stringWithFormat:@"Onboard runtime deployed · %@", deployed[@"destination"]];
              [[NSWorkspace sharedWorkspace]
                  activateFileViewerSelectingURLs:@[[NSURL
                                                      fileURLWithPath:deployed[@"destination"]]]];
            }];
      }];
}

- (void)createFlightTest:(id)sender {
  (void)sender;
  if (self.task)
    return;

  NSString* aircraftID = [self promptText:@"Aircraft Identifier"
                                  message:@"Use the controlled airframe identifier from the "
                                          @"configuration system."
                              placeholder:@"airframe-01"];
  if (!aircraftID)
    return;
  NSString* configuration = [self promptText:@"Aircraft Configuration"
                                     message:@"Use the exact configuration/baseline identifier "
                                             @"for this campaign."
                                 placeholder:@"configuration-2026-09"];
  if (!configuration)
    return;
  NSString* evidenceClass = [self promptText:@"Evidence Class"
                                     message:@"Enter measured_flight for controlled aircraft data, "
                                             @"public_deidentified for public data, or "
                                             @"synthetic_contract for software-only data."
                                 placeholder:@"measured_flight"];
  if (!evidenceClass)
    return;
  NSString* testPlan = [self promptText:@"Test Plan Identifier"
                                message:@"Enter the approved flight-test plan identifier."
                            placeholder:@"FT-001"];
  if (!testPlan)
    return;
  NSString* reviewer = [self promptText:@"Reviewer Identifier"
                                message:@"Enter the independent reviewer or organization "
                                        @"identifier."
                            placeholder:@"independent-reviewer"];
  if (!reviewer)
    return;

  NSAlert* safety = [[NSAlert alloc] init];
  safety.messageText = @"Confirm safety review";
  safety.informativeText =
      @"Continue only if the required safety review for this campaign is complete. "
      @"Galata records this explicit confirmation in the package manifest.";
  safety.alertStyle = NSAlertStyleWarning;
  [safety addButtonWithTitle:@"Safety Review Complete"];
  [safety addButtonWithTitle:@"Cancel"];
  if ([safety runModal] != NSAlertFirstButtonReturn)
    return;

  const NSArray<NSString*>* roles = @[
    @"test_plan",
    @"flight_record",
    @"calibration_manifest",
    @"configuration_manifest",
    @"reviewer_attestation"
  ];
  NSMutableArray* arguments = [NSMutableArray arrayWithObjects:@"flighttest", @"create", nil];
  [arguments addObjectsFromArray:@[
    @"--aircraft-id",
    aircraftID,
    @"--configuration",
    configuration,
    @"--evidence-class",
    evidenceClass,
    @"--test-plan-id",
    testPlan,
    @"--reviewer-id",
    reviewer,
    @"--safety-review-complete",
    @"true"
  ]];

  for (NSString* role in roles) {
    NSOpenPanel* chooser = [NSOpenPanel openPanel];
    chooser.title = [NSString stringWithFormat:@"Choose %@ Evidence", role];
    chooser.message =
        @"Choose the controlled regular file. The engine will copy it and hash the copied bytes.";
    chooser.canChooseFiles = YES;
    chooser.canChooseDirectories = NO;
    chooser.allowsMultipleSelection = NO;
    if ([chooser runModal] != NSModalResponseOK)
      return;
    [arguments addObject:@"--file"];
    [arguments addObject:[NSString stringWithFormat:@"%@=%@", role, chooser.URL.path]];
  }

  NSSavePanel* destinationChooser = [NSSavePanel savePanel];
  destinationChooser.title = @"Choose New Flight-Test Package Directory";
  destinationChooser.message =
      @"Choose a new directory. Existing destinations are refused by the engine.";
  destinationChooser.nameFieldStringValue = @"flight-test-package";
  destinationChooser.canCreateDirectories = YES;
  if ([destinationChooser runModal] != NSModalResponseOK)
    return;
  [arguments insertObject:destinationChooser.URL.path atIndex:2];

  [self command:arguments
      completion:^(NSDictionary* result, NSString* error) {
        if (![result[@"schema"] isEqual:@"galata.flight-test-evidence-package.v2"]
            || ![result[@"status"] isEqual:@"staged"]
            || ![result[@"evidence_references_verified"] boolValue]) {
          [self problem:error ? error : @"Engine refused to create the flight-test package."];
          return;
        }
        NSString* details = [NSString
            stringWithFormat:
                @"Directory: %@\nManifest: %@\nManifest SHA-256: %@\nFiles: %@\n\n"
                @"The evidence files were copied, hashed and verified atomically. This proves "
                @"package completeness only; it does not establish flight validity, "
                @"airworthiness, certification or qualification.",
                result[@"destination"],
                result[@"manifest"],
                result[@"manifest_sha256"],
                result[@"file_count"]];
        NSAlert* alert = [[NSAlert alloc] init];
        alert.messageText = @"Flight-test package created";
        alert.informativeText = details;
        alert.alertStyle = NSAlertStyleInformational;
        [alert addButtonWithTitle:@"OK"];
        [alert runModal];
        self.status.stringValue =
            [NSString stringWithFormat:@"Flight-test package created · %@", result[@"destination"]];
        [[NSWorkspace sharedWorkspace]
            activateFileViewerSelectingURLs:@[[NSURL fileURLWithPath:result[@"destination"]]]];
      }];
}

- (void)cancelRun:(id)sender {
  (void)sender;
  if (self.task
      && ([self.task.arguments containsObject:@"run"] ||
          [self.task.arguments containsObject:@"import-linear"])) {
    BOOL importing = [self.task.arguments containsObject:@"import-linear"];
    self.cancelling = YES;
    [self.task terminate];
    self.status.stringValue =
        importing
            ? @"Import cancellation requested. Waiting for engine cleanup…"
            : @"Cancellation requested. Waiting for engine exit and interrupted-run recovery…";
    [self refresh];
  }
}

- (NSString*)readArtifact:(NSString*)path limit:(NSUInteger)limit {
  if (![path isKindOfClass:[NSString class]] || !path.length)
    return nil;
  NSDictionary* attrs = [[NSFileManager defaultManager] attributesOfItemAtPath:path error:nil];
  if (!attrs || [attrs fileSize] > limit)
    return nil;
  return [NSString stringWithContentsOfFile:path encoding:NSUTF8StringEncoding error:nil];
}

- (void)selectRun:(id)sender {
  (void)sender;
  NSDictionary* run = self.history.selectedItem.representedObject;
  if (!run)
    return;
  self.selectedRunID = run[@"id"];
  NSString* rawEvidence = [self readArtifact:run[@"evidence_json"] limit:2 * 1024 * 1024];
  NSString* rawManifest = [self readArtifact:run[@"manifest_path"] limit:2 * 1024 * 1024];
  self.evidence.string = [NSString
      stringWithFormat:
          @"SELECTED RUN — EXECUTION EVIDENCE\n"
          @"Completed execution does not establish numerical acceptance.\n%@\n%@\n\n%@\n\n%@\n\n%@",
          self.projectView[@"origin"]
              ? [NSString stringWithFormat:@"This run's relation to ORIGINAL study: %@",
                                           OriginRelation(run[@"origin_relation"])]
              : @"",
          self.projectView[@"origin"]
              ? @"Original source context and manifest are available separately in Original Study."
              : @"",
          JSONText(run),
          rawEvidence ? rawEvidence
                      : @"Evidence unavailable or larger than the 2 MiB preview limit.",
          rawManifest ? rawManifest : @""];
  NSString* csv = [self readArtifact:run[@"trajectory_csv"] limit:16 * 1024 * 1024];
  NSMutableArray* samples = [NSMutableArray array];
  NSArray* lines = [csv componentsSeparatedByCharactersInSet:[NSCharacterSet newlineCharacterSet]];
  NSArray* columns = lines.count ? [lines[0] componentsSeparatedByString:@","] : @[];
  for (NSUInteger i = 1; i < lines.count; ++i) {
    NSArray* fields = [lines[i] componentsSeparatedByString:@","];
    if (fields.count != columns.count || fields.count < 2)
      continue;
    NSMutableArray* sample = [NSMutableArray array];
    for (NSString* field in fields) {
      NSScanner* scanner = [NSScanner scannerWithString:field];
      scanner.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
      double number = 0;
      if (![scanner scanDouble:&number] || !scanner.isAtEnd || !std::isfinite(number))
        break;
      [sample addObject:@(number)];
    }
    if (sample.count == fields.count)
      [samples addObject:sample];
  }
  [self.samplesTable setHeaders:columns rows:samples];
  NSMutableArray* displayed = [NSMutableArray array];
  NSUInteger stride = MAX((NSUInteger)1, (samples.count + 11998) / 11999);
  for (NSUInteger i = 0; i < samples.count; i += stride)
    [displayed addObject:samples[i]];
  if (samples.count && displayed.lastObject != samples.lastObject)
    [displayed addObject:samples.lastObject];
  self.plot.samples = displayed;
  self.plot.columns = columns;
  self.sampleSummary.stringValue = samples.count
                                       ? [NSString stringWithFormat:@"%lu rows · %lu plotted",
                                                                    (unsigned long)samples.count,
                                                                    (unsigned long)displayed.count]
                                       : @"No readable samples";
  self.sampleSummary.toolTip = @"Samples shows all finite rows in the CSV preview (up to 16 MiB). "
                               @"Plot may display fewer points; stored results remain unchanged.";
  self.plot.series = 1;
  [self.series removeAllItems];
  for (NSUInteger i = 1; i < columns.count; ++i)
    [self.series addItemWithTitle:columns[i]];
  self.plot.needsDisplay = YES;
  self.plot.accessibilityLabel =
      [NSString stringWithFormat:@"Trajectory preview, %lu plotted samples. Press Command 4 for "
                                 @"the numerical samples table.",
                                 (unsigned long)displayed.count];
}

- (void)selectSeries:(id)sender {
  (void)sender;
  self.plot.series = (NSUInteger)MAX((NSInteger)0, self.series.indexOfSelectedItem) + 1;
  self.plot.needsDisplay = YES;
}

- (BOOL)windowShouldClose:(NSWindow*)sender {
  if (sender != self.window)
    return YES;
  // Closing the sole editor is quitting, even when a read-only review window
  // remains open. Run the unsaved/active-command guard once, before it closes.
  [NSApp terminate:self];
  return NO;
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
  (void)sender;
  return YES;
}

- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication*)sender {
  (void)sender;
  [self persistRecoverySnapshot];
  if (![self canReplaceProject])
    return NSTerminateCancel;
  [self discardRecoverySnapshotForProject:self.project];
  return NSTerminateNow;
}

@end

int main(int argc, const char* argv[]) {
  (void)argc;
  (void)argv;
  @autoreleasepool {
    NSApplication* app = [NSApplication sharedApplication];
    [app setActivationPolicy:NSApplicationActivationPolicyRegular];
    GalataController* delegate = [[GalataController alloc] init];
    app.delegate = delegate;
    [app run];
  }
  return 0;
}
