// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_DESKTOP_DIAGRAM_EDITING_HPP
#define GALATA_DESKTOP_DIAGRAM_EDITING_HPP

#import <AppKit/AppKit.h>

// Presentation geometry only. Use the same coordinate space for points and
// cards. A path has 2..64 finite points within +/-100000, nonzero orthogonal
// segments, rightward departure/arrival and no segment inside any card beyond
// a 1e-7 presentation-only edge allowance for display-coordinate roundoff. Up to
// 128 finite, positive-sized rectangles are accepted. Card edges are allowed.
BOOL GalataWirePathClear(NSArray<NSValue*>* points, NSArray<NSValue*>* blockRects);

// Move the segment joining points[index] and points[index + 1] perpendicular to
// itself, keeping both wire endpoints fixed. Terminal segments gain 12-point
// horizontal endpoint stubs and a dogleg; a straight two-point wire is editable.
// Return nil if the resulting route is invalid or enters a card. An unchanged
// coordinate returns the original route. No argument is modified.
NSArray<NSValue*>* GalataMoveWireSegment(NSArray<NSValue*>* points,
                                         NSUInteger segmentIndex,
                                         CGFloat perpendicularCoordinate,
                                         NSArray<NSValue*>* blockRects);

// Halfway values round away from zero, including at negative coordinates.
// Invalid coordinates or nonpositive/nonfinite spacing return the coordinate.
CGFloat GalataSnapToGrid(CGFloat coordinate, CGFloat spacing = 16.0);

struct GalataBlockSnapResult {
  NSPoint origin;
  BOOL hasVerticalGuide;
  BOOL hasHorizontalGuide;
  CGFloat verticalGuide;
  CGFloat horizontalGuide;
};

// Snap a proposed origin to the 16-point grid. When alignPeers is YES, the nearest
// peer edge/center within six screen points overrides that axis's grid snap.
// Peers must exclude the dragged card and use the proposed rectangle's coordinate
// space. Stable input order breaks ties. Guides mark the matched peer coordinate.
// Invalid proposed geometry/magnification returns its origin without guides;
// invalid peers are ignored. Pass logical coordinates for a model-anchored grid.
GalataBlockSnapResult GalataSnapBlock(NSRect proposed,
                                      NSArray<NSValue*>* peers,
                                      CGFloat magnification,
                                      BOOL alignPeers);

#endif
