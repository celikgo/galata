// SPDX-License-Identifier: Apache-2.0
#import "diagram_editing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace {

constexpr CGFloat coordinateLimit = 100000;
constexpr CGFloat endpointStub = 12;
// Display-offset round trips can place a port a few floating-point bits inside
// its card. Ignore only this subpixel UI edge band; numerical model validation,
// connection identity and saved coordinates are unaffected.
constexpr CGFloat cardEdgeEpsilon = 1e-7;
constexpr NSUInteger maximumPoints = 64;
constexpr NSUInteger maximumCards = 128;

bool bounded(CGFloat coordinate) {
  return std::isfinite(coordinate) && std::abs(coordinate) <= coordinateLimit;
}

bool validPoint(NSPoint point) {
  return bounded(point.x) && bounded(point.y);
}

bool validRect(NSRect rect) {
  return validPoint(rect.origin) && std::isfinite(rect.size.width)
         && std::isfinite(rect.size.height) && rect.size.width > 0 && rect.size.height > 0
         && bounded(NSMaxX(rect)) && bounded(NSMaxY(rect));
}

bool isPointValue(id value) {
  return
      [value isKindOfClass:[NSValue class]] && std::strcmp([value objCType], @encode(NSPoint)) == 0;
}

bool isRectValue(id value) {
  return
      [value isKindOfClass:[NSValue class]] && std::strcmp([value objCType], @encode(NSRect)) == 0;
}

bool crossesInterior(NSPoint a, NSPoint b, NSRect rect) {
  if (a.y == b.y)
    return a.y > NSMinY(rect) + cardEdgeEpsilon && a.y < NSMaxY(rect) - cardEdgeEpsilon
           && std::max(a.x, b.x) > NSMinX(rect) + cardEdgeEpsilon
           && std::min(a.x, b.x) < NSMaxX(rect) - cardEdgeEpsilon;
  return a.x > NSMinX(rect) + cardEdgeEpsilon && a.x < NSMaxX(rect) - cardEdgeEpsilon
         && std::max(a.y, b.y) > NSMinY(rect) + cardEdgeEpsilon
         && std::min(a.y, b.y) < NSMaxY(rect) - cardEdgeEpsilon;
}

NSArray<NSValue*>* simplified(const std::vector<NSPoint>& points) {
  std::vector<NSPoint> result;
  for (NSPoint point : points) {
    if (!result.empty() && NSEqualPoints(result.back(), point))
      continue;
    while (result.size() >= 2) {
      NSPoint a = result[result.size() - 2];
      NSPoint b = result.back();
      // Only remove a point between its neighbours. Reversals are not equivalent
      // to a straight segment and must not erase a terminal shaft or excursion.
      bool between = (a.x == b.x && b.x == point.x && b.y >= std::min(a.y, point.y)
                      && b.y <= std::max(a.y, point.y))
                     || (a.y == b.y && b.y == point.y && b.x >= std::min(a.x, point.x)
                         && b.x <= std::max(a.x, point.x));
      if (!between)
        break;
      result.pop_back();
    }
    result.push_back(point);
  }
  NSMutableArray<NSValue*>* values = [NSMutableArray arrayWithCapacity:result.size()];
  for (NSPoint point : result)
    [values addObject:[NSValue valueWithPoint:point]];
  return values;
}

}  // namespace

BOOL GalataWirePathClear(NSArray<NSValue*>* points, NSArray<NSValue*>* blockRects) {
  if (![points isKindOfClass:[NSArray class]] || points.count < 2 || points.count > maximumPoints
      || ![blockRects isKindOfClass:[NSArray class]] || blockRects.count > maximumCards)
    return NO;
  for (id value in points)
    if (!isPointValue(value) || !validPoint([value pointValue]))
      return NO;
  for (id value in blockRects)
    if (!isRectValue(value) || !validRect([value rectValue]))
      return NO;

  NSPoint start = points.firstObject.pointValue;
  NSPoint next = points[1].pointValue;
  NSPoint end = points.lastObject.pointValue;
  NSPoint previous = points[points.count - 2].pointValue;
  if (next.x <= start.x || next.y != start.y || previous.x >= end.x || previous.y != end.y)
    return NO;
  for (NSUInteger i = 1; i < points.count; ++i) {
    NSPoint a = points[i - 1].pointValue;
    NSPoint b = points[i].pointValue;
    if ((a.x == b.x) == (a.y == b.y))
      return NO;
    for (NSValue* value in blockRects)
      if (crossesInterior(a, b, value.rectValue))
        return NO;
  }
  return YES;
}

NSArray<NSValue*>* GalataMoveWireSegment(NSArray<NSValue*>* points,
                                         NSUInteger segmentIndex,
                                         CGFloat perpendicularCoordinate,
                                         NSArray<NSValue*>* blockRects) {
  if (!GalataWirePathClear(points, blockRects) || segmentIndex >= points.count - 1
      || !bounded(perpendicularCoordinate))
    return nil;
  NSPoint a = points[segmentIndex].pointValue;
  NSPoint b = points[segmentIndex + 1].pointValue;
  const bool horizontal = a.y == b.y;
  if ((horizontal ? a.y : a.x) == perpendicularCoordinate)
    return points;

  std::vector<NSPoint> edited;
  edited.reserve(points.count + 4);
  if (points.count == 2) {
    edited = {a,
              NSMakePoint(a.x + endpointStub, a.y),
              NSMakePoint(a.x + endpointStub, perpendicularCoordinate),
              NSMakePoint(b.x - endpointStub, perpendicularCoordinate),
              NSMakePoint(b.x - endpointStub, b.y),
              b};
  } else if (segmentIndex == 0) {
    edited = {a,
              NSMakePoint(a.x + endpointStub, a.y),
              NSMakePoint(a.x + endpointStub, perpendicularCoordinate),
              NSMakePoint(b.x, perpendicularCoordinate)};
    for (NSUInteger i = 2; i < points.count; ++i)
      edited.push_back(points[i].pointValue);
  } else if (segmentIndex == points.count - 2) {
    for (NSUInteger i = 0; i < segmentIndex; ++i)
      edited.push_back(points[i].pointValue);
    edited.push_back(NSMakePoint(a.x, perpendicularCoordinate));
    edited.push_back(NSMakePoint(b.x - endpointStub, perpendicularCoordinate));
    edited.push_back(NSMakePoint(b.x - endpointStub, b.y));
    edited.push_back(b);
  } else {
    for (NSUInteger i = 0; i < points.count; ++i) {
      NSPoint point = points[i].pointValue;
      if (i == segmentIndex || i == segmentIndex + 1) {
        if (horizontal)
          point.y = perpendicularCoordinate;
        else
          point.x = perpendicularCoordinate;
      }
      edited.push_back(point);
    }
  }
  NSArray<NSValue*>* result = simplified(edited);
  return GalataWirePathClear(result, blockRects) ? result : nil;
}

CGFloat GalataSnapToGrid(CGFloat coordinate, CGFloat spacing) {
  if (!std::isfinite(coordinate) || !std::isfinite(spacing) || spacing <= 0)
    return coordinate;
  const CGFloat snapped = std::round(coordinate / spacing) * spacing;
  return std::isfinite(snapped) ? snapped : coordinate;
}

GalataBlockSnapResult GalataSnapBlock(NSRect proposed,
                                      NSArray<NSValue*>* peers,
                                      CGFloat magnification,
                                      BOOL alignPeers) {
  GalataBlockSnapResult result{proposed.origin, NO, NO, 0, 0};
  if (!validRect(proposed) || !std::isfinite(magnification) || magnification <= 0)
    return result;
  result.origin =
      NSMakePoint(GalataSnapToGrid(proposed.origin.x), GalataSnapToGrid(proposed.origin.y));
  if (!alignPeers || ![peers isKindOfClass:[NSArray class]])
    return result;
  const CGFloat threshold = 6 / magnification;
  CGFloat bestX = std::numeric_limits<CGFloat>::infinity();
  CGFloat bestY = std::numeric_limits<CGFloat>::infinity();
  const std::array<CGFloat, 3> proposedX{NSMinX(proposed), NSMidX(proposed), NSMaxX(proposed)};
  const std::array<CGFloat, 3> proposedY{NSMinY(proposed), NSMidY(proposed), NSMaxY(proposed)};
  for (id value in peers) {
    if (!isRectValue(value) || !validRect([value rectValue]))
      continue;
    NSRect peer = [value rectValue];
    const std::array<CGFloat, 3> peerX{NSMinX(peer), NSMidX(peer), NSMaxX(peer)};
    const std::array<CGFloat, 3> peerY{NSMinY(peer), NSMidY(peer), NSMaxY(peer)};
    for (CGFloat anchor : proposedX) {
      for (CGFloat guide : peerX) {
        const CGFloat delta = guide - anchor;
        if (std::abs(delta) <= threshold && std::abs(delta) < bestX) {
          bestX = std::abs(delta);
          result.origin.x = proposed.origin.x + delta;
          result.hasVerticalGuide = YES;
          result.verticalGuide = guide;
        }
      }
    }
    for (CGFloat anchor : proposedY) {
      for (CGFloat guide : peerY) {
        const CGFloat delta = guide - anchor;
        if (std::abs(delta) <= threshold && std::abs(delta) < bestY) {
          bestY = std::abs(delta);
          result.origin.y = proposed.origin.y + delta;
          result.hasHorizontalGuide = YES;
          result.horizontalGuide = guide;
        }
      }
    }
  }
  return result;
}
