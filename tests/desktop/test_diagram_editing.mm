// SPDX-License-Identifier: Apache-2.0
// Presentation geometry only. Check fixed endpoints, independent orthogonality
// and card-interior tests, and screen-space snapping outcomes.
#import "diagram_editing.hpp"
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

NSValue* point(CGFloat x, CGFloat y) {
  return [NSValue valueWithPoint:NSMakePoint(x, y)];
}

NSValue* card(CGFloat x, CGFloat y, CGFloat width, CGFloat height) {
  return [NSValue valueWithRect:NSMakeRect(x, y, width, height)];
}

NSArray<NSValue*>* bentPath() {
  return @[point(0, 0), point(40, 0), point(40, 80), point(140, 80), point(140, 0), point(200, 0)];
}

NSArray<NSValue*>* terminalCards() {
  return @[card(-100, -30, 100, 60), card(200, -30, 100, 60)];
}

void expectGeometry(NSArray<NSValue*>* points,
                    NSPoint start,
                    NSPoint end,
                    NSArray<NSValue*>* cards) {
  ASSERT_NE(points, nil);
  ASSERT_GE(points.count, 2U);
  EXPECT_TRUE(NSEqualPoints(points.firstObject.pointValue, start));
  EXPECT_TRUE(NSEqualPoints(points.lastObject.pointValue, end));
  EXPECT_GT(points[1].pointValue.x, start.x);
  EXPECT_EQ(points[1].pointValue.y, start.y);
  EXPECT_LT(points[points.count - 2].pointValue.x, end.x);
  EXPECT_EQ(points[points.count - 2].pointValue.y, end.y);
  for (NSUInteger i = 1; i < points.count; ++i) {
    SCOPED_TRACE(::testing::Message() << "segment " << i);
    NSPoint a = points[i - 1].pointValue;
    NSPoint b = points[i].pointValue;
    EXPECT_TRUE(std::isfinite(a.x) && std::isfinite(a.y));
    EXPECT_TRUE(std::isfinite(b.x) && std::isfinite(b.y));
    ASSERT_NE(a.x == b.x, a.y == b.y);
    for (NSValue* value in cards) {
      NSRect rect = value.rectValue;
      // Test open intervals directly, independently of the editor's validator.
      if (a.x == b.x && a.x > NSMinX(rect) && a.x < NSMaxX(rect))
        EXPECT_TRUE(std::max(a.y, b.y) <= NSMinY(rect) || std::min(a.y, b.y) >= NSMaxY(rect));
      if (a.y == b.y && a.y > NSMinY(rect) && a.y < NSMaxY(rect))
        EXPECT_TRUE(std::max(a.x, b.x) <= NSMinX(rect) || std::min(a.x, b.x) >= NSMaxX(rect));
    }
  }
}

bool containsSegmentAt(NSArray<NSValue*>* points, bool horizontal, CGFloat coordinate) {
  for (NSUInteger i = 1; i < points.count; ++i) {
    NSPoint a = points[i - 1].pointValue;
    NSPoint b = points[i].pointValue;
    if (horizontal ? (a.y == coordinate && b.y == coordinate && a.x != b.x)
                   : (a.x == coordinate && b.x == coordinate && a.y != b.y))
      return true;
  }
  return false;
}

TEST(DiagramEditing, HorizontalAndVerticalInteriorSegmentsMoveWithoutChangingEndpoints) {
  @autoreleasepool {
    NSArray<NSValue*>* original = bentPath();
    NSArray<NSValue*>* horizontal = GalataMoveWireSegment(original, 2, 112, terminalCards());
    expectGeometry(horizontal, NSMakePoint(0, 0), NSMakePoint(200, 0), terminalCards());
    EXPECT_TRUE(containsSegmentAt(horizontal, true, 112));
    NSArray<NSValue*>* vertical = GalataMoveWireSegment(original, 1, 64, terminalCards());
    expectGeometry(vertical, NSMakePoint(0, 0), NSMakePoint(200, 0), terminalCards());
    EXPECT_TRUE(containsSegmentAt(vertical, false, 64));
    EXPECT_TRUE([original isEqualToArray:bentPath()]);
  }
}

TEST(DiagramEditing, StraightConnectionBecomesAnEditableDoglegWithTerminalStubs) {
  @autoreleasepool {
    NSArray<NSValue*>* original = @[point(0, 0), point(200, 0)];
    for (NSNumber* coordinate in @[@(-48), @64]) {
      NSArray<NSValue*>* result =
          GalataMoveWireSegment(original, 0, coordinate.doubleValue, terminalCards());
      expectGeometry(result, NSMakePoint(0, 0), NSMakePoint(200, 0), terminalCards());
      ASSERT_GE(result.count, 6U);
      EXPECT_EQ(result[1].pointValue.x, 12);
      EXPECT_EQ(result[result.count - 2].pointValue.x, 188);
      EXPECT_TRUE(containsSegmentAt(result, true, coordinate.doubleValue));
    }
    EXPECT_EQ(original.count, 2U);
  }
}

TEST(DiagramEditing, FirstAndLastSegmentsKeepThePortsAndGainDoglegs) {
  @autoreleasepool {
    NSArray<NSValue*>* first = GalataMoveWireSegment(bentPath(), 0, -48, terminalCards());
    expectGeometry(first, NSMakePoint(0, 0), NSMakePoint(200, 0), terminalCards());
    ASSERT_GE(first.count, 2U);
    EXPECT_EQ(first[1].pointValue.x, 12);
    EXPECT_TRUE(containsSegmentAt(first, true, -48));
    NSArray<NSValue*>* last = GalataMoveWireSegment(bentPath(), 4, -64, terminalCards());
    expectGeometry(last, NSMakePoint(0, 0), NSMakePoint(200, 0), terminalCards());
    ASSERT_GE(last.count, 2U);
    EXPECT_EQ(last[last.count - 2].pointValue.x, 188);
    EXPECT_TRUE(containsSegmentAt(last, true, -64));
  }
}

TEST(DiagramEditing, NoOpKeepsTheOriginalAndCoincidentBendsCollapseCleanly) {
  @autoreleasepool {
    NSArray<NSValue*>* straight = @[point(0, 0), point(200, 0)];
    EXPECT_TRUE([GalataMoveWireSegment(straight, 0, 0, terminalCards()) isEqualToArray:straight]);
    NSArray<NSValue*>* bent = bentPath();
    EXPECT_TRUE([GalataMoveWireSegment(bent, 1, 40, terminalCards()) isEqualToArray:bent]);
    NSArray<NSValue*>* collapsed = GalataMoveWireSegment(bent, 2, 0, terminalCards());
    expectGeometry(collapsed, NSMakePoint(0, 0), NSMakePoint(200, 0), terminalCards());
    EXPECT_EQ(collapsed.count, 2U);
  }
}

TEST(DiagramEditing, SegmentMovesRefuseCardsAndReversedPortDirections) {
  @autoreleasepool {
    NSArray<NSValue*>* cards = [terminalCards() arrayByAddingObject:card(50, 100, 30, 30)];
    EXPECT_TRUE(GalataWirePathClear(bentPath(), cards));
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 2, 112, cards), nil);
    cards = [terminalCards() arrayByAddingObject:card(50, 20, 20, 30)];
    EXPECT_TRUE(GalataWirePathClear(bentPath(), cards));
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 1, 60, cards), nil);
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 1, -16, terminalCards()), nil);
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 3, 224, terminalCards()), nil);
    // A new terminal stub may itself hit a nearby card, even if the old short
    // straight connection did not cross either card's interior.
    NSArray<NSValue*>* closeCards = @[card(-20, -10, 20, 20), card(10, -10, 20, 20)];
    NSArray<NSValue*>* shortPath = @[point(0, 0), point(10, 0)];
    EXPECT_TRUE(GalataWirePathClear(shortPath, closeCards));
    EXPECT_EQ(GalataMoveWireSegment(shortPath, 0, 32, closeCards), nil);
  }
}

TEST(DiagramEditing, PathValidationAllowsCardEdgesAndChecksEverySegment) {
  @autoreleasepool {
    NSArray<NSValue*>* edge = [terminalCards() arrayByAddingObject:card(50, 80, 40, 40)];
    EXPECT_TRUE(GalataWirePathClear(bentPath(), edge));
    NSArray<NSValue*>* blocked = [terminalCards() arrayByAddingObject:card(50, 79, 40, 40)];
    EXPECT_FALSE(GalataWirePathClear(bentPath(), blocked));
    NSArray<NSValue*>* diagonal =
        @[point(0, 0), point(40, 0), point(80, 80), point(140, 0), point(200, 0)];
    EXPECT_FALSE(GalataWirePathClear(diagonal, terminalCards()));
    NSArray<NSValue*>* zeroLength = @[point(0, 0), point(40, 0), point(40, 0), point(200, 0)];
    EXPECT_FALSE(GalataWirePathClear(zeroLength, terminalCards()));
    EXPECT_FALSE(GalataWirePathClear(@[point(200, 0), point(0, 0)], @[]));
  }
}

TEST(DiagramEditing, PortDisplayOffsetRoundingRegressionKeepsWireShapeEditable) {
  @autoreleasepool {
    // A fractional target and a negative peer can give the canvas this offset.
    // Converting the displayed port back to logical coordinates rounds it just
    // inside the target card, while its actual position remains unchanged.
    const CGFloat targetX = 465.77435982914494;
    const CGFloat displayOffset = 911.0189537149157;
    const CGFloat restoredTargetX = (targetX + displayOffset) - displayOffset;
    ASSERT_GT(restoredTargetX, targetX);
    ASSERT_LT(restoredTargetX - targetX, 1e-9);
    NSArray<NSValue*>* cards = @[card(-100, -30, 100, 60), card(targetX, -30, 180, 60)];
    NSArray<NSValue*>* original = @[point(0, 0), point(restoredTargetX, 0)];
    EXPECT_TRUE(GalataWirePathClear(original, cards));
    NSArray<NSValue*>* edited = GalataMoveWireSegment(original, 0, 64, cards);
    ASSERT_NE(edited, nil);
    ASSERT_GE(edited.count, 2U);
    EXPECT_TRUE(NSEqualPoints(edited.firstObject.pointValue, NSMakePoint(0, 0)));
    EXPECT_TRUE(NSEqualPoints(edited.lastObject.pointValue, NSMakePoint(restoredTargetX, 0)));
    EXPECT_TRUE(containsSegmentAt(edited, true, 64));
    for (NSUInteger i = 1; i < edited.count; ++i) {
      NSPoint a = edited[i - 1].pointValue, b = edited[i].pointValue;
      EXPECT_NE(a.x == b.x, a.y == b.y);
    }
    // The display-rounding allowance must not admit an actual incursion.
    NSArray<NSValue*>* inside = @[point(0, 0), point(targetX + 1e-5, 0)];
    EXPECT_FALSE(GalataWirePathClear(inside, cards));
    EXPECT_EQ(GalataMoveWireSegment(inside, 0, 64, cards), nil);
  }
}

TEST(DiagramEditing, MalformedAndOutOfBoundsInputsAreRefusedWithoutExceptions) {
  @autoreleasepool {
    const CGFloat nan = std::numeric_limits<CGFloat>::quiet_NaN();
    const CGFloat infinity = std::numeric_limits<CGFloat>::infinity();
    EXPECT_FALSE(GalataWirePathClear(nil, @[]));
    EXPECT_FALSE(GalataWirePathClear(@[], @[]));
    EXPECT_FALSE(GalataWirePathClear(@[point(0, 0)], @[]));
    EXPECT_FALSE(GalataWirePathClear(@[point(0, 0), point(nan, 0)], @[]));
    EXPECT_FALSE(GalataWirePathClear(@[point(0, 0), point(100001, 0)], @[]));
    EXPECT_FALSE(GalataWirePathClear(@[point(-100001, 0), point(0, 0)], @[]));
    EXPECT_FALSE(GalataWirePathClear(@[card(0, 0, 10, 10), point(200, 0)], @[]));
    EXPECT_FALSE(GalataWirePathClear(bentPath(), @[point(0, 0)]));
    EXPECT_FALSE(GalataWirePathClear(bentPath(), @[card(20, 20, 0, 10)]));
    EXPECT_FALSE(GalataWirePathClear(bentPath(), @[card(20, 20, 10, -1)]));
    EXPECT_FALSE(GalataWirePathClear(bentPath(), @[card(nan, 20, 10, 10)]));
    EXPECT_FALSE(GalataWirePathClear(bentPath(), @[card(20, 20, infinity, 10)]));
    EXPECT_FALSE(GalataWirePathClear(bentPath(), @[card(99999, 20, 10, 10)]));
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), NSNotFound, 100, terminalCards()), nil);
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 5, 100, terminalCards()), nil);
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 2, nan, terminalCards()), nil);
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 2, infinity, terminalCards()), nil);
    EXPECT_EQ(GalataMoveWireSegment(bentPath(), 2, 100001, terminalCards()), nil);
    NSMutableArray<NSValue*>* manyPoints = [NSMutableArray array];
    for (NSUInteger i = 0; i < 65; ++i)
      [manyPoints addObject:point(i, 0)];
    EXPECT_FALSE(GalataWirePathClear(manyPoints, @[]));
    NSMutableArray<NSValue*>* manyCards = [NSMutableArray array];
    for (NSUInteger i = 0; i < 129; ++i)
      [manyCards addObject:card(300 + i * 20, 100, 10, 10)];
    EXPECT_FALSE(GalataWirePathClear(bentPath(), manyCards));
  }
}

TEST(DiagramEditing, GridSnappingHandlesPositiveNegativeAndHalfwayCoordinates) {
  EXPECT_EQ(GalataSnapToGrid(0), 0);
  EXPECT_EQ(GalataSnapToGrid(7), 0);
  EXPECT_EQ(GalataSnapToGrid(8), 16);
  EXPECT_EQ(GalataSnapToGrid(23), 16);
  EXPECT_EQ(GalataSnapToGrid(24), 32);
  EXPECT_EQ(GalataSnapToGrid(-7), 0);
  EXPECT_EQ(GalataSnapToGrid(-8), -16);
  EXPECT_EQ(GalataSnapToGrid(-23), -16);
  EXPECT_EQ(GalataSnapToGrid(-24), -32);
  EXPECT_EQ(GalataSnapToGrid(13, 10), 10);
  EXPECT_EQ(GalataSnapToGrid(13, 0), 13);
  EXPECT_EQ(GalataSnapToGrid(13, -1), 13);
  EXPECT_TRUE(std::isnan(GalataSnapToGrid(std::numeric_limits<CGFloat>::quiet_NaN())));
}

TEST(DiagramEditing, BlockSnappingUsesPeerEdgesAndCentersBeforeGridAndReportsGuides) {
  @autoreleasepool {
    NSRect proposed = NSMakeRect(97, 53, 180, 72);
    NSArray<NSValue*>* peers = @[card(100, 200, 180, 72), card(500, 50, 180, 72)];
    GalataBlockSnapResult snapped = GalataSnapBlock(proposed, peers, 1, YES);
    EXPECT_EQ(snapped.origin.x, 100);
    EXPECT_EQ(snapped.origin.y, 50);
    EXPECT_TRUE(snapped.hasVerticalGuide);
    EXPECT_TRUE(snapped.hasHorizontalGuide);
    EXPECT_EQ(snapped.verticalGuide, 100);
    EXPECT_EQ(snapped.horizontalGuide, 50);
    // Centre-to-centre alignment, with different widths, must translate origin.
    snapped = GalataSnapBlock(NSMakeRect(147, 500, 80, 72), @[card(100, 200, 180, 72)], 1, YES);
    EXPECT_EQ(snapped.origin.x, 150);
    EXPECT_TRUE(snapped.hasVerticalGuide);
    EXPECT_EQ(snapped.verticalGuide, 190);
    EXPECT_FALSE(snapped.hasHorizontalGuide);
    EXPECT_EQ(snapped.origin.y, 496);
    snapped = GalataSnapBlock(proposed, peers, 1, NO);
    EXPECT_EQ(snapped.origin.x, 96);
    EXPECT_EQ(snapped.origin.y, 48);
    EXPECT_FALSE(snapped.hasVerticalGuide);
    EXPECT_FALSE(snapped.hasHorizontalGuide);
  }
}

TEST(DiagramEditing, PeerSnapThresholdUsesScreenDistanceAndStableNearestMatches) {
  @autoreleasepool {
    NSRect proposed = NSMakeRect(95, 500, 180, 72);
    NSArray<NSValue*>* peers = @[card(100, 200, 180, 72)];
    GalataBlockSnapResult highZoom = GalataSnapBlock(proposed, peers, 2, YES);
    EXPECT_FALSE(highZoom.hasVerticalGuide);
    EXPECT_EQ(highZoom.origin.x, 96);
    GalataBlockSnapResult lowZoom = GalataSnapBlock(proposed, peers, 0.5, YES);
    EXPECT_TRUE(lowZoom.hasVerticalGuide);
    EXPECT_EQ(lowZoom.origin.x, 100);
    GalataBlockSnapResult closest =
        GalataSnapBlock(proposed, @[card(100, 200, 180, 72), card(94, 200, 180, 72)], 1, YES);
    EXPECT_EQ(closest.origin.x, 94);
    GalataBlockSnapResult tied =
        GalataSnapBlock(proposed, @[card(94, 200, 180, 72), card(96, 200, 180, 72)], 1, YES);
    EXPECT_EQ(tied.origin.x, 94);
  }
}

TEST(DiagramEditing, BlockSnappingHandlesNegativeSpaceAndInvalidGeometry) {
  @autoreleasepool {
    NSRect proposed = NSMakeRect(-103, -53, 180, 72);
    GalataBlockSnapResult result = GalataSnapBlock(proposed, @[], 1, YES);
    EXPECT_EQ(result.origin.x, -96);
    EXPECT_EQ(result.origin.y, -48);
    EXPECT_FALSE(result.hasVerticalGuide);
    result = GalataSnapBlock(proposed, @[card(-100, 200, 180, 72)], 1, YES);
    EXPECT_EQ(result.origin.x, -100);
    EXPECT_EQ(result.verticalGuide, -100);
    EXPECT_TRUE(result.hasVerticalGuide);
    result = GalataSnapBlock(proposed, @[point(0, 0), card(20, 20, 0, 10)], 1, YES);
    EXPECT_EQ(result.origin.x, -96);
    EXPECT_FALSE(result.hasVerticalGuide);
    result = GalataSnapBlock(proposed, @[], 0, YES);
    EXPECT_TRUE(NSEqualPoints(result.origin, proposed.origin));
    EXPECT_FALSE(result.hasVerticalGuide);
    NSRect invalid = NSMakeRect(30, 50, -20, 72);
    result = GalataSnapBlock(invalid, @[], 1, YES);
    EXPECT_TRUE(NSEqualPoints(result.origin, invalid.origin));
  }
}

}  // namespace
