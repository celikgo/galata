// SPDX-License-Identifier: Apache-2.0
// Presentation-only invariants: endpoint orientation and independent segment/card
// intersection checks. No path snapshots or numerical model behaviour asserted.
#import "diagram_routing.hpp"
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

NSValue* card(double x, double y, double width = 180, double height = 88) {
  return [NSValue valueWithRect:NSMakeRect(x, y, width, height)];
}

void expectClearPoints(NSPoint start,
                       NSPoint end,
                       NSArray<NSValue*>* cards,
                       NSArray<NSValue*>* points) {
  ASSERT_GE(points.count, 2U);
  EXPECT_TRUE(NSEqualPoints(points.firstObject.pointValue, start));
  EXPECT_TRUE(NSEqualPoints(points.lastObject.pointValue, end));
  NSPoint departure = points[1].pointValue;
  NSPoint approach = points[points.count - 2].pointValue;
  EXPECT_GT(departure.x, start.x);
  EXPECT_EQ(departure.y, start.y);
  EXPECT_LT(approach.x, end.x);
  EXPECT_EQ(approach.y, end.y);
  for (NSUInteger i = 1; i < points.count; ++i) {
    SCOPED_TRACE(::testing::Message() << "segment " << i);
    NSPoint a = points[i - 1].pointValue;
    NSPoint b = points[i].pointValue;
    EXPECT_TRUE(std::isfinite(a.x) && std::isfinite(a.y));
    EXPECT_TRUE(std::isfinite(b.x) && std::isfinite(b.y));
    ASSERT_NE(a.x == b.x, a.y == b.y) << "Each segment must be nonzero and orthogonal";
    for (NSValue* value in cards) {
      NSRect r = value.rectValue;
      // Intersect against actual card interiors directly. This checker has no
      // clearance grid, route cost or path-search implementation in common.
      if (a.x == b.x) {
        if (a.x <= r.origin.x || a.x >= r.origin.x + r.size.width)
          continue;
        const double low = std::fmin(a.y, b.y);
        const double high = std::fmax(a.y, b.y);
        EXPECT_TRUE(high <= r.origin.y || low >= r.origin.y + r.size.height);
      } else {
        if (a.y <= r.origin.y || a.y >= r.origin.y + r.size.height)
          continue;
        const double low = std::fmin(a.x, b.x);
        const double high = std::fmax(a.x, b.x);
        EXPECT_TRUE(high <= r.origin.x || low >= r.origin.x + r.size.width);
      }
    }
  }
}

void expectClearRoute(NSPoint start, NSPoint end, NSArray<NSValue*>* cards) {
  NSArray<NSValue*>* points = GalataWireRoute(start, end, cards);
  expectClearPoints(start, end, cards, points);
  EXPECT_TRUE([points isEqualToArray:GalataWireRoute(start, end, cards)]);
}

NSValue* endpoint(double x, double y) {
  return [NSValue valueWithPoint:NSMakePoint(x, y)];
}

void expectDistinctLongSegments(NSArray<NSArray<NSValue*>*>* routes, double maximumSharedLength) {
  // Compare line intervals directly, independently of search costs or its grid.
  // Crossings are allowed; a long coincident segment hides a separate signal.
  for (NSUInteger i = 0; i < routes.count; ++i) {
    for (NSUInteger j = i + 1; j < routes.count; ++j) {
      SCOPED_TRACE(::testing::Message() << "route pair " << i << ", " << j);
      for (NSUInteger a = 1; a < routes[i].count; ++a) {
        NSPoint p = routes[i][a - 1].pointValue;
        NSPoint q = routes[i][a].pointValue;
        for (NSUInteger b = 1; b < routes[j].count; ++b) {
          NSPoint r = routes[j][b - 1].pointValue;
          NSPoint s = routes[j][b].pointValue;
          if (p.x == q.x && r.x == s.x) {
            const double shared = std::fmin(std::fmax(p.y, q.y), std::fmax(r.y, s.y))
                                  - std::fmax(std::fmin(p.y, q.y), std::fmin(r.y, s.y));
            if (shared > maximumSharedLength)
              EXPECT_GE(std::abs(p.x - r.x), 6);
          } else if (p.y == q.y && r.y == s.y) {
            const double shared = std::fmin(std::fmax(p.x, q.x), std::fmax(r.x, s.x))
                                  - std::fmax(std::fmin(p.x, q.x), std::fmin(r.x, s.x));
            if (shared > maximumSharedLength)
              EXPECT_GE(std::abs(p.y - r.y), 6);
          }
        }
      }
    }
  }
}

NSArray<NSArray<NSValue*>*>* expectClearBatch(NSArray<NSValue*>* starts,
                                              NSArray<NSValue*>* ends,
                                              NSArray<NSValue*>* cards) {
  NSArray<NSArray<NSValue*>*>* routes = GalataWireRoutes(starts, ends, cards);
  EXPECT_EQ(routes.count, starts.count);
  for (NSUInteger i = 0; i < routes.count; ++i) {
    SCOPED_TRACE(::testing::Message() << "route " << i);
    expectClearPoints(starts[i].pointValue, ends[i].pointValue, cards, routes[i]);
  }
  EXPECT_TRUE([routes isEqualToArray:GalataWireRoutes(starts, ends, cards)]);
  return routes;
}

TEST(DiagramRouting, ForwardBackwardAndSelfConnectionsKeepEndpointDirection) {
  @autoreleasepool {
    expectClearRoute(
        NSMakePoint(280, 144), NSMakePoint(560, 144), @[card(100, 100), card(560, 100)]);
    expectClearRoute(
        NSMakePoint(740, 144), NSMakePoint(100, 144), @[card(100, 100), card(560, 100)]);
    expectClearRoute(NSMakePoint(280, 144), NSMakePoint(100, 144), @[card(100, 100)]);
  }
}

TEST(DiagramRouting, ForwardAndFeedbackRoutesAvoidInterveningCards) {
  @autoreleasepool {
    NSArray<NSValue*>* cards = @[card(100, 100), card(450, 70, 180, 148), card(800, 100)];
    expectClearRoute(NSMakePoint(280, 144), NSMakePoint(800, 144), cards);
    expectClearRoute(NSMakePoint(980, 144), NSMakePoint(100, 112), cards);
  }
}

TEST(DiagramRouting, ScalarStarterKeepsAllConnectionsAtOriginalAndTranslatedCoordinates) {
  @autoreleasepool {
    // The starter's x/y cards are separated by 30 points. A larger routing
    // margin once hid both wires leaving x, despite the cards not overlapping.
    for (NSValue* translation in @[
           [NSValue valueWithPoint:NSMakePoint(0, 0)],
           [NSValue valueWithPoint:NSMakePoint(-600, -300)],
           [NSValue valueWithPoint:NSMakePoint(600, 300)]
         ]) {
      NSPoint offset = translation.pointValue;
      SCOPED_TRACE(::testing::Message() << "translation " << offset.x << ", " << offset.y);
      NSArray<NSValue*>* cards = @[
        card(40 + offset.x, 55 + offset.y, 180, 72),
        card(360 + offset.x, 205 + offset.y, 180, 72),
        card(260 + offset.x, 55 + offset.y, 180, 72),
        card(480 + offset.x, 55 + offset.y, 180, 72),
        card(690 + offset.x, 55 + offset.y, 180, 72)
      ];
      const auto point = [&](double x, double y) {
        return NSMakePoint(x + offset.x, y + offset.y);
      };
      expectClearRoute(point(660, 91), point(360, 241), cards);  // x -> feedback
      expectClearRoute(point(220, 91), point(260, 67), cards);   // command -> rate[0]
      expectClearRoute(point(540, 241), point(260, 83), cards);  // feedback -> rate[1]
      expectClearRoute(point(440, 91), point(480, 91), cards);   // rate -> x
      expectClearRoute(point(660, 91), point(690, 91), cards);   // x -> y
    }
  }
}

TEST(DiagramRouting, DenseNineteenCardLayoutKeepsEveryPairClearAndRepeatable) {
  @autoreleasepool {
    // Close columns and narrow vertical gaps reproduce the important geometry
    // of an imported study without sharing its layout generator or graph data.
    NSMutableArray<NSValue*>* cards = [NSMutableArray array];
    for (int i = 0; i < 19; ++i)
      [cards addObject:card(100 + (i / 5) * 230, 100 + (i % 5) * 95)];
    for (NSUInteger i = 0; i < cards.count; ++i) {
      NSRect source = cards[i].rectValue;
      for (NSUInteger j = 0; j < cards.count; ++j) {
        SCOPED_TRACE(::testing::Message() << "source " << i << ", target " << j);
        NSRect target = cards[j].rectValue;
        expectClearRoute(NSMakePoint(NSMaxX(source), NSMidY(source)),
                         NSMakePoint(NSMinX(target), NSMinY(target) + 12 + 16 * (j % 5)),
                         cards);
      }
    }
  }
}

TEST(DiagramRouting, VariedManualPositionsKeepEveryPairClearAndRepeatable) {
  @autoreleasepool {
    NSMutableArray<NSValue*>* cards = [NSMutableArray array];
    for (int i = 0; i < 12; ++i)
      [cards addObject:card(80 + (i % 4) * 300 + ((i * 7) % 19),
                            80 + (i / 4) * 160 + ((i * 11) % 23),
                            180,
                            72 + (i % 2) * 16)];
    for (NSUInteger i = 0; i < cards.count; ++i) {
      for (NSUInteger j = 0; j < cards.count; ++j) {
        SCOPED_TRACE(::testing::Message() << "source " << i << ", target " << j);
        NSRect source = cards[i].rectValue;
        NSRect target = cards[j].rectValue;
        expectClearRoute(NSMakePoint(NSMaxX(source), NSMidY(source)),
                         NSMakePoint(NSMinX(target), NSMidY(target)),
                         cards);
      }
    }
  }
}

TEST(DiagramRouting, MalformedGeometryProducesAnExplicitUnresolvedRoute) {
  @autoreleasepool {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    EXPECT_EQ(GalataWireRoute(NSMakePoint(nan, 0), NSMakePoint(500, 0), @[]).count, 0U);
    EXPECT_EQ(GalataWireRoute(NSMakePoint(0, 0), NSMakePoint(500, infinity), @[]).count, 0U);
    for (NSValue* invalid in
         @[card(100, 100, 0), card(100, 100, 180, -1), card(nan, 100), card(100, 100, infinity)]) {
      NSArray<NSValue*>* route =
          GalataWireRoute(NSMakePoint(0, 0), NSMakePoint(500, 0), @[invalid]);
      EXPECT_EQ(route.count, 0U);
    }
  }
}

TEST(DiagramRouting, BlockedPortsAndExcessiveLayoutsProduceAnExplicitUnresolvedRoute) {
  @autoreleasepool {
    NSArray<NSValue*>* narrow = GalataWireRoute(
        NSMakePoint(280, 144), NSMakePoint(290, 144), @[card(100, 100), card(290, 100)]);
    EXPECT_EQ(narrow.count, 0U);
    NSArray<NSValue*>* overlapping = GalataWireRoute(
        NSMakePoint(280, 144), NSMakePoint(270, 144), @[card(100, 100), card(270, 100)]);
    EXPECT_EQ(overlapping.count, 0U);
    NSMutableArray<NSValue*>* tooMany = [NSMutableArray array];
    for (int i = 0; i < 129; ++i)
      [tooMany addObject:card(100 + i * 230, 100)];
    EXPECT_EQ(GalataWireRoute(NSMakePoint(280, 144), NSMakePoint(330, 144), tooMany).count, 0U);
  }
}

TEST(DiagramRouting, FeedbackSignalsUseDistinctLongTrunksAroundTheSameObstacle) {
  @autoreleasepool {
    NSArray<NSValue*>* cards = @[
      card(100, 100),
      card(450, 70, 180, 640),
      card(800, 100),
      card(800, 250),
      card(800, 400),
      card(800, 550)
    ];
    NSArray<NSValue*>* starts =
        @[endpoint(980, 144), endpoint(980, 294), endpoint(980, 444), endpoint(980, 594)];
    NSArray<NSValue*>* ends =
        @[endpoint(100, 112), endpoint(100, 128), endpoint(100, 144), endpoint(100, 160)];
    NSArray<NSArray<NSValue*>*>* routes = expectClearBatch(starts, ends, cards);
    // Only short common terminal approaches are acceptable in this roomy layout.
    expectDistinctLongSegments(routes, 48);
  }
}

TEST(DiagramRouting, FanoutSignalsSeparateAfterTheCommonOutputShaft) {
  @autoreleasepool {
    NSArray<NSValue*>* cards = @[
      card(100, 100),
      card(100, 250),
      card(100, 400),
      card(100, 550),
      card(450, 70, 180, 640),
      card(800, 306)
    ];
    NSArray<NSValue*>* starts =
        @[endpoint(980, 350), endpoint(980, 350), endpoint(980, 350), endpoint(980, 350)];
    NSArray<NSValue*>* ends =
        @[endpoint(100, 144), endpoint(100, 294), endpoint(100, 444), endpoint(100, 594)];
    NSArray<NSArray<NSValue*>*>* routes = expectClearBatch(starts, ends, cards);
    expectDistinctLongSegments(routes, 48);
  }
}

TEST(DiagramRouting, DenseBatchPreservesEveryConnectionAndCardClearance) {
  @autoreleasepool {
    NSMutableArray<NSValue*>* cards = [NSMutableArray array];
    NSMutableArray<NSValue*>* starts = [NSMutableArray array];
    NSMutableArray<NSValue*>* ends = [NSMutableArray array];
    for (int i = 0; i < 19; ++i)
      [cards addObject:card(100 + (i / 5) * 230, 100 + (i % 5) * 95)];
    for (NSUInteger i = 0; i < 5; ++i) {
      for (NSUInteger j = 0; j < 10; ++j) {
        NSRect source = cards[i + 10].rectValue;
        NSRect target = cards[j].rectValue;
        [starts addObject:endpoint(NSMaxX(source), NSMidY(source))];
        [ends addObject:endpoint(NSMinX(target), NSMinY(target) + 12 + 16 * i)];
      }
    }
    expectClearBatch(starts, ends, cards);
  }
}

TEST(DiagramRouting, BatchKeepsUnresolvedSlotsAndRefusesExcessiveRequests) {
  @autoreleasepool {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    NSArray<NSValue*>* starts = @[endpoint(nan, 0), endpoint(280, 144)];
    NSArray<NSValue*>* ends = @[endpoint(560, 144), endpoint(560, 144)];
    NSArray<NSValue*>* cards = @[card(100, 100), card(560, 100)];
    NSArray<NSArray<NSValue*>*>* routes = GalataWireRoutes(starts, ends, cards);
    ASSERT_EQ(routes.count, 2U);
    EXPECT_EQ(routes[0].count, 0U);
    expectClearPoints(starts[1].pointValue, ends[1].pointValue, cards, routes[1]);
    EXPECT_EQ(GalataWireRoutes(@[endpoint(280, 144)], @[], cards).count, 0U);
    NSMutableArray<NSValue*>* excessive = [NSMutableArray array];
    for (int i = 0; i < 257; ++i)
      [excessive addObject:endpoint(280, 144)];
    EXPECT_EQ(GalataWireRoutes(excessive, excessive, cards).count, 0U);
  }
}

}  // namespace
