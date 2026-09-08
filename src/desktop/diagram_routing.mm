// SPDX-License-Identifier: Apache-2.0
#import "diagram_routing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

namespace {

// Two clearance regions must fit inside the starter's 30-point card gap. The
// visible wire still keeps distance from every actual card after expansion.
constexpr CGFloat clearance = 12.0;
constexpr CGFloat bendCost = 24.0;
constexpr CGFloat laneSpacing = 8.0;
constexpr CGFloat parallelCost = 32.0;
constexpr std::size_t maxRectangles = 128;
constexpr std::size_t maxStates = 150000;
constexpr std::size_t maxRequests = 256;
constexpr std::size_t noParent = std::numeric_limits<std::size_t>::max();

struct Segment {
  NSPoint a;
  NSPoint b;
};

bool finitePoint(NSPoint point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool inside(NSPoint point, NSRect rect) {
  return point.x > NSMinX(rect) && point.x < NSMaxX(rect) && point.y > NSMinY(rect)
         && point.y < NSMaxY(rect);
}

bool crossesHorizontal(NSPoint a, NSPoint b, NSRect rect) {
  return a.y > NSMinY(rect) && a.y < NSMaxY(rect) && std::max(a.x, b.x) > NSMinX(rect)
         && std::min(a.x, b.x) < NSMaxX(rect);
}

void uniqueCoordinates(std::vector<CGFloat>& coordinates) {
  std::sort(coordinates.begin(), coordinates.end());
  coordinates.erase(std::unique(coordinates.begin(), coordinates.end()), coordinates.end());
}

std::size_t coordinateIndex(const std::vector<CGFloat>& coordinates, CGFloat coordinate) {
  return static_cast<std::size_t>(
      std::lower_bound(coordinates.begin(), coordinates.end(), coordinate) - coordinates.begin());
}

struct Visit {
  CGFloat estimate;
  CGFloat distance;
  std::size_t state;
};

struct LaterVisit {
  bool operator()(const Visit& lhs, const Visit& rhs) const {
    return std::tie(lhs.estimate, lhs.distance, lhs.state)
           > std::tie(rhs.estimate, rhs.distance, rhs.state);
  }
};

NSArray<NSValue*>* route(NSPoint start,
                         NSPoint end,
                         NSArray<NSValue*>* blockRects,
                         const std::vector<Segment>& occupied) {
  if (!finitePoint(start) || !finitePoint(end) || blockRects.count > maxRectangles)
    return @[];

  NSPoint exit = NSMakePoint(start.x + clearance, start.y);
  NSPoint entry = NSMakePoint(end.x - clearance, end.y);
  if (!finitePoint(exit) || !finitePoint(entry) || exit.x <= start.x || entry.x >= end.x)
    return @[];

  std::vector<NSRect> obstacles;
  std::vector<CGFloat> xs{exit.x, entry.x};
  std::vector<CGFloat> ys{exit.y, entry.y};
  for (NSValue* value in blockRects) {
    NSRect rect = value.rectValue;
    if (!finitePoint(rect.origin) || !std::isfinite(rect.size.width)
        || !std::isfinite(rect.size.height) || rect.size.width <= 0 || rect.size.height <= 0
        || !std::isfinite(NSMaxX(rect)) || !std::isfinite(NSMaxY(rect)))
      return @[];

    // Only endpoint shafts may pass through their own card's clearance region.
    // They must still avoid every actual card, including overlapping neighbours.
    if (inside(start, rect) || inside(end, rect) || crossesHorizontal(start, exit, rect)
        || crossesHorizontal(entry, end, rect))
      return @[];

    NSRect expanded = NSInsetRect(rect, -clearance, -clearance);
    if (!finitePoint(expanded.origin) || !std::isfinite(NSMaxX(expanded))
        || !std::isfinite(NSMaxY(expanded)) || inside(exit, expanded) || inside(entry, expanded))
      return @[];
    obstacles.push_back(expanded);
    xs.push_back(NSMinX(expanded));
    xs.push_back(NSMaxX(expanded));
    ys.push_back(NSMinY(expanded));
    ys.push_back(NSMaxY(expanded));
  }

  // Extra outer lanes also cover degenerate inputs with no obstacles or only a
  // shared endpoint row. All coordinates come from the displayed geometry.
  const auto [minX, maxX] = std::minmax_element(xs.begin(), xs.end());
  const auto [minY, maxY] = std::minmax_element(ys.begin(), ys.end());
  const CGFloat left = *minX - clearance;
  const CGFloat right = *maxX + clearance;
  const CGFloat top = *minY - clearance;
  const CGFloat bottom = *maxY + clearance;
  if (!finitePoint(NSMakePoint(left, top)) || !finitePoint(NSMakePoint(right, bottom)))
    return @[];
  xs.insert(xs.end(), {left, right});
  ys.insert(ys.end(), {top, bottom});
  uniqueCoordinates(xs);
  uniqueCoordinates(ys);
  if (xs.size() * ys.size() * 2 > maxStates)
    return @[];

  // Add actual alternatives before charging for shared lanes. Merely offsetting
  // a finished polyline can push it into a card, and a penalty without additional
  // grid coordinates still forces feedback wires into the same outer gutter.
  const auto addLane =
      [](std::vector<CGFloat>& coordinates, std::size_t otherCount, CGFloat coordinate) {
        if (!std::isfinite(coordinate) || (coordinates.size() + 1) * otherCount * 2 > maxStates)
          return;
        auto position = std::lower_bound(coordinates.begin(), coordinates.end(), coordinate);
        if (position == coordinates.end() || *position != coordinate)
          coordinates.insert(position, coordinate);
      };
  for (auto it = occupied.rbegin(); it != occupied.rend(); ++it) {
    if (it->a.x == it->b.x) {
      addLane(xs, ys.size(), it->a.x + laneSpacing);
      addLane(xs, ys.size(), it->a.x - laneSpacing);
    } else {
      addLane(ys, xs.size(), it->a.y - laneSpacing);
      addLane(ys, xs.size(), it->a.y + laneSpacing);
    }
  }
  const std::size_t width = xs.size();
  const std::size_t height = ys.size();
  const std::size_t states = width * height * 2;
  if (states > maxStates)
    return @[];

  // Adjacent grid coordinates cannot straddle an unrepresented rectangle edge.
  // Marking intervals once avoids scanning every card at each A* expansion.
  std::vector<bool> horizontal((width - 1) * height, true);
  std::vector<bool> vertical(width * (height - 1), true);
  for (NSRect rect : obstacles) {
    const std::size_t x0 = coordinateIndex(xs, NSMinX(rect));
    const std::size_t x1 = coordinateIndex(xs, NSMaxX(rect));
    const std::size_t y0 = coordinateIndex(ys, NSMinY(rect));
    const std::size_t y1 = coordinateIndex(ys, NSMaxY(rect));
    for (std::size_t y = y0 + 1; y < y1; ++y)
      for (std::size_t x = x0; x < x1; ++x)
        horizontal[y * (width - 1) + x] = false;
    for (std::size_t y = y0; y < y1; ++y)
      for (std::size_t x = x0 + 1; x < x1; ++x)
        vertical[y * width + x] = false;
  }

  std::vector<CGFloat> horizontalPenalty(horizontal.size(), 0);
  std::vector<CGFloat> verticalPenalty(vertical.size(), 0);
  // Rasterize only intervals near each occupied segment. Scanning every old
  // segment at every A* expansion becomes expensive during a block drag.
  for (const Segment& segment : occupied) {
    const bool isHorizontal = segment.a.y == segment.b.y;
    const auto& along = isHorizontal ? xs : ys;
    const auto& across = isHorizontal ? ys : xs;
    const CGFloat center = isHorizontal ? segment.a.y : segment.a.x;
    const CGFloat low =
        isHorizontal ? std::min(segment.a.x, segment.b.x) : std::min(segment.a.y, segment.b.y);
    const CGFloat high =
        isHorizontal ? std::max(segment.a.x, segment.b.x) : std::max(segment.a.y, segment.b.y);
    auto firstLane = std::upper_bound(across.begin(), across.end(), center - laneSpacing);
    const auto lastLane = std::lower_bound(across.begin(), across.end(), center + laneSpacing);
    auto firstInterval = std::upper_bound(along.begin(), along.end(), low);
    if (firstInterval != along.begin())
      --firstInterval;
    for (auto lane = firstLane; lane != lastLane; ++lane) {
      const auto laneIndex = static_cast<std::size_t>(lane - across.begin());
      const CGFloat weight = parallelCost * (1 - std::abs(*lane - center) / laneSpacing);
      for (auto interval = firstInterval; interval + 1 < along.end() && *interval < high;
           ++interval) {
        const CGFloat overlap = std::min(*(interval + 1), high) - std::max(*interval, low);
        if (overlap <= 0)
          continue;
        const auto intervalIndex = static_cast<std::size_t>(interval - along.begin());
        if (isHorizontal)
          horizontalPenalty[laneIndex * (width - 1) + intervalIndex] += weight * overlap;
        else
          verticalPenalty[intervalIndex * width + laneIndex] += weight * overlap;
      }
    }
  }

  const std::size_t source = coordinateIndex(ys, exit.y) * width + coordinateIndex(xs, exit.x);
  const std::size_t target = coordinateIndex(ys, entry.y) * width + coordinateIndex(xs, entry.x);
  const auto point = [&](std::size_t node) {
    return NSMakePoint(xs[node % width], ys[node / width]);
  };
  const auto heuristic = [&](std::size_t node) {
    const NSPoint p = point(node);
    return std::abs(p.x - entry.x) + std::abs(p.y - entry.y);
  };
  std::vector<CGFloat> distances(states, std::numeric_limits<CGFloat>::infinity());
  std::vector<std::size_t> parents(states, noParent);
  std::vector<bool> settled(states, false);
  std::priority_queue<Visit, std::vector<Visit>, LaterVisit> pending;
  // Direction 0 is horizontal, matching the already fixed exit shaft.
  distances[source * 2] = 0;
  pending.push({heuristic(source), 0, source * 2});
  CGFloat best = std::numeric_limits<CGFloat>::infinity();
  std::size_t goal = noParent;
  while (!pending.empty()) {
    const Visit visit = pending.top();
    pending.pop();
    if (visit.estimate >= best)
      break;
    if (settled[visit.state] || visit.distance != distances[visit.state])
      continue;
    settled[visit.state] = true;
    const std::size_t node = visit.state / 2;
    const std::size_t direction = visit.state % 2;
    if (node == target) {
      const CGFloat candidate = visit.distance + (direction == 0 ? 0 : bendCost);
      if (candidate < best) {
        best = candidate;
        goal = visit.state;
      }
      continue;
    }
    const std::size_t x = node % width;
    const std::size_t y = node / width;
    const auto advance = [&](std::size_t next, std::size_t nextDirection, CGFloat overlapPenalty) {
      const NSPoint from = point(node);
      const NSPoint to = point(next);
      // Neither terminal can double back along its fixed rightward shaft.
      if ((node == source && to.x < from.x) || (next == target && from.x > to.x))
        return;
      const std::size_t nextState = next * 2 + nextDirection;
      if (settled[nextState])
        return;
      const CGFloat candidate = visit.distance + std::abs(to.x - from.x) + std::abs(to.y - from.y)
                                + (direction == nextDirection ? 0 : bendCost) + overlapPenalty;
      if (candidate < distances[nextState]) {
        distances[nextState] = candidate;
        parents[nextState] = visit.state;
        pending.push({candidate + heuristic(next), candidate, nextState});
      }
    };
    if (x + 1 < width && horizontal[y * (width - 1) + x])
      advance(node + 1, 0, horizontalPenalty[y * (width - 1) + x]);
    if (y + 1 < height && vertical[y * width + x])
      advance(node + width, 1, verticalPenalty[y * width + x]);
    if (x > 0 && horizontal[y * (width - 1) + x - 1])
      advance(node - 1, 0, horizontalPenalty[y * (width - 1) + x - 1]);
    if (y > 0 && vertical[(y - 1) * width + x])
      advance(node - width, 1, verticalPenalty[(y - 1) * width + x]);
  }
  if (goal == noParent)
    return @[];

  std::vector<NSPoint> middle;
  for (std::size_t state = goal; state != noParent; state = parents[state])
    middle.push_back(point(state / 2));
  std::reverse(middle.begin(), middle.end());
  std::vector<NSPoint> polyline{start};
  middle.push_back(end);
  for (NSPoint p : middle) {
    if (NSEqualPoints(p, polyline.back()))
      continue;
    if (polyline.size() >= 2) {
      const NSPoint a = polyline[polyline.size() - 2];
      const NSPoint b = polyline.back();
      if ((a.x == b.x && b.x == p.x) || (a.y == b.y && b.y == p.y))
        polyline.pop_back();
    }
    polyline.push_back(p);
  }
  NSMutableArray<NSValue*>* result = [NSMutableArray arrayWithCapacity:polyline.size()];
  for (NSPoint p : polyline)
    [result addObject:[NSValue valueWithPoint:p]];
  return result;
}

}  // namespace

NSArray<NSValue*>* GalataWireRoute(NSPoint start, NSPoint end, NSArray<NSValue*>* blockRects) {
  return route(start, end, blockRects, {});
}

NSArray<NSArray<NSValue*>*>* GalataWireRoutes(NSArray<NSValue*>* starts,
                                              NSArray<NSValue*>* ends,
                                              NSArray<NSValue*>* blockRects) {
  if (starts.count != ends.count || starts.count > maxRequests)
    return @[];
  NSMutableArray<NSArray<NSValue*>*>* result = [NSMutableArray arrayWithCapacity:starts.count];
  std::vector<Segment> occupied;
  for (NSUInteger i = 0; i < starts.count; ++i) {
    NSArray<NSValue*>* points =
        route(starts[i].pointValue, ends[i].pointValue, blockRects, occupied);
    [result addObject:points];
    for (NSUInteger j = 1; j < points.count; ++j)
      occupied.push_back({points[j - 1].pointValue, points[j].pointValue});
  }
  return result;
}
