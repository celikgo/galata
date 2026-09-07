// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_DESKTOP_DIAGRAM_ROUTING_HPP
#define GALATA_DESKTOP_DIAGRAM_ROUTING_HPP

#import <AppKit/AppKit.h>

// Presentation geometry only. Include every block's displayed rectangle, including
// the source and target. Endpoints lie on the source's right and target's left
// edges. Returned points exit and enter rightward through 12-point stubs and
// otherwise remain outside block rectangles with 12 points of clearance.
//
// Empty means no route with this clearance, invalid geometry, or a work limit:
// at most 128 rectangles and 150000 directional grid states. This is not a graph
// validity result. Callers must expose unresolved connections and include route
// extents in the scrollable canvas; routes may use gutters outside all blocks.
NSArray<NSValue*>* GalataWireRoute(NSPoint start, NSPoint end, NSArray<NSValue*>* blockRects);

// Route a bounded, ordered set together. Earlier connections reserve preferred
// lanes; later ones pay for parallel overlap and can use nearby clear lanes.
// This reduces shared trunks without displacing a route through a card. Shared
// endpoint shafts and overlap in narrow passages can remain. The result has one
// point array per request, including empty arrays for unresolved connections.
// Mismatched endpoint counts or more than 256 requests return an empty result.
// Per-route geometry and search limits are the same as GalataWireRoute.
NSArray<NSArray<NSValue*>*>* GalataWireRoutes(NSArray<NSValue*>* starts,
                                              NSArray<NSValue*>* ends,
                                              NSArray<NSValue*>* blockRects);

#endif
