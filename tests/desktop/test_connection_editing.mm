// SPDX-License-Identifier: Apache-2.0
// Draft-editing contract: one occupied input, atomic replacement, and input
// independence. These are editor invariants, not numerical model validation.
#import "connection_editing.hpp"
#include <gtest/gtest.h>

#include <limits>

namespace {

NSArray<NSDictionary*>* blocks() {
  return @[
    @{@"id": @"command", @"kind": @"constant"},
    @{@"id": @"alternate", @"kind": @"constant"},
    @{@"id": @"rate",
      @"kind": @"sum",
      @"signs": @[@1, @-1]},
    @{@"id": @"row",
      @"kind": @"linear_combination",
      @"terms": @[@{}, @{}, @{}]},
    @{@"id": @"state", @"kind": @"integrator"},
    @{@"id": @"gain", @"kind": @"gain"},
    @{@"id": @"output", @"kind": @"output"}
  ];
}

NSDictionary* wire(NSString* source, NSString* target, id input = @0) {
  return @{@"source": source, @"target": target, @"input": input};
}

TEST(ConnectionEditing, AppendsAConnectionWithoutChangingExistingValuesOrOrder) {
  @autoreleasepool {
    NSArray<NSDictionary*>* original = @[wire(@"command", @"rate"), wire(@"state", @"output")];
    NSDictionary* addition = wire(@"alternate", @"rate", @1);
    NSString* error = @"previous error";
    NSArray<NSDictionary*>* result =
        GalataConnectionEdit(blocks(), original, nil, addition, &error);
    ASSERT_NE(result, nil);
    EXPECT_EQ(error, nil);
    EXPECT_TRUE(([result isEqualToArray:@[original[0], original[1], addition]]));
    EXPECT_EQ(original.count, 2U);
  }
}

TEST(ConnectionEditing, ReplacesOnlyTheSelectedWireInPlaceForSourceTargetAndInputEdits) {
  @autoreleasepool {
    NSDictionary* selected = wire(@"command", @"row");
    NSArray<NSDictionary*>* original =
        @[wire(@"state", @"output"), selected, wire(@"alternate", @"rate", @1)];
    for (NSDictionary* replacement in @[
           wire(@"alternate", @"row"),
           wire(@"command", @"rate"),
           wire(@"command", @"row", @2),
           wire(@"state", @"state")
         ]) {
      NSString* error = nil;
      NSArray<NSDictionary*>* result =
          GalataConnectionEdit(blocks(), original, selected, replacement, &error);
      ASSERT_NE(result, nil);
      EXPECT_EQ(error, nil);
      EXPECT_EQ(result.count, original.count);
      EXPECT_TRUE(([result isEqualToArray:@[original[0], replacement, original[2]]]));
      EXPECT_TRUE([original[1] isEqualToDictionary:selected]);
    }
  }
}

TEST(ConnectionEditing, RefusesOccupiedInputsWithoutDeletingTheSelectedWire) {
  @autoreleasepool {
    NSDictionary* selected = wire(@"command", @"rate");
    NSArray<NSDictionary*>* original = @[selected, wire(@"alternate", @"rate", @1)];
    for (id old in @[selected, [NSNull null]]) {
      NSString* error = nil;
      NSArray<NSDictionary*>* result = GalataConnectionEdit(blocks(),
                                                            original,
                                                            old == [NSNull null] ? nil : old,
                                                            wire(@"state", @"rate", @1),
                                                            &error);
      EXPECT_EQ(result, nil);
      EXPECT_GT(error.length, 0U);
      EXPECT_TRUE(([original isEqualToArray:@[selected, wire(@"alternate", @"rate", @1)]]));
    }
    EXPECT_EQ(GalataConnectionEdit(blocks(), original, nil, selected, nullptr), nil);
  }
}

TEST(ConnectionEditing, RefusesUnknownEndpointsAndInvalidTargetPortIndices) {
  @autoreleasepool {
    NSArray<NSDictionary*>* original = @[wire(@"command", @"rate")];
    NSMutableArray<NSDictionary*>* invalid = [@[
      wire(@"absent", @"rate", @1),
      wire(@"command", @"absent"),
      wire(@"command", @"alternate"),
      wire(@"command", @"rate", @2),
      wire(@"command", @"row", @3),
      wire(@"command", @"state", @1),
      wire(@"command", @"gain", @1),
      wire(@"command", @"output", @1)
    ] mutableCopy];
    for (id index in @[
           @-1,
           @0.5,
           @"1",
           @YES,
           [NSNull null],
           @(std::numeric_limits<double>::infinity()),
           @(std::numeric_limits<double>::quiet_NaN())
         ])
      [invalid addObject:wire(@"command", @"rate", index)];
    for (NSDictionary* proposed in invalid) {
      NSString* error = nil;
      EXPECT_EQ(GalataConnectionEdit(blocks(), original, nil, proposed, &error), nil);
      EXPECT_GT(error.length, 0U);
      EXPECT_TRUE([original isEqualToArray:@[wire(@"command", @"rate")]]);
    }
  }
}

TEST(ConnectionEditing, RefusesStaleOrAmbiguousSelections) {
  @autoreleasepool {
    NSDictionary* selected = wire(@"command", @"rate");
    for (NSArray<NSDictionary*>* original in
         @[@[], @[wire(@"alternate", @"rate")], @[selected, selected]]) {
      NSString* error = nil;
      EXPECT_EQ(
          GalataConnectionEdit(blocks(), original, selected, wire(@"command", @"state"), &error),
          nil);
      EXPECT_GT(error.length, 0U);
    }
  }
}

TEST(ConnectionEditing, NoOpIsSuccessfulAndItsContainersAreIndependent) {
  @autoreleasepool {
    NSMutableDictionary* selected = [wire(@"command", @"rate") mutableCopy];
    NSMutableDictionary* unrelated = [wire(@"state", @"output") mutableCopy];
    NSMutableArray<NSDictionary*>* original = [@[selected, unrelated] mutableCopy];
    NSString* error = @"previous error";
    NSArray<NSDictionary*>* result =
        GalataConnectionEdit(blocks(), original, selected, selected, &error);
    ASSERT_NE(result, nil);
    EXPECT_EQ(error, nil);
    EXPECT_TRUE([result isEqualToArray:original]);
    EXPECT_NE(result, original);
    EXPECT_NE(result[0], selected);
    EXPECT_NE(result[1], unrelated);
    selected[@"source"] = @"alternate";
    unrelated[@"source"] = @"gain";
    [original removeAllObjects];
    EXPECT_TRUE(([result isEqualToArray:@[wire(@"command", @"rate"), wire(@"state", @"output")]]));
    ASSERT_TRUE([result[0] isKindOfClass:[NSMutableDictionary class]]);
    ((NSMutableDictionary*)result[0])[@"source"] = @"state";
    EXPECT_TRUE([selected[@"source"] isEqual:@"alternate"]);
  }
}

TEST(ConnectionEditing, AllowsEveryDeclaredPortAndLeavesCompileChecksToTheWorker) {
  @autoreleasepool {
    for (NSDictionary* proposed in @[
           wire(@"command", @"rate", @0),
           wire(@"command", @"rate", @1),
           wire(@"command", @"row", @0),
           wire(@"command", @"row", @1),
           wire(@"command", @"row", @2),
           wire(@"state", @"state", @0),
           wire(@"gain", @"gain", @0),
           wire(@"output", @"output", @0)
         ]) {
      NSArray<NSDictionary*>* result = GalataConnectionEdit(blocks(), @[], nil, proposed, nullptr);
      ASSERT_NE(result, nil);
      EXPECT_TRUE([result isEqualToArray:@[proposed]]);
    }
  }
}

}  // namespace
