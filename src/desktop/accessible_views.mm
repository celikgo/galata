// SPDX-License-Identifier: Apache-2.0
#import "accessible_views.hpp"

#import "dim_theme.hpp"

#include <charconv>
#include <limits>

@interface GalataDimTableRow : NSTableRowView
@end

@implementation GalataDimTableRow

- (NSBackgroundStyle)interiorBackgroundStyle {
  return NSBackgroundStyleEmphasized;
}

- (void)drawSelectionInRect:(NSRect)dirtyRect {
  (void)dirtyRect;
  NSRect selection = NSInsetRect(self.bounds, 1, 1);
  NSBezierPath* outline = [NSBezierPath bezierPathWithRoundedRect:selection xRadius:3 yRadius:3];
  [DimRaised() setFill];
  [outline fill];
  [DimAccent() setStroke];
  outline.lineWidth = self.emphasized ? 2 : 1;
  [outline stroke];
}

@end

static NSTableView* NativeTable(NSView* parent,
                                NSString* identifier,
                                NSString* label,
                                NSString* help) {
  NSRect frame = NSMakeRect(
      1, 23, MAX(0, parent.bounds.size.width - 2), MAX(0, parent.bounds.size.height - 24));
  NSScrollView* scroll = [[NSScrollView alloc] initWithFrame:frame];
  scroll.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
  scroll.borderType = NSNoBorder;
  scroll.hasVerticalScroller = YES;
  scroll.hasHorizontalScroller = YES;
  scroll.autohidesScrollers = YES;
  scroll.backgroundColor = DimSurface();
  NSTableView* table = [[NSTableView alloc] initWithFrame:scroll.bounds];
  table.identifier = identifier;
  table.accessibilityIdentifier = identifier;
  table.accessibilityLabel = label;
  table.accessibilityHelp = help;
  table.backgroundColor = DimSurface();
  table.gridColor = DimBorder();
  table.gridStyleMask = NSTableViewSolidHorizontalGridLineMask;
  table.rowHeight = 25;
  table.intercellSpacing = NSMakeSize(8, 2);
  table.columnAutoresizingStyle = NSTableViewNoColumnAutoresizing;
  table.allowsColumnReordering = NO;
  table.allowsColumnResizing = YES;
  table.allowsMultipleSelection = NO;
  table.allowsEmptySelection = YES;
  table.allowsTypeSelect = YES;
  table.selectionHighlightStyle = NSTableViewSelectionHighlightStyleRegular;
  table.focusRingType = NSFocusRingTypeExterior;
  scroll.documentView = table;
  [parent addSubview:scroll];
  return table;
}

static NSTextField* Footer(NSView* parent) {
  NSTextField* footer = [NSTextField labelWithString:@""];
  footer.frame = NSMakeRect(7, 3, MAX(0, parent.bounds.size.width - 14), 17);
  footer.autoresizingMask = NSViewWidthSizable;
  footer.font = [NSFont systemFontOfSize:10];
  footer.textColor = DimMuted();
  footer.lineBreakMode = NSLineBreakByTruncatingTail;
  [parent addSubview:footer];
  return footer;
}

static void AddColumn(NSTableView* table, NSString* identifier, NSString* title, CGFloat width) {
  NSTableColumn* column = [[NSTableColumn alloc] initWithIdentifier:identifier];
  column.title = title;
  column.width = width;
  column.minWidth = 80;
  column.resizingMask = NSTableColumnUserResizingMask;
  column.headerCell.textColor = DimMuted();
  column.headerCell.font = [NSFont systemFontOfSize:11 weight:NSFontWeightSemibold];
  [table addTableColumn:column];
}

static NSTableCellView* TextCell(NSTableView* table,
                                 NSTableColumn* column,
                                 NSString* value,
                                 BOOL numeric) {
  NSTableCellView* cell = [table makeViewWithIdentifier:column.identifier owner:nil];
  if (!cell) {
    cell = [[NSTableCellView alloc] initWithFrame:NSMakeRect(0, 0, column.width, table.rowHeight)];
    cell.identifier = column.identifier;
    NSTextField* text = [NSTextField labelWithString:@""];
    text.frame = NSMakeRect(3, 3, MAX(0, column.width - 6), table.rowHeight - 6);
    text.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    text.lineBreakMode = NSLineBreakByTruncatingTail;
    text.font = numeric ? [NSFont monospacedDigitSystemFontOfSize:11 weight:NSFontWeightRegular]
                        : [NSFont systemFontOfSize:11];
    text.textColor = DimText();
    text.alignment = numeric ? NSTextAlignmentRight : NSTextAlignmentLeft;
    cell.textField = text;
    [cell addSubview:text];
  }
  cell.textField.stringValue = value;
  cell.textField.accessibilityLabel = column.title;
  cell.toolTip = [NSString stringWithFormat:@"%@: %@", column.title, value];
  return cell;
}

static void DrawTableBorder(NSView* view) {
  [DimBackground() setFill];
  NSRectFill(view.bounds);
  [DimBorder() setStroke];
  [NSBezierPath strokeRect:NSInsetRect(view.bounds, 0.5, 0.5)];
}

@interface GalataBlockList () <NSTableViewDataSource, NSTableViewDelegate>
@property(nonatomic, readwrite, strong) NSTableView* tableView;
@property(nonatomic, strong) NSTextField* footer;
@property(nonatomic, copy) NSArray<NSDictionary<NSString*, NSString*>*>* records;
@property(nonatomic) BOOL updatingSelection;
@end

@implementation GalataBlockList

- (instancetype)initWithFrame:(NSRect)frame {
  self = [super initWithFrame:frame];
  if (self) {
    _enabled = YES;
    _records = @[];
    self.tableView =
        NativeTable(self,
                    @"galata.block-list",
                    @"Model blocks",
                    @"Use Up and Down to select a block for inspection. Type a block ID, "
                    @"kind, or original channel name to find it.");
    self.tableView.dataSource = self;
    self.tableView.delegate = self;
    AddColumn(self.tableView, @"id", @"Block ID", 148);
    AddColumn(self.tableView, @"kind", @"Kind", 145);
    AddColumn(self.tableView, @"original_name", @"Original channel", 165);
    self.footer = Footer(self);
    self.footer.stringValue = @"No model blocks";
  }
  return self;
}

- (void)drawRect:(NSRect)dirtyRect {
  (void)dirtyRect;
  DrawTableBorder(self);
}

- (void)setEnabled:(BOOL)enabled {
  _enabled = enabled;
  self.tableView.enabled = enabled;
}

- (void)setBlocks:(NSArray<NSDictionary*>*)blocks
    originalNames:(NSDictionary<NSString*, NSString*>*)originalNames {
  // Snapshot the displayed fields: the editor mutates its draft dictionaries.
  // Stable records avoid reloading an unchanged table during canvas refreshes.
  NSMutableArray* records = [NSMutableArray arrayWithCapacity:blocks.count];
  for (NSDictionary* block in blocks) {
    NSString* identifier = block[@"id"];
    NSString* kind = block[@"kind"];
    NSString* originalName = originalNames[identifier];
    [records addObject:@{
      @"id": identifier ? [identifier copy] : @"",
      @"kind": kind ? [kind copy] : @"",
      @"original_name": originalName ? [originalName copy] : @""
    }];
  }
  if ([records isEqualToArray:self.records])
    return;
  self.records = records;
  self.updatingSelection = YES;
  [self.tableView reloadData];
  NSString* selected = self.selectedID;
  _selectedID = nil;
  [self setSelectedID:selected];
  self.updatingSelection = NO;
  self.footer.stringValue = [NSString
      stringWithFormat:@"%lu blocks · Arrow keys select", (unsigned long)self.records.count];
}

- (void)setSelectedID:(NSString*)selectedID {
  if ([_selectedID isEqualToString:selectedID] || (!_selectedID && !selectedID))
    return;
  BOOL wasUpdating = self.updatingSelection;
  self.updatingSelection = YES;
  _selectedID = [selectedID copy];
  NSUInteger index = [self.records
      indexOfObjectPassingTest:^BOOL(NSDictionary* record, NSUInteger row, BOOL* stop) {
        (void)row;
        (void)stop;
        return [record[@"id"] isEqualToString:selectedID];
      }];
  if (index == NSNotFound) {
    _selectedID = nil;
    [self.tableView deselectAll:nil];
  } else {
    [self.tableView selectRowIndexes:[NSIndexSet indexSetWithIndex:index] byExtendingSelection:NO];
    [self.tableView scrollRowToVisible:static_cast<NSInteger>(index)];
  }
  self.updatingSelection = wasUpdating;
}

- (NSInteger)numberOfRowsInTableView:(NSTableView*)tableView {
  (void)tableView;
  return static_cast<NSInteger>(self.records.count);
}

- (id)tableView:(NSTableView*)tableView
    objectValueForTableColumn:(NSTableColumn*)tableColumn
                          row:(NSInteger)row {
  (void)tableView;
  NSString* value = self.records[static_cast<NSUInteger>(row)][tableColumn.identifier];
  return value ? value : @"";
}

- (NSView*)tableView:(NSTableView*)tableView
    viewForTableColumn:(NSTableColumn*)tableColumn
                   row:(NSInteger)row {
  return TextCell(tableView,
                  tableColumn,
                  [self tableView:tableView objectValueForTableColumn:tableColumn row:row],
                  NO);
}

- (NSTableRowView*)tableView:(NSTableView*)tableView rowViewForRow:(NSInteger)row {
  (void)tableView;
  GalataDimTableRow* view = [[GalataDimTableRow alloc] initWithFrame:NSZeroRect];
  NSDictionary* record = self.records[static_cast<NSUInteger>(row)];
  view.accessibilityLabel =
      [NSString stringWithFormat:@"%@, %@%@",
                                 record[@"id"],
                                 record[@"kind"],
                                 [record[@"original_name"] length]
                                     ? [@", " stringByAppendingString:record[@"original_name"]]
                                     : @""];
  return view;
}

- (BOOL)selectionShouldChangeInTableView:(NSTableView*)tableView {
  (void)tableView;
  return self.enabled;
}

- (void)tableViewSelectionDidChange:(NSNotification*)notification {
  (void)notification;
  if (self.updatingSelection)
    return;
  NSInteger row = self.tableView.selectedRow;
  _selectedID = row >= 0 ? [self.records[static_cast<NSUInteger>(row)][@"id"] copy] : nil;
  if (self.selectionHandler)
    self.selectionHandler(self.selectedID);
}

- (NSString*)tableView:(NSTableView*)tableView
    typeSelectStringForTableColumn:(NSTableColumn*)tableColumn
                               row:(NSInteger)row {
  return [self tableView:tableView objectValueForTableColumn:tableColumn row:row];
}

@end

static NSString* SampleValue(NSNumber* value) {
  char buffer[64];
  auto result = std::to_chars(buffer,
                              buffer + sizeof(buffer),
                              value.doubleValue,
                              std::chars_format::general,
                              std::numeric_limits<double>::max_digits10);
  if (result.ec != std::errc{})
    return value.stringValue;
  return [[NSString alloc] initWithBytes:buffer
                                  length:static_cast<NSUInteger>(result.ptr - buffer)
                                encoding:NSUTF8StringEncoding];
}

@interface GalataTrajectoryTable () <NSTableViewDataSource, NSTableViewDelegate>
@property(nonatomic, readwrite, strong) NSTableView* tableView;
@property(nonatomic, strong) NSTextField* footer;
@property(nonatomic, copy) NSArray<NSString*>* headers;
@property(nonatomic, copy) NSArray<NSArray<NSNumber*>*>* rows;
@end

@implementation GalataTrajectoryTable

- (instancetype)initWithFrame:(NSRect)frame {
  self = [super initWithFrame:frame];
  if (self) {
    _headers = @[];
    _rows = @[];
    self.tableView =
        NativeTable(self,
                    @"galata.trajectory-table",
                    @"Trajectory samples",
                    @"Every retained trajectory sample is available in artifact order. "
                    @"Use Up and Down to select samples. Values retain binary64 precision; "
                    @"horizontal scrolling reveals every channel.");
    self.tableView.dataSource = self;
    self.tableView.delegate = self;
    self.footer = Footer(self);
    self.footer.stringValue = @"No retained samples";
  }
  return self;
}

- (void)drawRect:(NSRect)dirtyRect {
  (void)dirtyRect;
  DrawTableBorder(self);
}

- (void)setHeaders:(NSArray<NSString*>*)headers rows:(NSArray<NSArray<NSNumber*>*>*)rows {
  if (![self.headers isEqualToArray:headers]) {
    self.headers = headers;
    for (NSTableColumn* column in [self.tableView.tableColumns copy])
      [self.tableView removeTableColumn:column];
    [headers enumerateObjectsUsingBlock:^(NSString* title, NSUInteger index, BOOL* stop) {
      (void)stop;
      AddColumn(
          self.tableView, [NSString stringWithFormat:@"%lu", (unsigned long)index], title, 178);
    }];
  }
  NSMutableArray* retainedRows = [NSMutableArray arrayWithCapacity:rows.count];
  for (NSArray* row in rows)
    [retainedRows addObject:[row copy]];
  self.rows = retainedRows;
  [self.tableView reloadData];
  self.footer.stringValue = rows.count
                                ? [NSString stringWithFormat:@"%lu retained samples · SI values",
                                                             (unsigned long)rows.count]
                                : @"No retained samples";
}

- (NSInteger)numberOfRowsInTableView:(NSTableView*)tableView {
  (void)tableView;
  return static_cast<NSInteger>(self.rows.count);
}

- (id)tableView:(NSTableView*)tableView
    objectValueForTableColumn:(NSTableColumn*)tableColumn
                          row:(NSInteger)row {
  (void)tableView;
  NSUInteger column = static_cast<NSUInteger>(tableColumn.identifier.integerValue);
  NSArray<NSNumber*>* values = self.rows[static_cast<NSUInteger>(row)];
  return column < values.count ? SampleValue(values[column]) : @"";
}

- (NSView*)tableView:(NSTableView*)tableView
    viewForTableColumn:(NSTableColumn*)tableColumn
                   row:(NSInteger)row {
  return TextCell(tableView,
                  tableColumn,
                  [self tableView:tableView objectValueForTableColumn:tableColumn row:row],
                  YES);
}

- (NSTableRowView*)tableView:(NSTableView*)tableView rowViewForRow:(NSInteger)row {
  (void)tableView;
  (void)row;
  return [[GalataDimTableRow alloc] initWithFrame:NSZeroRect];
}

@end
