// SPDX-License-Identifier: Apache-2.0
// Twitter Dim-inspired desktop colors. Styling only; no engine behavior.
#ifndef GALATA_DESKTOP_DIM_THEME_HPP
#define GALATA_DESKTOP_DIM_THEME_HPP

#import <AppKit/AppKit.h>
#import <QuartzCore/QuartzCore.h>

inline NSColor* DimRGB(unsigned int rgb) {
  return [NSColor colorWithSRGBRed:((rgb >> 16U) & 255U) / 255.0
                             green:((rgb >> 8U) & 255U) / 255.0
                              blue:(rgb & 255U) / 255.0
                             alpha:1.0];
}

inline NSColor* DimBackground() {
  return DimRGB(0x15202B);
}

inline NSColor* DimSurface() {
  return DimRGB(0x192734);
}

inline NSColor* DimRaised() {
  return DimRGB(0x22303C);
}

inline NSColor* DimText() {
  return DimRGB(0xF7F9F9);
}

inline NSColor* DimMuted() {
  return DimRGB(0x8899AC);
}

inline NSColor* DimBorder() {
  return DimRGB(0x38444D);
}

inline NSColor* DimAccent() {
  return DimRGB(0x1DA1F2);
}

inline void DimPanel(NSView* view, NSColor* background, bool border = false) {
  view.wantsLayer = YES;
  view.layer.backgroundColor = background.CGColor;
  view.layer.borderColor = DimBorder().CGColor;
  view.layer.borderWidth = border ? 1.0 : 0.0;
}

inline void DimWindow(NSWindow* window) {
  window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];
  window.backgroundColor = DimBackground();
  window.titlebarAppearsTransparent = YES;
  DimPanel(window.contentView, DimBackground());
}

inline void DimPrimaryButton(NSButton* button) {
  button.bezelColor = DimAccent();
  // Dark lettering keeps blue actions legible instead of placing white on a
  // medium-luminance accent. Disabled controls retain native AppKit treatment.
  button.attributedTitle = [[NSAttributedString alloc]
      initWithString:button.title
          attributes:@{
            NSForegroundColorAttributeName: DimBackground(),
            NSFontAttributeName: [NSFont systemFontOfSize:12 weight:NSFontWeightSemibold]
          }];
}

#endif
