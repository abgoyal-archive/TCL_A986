

#import <Cocoa/Cocoa.h>

extern NSString *WebKitLocalCacheDefaultsKey;

@interface NSString (WebKitExtras)

- (void)_web_drawAtPoint:(NSPoint)point font:(NSFont *)font textColor:(NSColor *)textColor;
- (void)_web_drawDoubledAtPoint:(NSPoint)textPoint withTopColor:(NSColor *)topColor bottomColor:(NSColor *)bottomColor font:(NSFont *)font;

- (float)_web_widthWithFont:(NSFont *)font;

// Handles home directories that have symlinks in their paths.
// This works around 2774250.
- (NSString *)_web_stringByAbbreviatingWithTildeInPath;

- (NSString *)_web_stringByStrippingReturnCharacters;

+ (NSStringEncoding)_web_encodingForResource:(Handle)resource;

- (BOOL)_webkit_isCaseInsensitiveEqualToString:(NSString *)string;
- (BOOL)_webkit_hasCaseInsensitivePrefix:(NSString *)suffix;
- (BOOL)_webkit_hasCaseInsensitiveSuffix:(NSString *)suffix;
- (BOOL)_webkit_hasCaseInsensitiveSubstring:(NSString *)substring;
- (NSString *)_webkit_filenameByFixingIllegalCharacters;

- (NSString *)_webkit_stringByTrimmingWhitespace;
- (NSString *)_webkit_stringByCollapsingNonPrintingCharacters;
- (NSString *)_webkit_stringByCollapsingWhitespaceCharacters;
- (NSString *)_webkit_fixedCarbonPOSIXPath;

+ (NSString *)_webkit_localCacheDirectoryWithBundleIdentifier:(NSString*)bundleIdentifier;

@end
