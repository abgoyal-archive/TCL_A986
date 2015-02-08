

#import <Cocoa/Cocoa.h>

@interface NSImage (WebExtras)

- (void)_web_scaleToMaxSize:(NSSize)size;

- (void)_web_dissolveToFraction:(float)delta;

// Debug method. Saves an image and opens it in the preferred TIFF viewing application.
- (void)_web_saveAndOpen;

@end
